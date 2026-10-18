#include "drive.h"

#include "mymath.h"
#include <math.h>

#define DRIVE_MAX_TOTAL_TORQUE_NM (DRIVE_MAX_TORQUE_NM * 2.0f)

static void ApplySensorParams(Drive* obj);

// ---------------------------------------------------------------------------
// 観測量の更新
// ---------------------------------------------------------------------------

// 前輪の各輪の周速 (上位への報告用)。車体の速度は走行距離と加速度計のカルマンフィルタ (speed_kf.h) が出す。
// 生の角速度は微分ノイズで静止中も ±12m/s 振れるため、各輪の周速は LPF (τ≈10ms) を通した値を報告する
static void UpdateFrontWheelSpeeds(Drive* obj, float dt_s) {
  float omega_left = LPF_UpdateTau(&obj->lpf_front_left,
                                   Encoder_GetAngularVelocityLeft(obj->encoder) * DRIVE_FRONT_LEFT_DIR,
                                   DRIVE_LPF_TAU_FRONT_S, dt_s);
  float omega_right = LPF_UpdateTau(&obj->lpf_front_right,
                                    Encoder_GetAngularVelocityRight(obj->encoder) * DRIVE_FRONT_RIGHT_DIR,
                                    DRIVE_LPF_TAU_FRONT_S, dt_s);
  obj->front_speed_left_m_s = omega_left * DRIVE_FRONT_WHEEL_RADIUS_M;
  obj->front_speed_right_m_s = omega_right * DRIVE_FRONT_WHEEL_RADIUS_M;
}

// ヨーレートは IMU の実測値を優先する。舵角からの幾何計算 (自転車モデル) は前輪が滑ると
// 実際のヨーレートから乖離するため、まさにTCが必要な場面で基準速度が狂うことになる。
static float EstimateYawRate(Drive* obj) {
  if (obj->imu != NULL && Imu_IsReady(obj->imu)) {
    obj->yaw_rate_measured = true;
    return Radians(Imu_GetData(obj->imu)->gyro_z);
  }

  obj->yaw_rate_measured = false;
  float steer_rad = Steering_GetRoadWheelAngleRad(obj->steering);
  float cos_steer = cosf(steer_rad);
  if (Abs(cos_steer) < 0.1f) return 0.0f;
  return obj->vehicle_speed_m_s * (sinf(steer_rad) / cos_steer) / DRIVE_WHEELBASE_M;
}

// スリップ率 (上位への報告用) = (駆動輪周速 − その輪が本来出るべき速度) / max(|基準速度|, 下限)。
// 進行方向が基準で、正 = 空転・負 = ロック傾向。進行方向は基準速度と車輪速の和の符号で決める:
// 停止からの空転 (基準≈0・車輪が前へ) でも、前進中のロック (基準>0・車輪≈0) でも前進と読める。
// 基準速度だけの符号だと、停止付近でエンコーダのノイズが符号を決めてしまう
static float SlipRatio(float wheel_speed_m_s, float reference_speed_m_s, float floor_m_s) {
  float denominator = Abs(reference_speed_m_s);
  if (denominator < floor_m_s) denominator = floor_m_s;
  float dir = (reference_speed_m_s + wheel_speed_m_s) >= 0.0f ? 1.0f : -1.0f;
  return (wheel_speed_m_s - reference_speed_m_s) * dir / denominator;
}

// 走行距離と加速度計の車速推定 (speed_kf.h) を1周期ぶん進める。Encoder の更新ごとに1回だけ
static void UpdateSpeedKf(Drive* obj) {
  uint32_t encoder_seq = Encoder_GetUpdateSeq(obj->encoder);
  if (encoder_seq == obj->kf_encoder_seq) return;
  obj->kf_encoder_seq = encoder_seq;

  // 前輪の回転量の左右平均 → 走行距離。舵角で車体前後方向へ射影するのは LPF の車速と同じ
  float steer_rad = Steering_GetRoadWheelAngleRad(obj->steering);
  const float cos_steer = cosf(steer_rad);
  SpeedKf_Predict(&obj->speed_kf, Encoder_GetLastDt(obj->encoder));

  // 制御ループが長く止まると、エンコーダの角度差 (±半周に畳んである) は 1 回転ぶん折り返す。予測が見込む回転量に
  // 最も近い側へ、各輪の増分に 1 回転の整数倍を足して戻す (止まっていなければ何も変わらない)
  const float expected_rad = SpeedKf_ExpectedDistanceStep(&obj->speed_kf) / (DRIVE_FRONT_WHEEL_RADIUS_M * cos_steer);
  float gap_left = Encoder_GetLastGapLeft(obj->encoder) * DRIVE_FRONT_LEFT_DIR;
  float gap_right = Encoder_GetLastGapRight(obj->encoder) * DRIVE_FRONT_RIGHT_DIR;
  gap_left += TWO_PI * roundf((expected_rad - gap_left) / TWO_PI);
  gap_right += TWO_PI * roundf((expected_rad - gap_right) / TWO_PI);
  SpeedKf_UpdateDistance(&obj->speed_kf, 0.5f * (gap_left + gap_right) * DRIVE_FRONT_WHEEL_RADIUS_M * cos_steer);

  // 加速度は IMU のサンプル (約200Hz) が新しいときだけ。IMU が使えない間は走行距離だけで推定する
  if (obj->imu != NULL && Imu_IsReady(obj->imu)) {
    const ImuData* imu = Imu_GetData(obj->imu);
    if (imu->sample_seq != obj->kf_imu_seq) {
      obj->kf_imu_seq = imu->sample_seq;
      // 旋回で乗る -k·ω² を補正してから使う (control_params.h の kf_lever_arm_m。speed_kf.h の SPEED_KF_LEVER_ARM_M)
      const float gz_rad_s = Radians(imu->gyro_z);
      SpeedKf_UpdateAccel(&obj->speed_kf, imu->accel_x_raw + obj->params.kf_lever_arm_m * gz_rad_s * gz_rad_s);
    }
  }
}

// dt_s は前回からの実際の経過時間 (呼び出しが飛んだ周期も含む。ローパスの係数に使う)
static void UpdateObservations(Drive* obj, float dt_s) {
  UpdateFrontWheelSpeeds(obj, dt_s);
  UpdateSpeedKf(obj);
  // 車体の車速 = カルマンフィルタ。PID・TC のスリップの基準・自動停止の前後判定のすべてがこれを使う
  obj->vehicle_speed_m_s = SpeedKf_GetSpeed(&obj->speed_kf);

  float omega_left = LPF_UpdateTau(&obj->lpf_rear_left,
                                   BldcMotor_GetAngularSpeed(&obj->motors->rear_left) * DRIVE_REAR_LEFT_DIR,
                                   DRIVE_LPF_TAU_REAR_S, dt_s);
  float omega_right = LPF_UpdateTau(&obj->lpf_rear_right,
                                    BldcMotor_GetAngularSpeed(&obj->motors->rear_right) * DRIVE_REAR_RIGHT_DIR,
                                    DRIVE_LPF_TAU_REAR_S, dt_s);
  obj->rear_speed_left_m_s = omega_left * DRIVE_REAR_WHEEL_RADIUS_M;
  obj->rear_speed_right_m_s = omega_right * DRIVE_REAR_WHEEL_RADIUS_M;

  obj->yaw_rate_rad_s = EstimateYawRate(obj);

  // 旋回中は内輪と外輪で本来の速度が違う。ここを車体速度で共通化すると外輪が常時「空転」と
  // 判定されてTCが誤介入するため、ヨーレートから各輪の基準速度を作る。
  // 基準速度はカルマンフィルタの車速 (遅れがほぼ無い。以前の前輪LPFは τ10ms の遅れが加速で基準を低く見せ、
  // スリップ率に偏りを入れていた)
  float half_track = DRIVE_REAR_TRACK_M * 0.5f;
  obj->slip_ref_left_m_s = obj->vehicle_speed_m_s - obj->yaw_rate_rad_s * half_track;
  obj->slip_ref_right_m_s = obj->vehicle_speed_m_s + obj->yaw_rate_rad_s * half_track;
  float floor_m_s = obj->params.slip_speed_floor_m_s;
  obj->slip_left = SlipRatio(obj->rear_speed_left_m_s, obj->slip_ref_left_m_s, floor_m_s);
  obj->slip_right = SlipRatio(obj->rear_speed_right_m_s, obj->slip_ref_right_m_s, floor_m_s);
}

// ---------------------------------------------------------------------------
// 出力
// ---------------------------------------------------------------------------

static void ResetTraction(Drive* obj);

// トルク 0 を指令して惰行させる。
// 送信を止めて惰行させてはいけない: MD は無通信0.5秒で停止モードに入るまで直前のトルクを
// 出し続けるので、減速中の負トルクが停車後も残って後退する
static void Coast(Drive* obj) {
  ResetTraction(obj);
  obj->torque_left_nm = 0.0f;
  obj->torque_right_nm = 0.0f;
  obj->torque_request_left_nm = 0.0f;
  obj->torque_request_right_nm = 0.0f;
  BldcMotor_SetTorqueNm(&obj->motors->rear_left, 0.0f);
  BldcMotor_SetTorqueNm(&obj->motors->rear_right, 0.0f);
}

// 後輪MDを制動モードに切り替えて左右へ同じ制動トルクを掛ける。制動トルクは回転を妨げる
// 向きに掛かる (向きはMD側が回転方向から決める) ため、こちらから符号を与える必要はない。
// 上位への報告だけは駆動と区別できるよう負値にする
static void SendBrake(Drive* obj, float nm) {
  PID_Reset(&obj->speed_pid);
  ResetTraction(obj);
  // MD の制動モードは左右へ同じトルクしか出せないので、TV はこの間左右差を付けられない
  TorqueVectoring_Reset(&obj->tv);
  obj->torque_left_nm = -nm;
  obj->torque_right_nm = -nm;
  BldcMotor_SetBrakeNm(&obj->motors->rear_left, nm);
  BldcMotor_SetBrakeNm(&obj->motors->rear_right, nm);
}

// サイドブレーキ: 有効化された瞬間の機械角度をラッチし、位置制御で固定する。
// 左右は独立にラッチするため、旋回中に停止して左右輪の角度が異なっていても問題ない
// (各輪は自分自身の角度を自分自身へ送り返すだけなので、DRIVE_REAR_LEFT/RIGHT_DIR の
// 符号変換もwraparound補正も不要)
static void SendSideBrake(Drive* obj) {
  PID_Reset(&obj->speed_pid);
  TorqueVectoring_Reset(&obj->tv);

  if (!obj->side_brake_engaged) {
    bool left_valid = BldcMotor_IsDataValid(&obj->motors->rear_left);
    bool right_valid = BldcMotor_IsDataValid(&obj->motors->rear_right);
    if (!left_valid || !right_valid) {
      // 幽霊値 (角度0) をラッチしないよう、有効な角度が取れるまで通常のトルク制動で代用する
      obj->torque_request_left_nm = -DRIVE_MAX_BRAKE_TORQUE_NM;
      obj->torque_request_right_nm = -DRIVE_MAX_BRAKE_TORQUE_NM;
      SendBrake(obj, DRIVE_MAX_BRAKE_TORQUE_NM);
      return;
    }
    obj->side_brake_target_left_rad = BldcMotor_GetMechAngle(&obj->motors->rear_left);
    obj->side_brake_target_right_rad = BldcMotor_GetMechAngle(&obj->motors->rear_right);
    obj->side_brake_engaged = true;
  }

  ResetTraction(obj);
  obj->torque_left_nm = 0.0f;
  obj->torque_right_nm = 0.0f;
  obj->torque_request_left_nm = 0.0f;
  obj->torque_request_right_nm = 0.0f;
  BldcMotor_SetPosition(&obj->motors->rear_left, obj->side_brake_target_left_rad);
  BldcMotor_SetPosition(&obj->motors->rear_right, obj->side_brake_target_right_rad);
}

// 目標車速も実車速もほぼ0の間はトルク 0 を指令して自由回転させる (惰行)。上位が明示的に
// ブレーキ (RAS_CMD_FLAG_BRAKE) を指定しない限り、停車中も車両を押さえ込まない。
// 積分を持ち越すと再発進時に停止中に溜まった分が一気に出るのでリセットする
static void CoastStandstill(Drive* obj) {
  PID_Reset(&obj->speed_pid);
  TorqueVectoring_Reset(&obj->tv);
  Coast(obj);
}

// 車速PIの加速度フィードフォワード (drive.h の同名の節)。この周期に目標車速を step_m_s 動かした。
// 戻り値は PI の出力に足すトルク [Nm、後輪2輪の合計]
static float SpeedFeedforward(Drive* obj, float step_m_s, float dt_s) {
  if (dt_s > 0.0f) {
    obj->speed_ff_accel_m_s2 +=
        (step_m_s / dt_s - obj->speed_ff_accel_m_s2) * (dt_s / (DRIVE_SPEED_FF_TAU_S + dt_s));
  }
  float nm = obj->params.speed_ff_nm_per_m_s2 * obj->speed_ff_accel_m_s2;
  // 転がり抵抗ぶんは目標車速の向きへ。目標がほぼ 0 の間は向きが定まらないので足さない
  if (obj->target_speed_m_s >= DRIVE_STANDSTILL_SPEED_M_S) {
    nm += obj->params.speed_ff_rr_nm;
  } else if (obj->target_speed_m_s <= -DRIVE_STANDSTILL_SPEED_M_S) {
    nm -= obj->params.speed_ff_rr_nm;
  }
  return nm;
}

static void SendTorque(Drive* obj, float left_nm, float right_nm) {
  obj->torque_left_nm = left_nm;
  obj->torque_right_nm = right_nm;
  BldcMotor_SetTorqueNm(&obj->motors->rear_left, left_nm * DRIVE_REAR_LEFT_DIR);
  BldcMotor_SetTorqueNm(&obj->motors->rear_right, right_nm * DRIVE_REAR_RIGHT_DIR);
}

static bool IsStandstill(const Drive* obj) {
  return Abs(obj->target_speed_m_s) < DRIVE_STANDSTILL_SPEED_M_S &&
         Abs(obj->vehicle_speed_m_s) < DRIVE_STANDSTILL_SPEED_M_S;
}

// ---------------------------------------------------------------------------
// トルク配分
// ---------------------------------------------------------------------------

static void ResetSlipLimiter(SlipLimiter* st) {
  st->active = false;
  st->integral_nm = 0.0f;
}

// スリップ制限の PI を1周期進め、トルクの上限 [Nm] を返す (手順は drive.h の「制限のしかた」)。
//   error_m_s  目標スリップ速度 − 実際のスリップ速度 (正 = 余裕がある)
//   request_nm この周期に掛けたいトルクの大きさ
//   applied_nm 前の周期に実際に掛けたトルクの大きさ (働き始めるときの積分の初期値)
// 働いていない間は max_nm (= 制限なし) を返す
static float UpdateSlipLimiter(SlipLimiter* st, float error_m_s, float request_nm, float applied_nm,
                               float kp, float ki, float min_nm, float max_nm, float dt_s) {
  if (!st->active) {
    if (error_m_s >= 0.0f) return max_nm;
    st->active = true;
    st->integral_nm = applied_nm;
  }
  st->integral_nm = Constrain(st->integral_nm + ki * error_m_s * dt_s, min_nm, max_nm);
  float limit_nm = Constrain(st->integral_nm + kp * error_m_s, min_nm, max_nm);
  // 要求どおり掛けても滑らないところまで戻った。次に滑ったらそのときのトルクから始め直す
  if (limit_nm >= request_nm && error_m_s > 0.0f) {
    st->active = false;
    return max_nm;
  }
  return limit_nm;
}

// トルクを掛けている向きへのスリップ速度 [m/s]。駆動で空転していれば正、減速でロックしかけて
// いれば正 (どちらも「タイヤがトルクの向きへ路面より先に進んでいる」)。トルクが0なら0
static float SlipSpeedAlongTorque(float wheel_speed_m_s, float reference_speed_m_s, float torque_nm) {
  if (torque_nm > 0.0f) return wheel_speed_m_s - reference_speed_m_s;
  if (torque_nm < 0.0f) return reference_speed_m_s - wheel_speed_m_s;
  return 0.0f;
}

static float SlipSpeedTarget(float slip_target, float reference_speed_m_s, float floor_m_s) {
  float denominator = Abs(reference_speed_m_s);
  if (denominator < floor_m_s) denominator = floor_m_s;
  return slip_target * denominator;
}

static void ResetAbs(Drive* obj) {
  ResetSlipLimiter(&obj->abs);
  obj->abs_limit_nm = DRIVE_MAX_BRAKE_TORQUE_NM;
  obj->abs_floor_time_s = 0.0f;
  obj->abs_fallback_latched = false;
}

static void ResetTraction(Drive* obj) {
  ResetSlipLimiter(&obj->tc_left);
  ResetSlipLimiter(&obj->tc_right);
  obj->tc_limit_left_nm = DRIVE_MAX_TORQUE_NM;
  obj->tc_limit_right_nm = DRIVE_MAX_TORQUE_NM;
  obj->tc_limiting = false;
  obj->wheel_lift_limiting = false;
}

// ABS: 制動トルクの上限を更新し、実際に掛ける制動トルクを返す。左右のロックしかけている方に
// 合わせる (select-low)。MD の制動は車輪の回転を妨げる向きに掛かるので、ロック傾向は
// 「車輪が路面より遅い」= 進行方向を基準にしたスリップ速度の負側
static float ApplyAbs(Drive* obj, float requested_nm, float dt_s) {
  if (obj->abs_fallback_latched) return requested_nm;

  const ControlParams* p = &obj->params;
  float dir_left = (obj->slip_ref_left_m_s + obj->rear_speed_left_m_s) >= 0.0f ? 1.0f : -1.0f;
  float dir_right = (obj->slip_ref_right_m_s + obj->rear_speed_right_m_s) >= 0.0f ? 1.0f : -1.0f;
  float error_left = SlipSpeedTarget(p->abs_slip_target, obj->slip_ref_left_m_s, p->slip_speed_floor_m_s) -
                     (obj->slip_ref_left_m_s - obj->rear_speed_left_m_s) * dir_left;
  float error_right = SlipSpeedTarget(p->abs_slip_target, obj->slip_ref_right_m_s, p->slip_speed_floor_m_s) -
                      (obj->slip_ref_right_m_s - obj->rear_speed_right_m_s) * dir_right;
  float error = error_left < error_right ? error_left : error_right;

  // 前の周期に実際に掛けた制動トルク (SendBrake が負値で残している。制動の最初の周期は
  // 駆動トルクが残っているので 0 とみなす)
  float applied_nm = obj->torque_left_nm < 0.0f ? -obj->torque_left_nm : 0.0f;
  obj->abs_limit_nm = UpdateSlipLimiter(&obj->abs, error, requested_nm, applied_nm, p->abs_kp_nm_per_m_s,
                                        p->abs_ki_nm_per_m, 0.0f, DRIVE_MAX_BRAKE_TORQUE_NM, dt_s);

  if (requested_nm > 0.0f && obj->abs_limit_nm <= requested_nm * DRIVE_ABS_FALLBACK_LIMIT_RATIO) {
    obj->abs_floor_time_s += dt_s;
    if (obj->abs_floor_time_s >= DRIVE_ABS_FALLBACK_TIME_S) {
      obj->abs_fallback_latched = true;
      return requested_nm;
    }
  } else {
    obj->abs_floor_time_s = 0.0f;
  }
  return obj->abs_limit_nm < requested_nm ? obj->abs_limit_nm : requested_nm;
}

// TC + 片輪浮き対策: 1輪ぶんのトルク上限を更新する。
//   request_nm       この周期にその輪へ掛けたいトルク (符号つき。TV の差を載せた後)
//   applied_nm       前の周期に実際に掛けたトルク (符号つき)
//   slip_m_s         その輪のトルクの向きへのスリップ速度 (前輪基準)
//   lead_m_s         その輪が反対の輪よりトルクの向きへ先走っている速度 (ヨーレートぶんを除く)
//   reference_m_s    その輪の基準速度
// lift_limited には、片輪浮き対策の偏差の方が効いていたかを返す
static float UpdateWheelLimit(Drive* obj, SlipLimiter* st, float request_nm, float applied_nm,
                              float slip_m_s, float lead_m_s, float reference_m_s, float dt_s,
                              bool* lift_limited) {
  const ControlParams* p = &obj->params;
  *lift_limited = false;
  if (!obj->tc_enabled && !obj->wheel_lift_guard_enabled) {
    ResetSlipLimiter(st);
    return DRIVE_MAX_TORQUE_NM;
  }

  // どちらも無効な側の偏差は「十分に余裕がある」にしておく
  float error = 1.0e3f;
  if (obj->tc_enabled) {
    error = SlipSpeedTarget(p->tc_slip_target, reference_m_s, p->slip_speed_floor_m_s) - slip_m_s;
  }
  if (obj->wheel_lift_guard_enabled) {
    float lift_error = p->wheel_lift_diff_threshold_m_s - lead_m_s;
    if (lift_error < error) {
      error = lift_error;
      *lift_limited = true;
    }
  }
  // 掛けていたトルクと掛けたいトルクの向きが違う (駆動→減速の切り替わり) なら、覚えているのは
  // 逆向きのグリップなので持ち越さない
  if (st->active && request_nm * applied_nm < 0.0f) ResetSlipLimiter(st);
  return UpdateSlipLimiter(st, error, Abs(request_nm), Abs(applied_nm), p->tc_kp_nm_per_m_s,
                           p->tc_ki_nm_per_m, p->tc_min_torque_nm, DRIVE_MAX_TORQUE_NM, dt_s);
}

// 総駆動トルクと左右差を各輪へ配り、トルクの絶対上限 (DRIVE_MAX_TORQUE_NM) に各輪ごとに収める。
//
// 全開で外輪が上限に当たっても、内輪は荷重の比どおりのトルクのままにする (外輪が出せない分を
// 内輪へ載せない。内輪は荷重が抜けていて、載せれば空転する)。そのぶん総駆動トルクは減る
static void AllocateTorque(float total_nm, float diff_nm, float* left_nm, float* right_nm) {
  *left_nm = Constrain((total_nm - diff_nm) * 0.5f, -DRIVE_MAX_TORQUE_NM, DRIVE_MAX_TORQUE_NM);
  *right_nm = Constrain((total_nm + diff_nm) * 0.5f, -DRIVE_MAX_TORQUE_NM, DRIVE_MAX_TORQUE_NM);
}

// 車速フィードバックが壊れて積分が振り切れても、実速度が上限を超えたら加速させない
static float ApplyOverspeedLimit(const Drive* obj, float torque_nm) {
  if (obj->vehicle_speed_m_s > DRIVE_MAX_SPEED_M_S && torque_nm > 0.0f) return 0.0f;
  if (obj->vehicle_speed_m_s < -DRIVE_MAX_SPEED_M_S && torque_nm < 0.0f) return 0.0f;
  return torque_nm;
}

// TC・リミッタで削られた分だけ積分項を巻き戻す (back-calculation)。PIDライブラリ側の
// アンチワインドアップは自身の出力制限しか見ないため、外側で削った分はここで戻す必要がある
static void UnwindIntegral(Drive* obj, float requested_nm, float applied_nm, float dt_s) {
  if (dt_s <= 0.0f || obj->speed_pid.ki <= 0.0f) return;
  obj->speed_pid.integral += (applied_nm - requested_nm) / obj->speed_pid.ki * (dt_s / DRIVE_ANTIWINDUP_TT_S);
}

// ローンチコントロールの状態を進め、発進中なら真を返す (条件は drive.h の同名の節)。
// 車速指令で走っている周期 (制動・サイドブレーキ・torque_mode でない) にだけ呼ぶ
static bool UpdateLaunch(Drive* obj, float dt_s) {
  if (!obj->launch_requested) {
    obj->launching = false;
    obj->launch_spent = false;
    return false;
  }
  float dir = obj->speed_setpoint_m_s >= 0.0f ? 1.0f : -1.0f;
  // 目標へ向かう向きを正にした、目標車速と実車速
  float setpoint_m_s = obj->speed_setpoint_m_s * dir;
  float speed_m_s = obj->vehicle_speed_m_s * dir;

  if (!obj->launching) {
    if (obj->launch_spent || !obj->tc_enabled) return false;
    if (speed_m_s <= -DRIVE_LAUNCH_REVERSE_SPEED_M_S) return false;
    if (setpoint_m_s - speed_m_s < DRIVE_LAUNCH_MIN_STEP_M_S) return false;
    obj->launching = true;
    obj->launch_spent = true;
    obj->launch_time_s = 0.0f;
  }

  obj->launch_time_s += dt_s;
  if (speed_m_s >= setpoint_m_s - DRIVE_LAUNCH_END_MARGIN_M_S || !obj->tc_enabled ||
      obj->launch_time_s >= DRIVE_LAUNCH_TIMEOUT_S) {
    obj->launching = false;
  }
  return obj->launching;
}

// 車速PIを迂回している間 (制動・torque_mode)、目標車速を実車速に合わせ続ける。車速指令へ戻った
// とき、PIは「今の車速から残りをランプで詰める」形で始まる。止まっていれば起点は 0 なので、
// 古い目標から急発進することは無い。0 に固定すると、走っているうちに離したとき (短い制動・
// 自動停止の解除・torque_mode からの復帰) に目標が 0 から数え直しになり、ランプが実車速へ
// 追いつくまで PI が減速させ続ける
static void TrackVehicleSpeed(Drive* obj) {
  obj->target_speed_m_s = Constrain(obj->vehicle_speed_m_s, -DRIVE_MAX_SPEED_M_S, DRIVE_MAX_SPEED_M_S);
}

// ---------------------------------------------------------------------------

void Drive_Init(Drive* obj, Motors* motors, Encoder* encoder, Steering* steering, Imu* imu) {
  obj->motors = motors;
  obj->encoder = encoder;
  obj->steering = steering;
  obj->imu = imu;

  // 制限値は指令フレームに毎回載るので設定は1回でよい。初期値は0 (=トルク上限0) のため、
  // ここを通さないと後輪は一切回らない
  BldcMotor_SetTorqueLimitNm(&motors->rear_left, DRIVE_MAX_TORQUE_NM);
  BldcMotor_SetTorqueLimitNm(&motors->rear_right, DRIVE_MAX_TORQUE_NM);

  ControlParams_SetDefaults(&obj->params);
  PID_Init(&obj->speed_pid, obj->params.speed_kp, obj->params.speed_ki, DRIVE_SPEED_KD,
           -DRIVE_MAX_TOTAL_TORQUE_NM, DRIVE_MAX_TOTAL_TORQUE_NM);
  TorqueVectoring_Init(&obj->tv, &obj->params, DRIVE_REAR_TRACK_M, DRIVE_REAR_WHEEL_RADIUS_M);
  LPF_Init(&obj->lpf_front_left, 0.0f, 0.0f);
  LPF_Init(&obj->lpf_front_right, 0.0f, 0.0f);
  LPF_Init(&obj->lpf_rear_left, 0.0f, 0.0f);
  LPF_Init(&obj->lpf_rear_right, 0.0f, 0.0f);
  Timer_Init(&obj->timer);
  SpeedKf_Init(&obj->speed_kf);
  ApplySensorParams(obj);
  obj->kf_encoder_seq = Encoder_GetUpdateSeq(encoder);  // 起動前の更新は取り込まない
  obj->kf_imu_seq = 0;

  obj->enabled = false;
  obj->tc_enabled = false;  // 上位が送る (ras_link.c と同じ。毎周期 Drive_SetTractionControlEnabled で上書きされる)
  obj->wheel_lift_guard_enabled = true;
  obj->speed_setpoint_m_s = 0.0f;
  obj->accel_limit_m_s2 = DRIVE_DEFAULT_ACCEL_M_S2;
  obj->target_speed_m_s = 0.0f;
  obj->speed_ff_accel_m_s2 = 0.0f;
  obj->brake_active = false;
  obj->brake_torque_nm = DRIVE_MAX_BRAKE_TORQUE_NM;
  obj->torque_mode_active = false;
  obj->manual_torque_nm = 0.0f;
  obj->launch_requested = false;
  obj->launching = false;
  obj->launch_spent = false;
  obj->launch_time_s = 0.0f;
  obj->side_brake_active = false;
  obj->side_brake_engaged = false;
  obj->side_brake_target_left_rad = 0.0f;
  obj->side_brake_target_right_rad = 0.0f;

  obj->vehicle_speed_m_s = 0.0f;
  obj->yaw_rate_rad_s = 0.0f;
  obj->yaw_rate_measured = false;
  obj->front_speed_left_m_s = 0.0f;
  obj->front_speed_right_m_s = 0.0f;
  obj->rear_speed_left_m_s = 0.0f;
  obj->rear_speed_right_m_s = 0.0f;
  obj->slip_left = 0.0f;
  obj->slip_right = 0.0f;
  obj->slip_ref_left_m_s = 0.0f;
  obj->slip_ref_right_m_s = 0.0f;
  ResetTraction(obj);
  obj->abs_enabled = true;
  ResetAbs(obj);
  obj->torque_left_nm = 0.0f;
  obj->torque_right_nm = 0.0f;
  obj->torque_request_left_nm = 0.0f;
  obj->torque_request_right_nm = 0.0f;
}

void Drive_Update(Drive* obj) {
  const float elapsed_s = Timer_Read(&obj->timer);
  Timer_Reset(&obj->timer);
  float dt_s = elapsed_s;
  if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.0f;  // 初回・呼び出しが飛んだ周期では時間積分をしない

  // 観測量は無効時も更新しておく (惰行中の車速をそのまま使って再発進できるようにするため)。
  // ローパスは長く空いた後は新しい値をそのまま採るのが正しいので、丸める前の経過時間を渡す
  UpdateObservations(obj, elapsed_s);

  // ABSの状態は制動モードの間だけ持ち越す。離したら次の制動を全量から始め、フォールバックも解く
  if (!(obj->enabled && !obj->side_brake_active && obj->brake_active && obj->abs_enabled)) {
    ResetAbs(obj);
  }

  // ローンチコントロールは車速指令の間だけ。それ以外を通ったら、次の発進でまた使える
  if (!obj->enabled || obj->side_brake_active || obj->brake_active || obj->torque_mode_active) {
    obj->launching = false;
    obj->launch_spent = false;
    obj->speed_ff_accel_m_s2 = 0.0f;  // 車速PIを通らない間は、目標車速の傾きの記憶を持ち越さない
  }

  if (!obj->enabled) {
    Coast(obj);
    return;
  }
  // サイドブレーキは速度・通常ブレーキ・torque_modeより優先する (速度に関わらず即座に切替)
  if (obj->side_brake_active) {
    obj->target_speed_m_s = 0.0f;
    SendSideBrake(obj);
    return;
  }
  // ブレーキは車速制御より優先する。PIに「目標0」を与えるだけでは制動力がゲイン任せになり、
  // 上位が指定した制動トルクどおりに効かないため、指令中はPIごと迂回する。
  // 目標車速 (レート制限の起点) は実車速に合わせ続ける (TrackVehicleSpeed)
  if (obj->brake_active) {
    TrackVehicleSpeed(obj);
    obj->torque_request_left_nm = -obj->brake_torque_nm;
    obj->torque_request_right_nm = -obj->brake_torque_nm;
    SendBrake(obj, obj->abs_enabled ? ApplyAbs(obj, obj->brake_torque_nm, dt_s) : obj->brake_torque_nm);
    return;
  }

  float requested_total_nm;
  bool launching = false;
  if (obj->torque_mode_active) {
    // 車速PIを迂回して指令トルクをそのまま使う。離脱時に積分が溜まったまま復帰しないよう
    // SendBrake と同様に毎周期リセットしておく。目標車速はブレーキ同様、実車速に合わせ続ける
    TrackVehicleSpeed(obj);
    PID_Reset(&obj->speed_pid);
    requested_total_nm = obj->manual_torque_nm * 2.0f;
  } else {
    // 目標車速を accel_limit_m_s2 で speed_setpoint_m_s へ近づける (急な指令変化でタイヤを
    // 滑らせないため)。IsStandstill() はこのレート制限後の値で判定する
    launching = UpdateLaunch(obj, dt_s);
    if (launching) {
      // 全開を要求して TC にグリップの限界へ絞らせる。目標車速を実車速に合わせ続け、PIへ戻す
      // ときは今の車速から残りをランプで詰める。積分は「その車速を保つトルク」から始める
      // (0 から始めると、積分が溜まるまで車速が落ち込む)
      float dir = obj->speed_setpoint_m_s >= 0.0f ? 1.0f : -1.0f;
      obj->target_speed_m_s = obj->vehicle_speed_m_s;
      PID_Reset(&obj->speed_pid);
      if (obj->speed_pid.ki > 0.0f) {
        obj->speed_pid.integral = dir * obj->params.launch_exit_torque_nm * 2.0f / obj->speed_pid.ki;
      }
      obj->speed_ff_accel_m_s2 = 0.0f;
      requested_total_nm = dir * DRIVE_MAX_TOTAL_TORQUE_NM;
    } else {
      float speed_step = obj->accel_limit_m_s2 * dt_s;
      float step_m_s = Constrain(obj->speed_setpoint_m_s - obj->target_speed_m_s, -speed_step, speed_step);
      obj->target_speed_m_s += step_m_s;
      if (IsStandstill(obj)) {
        obj->launch_spent = false;
        obj->speed_ff_accel_m_s2 = 0.0f;
        CoastStandstill(obj);
        return;
      }
      requested_total_nm = PID_Update(&obj->speed_pid, obj->target_speed_m_s, obj->vehicle_speed_m_s) +
                           SpeedFeedforward(obj, step_m_s, dt_s);
      requested_total_nm =
          Constrain(requested_total_nm, -DRIVE_MAX_TOTAL_TORQUE_NM, DRIVE_MAX_TOTAL_TORQUE_NM);
    }
  }

  // トルクベクタリングには実測ヨーレートが要る。IMU が使えないときの代用値 (舵角からの
  // 幾何計算) は、車体が滑って曲がれていなくても横加速度が出ていることにしてしまう
  float diff_nm = 0.0f;
  if (obj->yaw_rate_measured) {
    diff_nm = TorqueVectoring_Update(&obj->tv, requested_total_nm, obj->vehicle_speed_m_s,
                                     obj->yaw_rate_rad_s);
  } else {
    TorqueVectoring_Reset(&obj->tv);
  }

  // 左右等配分 + トルク差。上限に当たらない限り総駆動力は変わらず、車速制御と干渉しない
  float request_left_nm;
  float request_right_nm;
  AllocateTorque(requested_total_nm, diff_nm, &request_left_nm, &request_right_nm);
  obj->torque_request_left_nm = request_left_nm;
  obj->torque_request_right_nm = request_right_nm;

  // 各輪のトルクの向きへのスリップ速度と、反対の輪に対する先走り (前輪に依らない)。
  // 先走りからは、旋回で内外輪に付く正常な速度差 (基準速度の差 = ヨーレート×トレッド) を引く
  float excess_left_m_s = obj->rear_speed_left_m_s - obj->slip_ref_left_m_s;
  float excess_right_m_s = obj->rear_speed_right_m_s - obj->slip_ref_right_m_s;
  float slip_left_m_s = SlipSpeedAlongTorque(obj->rear_speed_left_m_s, obj->slip_ref_left_m_s, request_left_nm);
  float slip_right_m_s =
      SlipSpeedAlongTorque(obj->rear_speed_right_m_s, obj->slip_ref_right_m_s, request_right_nm);
  float lead_left_m_s = SlipSpeedAlongTorque(excess_left_m_s, excess_right_m_s, request_left_nm);
  float lead_right_m_s = SlipSpeedAlongTorque(excess_right_m_s, excess_left_m_s, request_right_nm);

  bool lift_left = false;
  bool lift_right = false;
  obj->tc_limit_left_nm = UpdateWheelLimit(obj, &obj->tc_left, request_left_nm, obj->torque_left_nm,
                                           slip_left_m_s, lead_left_m_s, obj->slip_ref_left_m_s, dt_s,
                                           &lift_left);
  obj->tc_limit_right_nm = UpdateWheelLimit(obj, &obj->tc_right, request_right_nm, obj->torque_right_nm,
                                            slip_right_m_s, lead_right_m_s, obj->slip_ref_right_m_s, dt_s,
                                            &lift_right);

  // 片輪を絞っても反対の輪は要求のまま (絞った分を載せない)
  float left_nm = Constrain(request_left_nm, -obj->tc_limit_left_nm, obj->tc_limit_left_nm);
  float right_nm = Constrain(request_right_nm, -obj->tc_limit_right_nm, obj->tc_limit_right_nm);
  bool limited_left = left_nm != request_left_nm;
  bool limited_right = right_nm != request_right_nm;

  // 後輪周速が物理的にあり得ない値まで来たら、判定に関係なくその輪のトルクを0にする (最終防波堤)
  if (obj->wheel_lift_guard_enabled) {
    if (Abs(obj->rear_speed_left_m_s) > DRIVE_WHEEL_LIFT_MAX_WHEEL_SPEED_M_S) {
      left_nm = 0.0f;
      limited_left = lift_left = true;
    }
    if (Abs(obj->rear_speed_right_m_s) > DRIVE_WHEEL_LIFT_MAX_WHEEL_SPEED_M_S) {
      right_nm = 0.0f;
      limited_right = lift_right = true;
    }
  }
  obj->tc_limiting = limited_left || limited_right;
  obj->wheel_lift_limiting = (limited_left && lift_left) || (limited_right && lift_right);

  left_nm = ApplyOverspeedLimit(obj, left_nm);
  right_nm = ApplyOverspeedLimit(obj, right_nm);

  // torque_mode・ローンチコントロール中は PID を使っていない (毎周期リセット済み) ので
  // 巻き戻しは無意味 (積分 0 から巻き戻すと、PIへ戻した瞬間に逆向きのトルクが出る)
  if (!obj->torque_mode_active && !launching) {
    UnwindIntegral(obj, requested_total_nm, left_nm + right_nm, dt_s);
  }
  SendTorque(obj, left_nm, right_nm);
}

// 調整パラメータのうち、エンコーダの校正と車速のカルマンフィルタの調整値を各モジュールへ渡す
static void ApplySensorParams(Drive* obj) {
  const ControlParams* p = &obj->params;
  const EncoderCal left = {p->enc_left_lo, p->enc_left_hi, p->enc_left_h1s, p->enc_left_h1c, p->enc_left_h2s,
                           p->enc_left_h2c};
  const EncoderCal right = {p->enc_right_lo, p->enc_right_hi, p->enc_right_h1s, p->enc_right_h1c,
                            p->enc_right_h2s, p->enc_right_h2c};
  Encoder_SetCalibration(obj->encoder, &left, &right);
  const SpeedKfConfig kf = {p->kf_sigma_s_m, p->kf_sigma_a_m_s2, p->kf_sigma_a_per_m_s, p->kf_jerk_psd,
                            p->kf_bias_walk_psd};
  SpeedKf_SetConfig(&obj->speed_kf, &kf);
}

void Drive_SetParams(Drive* obj, const ControlParams* params) {
  obj->params = *params;
  obj->speed_pid.kp = params->speed_kp;
  obj->speed_pid.ki = params->speed_ki;
  ApplySensorParams(obj);
}

void Drive_SetTargetSpeed(Drive* obj, float m_s, float accel_limit_m_s2) {
  obj->speed_setpoint_m_s = Constrain(m_s, -DRIVE_MAX_SPEED_M_S, DRIVE_MAX_SPEED_M_S);
  // 上位が指定した値に上限は掛けない (drive.h の DRIVE_DEFAULT_ACCEL_M_S2 の注記)
  if (accel_limit_m_s2 > 0.0f) {
    obj->accel_limit_m_s2 =
        accel_limit_m_s2 > DRIVE_MIN_ACCEL_LIMIT_M_S2 ? accel_limit_m_s2 : DRIVE_MIN_ACCEL_LIMIT_M_S2;
  } else {
    obj->accel_limit_m_s2 = DRIVE_DEFAULT_ACCEL_M_S2;
  }
}

void Drive_SetBrake(Drive* obj, bool on, float torque_nm) {
  obj->brake_active = on;
  obj->brake_torque_nm = Constrain(torque_nm, 0.0f, DRIVE_MAX_BRAKE_TORQUE_NM);
}

void Drive_SetTorque(Drive* obj, bool on, float torque_nm) {
  obj->torque_mode_active = on;
  obj->manual_torque_nm = Constrain(torque_nm, -DRIVE_MAX_TORQUE_NM, DRIVE_MAX_TORQUE_NM);
}

void Drive_SetLaunch(Drive* obj, bool on) {
  obj->launch_requested = on;
}

bool Drive_IsLaunchActive(const Drive* obj) {
  return obj->enabled && obj->launching;
}

void Drive_SetSideBrake(Drive* obj, bool on) {
  obj->side_brake_active = on;
  if (!on) obj->side_brake_engaged = false;
}

bool Drive_IsSideBrakeEngaged(const Drive* obj) {
  return obj->side_brake_engaged;
}

void Drive_SetTorqueVectoringEnabled(Drive* obj, bool enabled) {
  TorqueVectoring_SetEnabled(&obj->tv, enabled);
}

void Drive_SetTractionControlEnabled(Drive* obj, bool enabled) {
  obj->tc_enabled = enabled;
}

void Drive_SetWheelLiftGuardEnabled(Drive* obj, bool enabled) {
  obj->wheel_lift_guard_enabled = enabled;
}

void Drive_SetAbsEnabled(Drive* obj, bool enabled) {
  obj->abs_enabled = enabled;
}

bool Drive_IsAbsActive(const Drive* obj) {
  return obj->enabled && !obj->side_brake_active && obj->brake_active && obj->abs_enabled &&
         !obj->abs_fallback_latched && obj->abs_limit_nm < obj->brake_torque_nm;
}

void Drive_Enable(Drive* obj) {
  PID_Reset(&obj->speed_pid);
  ResetTraction(obj);
  // 無効化中に古い目標車速が残っていると再有効化した瞬間に急発進するため、0から始める
  obj->speed_setpoint_m_s = 0.0f;
  obj->target_speed_m_s = 0.0f;
  obj->speed_ff_accel_m_s2 = 0.0f;
  obj->launching = false;
  obj->launch_spent = false;
  obj->side_brake_engaged = false;
  TorqueVectoring_Reset(&obj->tv);
  Timer_Reset(&obj->timer);
  obj->enabled = true;
}

void Drive_Disable(Drive* obj) {
  obj->enabled = false;
  obj->speed_setpoint_m_s = 0.0f;
  obj->target_speed_m_s = 0.0f;
  obj->side_brake_engaged = false;
  TorqueVectoring_Reset(&obj->tv);
  Coast(obj);
}

bool Drive_IsEnabled(const Drive* obj) {
  return obj->enabled;
}

float Drive_GetVehicleSpeed(const Drive* obj) {
  return obj->vehicle_speed_m_s;
}

float Drive_GetAccelKf(const Drive* obj) {
  return SpeedKf_GetAccel(&obj->speed_kf);
}

void Drive_TakeKfDiag(Drive* obj, float nis[SPEED_KF_DIAG_COUNT], float autocorr[SPEED_KF_DIAG_COUNT]) {
  for (int c = 0; c < SPEED_KF_DIAG_COUNT; c++) {
    SpeedKf_TakeDiag(&obj->speed_kf, (SpeedKfDiagChannel)c, &nis[c], &autocorr[c]);
  }
}

float Drive_GetAccelBiasKf(const Drive* obj) {
  return SpeedKf_GetAccelBias(&obj->speed_kf);
}

uint32_t Drive_GetKfGatedCount(const Drive* obj) {
  return obj->speed_kf.gated_count;
}

float Drive_GetSlipLeft(const Drive* obj) {
  return obj->slip_left;
}

float Drive_GetSlipRight(const Drive* obj) {
  return obj->slip_right;
}

float Drive_GetTcLimitLeft(const Drive* obj) {
  return obj->tc_limit_left_nm;
}

float Drive_GetTcLimitRight(const Drive* obj) {
  return obj->tc_limit_right_nm;
}

bool Drive_IsTractionControlActive(const Drive* obj) {
  return obj->enabled && obj->tc_limiting && !obj->wheel_lift_limiting;
}

bool Drive_IsWheelLiftGuardActive(const Drive* obj) {
  return obj->enabled && obj->wheel_lift_limiting;
}

bool Drive_IsTorqueVectoringActive(const Drive* obj) {
  return obj->enabled && TorqueVectoring_IsActive(&obj->tv);
}

float Drive_GetTvRatio(const Drive* obj) {
  return TorqueVectoring_GetRatio(&obj->tv);
}

float Drive_GetTvYawMoment(const Drive* obj) {
  return TorqueVectoring_GetYawMoment(&obj->tv);
}

float Drive_GetAbsLimit(const Drive* obj) {
  return obj->abs_limit_nm;
}

float Drive_GetTorqueRequestLeft(const Drive* obj) {
  return obj->torque_request_left_nm;
}

float Drive_GetTorqueRequestRight(const Drive* obj) {
  return obj->torque_request_right_nm;
}

float Drive_GetTorqueLeft(const Drive* obj) {
  return obj->torque_left_nm;
}

float Drive_GetTorqueRight(const Drive* obj) {
  return obj->torque_right_nm;
}
