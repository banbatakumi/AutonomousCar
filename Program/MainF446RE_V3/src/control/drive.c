#include "drive.h"

#include "mymath.h"

#define DRIVE_MAX_TOTAL_TORQUE_NM (DRIVE_MAX_TORQUE_NM * 2.0f)

// ---------------------------------------------------------------------------
// 観測量の更新
// ---------------------------------------------------------------------------

// 前輪は操舵されているため、その転がり速度は車体前後方向速度そのものではなく v/cos(delta) になる。
// 逆に cos(delta) を掛けて車体前後方向の速度へ戻す。左右の平均を取ることで、旋回による
// 左右の速度差 (前輪トレッド分) は相殺される。
static float EstimateVehicleSpeed(Drive* obj) {
  float omega_left = LPF_Update(&obj->lpf_front_left,
                                Encoder_GetAngularVelocityLeft(obj->encoder) * DRIVE_FRONT_LEFT_DIR);
  float omega_right = LPF_Update(&obj->lpf_front_right,
                                 Encoder_GetAngularVelocityRight(obj->encoder) * DRIVE_FRONT_RIGHT_DIR);

  // フィルタ後の各輪周速も残す。生の角速度は微分ノイズで静止中も ±12m/s 振れるため、
  // 上位へ報告する車輪速はここで作った値を使うこと
  obj->front_speed_left_m_s = omega_left * DRIVE_FRONT_WHEEL_RADIUS_M;
  obj->front_speed_right_m_s = omega_right * DRIVE_FRONT_WHEEL_RADIUS_M;

  // 射影に使うのはモータ機械角ではなく路面舵角。リンク比が1でない機体では別物になる
  float steer_rad = Steering_GetRoadWheelAngleRad(obj->steering);
  return (obj->front_speed_left_m_s + obj->front_speed_right_m_s) * 0.5f * Cos(steer_rad);
}

// TCのスリップ判定専用の車体速度推定。EstimateVehicleSpeed() と同じ計算だが、
// DRIVE_LPF_K_FRONT (τ≈10ms) ではなく軽い DRIVE_LPF_K_FRONT_TC (τ≈5ms) を使う。
// フル加速のようなランプ入力では前者の遅れが τ×加速度 ぶん基準速度を系統的に低く見せ、
// 遅れのほぼ無い後輪速度との差が「常時空転」という誤ったスリップ率を生むため、
// PID用の値とは別に用意する (詳細は DRIVE_LPF_K_FRONT_TC のコメント参照)
static float EstimateVehicleSpeedForSlip(Drive* obj) {
  float omega_left = LPF_Update(&obj->lpf_front_left_tc,
                                Encoder_GetAngularVelocityLeft(obj->encoder) * DRIVE_FRONT_LEFT_DIR);
  float omega_right = LPF_Update(&obj->lpf_front_right_tc,
                                 Encoder_GetAngularVelocityRight(obj->encoder) * DRIVE_FRONT_RIGHT_DIR);
  float steer_rad = Steering_GetRoadWheelAngleRad(obj->steering);
  return (omega_left + omega_right) * 0.5f * DRIVE_FRONT_WHEEL_RADIUS_M * Cos(steer_rad);
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
  float cos_steer = Cos(steer_rad);
  if (Abs(cos_steer) < 0.1f) return 0.0f;
  return obj->vehicle_speed_m_s * (Sin(steer_rad) / cos_steer) / DRIVE_WHEELBASE_M;
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

static void UpdateObservations(Drive* obj) {
  obj->vehicle_speed_m_s = EstimateVehicleSpeed(obj);
  float vehicle_speed_for_slip_m_s = EstimateVehicleSpeedForSlip(obj);
  obj->vehicle_speed_for_direction_m_s = vehicle_speed_for_slip_m_s;

  float omega_left = LPF_Update(&obj->lpf_rear_left,
                                BldcMotor_GetAngularSpeed(&obj->motors->rear_left) * DRIVE_REAR_LEFT_DIR);
  float omega_right = LPF_Update(&obj->lpf_rear_right,
                                 BldcMotor_GetAngularSpeed(&obj->motors->rear_right) * DRIVE_REAR_RIGHT_DIR);
  obj->rear_speed_left_m_s = omega_left * DRIVE_REAR_WHEEL_RADIUS_M;
  obj->rear_speed_right_m_s = omega_right * DRIVE_REAR_WHEEL_RADIUS_M;

  obj->yaw_rate_rad_s = EstimateYawRate(obj);

  // 旋回中は内輪と外輪で本来の速度が違う。ここを車体速度で共通化すると外輪が常時「空転」と
  // 判定されてTCが誤介入するため、ヨーレートから各輪の基準速度を作る。
  // 基準速度は PID フィードバック用 (vehicle_speed_m_s) ではなく、遅れの少ない
  // vehicle_speed_for_slip_m_s (EstimateVehicleSpeedForSlip 参照) を使う
  float half_track = DRIVE_REAR_TRACK_M * 0.5f;
  obj->slip_ref_left_m_s = vehicle_speed_for_slip_m_s - obj->yaw_rate_rad_s * half_track;
  obj->slip_ref_right_m_s = vehicle_speed_for_slip_m_s + obj->yaw_rate_rad_s * half_track;
  float floor_m_s = obj->params.slip_speed_floor_m_s;
  obj->slip_left = SlipRatio(obj->rear_speed_left_m_s, obj->slip_ref_left_m_s, floor_m_s);
  obj->slip_right = SlipRatio(obj->rear_speed_right_m_s, obj->slip_ref_right_m_s, floor_m_s);
}

// ---------------------------------------------------------------------------
// 出力
// ---------------------------------------------------------------------------

static void ResetTraction(Drive* obj);

// 指令の送信を止めて惰行させる (MD側は無通信0.5秒で停止モードに入る)
static void Coast(Drive* obj) {
  ResetTraction(obj);
  obj->torque_left_nm = 0.0f;
  obj->torque_right_nm = 0.0f;
  obj->torque_request_left_nm = 0.0f;
  obj->torque_request_right_nm = 0.0f;
  BldcMotor_Stop(&obj->motors->rear_left);
  BldcMotor_Stop(&obj->motors->rear_right);
}

// 後輪MDを制動モードに切り替えて左右へ同じ制動トルクを掛ける。制動トルクは回転を妨げる
// 向きに掛かる (向きはMD側が回転方向から決める) ため、こちらから符号を与える必要はない。
// 上位への報告だけは駆動と区別できるよう負値にする
static void SendBrake(Drive* obj, float nm) {
  PID_Reset(&obj->speed_pid);
  ResetTraction(obj);
  // MD の制動モードは左右へ同じトルクしか出せないので、TV はこの間ヨーモーメントを作れない。
  // 積分を持ち越すとブレーキを離した瞬間に溜まった分が一気に出る
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

// 目標車速も実車速もほぼ0の間は指令を止めて自由回転させる (惰行)。上位が明示的に
// ブレーキ (RAS_CMD_FLAG_BRAKE) を指定しない限り、停車中も車両を押さえ込まない。
// 積分・TV を持ち越すと再発進時に停止中に溜まった分が一気に出るのでリセットする
static void CoastStandstill(Drive* obj) {
  PID_Reset(&obj->speed_pid);
  TorqueVectoring_Reset(&obj->tv);
  Coast(obj);
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

// 総駆動トルクと左右差を各輪へ配り、トルクの絶対上限 (DRIVE_MAX_TORQUE_NM) に収める。
//
// 片方が上限に当たったら、**左右差を保ったまま**もう片方を同じだけ下げる (総駆動トルクが減る)。
// 個別にクランプするだけだと差が半分以下に削られ、全開 (両輪とも上限) では差がまったく付かない
// = 一番滑りやすい場面で TV が効かない。2026-10-05 までは総和を保つ方を優先して差を丸めていた
static void AllocateTorque(float total_nm, float diff_nm, float* left_nm, float* right_nm) {
  float left = (total_nm - diff_nm) * 0.5f;
  float right = (total_nm + diff_nm) * 0.5f;
  float left_clamped = Constrain(left, -DRIVE_MAX_TORQUE_NM, DRIVE_MAX_TORQUE_NM);
  float right_clamped = Constrain(right, -DRIVE_MAX_TORQUE_NM, DRIVE_MAX_TORQUE_NM);
  // 片方を削った分だけもう片方も同じ向きへ動かす (差は変わらない)。両方が削られるのは
  // 総駆動トルクだけで上限を超えているときで、そのときは両方とも上限に張り付く
  float left_cut = left_clamped - left;
  float right_cut = right_clamped - right;
  *left_nm = Constrain(left_clamped + right_cut, -DRIVE_MAX_TORQUE_NM, DRIVE_MAX_TORQUE_NM);
  *right_nm = Constrain(right_clamped + left_cut, -DRIVE_MAX_TORQUE_NM, DRIVE_MAX_TORQUE_NM);
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
  PID_Init(&obj->speed_pid, DRIVE_SPEED_KP, DRIVE_SPEED_KI, DRIVE_SPEED_KD,
           -DRIVE_MAX_TOTAL_TORQUE_NM, DRIVE_MAX_TOTAL_TORQUE_NM);
  TorqueVectoring_Init(&obj->tv, &obj->params, DRIVE_WHEELBASE_M, DRIVE_REAR_TRACK_M,
                       DRIVE_REAR_WHEEL_RADIUS_M);
  LPF_Init(&obj->lpf_front_left, DRIVE_LPF_K_FRONT, 0.0f);
  LPF_Init(&obj->lpf_front_right, DRIVE_LPF_K_FRONT, 0.0f);
  LPF_Init(&obj->lpf_front_left_tc, DRIVE_LPF_K_FRONT_TC, 0.0f);
  LPF_Init(&obj->lpf_front_right_tc, DRIVE_LPF_K_FRONT_TC, 0.0f);
  LPF_Init(&obj->lpf_rear_left, DRIVE_LPF_K_REAR, 0.0f);
  LPF_Init(&obj->lpf_rear_right, DRIVE_LPF_K_REAR, 0.0f);
  Timer_Init(&obj->timer);

  obj->enabled = false;
  obj->tc_enabled = true;
  obj->wheel_lift_guard_enabled = true;
  obj->speed_setpoint_m_s = 0.0f;
  obj->accel_limit_m_s2 = DRIVE_MAX_ACCEL_M_S2;
  obj->target_speed_m_s = 0.0f;
  obj->brake_active = false;
  obj->brake_torque_nm = DRIVE_MAX_BRAKE_TORQUE_NM;
  obj->torque_mode_active = false;
  obj->manual_torque_nm = 0.0f;
  obj->side_brake_active = false;
  obj->side_brake_engaged = false;
  obj->side_brake_target_left_rad = 0.0f;
  obj->side_brake_target_right_rad = 0.0f;

  obj->vehicle_speed_m_s = 0.0f;
  obj->vehicle_speed_for_direction_m_s = 0.0f;
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
  float dt_s = Timer_Read(&obj->timer);
  Timer_Reset(&obj->timer);
  if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.0f;  // 初回・呼び出しが飛んだ周期では時間積分をしない

  // 観測量は無効時も更新しておく (惰行中の車速をそのまま使って再発進できるようにするため)
  UpdateObservations(obj);

  // ABSの状態は制動モードの間だけ持ち越す。離したら次の制動を全量から始め、フォールバックも解く
  if (!(obj->enabled && !obj->side_brake_active && obj->brake_active && obj->abs_enabled)) {
    ResetAbs(obj);
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
  // 離脱時に停止中の目標車速から急発進しないよう、ここでも目標車速をレート制限の起点0へ戻す
  if (obj->brake_active) {
    obj->target_speed_m_s = 0.0f;
    obj->torque_request_left_nm = -obj->brake_torque_nm;
    obj->torque_request_right_nm = -obj->brake_torque_nm;
    SendBrake(obj, obj->abs_enabled ? ApplyAbs(obj, obj->brake_torque_nm, dt_s) : obj->brake_torque_nm);
    return;
  }

  float requested_total_nm;
  if (obj->torque_mode_active) {
    // 車速PIを迂回して指令トルクをそのまま使う。離脱時に積分が溜まったまま復帰しないよう
    // SendBrake と同様に毎周期リセットしておく。目標車速もブレーキ同様0へ戻す
    obj->target_speed_m_s = 0.0f;
    PID_Reset(&obj->speed_pid);
    requested_total_nm = obj->manual_torque_nm * 2.0f;
  } else {
    // 目標車速を accel_limit_m_s2 で speed_setpoint_m_s へ近づける (急な指令変化でタイヤを
    // 滑らせないため)。IsStandstill() はこのレート制限後の値で判定する
    float speed_step = obj->accel_limit_m_s2 * dt_s;
    obj->target_speed_m_s +=
        Constrain(obj->speed_setpoint_m_s - obj->target_speed_m_s, -speed_step, speed_step);
    if (IsStandstill(obj)) {
      CoastStandstill(obj);
      return;
    }
    requested_total_nm = PID_Update(&obj->speed_pid, obj->target_speed_m_s, obj->vehicle_speed_m_s);
  }

  // トルクベクタリングには実測ヨーレートが要る。IMU が使えないときの代用値 (舵角からの
  // 幾何計算) は規範モデルとほぼ同じ式なので、偏差が常に0付近になり制御として成立しない
  bool tv_active = obj->yaw_rate_measured;
  float diff_nm = 0.0f;
  if (tv_active) {
    diff_nm = TorqueVectoring_Update(&obj->tv, obj->vehicle_speed_m_s,
                                     Steering_GetRoadWheelAngleRad(obj->steering),
                                     obj->yaw_rate_rad_s, dt_s);
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

  // TV へは「要求した差のうち実際に効いた分」を返す (積分の巻き戻し用)。TV の要求が 0 でも
  // TC の絞りで付いていたはずの左右差は、TV の出力ではないので引く
  if (tv_active) {
    float half_nm = Constrain(requested_total_nm * 0.5f, -DRIVE_MAX_TORQUE_NM, DRIVE_MAX_TORQUE_NM);
    float base_left_nm = ApplyOverspeedLimit(obj, Constrain(half_nm, -obj->tc_limit_left_nm, obj->tc_limit_left_nm));
    float base_right_nm =
        ApplyOverspeedLimit(obj, Constrain(half_nm, -obj->tc_limit_right_nm, obj->tc_limit_right_nm));
    float applied_diff_nm = right_nm - left_nm;
    TorqueVectoring_ReportApplied(&obj->tv, applied_diff_nm - (base_right_nm - base_left_nm), applied_diff_nm,
                                  dt_s);
  }

  // torque_mode 中は PID を使っていない (毎周期リセット済み) ので巻き戻しは無意味
  if (!obj->torque_mode_active) UnwindIntegral(obj, requested_total_nm, left_nm + right_nm, dt_s);
  SendTorque(obj, left_nm, right_nm);
}

void Drive_SetParams(Drive* obj, const ControlParams* params) {
  obj->params = *params;
}

void Drive_SetTargetSpeed(Drive* obj, float m_s, float accel_limit_m_s2) {
  obj->speed_setpoint_m_s = Constrain(m_s, -DRIVE_MAX_SPEED_M_S, DRIVE_MAX_SPEED_M_S);
  obj->accel_limit_m_s2 = accel_limit_m_s2 > 0.0f
                               ? Constrain(accel_limit_m_s2, 0.01f, DRIVE_MAX_ACCEL_M_S2)
                               : DRIVE_MAX_ACCEL_M_S2;
}

void Drive_SetBrake(Drive* obj, bool on, float torque_nm) {
  obj->brake_active = on;
  obj->brake_torque_nm = Constrain(torque_nm, 0.0f, DRIVE_MAX_BRAKE_TORQUE_NM);
}

void Drive_SetTorque(Drive* obj, bool on, float torque_nm) {
  obj->torque_mode_active = on;
  obj->manual_torque_nm = Constrain(torque_nm, -DRIVE_MAX_TORQUE_NM, DRIVE_MAX_TORQUE_NM);
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

float Drive_GetVehicleSpeedForDirection(const Drive* obj) {
  return obj->vehicle_speed_for_direction_m_s;
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

float Drive_GetTargetYawRate(const Drive* obj) {
  return TorqueVectoring_GetTargetYawRate(&obj->tv);
}

float Drive_GetYawMomentTorque(const Drive* obj) {
  return TorqueVectoring_GetDiffTorque(&obj->tv);
}

float Drive_GetTvYawMoment(const Drive* obj) {
  return obj->tv.yaw_moment_nm;
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
