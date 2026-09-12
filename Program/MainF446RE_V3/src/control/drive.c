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
// DRIVE_LPF_K_FRONT (τ≈100ms) ではなく軽い DRIVE_LPF_K_FRONT_TC (τ≈25ms) を使う。
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

// スリップ率 = (駆動輪周速 - その輪が本来出るべき速度) / 基準速度。
// 低速域は分母が0に近づいて発散するため、TCごと無効化する。
static float SlipRatio(float wheel_speed_m_s, float reference_speed_m_s) {
  float denominator = Abs(reference_speed_m_s);
  if (denominator < DRIVE_TC_MIN_SPEED_M_S) return 0.0f;
  return (wheel_speed_m_s - reference_speed_m_s) / denominator;
}

static void UpdateObservations(Drive* obj) {
  obj->vehicle_speed_m_s = EstimateVehicleSpeed(obj);
  float vehicle_speed_for_slip_m_s = EstimateVehicleSpeedForSlip(obj);

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
  float reference_left = vehicle_speed_for_slip_m_s - obj->yaw_rate_rad_s * half_track;
  float reference_right = vehicle_speed_for_slip_m_s + obj->yaw_rate_rad_s * half_track;
  obj->slip_left = SlipRatio(obj->rear_speed_left_m_s, reference_left);
  obj->slip_right = SlipRatio(obj->rear_speed_right_m_s, reference_right);
}

// 後輪左右の速度差から、ヨーレートで期待される差 (旋回による正常な差) を差し引いた異常成分。
// 正なら左輪が右輪に対して異常に速い (左が浮いている可能性)、負なら右輪側。
// 前輪基準速度を使わないため、DRIVE_TC_MIN_SPEED_M_S 未満の低速域でも機能する。
static float WheelSpeedDiffAnomaly(const Drive* obj) {
  float diff_raw = obj->rear_speed_left_m_s - obj->rear_speed_right_m_s;
  float diff_expected = -obj->yaw_rate_rad_s * DRIVE_REAR_TRACK_M;  // reference_left - reference_right相当
  return diff_raw - diff_expected;
}

// ---------------------------------------------------------------------------
// 出力
// ---------------------------------------------------------------------------

// 指令の送信を止めて惰行させる (MD側は無通信0.5秒で停止モードに入る)
static void Coast(Drive* obj) {
  obj->torque_left_nm = 0.0f;
  obj->torque_right_nm = 0.0f;
  BldcMotor_Stop(&obj->motors->rear_left);
  BldcMotor_Stop(&obj->motors->rear_right);
}

// 後輪MDを制動モードに切り替えて左右へ同じ制動トルクを掛ける。制動トルクは回転を妨げる
// 向きに掛かる (向きはMD側が回転方向から決める) ため、こちらから符号を与える必要はない。
// 上位への報告だけは駆動と区別できるよう負値にする
static void SendBrake(Drive* obj, float nm) {
  PID_Reset(&obj->speed_pid);
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
      SendBrake(obj, DRIVE_MAX_BRAKE_TORQUE_NM);
      return;
    }
    obj->side_brake_target_left_rad = BldcMotor_GetMechAngle(&obj->motors->rear_left);
    obj->side_brake_target_right_rad = BldcMotor_GetMechAngle(&obj->motors->rear_right);
    obj->side_brake_engaged = true;
  }

  obj->torque_left_nm = 0.0f;
  obj->torque_right_nm = 0.0f;
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

// スリップ超過中はトルク上限を削り、グリップが戻ったらゆっくり戻す。実車のTC ECU と同じ
// 「即座に削って緩やかに復帰」型。スリップ率の微分を使わないのでノイズに強い。
// しきい値超過が DRIVE_TC_SLIP_DEBOUNCE_S 継続するまではカットを始めない (デバウンス)。
// EstimateVehicleSpeedForSlip() の軽いフィルタで残るノイズ由来の一瞬の超過を無視するため。
// 本物の空転は超過が持続するのでこの遅延はほぼ影響しない
static float UpdateTractionLimit(float limit_nm, float slip, float dt_s, float* excess_time_s) {
  // slip は正=空転・負=ロック傾向 (drive.h の slip_left/right 参照)。制動 (SendBrake/
  // SendSideBrake) はここを迂回する別経路なので、このパスで意味を持つ異常は空転側だけ。
  // 負のスリップは減速などによる一時的な基準速度割れに過ぎず、駆動トルクを削る理由にならない
  // ため Abs() を取らず符号付きで判定する
  float excess = slip - DRIVE_TC_SLIP_THRESHOLD;
  if (excess > 0.0f) {
    *excess_time_s += dt_s;
    if (*excess_time_s >= DRIVE_TC_SLIP_DEBOUNCE_S) {
      limit_nm -= DRIVE_TC_CUT_GAIN * excess * dt_s;
    }
  } else {
    *excess_time_s = 0.0f;
    limit_nm += DRIVE_TC_RECOVER_RATE * dt_s;
  }
  return Constrain(limit_nm, DRIVE_TC_MIN_TORQUE_NM, DRIVE_MAX_TORQUE_NM);
}

// 片輪浮き対策: 左右速度差の異常成分が超過している間はトルク上限を削り、収まったら
// ゆっくり戻す。上のUpdateTractionLimitと同じ「即座に削って緩やかに復帰」型だが、
// 状態(limit_nm)・しきい値ともTC本体とは独立に持つ (RasConfig経由で個別にON/OFFする要件のため)。
static float UpdateWheelLiftLimit(float limit_nm, float excess, float dt_s) {
  if (excess > 0.0f) {
    limit_nm -= DRIVE_WHEEL_LIFT_CUT_GAIN * excess * dt_s;
  } else {
    limit_nm += DRIVE_WHEEL_LIFT_RECOVER_RATE * dt_s;
  }
  return Constrain(limit_nm, 0.0f, DRIVE_MAX_TORQUE_NM);
}

// 後輪周速が物理的にあり得ない絶対値まで来たら、基準速度や左右差の判定結果に関係なく
// 即座に上限を0にする (最終防波堤)
static float ApplyWheelLiftHardSpeedLimit(float limit_nm, float wheel_speed_m_s) {
  if (Abs(wheel_speed_m_s) > DRIVE_WHEEL_LIFT_MAX_WHEEL_SPEED_M_S) return 0.0f;
  return limit_nm;
}

// 左右トルク差を、両輪ともTCの上限に収まる範囲へ丸める。
//
// 先に丸めるのが肝で、丸めずに配分してから左右を個別にクランプすると削られ方が左右非対称に
// なり、要求したのと違うヨーモーメントが残る (総駆動トルクが大きいほど片側だけが削られる)。
// left = (total - d)/2, right = (total + d)/2 を各輪の上限に収める条件から d の範囲が決まる。
static float LimitDiffTorque(float limit_left_nm, float limit_right_nm, float total_nm, float diff_nm) {
  float min_by_left = total_nm - 2.0f * limit_left_nm;
  float min_by_right = -2.0f * limit_right_nm - total_nm;
  float min_diff = min_by_left > min_by_right ? min_by_left : min_by_right;

  float max_by_left = total_nm + 2.0f * limit_left_nm;
  float max_by_right = 2.0f * limit_right_nm - total_nm;
  float max_diff = max_by_left < max_by_right ? max_by_left : max_by_right;

  // 総駆動トルクだけで既に両輪の上限を超えている場合は差を付ける余力が無い。
  // ここで無理に範囲へ寄せると左右非対称な飽和になるので、等配分 (差0) に倒す
  if (min_diff > max_diff) return 0.0f;
  return Constrain(diff_nm, min_diff, max_diff);
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

  PID_Init(&obj->speed_pid, DRIVE_SPEED_KP, DRIVE_SPEED_KI, DRIVE_SPEED_KD,
           -DRIVE_MAX_TOTAL_TORQUE_NM, DRIVE_MAX_TOTAL_TORQUE_NM);
  TorqueVectoring_Init(&obj->tv, DRIVE_WHEELBASE_M, DRIVE_REAR_TRACK_M, DRIVE_REAR_WHEEL_RADIUS_M);
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
  obj->yaw_rate_rad_s = 0.0f;
  obj->yaw_rate_measured = false;
  obj->front_speed_left_m_s = 0.0f;
  obj->front_speed_right_m_s = 0.0f;
  obj->rear_speed_left_m_s = 0.0f;
  obj->rear_speed_right_m_s = 0.0f;
  obj->slip_left = 0.0f;
  obj->slip_right = 0.0f;
  obj->tc_slip_excess_time_left_s = 0.0f;
  obj->tc_slip_excess_time_right_s = 0.0f;
  obj->tc_limit_left_nm = DRIVE_MAX_TORQUE_NM;
  obj->tc_limit_right_nm = DRIVE_MAX_TORQUE_NM;
  obj->wheel_lift_limit_left_nm = DRIVE_MAX_TORQUE_NM;
  obj->wheel_lift_limit_right_nm = DRIVE_MAX_TORQUE_NM;
  obj->torque_left_nm = 0.0f;
  obj->torque_right_nm = 0.0f;
}

void Drive_Update(Drive* obj) {
  float dt_s = Timer_Read(&obj->timer);
  Timer_Reset(&obj->timer);
  if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.0f;  // 初回・呼び出しが飛んだ周期では時間積分をしない

  // 観測量は無効時も更新しておく (惰行中の車速をそのまま使って再発進できるようにするため)
  UpdateObservations(obj);

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
    SendBrake(obj, obj->brake_torque_nm);
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

  if (obj->tc_enabled) {
    obj->tc_limit_left_nm = UpdateTractionLimit(obj->tc_limit_left_nm, obj->slip_left, dt_s,
                                                &obj->tc_slip_excess_time_left_s);
    obj->tc_limit_right_nm = UpdateTractionLimit(obj->tc_limit_right_nm, obj->slip_right, dt_s,
                                                 &obj->tc_slip_excess_time_right_s);
  } else {
    obj->tc_limit_left_nm = DRIVE_MAX_TORQUE_NM;
    obj->tc_limit_right_nm = DRIVE_MAX_TORQUE_NM;
    obj->tc_slip_excess_time_left_s = 0.0f;
    obj->tc_slip_excess_time_right_s = 0.0f;
  }

  if (obj->wheel_lift_guard_enabled) {
    // WheelSpeedDiffAnomaly() の符号は「左が右より速いか」を表すだけで、前進中の解釈
    // (anomaly>0 なら左が浮いている) は後退中は逆転する (後退中は浮いて空転している輪ほど
    // より負に大きい値になるため)。進行方向で正規化してから前進基準の符号判定を再利用する
    float dir = (obj->rear_speed_left_m_s + obj->rear_speed_right_m_s) >= 0.0f ? 1.0f : -1.0f;
    float anomaly = WheelSpeedDiffAnomaly(obj) * dir;
    float excess = Abs(anomaly) - DRIVE_WHEEL_LIFT_DIFF_THRESHOLD_M_S;
    // 異常に速い方だけを絞る。excessが負のとき (=閾値未満) は両輪とも回復させる
    float excess_left = anomaly > 0.0f ? excess : -1.0f;
    float excess_right = anomaly < 0.0f ? excess : -1.0f;
    obj->wheel_lift_limit_left_nm = UpdateWheelLiftLimit(obj->wheel_lift_limit_left_nm, excess_left, dt_s);
    obj->wheel_lift_limit_right_nm = UpdateWheelLiftLimit(obj->wheel_lift_limit_right_nm, excess_right, dt_s);
    obj->wheel_lift_limit_left_nm =
        ApplyWheelLiftHardSpeedLimit(obj->wheel_lift_limit_left_nm, obj->rear_speed_left_m_s);
    obj->wheel_lift_limit_right_nm =
        ApplyWheelLiftHardSpeedLimit(obj->wheel_lift_limit_right_nm, obj->rear_speed_right_m_s);
  } else {
    obj->wheel_lift_limit_left_nm = DRIVE_MAX_TORQUE_NM;
    obj->wheel_lift_limit_right_nm = DRIVE_MAX_TORQUE_NM;
  }

  // TC本体と片輪浮き対策はそれぞれ独立に上限を決めるため、実際に使う上限は両者の小さい方
  float effective_limit_left_nm =
      obj->tc_limit_left_nm < obj->wheel_lift_limit_left_nm ? obj->tc_limit_left_nm : obj->wheel_lift_limit_left_nm;
  float effective_limit_right_nm = obj->tc_limit_right_nm < obj->wheel_lift_limit_right_nm
                                        ? obj->tc_limit_right_nm
                                        : obj->wheel_lift_limit_right_nm;

  // トルクベクタリングには実測ヨーレートが要る。IMU が使えないときの代用値 (舵角からの
  // 幾何計算) は規範モデルとほぼ同じ式なので、偏差が常に0付近になり制御として成立しない
  float diff_nm = 0.0f;
  if (obj->yaw_rate_measured) {
    diff_nm = TorqueVectoring_Update(&obj->tv, obj->vehicle_speed_m_s,
                                     Steering_GetRoadWheelAngleRad(obj->steering),
                                     obj->yaw_rate_rad_s, dt_s);
    diff_nm = LimitDiffTorque(effective_limit_left_nm, effective_limit_right_nm, requested_total_nm, diff_nm);
    TorqueVectoring_ReportApplied(&obj->tv, diff_nm, dt_s);
  } else {
    TorqueVectoring_Reset(&obj->tv);
  }

  // 左右等配分 + トルク差。差だけを付けるので総駆動力は変わらず、車速制御と干渉しない
  float left_nm = (requested_total_nm - diff_nm) * 0.5f;
  float right_nm = (requested_total_nm + diff_nm) * 0.5f;

  left_nm = Constrain(left_nm, -effective_limit_left_nm, effective_limit_left_nm);
  right_nm = Constrain(right_nm, -effective_limit_right_nm, effective_limit_right_nm);

  left_nm = ApplyOverspeedLimit(obj, left_nm);
  right_nm = ApplyOverspeedLimit(obj, right_nm);

  // torque_mode 中は PID を使っていない (毎周期リセット済み) ので巻き戻しは無意味
  if (!obj->torque_mode_active) UnwindIntegral(obj, requested_total_nm, left_nm + right_nm, dt_s);
  SendTorque(obj, left_nm, right_nm);
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

void Drive_Enable(Drive* obj) {
  PID_Reset(&obj->speed_pid);
  obj->tc_limit_left_nm = DRIVE_MAX_TORQUE_NM;
  obj->tc_limit_right_nm = DRIVE_MAX_TORQUE_NM;
  obj->tc_slip_excess_time_left_s = 0.0f;
  obj->tc_slip_excess_time_right_s = 0.0f;
  obj->wheel_lift_limit_left_nm = DRIVE_MAX_TORQUE_NM;
  obj->wheel_lift_limit_right_nm = DRIVE_MAX_TORQUE_NM;
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
  return obj->tc_limit_left_nm < DRIVE_MAX_TORQUE_NM || obj->tc_limit_right_nm < DRIVE_MAX_TORQUE_NM;
}

bool Drive_IsWheelLiftGuardActive(const Drive* obj) {
  return obj->wheel_lift_limit_left_nm < DRIVE_MAX_TORQUE_NM ||
         obj->wheel_lift_limit_right_nm < DRIVE_MAX_TORQUE_NM;
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

float Drive_GetTorqueLeft(const Drive* obj) {
  return obj->torque_left_nm;
}

float Drive_GetTorqueRight(const Drive* obj) {
  return obj->torque_right_nm;
}
