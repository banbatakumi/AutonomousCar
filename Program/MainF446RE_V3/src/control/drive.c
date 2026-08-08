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

// ヨーレートは IMU の実測値を優先する。舵角からの幾何計算 (自転車モデル) は前輪が滑ると
// 実際のヨーレートから乖離するため、まさにTCが必要な場面で基準速度が狂うことになる。
static float EstimateYawRate(Drive* obj) {
  if (obj->imu != NULL && Imu_IsReady(obj->imu)) {
    return Radians(Imu_GetData(obj->imu)->gyro_z);
  }

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

  float omega_left = LPF_Update(&obj->lpf_rear_left,
                                BldcMotor_GetAngularSpeed(&obj->motors->rear_left) * DRIVE_REAR_LEFT_DIR);
  float omega_right = LPF_Update(&obj->lpf_rear_right,
                                 BldcMotor_GetAngularSpeed(&obj->motors->rear_right) * DRIVE_REAR_RIGHT_DIR);
  obj->rear_speed_left_m_s = omega_left * DRIVE_REAR_WHEEL_RADIUS_M;
  obj->rear_speed_right_m_s = omega_right * DRIVE_REAR_WHEEL_RADIUS_M;

  obj->yaw_rate_rad_s = EstimateYawRate(obj);

  // 旋回中は内輪と外輪で本来の速度が違う。ここを車体速度で共通化すると外輪が常時「空転」と
  // 判定されてTCが誤介入するため、ヨーレートから各輪の基準速度を作る。
  float half_track = DRIVE_REAR_TRACK_M * 0.5f;
  float reference_left = obj->vehicle_speed_m_s - obj->yaw_rate_rad_s * half_track;
  float reference_right = obj->vehicle_speed_m_s + obj->yaw_rate_rad_s * half_track;
  obj->slip_left = SlipRatio(obj->rear_speed_left_m_s, reference_left);
  obj->slip_right = SlipRatio(obj->rear_speed_right_m_s, reference_right);
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

// 停車保持。トルク制御は静止時の保持剛性がゼロなので、坂道では制動モードで押さえる
static void HoldStandstill(Drive* obj) {
  PID_Reset(&obj->speed_pid);
  obj->torque_left_nm = 0.0f;
  obj->torque_right_nm = 0.0f;
  BldcMotor_SetBrakeNm(&obj->motors->rear_left, DRIVE_STANDSTILL_BRAKE_NM);
  BldcMotor_SetBrakeNm(&obj->motors->rear_right, DRIVE_STANDSTILL_BRAKE_NM);
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
static float UpdateTractionLimit(float limit_nm, float slip, float dt_s) {
  float excess = Abs(slip) - DRIVE_TC_SLIP_THRESHOLD;
  if (excess > 0.0f) {
    limit_nm -= DRIVE_TC_CUT_GAIN * excess * dt_s;
  } else {
    limit_nm += DRIVE_TC_RECOVER_RATE * dt_s;
  }
  return Constrain(limit_nm, DRIVE_TC_MIN_TORQUE_NM, DRIVE_MAX_TORQUE_NM);
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
  LPF_Init(&obj->lpf_front_left, DRIVE_LPF_K_FRONT, 0.0);
  LPF_Init(&obj->lpf_front_right, DRIVE_LPF_K_FRONT, 0.0);
  LPF_Init(&obj->lpf_rear_left, DRIVE_LPF_K_REAR, 0.0);
  LPF_Init(&obj->lpf_rear_right, DRIVE_LPF_K_REAR, 0.0);
  Timer_Init(&obj->timer);

  obj->enabled = false;
  obj->target_speed_m_s = 0.0f;
  obj->yaw_moment_torque_nm = 0.0f;

  obj->vehicle_speed_m_s = 0.0f;
  obj->yaw_rate_rad_s = 0.0f;
  obj->front_speed_left_m_s = 0.0f;
  obj->front_speed_right_m_s = 0.0f;
  obj->rear_speed_left_m_s = 0.0f;
  obj->rear_speed_right_m_s = 0.0f;
  obj->slip_left = 0.0f;
  obj->slip_right = 0.0f;
  obj->tc_limit_left_nm = DRIVE_MAX_TORQUE_NM;
  obj->tc_limit_right_nm = DRIVE_MAX_TORQUE_NM;
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
  if (IsStandstill(obj)) {
    HoldStandstill(obj);
    return;
  }

  float requested_total_nm = PID_Update(&obj->speed_pid, obj->target_speed_m_s, obj->vehicle_speed_m_s);

  // 左右等配分 + ヨーモーメント項。左右差だけを付けるので総駆動力は変わらず、車速制御と干渉しない
  float left_nm = (requested_total_nm - obj->yaw_moment_torque_nm) * 0.5f;
  float right_nm = (requested_total_nm + obj->yaw_moment_torque_nm) * 0.5f;

  obj->tc_limit_left_nm = UpdateTractionLimit(obj->tc_limit_left_nm, obj->slip_left, dt_s);
  obj->tc_limit_right_nm = UpdateTractionLimit(obj->tc_limit_right_nm, obj->slip_right, dt_s);
  left_nm = Constrain(left_nm, -obj->tc_limit_left_nm, obj->tc_limit_left_nm);
  right_nm = Constrain(right_nm, -obj->tc_limit_right_nm, obj->tc_limit_right_nm);

  left_nm = ApplyOverspeedLimit(obj, left_nm);
  right_nm = ApplyOverspeedLimit(obj, right_nm);

  UnwindIntegral(obj, requested_total_nm, left_nm + right_nm, dt_s);
  SendTorque(obj, left_nm, right_nm);
}

void Drive_SetTargetSpeed(Drive* obj, float m_s) {
  obj->target_speed_m_s = Constrain(m_s, -DRIVE_MAX_SPEED_M_S, DRIVE_MAX_SPEED_M_S);
}

void Drive_SetYawMomentTorque(Drive* obj, float nm) {
  obj->yaw_moment_torque_nm = Constrain(nm, -DRIVE_MAX_TOTAL_TORQUE_NM, DRIVE_MAX_TOTAL_TORQUE_NM);
}

void Drive_Enable(Drive* obj) {
  PID_Reset(&obj->speed_pid);
  obj->tc_limit_left_nm = DRIVE_MAX_TORQUE_NM;
  obj->tc_limit_right_nm = DRIVE_MAX_TORQUE_NM;
  obj->yaw_moment_torque_nm = 0.0f;
  Timer_Reset(&obj->timer);
  obj->enabled = true;
}

void Drive_Disable(Drive* obj) {
  obj->enabled = false;
  obj->target_speed_m_s = 0.0f;
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

bool Drive_IsTractionControlActive(const Drive* obj) {
  return obj->tc_limit_left_nm < DRIVE_MAX_TORQUE_NM || obj->tc_limit_right_nm < DRIVE_MAX_TORQUE_NM;
}

float Drive_GetTorqueLeft(const Drive* obj) {
  return obj->torque_left_nm;
}

float Drive_GetTorqueRight(const Drive* obj) {
  return obj->torque_right_nm;
}
