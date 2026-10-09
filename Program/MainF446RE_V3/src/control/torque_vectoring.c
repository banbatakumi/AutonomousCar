#include "torque_vectoring.h"

#include "mymath.h"

// ヨーモーメント Mz [Nm] ↔ 左右トルク差 ΔT = T_right − T_left [Nm] の換算。
// 駆動力の差 ΔF = ΔT / r_wheel がトレッドの半分の腕で効くので Mz = ΔF * track/2
static float YawMomentToDiffTorque(const TorqueVectoring* obj, float moment_nm) {
  return moment_nm * 2.0f * obj->wheel_radius_m / obj->track_m;
}

static float DiffTorqueToYawMoment(const TorqueVectoring* obj, float diff_nm) {
  return diff_nm * obj->track_m / (2.0f * obj->wheel_radius_m);
}

void TorqueVectoring_Init(TorqueVectoring* obj, const ControlParams* params, float track_m,
                          float wheel_radius_m) {
  obj->params = params;
  obj->track_m = track_m;
  obj->wheel_radius_m = wheel_radius_m;

  obj->enabled = true;
  TorqueVectoring_Reset(obj);
}

float TorqueVectoring_Update(TorqueVectoring* obj, float total_torque_nm, float speed_m_s,
                             float yaw_rate_rad_s) {
  const ControlParams* p = obj->params;

  if (!obj->enabled) {
    TorqueVectoring_Reset(obj);
    return 0.0f;
  }

  // 同定用: 配分を止めて指定のヨーモーメントだけを出す (ヨーモーメント→ヨーレートの応答を測る)
  if (p->tv_test_moment_nm != 0.0f) {
    obj->ratio = 0.0f;
    obj->diff_torque_nm = YawMomentToDiffTorque(obj, p->tv_test_moment_nm);
    return obj->diff_torque_nm;
  }

  float lateral_accel_m_s2 = speed_m_s * yaw_rate_rad_s;
  obj->ratio = Constrain(p->tv_load_gain_s2_per_m * lateral_accel_m_s2, -p->tv_max_ratio, p->tv_max_ratio);
  obj->diff_torque_nm = total_torque_nm * obj->ratio;
  return obj->diff_torque_nm;
}

void TorqueVectoring_Reset(TorqueVectoring* obj) {
  obj->ratio = 0.0f;
  obj->diff_torque_nm = 0.0f;
}

void TorqueVectoring_SetEnabled(TorqueVectoring* obj, bool enabled) {
  if (enabled == obj->enabled) return;
  obj->enabled = enabled;
  TorqueVectoring_Reset(obj);
}

bool TorqueVectoring_IsEnabled(const TorqueVectoring* obj) {
  return obj->enabled;
}

bool TorqueVectoring_IsActive(const TorqueVectoring* obj) {
  return Abs(obj->diff_torque_nm) > TV_ACTIVE_TORQUE_NM;
}

float TorqueVectoring_GetRatio(const TorqueVectoring* obj) {
  return obj->ratio;
}

float TorqueVectoring_GetDiffTorque(const TorqueVectoring* obj) {
  return obj->diff_torque_nm;
}

float TorqueVectoring_GetYawMoment(const TorqueVectoring* obj) {
  return DiffTorqueToYawMoment(obj, obj->diff_torque_nm);
}
