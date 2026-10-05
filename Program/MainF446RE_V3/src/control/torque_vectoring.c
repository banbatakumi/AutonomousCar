#include "torque_vectoring.h"

#include <math.h>

#include "mymath.h"

// ヨーモーメント Mz [Nm] ↔ 左右トルク差 ΔT = T_right − T_left [Nm] の換算。
// 駆動力の差 ΔF = ΔT / r_wheel がトレッドの半分の腕で効くので Mz = ΔF * track/2
static float YawMomentToDiffTorque(const TorqueVectoring* obj, float moment_nm) {
  return moment_nm * 2.0f * obj->wheel_radius_m / obj->track_m;
}

static float DiffTorqueToYawMoment(const TorqueVectoring* obj, float diff_nm) {
  return diff_nm * obj->track_m / (2.0f * obj->wheel_radius_m);
}

// 舵角と車速から「本来出るはずのヨーレート」を作る (実効舵角 + 自転車モデル + 安定係数)。
//
// tan に mymath の Sin/Cos を使わないのは、あちらが1度刻みのテーブル引きで、目標ヨーレートが
// 1度ごとの階段状になるため。刻み幅は 2m/s で 0.15rad/s 程度あり、不感帯 (0.05rad/s) より
// 大きい = 直進付近で出力がカタカタ切り替わることになる。ここは精度が要るので libm を使う
static float TargetYawRate(const TorqueVectoring* obj, float speed_m_s, float steer_rad) {
  const ControlParams* p = obj->params;
  float effective_rad = p->tv_steer_gain * steer_rad + p->tv_steer_gain_cubic * steer_rad * steer_rad * steer_rad;
  float yaw_rate = speed_m_s * tanf(effective_rad) /
                   (obj->wheelbase_m * (1.0f + p->tv_stability_factor * speed_m_s * speed_m_s));

  // 横加速度 a_y = v * r の頭打ち。低速側は分母が 0 に近づくので制限しない
  // (どのみち TV_MIN_SPEED_M_S 以下では介入しない)
  float speed_abs = Abs(speed_m_s);
  if (speed_abs > TV_MIN_SPEED_M_S) {
    float limit = p->tv_max_lateral_accel_m_s2 / speed_abs;
    yaw_rate = Constrain(yaw_rate, -limit, limit);
  }
  return yaw_rate;
}

static float ApplyDeadband(float error_rad_s, float deadband_rad_s) {
  if (error_rad_s > deadband_rad_s) return error_rad_s - deadband_rad_s;
  if (error_rad_s < -deadband_rad_s) return error_rad_s + deadband_rad_s;
  return 0.0f;
}

void TorqueVectoring_Init(TorqueVectoring* obj, const ControlParams* params, float wheelbase_m,
                          float track_m, float wheel_radius_m) {
  obj->params = params;
  obj->wheelbase_m = wheelbase_m;
  obj->track_m = track_m;
  obj->wheel_radius_m = wheel_radius_m;

  obj->enabled = true;
  TorqueVectoring_Reset(obj);
}

float TorqueVectoring_Update(TorqueVectoring* obj, float speed_m_s, float steer_rad,
                             float yaw_rate_rad_s, float dt_s) {
  const ControlParams* p = obj->params;
  float max_moment_nm = p->tv_max_yaw_moment_nm;

  // 目標と偏差は介入しない条件でも残す。実機で「なぜ効かないのか」を追うとき、
  // 出力だけ見ても規範モデルが妥当なのか速度が足りないのかを切り分けられないため
  float ref = TargetYawRate(obj, speed_m_s, steer_rad);
  // リセット直後 (制動を離した直後など、旋回の途中でもあり得る) は遅れの状態を今の値から始める。
  // 0 から始めると、遅れが追いつくまで「回り過ぎ」と誤って判定する
  if (obj->ref_valid && p->tv_ref_lag_s > 0.0f && dt_s > 0.0f) {
    obj->ref_lagged_rad_s += (ref - obj->ref_lagged_rad_s) * (dt_s / (p->tv_ref_lag_s + dt_s));
  } else {
    obj->ref_lagged_rad_s = ref;
  }
  obj->ref_valid = true;
  obj->target_yaw_rate_rad_s = obj->ref_lagged_rad_s;
  obj->yaw_rate_error_rad_s = obj->target_yaw_rate_rad_s - yaw_rate_rad_s;

  if (!obj->enabled || Abs(speed_m_s) < TV_MIN_SPEED_M_S) {
    obj->integral_nm = 0.0f;
    obj->yaw_moment_nm = 0.0f;
    obj->diff_torque_nm = 0.0f;
    return 0.0f;
  }

  // 同定用: PI を止めて指定のヨーモーメントだけを出す (ヨーモーメント→ヨーレートの応答を測る。
  // PI を動かしたままだと、入れたモーメントを PI が打ち消しにいく)
  if (p->tv_test_moment_nm != 0.0f) {
    obj->integral_nm = 0.0f;
    obj->yaw_moment_nm = Constrain(p->tv_test_moment_nm, -max_moment_nm, max_moment_nm);
    obj->diff_torque_nm = YawMomentToDiffTorque(obj, obj->yaw_moment_nm);
    return obj->diff_torque_nm;
  }

  float error = ApplyDeadband(obj->yaw_rate_error_rad_s, p->tv_deadband_rad_s);
  // 不感帯の中では偏差が0に潰れるため積分がそのまま居座る。旋回で溜めた分が直進に戻っても
  // 抜けず、一定のトルク差を出し続けることになるので、この間は積分を0へ漏らす
  float integral_step_nm = error != 0.0f ? p->tv_ki_nm_per_rad * error * dt_s
                                         : -obj->integral_nm * (dt_s / TV_INTEGRAL_LEAK_TT_S);
  obj->integral_nm = Constrain(obj->integral_nm + integral_step_nm, -max_moment_nm, max_moment_nm);
  obj->yaw_moment_nm = Constrain(p->tv_kp_nm_per_rad_s * error + obj->integral_nm, -max_moment_nm, max_moment_nm);

  obj->diff_torque_nm = YawMomentToDiffTorque(obj, obj->yaw_moment_nm);
  return obj->diff_torque_nm;
}

void TorqueVectoring_ReportApplied(TorqueVectoring* obj, float achieved_diff_nm,
                                   float applied_diff_nm, float dt_s) {
  obj->diff_torque_nm = applied_diff_nm;
  if (dt_s <= 0.0f) return;

  float max_moment_nm = obj->params->tv_max_yaw_moment_nm;
  float achieved_moment_nm = DiffTorqueToYawMoment(obj, achieved_diff_nm);
  obj->integral_nm =
      Constrain(obj->integral_nm + (achieved_moment_nm - obj->yaw_moment_nm) *
                                       (dt_s / TV_ANTIWINDUP_TT_S),
                -max_moment_nm, max_moment_nm);
}

void TorqueVectoring_Reset(TorqueVectoring* obj) {
  obj->integral_nm = 0.0f;
  obj->ref_lagged_rad_s = 0.0f;
  obj->ref_valid = false;
  obj->target_yaw_rate_rad_s = 0.0f;
  obj->yaw_rate_error_rad_s = 0.0f;
  obj->yaw_moment_nm = 0.0f;
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

float TorqueVectoring_GetTargetYawRate(const TorqueVectoring* obj) {
  return obj->target_yaw_rate_rad_s;
}

float TorqueVectoring_GetDiffTorque(const TorqueVectoring* obj) {
  return obj->diff_torque_nm;
}
