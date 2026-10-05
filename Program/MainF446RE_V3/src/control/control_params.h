#ifndef CONTROL_PARAMS_H_
#define CONTROL_PARAMS_H_

#include <stdbool.h>
#include <stdint.h>

// ===========================================================================
// 足回りの制御 (TC・ABS・片輪浮き対策・TV) の調整パラメータ
//
// 上位 (Raspberry Pi) が CONFIG_SET / CONFIG_GET (param_id は下の表の2列目) で実行時に
// 読み書きする。Flash へは保存しないので、電源を入れ直すと下の既定値へ戻る — 上位は
// 接続のたびに全項目を送り、CONFIG_ACK の applied (実際に適用された値) と突き合わせること
// (Pi 側リポジトリの config/vehicle.toml [control] が唯一の定義で、ここの既定値は
// 「上位が何も送らなくても安全に走れる、最後に確かめた値」)。
//
// 値を決める手順は Pi 側の tools/ctrl_tune (実機の記録で同定した車輪+タイヤのモデルと
// このファームの src/control をホストでコンパイルしたものを閉ループにして最適化する)。
// ★既定値を変えたら Pi 側の vehicle.toml [control] も揃えること★
//
// 表の列: X(フィールド名, param_id, 既定値, 下限, 上限)
// ===========================================================================
#define CONTROL_PARAM_TABLE(X)                                                                  \
  /* --- スリップの測り方 (TC・ABS 共通) --- */                                                 \
  /* スリップ率の分母の下限 [m/s]。スリップ率 = スリップ速度 / max(|基準速度|, これ)。 */       \
  /* これより遅い間は「スリップ速度がこの値×目標スリップ率を超えたか」で見ることになる */       \
  X(slip_speed_floor_m_s, 0x0015, 0.5f, 0.1f, 2.0f)                                             \
  /* --- TC (駆動・車速PIの減速。各輪独立) --- */                                               \
  X(tc_slip_target, 0x0011, 0.10f, 0.02f, 1.0f)  /* 保つスリップ率 */                           \
  X(tc_kp_nm_per_m_s, 0x0012, 0.50f, 0.0f, 1.0f) /* スリップ速度の偏差→トルク上限 */           \
  X(tc_ki_nm_per_m, 0x0013, 15.0f, 0.0f, 50.0f)  /* 同 積分 [Nm/(m/s)/s] */                     \
  /* 上限の下限。0 でよい (滑りが収まれば積分が上限を戻す)。浮いた輪は 0.005Nm でも回り続ける */ \
  X(tc_min_torque_nm, 0x0014, 0.0f, 0.0f, 0.05f)                                                \
  /* --- 片輪浮き対策 (後輪左右の速度差。TC と同じ上限を共有する) --- */                        \
  X(wheel_lift_diff_threshold_m_s, 0x0051, 0.35f, 0.05f, 3.0f)                                  \
  /* --- ABS (制動モード。左右共通 = select-low) --- */                                         \
  X(abs_slip_target, 0x0071, 0.15f, 0.02f, 1.0f)                                                \
  X(abs_kp_nm_per_m_s, 0x0072, 0.14f, 0.0f, 1.0f)                                              \
  X(abs_ki_nm_per_m, 0x0073, 5.0f, 0.0f, 50.0f)                                                 \
  /* --- TV (ヨーレートPI) --- */                                                               \
  X(tv_kp_nm_per_rad_s, 0x0021, 0.08f, 0.0f, 2.0f)                                              \
  X(tv_ki_nm_per_rad, 0x0022, 0.16f, 0.0f, 20.0f)                                               \
  X(tv_deadband_rad_s, 0x0023, 0.05f, 0.0f, 1.0f)                                               \
  X(tv_max_yaw_moment_nm, 0x0024, 0.15f, 0.0f, 0.4f)                                            \
  /* 規範ヨーレートの頭打ち |r| <= これ / |v| [m/s^2] (実測の限界の約0.92倍) */                 \
  X(tv_max_lateral_accel_m_s2, 0x0025, 4.1f, 0.5f, 20.0f)                                       \
  /* 規範の実効舵角 δ_eff = gain*δ + cubic*δ^3 (Pi 側の同定 steer_gain / steer_gain_cubic) */   \
  X(tv_steer_gain, 0x0026, 0.993f, 0.5f, 1.5f)                                                  \
  X(tv_steer_gain_cubic, 0x0027, -0.39f, -3.0f, 3.0f)                                           \
  X(tv_stability_factor, 0x0028, 0.0f, -0.5f, 0.5f) /* [s^2/m^2]。0 = 幾何どおり */             \
  /* 規範ヨーレートに掛ける1次遅れ [s] (車のヨー応答ぶん)。0 = 遅れなし */                      \
  X(tv_ref_lag_s, 0x0029, 0.0f, 0.0f, 0.5f)                                                     \
  /* 同定用: 0 以外の間は PI を止めてこのヨーモーメント [Nm] だけを出す (左旋回が正) */                 \
  X(tv_test_moment_nm, 0x002A, 0.0f, -0.2f, 0.2f)

typedef struct {
#define CONTROL_PARAM_FIELD(name, id, def, lo, hi) float name;
  CONTROL_PARAM_TABLE(CONTROL_PARAM_FIELD)
#undef CONTROL_PARAM_FIELD
} ControlParams;

// CONFIG_ACK.result と同じ値 (RasConfigResult)。comm 層に依存しないようここでも定義する
typedef enum {
  CONTROL_PARAM_OK = 0,
  CONTROL_PARAM_UNKNOWN = 1,
  CONTROL_PARAM_CLAMPED = 2,
} ControlParamResult;

/**
 * @brief 全項目を既定値にする。
 */
void ControlParams_SetDefaults(ControlParams* obj);

/**
 * @brief param_id の項目へ value を入れる。範囲外・非数は範囲内へ丸めて CONTROL_PARAM_CLAMPED を
 * 返す (非数は既定値にする)。applied には実際に入った値が入る。表に無い param_id は何もせず
 * CONTROL_PARAM_UNKNOWN。
 */
ControlParamResult ControlParams_Set(ControlParams* obj, uint16_t param_id, float value, float* applied);

/**
 * @brief param_id の項目の現在値を取得する。表に無ければ偽を返す。
 */
bool ControlParams_Get(const ControlParams* obj, uint16_t param_id, float* value);

#endif  // CONTROL_PARAMS_H_
