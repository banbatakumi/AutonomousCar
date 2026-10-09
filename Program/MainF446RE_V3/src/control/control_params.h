#ifndef CONTROL_PARAMS_H_
#define CONTROL_PARAMS_H_

#include <stdbool.h>
#include <stdint.h>

// ===========================================================================
// 足回りの制御 (TC・ABS・片輪浮き対策・TV) の調整パラメータと、ステアのリンクの換算
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
  /* --- TV (後輪左右の荷重に比例した配分。v0.17 でヨーレートPI から置き換えた。 */              \
  /*     PI の頃の 0x0021〜0x0029 は欠番で、送られてきたら UNKNOWN を返す) --- */                 \
  /* 同定用: 0 以外の間は配分を止めてこのヨーモーメント [Nm] だけを出す (左旋回が正) */          \
  X(tv_test_moment_nm, 0x002A, 0.0f, -0.2f, 0.2f)                                               \
  /* 配分の比率 = これ × 横加速度 (車速×ヨーレート) [1/(m/s^2)]。0 で TV なし */                 \
  X(tv_load_gain_s2_per_m, 0x002B, 0.045f, 0.0f, 0.5f)                                          \
  X(tv_max_ratio, 0x002C, 0.5f, 0.0f, 1.0f) /* 比率の上限。1 で内輪のトルクが 0 */                  \
  /* --- ステアのリンク (v0.18。モータ角 → 路面舵角の換算。src/control/steering.h) --- */       \
  /* 路面舵角 = gain*x + cubic*x^3、x = モータ角 × STEERING_LINKAGE_RATIO [rad]。 */              \
  /* 既定は 1 と 0 (= 換算比が一定)。上位がシステム同定の結果を送る */                           \
  X(steer_link_gain, 0x0081, 1.0f, 0.5f, 1.5f)                                                  \
  X(steer_link_cubic, 0x0082, 0.0f, -1.5f, 1.5f)

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
