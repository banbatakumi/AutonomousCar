#ifndef CONTROL_PARAMS_H_
#define CONTROL_PARAMS_H_

#include <stdbool.h>
#include <stdint.h>

// ===========================================================================
// 足回りの制御 (TC・ABS・片輪浮き対策・TV・ローンチコントロール) の調整パラメータと、ステアのリンクの換算
//
// 上位 (Raspberry Pi) が CONFIG_SET / CONFIG_GET (param_id は下の表の2列目) で実行時に
// 読み書きする。Flash へは保存しないので、電源を入れ直すと下の既定値へ戻る — 上位は
// 接続のたびに全項目を送り、CONFIG_ACK の applied (実際に適用された値) と突き合わせること
// (Pi 側リポジトリの config/vehicle.toml [control] が唯一の定義で、ここの既定値は
// 「上位が送るまでの仮置き」(各範囲の下限＝何もしない値。ステアのリンクだけ恒等の 1 と 0)。
// 本物の値は持たない: 上位が全項目を送って CONFIG_ACK で確かめるまで ARM を通さない (io_node の _arm_block_reason))。
//
// 値を決める手順は Pi 側の tools/ctrl_tune (実機の記録で同定した車輪+タイヤのモデルと
// このファームの src/control をホストでコンパイルしたものを閉ループにして最適化する)。
//
// 表の列: X(フィールド名, param_id, 仮置き, 下限, 上限)
//
// 校正値 (前輪エンコーダ・車輪の有効半径) と車速のカルマンフィルタの調整値だけは、仮置きが「何もしない値」
// ではなく**今までの固定値** (校正なし・名目の半径・speed_kf.h の既定) にしてある。上位が送る前でも車速と
// 走行距離は今までどおりに出るようにするため (KF の仮置きは speed_kf.h の SPEED_KF_* と同じ値。上位の
// tools/ctrl_tune/tests/test_fw_constants.py が突き合わせる)
// ===========================================================================
#define CONTROL_PARAM_TABLE(X)                                                                  \
  /* --- スリップの測り方 (TC・ABS 共通) --- */                                                 \
  /* スリップ率の分母の下限 [m/s]。スリップ率 = スリップ速度 / max(|基準速度|, これ)。 */       \
  /* これより遅い間は「スリップ速度がこの値×目標スリップ率を超えたか」で見ることになる */       \
  X(slip_speed_floor_m_s, 0x0015, 0.1f, 0.1f, 2.0f)                                             \
  /* --- TC (駆動・車速PIの減速。各輪独立) --- */                                               \
  X(tc_slip_target, 0x0011, 0.02f, 0.02f, 1.0f)  /* 保つスリップ率 */                           \
  X(tc_kp_nm_per_m_s, 0x0012, 0.0f, 0.0f, 1.0f) /* スリップ速度の偏差→トルク上限 */           \
  X(tc_ki_nm_per_m, 0x0013, 0.0f, 0.0f, 50.0f)  /* 同 積分 [Nm/(m/s)/s] */                     \
  /* 上限の下限。0 でよい (滑りが収まれば積分が上限を戻す)。浮いた輪は 0.005Nm でも回り続ける */ \
  X(tc_min_torque_nm, 0x0014, 0.0f, 0.0f, 0.05f)                                                \
  /* --- 片輪浮き対策 (後輪左右の速度差。TC と同じ上限を共有する) --- */                        \
  X(wheel_lift_diff_threshold_m_s, 0x0051, 0.05f, 0.05f, 3.0f)                                  \
  /* --- ABS (制動モード。左右共通 = select-low) --- */                                         \
  X(abs_slip_target, 0x0071, 0.02f, 0.02f, 1.0f)                                                \
  X(abs_kp_nm_per_m_s, 0x0072, 0.0f, 0.0f, 1.0f)                                              \
  X(abs_ki_nm_per_m, 0x0073, 0.0f, 0.0f, 50.0f)                                                 \
  /* --- TV (後輪左右の荷重に比例した配分) --- */              \
  /* 0x0021〜0x0029 は欠番で、送られてきたら UNKNOWN を返す */                 \
  /* 同定用: 0 以外の間は配分を止めてこのヨーモーメント [Nm] だけを出す (左旋回が正) */          \
  X(tv_test_moment_nm, 0x002A, 0.0f, -0.2f, 0.2f)                                               \
  /* 配分の比率 = これ × 横加速度 (車速×ヨーレート) [1/(m/s^2)]。0 で TV なし */                 \
  X(tv_load_gain_s2_per_m, 0x002B, 0.0f, 0.0f, 0.5f)                                          \
  X(tv_max_ratio, 0x002C, 0.0f, 0.0f, 1.0f) /* 比率の上限。1 で内輪のトルクが 0 */                  \
  /* --- 車速PI (後輪2輪の合計トルク。drive.c の speed_pid へ毎周期写す) --- */                    \
  X(speed_kp, 0x0030, 0.0f, 0.0f, 2.0f)  /* 比例ゲイン [Nm/(m/s)] */                              \
  X(speed_ki, 0x0031, 0.0f, 0.0f, 10.0f) /* 積分ゲイン [Nm/(m/s)/s] */                            \
  /* --- ステアのリンク (モータ角 → 路面舵角の換算。src/control/steering.h) --- */       \
  /* 路面舵角 = gain*x + cubic*x^3、x = モータ角 × STEERING_LINKAGE_RATIO [rad]。 */              \
  /* 既定は 1 と 0 (= 換算比が一定)。上位がシステム同定の結果を送る */                           \
  X(steer_link_gain, 0x0081, 1.0f, 0.5f, 1.5f)                                                  \
  X(steer_link_cubic, 0x0082, 0.0f, -1.5f, 1.5f)                                                \
  /* --- ローンチコントロール (drive.h の同名の節) --- */                                 \
  /* 発進を終えて車速PIへ戻すときの積分の初期値 [Nm、1輪あたり] = その車速を保つのに要る */      \
  /* トルク (転がり抵抗ぶん)。0 だと積分が溜まるまで車速が目標より これ×2/speed_kp */     \
  /* [m/s] 落ち込む。大きすぎれば同じだけ行き過ぎる */                                           \
  X(launch_exit_torque_nm, 0x00A1, 0.0f, 0.0f, 0.05f)                                           \
  /* --- 車速PIの加速度フィードフォワード (drive.h の同名の節。後輪2輪の合計トルク) --- */  \
  /* 車速PIの出力に「これ × 目標車速の傾き [m/s^2]」を足す [Nm/(m/s^2)] = 1 / 車体の利得。 */    \
  /* 0 で無し (既定)。上位がシステム同定の結果 (speed_plant_gain) から決めて送る */               \
  X(speed_ff_nm_per_m_s2, 0x00B1, 0.0f, 0.0f, 0.2f)                                             \
  /* 同じく「これ × 進む向き」を足す [Nm] = 転がり抵抗ぶんのトルク (その車速を保つのに要る) */   \
  X(speed_ff_rr_nm, 0x00B2, 0.0f, 0.0f, 0.05f)                                                  \
  /* --- 前輪エンコーダの校正 (lib/encoder_cal。左右別。上位のエンコーダ校正試験が決める) --- */      \
  /* lo・hi: 角度 0・2π に当たる ADC の読み (フルスケールに対する比) */                            \
  X(enc_left_lo, 0x00C1, 0.0f, 0.0f, 0.2f)                                                      \
  X(enc_left_hi, 0x00C2, 1.0f, 0.8f, 1.0f)                                                      \
  /* 1回転の周期誤差 e(θ) = h1s sinθ + h1c cosθ + h2s sin2θ + h2c cos2θ [rad] (θ は取付向きの符号のまま) */ \
  X(enc_left_h1s, 0x00C3, 0.0f, -0.1f, 0.1f)                                                    \
  X(enc_left_h1c, 0x00C4, 0.0f, -0.1f, 0.1f)                                                    \
  X(enc_left_h2s, 0x00C5, 0.0f, -0.1f, 0.1f)                                                    \
  X(enc_left_h2c, 0x00C6, 0.0f, -0.1f, 0.1f)                                                    \
  X(enc_right_lo, 0x00C9, 0.0f, 0.0f, 0.2f)                                                     \
  X(enc_right_hi, 0x00CA, 1.0f, 0.8f, 1.0f)                                                     \
  X(enc_right_h1s, 0x00CB, 0.0f, -0.1f, 0.1f)                                                   \
  X(enc_right_h1c, 0x00CC, 0.0f, -0.1f, 0.1f)                                                   \
  X(enc_right_h2s, 0x00CD, 0.0f, -0.1f, 0.1f)                                                   \
  X(enc_right_h2c, 0x00CE, 0.0f, -0.1f, 0.1f)                                                   \
  /* --- 車速のカルマンフィルタ (speed_kf.h。上位が記録から推定して送る) --- */                     \
  X(kf_sigma_s_m, 0x00E1, 0.0008f, 0.00001f, 0.005f)    /* 走行距離の測定ノイズ [m] */     \
  /* 加速度の測定ノイズ [m/s^2] = kf_sigma_a_m_s2 + kf_sigma_a_per_m_s × |車速| (振動は速度とともに増える) */ \
  X(kf_sigma_a_m_s2, 0x00E2, 0.6f, 0.01f, 5.0f)                                \
  X(kf_sigma_a_per_m_s, 0x00E3, 0.0f, 0.0f, 2.0f)                                               \
  X(kf_jerk_psd, 0x00E4, 100.0f, 1.0f, 10000.0f)        /* [m^2/s^5] */                \
  X(kf_bias_walk_psd, 0x00E5, 0.05f, 0.00001f, 10.0f) /* [(m/s^2)^2/s] */        \
  X(kf_lever_arm_m, 0x00E6, 0.15f, -0.5f, 0.5f)     /* 旋回の -k·ω² の補正 [m] */             \
  /* --- ジャイロ z 軸の感度の補正 (imu.c。上位が vehicle.toml の値を送る。v0.28) --- */              \
  /* バイアスを引いた後の z の角速度に掛ける倍率。既定 1 (データシートの感度のまま) */             \
  X(imu_gyro_z_scale, 0x00F1, 1.0f, 0.9f, 1.1f)

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
