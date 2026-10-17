#ifndef SPEED_KF_H_
#define SPEED_KF_H_

#include <stdbool.h>
#include <stdint.h>

// ===========================================================================
// 車速のカルマンフィルタ (前輪の走行距離 + 加速度計)
//
// 前輪エンコーダを微分して LPF を掛ける従来の車速は、ADC の量子化ノイズ (微分すると 1LSB で
// 0.09m/s) を落とす時定数と遅れが両立しない (τ≈10ms で遅れ 10ms、τ≈5ms でノイズが倍)。
// ここでは前輪の「走行距離」を測定値として直接使い、加速度計で動きを先読みする。
//
//   状態 x = [s 走行距離, v 車速, a 加速度, b 加速度計のバイアス]
//   予測: 加速度が jerk の白色雑音で変わる等加速度モデル。バイアスはランダムウォーク
//   測定1 (毎周期): 走行距離  z = s            (前輪の角度増分 × 半径 × cos(舵角) の累積)
//   測定2 (IMU サンプルごと): 加速度 z = a + b   (機体 X 軸の生の加速度。重力・取付ずれはバイアスが吸収)
//
// ★実機の手押し (加速度 ±6m/s²) の実測は **旧調整値 (σs=0.03mm・σa=0.3) のとき**: 従来のLPF (τ10ms) に対し動作時の誤差
// 19→4.7mm/s、遅れ 10→1ms、静止ノイズ 2.5→1.0mm/s。**今の調整値 (σs=0.8mm・σa=0.6) での実機の値は未計測**で、
// シム (真値あり・周期誤差 0〜3%) の 2〜5mm/s があるだけ。加速度計の遅れ (DLPF 21Hz 相当の +5ms) にはほぼ影響されない
// (位置が主で、加速度は補助)。駆動電源ON時・走行時のノイズは未確認。
//
// ★調整値は検定できる: フィルタは測定ごとに残差 ÷ 自分が主張する標準偏差 z を作り、NIS (=平均 z²) が 1・隣り合う z の
// 相関が 0 なら前提が現実と合っている。TELEMETRY v0.26 で送り、`raspi/tools/kf_consistency.py` が記録から検定する。
// ★前輪エンコーダの周期誤差の実測 (2026-10-17、走行ログ 238秒。front_angle の絶対角に位置の高域成分を回帰): 位置にして
// 左 1次0.10・2次0.51mm、右 1次0.70・2次0.78mm (山-谷 1.2mm / 2.4mm = ±1.1° / ±2.3°)。速度・時間(3分割)によらず
// 振幅も位相もほぼ一定 (位相のずれ ±5°) = 角度に固定された再現する誤差。右の ±2.3° はデータシートの INL ±1° (+DAC ±0.44°) を
// 超える (磁石のずれ・エアギャップの疑い)。速度の揺れにすると左 3.8%・右 7.5% で、これまでの想定 (2%) より大きい。
// 走行中の NIS は走行距離 0.4〜0.6・ρ≈0.96 (白色でない)、加速度 0.6→4.3 (速度とともに増える。振動)。
// 静止の実測 (2026-10-17): 走行距離の NIS≈0.001 (=(実際の雑音 0.022mm ÷ 仮定 0.8mm)²)、加速度の NIS≈0.001。
// つまり σs=0.8mm は白色雑音としては 36 倍の過大で、**周期誤差を白色雑音に換算して通さないための水増し**であり、
// 理論から出した値ではない (周期誤差は角度の関数で、白色ではない)。根本の対処は周期誤差の補正 (front_angle で同定できる)
//
// 外れ値 (前輪のロック・スリップ、段差の衝撃) は、予測との差が大きいほど測定ノイズを膨らませて
// 効きを弱める (SPEED_KF_GATE_SIGMA)。弱めた回数は gated_count が数える。
// ===========================================================================

// 調整値。ホストのテストが -D で上書きする。
//
// 走行距離の測定ノイズ SIGMA_S は、**静止実測の 0.022mm (白色の量子化ノイズ) ではなく 0.8mm にする** (経験値。上記)。
// 実機の走行記録 (2026-10-16、強い加速) で、前輪の絶対角エンコーダの位置誤差 (1回転に2周期の周期誤差
// = 位置にして 0.2mm 前後、特定の角度の段差) が、静止実測よりはるかに大きいと分かったため。
// 0.03mm のままだと周期誤差を「正しい測定」として速度の揺れに通し (周期誤差 2〜3% で LPF と同等まで
// 劣化、ゲートが走行中に毎秒 60〜110 回作動)、σs を上げると加速度計が高周波を担当して揺れを抑える。
// シム (真値あり) の掃引: 周期誤差 0 / 2 / 3% で車速誤差の平均 LPF 15.6 / 20.8 / 24.7mm/s に対し、
// σs=0.8mm・σa=0.6 は 2.0 / 4.8 / 5.1mm/s、σs=0.03mm は 1.3 / 16.6 / 24.1mm/s。
// SIGMA_A は加速度計の雑音に対する頑健さのため 0.6 (走行中の実測は残差 0.6m/s² 前後。雑音 0.5 で
// 6.3、雑音 1.0＋倍率 -7% で 10.0mm/s。雑音が 4m/s² 級になると LPF より悪化するが、加速度計自体が使えない域)
// 前輪の絶対角エンコーダ AS5600 (アナログ出力、データシート v1-00 2014-08-11) との関係:
//   分解能 12bit (0.088° = 車輪半径 30mm で 0.046mm)、システム INL ±1° (**PWM/I²C 出力での値**。直線近似からの偏差、
//   磁石のずれ無し) = 位置にして最大 ±0.52mm。**アナログ出力ではこれに DAC の INL ±5LSB (±0.44° = ±0.23mm)・DNL ±1LSB が
//   加わる**ほか MCU 側の ADC の誤差も乗る。出力雑音 0.015° (遅いフィルタ SF=00・整定 2.2ms) 〜 0.043° (速い SF=11・整定 0.286ms)
//   = 位置にして 0.008〜0.022mm (静止実測 0.022mm と同程度)。実測の周期誤差 (位置で約 0.2mm = 0.4°) は INL の範囲内で、
//   SIGMA_S=0.8mm (約 1.5°) は INL の最大値 (1° + DAC 0.44° ≒ 1.5°) を覆う値
//   (磁石の取り付けがずれると INL は増える。静止雑音の値ではなく INL で決まる)
// MPU6050 のデータシート (PS-MPU-6000A-00 Rev3.2 §6.2、RM-MPU-6000A-00 Rev4.0 DLPF 表) との関係:
//   加速度の雑音密度 400µg/√Hz (@10Hz) = 3.9e-3 m/s²/√Hz → DLPF 94Hz で約 0.04〜0.05 m/s² rms (実測の静止 0.02〜0.03、
//   駆動電源ON 0.16)。SIGMA_A=0.6 はセンサの雑音の約 10 倍で、**センサではなく走行中の振動・車体のピッチの揺れで決まる**
//   (データシートからは決まらない。畳・平らな床の実走で残差 0.46〜0.52)。
//   ゼロg 初期誤差 ±50mg (x,y)=±0.49m/s²・温度 ±35mg/0〜70℃、感度 ±3%・クロス軸 ±2% は、SIGMA_A に比べ十分小さい
//   (感度 ±3% は 3m/s² で 0.09m/s²)。温度ドリフトは遅い (約 0.005m/s²/℃) ので、バイアスの変化の強さ
//   BIAS_WALK_PSD は温度ではなく坂・傾き (1° = 0.17m/s²) で決まる。DLPF 94Hz の加速度の遅れは 3.0ms (21Hz は 8.5ms)
#ifndef SPEED_KF_JERK_PSD
#define SPEED_KF_JERK_PSD 100.0f      // 加速度の変化の強さ [m²/s⁵]。大きいほど速く追う (雑音も増える)
#endif
#ifndef SPEED_KF_SIGMA_S_M
#define SPEED_KF_SIGMA_S_M 0.0008f    // 走行距離の測定ノイズ [m]。上の説明のとおり静止実測より大きくする
#endif
#ifndef SPEED_KF_SIGMA_A_M_S2
#define SPEED_KF_SIGMA_A_M_S2 0.6f    // 加速度の測定ノイズ [m/s²]。静止実測 0.02 (駆動電源ON時 0.16〜)、走行中の残差 0.6 前後
#endif
#ifndef SPEED_KF_BIAS_WALK_PSD
#define SPEED_KF_BIAS_WALK_PSD 0.05f  // バイアスの変化の強さ [(m/s²)²/s]。坂・傾きの変化を追う
#endif
#ifndef SPEED_KF_LEVER_ARM_M
// 旋回で加速度計の前後軸に乗る系統誤差の補正: 測定 a_x に + k·ω² を足してから使う (ω はジャイロの z)。
// 実機の走行ログ (2026-10-16 23:11、238秒) で ax − dv/dt (dv/dt は KF の車速の微分) を ω² に回帰: 係数 -0.153 (R²=0.46)、
// 速度域別 -0.155 / -0.147 / -0.139 と一定。IMU が車体の回転中心より前に付いている分 (-x·ω²) と、
// 横滑り速度との積 (-vy·ω、運動学的には -lr·ω²) の和。ホイールベース 0.23m に対し妥当な大きさ。
// 典型のヨーレートで -0.4m/s²、最大 (ω²≈20) で -3m/s²。補正しないと加速度バイアスの状態 (変化の強さ BIAS_WALK_PSD) では追えず、
// 加速度の NIS が 2.5 (過小見積もり) になっていた。0 にすると補正しない
#define SPEED_KF_LEVER_ARM_M 0.15f
#endif
#ifndef SPEED_KF_GATE_SIGMA
#define SPEED_KF_GATE_SIGMA 6.0f      // 予測との差がこれ以上 (σ) なら測定ノイズを膨らませる
#endif
#define SPEED_KF_REBASE_M 0.25f       // 走行距離がこれを超えたら 0 へ引き戻す (float の桁落ち対策)
#define SPEED_KF_MAX_DT_S 1.0f        // これ以上空いたら速度を信用せず初期化し直す (起動直後の長い Setup の後など)。
                                      // それ未満の停止は、予測を長い dt で進めて (P が Q·dt ぶん広がる) 次の測定で引き戻す

// 一貫性の診断 (NIS と自己相関)。測定ごとの「正規化した残差」 z = (z_meas − h x) / sqrt(S) を集計する。
// フィルタの前提 (SIGMA_*・JERK_PSD・BIAS_WALK_PSD) が合っていれば z は平均 0・分散 1 の白色雑音になるので、
//   NIS = mean(z²)         → 1 より大きければ前提より誤差が大きい (R・Q が小さすぎる)、小さければ大きすぎる
//   ac  = mean(z_k z_{k-1}) → 0 でなければ残差が時間的に相関している (周期誤差・モデルの不足。白色雑音の仮定の破れ)
// 制御には使わない。TELEMETRY で上位へ出し、記録から調整値を検証する (raspi/tools/kf_consistency.py)。
// 外れ値 1 個で平均が埋もれないよう、z は ±SPEED_KF_DIAG_Z_CLIP で切る (ゲートに掛かる 6σ より大きい)。
#define SPEED_KF_DIAG_Z_CLIP 10.0f

typedef struct {
  uint32_t n;       // 取り込んだ残差の数
  uint32_t n_ac;    // 自己相関の積を取れた数 (直前の残差があった回数)
  float nis_sum;    // Σ z²
  float ac_sum;     // Σ z_k z_{k-1}
  float z_prev;
  bool has_prev;
} SpeedKfDiag;

typedef enum {
  SPEED_KF_DIAG_DISTANCE = 0,  // 走行距離の測定 (Encoder の更新ごと = 2kHz)
  SPEED_KF_DIAG_ACCEL = 1,     // 加速度の測定 (IMU のサンプルごと = 200Hz)
  SPEED_KF_DIAG_COUNT
} SpeedKfDiagChannel;

typedef struct {
  // 状態: 0=s (走行距離), 1=v, 2=a, 3=b。s を先頭に置くのは測定行列を単純にするため
  float x[4];
  float p[4][4];
  float s_meas;  // 測定側の走行距離の累積 (s と同じ基準で引き戻す)
  bool initialized;
  bool resync;           // 長い停止で初期化した直後。最初の走行距離の増分は、止まっていた間の回転なので捨てる
  uint32_t gated_count;  // 測定ノイズを膨らませた回数 (走行距離 + 加速度。オーバーフローは無視)
  SpeedKfDiag diag[SPEED_KF_DIAG_COUNT];
} SpeedKf;

void SpeedKf_Init(SpeedKf* kf);

/** @brief 状態を dt [s] 進める。毎周期、測定の更新より先に呼ぶ。 */
void SpeedKf_Predict(SpeedKf* kf, float dt_s);

/** @brief 前回からの走行距離の増分 [m] (前進が正) で更新する。 */
void SpeedKf_UpdateDistance(SpeedKf* kf, float delta_m);

/**
 * @brief 予測が見込む、前回の測定から今回までの走行距離の増分 [m] (Predict の直後に呼ぶ)。
 *
 * 絶対角エンコーダの角度差は ±半周に畳んで求める (Encoder_Update) ので、制御ループが止まって半周を超えて回ると
 * 折り返した値 (本来の増分から 1 回転ぶん外れた値) が来る (前輪の周速 3m/s で 31ms 以上の停止)。
 * 呼び出し側が各輪の増分へ 1 回転の整数倍を足して、この値に最も近い側を選ぶ (drive.c)。
 */
static inline float SpeedKf_ExpectedDistanceStep(const SpeedKf* kf) { return kf->x[0] - kf->s_meas; }

/** @brief 機体 X 軸の生の加速度 [m/s²] (前進加速が正・重力込み) で更新する。 */
void SpeedKf_UpdateAccel(SpeedKf* kf, float accel_x_m_s2);

/**
 * @brief 前回この関数を呼んでから溜めた残差の NIS (平均 z²) と自己相関 (平均 z_k z_{k-1}) を返し、集計をやり直す。
 * 溜まっていなければ (0, 0) を返す。TELEMETRY の1周期ぶん (10ms) ごとに呼ぶ想定。
 */
void SpeedKf_TakeDiag(SpeedKf* kf, SpeedKfDiagChannel channel, float* nis, float* autocorr);

static inline float SpeedKf_GetSpeed(const SpeedKf* kf) { return kf->x[1]; }
static inline float SpeedKf_GetAccel(const SpeedKf* kf) { return kf->x[2]; }
static inline float SpeedKf_GetAccelBias(const SpeedKf* kf) { return kf->x[3]; }

#endif  // SPEED_KF_H_
