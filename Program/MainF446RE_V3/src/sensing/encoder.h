#ifndef ENCODER_H_
#define ENCODER_H_

#include "adc_dma.h"
#include "encoder_cal.h"
#include "timer.h"

// ADC2 の Rank 順 (Core/Src/adc.c の MX_ADC2_Init(): Rank1 = IN5 (PA5)、Rank2 = IN4 (PA4))。
// **実機では PA5 に右前輪、PA4 に左前輪のエンコーダが繋がっている** (右の車輪を回すと index 0 が
// 動くことを確認済み)。CubeMX のピンラベル (PA5 = ENCODER_LEFT、PA4 = ENCODER_RIGHT。main.h の
// ENCODER_LEFT_Pin / ENCODER_RIGHT_Pin) は実配線と左右が逆で、正しいのはこの並びの方。
// ラベルはピンをアナログ入力にするのに使われているだけなので、読みには影響しない
typedef enum {
  ENCODER_ADC2_CH_RIGHT = 0,  // Rank1 = IN5 (PA5)
  ENCODER_ADC2_CH_LEFT,       // Rank2 = IN4 (PA4)
  ENCODER_ADC2_NUM_CH,
} EncoderAdc2Channel;

// ADC2 の DMA 循環バッファに溜めるスキャン数。毎周期これを全部平均して角度の ADC ノイズを
// √N 倍に下げる。1スキャン (2ch × (144+12)cycle @22.5MHz) ≈ 13.9us なので 32 スキャンで約 444us。
// バッファが1制御周期 (500us) より長いと、前周期と同じサンプルを平均に含めてしまい
// 実質的な移動平均の窓が延びて遅れが増えるので、500us に収まる範囲で選ぶこと
#define ENCODER_ADC2_NUM_SAMPLES 32

typedef struct {
  AdcDma* adc2;
  // 校正値 (lib/encoder_cal。既定は恒等)。角度・角速度・回転量・累積回転角はすべて校正後の値
  EncoderCal cal_left;
  EncoderCal cal_right;
  // 直近に読んだ ADC のフルスケールに対する比 (0〜1。校正前の生の読み)
  float raw_left;
  float raw_right;
  // 前回、角度を読んだ瞬間の DWT サイクルカウンタ。角速度・回転量の dt は「角度を読む瞬間どうし」の差で求める
  // (Timer を関数の最後でリセットすると、関数内の処理時間 (約9us) が dt から抜け、車速が約1.8%速く出た)
  uint32_t last_cycles;
  float angle_left_rad;
  float angle_right_rad;
  float angular_velocity_left_rad_s;
  float angular_velocity_right_rad_s;

  // 起動時を0とする累積回転角 [1e-3 rad]。上位へ送るオドメトリの元になる。
  // float で累積すると仮数24bitのため数百m走った時点で 0.1mm の分解能を保てなくなるので、
  // 整数で持ち、1e-3 rad に満たない端数だけを float で繰り越す。
  // int32 の範囲は約 2.1e6 rad = 車輪半径30mmなら約64km で、実用上ラップしない。
  int32_t accum_angle_left_mrad;
  int32_t accum_angle_right_mrad;
  float accum_carry_left_mrad;
  float accum_carry_right_mrad;

  // 直近の Encoder_Update での回転量 [rad] と、その間の経過時間。車速のカルマンフィルタが走行距離の
  // 測定値として使う。角速度と違い、長く空いた周期でも (半周以内なら) 正しい値が入っている。
  // update_seq は成功した更新ごとに進むので、利用側は前回と違うときだけ取り込む
  float last_gap_left_rad;
  float last_gap_right_rad;
  float last_dt_s;
  uint32_t update_seq;
} Encoder;

/**
 * @brief エンコーダ読み取りモジュールを初期化する。ADC2 (ENCODER_LEFT/RIGHT) を DMA で読む AdcDma を渡す
 * (num_samples = ENCODER_ADC2_NUM_SAMPLES で初期化しておくこと)。循環バッファが一巡するのを待つため 1〜2ms ブロックする。
 * アナログ出力エンコーダは +3V3 をフルスケールとして 1回転 (0-2*PI rad) で電圧が線形に一周する前提。
 */
void Encoder_Init(Encoder* obj, AdcDma* adc2);

/**
 * @brief 校正値を入れる (左右)。値が変わったときは、今の生の読みを新しい校正値で読み直したものを
 * 前回の角度にする (校正値を入れ替えた瞬間に、回転量・累積回転角へ偽の段差が入らない)。毎周期呼んでよい。
 */
void Encoder_SetCalibration(Encoder* obj, const EncoderCal* left, const EncoderCal* right);

/**
 * @brief ホイール角度・角速度を再計算する。制御周期ごとに呼ぶこと (前回呼び出しからの経過時間で角速度を微分計算する)。
 */
void Encoder_Update(Encoder* obj);

/**
 * @brief 校正前の生の角度 [rad] (0-2*PI。ADC の読み ÷ フルスケール × 2π) を取得する。上位の校正 (周期誤差・
 * 出力の範囲の同定) 用。校正値に依らない
 */
float Encoder_GetRawAngleLeft(const Encoder* obj);
float Encoder_GetRawAngleRight(const Encoder* obj);

/**
 * @brief 左ホイールの角度 [rad] (0-2*PI。校正後) を取得する。
 */
float Encoder_GetAngleLeft(Encoder* obj);

/**
 * @brief 右ホイールの角度 [rad] (0-2*PI) を取得する。
 */
float Encoder_GetAngleRight(Encoder* obj);

/**
 * @brief 左ホイールの角速度 [rad/s] を取得する (0/2*PI の境界をまたぐ回転も正しい符号で計算される)。
 */
float Encoder_GetAngularVelocityLeft(Encoder* obj);

/**
 * @brief 右ホイールの角速度 [rad/s] を取得する。
 */
float Encoder_GetAngularVelocityRight(Encoder* obj);

/**
 * @brief 直近の Encoder_Update での左右の回転量 [rad]・その間の経過時間 [s]・更新番号を取得する。
 * 取付向きの符号のまま (角速度と同じ)。
 */
float Encoder_GetLastGapLeft(const Encoder* obj);
float Encoder_GetLastGapRight(const Encoder* obj);
float Encoder_GetLastDt(const Encoder* obj);
uint32_t Encoder_GetUpdateSeq(const Encoder* obj);

/**
 * @brief 起動時を0とする左ホイールの累積回転角 [1e-3 rad] を取得する (逆転で減る)。
 * 走行距離 = 累積回転角 × 車輪半径。速度を上位側で積分するのと違い、通信が途切れても
 * その間の移動量が失われない。
 */
int32_t Encoder_GetAccumAngleLeftMrad(const Encoder* obj);

/**
 * @brief 起動時を0とする右ホイールの累積回転角 [1e-3 rad] を取得する。
 */
int32_t Encoder_GetAccumAngleRightMrad(const Encoder* obj);

#endif  // ENCODER_H_
