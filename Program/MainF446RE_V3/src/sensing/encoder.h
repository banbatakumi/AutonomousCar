#ifndef ENCODER_H_
#define ENCODER_H_

#include "adc_dma.h"
#include "timer.h"

// ADC2 の Rank 順 (Core/Src/adc.c の MX_ADC2_Init() のチャンネル設定順と一致させること)
typedef enum {
  ENCODER_ADC2_CH_RIGHT = 0,
  ENCODER_ADC2_CH_LEFT,
  ENCODER_ADC2_NUM_CH,
} EncoderAdc2Channel;

typedef struct {
  AdcDma* adc2;
  Timer timer;
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
} Encoder;

/**
 * @brief エンコーダ読み取りモジュールを初期化する。ADC2 (ENCODER_LEFT/RIGHT) を DMA で読む AdcDma を渡す。
 * アナログ出力エンコーダは +3V3 をフルスケールとして 1回転 (0-2*PI rad) で電圧が線形に一周する前提。
 */
void Encoder_Init(Encoder* obj, AdcDma* adc2);

/**
 * @brief ホイール角度・角速度を再計算する。制御周期ごとに呼ぶこと (前回呼び出しからの経過時間で角速度を微分計算する)。
 */
void Encoder_Update(Encoder* obj);

/**
 * @brief 左ホイールの角度 [rad] (0-2*PI) を取得する。
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
