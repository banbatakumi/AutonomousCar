#ifndef ENCODER_H_
#define ENCODER_H_

#include "adc_dma.h"
#include "timer.h"

// ADC2 の Rank 順 (Core/Src/adc.c の MX_ADC2_Init() のチャンネル設定順と一致させること)
typedef enum {
  ENCODER_ADC2_CH_LEFT = 0,
  ENCODER_ADC2_CH_RIGHT,
  ENCODER_ADC2_NUM_CH,
} EncoderAdc2Channel;

typedef struct {
  AdcDma *adc2;
  Timer timer;
  float angle_left_rad;
  float angle_right_rad;
  float angular_velocity_left_rad_s;
  float angular_velocity_right_rad_s;
} Encoder;

/**
 * @brief エンコーダ読み取りモジュールを初期化する。ADC2 (ENCODER_LEFT/RIGHT) を DMA で読む AdcDma を渡す。
 * アナログ出力エンコーダは +3V3 をフルスケールとして 1回転 (0-2*PI rad) で電圧が線形に一周する前提。
 */
void Encoder_Init(Encoder *obj, AdcDma *adc2);

/**
 * @brief ホイール角度・角速度を再計算する。制御周期ごとに呼ぶこと (前回呼び出しからの経過時間で角速度を微分計算する)。
 */
void Encoder_Update(Encoder *obj);

/**
 * @brief 左ホイールの角度 [rad] (0-2*PI) を取得する。
 */
float Encoder_GetAngleLeft(Encoder *obj);

/**
 * @brief 右ホイールの角度 [rad] (0-2*PI) を取得する。
 */
float Encoder_GetAngleRight(Encoder *obj);

/**
 * @brief 左ホイールの角速度 [rad/s] を取得する (0/2*PI の境界をまたぐ回転も正しい符号で計算される)。
 */
float Encoder_GetAngularVelocityLeft(Encoder *obj);

/**
 * @brief 右ホイールの角速度 [rad/s] を取得する。
 */
float Encoder_GetAngularVelocityRight(Encoder *obj);

#endif  // ENCODER_H_
