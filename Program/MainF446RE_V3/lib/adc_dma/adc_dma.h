#ifndef ADC_DMA_H_
#define ADC_DMA_H_

#include "main.h"

// ADC を DMA (Circular + ContinuousConvMode) で連続変換させ、最新値をノンブロッキングで
// 読み出すための薄いラッパ。MX_ADCx_Init() 側で ScanConvMode/ContinuousConvMode/
// DMAContinuousRequests が有効になっていること、NbrOfConversion 分のチャンネルが
// Rank 順に設定されていることが前提。
typedef struct {
  uint16_t *buffer;
  uint32_t num_ranks;
  uint32_t num_samples;
} AdcDma;

// buffer は呼び出し側で num_ranks * num_samples 分確保しておくこと。
// DMA は num_samples 回分のスキャンを循環バッファに書き続けるので、平均化したい場合は
// num_samples を 2 以上にして AdcDma_GetRawAt() で全サンプルを読む。
// buffer[sample * num_ranks + index] は Rank (index+1) の変換値に対応する。
static inline void AdcDma_Init(AdcDma *obj, ADC_HandleTypeDef *hadc, uint16_t *buffer, uint32_t num_ranks,
                               uint32_t num_samples) {
  obj->buffer = buffer;
  obj->num_ranks = num_ranks;
  obj->num_samples = num_samples;
  HAL_ADC_Start_DMA(hadc, (uint32_t *)buffer, num_ranks * num_samples);
}

// Rank (index+1) の循環バッファ内全サンプルの平均値を取得する (12bit, 0-4095)。DMA が裏で継続更新するため待ち時間なし。
// 0/4095 の境界を跨ぐと意味を持つ信号 (角度など) には使わないこと (境界を跨いだ窓で半周ずれた値になる)。
static inline float AdcDma_GetAverageRaw(AdcDma *obj, uint32_t index) {
  uint32_t sum = 0;
  for (uint32_t i = 0; i < obj->num_samples; i++) sum += obj->buffer[i * obj->num_ranks + index];
  return (float)sum / (float)obj->num_samples;
}

// 循環バッファ中の sample 番目 (0 〜 num_samples-1) のスキャンの Rank (index+1) の変換値を取得する。
// どのサンプルが最新かは DMA の書き込み位置次第なので、順序に意味を持たせないこと。
static inline uint16_t AdcDma_GetRawAt(AdcDma *obj, uint32_t sample, uint32_t index) {
  return obj->buffer[sample * obj->num_ranks + index];
}

// AdcDma_GetAverageRaw() の値を vref [V] (通常 VDDA ≒ 3.3V) を基準に電圧へ変換する
static inline float AdcDma_GetVoltage(AdcDma *obj, uint32_t index, float vref) {
  return AdcDma_GetAverageRaw(obj, index) * vref / 4095.0f;
}

#endif  // ADC_DMA_H_
