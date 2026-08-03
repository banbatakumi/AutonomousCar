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
} AdcDma;

// buffer は呼び出し側で num_ranks 分確保しておくこと。
// buffer[index] は Rank (index+1) の変換値に対応する。
static inline void AdcDma_Init(AdcDma *obj, ADC_HandleTypeDef *hadc, uint16_t *buffer, uint32_t num_ranks) {
  obj->buffer = buffer;
  obj->num_ranks = num_ranks;
  HAL_ADC_Start_DMA(hadc, (uint32_t *)buffer, num_ranks);
}

// Rank (index+1) の最新変換値を取得する (12bit, 0-4095)。DMA が裏で継続更新するため待ち時間なし。
static inline uint16_t AdcDma_GetRaw(AdcDma *obj, uint32_t index) {
  return obj->buffer[index];
}

// vref [V] (通常 VDDA ≒ 3.3V) を基準に電圧へ変換する
static inline float AdcDma_GetVoltage(AdcDma *obj, uint32_t index, float vref) {
  return (float)obj->buffer[index] * vref / 4095.0f;
}

#endif  // ADC_DMA_H_
