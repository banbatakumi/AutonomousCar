#ifndef POWER_H_
#define POWER_H_

#include "adc_dma.h"

// ADC1 の Rank 順 (Core/Src/adc.c の MX_ADC1_Init() のチャンネル設定順と一致させること)
typedef enum {
  POWER_ADC1_CH_VOLTAGE_S = 0,  // シグナル系(ロジック電源)電圧
  POWER_ADC1_CH_VOLTAGE_P,      // 駆動系(モーター電源)電圧
  POWER_ADC1_CH_CURRENT_S,      // シグナル系電流
  POWER_ADC1_CH_CURRENT_P,      // 駆動系電流
  POWER_ADC1_CH_TEMP,           // マイコン内蔵温度センサ
  POWER_ADC1_NUM_CH,
} PowerAdc1Channel;

typedef struct {
  AdcDma *adc1;
} Power;

/**
 * @brief 電源監視モジュールを初期化する。ADC1 (VOLTAGE_S/P, CURRENT_S/P, TEMP) を DMA で読む AdcDma を渡す。
 */
void Power_Init(Power *obj, AdcDma *adc1);

/**
 * @brief シグナル系(ロジック電源)の入力電圧 [V] を取得する (R17=10k/R18=1k 分圧、MainBoard_V3_2 回路図実測)。
 */
float Power_GetVoltageSignal(Power *obj);

/**
 * @brief 駆動系(モーター電源)の入力電圧 [V] を取得する (R19=10k/R20=1k 分圧、MainBoard_V3_2 回路図実測)。
 */
float Power_GetVoltageDrive(Power *obj);

/**
 * @brief シグナル系の消費電流 [A] を取得する (INA180A2 ゲイン50V/V, シャント R3=5mΩ)。
 */
float Power_GetCurrentSignal(Power *obj);

/**
 * @brief 駆動系の消費電流 [A] を取得する (INA180A2 ゲイン50V/V, シャント R28=5mΩ)。
 */
float Power_GetCurrentDrive(Power *obj);

/**
 * @brief マイコン内蔵温度センサの温度 [degC] を取得する (工場較正なし、データシート標準値による概算)。
 */
float Power_GetTemperatureC(Power *obj);

#endif  // POWER_H_
