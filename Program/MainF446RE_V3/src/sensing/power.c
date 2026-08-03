#include "power.h"

#define ADC_VREF 3.3f  // VDDA 公称値 (未較正)

// 電圧分圧比 R18/(R17+R18) = R20/(R19+R20) = 1k/11k (VOLTAGE_S/VOLTAGE_P とも同一比率)
#define VOLTAGE_DIVIDER_RATIO (1.0f / 11.0f)

// INA180A2 ゲイン (両ch共通、型番A2は50V/V)
#define CURRENT_SENSE_GAIN 50.0f
// シャント抵抗 [Ω] (シグナル系 R3, 駆動系 R28 とも実装は5mΩ。回路図のBOMは R3=10mΩ 表記だが実基板と異なる)
#define CURRENT_SIGNAL_SHUNT_OHM 0.005f
#define CURRENT_DRIVE_SHUNT_OHM 0.005f

// STM32F446 データシート記載の温度センサ標準特性値 (個体較正なし)
#define TEMP_V25 0.76f
#define TEMP_AVG_SLOPE_V_PER_C 0.0025f

void Power_Init(Power *obj, AdcDma *adc1) {
  obj->adc1 = adc1;
}

float Power_GetVoltageSignal(Power *obj) {
  return AdcDma_GetVoltage(obj->adc1, POWER_ADC1_CH_VOLTAGE_S, ADC_VREF) / VOLTAGE_DIVIDER_RATIO;
}

float Power_GetVoltageDrive(Power *obj) {
  return AdcDma_GetVoltage(obj->adc1, POWER_ADC1_CH_VOLTAGE_P, ADC_VREF) / VOLTAGE_DIVIDER_RATIO;
}

float Power_GetCurrentSignal(Power *obj) {
  float v_sense = AdcDma_GetVoltage(obj->adc1, POWER_ADC1_CH_CURRENT_S, ADC_VREF);
  return v_sense / (CURRENT_SIGNAL_SHUNT_OHM * CURRENT_SENSE_GAIN);
}

float Power_GetCurrentDrive(Power *obj) {
  float v_sense = AdcDma_GetVoltage(obj->adc1, POWER_ADC1_CH_CURRENT_P, ADC_VREF);
  return v_sense / (CURRENT_DRIVE_SHUNT_OHM * CURRENT_SENSE_GAIN);
}

float Power_GetTemperatureC(Power *obj) {
  float v_sense = AdcDma_GetVoltage(obj->adc1, POWER_ADC1_CH_TEMP, ADC_VREF);
  return (v_sense - TEMP_V25) / TEMP_AVG_SLOPE_V_PER_C + 25.0f;
}
