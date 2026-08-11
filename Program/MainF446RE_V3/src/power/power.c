#include "power.h"

#include <stdio.h>

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

// 生の判定値が duration_s の間continuousに続いたときだけ結果に反映する。突入電流や負荷時の
// 一時的な電圧サグ、単発のADCノイズで遮断・警告してしまうと走行不能になるため、瞬時値では
// 判定しない。検出側だけでなく復帰側にも継続時間を要求することで、閾値近傍でのばたつきも防ぐ。
static int UpdateDetector(PowerDetector *det, int abnormal, float duration_s) {
  if (abnormal == det->state) {
    det->pending = det->state;
    return det->state;
  }
  if (abnormal != det->pending) {
    det->pending = abnormal;
    Timer_Reset(&det->timer);
    return det->state;
  }
  if (Timer_Read(&det->timer) >= duration_s) {
    det->state = abnormal;
  }
  return det->state;
}

// 遮断を伴うフォールト。処置が不可逆なのでラッチし、条件が消えても下ろさない。
static void LatchFault(Power *obj, uint32_t fault, const char *message) {
  if (obj->faults & fault) {
    return;
  }
  obj->faults |= fault;
  printf("Power fault: %s\n", message);
}

// 警告だけのフォールト。何も遮断していないので、条件が消えたらクリアする。
static void SetFault(Power *obj, uint32_t fault, int active, const char *message) {
  if (active == ((obj->faults & fault) != 0)) {
    return;
  }
  if (active) {
    obj->faults |= fault;
    printf("Power fault: %s\n", message);
  } else {
    obj->faults &= ~fault;
    printf("Power fault cleared: %s\n", message);
  }
}

// 電圧低下は検出後の復帰閾値を高くしてヒステリシスを持たせる。時間デバウンスだけでは
// 閾値をまたいで揺れる電圧に追従してしまい、表示がちらつく。
static int IsUndervoltage(PowerDetector *det, float voltage) {
  float threshold = det->state ? POWER_UNDERVOLTAGE_RECOVERY_V : POWER_UNDERVOLTAGE_THRESHOLD_V;
  return voltage < threshold;
}

// 駆動電源の要求値とフォールト状態から実際のピン出力を決める。シグナル系の過電流でも駆動電源を
// 落とすのは、ロジック電源が過負荷の状態で走らせるとブラウンアウトによるマイコンのリセット中に
// モーターへ指令が残る危険があるため。
static void ApplyDrivePower(Power *obj) {
  uint32_t cutoff_faults = POWER_FAULT_DRIVE_OVERCURRENT | POWER_FAULT_SIGNAL_OVERCURRENT;
  int on = obj->drive_power_request && !(obj->faults & cutoff_faults);
  DigitalOut_Write(&obj->drive_power, on);
}

static void InitDetector(PowerDetector *det) {
  det->state = 0;
  det->pending = 0;
  Timer_Init(&det->timer);
}

void Power_Init(Power *obj, AdcDma *adc1,
                 GPIO_TypeDef *drive_power_port, uint16_t drive_power_pin,
                 GPIO_TypeDef *lidar_power_port, uint16_t lidar_power_pin) {
  obj->adc1 = adc1;
  obj->drive_power_request = 0;
  obj->faults = POWER_FAULT_NONE;
  // ADC の初回変換が終わっている保証がないため、LPF の初期値は初回の Power_Update() で与える。
  // 0V から立ち上げると収束途中に電圧低下と誤判定してしまう
  obj->voltage_lpf_seeded = 0;
  obj->current_lpf_seeded = 0;
  InitDetector(&obj->drive_overcurrent);
  InitDetector(&obj->signal_overcurrent);
  InitDetector(&obj->drive_undervoltage);
  InitDetector(&obj->signal_undervoltage);
  // DigitalOut_Init は Low 出力から始めるため、両電源とも OFF で起動する
  DigitalOut_Init(&obj->drive_power, drive_power_port, drive_power_pin);
  DigitalOut_Init(&obj->lidar_power, lidar_power_port, lidar_power_pin);
}

void Power_Update(Power *obj) {
  if (!obj->voltage_lpf_seeded) {
    LPF_Init(&obj->voltage_signal_lpf, POWER_VOLTAGE_LPF_K, Power_GetVoltageSignal(obj));
    LPF_Init(&obj->voltage_drive_lpf, POWER_VOLTAGE_LPF_K, Power_GetVoltageDrive(obj));
    obj->voltage_lpf_seeded = 1;
  }
  LPF_Update(&obj->voltage_signal_lpf, Power_GetVoltageSignal(obj));
  LPF_Update(&obj->voltage_drive_lpf, Power_GetVoltageDrive(obj));

  if (!obj->current_lpf_seeded) {
    LPF_Init(&obj->current_signal_lpf, POWER_CURRENT_LPF_K, Power_GetCurrentSignal(obj));
    LPF_Init(&obj->current_drive_lpf, POWER_CURRENT_LPF_K, Power_GetCurrentDrive(obj));
    obj->current_lpf_seeded = 1;
  }
  LPF_Update(&obj->current_signal_lpf, Power_GetCurrentSignal(obj));
  LPF_Update(&obj->current_drive_lpf, Power_GetCurrentDrive(obj));

  if (UpdateDetector(&obj->signal_overcurrent,
                     Power_GetCurrentSignal(obj) > POWER_SIGNAL_OVERCURRENT_THRESHOLD_A,
                     POWER_OVERCURRENT_DEBOUNCE_S)) {
    LatchFault(obj, POWER_FAULT_SIGNAL_OVERCURRENT, "signal overcurrent");
    // シグナル系はマイコン自身の電源でもあるため丸ごとは切れない。切り離せる負荷である
    // LiDAR (と ApplyDrivePower 側で駆動系) を落として電流を下げる
    DigitalOut_Write(&obj->lidar_power, 0);
  }

  if (UpdateDetector(&obj->drive_overcurrent,
                     Power_GetCurrentDrive(obj) > POWER_DRIVE_OVERCURRENT_THRESHOLD_A,
                     POWER_OVERCURRENT_DEBOUNCE_S)) {
    LatchFault(obj, POWER_FAULT_DRIVE_OVERCURRENT, "drive overcurrent");
  }

  SetFault(obj, POWER_FAULT_SIGNAL_UNDERVOLTAGE,
           UpdateDetector(&obj->signal_undervoltage,
                          IsUndervoltage(&obj->signal_undervoltage, Power_GetVoltageSignalFiltered(obj)),
                          POWER_UNDERVOLTAGE_DEBOUNCE_S),
           "signal undervoltage");

  SetFault(obj, POWER_FAULT_DRIVE_UNDERVOLTAGE,
           UpdateDetector(&obj->drive_undervoltage,
                          IsUndervoltage(&obj->drive_undervoltage, Power_GetVoltageDriveFiltered(obj)),
                          POWER_UNDERVOLTAGE_DEBOUNCE_S),
           "drive undervoltage");

  ApplyDrivePower(obj);
}

uint32_t Power_GetFaults(Power *obj) {
  return obj->faults;
}

void Power_SetDrivePower(Power *obj, int on) {
  obj->drive_power_request = on ? 1 : 0;
  ApplyDrivePower(obj);
}

int Power_IsDriveOn(Power *obj) {
  uint32_t cutoff_faults = POWER_FAULT_DRIVE_OVERCURRENT | POWER_FAULT_SIGNAL_OVERCURRENT;
  return obj->drive_power_request && !(obj->faults & cutoff_faults);
}

void Power_SetLidarPower(Power *obj, int on) {
  DigitalOut_Write(&obj->lidar_power, on && !(obj->faults & POWER_FAULT_SIGNAL_OVERCURRENT));
}

float Power_GetVoltageSignal(Power *obj) {
  return AdcDma_GetVoltage(obj->adc1, POWER_ADC1_CH_VOLTAGE_S, ADC_VREF) / VOLTAGE_DIVIDER_RATIO;
}

float Power_GetVoltageDrive(Power *obj) {
  return AdcDma_GetVoltage(obj->adc1, POWER_ADC1_CH_VOLTAGE_P, ADC_VREF) / VOLTAGE_DIVIDER_RATIO;
}

float Power_GetVoltageSignalFiltered(Power *obj) {
  return (float)obj->voltage_signal_lpf.current_val;
}

float Power_GetVoltageDriveFiltered(Power *obj) {
  return (float)obj->voltage_drive_lpf.current_val;
}

float Power_GetCurrentSignal(Power *obj) {
  float v_sense = AdcDma_GetVoltage(obj->adc1, POWER_ADC1_CH_CURRENT_S, ADC_VREF);
  return v_sense / (CURRENT_SIGNAL_SHUNT_OHM * CURRENT_SENSE_GAIN);
}

float Power_GetCurrentDrive(Power *obj) {
  float v_sense = AdcDma_GetVoltage(obj->adc1, POWER_ADC1_CH_CURRENT_P, ADC_VREF);
  return v_sense / (CURRENT_DRIVE_SHUNT_OHM * CURRENT_SENSE_GAIN);
}

float Power_GetCurrentSignalFiltered(Power *obj) {
  return (float)obj->current_signal_lpf.current_val;
}

float Power_GetCurrentDriveFiltered(Power *obj) {
  return (float)obj->current_drive_lpf.current_val;
}

float Power_GetTemperatureC(Power *obj) {
  float v_sense = AdcDma_GetVoltage(obj->adc1, POWER_ADC1_CH_TEMP, ADC_VREF);
  return (v_sense - TEMP_V25) / TEMP_AVG_SLOPE_V_PER_C + 25.0f;
}
