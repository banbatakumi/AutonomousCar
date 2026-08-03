#ifndef POWER_H_
#define POWER_H_

#include "adc_dma.h"
#include "digitalinout.h"

// ADC1 の Rank 順 (Core/Src/adc.c の MX_ADC1_Init() のチャンネル設定順と一致させること)
typedef enum {
  POWER_ADC1_CH_VOLTAGE_S = 0,  // シグナル系(ロジック電源)電圧
  POWER_ADC1_CH_VOLTAGE_P,      // 駆動系(モーター電源)電圧
  POWER_ADC1_CH_CURRENT_S,      // シグナル系電流
  POWER_ADC1_CH_CURRENT_P,      // 駆動系電流
  POWER_ADC1_CH_TEMP,           // マイコン内蔵温度センサ
  POWER_ADC1_NUM_CH,
} PowerAdc1Channel;

// 駆動系の過電流保護閾値 [A]。超えたら DRIVE_POWER を遮断する。
#define POWER_DRIVE_OVERCURRENT_THRESHOLD_A 5.0f

typedef struct {
  AdcDma *adc1;
  DigitalOut drive_power;  // DRIVE_POWER: モータードライバ電源スイッチ
  DigitalOut lidar_power;  // LIDAR_POWER: LiDAR 電源スイッチ
  int drive_tripped;       // 過電流により駆動電源を遮断済みか (一度遮断したらリセットまで復帰しない)
} Power;

/**
 * @brief 電源管理モジュールを初期化する。ADC1 (VOLTAGE_S/P, CURRENT_S/P, TEMP) を DMA で読む AdcDma と、
 * 駆動電源 (DRIVE_POWER) / LiDAR 電源 (LIDAR_POWER) スイッチのポート・ピンを渡す。
 * 初期化時に駆動電源は ON、LiDAR 電源は OFF になる。
 */
void Power_Init(Power *obj, AdcDma *adc1,
                 GPIO_TypeDef *drive_power_port, uint16_t drive_power_pin,
                 GPIO_TypeDef *lidar_power_port, uint16_t lidar_power_pin);

/**
 * @brief 駆動系の過電流保護を更新する。駆動電流が POWER_DRIVE_OVERCURRENT_THRESHOLD_A を超えたら
 * 駆動電源を遮断し、以降は遮断状態を保持する (自動復帰しない)。制御周期ごとに呼ぶこと。
 */
void Power_Update(Power *obj);

/**
 * @brief 過電流保護により駆動電源が遮断されたかどうかを取得する。
 */
int Power_IsDriveTripped(Power *obj);

/**
 * @brief LiDAR 電源 (LIDAR_POWER) の ON/OFF を切り替える。
 */
void Power_SetLidarPower(Power *obj, int on);

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
