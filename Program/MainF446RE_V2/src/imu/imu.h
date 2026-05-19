#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stdint.h>

#include "i2c.h"
#include "lpf.h"
#include "mpu6050.h"

// MPU6050 + flash 永続キャリブレーションのラッパ
typedef struct {
  MPU6050 mpu;
  bool initialized;
  bool calibration_loaded;
  bool data_valid;
  uint32_t last_update_tick;
  uint32_t last_reconnect_attempt_tick;
  LPF lpf_accel_x;
  LPF lpf_accel_y;
  LPF lpf_accel_z;
} Imu;

// 初期化。button2 が起動時に押されていればキャリブレーションを実行して flash に保存。
// それ以外の場合は flash からキャリブレーションを読み込み、無ければゼロオフセットで開始。
void Imu_Init(Imu* imu, I2C_HandleTypeDef* i2c, bool calibrate_on_boot);

// センサデータ更新 (非同期I2Cが完了していれば true、未完了なら false)
bool Imu_Update(Imu* imu);

// 直近の出力データを取得
const MPU6050_Data* Imu_GetData(const Imu* imu);

// HAL の I2C MemRxCplt コールバックから呼び出して非同期読み出しの完了を通知
void Imu_OnI2CRxComplete(Imu* imu, I2C_HandleTypeDef* hi2c);

// HAL の I2C Error コールバックから呼び出して非同期読み出しの固着を解除
void Imu_OnI2CError(Imu* imu, I2C_HandleTypeDef* hi2c);

// 強制的に再キャリブレーションして flash に保存 (デバッグ/手動再校正用)
bool Imu_RecalibrateAndSave(Imu* imu);

#endif  // IMU_H_
