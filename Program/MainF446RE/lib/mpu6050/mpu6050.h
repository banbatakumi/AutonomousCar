#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

// MPU6050 I2C addresses (AD0 = 0 / 1)
#define MPU6050_I2C_ADDR_DEFAULT 0x68
#define MPU6050_I2C_ADDR_ALT 0x69

// 出力レンジ
//   ジャイロ: ±250 [deg/s] -> 131 LSB / (deg/s)
//   加速度  : ±2 [g]       -> 16384 LSB / g
#define MPU6050_GYRO_LSB_PER_DPS 131.0f
#define MPU6050_ACCEL_LSB_PER_G 16384.0f
#define MPU6050_GRAVITY_MPS2 9.80665f

// Calibration data (gyro / accel offsets, raw LSB units)
typedef struct {
  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;
  int16_t accel_offset_x;
  int16_t accel_offset_y;
  int16_t accel_offset_z;
} MPU6050_Calibration;

// 出力データ
typedef struct {
  // 姿勢 [deg]
  float yaw;
  float pitch;
  float roll;

  // 角速度 [deg/s]
  float gyro_x;
  float gyro_y;
  float gyro_z;

  // 角加速度 [deg/s^2]
  float angular_accel_x;
  float angular_accel_y;
  float angular_accel_z;

  // 並進加速度 [m/s^2]
  float accel_x;
  float accel_y;
  float accel_z;

  // 温度 [degC]
  float temp;
} MPU6050_Data;

// MPU6050 ハンドル
typedef struct {
  I2C_HandleTypeDef* i2c;
  uint8_t i2c_addr;

  MPU6050_Calibration calib;

  // Mahony AHRS quaternion + ゲイン
  float q0, q1, q2, q3;
  float two_kp;
  float two_ki;
  float integral_fbx;
  float integral_fby;
  float integral_fbz;

  // 角加速度計算用に前回ジャイロ値と時刻を保持
  float prev_gyro_x;
  float prev_gyro_y;
  float prev_gyro_z;
  uint32_t prev_update_us;
  bool has_prev_gyro;

  // 非同期I2C読み出しバッファ
  uint8_t rx_buffer[14];
  volatile uint8_t rx_ready;
  volatile uint8_t async_active;

  MPU6050_Data data;
  bool initialized;
} MPU6050;

// 初期化(I2C接続/レジスタ設定)。成功時 true。
bool MPU6050_Init(MPU6050* mpu, I2C_HandleTypeDef* i2c, uint8_t i2c_addr);

// キャリブレーション実行。静止状態で sample_count 個サンプルを平均してオフセット決定。
bool MPU6050_Calibrate(MPU6050* mpu, uint16_t sample_count);

// キャリブレーション値を直接適用 (flashから読んだ値を流し込む用)
void MPU6050_SetCalibration(MPU6050* mpu, const MPU6050_Calibration* calib);

// 加速度センサから初期姿勢を求めて quaternion を初期化する
bool MPU6050_PrimeOrientation(MPU6050* mpu);

// センサデータ更新。非同期I2C読みがレディなら data を更新して true。
// 未到着なら次回読みを起動して false。
bool MPU6050_Update(MPU6050* mpu);

// 非同期読み出しを開始 (HAL_I2C_Mem_Read_IT)。HAL I2C MemRxCplt callbackで完了通知。
bool MPU6050_StartAsyncRead(MPU6050* mpu);

// HAL I2C コールバックから呼び出してフラグを立てる (Drv非依存にするため公開)
void MPU6050_OnI2CRxComplete(MPU6050* mpu, I2C_HandleTypeDef* hi2c);

// 直近の出力データを取得
const MPU6050_Data* MPU6050_GetData(const MPU6050* mpu);

#endif  // MPU6050_H_
