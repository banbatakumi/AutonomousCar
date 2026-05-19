#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdbool.h>

#include "i2c.h"
#include "ld06.h"
#include "mpu6050.h"
#include "pwm_out.h"

#define ADC_VALUE_COUNT 5
#define ADC2VOLT (3.3 / 4095.0)
#define VOLTAGE_DIVIDER_RATIO ((10.0 + 1.0) / 1.0)
#define MIN_VOLTAGE 8.0
#define MAX_VOLTAGE 11.0
#define BLINK_PERIOD_US_AT_MIN_VOLTAGE 100U
#define BLINK_PERIOD_US_AT_MAX_VOLTAGE 1000000U

/**
 * @brief センサ・周辺機器を初期化する。
 * @param recalibrate_imu true のとき IMU を再キャリブレーションして Flash に保存する。
 */
void Sensor_Init(bool recalibrate_imu);

/**
 * @brief 全センサを更新し、バッテリー電圧チェックを行う。
 *        バッテリー低電圧時は Sensor_GetBatteryError() が true を返す。
 *        IMU データは Sensor_GetImuData() で取得すること。
 *        毎制御ループの先頭で呼ぶこと。
 */
void Sensor_Update(void);

/**
 * @brief バッテリー電圧をSin波で輝度変調した PWM LED に反映する。
 *        高電圧=ゆっくり点滅、低電圧=速く点滅。
 */
void Sensor_UpdateVoltageLeds(PwmOut* led_signal, PwmOut* led_power);

/** @brief HAL I2C MemRxCplt コールバックから呼ぶこと (MPU6050 非同期読み出し完了通知)。 */
void Sensor_OnI2CRxComplete(I2C_HandleTypeDef* hi2c);

/** @brief HAL I2C Error コールバックから呼ぶこと (MPU6050 非同期読み出しエラー通知)。 */
void Sensor_OnI2CError(I2C_HandleTypeDef* hi2c);

bool Sensor_GetBatteryError(void);
double Sensor_GetVoltageSignal(void);
double Sensor_GetVoltagePower(void);
const MPU6050_Data* Sensor_GetImuData(void);

float Sensor_GetUltrasonicFront(void);
float Sensor_GetUltrasonicRight(void);
float Sensor_GetUltrasonicLeft(void);
float Sensor_GetUltrasonicBack(void);

const LD06* Sensor_GetLidar(void);

#endif  // SENSOR_H_
