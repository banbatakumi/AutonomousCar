#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

// MPU6050 (3軸ジャイロ + 3軸加速度) の I2C ドライバ。
//
// 読み出しは割り込み駆動 (HAL_I2C_Mem_Read_IT) で行いブロッキングしない。
// センサの DATA_RDY 割り込み (INT ピン) を EXTI で受けて 1 サンプルずつ取り込むため、
// 取得周期はセンサ内蔵クロックで決まり、メインループの負荷変動の影響を受けない。
// 取り込んだサンプルは ISR 内でリングバッファに積まれ、メインループ側が
// Mpu6050_PopSample() で 1 サンプルずつ取り出す (メインループが一時的に遅れても
// サンプルが欠落せず、姿勢積分の精度が落ちない)。

// I2C 7bit アドレス (AD0 ピンの L/H で決まる)
#define MPU6050_I2C_ADDR_LOW 0x68
#define MPU6050_I2C_ADDR_HIGH 0x69

// ACCEL_XOUT_H から連続読みするバイト数 (accel 6 + temp 2 + gyro 6)
#define MPU6050_BURST_LEN 14

// ISR が取り込んだサンプルを溜めるリングバッファ段数
#define MPU6050_RING_DEPTH 8

// 出力レンジ (精度優先のため最も分解能の高いレンジに固定)
//   ジャイロ ±250 deg/s -> 131 LSB/(deg/s)
//   加速度   ±2 g       -> 16384 LSB/g
#define MPU6050_GYRO_LSB_PER_DPS 131.0f
#define MPU6050_ACCEL_LSB_PER_G 16384.0f
#define MPU6050_GRAVITY_MPS2 9.80665f

// デジタルローパスフィルタ (DLPF) の帯域。狭いほど低ノイズだが位相遅れが増える。
// 260Hz のときだけ内部サンプルレートが 8kHz、それ以外は 1kHz になる。
typedef enum {
  MPU6050_DLPF_BW_260HZ = 0,
  MPU6050_DLPF_BW_184HZ = 1,
  MPU6050_DLPF_BW_94HZ = 2,
  MPU6050_DLPF_BW_44HZ = 3,
  MPU6050_DLPF_BW_21HZ = 4,
  MPU6050_DLPF_BW_10HZ = 5,
  MPU6050_DLPF_BW_5HZ = 6,
} Mpu6050DlpfBandwidth;

// センサ生値のオフセット [LSB] (Mpu6050_CalibrateOffset() で求める)
typedef struct {
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
} Mpu6050Offset;

// 物理量に変換した 1 サンプル (チップ座標系。機体座標系への変換は上位で行う)
typedef struct {
  float accel_x;  // [m/s^2]
  float accel_y;
  float accel_z;
  float gyro_x;  // [deg/s]
  float gyro_y;
  float gyro_z;
  float temp;  // [degC]
} Mpu6050Sample;

typedef struct {
  I2C_HandleTypeDef *i2c;
  uint8_t i2c_addr;  // 7bit アドレス
  uint16_t sample_rate_hz;
  Mpu6050Offset offset;

  uint8_t rx_frame[MPU6050_BURST_LEN];  // 転送先バッファ (HAL が直接書き込む)
  uint8_t ring[MPU6050_RING_DEPTH][MPU6050_BURST_LEN];
  volatile uint8_t head;  // ISR (書き込み側) が進める
  volatile uint8_t tail;  // メインループ (読み出し側) が進める

  volatile bool reading;            // I2C 転送中
  volatile uint32_t dropped_count;  // リングバッファ満杯/転送中で捨てたサンプル数
  volatile uint32_t sample_count;   // 取り込みに成功したサンプル総数

  bool initialized;
} Mpu6050;

/**
 * @brief MPU6050 を初期化する (リセット・レンジ設定・DATA_RDY 割り込み有効化まで)。
 * ブロッキング I2C を使うので EXTI を有効にする前に呼ぶこと。
 * @param i2c_addr MPU6050_I2C_ADDR_LOW / _HIGH。応答しなければもう一方も自動で試す。
 * @param sample_rate_hz 出力レート [Hz]。内部レート (DLPF により 1kHz or 8kHz) の整数分の1になる。
 * @param dlpf デジタルローパスフィルタ帯域。
 * @return 初期化に成功したら true。
 */
bool Mpu6050_Init(Mpu6050 *obj, I2C_HandleTypeDef *i2c, uint8_t i2c_addr,
                  uint16_t sample_rate_hz, Mpu6050DlpfBandwidth dlpf);

/**
 * @brief 静止状態でサンプルを平均してオフセットを求める (ブロッキング)。
 * 水平な場所に置いて静止させた状態で呼ぶこと。加速度は重力成分を除いた分をオフセットとする。
 * @return 成功したら true。
 */
bool Mpu6050_CalibrateOffset(Mpu6050 *obj, uint16_t sample_count);

/**
 * @brief オフセットを外部から与える (Flash から読んだ値を流し込む用)。
 */
void Mpu6050_SetOffset(Mpu6050 *obj, const Mpu6050Offset *offset);

/**
 * @brief 現在のオフセットを取得する (Flash へ保存する用)。
 */
const Mpu6050Offset *Mpu6050_GetOffset(const Mpu6050 *obj);

/**
 * @brief ブロッキングで 1 サンプル読む (初期姿勢の推定やキャリブレーション用)。
 */
bool Mpu6050_ReadBlocking(Mpu6050 *obj, Mpu6050Sample *out);

/**
 * @brief DATA_RDY 割り込み (INT ピンの EXTI) から呼ぶこと。非同期読み出しを開始する。
 */
void Mpu6050_OnDataReady(Mpu6050 *obj);

/**
 * @brief HAL_I2C_MemRxCpltCallback から呼ぶこと。受信したフレームをリングバッファへ積む。
 */
void Mpu6050_OnI2cRxComplete(Mpu6050 *obj, I2C_HandleTypeDef *hi2c);

/**
 * @brief HAL_I2C_ErrorCallback / AbortCpltCallback から呼ぶこと。転送中フラグの固着を解除する。
 */
void Mpu6050_OnI2cError(Mpu6050 *obj, I2C_HandleTypeDef *hi2c);

/**
 * @brief 未処理のサンプルを 1 つ取り出して物理量に変換する。溜まっている限り true を返す。
 * メインループで while ループにして呼び、溜まった分をすべて処理すること。
 */
bool Mpu6050_PopSample(Mpu6050 *obj, Mpu6050Sample *out);

/**
 * @brief 未処理サンプル数を取得する。
 */
uint8_t Mpu6050_GetPendingCount(const Mpu6050 *obj);

#endif  // MPU6050_H_
