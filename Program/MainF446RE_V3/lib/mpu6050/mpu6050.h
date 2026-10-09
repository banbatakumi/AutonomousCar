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

// 出力レンジ
//   ジャイロ ±500 deg/s -> 65.5 LSB/(deg/s)
//   加速度   ±2 g       -> 16384 LSB/g (最も分解能の高いレンジ)
// ジャイロは 2026-10 に ±250 deg/s (4.36 rad/s) から広げた。この車は約 1.75m/s 以上の
// フルステアでヨーレートが 4.36 rad/s を超え、飽和した値が TC の基準速度と TV に入っていた。
// 分解能は 0.015 deg/s/LSB でノイズ (約 0.05 deg/s rms) より細かいので失うものは無い。
// **レンジを変えたらオフセット (LSB 単位で Flash に保存) の換算も要る** (imu.c の LoadCalibration)
#define MPU6050_GYRO_FS_SEL 1  // GYRO_CONFIG の FS_SEL (0:±250 1:±500 2:±1000 3:±2000 deg/s)
#define MPU6050_GYRO_LSB_PER_DPS 65.5f
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

  // ブロッキング I2C のタイムアウト [ms] と試行回数 (Mpu6050_BeginInit で決める)
  uint16_t io_timeout_ms;
  uint8_t io_retry_count;
  Mpu6050DlpfBandwidth dlpf;  // 段階的な初期化の間、設定値を持ち回る
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

// --- 待ち時間を呼び出し側が持つ、段階的な初期化 ---
// Mpu6050_Init は途中の待ち (リセット 100ms・PLL のロック 50ms) を HAL_Delay で待つので
// 約 150ms ブロッキングする。走行中の復旧では制御ループを止められないため、同じ手順を
// 「1回の呼び出し = I2C 1〜2 往復」に分けたものを公開する。順に:
//   Mpu6050_BeginInit → MPU6050_RESET_WAIT_MS 待つ → Mpu6050_WakeUp → MPU6050_WAKE_WAIT_MS 待つ
//   → Mpu6050_ConfigureStep を true (完了) になるまで繰り返す
// どれかが失敗 (false / ok=false) したら最初からやり直すこと。
#define MPU6050_RESET_WAIT_MS 100
#define MPU6050_WAKE_WAIT_MS 50
// 通常のブロッキング I2C のタイムアウトと試行回数
#define MPU6050_IO_TIMEOUT_MS 100
#define MPU6050_IO_RETRY_COUNT 3

/**
 * @brief デバイスを探してリセットを掛ける。obj は作り直される (オフセットも消える)。
 * io_timeout_ms / io_retry_count は以後のブロッキング I2C (Mpu6050_ReadBlocking を含む) に使う。
 * 走行中の復旧では、応答が無いときに止まる時間をこれで縛る。
 */
bool Mpu6050_BeginInit(Mpu6050 *obj, I2C_HandleTypeDef *i2c, uint8_t i2c_addr,
                       uint16_t sample_rate_hz, Mpu6050DlpfBandwidth dlpf,
                       uint16_t io_timeout_ms, uint8_t io_retry_count);

/**
 * @brief スリープを解除してクロック源を選ぶ。
 */
bool Mpu6050_WakeUp(Mpu6050 *obj);

/**
 * @brief レジスタを1つ設定する。*step は 0 から始めて呼ぶたびに進む。
 * @param ok 書き込みに失敗したら false が入る (そのときの戻り値は true = これ以上進めない)。
 * @return すべて設定し終えた (initialized になった) か、失敗したら true。
 */
bool Mpu6050_ConfigureStep(Mpu6050 *obj, uint8_t *step, bool *ok);

/**
 * @brief ブロッキング I2C のタイムアウトと試行回数を変える。
 */
void Mpu6050_SetIoTimeout(Mpu6050 *obj, uint16_t io_timeout_ms, uint8_t io_retry_count);

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
