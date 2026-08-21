#ifndef IMU_H_
#define IMU_H_

#include <stdbool.h>
#include <stdint.h>

#include "ahrs.h"
#include "i2c.h"
#include "lpf.h"
#include "mpu6050.h"

// 機体の姿勢センシング。MPU6050 ドライバ (lib/mpu6050) と姿勢推定フィルタ (lib/ahrs) を束ね、
// 「機体座標系での yaw/pitch/roll と加速度」を提供する。
//
// 精度を優先した設計:
//   - 取得はセンサの DATA_RDY 割り込み (PC5/INT) 起点。周期はセンサ内蔵クロックで決まりジッタがない
//   - サンプルはリングバッファに積まれるので、メインループが遅れても積分対象が欠落しない
//   - ジャイロのオフセットは起動時キャリブレーション + Flash 保存 (温度が変わるとずれるため、
//     静止を検出している間はオンラインで残留バイアスを追従補正する)
//   - 加速度は重力ノルムから外れたサンプルを姿勢補正に使わない (加減速時に姿勢が引っ張られるのを防ぐ)
//
// 地磁気センサがないため yaw は絶対方位ではなく、起動時 (または Imu_ResetYaw() を呼んだ時点)
// を 0 とする相対角。長時間ではゆっくりドリフトする。

// 機体座標系の定義: X = 前方, Y = 左方, Z = 上方 の右手系 (ROS REP-103 と同じ取り方)。
// したがって左旋回で yaw が増え、右へ傾けると roll が正、機首上げで pitch が正になる。
typedef struct {
  float yaw;    // [deg] -180..180 (左旋回が正)
  float pitch;  // [deg] -90..90
  float roll;   // [deg] -180..180

  float gyro_x;  // [deg/s] 機体座標系 (バイアス補正済み)
  float gyro_y;
  float gyro_z;

  float accel_x;  // [m/s^2] 機体座標系 (重力を含む・ローパス後)
  float accel_y;
  float accel_z;

  float linear_accel_x;  // [m/s^2] 推定姿勢から重力成分を差し引いた並進加速度
  float linear_accel_y;
  float linear_accel_z;

  float temp;       // [degC] センサ内部温度
  bool stationary;  // 静止と判定されているか (ジャイロバイアス追従中)
} ImuData;

typedef struct {
  Mpu6050 mpu;
  Ahrs ahrs;

  LPF lpf_accel_x;
  LPF lpf_accel_y;
  LPF lpf_accel_z;

  // 静止中に推定する残留ジャイロバイアス [deg/s] (温度ドリフト分を吸収する)
  float gyro_bias_x;
  float gyro_bias_y;
  float gyro_bias_z;
  uint32_t stationary_count;

  float yaw_offset;  // [deg] Imu_ResetYaw() で与える yaw の原点

  ImuData data;
  bool initialized;
  bool data_valid;
  uint32_t last_sample_tick;
  uint32_t last_recovery_tick;
  // I2C復旧 (Recover()) が走ったら立つ。復旧処理自体が~190msメインループを
  // ブロッキングするため、Imu_ConsumeRecoveryRan() で消費してハートビートの
  // 誤タイムアウト対策に使うことを想定している (docs/code_review_2026-08-21.md A-3)
  bool recovery_ran;
} Imu;

/**
 * @brief IMU を初期化する。I2C バス復旧 → MPU6050 初期化 → キャリブレーション読み込み →
 * 初期姿勢の推定 → DATA_RDY 割り込み (PC5) の有効化まで行う。
 * @param calibrate_on_boot true なら静止キャリブレーションを実行して Flash に保存する
 * (数秒かかるのでボタン押下起動などの明示的な操作に割り当てること)。
 * false なら Flash に保存済みの値を読み込む。
 */
void Imu_Init(Imu *obj, I2C_HandleTypeDef *i2c, bool calibrate_on_boot);

/**
 * @brief 溜まっているサンプルをすべて処理して姿勢・加速度を更新する。制御周期ごとに呼ぶこと。
 * @return 1 サンプル以上処理したら true。
 */
bool Imu_Update(Imu *obj);

/**
 * @brief 最新の推定結果を取得する (常に非 NULL。初回サンプル前はゼロ)。
 */
const ImuData *Imu_GetData(const Imu *obj);

/**
 * @brief 有効なデータを取得できているか (センサ未接続・通信断のときは false)。
 */
bool Imu_IsReady(const Imu *obj);

/**
 * @brief I2C復旧 (Recover()) が前回の呼び出し以降に走ったかを取得し、フラグをクリアする。
 * Recover() は ~190ms メインループをブロッキングするため (docs/code_review_2026-08-21.md A-3)、
 * 呼び出し側はこれを見てハートビート監視の基準時刻をリセットするなど、ブロッキングを
 * 前提としていない他モジュールの誤動作を避ける用途に使うこと。
 */
bool Imu_ConsumeRecoveryRan(Imu *obj);

/**
 * @brief 現在の向きを yaw = 0 とし直す。
 */
void Imu_ResetYaw(Imu *obj);

/**
 * @brief 静止キャリブレーションをやり直して Flash に保存する (ブロッキング)。
 */
bool Imu_RecalibrateAndSave(Imu *obj);

/**
 * @brief MPU6050 の INT ピン (PC5) の EXTI 割り込みから呼ぶこと。
 */
void Imu_OnDataReady(Imu *obj);

/**
 * @brief HAL_I2C_MemRxCpltCallback から呼ぶこと。
 */
void Imu_OnI2cRxComplete(Imu *obj, I2C_HandleTypeDef *hi2c);

/**
 * @brief HAL_I2C_ErrorCallback / HAL_I2C_AbortCpltCallback から呼ぶこと。
 */
void Imu_OnI2cError(Imu *obj, I2C_HandleTypeDef *hi2c);

#endif  // IMU_H_
