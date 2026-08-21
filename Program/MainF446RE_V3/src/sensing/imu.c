#include "imu.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flash.h"

// サンプリング設定。制御には使わない (高周期より精度優先) ので、
// DLPF を 21Hz まで絞ってノイズを落とし、その 10 倍弱の 200Hz で取り込む。
#define IMU_SAMPLE_RATE_HZ 200
#define IMU_DLPF MPU6050_DLPF_BW_21HZ

// Mahony フィルタのゲイン。Kp を小さめにして加減速の影響を受けにくくし、
// Ki で残留バイアスをゆっくり吸収する。
#define IMU_AHRS_KP 0.5f
#define IMU_AHRS_KI 0.01f

// 加速度出力のローパス係数 (0<k<1)。200Hz サンプルで実効カットオフ ≒ 7Hz。
#define IMU_ACCEL_LPF_K 0.8f

// 静止キャリブレーションのサンプル数 (200Hz なので約 5 秒)
#define IMU_CALIB_SAMPLE_COUNT 1000

// 静止判定のしきい値と、静止と認めるまでの連続サンプル数 (200Hz で約 0.5 秒)
#define IMU_STATIONARY_GYRO_DPS 1.5f
#define IMU_STATIONARY_ACCEL_TOL_MPS2 0.25f
#define IMU_STATIONARY_HOLD_SAMPLES 100

// 静止中にジャイロ残留バイアスへ追従する速さ (200Hz で時定数 ≒ 2.5 秒)
#define IMU_BIAS_TRACK_GAIN 0.002f

// 通信断とみなすまでの無サンプル時間と、復旧を試みる間隔
#define IMU_SAMPLE_TIMEOUT_MS 250U
#define IMU_RECOVERY_INTERVAL_MS 500U

// I2C1 のピン (Core/Src/i2c.c の HAL_I2C_MspInit と一致させること)
#define IMU_I2C_SCL_PORT GPIOB
#define IMU_I2C_SCL_PIN GPIO_PIN_8
#define IMU_I2C_SDA_PORT GPIOB
#define IMU_I2C_SDA_PIN GPIO_PIN_9

// Flash に保存するキャリブレーションデータ
typedef struct {
  uint32_t magic;
  uint32_t version;
  Mpu6050Offset offset;
  uint32_t checksum;
} ImuCalibrationRecord;

#define IMU_CALIB_MAGIC 0x4D505536UL  // "MPU6"
#define IMU_CALIB_VERSION 2UL

// INT ピン (PC5) の EXTI 設定は CubeMX 側 (.ioc -> MX_GPIO_Init) が行う。
// EXTI9_5 は後方超音波の ECHO ピン (PC9) と割り込みベクタを共有するため、
// NVIC ごと止めずに EXTI のマスクレジスタで PC5 のラインだけを開閉する。
static void EnableDataReadyInterrupt(bool enable) {
  if (enable) {
    __HAL_GPIO_EXTI_CLEAR_IT(INT_Pin);
    EXTI->IMR |= INT_Pin;
  } else {
    EXTI->IMR &= ~INT_Pin;
  }
}

// MCU だけがリセットされると、MPU6050 が送信途中のまま SDA を Low に保持して
// バスがロックすることがある。SCL を手動で 9 回叩いて転送を完了させ、STOP を出して解放する。
static void RecoverI2cBus(I2C_HandleTypeDef *i2c) {
  HAL_I2C_DeInit(i2c);
  HAL_Delay(5);

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = IMU_I2C_SCL_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_I2C_SCL_PORT, &gpio);
  HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);

  gpio.Pin = IMU_I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IMU_I2C_SDA_PORT, &gpio);

  for (int i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    if (HAL_GPIO_ReadPin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN) == GPIO_PIN_SET) break;
  }

  // STOP コンディション: SCL が High の間に SDA を Low -> High
  gpio.Pin = IMU_I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_I2C_SDA_PORT, &gpio);
  HAL_GPIO_WritePin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_SET);
  HAL_Delay(1);

  // MspInit がピンを AF に設定し直せるよう GPIO を戻してから再初期化する
  HAL_GPIO_DeInit(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN);
  HAL_GPIO_DeInit(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN);
  HAL_I2C_Init(i2c);
  HAL_Delay(10);
}

static uint32_t ComputeChecksum(const ImuCalibrationRecord *record) {
  return record->magic ^ record->version ^
         (uint32_t)(uint16_t)record->offset.gyro_x ^
         (uint32_t)(uint16_t)record->offset.gyro_y ^
         (uint32_t)(uint16_t)record->offset.gyro_z ^
         (uint32_t)(uint16_t)record->offset.accel_x ^
         (uint32_t)(uint16_t)record->offset.accel_y ^
         (uint32_t)(uint16_t)record->offset.accel_z ^ 0xA5A5A5A5UL;
}

static bool LoadCalibration(Mpu6050Offset *out) {
  ImuCalibrationRecord record;
  Flash_ReadData(FLASH_MPU_START_ADDR, &record, sizeof(record));
  if (record.magic != IMU_CALIB_MAGIC || record.version != IMU_CALIB_VERSION) {
    return false;
  }
  if (record.checksum != ComputeChecksum(&record)) return false;
  *out = record.offset;
  return true;
}

static bool SaveCalibration(const Mpu6050Offset *offset) {
  ImuCalibrationRecord record = {
      .magic = IMU_CALIB_MAGIC,
      .version = IMU_CALIB_VERSION,
      .offset = *offset,
  };
  record.checksum = ComputeChecksum(&record);
  return Flash_WriteDataToSector(FLASH_MPU_START_ADDR, FLASH_MPU_SECTOR,
                                 FLASH_MPU_VOLTAGE_RANGE, &record,
                                 sizeof(record)) == HAL_OK;
}

typedef struct {
  float x, y, z;
} ImuVec3;

// チップ座標系 -> 機体座標系 (X=前方 / Y=左方 / Z=上方) の変換。
// この機体の MPU6050 は基板に 90° 回して裏向きに実装されているため、
// 単なる軸の符号反転では表せず、X と Y の入れ替えが必要になる。
//   機体X(前) = +chip_y,  機体Y(左) = +chip_x,  機体Z(上) = -chip_z
//
// 基板を張り替えたときの確認手順 (Imu_Init が起動時にチップ座標系の加速度を表示する):
//   1. 水平に置いて静止 -> chip az の符号が機体Z(上)の向きを決める (+9.8 なら +chip_z)
//   2. 機首を持ち上げる -> 大きく振れた軸が機体X(前)。機首上げで機体X の加速度が
//      負になるように符号を選ぶ
//   3. 残った軸が機体Y。3軸の変換行列の行列式が +1 (右手系のまま) になるよう符号を決める
// 行列式が -1 (鏡像) になる組み合わせを選ぶと、Mahony フィルタが鏡像姿勢に収束して
// 「静止しているのに roll が 180° 付近」「回転させると一瞬逆に動く」といった症状が出る。
static ImuVec3 ChipToBody(float chip_x, float chip_y, float chip_z) {
  ImuVec3 body = {chip_y, chip_x, -chip_z};
  return body;
}

static void ResetAccelFilters(Imu *obj, const Mpu6050Sample *initial) {
  ImuVec3 accel = {0.0f, 0.0f, 0.0f};
  if (initial != NULL) {
    accel = ChipToBody(initial->accel_x, initial->accel_y, initial->accel_z);
  }
  LPF_Init(&obj->lpf_accel_x, IMU_ACCEL_LPF_K, accel.x);
  LPF_Init(&obj->lpf_accel_y, IMU_ACCEL_LPF_K, accel.y);
  LPF_Init(&obj->lpf_accel_z, IMU_ACCEL_LPF_K, accel.z);
}

// 加速度から初期姿勢を決めて、水平へ収束するまでの待ち時間をなくす
static void PrimeOrientation(Imu *obj) {
  Mpu6050Sample sample;
  if (!Mpu6050_ReadBlocking(&obj->mpu, &sample)) {
    ResetAccelFilters(obj, NULL);
    return;
  }
  ImuVec3 accel = ChipToBody(sample.accel_x, sample.accel_y, sample.accel_z);
  Ahrs_SetFromAccel(&obj->ahrs, accel.x, accel.y, accel.z);
  ResetAccelFilters(obj, &sample);
}

static float NormalizeDeg(float deg) {
  while (deg < -180.0f) deg += 360.0f;
  while (deg >= 180.0f) deg -= 360.0f;
  return deg;
}

// 静止していれば残留ジャイロバイアスへゆっくり追従する。
// 起動時キャリブレーション後も温度変化でバイアスがずれ、それが yaw のドリフトになるため。
static void TrackGyroBias(Imu *obj, float gx, float gy, float gz, float accel_norm) {
  bool quiet = fabsf(gx) < IMU_STATIONARY_GYRO_DPS &&
               fabsf(gy) < IMU_STATIONARY_GYRO_DPS &&
               fabsf(gz) < IMU_STATIONARY_GYRO_DPS &&
               fabsf(accel_norm - AHRS_GRAVITY_MPS2) < IMU_STATIONARY_ACCEL_TOL_MPS2;

  if (!quiet) {
    obj->stationary_count = 0;
    obj->data.stationary = false;
    return;
  }

  if (obj->stationary_count < IMU_STATIONARY_HOLD_SAMPLES) {
    obj->stationary_count++;
    return;
  }

  obj->data.stationary = true;
  obj->gyro_bias_x += IMU_BIAS_TRACK_GAIN * gx;
  obj->gyro_bias_y += IMU_BIAS_TRACK_GAIN * gy;
  obj->gyro_bias_z += IMU_BIAS_TRACK_GAIN * gz;
}

static void ProcessSample(Imu *obj, const Mpu6050Sample *sample, float dt) {
  ImuVec3 accel = ChipToBody(sample->accel_x, sample->accel_y, sample->accel_z);
  ImuVec3 gyro = ChipToBody(sample->gyro_x, sample->gyro_y, sample->gyro_z);

  float ax = accel.x;
  float ay = accel.y;
  float az = accel.z;
  float gx = gyro.x - obj->gyro_bias_x;
  float gy = gyro.y - obj->gyro_bias_y;
  float gz = gyro.z - obj->gyro_bias_z;

  float accel_norm = sqrtf(ax * ax + ay * ay + az * az);
  TrackGyroBias(obj, gx, gy, gz, accel_norm);

  Ahrs_Update(&obj->ahrs, gx, gy, gz, ax, ay, az, dt);

  obj->data.yaw = NormalizeDeg(obj->ahrs.yaw - obj->yaw_offset);
  obj->data.pitch = obj->ahrs.pitch;
  obj->data.roll = obj->ahrs.roll;

  obj->data.gyro_x = gx;
  obj->data.gyro_y = gy;
  obj->data.gyro_z = gz;

  obj->data.accel_x = (float)LPF_Update(&obj->lpf_accel_x, ax);
  obj->data.accel_y = (float)LPF_Update(&obj->lpf_accel_y, ay);
  obj->data.accel_z = (float)LPF_Update(&obj->lpf_accel_z, az);

  // 推定姿勢から重力の向きを求めて差し引き、機体の並進加速度だけを取り出す
  float grav_x, grav_y, grav_z;
  Ahrs_GetGravityDirection(&obj->ahrs, &grav_x, &grav_y, &grav_z);
  obj->data.linear_accel_x = obj->data.accel_x - grav_x * AHRS_GRAVITY_MPS2;
  obj->data.linear_accel_y = obj->data.accel_y - grav_y * AHRS_GRAVITY_MPS2;
  obj->data.linear_accel_z = obj->data.accel_z - grav_z * AHRS_GRAVITY_MPS2;

  obj->data.temp = sample->temp;
}

// 通信が止まったときに I2C と MPU6050 を初期化し直す。RecoverI2cBus + Mpu6050_Init だけで
// 約190msメインループをブロッキングする (docs/code_review_2026-08-21.md A-3)。この間に
// ハートビートの矩形波エッジを取りこぼすと誤って緊急停止がラッチされうるため、
// recovery_ran を立てて呼び出し側 (app.c) がハートビートの基準時刻をリセットできるようにする。
// これは応急処置であり、根本対策 (Recover のステートマシン化) は別途必要
static bool Recover(Imu *obj) {
  EnableDataReadyInterrupt(false);

  I2C_HandleTypeDef *i2c = obj->mpu.i2c;
  uint8_t addr = obj->mpu.i2c_addr;
  Mpu6050Offset offset = obj->mpu.offset;

  obj->data_valid = false;
  obj->recovery_ran = true;
  HAL_I2C_Master_Abort_IT(i2c, (uint16_t)(addr << 1));
  RecoverI2cBus(i2c);

  if (!Mpu6050_Init(&obj->mpu, i2c, addr, IMU_SAMPLE_RATE_HZ, IMU_DLPF)) {
    printf("[IMU] Recovery failed\n");
    obj->last_recovery_tick = HAL_GetTick();
    return false;
  }
  Mpu6050_SetOffset(&obj->mpu, &offset);
  PrimeOrientation(obj);

  obj->last_sample_tick = HAL_GetTick();
  obj->last_recovery_tick = obj->last_sample_tick;
  EnableDataReadyInterrupt(true);
  printf("[IMU] Recovered\n");
  return true;
}

void Imu_Init(Imu *obj, I2C_HandleTypeDef *i2c, bool calibrate_on_boot) {
  if (obj == NULL || i2c == NULL) return;
  memset(obj, 0, sizeof(*obj));

  EnableDataReadyInterrupt(false);  // 初期化中はブロッキング I2C を使うので割り込みを止めておく

  RecoverI2cBus(i2c);

  if (!Mpu6050_Init(&obj->mpu, i2c, MPU6050_I2C_ADDR_LOW, IMU_SAMPLE_RATE_HZ,
                    IMU_DLPF)) {
    printf("[IMU] Init failed\n");
    return;
  }

  Ahrs_Init(&obj->ahrs, IMU_AHRS_KP, IMU_AHRS_KI);

  if (calibrate_on_boot) {
    printf("[IMU] Calibrating... keep the vehicle still\n");
    if (Mpu6050_CalibrateOffset(&obj->mpu, IMU_CALIB_SAMPLE_COUNT)) {
      if (SaveCalibration(Mpu6050_GetOffset(&obj->mpu))) {
        printf("[IMU] Calibration saved\n");
      } else {
        printf("[IMU] Failed to save calibration\n");
      }
    } else {
      printf("[IMU] Calibration failed\n");
    }
  } else {
    Mpu6050Offset offset;
    if (LoadCalibration(&offset)) {
      Mpu6050_SetOffset(&obj->mpu, &offset);
      printf("[IMU] Calibration loaded from flash\n");
    } else {
      printf("[IMU] No calibration in flash; using zero offset\n");
    }
  }

  // 取付方向 (ChipToBody) を確認するための実測値。この機体は裏向き実装なので水平静止で chip az ≒ -9.8
  Mpu6050Sample sample;
  if (Mpu6050_ReadBlocking(&obj->mpu, &sample)) {
    printf("[IMU] Chip frame accel: %.2f, %.2f, %.2f [m/s^2]\n",
           (double)sample.accel_x, (double)sample.accel_y,
           (double)sample.accel_z);
  }

  PrimeOrientation(obj);

  obj->initialized = true;
  obj->last_sample_tick = HAL_GetTick();
  obj->last_recovery_tick = obj->last_sample_tick;
  EnableDataReadyInterrupt(true);
}

bool Imu_Update(Imu *obj) {
  if (obj == NULL || !obj->initialized) return false;

  const float dt = 1.0f / (float)obj->mpu.sample_rate_hz;
  Mpu6050Sample sample;
  bool updated = false;

  while (Mpu6050_PopSample(&obj->mpu, &sample)) {
    ProcessSample(obj, &sample, dt);
    updated = true;
  }

  uint32_t now = HAL_GetTick();
  if (updated) {
    obj->last_sample_tick = now;
    obj->data_valid = true;
    return true;
  }

  // Recover() の成否を待たず、サンプルが途絶した時点で即座に無効化する。以前はここを
  // Recover() 呼び出しの中でしか false にしておらず、IMU_RECOVERY_INTERVAL_MS の間隔で
  // しか Recover() を試みないため、その間 Imu_IsReady() が固まった値のまま true を
  // 返し続けていた (TV がヨーレートの固まった実測値で動き続ける、TCの基準速度も
  // 固まったヨーレートで計算されるなど、無効なデータを有効として使ってしまう)
  if ((uint32_t)(now - obj->last_sample_tick) >= IMU_SAMPLE_TIMEOUT_MS) {
    obj->data_valid = false;
    if ((uint32_t)(now - obj->last_recovery_tick) >= IMU_RECOVERY_INTERVAL_MS) {
      printf("[IMU] Sample timeout; recovering\n");
      Recover(obj);
    }
  }
  return false;
}

const ImuData *Imu_GetData(const Imu *obj) { return &obj->data; }

bool Imu_IsReady(const Imu *obj) { return obj->initialized && obj->data_valid; }

bool Imu_ConsumeRecoveryRan(Imu *obj) {
  bool ran = obj->recovery_ran;
  obj->recovery_ran = false;
  return ran;
}

void Imu_ResetYaw(Imu *obj) {
  if (obj == NULL) return;
  obj->yaw_offset = obj->ahrs.yaw;
  obj->data.yaw = 0.0f;
}

bool Imu_RecalibrateAndSave(Imu *obj) {
  if (obj == NULL || !obj->initialized) return false;

  EnableDataReadyInterrupt(false);
  bool ok = Mpu6050_CalibrateOffset(&obj->mpu, IMU_CALIB_SAMPLE_COUNT);
  if (ok) ok = SaveCalibration(Mpu6050_GetOffset(&obj->mpu));

  obj->gyro_bias_x = 0.0f;
  obj->gyro_bias_y = 0.0f;
  obj->gyro_bias_z = 0.0f;
  obj->stationary_count = 0;
  Ahrs_Init(&obj->ahrs, IMU_AHRS_KP, IMU_AHRS_KI);
  PrimeOrientation(obj);

  obj->last_sample_tick = HAL_GetTick();
  EnableDataReadyInterrupt(true);
  return ok;
}

void Imu_OnDataReady(Imu *obj) { Mpu6050_OnDataReady(&obj->mpu); }

void Imu_OnI2cRxComplete(Imu *obj, I2C_HandleTypeDef *hi2c) {
  Mpu6050_OnI2cRxComplete(&obj->mpu, hi2c);
}

void Imu_OnI2cError(Imu *obj, I2C_HandleTypeDef *hi2c) {
  Mpu6050_OnI2cError(&obj->mpu, hi2c);
}
