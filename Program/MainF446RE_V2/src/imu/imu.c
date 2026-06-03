#include "imu.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "flash.h"
#include "mymath.h"

// flash 上のキャリブレーションデータ
typedef struct {
  uint32_t magic;
  uint32_t version;
  MPU6050_Calibration calib;
  uint32_t checksum;
} ImuCalibrationFlash;

#define IMU_CALIB_MAGIC 0x4D505536UL  // "MPU6"
#define IMU_CALIB_VERSION 1UL
#define IMU_CALIB_FLASH_ADDR FLASH_MPU_START_ADDR

// キャリブレーション中のサンプル数 (静止 5 秒程度)
#define IMU_CALIB_SAMPLE_COUNT 500

// 並進加速度の LPF 平滑化係数 (0 < k < 1, 大きいほど強い平滑)
// IMU 更新レート ~1 kHz 想定、実効カットオフ ~16 Hz
#define IMU_ACCEL_LPF_K 0.9

// 非同期 I2C が止まったと判断するまでの時間と再接続試行間隔
#define IMU_RECONNECT_TIMEOUT_MS 250U
#define IMU_RECONNECT_INTERVAL_MS 500U

// 機体への取付方向 (chip frame -> body frame)
// AutonomousCar: MPU6050 はチップ Z軸まわり 180° 回転して実装されている。
// (チップ X が機体後方、Y が機体右方向、Z は上向きのまま)
// 取付を変更したらここを書き換える。
#define IMU_MOUNT_X_SIGN (-1.0f)
#define IMU_MOUNT_Y_SIGN (-1.0f)
#define IMU_MOUNT_Z_SIGN (1.0f)

// I2C1 バス回復用 GPIO ピン (i2c.c の MX_I2C1_Init に対応)
#define IMU_I2C_SCL_PORT GPIOB
#define IMU_I2C_SCL_PIN  GPIO_PIN_8
#define IMU_I2C_SDA_PORT GPIOB
#define IMU_I2C_SDA_PIN  GPIO_PIN_9

// 出力 Euler 角の取付起因の補正
//   yaw   : 回転方向反転 (true なら符号反転)
//   roll  : 静止水平で 180° となる原点を 0° へシフト
#define IMU_YAW_INVERT 1
#define IMU_ROLL_OFFSET_DEG 180.0f

// STMリセット後に非同期I2C転送が中断し SDA がスタックする問題を回復する。
// SCLを9回手動トグル → STOPコンディション発行 → I2C再初期化。
static void RecoverI2CBus(I2C_HandleTypeDef* i2c) {
  HAL_I2C_DeInit(i2c);
  HAL_Delay(5);

  GPIO_InitTypeDef gpio = {0};

  // SCL (PB8): オープンドレイン出力
  gpio.Pin   = IMU_I2C_SCL_PIN;
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_I2C_SCL_PORT, &gpio);
  HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);

  // SDA (PB9): 入力でスタック状態をモニタ
  gpio.Pin  = IMU_I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(IMU_I2C_SDA_PORT, &gpio);

  // SDAがHighになるまで最大9クロック
  for (int i = 0; i < 9; i++) {
    HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    if (HAL_GPIO_ReadPin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN) == GPIO_PIN_SET) {
      break;
    }
  }

  // STOPコンディション: SCL=H中にSDA L→H
  gpio.Pin  = IMU_I2C_SDA_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_I2C_SDA_PORT, &gpio);
  HAL_GPIO_WritePin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN, GPIO_PIN_SET);
  HAL_Delay(1);

  // GPIOをリセットして HAL_I2C_Init の MspInit がピンを正しく設定できるようにする
  HAL_GPIO_DeInit(IMU_I2C_SCL_PORT, IMU_I2C_SCL_PIN);
  HAL_GPIO_DeInit(IMU_I2C_SDA_PORT, IMU_I2C_SDA_PIN);
  HAL_I2C_Init(i2c);
  HAL_Delay(10);
}

static uint32_t ComputeChecksum(const ImuCalibrationFlash* data) {
  return data->magic ^ data->version ^
         (uint32_t)(uint16_t)data->calib.gyro_offset_x ^
         (uint32_t)(uint16_t)data->calib.gyro_offset_y ^
         (uint32_t)(uint16_t)data->calib.gyro_offset_z ^
         (uint32_t)(uint16_t)data->calib.accel_offset_x ^
         (uint32_t)(uint16_t)data->calib.accel_offset_y ^
         (uint32_t)(uint16_t)data->calib.accel_offset_z ^ 0xA5A5A5A5UL;
}

static bool LoadCalibration(MPU6050_Calibration* out) {
  ImuCalibrationFlash data;
  Flash_ReadData(IMU_CALIB_FLASH_ADDR, &data, sizeof(data));

  if (data.magic != IMU_CALIB_MAGIC || data.version != IMU_CALIB_VERSION) {
    return false;
  }
  if (data.checksum != ComputeChecksum(&data)) {
    return false;
  }
  *out = data.calib;
  return true;
}

static bool SaveCalibration(const MPU6050_Calibration* calib) {
  ImuCalibrationFlash data = {
      .magic = IMU_CALIB_MAGIC,
      .version = IMU_CALIB_VERSION,
      .calib = *calib,
  };
  data.checksum = ComputeChecksum(&data);

  return Flash_WriteDataToSector(IMU_CALIB_FLASH_ADDR, FLASH_MPU_SECTOR,
                                 FLASH_MPU_VOLTAGE_RANGE, &data,
                                 sizeof(data)) == HAL_OK;
}

static void ApplyMount(MPU6050* mpu) {
  MPU6050_Mount mount = {
      .x_sign = IMU_MOUNT_X_SIGN,
      .y_sign = IMU_MOUNT_Y_SIGN,
      .z_sign = IMU_MOUNT_Z_SIGN,
  };
  MPU6050_SetMount(mpu, &mount);
}

static void ResetAccelFilters(Imu* imu) {
  LPF_Init(&imu->lpf_accel_x, IMU_ACCEL_LPF_K, 0.0);
  LPF_Init(&imu->lpf_accel_y, IMU_ACCEL_LPF_K, 0.0);
  LPF_Init(&imu->lpf_accel_z, IMU_ACCEL_LPF_K, 0.0);
}

static bool Reconnect(Imu* imu) {
  if (!imu || !imu->mpu.i2c) {
    return false;
  }

  I2C_HandleTypeDef* i2c = imu->mpu.i2c;
  uint8_t i2c_addr = imu->mpu.i2c_addr;
  MPU6050_Calibration calib = imu->mpu.calib;
  bool calibration_loaded = imu->calibration_loaded;

  imu->initialized = false;
  imu->data_valid = false;

  HAL_I2C_Master_Abort_IT(i2c, i2c_addr << 1);
  HAL_I2C_DeInit(i2c);
  if (HAL_I2C_Init(i2c) != HAL_OK) {
    printf("[IMU] I2C reinit failed\n");
    imu->initialized = true;
    return false;
  }

  if (!MPU6050_Init(&imu->mpu, i2c, i2c_addr)) {
    printf("[IMU] MPU6050 reconnect failed\n");
    imu->initialized = true;
    return false;
  }

  ApplyMount(&imu->mpu);
  if (calibration_loaded) {
    MPU6050_SetCalibration(&imu->mpu, &calib);
  }
  imu->calibration_loaded = calibration_loaded;
  ResetAccelFilters(imu);

  MPU6050_PrimeOrientation(&imu->mpu);
  MPU6050_StartAsyncRead(&imu->mpu);

  imu->initialized = true;
  imu->last_update_tick = HAL_GetTick();
  printf("[IMU] Reconnected\n");
  return true;
}

void Imu_Init(Imu* imu, I2C_HandleTypeDef* i2c, bool calibrate_on_boot) {
  if (!imu || !i2c) {
    return;
  }
  memset(imu, 0, sizeof(*imu));

  // STMリセット時に非同期I2C転送が中断してSDAがスタックする場合があるため回復する
  RecoverI2CBus(i2c);

  if (!MPU6050_Init(&imu->mpu, i2c, MPU6050_I2C_ADDR_DEFAULT)) {
    printf("[IMU] MPU6050 init failed\n");
    return;
  }
  imu->initialized = true;

  // 機体への取付方向をライブラリへ通知
  ApplyMount(&imu->mpu);

  if (calibrate_on_boot) {
    printf("[IMU] Button2 held -> calibration\n");
    if (MPU6050_Calibrate(&imu->mpu, IMU_CALIB_SAMPLE_COUNT)) {
      if (SaveCalibration(&imu->mpu.calib)) {
        printf("[IMU] Calibration saved to flash\n");
        imu->calibration_loaded = true;
      } else {
        printf("[IMU] Failed to save calibration\n");
      }
    } else {
      printf("[IMU] Calibration failed\n");
    }
  } else {
    MPU6050_Calibration calib;
    if (LoadCalibration(&calib)) {
      MPU6050_SetCalibration(&imu->mpu, &calib);
      imu->calibration_loaded = true;
      printf("[IMU] Calibration loaded from flash\n");
    } else {
      printf("[IMU] No valid calibration in flash; using zero offsets\n");
    }
  }

  ResetAccelFilters(imu);

  // 加速度から初期姿勢を推定
  MPU6050_PrimeOrientation(&imu->mpu);

  // 非同期I2C読み出し開始
  MPU6050_StartAsyncRead(&imu->mpu);
  imu->last_update_tick = HAL_GetTick();
}

bool Imu_Update(Imu* imu) {
  if (!imu || !imu->initialized) {
    return false;
  }
  if (!MPU6050_Update(&imu->mpu)) {
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - imu->last_update_tick) >= IMU_RECONNECT_TIMEOUT_MS &&
        (uint32_t)(now - imu->last_reconnect_attempt_tick) >= IMU_RECONNECT_INTERVAL_MS) {
      imu->last_reconnect_attempt_tick = now;
      printf("[IMU] Update timeout; reconnecting\n");
      Reconnect(imu);
    }
    return false;
  }
  imu->last_update_tick = HAL_GetTick();
  imu->data_valid = true;

  // 取付起因の出力 Euler 角の補正 (ライブラリ計算には影響を与えない後処理)
#if IMU_YAW_INVERT
  imu->mpu.data.yaw = -imu->mpu.data.yaw;
#endif
  imu->mpu.data.roll -= IMU_ROLL_OFFSET_DEG;
  if (imu->mpu.data.roll < -180.0f) {
    imu->mpu.data.roll += 360.0f;
  } else if (imu->mpu.data.roll >= 180.0f) {
    imu->mpu.data.roll -= 360.0f;
  }

  imu->mpu.data.accel_x = (float)LPF_Update(&imu->lpf_accel_x, imu->mpu.data.accel_x);
  imu->mpu.data.accel_y = (float)LPF_Update(&imu->lpf_accel_y, imu->mpu.data.accel_y);
  imu->mpu.data.accel_z = (float)LPF_Update(&imu->lpf_accel_z, imu->mpu.data.accel_z);

  float pitch_rad = imu->mpu.data.pitch * (float)DEG_TO_RAD;
  float roll_rad = imu->mpu.data.roll * (float)DEG_TO_RAD;
  imu->mpu.data.accel_x += 9.81f * sinf(pitch_rad);
  imu->mpu.data.accel_y += 9.81f * sinf(roll_rad);
  return true;
}

const MPU6050_Data* Imu_GetData(const Imu* imu) {
  if (!imu || !imu->initialized || !imu->data_valid) {
    return NULL;
  }
  return MPU6050_GetData(&imu->mpu);
}

void Imu_OnI2CRxComplete(Imu* imu, I2C_HandleTypeDef* hi2c) {
  if (!imu || !imu->initialized) {
    return;
  }
  MPU6050_OnI2CRxComplete(&imu->mpu, hi2c);
}

void Imu_OnI2CError(Imu* imu, I2C_HandleTypeDef* hi2c) {
  if (!imu || !imu->initialized) {
    return;
  }
  MPU6050_OnI2CError(&imu->mpu, hi2c);
}

bool Imu_RecalibrateAndSave(Imu* imu) {
  if (!imu || !imu->initialized) {
    return false;
  }
  if (!MPU6050_Calibrate(&imu->mpu, IMU_CALIB_SAMPLE_COUNT)) {
    return false;
  }
  if (!SaveCalibration(&imu->mpu.calib)) {
    return false;
  }
  imu->calibration_loaded = true;
  MPU6050_PrimeOrientation(&imu->mpu);
  MPU6050_StartAsyncRead(&imu->mpu);
  return true;
}
