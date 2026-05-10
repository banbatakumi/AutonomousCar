#include "imu.h"

#include <stdio.h>
#include <string.h>

#include "flash.h"

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

void Imu_Init(Imu* imu, I2C_HandleTypeDef* i2c, bool calibrate_on_boot) {
  if (!imu || !i2c) {
    return;
  }
  memset(imu, 0, sizeof(*imu));

  if (!MPU6050_Init(&imu->mpu, i2c, MPU6050_I2C_ADDR_DEFAULT)) {
    printf("[IMU] MPU6050 init failed\n");
    return;
  }
  imu->initialized = true;

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

  // 加速度から初期姿勢を推定
  MPU6050_PrimeOrientation(&imu->mpu);

  // 非同期I2C読み出し開始
  MPU6050_StartAsyncRead(&imu->mpu);
}

bool Imu_Update(Imu* imu) {
  if (!imu || !imu->initialized) {
    return false;
  }
  return MPU6050_Update(&imu->mpu);
}

const MPU6050_Data* Imu_GetData(const Imu* imu) {
  if (!imu || !imu->initialized) {
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
