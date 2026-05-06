#include "imu_manager.h"

#include <stdio.h>
#include <string.h>

#include "flash.h"

typedef struct {
  uint32_t magic;
  uint32_t version;
  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;
  int16_t accel_offset_x;
  int16_t accel_offset_y;
  int16_t accel_offset_z;
  uint32_t checksum;
} MPU6050_CalibrationData;

#define MPU6050_CALIB_MAGIC 0x4D505531UL
#define MPU6050_CALIB_VERSION 1UL
#define MPU6050_CALIB_FLASH_ADDR FLASH_MPU_START_ADDR

static uint32_t IMU_CalibrationChecksum(const MPU6050_CalibrationData* data) {
  return data->magic ^ data->version ^ (uint32_t)data->gyro_offset_x ^
         (uint32_t)data->gyro_offset_y ^ (uint32_t)data->gyro_offset_z ^
         (uint32_t)data->accel_offset_x ^ (uint32_t)data->accel_offset_y ^
         (uint32_t)data->accel_offset_z ^ 0xA5A5A5A5UL;
}

static bool IMU_LoadCalibration(MPU6050_Handle* handle) {
  MPU6050_CalibrationData data;
  Flash_ReadData(MPU6050_CALIB_FLASH_ADDR, &data, sizeof(data));

  if (data.magic != MPU6050_CALIB_MAGIC ||
      data.version != MPU6050_CALIB_VERSION ||
      data.checksum != IMU_CalibrationChecksum(&data)) {
    return false;
  }

  handle->gyro_offset_x = data.gyro_offset_x;
  handle->gyro_offset_y = data.gyro_offset_y;
  handle->gyro_offset_z = data.gyro_offset_z;
  handle->accel_offset_x = data.accel_offset_x;
  handle->accel_offset_y = data.accel_offset_y;
  handle->accel_offset_z = data.accel_offset_z;
  return true;
}

static bool IMU_SaveCalibration(const MPU6050_Handle* handle) {
  MPU6050_CalibrationData data = {
      .magic = MPU6050_CALIB_MAGIC,
      .version = MPU6050_CALIB_VERSION,
      .gyro_offset_x = handle->gyro_offset_x,
      .gyro_offset_y = handle->gyro_offset_y,
      .gyro_offset_z = handle->gyro_offset_z,
      .accel_offset_x = handle->accel_offset_x,
      .accel_offset_y = handle->accel_offset_y,
      .accel_offset_z = handle->accel_offset_z,
  };
  data.checksum = IMU_CalibrationChecksum(&data);

  return Flash_WriteDataToSector(MPU6050_CALIB_FLASH_ADDR,
                                 FLASH_MPU_SECTOR,
                                 FLASH_MPU_VOLTAGE_RANGE,
                                 &data,
                                 sizeof(data)) == HAL_OK;
}

void IMU_Manager_Init(ImuManager* imu, I2C_HandleTypeDef* i2c_handle,
                      bool calibrate_on_boot) {
  if (!imu) {
    return;
  }

  memset(imu, 0, sizeof(*imu));

  printf("Initializing MPU6050...\n");
  printf("[DEBUG] I2C1 handle State before init: %d (READY=%d)\n",
         i2c_handle->State, HAL_I2C_STATE_READY);

  if (MPU6050_Init(&imu->handle, i2c_handle, MPU6050_I2C_ADDR_DEFAULT)) {
    imu->initialized = true;
    printf("MPU6050 initialized successfully\n");

    if (calibrate_on_boot) {
      printf("Button2 held at boot - Starting MPU6050 calibration\n");
      if (MPU6050_Calibrate(&imu->handle, 500)) {
        if (IMU_SaveCalibration(&imu->handle)) {
          printf("Calibration saved to flash\n");
        } else {
          printf("Calibration flash save failed\n");
        }
      } else {
        printf("Calibration failed!\n");
      }
    } else if (IMU_LoadCalibration(&imu->handle)) {
      printf("[MPU6050] Calibration loaded from flash\n");
    } else {
      printf("[MPU6050] No valid calibration in flash; using zero offsets\n");
    }

    if (MPU6050_PrimeOrientation(&imu->handle)) {
      printf("[MPU6050] Initial orientation primed from accel\n");
    }
  } else {
    printf("Failed to initialize MPU6050\n");
  }
}

bool IMU_Manager_Update(ImuManager* imu) {
  if (!imu || !imu->initialized) {
    return false;
  }
  return MPU6050_Update(&imu->handle, &imu->data);
}

const MPU6050_Data* IMU_Manager_GetData(const ImuManager* imu) {
  if (!imu) {
    return NULL;
  }
  return &imu->data;
}