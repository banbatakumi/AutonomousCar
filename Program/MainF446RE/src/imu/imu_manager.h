#ifndef IMU_MANAGER_H_
#define IMU_MANAGER_H_

#include <stdbool.h>

#include "i2c.h"
#include "mpu6050_dmp.h"

typedef struct {
  MPU6050_Handle handle;
  MPU6050_Data data;
  bool initialized;
} ImuManager;

void IMU_Manager_Init(ImuManager* imu, I2C_HandleTypeDef* i2c_handle,
                      bool calibrate_on_boot);
bool IMU_Manager_Update(ImuManager* imu);
const MPU6050_Data* IMU_Manager_GetData(const ImuManager* imu);

#endif  // IMU_MANAGER_H_