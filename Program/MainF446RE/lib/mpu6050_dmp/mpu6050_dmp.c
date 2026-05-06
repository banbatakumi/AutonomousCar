#include "mpu6050_dmp.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define MPU6050_I2C_TIMEOUT 2000  // Increased from 1000ms for reliability
#define MPU6050_DEG_PER_RAD (57.2957795f)
#define MPU6050_RAD_PER_DEG (0.0174532925f)
#define MPU6050_I2C_RETRY_COUNT 2
#define MPU6050_TWO_KP_DEF (2.0f * 0.5f)
#define MPU6050_TWO_KI_DEF (0.0f)

// Keep the raw MPU6050 body frame until the mounting orientation is verified.
#define MPU6050_AXIS_X_SIGN (1.0f)
#define MPU6050_AXIS_Y_SIGN (1.0f)
#define MPU6050_AXIS_Z_SIGN (1.0f)
#define MPU6050_GYRO_X_SIGN (1.0f)
#define MPU6050_GYRO_Y_SIGN (1.0f)
#define MPU6050_GYRO_Z_SIGN (1.0f)

static void MPU6050_LogI2CStatus(I2C_HandleTypeDef* i2c_handle, const char* tag) {
  printf("[MPU6050] %s: state=%d mode=%d err=0x%08lX\n", tag,
         (int)i2c_handle->State,
         (int)i2c_handle->Mode,
         (unsigned long)i2c_handle->ErrorCode);
}

/* Helper function to forcefully reset I2C bus */
static void MPU6050_ForceResetI2C(I2C_HandleTypeDef* i2c_handle) {
  printf("[MPU6050] Recovering I2C bus...\n");
  MPU6050_LogI2CStatus(i2c_handle, "before recover");

  // Step 1: Disable and deinit I2C
  __HAL_I2C_DISABLE(i2c_handle);
  HAL_I2C_DeInit(i2c_handle);

  // Step 2: Force peripheral reset
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_I2C1_FORCE_RESET();
  HAL_Delay(2);
  __HAL_RCC_I2C1_RELEASE_RESET();

  // Step 3: Bus clear on PB8/PB9 (SCL/SDA)
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  for (int i = 0; i < 16 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  // STOP condition
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  HAL_Delay(1);

  // Step 4: Restore I2C alternate function
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Step 5: Reinitialize I2C peripheral
  if (HAL_I2C_Init(i2c_handle) != HAL_OK) {
    printf("[MPU6050] ERROR: HAL_I2C_Init failed!\n");
  }

  MPU6050_LogI2CStatus(i2c_handle, "after recover");
}

static HAL_StatusTypeDef MPU6050_MemWriteWithRetry(I2C_HandleTypeDef* i2c_handle,
                                                   uint16_t dev_addr,
                                                   uint16_t reg,
                                                   uint8_t* data,
                                                   uint16_t len,
                                                   uint32_t timeout) {
  HAL_StatusTypeDef status = HAL_ERROR;

  for (int retry = 0; retry < MPU6050_I2C_RETRY_COUNT; retry++) {
    status = HAL_I2C_Mem_Write(i2c_handle, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout);
    if (status == HAL_OK) {
      return HAL_OK;
    }
    if (status == HAL_BUSY || status == HAL_TIMEOUT || status == HAL_ERROR) {
      MPU6050_ForceResetI2C(i2c_handle);
    }
  }

  return status;
}

static HAL_StatusTypeDef MPU6050_MemReadWithRetry(I2C_HandleTypeDef* i2c_handle,
                                                  uint16_t dev_addr,
                                                  uint16_t reg,
                                                  uint8_t* data,
                                                  uint16_t len,
                                                  uint32_t timeout) {
  HAL_StatusTypeDef status = HAL_ERROR;

  for (int retry = 0; retry < MPU6050_I2C_RETRY_COUNT; retry++) {
    status = HAL_I2C_Mem_Read(i2c_handle, dev_addr, reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout);
    if (status == HAL_OK) {
      return HAL_OK;
    }
    if (status == HAL_BUSY || status == HAL_TIMEOUT || status == HAL_ERROR) {
      MPU6050_ForceResetI2C(i2c_handle);
    }
  }

  return status;
}

/* Helper function to write a byte to MPU6050 */
static bool MPU6050_WriteReg(MPU6050_Handle* handle, uint8_t reg, uint8_t value) {
  HAL_StatusTypeDef status = MPU6050_MemWriteWithRetry(handle->i2c_handle, handle->i2c_addr << 1,
                                                       reg, &value, 1, MPU6050_I2C_TIMEOUT);
  if (status != HAL_OK) {
    printf("[MPU6050] I2C write error: %d err=0x%08lX\n", status, (unsigned long)handle->i2c_handle->ErrorCode);
    return false;
  }
  return true;
}

/* Helper function to read a byte from MPU6050 */
static bool MPU6050_ReadReg(MPU6050_Handle* handle, uint8_t reg, uint8_t* value) {
  HAL_StatusTypeDef status = MPU6050_MemReadWithRetry(handle->i2c_handle, handle->i2c_addr << 1,
                                                      reg, value, 1, MPU6050_I2C_TIMEOUT);
  if (status != HAL_OK) {
    printf("[MPU6050] I2C read error: %d err=0x%08lX\n", status, (unsigned long)handle->i2c_handle->ErrorCode);
    return false;
  }
  return true;
}

/* Helper function to read multiple bytes from MPU6050 */
static bool MPU6050_ReadRegs(MPU6050_Handle* handle, uint8_t reg, uint8_t* buffer, uint8_t len) {
  HAL_StatusTypeDef status = MPU6050_MemReadWithRetry(handle->i2c_handle, handle->i2c_addr << 1,
                                                      reg, buffer, len, MPU6050_I2C_TIMEOUT);
  return status == HAL_OK;
}

/* Combine two bytes into a 16-bit signed integer */
static int16_t MPU6050_CombineBytes(uint8_t high, uint8_t low) {
  return (int16_t)((high << 8) | low);
}

static bool MPU6050_SetQuaternionFromAccel(MPU6050_Handle* handle, float ax, float ay, float az) {
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm <= 1e-6f) {
    return false;
  }

  ax /= norm;
  ay /= norm;
  az /= norm;

  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

  float half_roll = 0.5f * roll;
  float half_pitch = 0.5f * pitch;
  float cr = cosf(half_roll);
  float sr = sinf(half_roll);
  float cp = cosf(half_pitch);
  float sp = sinf(half_pitch);

  handle->q0 = cr * cp;
  handle->q1 = sr * cp;
  handle->q2 = cr * sp;
  handle->q3 = -sr * sp;
  handle->yaw = 0.0f;
  handle->pitch = pitch * MPU6050_DEG_PER_RAD;
  handle->roll = roll * MPU6050_DEG_PER_RAD;
  return true;
}

static void MPU6050_ApplyBodyFrameTransform(float* ax, float* ay, float* az,
                                            float* gx, float* gy, float* gz) {
  if (ax) *ax *= MPU6050_AXIS_X_SIGN;
  if (ay) *ay *= MPU6050_AXIS_Y_SIGN;
  if (az) *az *= MPU6050_AXIS_Z_SIGN;
  if (gx) *gx *= MPU6050_GYRO_X_SIGN;
  if (gy) *gy *= MPU6050_GYRO_Y_SIGN;
  if (gz) *gz *= MPU6050_GYRO_Z_SIGN;
}

/* Mahony AHRS IMU update (6-axis: gyro + accel) */
static void MPU6050_UpdateAngles(MPU6050_Handle* handle, float gx_dps, float gy_dps, float gz_dps,
                                 float ax, float ay, float az, float dt) {
  float q0 = handle->q0;
  float q1 = handle->q1;
  float q2 = handle->q2;
  float q3 = handle->q3;

  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  // Convert gyro from deg/s to rad/s
  float gx = gx_dps * MPU6050_RAD_PER_DEG;
  float gy = gy_dps * MPU6050_RAD_PER_DEG;
  float gz = gz_dps * MPU6050_RAD_PER_DEG;

  // Compute feedback only if accelerometer measurement valid
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    float recipNorm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity (Mahony)
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is cross product between estimated and measured gravity direction
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (handle->twoKi > 0.0f) {
      handle->integralFBx += handle->twoKi * halfex * dt;
      handle->integralFBy += handle->twoKi * halfey * dt;
      handle->integralFBz += handle->twoKi * halfez * dt;
      gx += handle->integralFBx;
      gy += handle->integralFBy;
      gz += handle->integralFBz;
    } else {
      handle->integralFBx = 0.0f;
      handle->integralFBy = 0.0f;
      handle->integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += handle->twoKp * halfex;
    gy += handle->twoKp * halfey;
    gz += handle->twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalize quaternion
  float qnorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (qnorm > 1e-6f) {
    q0 /= qnorm;
    q1 /= qnorm;
    q2 /= qnorm;
    q3 /= qnorm;
  }

  handle->q0 = q0;
  handle->q1 = q1;
  handle->q2 = q2;
  handle->q3 = q3;

  // Convert quaternion to Euler angles (degrees)
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  handle->roll = atan2f(sinr_cosp, cosr_cosp) * MPU6050_DEG_PER_RAD;

  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  handle->pitch = asinf(sinp) * MPU6050_DEG_PER_RAD;

  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  handle->yaw = atan2f(siny_cosp, cosy_cosp) * MPU6050_DEG_PER_RAD;
}

/**
 * @brief Check if MPU6050 is connected
 */
bool MPU6050_IsConnected(MPU6050_Handle* handle) {
  uint8_t who_am_i;
  printf("[MPU6050] Checking connection at I2C address 0x%02X...\n", handle->i2c_addr);

  // Try reading WHO_AM_I multiple times
  for (int retry = 0; retry < 3; retry++) {
    if (MPU6050_ReadReg(handle, MPU6050_REG_WHO_AM_I, &who_am_i)) {
      printf("[MPU6050] WHO_AM_I = 0x%02X (expected 0x68)\n", who_am_i);
      if (who_am_i == 0x68) {
        return true;
      }
    }
    HAL_Delay(10);
  }

  printf("[MPU6050] I2C read failed or invalid response!\n");
  return false;
}

/**
 * @brief Initialize MPU6050
 */
bool MPU6050_Init(MPU6050_Handle* handle, I2C_HandleTypeDef* i2c_handle, uint8_t i2c_addr) {
  if (!handle || !i2c_handle) {
    printf("[MPU6050] Invalid handle or I2C pointer\n");
    return false;
  }

  memset(handle, 0, sizeof(MPU6050_Handle));
  handle->i2c_handle = i2c_handle;
  handle->i2c_addr = i2c_addr;
  handle->twoKp = MPU6050_TWO_KP_DEF;
  handle->twoKi = MPU6050_TWO_KI_DEF;

  // Initialize quaternion to identity
  handle->q0 = 1.0f;
  handle->q1 = 0.0f;
  handle->q2 = 0.0f;
  handle->q3 = 0.0f;

  // Initialize Euler angles
  handle->yaw = 0.0f;
  handle->pitch = 0.0f;
  handle->roll = 0.0f;

  // Check connection
  printf("[MPU6050] Step 1: Checking connection...\n");
  if (!MPU6050_IsConnected(handle)) {
    printf("[MPU6050] Connection check failed at address 0x%02X\n", i2c_addr);

    // Try alternate address
    if (i2c_addr == MPU6050_I2C_ADDR_DEFAULT) {
      printf("[MPU6050] Trying alternate address 0x69...\n");
      handle->i2c_addr = MPU6050_I2C_ADDR_ALT;
      if (MPU6050_IsConnected(handle)) {
        printf("[MPU6050] Found MPU6050 at 0x69!\n");
      } else {
        printf("[MPU6050] Not found at alternate address either. Is the sensor connected?\n");
        return false;
      }
    } else {
      printf("[MPU6050] Connection check failed! Is the sensor connected?\n");
      return false;
    }
  }

  // Reset device
  printf("[MPU6050] Step 2: Resetting device...\n");
  if (!MPU6050_WriteReg(handle, MPU6050_REG_PWR_MGMT_1, 0x80)) {
    printf("[MPU6050] Reset write failed!\n");
    return false;
  }
  HAL_Delay(100);

  // Wake up and set clock source to gyro X
  printf("[MPU6050] Step 3: Waking up device...\n");
  if (!MPU6050_WriteReg(handle, MPU6050_REG_PWR_MGMT_1, 0x01)) {
    printf("[MPU6050] Wakeup write failed!\n");
    return false;
  }

  // Set sample rate divider for 100 Hz
  printf("[MPU6050] Step 4: Setting sample rate...\n");
  uint8_t sample_rate_div = 80 / MPU6050_DMP_SAMPLE_RATE - 1;  // 80 MHz / 100 Hz = 8
  if (!MPU6050_WriteReg(handle, MPU6050_REG_SMPLRT_DIV, sample_rate_div)) {
    printf("[MPU6050] Sample rate write failed!\n");
    return false;
  }

  // Configure gyro range (±250 deg/s)
  printf("[MPU6050] Step 5: Configuring gyro...\n");
  if (!MPU6050_WriteReg(handle, MPU6050_REG_GYRO_CONFIG, 0x00)) {
    printf("[MPU6050] Gyro config write failed!\n");
    return false;
  }

  // Configure accel range (±2g)
  printf("[MPU6050] Step 6: Configuring accel...\n");
  if (!MPU6050_WriteReg(handle, MPU6050_REG_ACCEL_CONFIG, 0x00)) {
    printf("[MPU6050] Accel config write failed!\n");
    return false;
  }

  // Set low-pass filter to 20 Hz
  printf("[MPU6050] Step 7: Setting low-pass filter...\n");
  if (!MPU6050_WriteReg(handle, MPU6050_REG_CONFIG, 0x04)) {
    printf("[MPU6050] Config write failed!\n");
    return false;
  }

  // Enable data ready interrupt
  printf("[MPU6050] Step 8: Enabling interrupt...\n");
  if (!MPU6050_WriteReg(handle, MPU6050_REG_INT_ENABLE, 0x01)) {
    printf("[MPU6050] Interrupt enable write failed!\n");
    return false;
  }

  handle->last_update_ms = HAL_GetTick();
  printf("[MPU6050] Initialization completed successfully at address 0x%02X!\n", handle->i2c_addr);

  return true;
}

/**
 * @brief Reset MPU6050
 */
bool MPU6050_Reset(MPU6050_Handle* handle) {
  if (!handle) {
    return false;
  }

  return MPU6050_WriteReg(handle, MPU6050_REG_PWR_MGMT_1, 0x80);
}

bool MPU6050_PrimeOrientation(MPU6050_Handle* handle) {
  if (!handle) {
    return false;
  }

  uint8_t buffer[14];
  if (!MPU6050_ReadRegs(handle, MPU6050_REG_ACCEL_XOUT_H, buffer, 14)) {
    return false;
  }

  int16_t ax_raw = MPU6050_CombineBytes(buffer[0], buffer[1]);
  int16_t ay_raw = MPU6050_CombineBytes(buffer[2], buffer[3]);
  int16_t az_raw = MPU6050_CombineBytes(buffer[4], buffer[5]);

  int16_t ax = ax_raw - handle->accel_offset_x;
  int16_t ay = ay_raw - handle->accel_offset_y;
  int16_t az = az_raw - handle->accel_offset_z;

  float fax = (float)ax;
  float fay = (float)ay;
  float faz = (float)az;
  MPU6050_ApplyBodyFrameTransform(&fax, &fay, &faz, NULL, NULL, NULL);

  return MPU6050_SetQuaternionFromAccel(handle, fax, fay, faz);
}

/**
 * @brief Calibrate MPU6050 (calculate gyro and accel offsets)
 */
bool MPU6050_Calibrate(MPU6050_Handle* handle, uint16_t sample_count) {
  if (!handle || sample_count == 0) {
    return false;
  }

  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
  uint8_t buffer[14];

  printf("Starting MPU6050 calibration... (samples: %d)\n", sample_count);

  for (uint16_t i = 0; i < sample_count; i++) {
    if (!MPU6050_ReadRegs(handle, MPU6050_REG_ACCEL_XOUT_H, buffer, 14)) {
      return false;
    }

    // Extract accel data
    int16_t ax = MPU6050_CombineBytes(buffer[0], buffer[1]);
    int16_t ay = MPU6050_CombineBytes(buffer[2], buffer[3]);
    int16_t az = MPU6050_CombineBytes(buffer[4], buffer[5]);

    // Extract gyro data (skip temp at buffer[6:8])
    int16_t gx = MPU6050_CombineBytes(buffer[8], buffer[9]);
    int16_t gy = MPU6050_CombineBytes(buffer[10], buffer[11]);
    int16_t gz = MPU6050_CombineBytes(buffer[12], buffer[13]);

    // Accumulate
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;

    if (i % 20 == 0) {
      printf(".");
    }
    HAL_Delay(10);
  }

  printf("\nCalibration complete\n");

  // Calculate offsets
  handle->gyro_offset_x = (int16_t)(gx_sum / sample_count);
  handle->gyro_offset_y = (int16_t)(gy_sum / sample_count);
  handle->gyro_offset_z = (int16_t)(gz_sum / sample_count);

  // For accelerometer, we need to account for gravity (assuming Z is up)
  handle->accel_offset_x = (int16_t)(ax_sum / sample_count);
  handle->accel_offset_y = (int16_t)(ay_sum / sample_count);
  handle->accel_offset_z = (int16_t)(az_sum / sample_count);

  // Adjust Z offset for gravity (1g = ~16384 LSB for ±2g range)
  handle->accel_offset_z -= 16384;

  printf("Gyro offsets: X=%d, Y=%d, Z=%d\n", handle->gyro_offset_x, handle->gyro_offset_y, handle->gyro_offset_z);
  printf("Accel offsets: X=%d, Y=%d, Z=%d\n", handle->accel_offset_x, handle->accel_offset_y, handle->accel_offset_z);

  return true;
}

/**
 * @brief Update sensor data from MPU6050
 */
bool MPU6050_Update(MPU6050_Handle* handle, MPU6050_Data* data) {
  if (!handle || !data) {
    return false;
  }

  uint8_t buffer[14];

  if (!MPU6050_ReadRegs(handle, MPU6050_REG_ACCEL_XOUT_H, buffer, 14)) {
    return false;
  }

  // Extract raw data
  int16_t ax_raw = MPU6050_CombineBytes(buffer[0], buffer[1]);
  int16_t ay_raw = MPU6050_CombineBytes(buffer[2], buffer[3]);
  int16_t az_raw = MPU6050_CombineBytes(buffer[4], buffer[5]);

  int16_t temp_raw = MPU6050_CombineBytes(buffer[6], buffer[7]);

  int16_t gx_raw = MPU6050_CombineBytes(buffer[8], buffer[9]);
  int16_t gy_raw = MPU6050_CombineBytes(buffer[10], buffer[11]);
  int16_t gz_raw = MPU6050_CombineBytes(buffer[12], buffer[13]);

  // Apply offsets
  int16_t ax = ax_raw - handle->accel_offset_x;
  int16_t ay = ay_raw - handle->accel_offset_y;
  int16_t az = az_raw - handle->accel_offset_z;

  int16_t gx = gx_raw - handle->gyro_offset_x;
  int16_t gy = gy_raw - handle->gyro_offset_y;
  int16_t gz = gz_raw - handle->gyro_offset_z;

  float fax = (float)ax;
  float fay = (float)ay;
  float faz = (float)az;
  float fgx = (float)gx;
  float fgy = (float)gy;
  float fgz = (float)gz;
  MPU6050_ApplyBodyFrameTransform(&fax, &fay, &faz, &fgx, &fgy, &fgz);

  // Convert to physical units
  // Accel: ±2g range, 16384 LSB/g
  float accel_scale = 9.81f / 16384.0f;
  data->accelx = fax * accel_scale;
  data->accely = fay * accel_scale;
  data->accelz = faz * accel_scale;

  // Gyro: ±250 deg/s range, 131 LSB/(deg/s)
  float gyro_scale = 1.0f / 131.0f;
  data->gyrox = fgx * gyro_scale;
  data->gyroy = fgy * gyro_scale;
  data->gyroz = fgz * gyro_scale;

  // Temperature: 36.53°C + ((TEMP_OUT) / 340.0)
  data->temp = 36.53f + (temp_raw / 340.0f);

  // Update Euler angles using complementary filter
  uint32_t current_ms = HAL_GetTick();
  float dt = (current_ms - handle->last_update_ms) / 1000.0f;
  handle->last_update_ms = current_ms;

  if (dt > 0.0f && dt < 1.0f) {  // Sanity check
    MPU6050_UpdateAngles(handle, data->gyrox, data->gyroy, data->gyroz,
                         data->accelx, data->accely, data->accelz, dt);
  }

  // Store Euler angles in data structure
  data->yaw = handle->yaw;
  data->pitch = handle->pitch;
  data->roll = handle->roll;

  return true;
}

/**
 * @brief Get current Euler angles
 */
void MPU6050_GetEulerAngles(MPU6050_Handle* handle, float* yaw, float* pitch, float* roll) {
  if (yaw) *yaw = handle->yaw;
  if (pitch) *pitch = handle->pitch;
  if (roll) *roll = handle->roll;
}

/**
 * @brief Get HAL I2C status codes as string for debugging
 */
const char* MPU6050_GetHALStatusString(int status) {
  switch (status) {
    case HAL_OK:
      return "HAL_OK (0)";
    case HAL_ERROR:
      return "HAL_ERROR (1)";
    case HAL_BUSY:
      return "HAL_BUSY (2)";
    case HAL_TIMEOUT:
      return "HAL_TIMEOUT (3)";
    default:
      return "UNKNOWN";
  }
}

/**
 * @brief Force reset I2C bus (public API)
 */
void MPU6050_I2CForceReset(I2C_HandleTypeDef* i2c_handle) {
  MPU6050_ForceResetI2C(i2c_handle);
}

/**
 * @brief Scan I2C bus for MPU6050 device
 */
uint8_t MPU6050_ScanI2C(I2C_HandleTypeDef* i2c_handle) {
  printf("[MPU6050] Scanning I2C bus...\n");
  MPU6050_LogI2CStatus(i2c_handle, "scan start");

  // Check common MPU6050 addresses
  uint8_t addresses[] = {MPU6050_I2C_ADDR_DEFAULT, MPU6050_I2C_ADDR_ALT};

  for (uint8_t addr_idx = 0; addr_idx < 2; addr_idx++) {
    uint8_t addr = addresses[addr_idx];
    uint8_t who_am_i = 0xFF;
    HAL_StatusTypeDef status = HAL_ERROR;

    printf("[MPU6050] Checking address 0x%02X...", addr);

    status = HAL_I2C_IsDeviceReady(i2c_handle, addr << 1, 2, 50);
    if (status == HAL_BUSY) {
      printf(" [device-ready BUSY, recovering]");
      MPU6050_ForceResetI2C(i2c_handle);
      status = HAL_I2C_IsDeviceReady(i2c_handle, addr << 1, 2, 50);
    }

    if (status != HAL_OK) {
      printf(" no-ack (%s, err=0x%08lX)\n", MPU6050_GetHALStatusString(status), (unsigned long)i2c_handle->ErrorCode);
      continue;
    }

    // Try to read WHO_AM_I register with retry logic
    for (int retry = 0; retry < 3; retry++) {
      status = HAL_I2C_Mem_Read(i2c_handle, addr << 1, MPU6050_REG_WHO_AM_I,
                                I2C_MEMADD_SIZE_8BIT, &who_am_i, 1, 500);
      if (status == HAL_OK) {
        break;
      }
      if (status == HAL_BUSY) {
        MPU6050_ForceResetI2C(i2c_handle);
      }
      printf(" [retry %d: %s]", retry, MPU6050_GetHALStatusString(status));
      HAL_Delay(10);
    }

    printf(" WHO_AM_I=0x%02X (%s)", who_am_i, MPU6050_GetHALStatusString(status));
    if (status == HAL_OK && who_am_i == 0x68) {
      printf(" [FOUND MPU6050]\n");
      return addr;
    }
    printf(" [no device]\n");
  }

  printf("[MPU6050] No MPU6050 found on I2C bus!\n");
  return 0xFF;
}
