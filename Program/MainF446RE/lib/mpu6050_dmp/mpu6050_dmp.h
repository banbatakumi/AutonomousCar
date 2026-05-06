#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include <stdbool.h>
#include <stdint.h>

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MPU6050 I2C Addresses */
#define MPU6050_I2C_ADDR_DEFAULT 0x68  // AD0 = 0
#define MPU6050_I2C_ADDR_ALT 0x69      // AD0 = 1

/* MPU6050 Register Addresses */
#define MPU6050_REG_SELF_TEST_X 0x0D
#define MPU6050_REG_SELF_TEST_Y 0x0E
#define MPU6050_REG_SELF_TEST_Z 0x0F
#define MPU6050_REG_SELF_TEST_A 0x10
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_INT_ENABLE 0x38
#define MPU6050_REG_INT_STATUS 0x3A
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_USER_CTRL 0x6A
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_PWR_MGMT_2 0x6C
#define MPU6050_REG_WHO_AM_I 0x75

/* DMP Configuration */
#define MPU6050_DMP_SAMPLE_RATE 100  // Hz
#define MPU6050_DMP_FIFO_SIZE 512

/* Sensor data structure */
typedef struct {
  float yaw;    // Yaw angle (degrees)
  float pitch;  // Pitch angle (degrees)
  float roll;   // Roll angle (degrees)

  float gyrox;  // Gyro X (degrees/sec)
  float gyroy;  // Gyro Y (degrees/sec)
  float gyroz;  // Gyro Z (degrees/sec)

  float accelx;  // Acceleration X (m/s^2)
  float accely;  // Acceleration Y (m/s^2)
  float accelz;  // Acceleration Z (m/s^2)

  float temp;  // Temperature (degrees C)
} MPU6050_Data;

/* Handle structure */
typedef struct {
  I2C_HandleTypeDef* i2c_handle;
  uint8_t i2c_addr;

  // Calibration offsets
  int16_t gyro_offset_x;
  int16_t gyro_offset_y;
  int16_t gyro_offset_z;

  int16_t accel_offset_x;
  int16_t accel_offset_y;
  int16_t accel_offset_z;

  // Quaternion for DMP (or Euler angles)
  float q0, q1, q2, q3;  // Quaternion
  float twoKp;           // Mahony proportional gain * 2
  float twoKi;           // Mahony integral gain * 2
  float integralFBx;     // Mahony integral feedback x
  float integralFBy;     // Mahony integral feedback y
  float integralFBz;     // Mahony integral feedback z
  float yaw, pitch, roll;

  // Last update time
  uint32_t last_update_ms;
  // Async I2C (IT) support
  uint8_t rx_buffer[14];
  volatile uint8_t rx_ready;       // set by I2C Rx complete callback
  volatile uint8_t async_active;   // indicates an ongoing non-blocking transfer
} MPU6050_Handle;

/* Function Prototypes */

/**
 * @brief Initialize MPU6050 with DMP mode
 * @param handle: Pointer to MPU6050_Handle
 * @param i2c_handle: Pointer to I2C_HandleTypeDef
 * @param i2c_addr: I2C address (0x68 or 0x69)
 * @return true if successful, false otherwise
 */
bool MPU6050_Init(MPU6050_Handle* handle, I2C_HandleTypeDef* i2c_handle, uint8_t i2c_addr);

/**
 * @brief Calibrate MPU6050 (calculate gyro and accel offsets)
 * @param handle: Pointer to MPU6050_Handle
 * @param sample_count: Number of samples to average (e.g., 200)
 * @return true if successful, false otherwise
 */
bool MPU6050_Calibrate(MPU6050_Handle* handle, uint16_t sample_count);

/**
 * @brief Update sensor data from MPU6050
 * @param handle: Pointer to MPU6050_Handle
 * @param data: Pointer to MPU6050_Data structure to store results
 * @return true if successful, false otherwise
 */
bool MPU6050_Update(MPU6050_Handle* handle, MPU6050_Data* data);

/**
 * @brief Get current Euler angles (yaw, pitch, roll)
 * @param handle: Pointer to MPU6050_Handle
 * @param yaw: Pointer to store yaw angle (degrees)
 * @param pitch: Pointer to store pitch angle (degrees)
 * @param roll: Pointer to store roll angle (degrees)
 */
void MPU6050_GetEulerAngles(MPU6050_Handle* handle, float* yaw, float* pitch, float* roll);

/**
 * @brief Reset MPU6050
 * @param handle: Pointer to MPU6050_Handle
 * @return true if successful, false otherwise
 */
bool MPU6050_Reset(MPU6050_Handle* handle);

/**
 * @brief Seed the attitude estimate from the current accelerometer direction
 * @param handle: Pointer to MPU6050_Handle
 * @return true if successful, false otherwise
 */
bool MPU6050_PrimeOrientation(MPU6050_Handle* handle);

/**
 * @brief Check if MPU6050 is connected
 * @param handle: Pointer to MPU6050_Handle
 * @return true if connected, false otherwise
 */
bool MPU6050_IsConnected(MPU6050_Handle* handle);

/**
 * @brief Force reset I2C bus
 * @param i2c_handle: Pointer to I2C_HandleTypeDef
 */
void MPU6050_I2CForceReset(I2C_HandleTypeDef* i2c_handle);

/**
 * @brief Scan I2C bus for MPU6050 device (debugging function)
 * @param i2c_handle: Pointer to I2C_HandleTypeDef
 * @return detected I2C address (0x68 or 0x69), or 0xFF if not found
 */
uint8_t MPU6050_ScanI2C(I2C_HandleTypeDef* i2c_handle);

/**
 * @brief Get HAL I2C status codes as string for debugging
 * @param status: HAL_StatusTypeDef value
 * @return String representation of status
 */
const char* MPU6050_GetHALStatusString(int status);

// Start a non-blocking read of sensor registers (14 bytes accel/temp/gyro)
bool MPU6050_StartAsyncRead(MPU6050_Handle* handle);

#ifdef __cplusplus
}
#endif

#endif  // MPU6050_DMP_H
