#include "mpu6050.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "timer.h"

// レジスタアドレス
#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_INT_ENABLE 0x38
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_WHO_AM_I 0x75

// I2C 通信パラメータ
#define MPU6050_I2C_TIMEOUT_MS 100
#define MPU6050_I2C_RETRY_COUNT 2

// サンプリング設定
#define MPU6050_SAMPLE_RATE_HZ 200

// Mahony AHRS デフォルトゲイン
#define MPU6050_TWO_KP_DEFAULT (2.0f * 0.5f)
#define MPU6050_TWO_KI_DEFAULT 0.0f

#define DEG_PER_RAD 57.2957795f
#define RAD_PER_DEG 0.0174532925f

// 取付方向の軸符号変換 (raw -> body frame)
// 現在の機体は MPU6050 を Z軸周りに180°回転させてマウント (チップX軸が機体後方、
// Y軸が機体右方向を向く) しているため X/Y 軸の符号を反転して body frame
// "X forward, Y left, Z up" に揃える。Z 軸はそのまま (チップZ = 機体Z 上向き)。
// 取付方向を変える場合はここを変更すること。
//   全軸 +1 : チップ通常取付 (X前, Y左, Z上)
//   X=-1, Y=-1, Z=+1 : Z軸まわり180°回転 (X後, Y右, Z上)
//   X=+1, Y=-1, Z=-1 : X軸まわり180°回転 (基板表面が下向き)
//   X=-1, Y=+1, Z=-1 : Y軸まわり180°回転
#ifndef MPU6050_AXIS_X_SIGN
#define MPU6050_AXIS_X_SIGN (-1.0f)
#endif
#ifndef MPU6050_AXIS_Y_SIGN
#define MPU6050_AXIS_Y_SIGN (-1.0f)
#endif
#ifndef MPU6050_AXIS_Z_SIGN
#define MPU6050_AXIS_Z_SIGN (1.0f)
#endif

// バイト結合 (ビッグエンディアン -> int16)
static int16_t CombineBytes(uint8_t high, uint8_t low) {
  return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

// I2C 読み書きヘルパ (リトライ付き)
static HAL_StatusTypeDef I2CWrite(MPU6050* mpu, uint8_t reg, uint8_t value) {
  HAL_StatusTypeDef status = HAL_ERROR;
  for (int i = 0; i < MPU6050_I2C_RETRY_COUNT; i++) {
    status = HAL_I2C_Mem_Write(mpu->i2c, mpu->i2c_addr << 1, reg,
                               I2C_MEMADD_SIZE_8BIT, &value, 1,
                               MPU6050_I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
      return HAL_OK;
    }
  }
  return status;
}

static HAL_StatusTypeDef I2CRead(MPU6050* mpu, uint8_t reg, uint8_t* buffer,
                                 uint16_t len) {
  HAL_StatusTypeDef status = HAL_ERROR;
  for (int i = 0; i < MPU6050_I2C_RETRY_COUNT; i++) {
    status = HAL_I2C_Mem_Read(mpu->i2c, mpu->i2c_addr << 1, reg,
                              I2C_MEMADD_SIZE_8BIT, buffer, len,
                              MPU6050_I2C_TIMEOUT_MS);
    if (status == HAL_OK) {
      return HAL_OK;
    }
  }
  return status;
}

// 加速度センサ値から quaternion を初期化 (重力方向から roll/pitch を求める)
static void SetQuaternionFromAccel(MPU6050* mpu, float ax, float ay, float az) {
  float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm <= 1e-6f) {
    mpu->q0 = 1.0f;
    mpu->q1 = 0.0f;
    mpu->q2 = 0.0f;
    mpu->q3 = 0.0f;
    return;
  }
  ax /= norm;
  ay /= norm;
  az /= norm;

  float roll = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

  float cr = cosf(roll * 0.5f);
  float sr = sinf(roll * 0.5f);
  float cp = cosf(pitch * 0.5f);
  float sp = sinf(pitch * 0.5f);

  // yaw = 0 として初期化
  mpu->q0 = cr * cp;
  mpu->q1 = sr * cp;
  mpu->q2 = cr * sp;
  mpu->q3 = -sr * sp;
}

// Mahony AHRS による姿勢更新 (6軸: gyro + accel)
static void UpdateAhrs(MPU6050* mpu, float gx_dps, float gy_dps, float gz_dps,
                       float ax, float ay, float az, float dt) {
  float q0 = mpu->q0;
  float q1 = mpu->q1;
  float q2 = mpu->q2;
  float q3 = mpu->q3;

  float gx = gx_dps * RAD_PER_DEG;
  float gy = gy_dps * RAD_PER_DEG;
  float gz = gz_dps * RAD_PER_DEG;

  // 加速度がゼロでなければフィードバック計算
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    float recip_norm = 1.0f / sqrtf(ax * ax + ay * ay + az * az);
    ax *= recip_norm;
    ay *= recip_norm;
    az *= recip_norm;

    // 推定重力方向
    float halfvx = q1 * q3 - q0 * q2;
    float halfvy = q0 * q1 + q2 * q3;
    float halfvz = q0 * q0 - 0.5f + q3 * q3;

    // 推定 - 観測の外積をエラーとする
    float halfex = (ay * halfvz - az * halfvy);
    float halfey = (az * halfvx - ax * halfvz);
    float halfez = (ax * halfvy - ay * halfvx);

    if (mpu->two_ki > 0.0f) {
      mpu->integral_fbx += mpu->two_ki * halfex * dt;
      mpu->integral_fby += mpu->two_ki * halfey * dt;
      mpu->integral_fbz += mpu->two_ki * halfez * dt;
      gx += mpu->integral_fbx;
      gy += mpu->integral_fby;
      gz += mpu->integral_fbz;
    } else {
      mpu->integral_fbx = 0.0f;
      mpu->integral_fby = 0.0f;
      mpu->integral_fbz = 0.0f;
    }

    gx += mpu->two_kp * halfex;
    gy += mpu->two_kp * halfey;
    gz += mpu->two_kp * halfez;
  }

  // quaternion 積分
  gx *= 0.5f * dt;
  gy *= 0.5f * dt;
  gz *= 0.5f * dt;
  float qa = q0;
  float qb = q1;
  float qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // 正規化
  float qnorm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  if (qnorm > 1e-6f) {
    q0 /= qnorm;
    q1 /= qnorm;
    q2 /= qnorm;
    q3 /= qnorm;
  }

  mpu->q0 = q0;
  mpu->q1 = q1;
  mpu->q2 = q2;
  mpu->q3 = q3;

  // quaternion -> オイラー角 (deg)
  float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
  float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
  mpu->data.roll = atan2f(sinr_cosp, cosr_cosp) * DEG_PER_RAD;

  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (sinp > 1.0f) sinp = 1.0f;
  if (sinp < -1.0f) sinp = -1.0f;
  mpu->data.pitch = asinf(sinp) * DEG_PER_RAD;

  float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
  float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
  mpu->data.yaw = atan2f(siny_cosp, cosy_cosp) * DEG_PER_RAD;
}

// WHO_AM_I を読んで MPU6050 が応答するか確認
static bool CheckConnection(MPU6050* mpu) {
  uint8_t who_am_i = 0;
  for (int i = 0; i < 3; i++) {
    if (I2CRead(mpu, MPU6050_REG_WHO_AM_I, &who_am_i, 1) == HAL_OK) {
      if (who_am_i == 0x68) {
        return true;
      }
    }
    HAL_Delay(5);
  }
  return false;
}

bool MPU6050_Init(MPU6050* mpu, I2C_HandleTypeDef* i2c, uint8_t i2c_addr) {
  if (!mpu || !i2c) {
    return false;
  }

  memset(mpu, 0, sizeof(*mpu));
  mpu->i2c = i2c;
  mpu->i2c_addr = i2c_addr;
  mpu->two_kp = MPU6050_TWO_KP_DEFAULT;
  mpu->two_ki = MPU6050_TWO_KI_DEFAULT;
  mpu->q0 = 1.0f;

  if (!CheckConnection(mpu)) {
    // 代替アドレスで再試行
    if (i2c_addr == MPU6050_I2C_ADDR_DEFAULT) {
      mpu->i2c_addr = MPU6050_I2C_ADDR_ALT;
      if (!CheckConnection(mpu)) {
        printf("[MPU6050] Not found on I2C bus\n");
        return false;
      }
    } else {
      printf("[MPU6050] Not found on I2C bus\n");
      return false;
    }
  }

  // デバイスリセット
  if (I2CWrite(mpu, MPU6050_REG_PWR_MGMT_1, 0x80) != HAL_OK) {
    return false;
  }
  HAL_Delay(100);

  // ウェイクアップ + クロック=Gyro X
  if (I2CWrite(mpu, MPU6050_REG_PWR_MGMT_1, 0x01) != HAL_OK) {
    return false;
  }

  // サンプルレート設定 (DLPF有効時の内部レートは1kHz、200Hzなら DIV=4)
  uint8_t smplrt_div = (uint8_t)(1000 / MPU6050_SAMPLE_RATE_HZ - 1);
  if (I2CWrite(mpu, MPU6050_REG_SMPLRT_DIV, smplrt_div) != HAL_OK) {
    return false;
  }

  // ジャイロ ±250 deg/s
  if (I2CWrite(mpu, MPU6050_REG_GYRO_CONFIG, 0x00) != HAL_OK) {
    return false;
  }

  // 加速度 ±2g
  if (I2CWrite(mpu, MPU6050_REG_ACCEL_CONFIG, 0x00) != HAL_OK) {
    return false;
  }

  // DLPF: 帯域 ~21Hz (CONFIG=0x04)
  if (I2CWrite(mpu, MPU6050_REG_CONFIG, 0x04) != HAL_OK) {
    return false;
  }

  // 割り込みは未使用 (ポーリング/非同期I2Cで取得)
  if (I2CWrite(mpu, MPU6050_REG_INT_ENABLE, 0x00) != HAL_OK) {
    return false;
  }

  mpu->initialized = true;
  printf("[MPU6050] Initialized at 0x%02X\n", mpu->i2c_addr);
  return true;
}

void MPU6050_SetCalibration(MPU6050* mpu, const MPU6050_Calibration* calib) {
  if (!mpu || !calib) {
    return;
  }
  mpu->calib = *calib;
}

bool MPU6050_Calibrate(MPU6050* mpu, uint16_t sample_count) {
  if (!mpu || !mpu->initialized || sample_count == 0) {
    return false;
  }

  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
  uint8_t buffer[14];

  printf("[MPU6050] Calibrating (samples=%u)...\n", sample_count);

  for (uint16_t i = 0; i < sample_count; i++) {
    if (I2CRead(mpu, MPU6050_REG_ACCEL_XOUT_H, buffer, 14) != HAL_OK) {
      printf("[MPU6050] Calibration read failed at %u\n", i);
      return false;
    }

    int16_t ax = CombineBytes(buffer[0], buffer[1]);
    int16_t ay = CombineBytes(buffer[2], buffer[3]);
    int16_t az = CombineBytes(buffer[4], buffer[5]);
    int16_t gx = CombineBytes(buffer[8], buffer[9]);
    int16_t gy = CombineBytes(buffer[10], buffer[11]);
    int16_t gz = CombineBytes(buffer[12], buffer[13]);

    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    gy_sum += gy;
    gz_sum += gz;

    HAL_Delay(5);
  }

  mpu->calib.gyro_offset_x = (int16_t)(gx_sum / sample_count);
  mpu->calib.gyro_offset_y = (int16_t)(gy_sum / sample_count);
  mpu->calib.gyro_offset_z = (int16_t)(gz_sum / sample_count);

  // 加速度オフセット: Z軸の重力方向を平均値の符号から自動判別 (取付方向に依存しない)。
  int16_t az_avg = (int16_t)(az_sum / sample_count);
  int16_t expected_gz =
      (az_avg >= 0) ? (int16_t)MPU6050_ACCEL_LSB_PER_G
                    : -(int16_t)MPU6050_ACCEL_LSB_PER_G;
  mpu->calib.accel_offset_x = (int16_t)(ax_sum / sample_count);
  mpu->calib.accel_offset_y = (int16_t)(ay_sum / sample_count);
  mpu->calib.accel_offset_z = az_avg - expected_gz;

  printf("[MPU6050] Gyro offsets: %d,%d,%d\n", mpu->calib.gyro_offset_x,
         mpu->calib.gyro_offset_y, mpu->calib.gyro_offset_z);
  printf("[MPU6050] Accel offsets: %d,%d,%d\n", mpu->calib.accel_offset_x,
         mpu->calib.accel_offset_y, mpu->calib.accel_offset_z);

  return true;
}

bool MPU6050_PrimeOrientation(MPU6050* mpu) {
  if (!mpu || !mpu->initialized) {
    return false;
  }
  uint8_t buffer[14];
  if (I2CRead(mpu, MPU6050_REG_ACCEL_XOUT_H, buffer, 14) != HAL_OK) {
    return false;
  }

  int16_t ax_raw = CombineBytes(buffer[0], buffer[1]);
  int16_t ay_raw = CombineBytes(buffer[2], buffer[3]);
  int16_t az_raw = CombineBytes(buffer[4], buffer[5]);

  float ax = (float)(ax_raw - mpu->calib.accel_offset_x) * MPU6050_AXIS_X_SIGN;
  float ay = (float)(ay_raw - mpu->calib.accel_offset_y) * MPU6050_AXIS_Y_SIGN;
  float az = (float)(az_raw - mpu->calib.accel_offset_z) * MPU6050_AXIS_Z_SIGN;

  SetQuaternionFromAccel(mpu, ax, ay, az);
  return true;
}

bool MPU6050_StartAsyncRead(MPU6050* mpu) {
  if (!mpu || !mpu->initialized) {
    return false;
  }
  if (mpu->async_active) {
    return false;
  }

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read_IT(
      mpu->i2c, mpu->i2c_addr << 1, MPU6050_REG_ACCEL_XOUT_H,
      I2C_MEMADD_SIZE_8BIT, mpu->rx_buffer, 14);

  if (status == HAL_OK) {
    mpu->async_active = 1;
    mpu->rx_ready = 0;
    return true;
  }
  return false;
}

void MPU6050_OnI2CRxComplete(MPU6050* mpu, I2C_HandleTypeDef* hi2c) {
  if (!mpu || !mpu->initialized) {
    return;
  }
  if (mpu->i2c == hi2c && mpu->async_active) {
    mpu->rx_ready = 1;
    mpu->async_active = 0;
  }
}

bool MPU6050_Update(MPU6050* mpu) {
  if (!mpu || !mpu->initialized) {
    return false;
  }

  uint8_t buffer[14];
  if (mpu->rx_ready) {
    memcpy(buffer, mpu->rx_buffer, 14);
    mpu->rx_ready = 0;
    MPU6050_StartAsyncRead(mpu);
  } else {
    if (!mpu->async_active) {
      MPU6050_StartAsyncRead(mpu);
    }
    return false;
  }

  // 生値抽出
  int16_t ax_raw = CombineBytes(buffer[0], buffer[1]);
  int16_t ay_raw = CombineBytes(buffer[2], buffer[3]);
  int16_t az_raw = CombineBytes(buffer[4], buffer[5]);
  int16_t temp_raw = CombineBytes(buffer[6], buffer[7]);
  int16_t gx_raw = CombineBytes(buffer[8], buffer[9]);
  int16_t gy_raw = CombineBytes(buffer[10], buffer[11]);
  int16_t gz_raw = CombineBytes(buffer[12], buffer[13]);

  // オフセット補正 + 取付方向の軸符号変換 (raw -> body frame)
  float ax = (float)(ax_raw - mpu->calib.accel_offset_x) * MPU6050_AXIS_X_SIGN;
  float ay = (float)(ay_raw - mpu->calib.accel_offset_y) * MPU6050_AXIS_Y_SIGN;
  float az = (float)(az_raw - mpu->calib.accel_offset_z) * MPU6050_AXIS_Z_SIGN;
  float gx = (float)(gx_raw - mpu->calib.gyro_offset_x) * MPU6050_AXIS_X_SIGN;
  float gy = (float)(gy_raw - mpu->calib.gyro_offset_y) * MPU6050_AXIS_Y_SIGN;
  float gz = (float)(gz_raw - mpu->calib.gyro_offset_z) * MPU6050_AXIS_Z_SIGN;

  // 物理量へ変換
  const float accel_scale = MPU6050_GRAVITY_MPS2 / MPU6050_ACCEL_LSB_PER_G;
  const float gyro_scale = 1.0f / MPU6050_GYRO_LSB_PER_DPS;

  mpu->data.accel_x = ax * accel_scale;
  mpu->data.accel_y = ay * accel_scale;
  mpu->data.accel_z = az * accel_scale;
  mpu->data.gyro_x = gx * gyro_scale;
  mpu->data.gyro_y = gy * gyro_scale;
  mpu->data.gyro_z = gz * gyro_scale;
  mpu->data.temp = 36.53f + (float)temp_raw / 340.0f;

  // dt 計算 (DWT サイクルカウンタを使う Timer.h と同じ手法)
  uint32_t now_us = (uint32_t)(((float)DWT->CYCCNT / HAL_RCC_GetSysClockFreq()) * 1000000.0f);
  float dt = 0.0f;
  if (mpu->prev_update_us != 0) {
    uint32_t delta = now_us - mpu->prev_update_us;
    dt = (float)delta * 1e-6f;
  }
  mpu->prev_update_us = now_us;

  // 角加速度 = (今回ジャイロ - 前回ジャイロ) / dt
  if (mpu->has_prev_gyro && dt > 1e-6f && dt < 1.0f) {
    mpu->data.angular_accel_x = (mpu->data.gyro_x - mpu->prev_gyro_x) / dt;
    mpu->data.angular_accel_y = (mpu->data.gyro_y - mpu->prev_gyro_y) / dt;
    mpu->data.angular_accel_z = (mpu->data.gyro_z - mpu->prev_gyro_z) / dt;
  } else {
    mpu->data.angular_accel_x = 0.0f;
    mpu->data.angular_accel_y = 0.0f;
    mpu->data.angular_accel_z = 0.0f;
  }
  mpu->prev_gyro_x = mpu->data.gyro_x;
  mpu->prev_gyro_y = mpu->data.gyro_y;
  mpu->prev_gyro_z = mpu->data.gyro_z;
  mpu->has_prev_gyro = true;

  // 姿勢更新 (Mahony AHRS)
  if (dt > 1e-6f && dt < 1.0f) {
    UpdateAhrs(mpu, mpu->data.gyro_x, mpu->data.gyro_y, mpu->data.gyro_z,
               mpu->data.accel_x, mpu->data.accel_y, mpu->data.accel_z, dt);
  }

  return true;
}

const MPU6050_Data* MPU6050_GetData(const MPU6050* mpu) {
  if (!mpu) {
    return NULL;
  }
  return &mpu->data;
}
