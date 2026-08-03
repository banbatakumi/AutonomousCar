#include "mpu6050.h"

#include <stdio.h>
#include <string.h>

#define MPU6050_REG_SMPLRT_DIV 0x19
#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_REG_INT_ENABLE 0x38
#define MPU6050_REG_INT_STATUS 0x3A
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_WHO_AM_I 0x75

#define MPU6050_I2C_TIMEOUT_MS 100
#define MPU6050_I2C_RETRY_COUNT 3

// PWR_MGMT_1
#define MPU6050_PWR_DEVICE_RESET 0x80
// クロック源にジャイロ X 軸の PLL を選ぶ (内蔵 RC より周波数安定度が高く、
// サンプル周期のばらつきが姿勢積分の誤差になるのを防ぐ)
#define MPU6050_PWR_CLK_PLL_XGYRO 0x01

// INT_ENABLE: DATA_RDY_EN のみ
#define MPU6050_INT_DATA_RDY_EN 0x01

// INT_PIN_CFG: アクティブHigh / プッシュプル / 50us パルス (ラッチしない)
// -> EXTI の立ち上がりエッジで受けられ、クリアのための追加 I2C アクセスが不要
#define MPU6050_INT_PIN_CFG_PULSE 0x00

static int16_t CombineBytes(uint8_t high, uint8_t low) {
  return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static HAL_StatusTypeDef WriteReg(Mpu6050 *obj, uint8_t reg, uint8_t value) {
  HAL_StatusTypeDef status = HAL_ERROR;
  for (int i = 0; i < MPU6050_I2C_RETRY_COUNT; i++) {
    status = HAL_I2C_Mem_Write(obj->i2c, (uint16_t)(obj->i2c_addr << 1), reg,
                               I2C_MEMADD_SIZE_8BIT, &value, 1,
                               MPU6050_I2C_TIMEOUT_MS);
    if (status == HAL_OK) return HAL_OK;
  }
  return status;
}

static HAL_StatusTypeDef ReadReg(Mpu6050 *obj, uint8_t reg, uint8_t *buffer,
                                 uint16_t len) {
  HAL_StatusTypeDef status = HAL_ERROR;
  for (int i = 0; i < MPU6050_I2C_RETRY_COUNT; i++) {
    status = HAL_I2C_Mem_Read(obj->i2c, (uint16_t)(obj->i2c_addr << 1), reg,
                              I2C_MEMADD_SIZE_8BIT, buffer, len,
                              MPU6050_I2C_TIMEOUT_MS);
    if (status == HAL_OK) return HAL_OK;
  }
  return status;
}

// 生バイト列 -> オフセット補正済みの物理量
static void ParseFrame(const Mpu6050 *obj, const uint8_t *frame,
                       Mpu6050Sample *out) {
  int16_t ax = CombineBytes(frame[0], frame[1]);
  int16_t ay = CombineBytes(frame[2], frame[3]);
  int16_t az = CombineBytes(frame[4], frame[5]);
  int16_t temp = CombineBytes(frame[6], frame[7]);
  int16_t gx = CombineBytes(frame[8], frame[9]);
  int16_t gy = CombineBytes(frame[10], frame[11]);
  int16_t gz = CombineBytes(frame[12], frame[13]);

  const float accel_scale = MPU6050_GRAVITY_MPS2 / MPU6050_ACCEL_LSB_PER_G;
  const float gyro_scale = 1.0f / MPU6050_GYRO_LSB_PER_DPS;

  out->accel_x = (float)(ax - obj->offset.accel_x) * accel_scale;
  out->accel_y = (float)(ay - obj->offset.accel_y) * accel_scale;
  out->accel_z = (float)(az - obj->offset.accel_z) * accel_scale;
  out->gyro_x = (float)(gx - obj->offset.gyro_x) * gyro_scale;
  out->gyro_y = (float)(gy - obj->offset.gyro_y) * gyro_scale;
  out->gyro_z = (float)(gz - obj->offset.gyro_z) * gyro_scale;
  out->temp = 36.53f + (float)temp / 340.0f;
}

// WHO_AM_I が応答するアドレスを探す
static bool FindDevice(Mpu6050 *obj, uint8_t preferred_addr) {
  const uint8_t candidates[2] = {
      preferred_addr,
      (preferred_addr == MPU6050_I2C_ADDR_LOW) ? MPU6050_I2C_ADDR_HIGH
                                               : MPU6050_I2C_ADDR_LOW,
  };
  for (int i = 0; i < 2; i++) {
    obj->i2c_addr = candidates[i];
    uint8_t who_am_i = 0;
    if (ReadReg(obj, MPU6050_REG_WHO_AM_I, &who_am_i, 1) == HAL_OK) {
      // 純正は 0x68 を返すが、互換品は別値を返すことがあるため未接続 (0x00/0xFF) 以外は許容する
      if (who_am_i != 0x00 && who_am_i != 0xFF) {
        if (who_am_i != 0x68) {
          printf("[MPU6050] Unexpected WHO_AM_I 0x%02X (clone?)\n", who_am_i);
        }
        return true;
      }
    }
  }
  return false;
}

bool Mpu6050_Init(Mpu6050 *obj, I2C_HandleTypeDef *i2c, uint8_t i2c_addr,
                  uint16_t sample_rate_hz, Mpu6050DlpfBandwidth dlpf) {
  if (obj == NULL || i2c == NULL || sample_rate_hz == 0) return false;

  memset(obj, 0, sizeof(*obj));
  obj->i2c = i2c;
  obj->sample_rate_hz = sample_rate_hz;

  if (!FindDevice(obj, i2c_addr)) {
    printf("[MPU6050] Not found on I2C bus\n");
    return false;
  }

  if (WriteReg(obj, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_DEVICE_RESET) != HAL_OK) {
    return false;
  }
  HAL_Delay(100);

  if (WriteReg(obj, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_CLK_PLL_XGYRO) != HAL_OK) {
    return false;
  }
  HAL_Delay(50);  // PLL のロック待ち

  if (WriteReg(obj, MPU6050_REG_CONFIG, (uint8_t)dlpf) != HAL_OK) return false;

  uint16_t internal_rate_hz = (dlpf == MPU6050_DLPF_BW_260HZ) ? 8000 : 1000;
  if (sample_rate_hz > internal_rate_hz) sample_rate_hz = internal_rate_hz;
  uint16_t div = (uint16_t)(internal_rate_hz / sample_rate_hz);
  if (div == 0) div = 1;
  if (div > 256) div = 256;
  // 実際に出るレートは内部レートの整数分の1にしかならないので、丸めた値を保持しておく
  obj->sample_rate_hz = (uint16_t)(internal_rate_hz / div);
  if (WriteReg(obj, MPU6050_REG_SMPLRT_DIV, (uint8_t)(div - 1)) != HAL_OK) {
    return false;
  }

  if (WriteReg(obj, MPU6050_REG_GYRO_CONFIG, 0x00) != HAL_OK) return false;   // ±250 deg/s
  if (WriteReg(obj, MPU6050_REG_ACCEL_CONFIG, 0x00) != HAL_OK) return false;  // ±2 g

  if (WriteReg(obj, MPU6050_REG_INT_PIN_CFG, MPU6050_INT_PIN_CFG_PULSE) != HAL_OK) {
    return false;
  }
  if (WriteReg(obj, MPU6050_REG_INT_ENABLE, MPU6050_INT_DATA_RDY_EN) != HAL_OK) {
    return false;
  }

  uint8_t int_status = 0;
  ReadReg(obj, MPU6050_REG_INT_STATUS, &int_status, 1);

  obj->initialized = true;
  printf("[MPU6050] Initialized at 0x%02X (%u Hz, DLPF %u)\n", obj->i2c_addr,
         obj->sample_rate_hz, (unsigned)dlpf);
  return true;
}

bool Mpu6050_CalibrateOffset(Mpu6050 *obj, uint16_t sample_count) {
  if (obj == NULL || !obj->initialized || sample_count == 0) return false;

  int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
  uint8_t frame[MPU6050_BURST_LEN];

  const uint32_t interval_ms = (obj->sample_rate_hz > 0)
                                   ? (1000U / obj->sample_rate_hz + 1U)
                                   : 5U;

  for (uint16_t i = 0; i < sample_count; i++) {
    if (ReadReg(obj, MPU6050_REG_ACCEL_XOUT_H, frame, MPU6050_BURST_LEN) != HAL_OK) {
      printf("[MPU6050] Calibration read failed at %u\n", i);
      return false;
    }
    ax_sum += CombineBytes(frame[0], frame[1]);
    ay_sum += CombineBytes(frame[2], frame[3]);
    az_sum += CombineBytes(frame[4], frame[5]);
    gx_sum += CombineBytes(frame[8], frame[9]);
    gy_sum += CombineBytes(frame[10], frame[11]);
    gz_sum += CombineBytes(frame[12], frame[13]);
    HAL_Delay(interval_ms);
  }

  obj->offset.gyro_x = (int16_t)(gx_sum / sample_count);
  obj->offset.gyro_y = (int16_t)(gy_sum / sample_count);
  obj->offset.gyro_z = (int16_t)(gz_sum / sample_count);

  // 加速度は静止時に Z 軸へ 1g が乗る。その分を残してオフセットを求める
  // (重力の向きは平均値の符号から判定するので、上下どちら向きの取付でも成立する)
  int16_t az_avg = (int16_t)(az_sum / sample_count);
  int16_t gravity_lsb = (az_avg >= 0) ? (int16_t)MPU6050_ACCEL_LSB_PER_G
                                      : -(int16_t)MPU6050_ACCEL_LSB_PER_G;
  obj->offset.accel_x = (int16_t)(ax_sum / sample_count);
  obj->offset.accel_y = (int16_t)(ay_sum / sample_count);
  obj->offset.accel_z = (int16_t)(az_avg - gravity_lsb);

  printf("[MPU6050] Gyro offset : %d, %d, %d\n", obj->offset.gyro_x,
         obj->offset.gyro_y, obj->offset.gyro_z);
  printf("[MPU6050] Accel offset: %d, %d, %d\n", obj->offset.accel_x,
         obj->offset.accel_y, obj->offset.accel_z);
  return true;
}

void Mpu6050_SetOffset(Mpu6050 *obj, const Mpu6050Offset *offset) {
  if (obj == NULL || offset == NULL) return;
  obj->offset = *offset;
}

const Mpu6050Offset *Mpu6050_GetOffset(const Mpu6050 *obj) {
  return &obj->offset;
}

bool Mpu6050_ReadBlocking(Mpu6050 *obj, Mpu6050Sample *out) {
  if (obj == NULL || out == NULL || !obj->initialized) return false;
  uint8_t frame[MPU6050_BURST_LEN];
  if (ReadReg(obj, MPU6050_REG_ACCEL_XOUT_H, frame, MPU6050_BURST_LEN) != HAL_OK) {
    return false;
  }
  ParseFrame(obj, frame, out);
  return true;
}

void Mpu6050_OnDataReady(Mpu6050 *obj) {
  if (obj == NULL || !obj->initialized) return;
  if (obj->reading) {
    // 前回の転送が終わる前に次のサンプルが来た (I2C が詰まっている)
    obj->dropped_count++;
    return;
  }
  obj->reading = true;
  if (HAL_I2C_Mem_Read_IT(obj->i2c, (uint16_t)(obj->i2c_addr << 1),
                          MPU6050_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                          obj->rx_frame, MPU6050_BURST_LEN) != HAL_OK) {
    obj->reading = false;
    obj->dropped_count++;
  }
}

void Mpu6050_OnI2cRxComplete(Mpu6050 *obj, I2C_HandleTypeDef *hi2c) {
  if (obj == NULL || !obj->initialized || obj->i2c != hi2c) return;
  if (!obj->reading) return;
  obj->reading = false;

  uint8_t next = (uint8_t)((obj->head + 1) % MPU6050_RING_DEPTH);
  if (next == obj->tail) {
    // リングバッファが満杯 (メインループが Pop していない) -> 最新を捨てる
    obj->dropped_count++;
    return;
  }
  memcpy(obj->ring[obj->head], obj->rx_frame, MPU6050_BURST_LEN);
  obj->head = next;
  obj->sample_count++;
}

void Mpu6050_OnI2cError(Mpu6050 *obj, I2C_HandleTypeDef *hi2c) {
  if (obj == NULL || obj->i2c != hi2c) return;
  if (obj->reading) obj->dropped_count++;
  obj->reading = false;
}

bool Mpu6050_PopSample(Mpu6050 *obj, Mpu6050Sample *out) {
  if (obj == NULL || out == NULL || !obj->initialized) return false;
  uint8_t tail = obj->tail;
  if (tail == obj->head) return false;
  ParseFrame(obj, obj->ring[tail], out);
  obj->tail = (uint8_t)((tail + 1) % MPU6050_RING_DEPTH);
  return true;
}

uint8_t Mpu6050_GetPendingCount(const Mpu6050 *obj) {
  return (uint8_t)((obj->head + MPU6050_RING_DEPTH - obj->tail) % MPU6050_RING_DEPTH);
}
