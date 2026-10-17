#ifndef IMU_H_
#define IMU_H_

// ホスト用の差し替え (host/README.md)。Drive が読むのはヨーレート [deg/s] と、
// 車速のカルマンフィルタが使う生の加速度 X・サンプル番号
#include <stdbool.h>
#include <stddef.h>  // 実物の imu.h は HAL 経由で NULL を持ち込む
#include <stdint.h>

typedef struct {
  float gyro_z;
  float accel_x_raw;
  uint32_t sample_seq;
} ImuData;

typedef struct {
  ImuData data;
  bool ready;
} Imu;

static inline const ImuData* Imu_GetData(const Imu* obj) { return &obj->data; }
static inline bool Imu_IsReady(const Imu* obj) { return obj->ready; }

#endif  // IMU_H_
