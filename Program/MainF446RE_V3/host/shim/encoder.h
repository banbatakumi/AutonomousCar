#ifndef ENCODER_H_
#define ENCODER_H_

// ホスト用の差し替え (host/README.md)。前輪の角速度 [rad/s] (取付向きの符号のまま) と、
// 車速のカルマンフィルタが使う直近の回転量・経過時間・更新番号を置くだけ。校正値 (Drive が
// Encoder_SetCalibration で入れる) は持っておき、車両モデル (host_sim.c) が角度の読みに同じ式を掛ける
#include <stdint.h>

#include "encoder_cal.h"

typedef struct {
  EncoderCal cal_left;
  EncoderCal cal_right;
  float angular_velocity_left;
  float angular_velocity_right;
  float last_gap_left_rad;
  float last_gap_right_rad;
  float last_dt_s;
  uint32_t update_seq;
} Encoder;

static inline float Encoder_GetAngularVelocityLeft(Encoder* obj) { return obj->angular_velocity_left; }
static inline float Encoder_GetAngularVelocityRight(Encoder* obj) { return obj->angular_velocity_right; }
static inline float Encoder_GetLastGapLeft(const Encoder* obj) { return obj->last_gap_left_rad; }
static inline float Encoder_GetLastGapRight(const Encoder* obj) { return obj->last_gap_right_rad; }
static inline float Encoder_GetLastDt(const Encoder* obj) { return obj->last_dt_s; }
static inline uint32_t Encoder_GetUpdateSeq(const Encoder* obj) { return obj->update_seq; }
static inline void Encoder_SetCalibration(Encoder* obj, const EncoderCal* left, const EncoderCal* right) {
  obj->cal_left = *left;
  obj->cal_right = *right;
}

#endif  // ENCODER_H_
