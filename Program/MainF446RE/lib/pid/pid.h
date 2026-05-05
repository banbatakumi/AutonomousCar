#ifndef PID_H_
#define PID_H_

#include <stdbool.h>

#include "timer.h"

typedef struct {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float output_min;
  float output_max;
  Timer timer;
  bool is_first_update;
} PID;

static inline void PID_Init(PID *pid, float kp, float ki, float kd,
                             float output_min, float output_max) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->output_min = output_min;
  pid->output_max = output_max;
  pid->is_first_update = true;
  Timer_Init(&pid->timer);
}

static inline void PID_SetGains(PID *pid, float kp, float ki, float kd) {
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

static inline void PID_Reset(PID *pid) {
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
  pid->is_first_update = true;
  Timer_Reset(&pid->timer);
}

static inline float PID_Update(PID *pid, float target, float current) {
  float dt = Timer_Read(&pid->timer);
  Timer_Reset(&pid->timer);

  // 初回呼び出し or 異常なdt の場合はスキップ
  if (pid->is_first_update || dt <= 0.0f || dt > 1.0f) {
    pid->is_first_update = false;
    pid->prev_error = target - current;
    return 0.0f;
  }

  float error = target - current;

  // P: 比例
  float p = pid->kp * error;

  // I: 積分（アンチワインドアップ付き）
  pid->integral += error * dt;
  float i = pid->ki * pid->integral;

  // D: 微分
  float d = pid->kd * (error - pid->prev_error) / dt;
  pid->prev_error = error;

  float output = p + i + d;

  // 出力制限 + アンチワインドアップ
  if (output > pid->output_max) {
    output = pid->output_max;
    if (error > 0) pid->integral -= error * dt;
  } else if (output < pid->output_min) {
    output = pid->output_min;
    if (error < 0) pid->integral -= error * dt;
  }

  return output;
}

#endif  // PID_H_
