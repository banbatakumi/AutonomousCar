#ifndef ENCODER_H_
#define ENCODER_H_

// ホスト用の差し替え (host/README.md)。前輪の角速度 [rad/s] (取付向きの符号のまま) を置くだけ
typedef struct {
  float angular_velocity_left;
  float angular_velocity_right;
} Encoder;

static inline float Encoder_GetAngularVelocityLeft(Encoder* obj) { return obj->angular_velocity_left; }
static inline float Encoder_GetAngularVelocityRight(Encoder* obj) { return obj->angular_velocity_right; }

#endif  // ENCODER_H_
