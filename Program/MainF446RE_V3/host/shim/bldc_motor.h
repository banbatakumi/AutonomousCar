#ifndef BLDC_MOTOR_H_
#define BLDC_MOTOR_H_

// ホスト用の差し替え (host/README.md)。Drive が呼ぶ分だけを持ち、指令は構造体に残すだけ
// (MD の遅れ・制動の抜けは host_sim.c の車両モデル側)
#include <stdbool.h>
#include <stdint.h>

#include "serial.h"

typedef enum {
  HOST_MD_STOP = 0,
  HOST_MD_TORQUE = 1,
  HOST_MD_BRAKE = 2,
  HOST_MD_POSITION = 3,
} HostMdMode;

typedef struct {
  HostMdMode mode;
  float command;       // トルク [Nm] (符号つき) / 制動トルク [Nm] / 位置 [rad]
  float limit_nm;
  float speed_rad_s;   // MD が報告する角速度 (モータの取付向きの符号のまま)
  float mech_angle_rad;
  bool data_valid;
} BldcMotor;

static inline void BldcMotor_SetTorqueNm(BldcMotor* obj, float nm) {
  obj->mode = HOST_MD_TORQUE;
  obj->command = nm;
}
static inline void BldcMotor_SetBrakeNm(BldcMotor* obj, float nm) {
  obj->mode = HOST_MD_BRAKE;
  obj->command = nm;
}
static inline void BldcMotor_SetPosition(BldcMotor* obj, float rad) {
  obj->mode = HOST_MD_POSITION;
  obj->command = rad;
}
static inline void BldcMotor_Stop(BldcMotor* obj) {
  obj->mode = HOST_MD_STOP;
  obj->command = 0.0f;
}
static inline void BldcMotor_SetTorqueLimitNm(BldcMotor* obj, float nm) { obj->limit_nm = nm; }
static inline float BldcMotor_GetAngularSpeed(const BldcMotor* obj) { return obj->speed_rad_s; }
static inline float BldcMotor_GetMechAngle(const BldcMotor* obj) { return obj->mech_angle_rad; }
static inline bool BldcMotor_IsDataValid(const BldcMotor* obj) { return obj->data_valid; }

#endif  // BLDC_MOTOR_H_
