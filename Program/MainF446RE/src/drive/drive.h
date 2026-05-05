#ifndef DRIVE_H_
#define DRIVE_H_

#define DIFFERENTIAL 0.5f
#define MAX_ACCELERATION 5.0f
#define MAX_STEER_SPEED 2.0f // rad/s ステアリングの最大回転速度

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "lpf.h"
#include "main.h"
#include "mymath.h"
#include "pid.h"
#include "serial.h"
#include "timer.h"

typedef struct {
  double min_rad;
  double max_rad;
} SteerConfig;

typedef struct {
  float mech_theta;
  float amp_volt;
  float speed;
  uint8_t flags;
  bool is_enable;
  bool is_voltage_out_of_range;
  bool is_overheat;
} RecvData;

typedef struct {
  float acceleration_left;
  float acceleration_right;
  float steer;
  LPF lpf_steer;
  bool do_brake;
  float brake_strength;
} SendData;

typedef struct {
  float speed;
  LPF lpf_speed;
  float current_acceleration;
  Timer accel_timer;
  float current_target_velocity;
  Timer velocity_timer;
  PID pid_velocity;
  Timer steer_timer;
  bool is_free;
} Drive;

void Drive_Init(bool do_steer_setup);
bool Drive_SetupSteer();

void Drive_Serial();

void Drive_Set(float max_acceleration, float acceleration_rate, float steer);

void Drive_SetVelocity(float target_velocity, float acceleration, float steer);

void Drive_Brake(float deceleration);

void Drive_Free();

float Drive_GetSpeed();

bool Drive_HasError();

#endif // DRIVE_H_
