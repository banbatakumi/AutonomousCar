#ifndef DRIVE_H_
#define DRIVE_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "lpf.h"
#include "main.h"
#include "mymath.h"
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
} Drive;

void Drive_Init();
bool Drive_SetupSteer();

void Drive_Serial();

void Drive_Set(float acceleration, float steer);

void Drive_Brake(float deceleration);

float Drive_GetSpeed();

#endif // DRIVE_H_
