#ifndef DRIVE_H_
#define DRIVE_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

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
} MotorRecvData;

typedef struct {
  float acceleration_left;
  float acceleration_right;
  float steer;
  bool do_brake;
  float brake_strength;
} Drive;

void Drive_Init();
bool Drive_SetupSteer();

void Drive_Serial();

void Drive_Set(float acceleration, float steer);

void Drive_Brake(float deceleration);

#endif  // DRIVE_H_
