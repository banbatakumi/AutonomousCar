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
} MotorRecvData;

typedef struct {
  float acceleration_left;
  float acceleration_right;
  float steer;
} Drive;

void Drive_Init();
bool Drive_SetupSteer();

void Drive_Serial();

void Drive_SetSteer(float steer);

void Drive_Set(float acceleration, float steer);

#endif  // DRIVE_H_
