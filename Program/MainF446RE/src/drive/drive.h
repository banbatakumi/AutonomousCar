#ifndef DRIVE_H_
#define DRIVE_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "main.h"
#include "mymath.h"
#include "serial.h"
#include "timer.h"

#define SPEED_HEADER 0xFE
#define POSITION_HEADER 0xFD
#define TORQUE_HEADER 0xFC
#define BRAKE_HEADER 0xFB

#define DIFFERENTIAL 0.5f

typedef struct {
  double min_rad;
  double max_rad;
} SteerConfig;

void Drive_Init();
bool Drive_SetupSteer();

void Drive_SetAcceleration(float acceleration_left, float acceleration_right);
void Drive_SetSteer(float steer);

void Drive(float acceleration, float steer);

#endif  // DRIVE_H_
