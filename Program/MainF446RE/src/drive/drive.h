#ifndef DRIVE_H_
#define DRIVE_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "main.h"
#include "serial.h"
#include "timer.h"

#define SPEED_HEADER 0xFE
#define POSITION_HEADER 0xFD
#define TORQUE_HEADER 0xFC
#define BRAKE_HEADER 0xFB

typedef struct {
  double min_rad;
  double max_rad;
} SteerConfig;

void Drive_Init();

void Drive_SetAcceleration(float acceleration);

void Drive_SetSteer(float steer);
bool Drive_SetupSteer();

#endif  // DRIVE_H_
