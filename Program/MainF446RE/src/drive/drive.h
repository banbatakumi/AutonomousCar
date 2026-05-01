#ifndef DRIVE_H_
#define DRIVE_H_

#include <stdint.h>

#include "serial.h"

#define SPEED_HEADER 0xFE
#define POSITION_HEADER 0xFD
#define TORQUE_HEADER 0xFC
#define BRAKE_HEADER 0xFB

void Drive_Init();

void Drive_SetSpeed(int16_t left, int16_t right);

#endif  // DRIVE_H_
