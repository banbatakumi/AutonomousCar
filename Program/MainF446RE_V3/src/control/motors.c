#include "motors.h"

void Motors_Init(Motors* obj, Serial* steering_serial, Serial* rear_left_serial, Serial* rear_right_serial) {
  BldcMotor_Init(&obj->steering, steering_serial);
  BldcMotor_Init(&obj->rear_left, rear_left_serial);
  BldcMotor_Init(&obj->rear_right, rear_right_serial);
}

void Motors_Receive(Motors* obj, bool powered) {
  if (!powered) {
    BldcMotor_Reset(&obj->steering);
    BldcMotor_Reset(&obj->rear_left);
    BldcMotor_Reset(&obj->rear_right);
    return;
  }
  BldcMotor_Receive(&obj->steering);
  BldcMotor_Receive(&obj->rear_left);
  BldcMotor_Receive(&obj->rear_right);
}

void Motors_Transmit(Motors* obj, bool powered) {
  if (!powered) return;
  BldcMotor_Transmit(&obj->steering);
  BldcMotor_Transmit(&obj->rear_left);
  BldcMotor_Transmit(&obj->rear_right);
}
