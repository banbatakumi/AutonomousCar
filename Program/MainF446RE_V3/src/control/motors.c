#include "motors.h"

void Motors_Init(Motors* obj, Serial* steering_serial, Serial* rear_left_serial, Serial* rear_right_serial) {
  BldcMotor_Init(&obj->steering, steering_serial);
  BldcMotor_Init(&obj->rear_left, rear_left_serial);
  BldcMotor_Init(&obj->rear_right, rear_right_serial);
}

void Motors_Update(Motors* obj) {
  BldcMotor_Update(&obj->steering);
  BldcMotor_Update(&obj->rear_left);
  BldcMotor_Update(&obj->rear_right);
}
