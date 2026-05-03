#ifndef APP_H_
#define APP_H_

#include <stdio.h>

#include "adc.h"
#include "digitalinout.h"
#include "drive.h"
#include "main.h"
#include "mymath.h"
#include "pwm_out.h"
#include "serial.h"
#include "stdbool.h"
#include "timer.h"
#include "ultrasonic.h"

void Setup();
void MainApp();

#endif  // APP_H_