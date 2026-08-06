#ifndef APP_H_
#define APP_H_

#include <stdbool.h>
#include <stdio.h>

#include "adc.h"
#include "adc_dma.h"
#include "buzzer.h"
#include "digitalinout.h"
#include "drive.h"
#include "encoder.h"
#include "i2c.h"
#include "imu.h"
#include "lighting.h"
#include "main.h"
#include "motors.h"
#include "mymath.h"
#include "power.h"
#include "pwm_out.h"
#include "serial.h"
#include "steering.h"
#include "timer.h"
#include "ultrasonic.h"

void Setup();
void MainApp();

#endif  // APP_H_
