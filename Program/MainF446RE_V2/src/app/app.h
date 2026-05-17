#ifndef APP_H_
#define APP_H_

#include <stdio.h>

#include "buzzer.h"
#include "digitalinout.h"
#include "drive.h"
#include "main.h"
#include "pwm_out.h"
#include "serial.h"
#include "stdbool.h"
#include "timer.h"

void Setup();
void MainApp();

extern Buzzer buzzer;

extern DigitalOut user_led1;
extern DigitalOut user_led2;
extern PwmOut user_led3;
extern PwmOut user_led4;

extern DigitalIn button1;
extern DigitalIn button2;

extern Timer control_interval_timer;

extern Serial serial6;

#define CONTRO_FREQUENCY_HZ 10000
#define CONTROL_INTERVAL_US (1000000 / CONTRO_FREQUENCY_HZ)

#endif  // APP_H_
