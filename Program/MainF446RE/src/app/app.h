#ifndef APP_H_
#define APP_H_

#include <stdio.h>

#include "adc.h"
#include "digitalinout.h"
#include "drive.h"
// #include "i2c.h"
// #include "imu_manager.h"
#include "ld06.h"
#include "main.h"
#include "mymath.h"
#include "pwm_out.h"
#include "serial.h"
#include "stdbool.h"
#include "timer.h"
#include "ultrasonic.h"

void Setup();
void MainApp();

// Externs for globals used across app modules
extern DigitalOut user_led1;
extern DigitalOut user_led2;
extern DigitalOut user_led3;
extern PwmOut user_led4;

extern PwmOut front_led;

extern DigitalIn button1;
extern DigitalIn button2;

extern Timer control_interval_timer;

extern Ultrasonic ultrasonic_front;
extern Ultrasonic ultrasonic_right;
extern Ultrasonic ultrasonic_left;
extern Ultrasonic ultrasonic_back;

extern Serial serial3;
extern LD06 lidar;

// extern ImuManager imu_manager;

extern uint16_t adc_value[];

extern double voltage_signal;
extern double voltage_power;

extern Timer voltage_signal_led_timer;
extern Timer voltage_power_led_timer;
extern bool voltage_signal_led_state;
extern bool voltage_power_led_state;

extern bool battery_error;

// App-wide macros
#define ADC2VOLT (3.3 / 4095.0)
#define VOLTAGE_DIVIDER_RATIO ((10.0 + 1.0) / 1.0)
#define MIN_VOLTAGE 8.0
#define MAX_VOLTAGE 11.0
#define BLINK_PERIOD_US_AT_MIN_VOLTAGE 100U
#define BLINK_PERIOD_US_AT_MAX_VOLTAGE 1000000U

#define CONTRO_FREQUENCY_HZ 10000
#define CONTROL_INTERVAL_US (1000000 / CONTRO_FREQUENCY_HZ)
#define ADC_VALUE_COUNT 5

#endif  // APP_H_