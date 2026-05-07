#include "app.h"

#include "algo_forward.h"
#include "algorithm.h"
#include "drive.h"
#include "mode.h"

DigitalOut user_led1;
DigitalOut user_led2;
DigitalOut user_led3;
PwmOut user_led4;

PwmOut front_led;

DigitalIn button1;
DigitalIn button2;

Timer control_interval_timer;

Ultrasonic ultrasonic_front;
Ultrasonic ultrasonic_right;
Ultrasonic ultrasonic_left;
Ultrasonic ultrasonic_back;

Serial serial3;
LD06 lidar;
DigitalOut lidar_motor;

// ImuManager imu_manager;
uint16_t adc_value[ADC_VALUE_COUNT];

double voltage_signal;
double voltage_power;

LPF voltage_signal_lpf;
LPF voltage_power_lpf;

Timer voltage_signal_led_timer;
Timer voltage_power_led_timer;
bool voltage_signal_led_state = true;
bool voltage_power_led_state = true;

// バッテリー低電圧エラーフラグ
bool battery_error = false;

// Runtime functions moved to app_loop.c
// Globals remain in this file.
