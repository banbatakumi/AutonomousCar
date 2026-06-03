#include "sensor.h"

#include <math.h>
#include <stdio.h>

#include "adc.h"
#include "imu.h"
#include "ld06.h"
#include "lidar_utils.h"
#include "lpf.h"
#include "main.h"
#include "pwm_out.h"
#include "serial.h"
#include "tim.h"
#include "timer.h"
#include "ultrasonic.h"
#include "usart.h"

static Ultrasonic ultrasonic_front;
static Ultrasonic ultrasonic_right;
static Ultrasonic ultrasonic_left;
static Ultrasonic ultrasonic_back;

static Serial serial3;
static LD06 lidar;
static PwmOut lidar_motor;

static Imu imu;
static uint16_t adc_value[ADC_VALUE_COUNT];

static double voltage_signal;
static double voltage_power;
static LPF voltage_signal_lpf;
static LPF voltage_power_lpf;

static Timer voltage_signal_led_timer;
static Timer voltage_power_led_timer;

static bool battery_error = false;

void Sensor_Init(bool recalibrate_imu) {
  Ultrasonic_Init(&ultrasonic_front, TRIG1_GPIO_Port, TRIG1_Pin, ECHO1_GPIO_Port, ECHO1_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_right, TRIG2_GPIO_Port, TRIG2_Pin, ECHO2_GPIO_Port, ECHO2_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_left, TRIG3_GPIO_Port, TRIG3_Pin, ECHO3_GPIO_Port, ECHO3_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_back, TRIG4_GPIO_Port, TRIG4_Pin, ECHO4_GPIO_Port, ECHO4_Pin, 0.8);

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, ADC_VALUE_COUNT);

  Serial_Init(&serial3, &huart3, 2048);

  PwmOut_Init(&lidar_motor, &htim1, TIM_CHANNEL_4);
  LD06_Init(&lidar, &serial3, &lidar_motor);

  Imu_Init(&imu, &hi2c1, recalibrate_imu);

  Timer_Init(&voltage_signal_led_timer);
  Timer_Init(&voltage_power_led_timer);
  LPF_Init(&voltage_signal_lpf, 0.8, 0.0);
  LPF_Init(&voltage_power_lpf, 0.8, 0.0);
}

void Sensor_Update(bool update_ultrasonic) {
  LD06_Update(&lidar);

  if (update_ultrasonic) {
    Ultrasonic_Update(&ultrasonic_front);
    Ultrasonic_Update(&ultrasonic_right);
    Ultrasonic_Update(&ultrasonic_left);
    Ultrasonic_Update(&ultrasonic_back);
  }

  double raw_voltage_signal = adc_value[4] * ADC2VOLT * VOLTAGE_DIVIDER_RATIO;
  double raw_voltage_power = adc_value[3] * ADC2VOLT * VOLTAGE_DIVIDER_RATIO;
  voltage_signal = LPF_Update(&voltage_signal_lpf, raw_voltage_signal);
  voltage_power = LPF_Update(&voltage_power_lpf, raw_voltage_power);

  Imu_Update(&imu);

  if (voltage_signal < MIN_VOLTAGE || voltage_power < MIN_VOLTAGE) {
    if (!battery_error) {
      printf("Battery voltage critical: signal=%.2fV power=%.2fV (minimum: %.2fV)\n",
             voltage_signal, voltage_power, MIN_VOLTAGE);
      battery_error = true;
    }
  } else if (voltage_signal >= MIN_VOLTAGE + 0.5 && voltage_power >= MIN_VOLTAGE + 0.5) {
    battery_error = false;
  }
}

static uint32_t VoltageToBlinkPeriodUs(double voltage) {
  double v = voltage;
  if (v < MIN_VOLTAGE) v = MIN_VOLTAGE;
  if (v > MAX_VOLTAGE) v = MAX_VOLTAGE;
  double ratio = (v - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE);
  return (uint32_t)(BLINK_PERIOD_US_AT_MIN_VOLTAGE +
                    (BLINK_PERIOD_US_AT_MAX_VOLTAGE - BLINK_PERIOD_US_AT_MIN_VOLTAGE) * ratio);
}

static void UpdateSinLed(PwmOut* led, Timer* timer, double voltage) {
  uint32_t period_us = VoltageToBlinkPeriodUs(voltage);
  uint32_t elapsed_us = Timer_ReadUs(timer);
  if (elapsed_us >= period_us) {
    Timer_Reset(timer);
    elapsed_us = 0U;
  }
  float phase = 6.28318530f * (float)elapsed_us / (float)period_us;
  PwmOut_Write(led, 0.5f * (1.0f + sinf(phase)));
}

void Sensor_UpdateVoltageLeds(PwmOut* led_signal, PwmOut* led_power) {
  UpdateSinLed(led_signal, &voltage_signal_led_timer, voltage_signal);
  UpdateSinLed(led_power, &voltage_power_led_timer, voltage_power);
}

void Sensor_OnI2CRxComplete(I2C_HandleTypeDef* hi2c) {
  Imu_OnI2CRxComplete(&imu, hi2c);
}

void Sensor_OnI2CError(I2C_HandleTypeDef* hi2c) {
  Imu_OnI2CError(&imu, hi2c);
}

bool Sensor_GetBatteryError(void) { return battery_error; }
double Sensor_GetVoltageSignal(void) { return voltage_signal; }
double Sensor_GetVoltagePower(void) { return voltage_power; }
const MPU6050_Data* Sensor_GetImuData(void) { return Imu_GetData(&imu); }

float Sensor_GetUltrasonicFront(void) { return Ultrasonic_Get(&ultrasonic_front); }
float Sensor_GetUltrasonicRight(void) { return Ultrasonic_Get(&ultrasonic_right); }
float Sensor_GetUltrasonicLeft(void) { return Ultrasonic_Get(&ultrasonic_left); }
float Sensor_GetUltrasonicBack(void) { return Ultrasonic_Get(&ultrasonic_back); }

const LD06* Sensor_GetLidar(void) { return &lidar; }
