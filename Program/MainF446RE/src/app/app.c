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

ImuManager imu_manager;

#define ADC2VOLT (3.3 / 4095.0)
#define VOLTAGE_DIVIDER_RATIO ((10.0 + 1.0) / 1.0)
#define MIN_VOLTAGE 8.0
#define MAX_VOLTAGE 11.0
#define BLINK_PERIOD_US_AT_MIN_VOLTAGE 100U
#define BLINK_PERIOD_US_AT_MAX_VOLTAGE 1000000U

#define CONTRO_FREQUENCY_HZ 10000
#define CONTROL_INTERVAL_US (1000000 / CONTRO_FREQUENCY_HZ)
#define ADC_VALUE_COUNT 5

uint16_t adc_value[ADC_VALUE_COUNT];

double voltage_signal;
double voltage_power;

static Timer voltage_signal_led_timer;
static Timer voltage_power_led_timer;
static bool voltage_signal_led_state = true;
static bool voltage_power_led_state = true;

// バッテリー低電圧エラーフラグ
static bool battery_error = false;

static uint32_t VoltageToBlinkPeriodUs(double voltage) {
  double clamped_voltage = voltage;

  if (clamped_voltage < MIN_VOLTAGE) {
    clamped_voltage = MIN_VOLTAGE;
  } else if (clamped_voltage > MAX_VOLTAGE) {
    clamped_voltage = MAX_VOLTAGE;
  }

  double ratio = (clamped_voltage - MIN_VOLTAGE) / (MAX_VOLTAGE - MIN_VOLTAGE);
  double period_us = BLINK_PERIOD_US_AT_MIN_VOLTAGE +
                     (BLINK_PERIOD_US_AT_MAX_VOLTAGE - BLINK_PERIOD_US_AT_MIN_VOLTAGE) * ratio;
  return (uint32_t)period_us;
}

static void UpdateVoltageBlinkLed(DigitalOut* led, Timer* timer, bool* state, double voltage) {
  uint32_t period_us = VoltageToBlinkPeriodUs(voltage);
  uint32_t half_period_us = period_us / 2U;

  if (half_period_us == 0U) {
    half_period_us = 1U;
  }

  if (Timer_ReadUs(timer) >= half_period_us) {
    *state = !*state;
    DigitalOut_Write(led, *state);
    Timer_Reset(timer);
  }
}

// バッテリー電圧をチェック、低電圧時にエラー対応
static void CheckBatteryVoltage(void) {
  if (voltage_signal < MIN_VOLTAGE) {
    // バッテリー低電圧エラー
    if (!battery_error) {
      printf("Battery voltage critical: %.2fV (minimum: %.2fV)\n", voltage_signal, MIN_VOLTAGE);
      battery_error = true;
    }
    // ロボット全体を停止
    Drive_Free();
  } else if (voltage_signal >= MIN_VOLTAGE + 0.5) {
    // ヒステリシス: 0.5V余裕を持たせてエラーをクリア
    battery_error = false;
  }
}

void Setup() {
  printf("Setup started\n");
  DigitalOut_Init(&user_led1, USER_LED1_GPIO_Port, USER_LED1_Pin);
  DigitalOut_Init(&user_led2, USER_LED2_GPIO_Port, USER_LED2_Pin);
  DigitalOut_Init(&user_led3, USER_LED3_GPIO_Port, USER_LED3_Pin);
  PwmOut_Init(&user_led4, &htim4, TIM_CHANNEL_2);

  PwmOut_Init(&front_led, &htim3, TIM_CHANNEL_1);

  DigitalIn_Init(&button1, BUTTON1_GPIO_Port, BUTTON1_Pin);
  DigitalIn_Init(&button2, BUTTON2_GPIO_Port, BUTTON2_Pin);

  Ultrasonic_Init(&ultrasonic_front, TRIG1_GPIO_Port, TRIG1_Pin, ECHO1_GPIO_Port, ECHO1_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_right, TRIG2_GPIO_Port, TRIG2_Pin, ECHO2_GPIO_Port, ECHO2_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_left, TRIG3_GPIO_Port, TRIG3_Pin, ECHO3_GPIO_Port, ECHO3_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_back, TRIG4_GPIO_Port, TRIG4_Pin, ECHO4_GPIO_Port, ECHO4_Pin, 0.8);

  Drive_Init(DigitalIn_Read(&button1));

  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_value, ADC_VALUE_COUNT);
  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle, DMA_IT_HT | DMA_IT_TC);

  Serial_Init(&serial3, &huart3, 2048);
  LD06_Init(&lidar, &serial3);

  IMU_Manager_Init(&imu_manager, &hi2c1, DigitalIn_Read(&button2));

  Timer_Init(&control_interval_timer);
  Timer_Init(&voltage_signal_led_timer);
  Timer_Init(&voltage_power_led_timer);
  DigitalOut_Write(&user_led2, voltage_power_led_state);
  DigitalOut_Write(&user_led3, voltage_signal_led_state);
  Mode_Init(DigitalIn_Read(&button1), DigitalIn_Read(&button2));
  printf("Setup finished\n");

  PwmOut_Write(&front_led, 0.05f);
}

void GetSensors() {
  Ultrasonic_Update(&ultrasonic_front);
  Ultrasonic_Update(&ultrasonic_right);
  Ultrasonic_Update(&ultrasonic_left);
  Ultrasonic_Update(&ultrasonic_back);

  if (LD06_Update(&lidar)) {
  }

  voltage_signal = adc_value[4] * ADC2VOLT * VOLTAGE_DIVIDER_RATIO;
  voltage_power = adc_value[3] * ADC2VOLT * VOLTAGE_DIVIDER_RATIO;

  // Update MPU6050 sensor data
  // IMU_Manager_Update(&imu_manager);
  // const MPU6050_Data* imu_data = IMU_Manager_GetData(&imu_manager);
  // printf("IMU Yaw: %.2f, Pitch: %.2f, Roll: %.2f\n", imu_data->yaw, imu_data->pitch, imu_data->roll);
}

void MainApp() {
  while (1) {
    GetSensors();
    CheckBatteryVoltage();  // バッテリー電圧チェック
    Drive_Serial();
    UpdateVoltageBlinkLed(&user_led3, &voltage_signal_led_timer, &voltage_signal_led_state, voltage_signal);
    UpdateVoltageBlinkLed(&user_led2, &voltage_power_led_timer, &voltage_power_led_state, voltage_power);
    PwmOut_Write(&user_led4, (Drive_HasError() || battery_error) ? 1.0f : 0.0f);

    // モード切替の処理（button1, button2を使用）
    Mode_Update(DigitalIn_Read(&button1), DigitalIn_Read(&button2));

    switch (Mode_Get()) {
      case MODE_STANDBY:
        // 待機モード：モータを停止
        Drive_Free();
        break;

      case MODE_RUN:
        // 走行モード：従来の自動走行ロジック
        Algorithm_Run(&lidar);
        break;

      case MODE_FORWARD_ONLY:
        // 一旦前進だけのモード
        Algorithm_ForwardOnly(&lidar);
        break;

      default:
        break;
    }

    // 制御周期
    while (Timer_ReadUs(&control_interval_timer) < CONTROL_INTERVAL_US) {
      DigitalOut_Write(&user_led1, 1);
    }
    DigitalOut_Write(&user_led1, 0);
    Timer_Reset(&control_interval_timer);
  }
}
