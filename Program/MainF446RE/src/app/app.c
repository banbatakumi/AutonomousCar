#include "app.h"

DigitalOut user_led1;
DigitalOut user_led2;
PwmOut user_led3;
PwmOut user_led4;

DigitalIn button1;
DigitalIn button2;

Timer control_interval_timer;

Timer timer;

Ultrasonic ultrasonic_front;
Ultrasonic ultrasonic_right;
Ultrasonic ultrasonic_left;
Ultrasonic ultrasonic_back;

Serial serial3;
LD06 lidar;

#define ADC2VOLT 0.0008058608059

#define CONTRO_FREQUENCY_HZ 10000
#define CONTROL_INTERVAL_US (1000000 / CONTRO_FREQUENCY_HZ)

uint16_t adc_value[5];

double voltage_signal;
double voltage_power;

void Setup() {
  printf("Setup started\n");
  DigitalOut_Init(&user_led1, USER_LED1_GPIO_Port, USER_LED1_Pin);
  DigitalOut_Init(&user_led2, USER_LED2_GPIO_Port, USER_LED2_Pin);
  PwmOut_Init(&user_led3, &htim4, TIM_CHANNEL_1);
  PwmOut_Init(&user_led4, &htim4, TIM_CHANNEL_2);
  DigitalIn_Init(&button1, BUTTON1_GPIO_Port, BUTTON1_Pin);
  DigitalIn_Init(&button2, BUTTON2_GPIO_Port, BUTTON2_Pin);

  Ultrasonic_Init(&ultrasonic_front, TRIG1_GPIO_Port, TRIG1_Pin, ECHO1_GPIO_Port, ECHO1_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_right, TRIG2_GPIO_Port, TRIG2_Pin, ECHO2_GPIO_Port, ECHO2_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_left, TRIG3_GPIO_Port, TRIG3_Pin, ECHO3_GPIO_Port, ECHO3_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_back, TRIG4_GPIO_Port, TRIG4_Pin, ECHO4_GPIO_Port, ECHO4_Pin, 0.8);

  Drive_Init();

  // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_value, sizeof(adc_value));
  // for (uint8_t i = 0; i < sizeof(adc_value); i++) {
  //   while (!(adc_value[i] > 0));  // ADCの値が代入されるまで待つ
  // }

  Serial_Init(&serial3, &huart3, 2048);
  LD06_Init(&lidar, &serial3);

  Timer_Init(&timer);
  Timer_Init(&control_interval_timer);
  printf("Setup finished\n");
}

void GetSensors() {
  Ultrasonic_Read(&ultrasonic_front);
  Ultrasonic_Read(&ultrasonic_right);
  Ultrasonic_Read(&ultrasonic_left);
  Ultrasonic_Read(&ultrasonic_back);

  voltage_signal = adc_value[4] * ADC2VOLT * 10;
  voltage_power = adc_value[3] * ADC2VOLT * 10;
}

void MainApp() {
  while (1) {
    // GetSensors();
    // Drive_Serial();
    // uint16_t front_distance = Ultrasonic_Get(&ultrasonic_front);
    // uint16_t right_distance = Ultrasonic_Get(&ultrasonic_right);
    // uint16_t left_distance = Ultrasonic_Get(&ultrasonic_left);
    // uint16_t back_distance = Ultrasonic_Get(&ultrasonic_back);

    // if (front_distance < 10) {
    //   Drive_Set(-2, left_distance > right_distance ? -1 : 1);
    // } else if (back_distance < 10) {
    //   Drive_Set(2, left_distance > right_distance ? 1 : -1);
    // } else {
    //   Drive_Set(1.5, (float)(left_distance - right_distance) / 75.0f);
    // }

    // Drive_Brake(3);
    // if (Timer_ReadMs(&timer) < 1000) {
    //   Drive_Set(3, 0);
    // }

    // if (DigitalIn_Read(&button1)) {
    //   Timer_Reset(&timer);
    // }
    if (LD06_Update(&lidar)) {
      // is_updated が true なら新しいパケットを受信済み

      // 前方の距離を取得する例 (0度が前方として)
      uint16_t dist_front = lidar.distances_360[0];
      // 右側の距離を取得する例
      uint16_t dist_right = lidar.distances_360[90];

      // ... 制御に使用
      printf("Front distance: %d mm, Right distance: %d mm\n", dist_front, dist_right);
    }
    // // 制御周期
    // while (Timer_ReadUs(&control_interval_timer) < CONTROL_INTERVAL_US) {
    //   DigitalOut_Write(&user_led1, 1);
    // }
    // DigitalOut_Write(&user_led1, 0);
    // Timer_Reset(&control_interval_timer);
  }
}
