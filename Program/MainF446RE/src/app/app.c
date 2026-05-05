#include "app.h"

DigitalOut user_led1;
DigitalOut user_led2;
PwmOut user_led3;
PwmOut user_led4;

DigitalIn button1;
DigitalIn button2;

Timer control_interval_timer;

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

  Ultrasonic_Init(&ultrasonic_front, TRIG1_GPIO_Port, TRIG1_Pin,
                  ECHO1_GPIO_Port, ECHO1_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_right, TRIG2_GPIO_Port, TRIG2_Pin,
                  ECHO2_GPIO_Port, ECHO2_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_left, TRIG3_GPIO_Port, TRIG3_Pin, ECHO3_GPIO_Port,
                  ECHO3_Pin, 0.8);
  Ultrasonic_Init(&ultrasonic_back, TRIG4_GPIO_Port, TRIG4_Pin, ECHO4_GPIO_Port,
                  ECHO4_Pin, 0.8);

  Drive_Init();

  // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_value, sizeof(adc_value));
  // for (uint8_t i = 0; i < sizeof(adc_value); i++) {
  //   while (!(adc_value[i] > 0));  // ADCの値が代入されるまで待つ
  // }

  Serial_Init(&serial3, &huart3, 2048);
  LD06_Init(&lidar, &serial3);

  Timer_Init(&control_interval_timer);
  printf("Setup finished\n");
}

void GetSensors() {
  Ultrasonic_Update(&ultrasonic_front);
  Ultrasonic_Update(&ultrasonic_right);
  Ultrasonic_Update(&ultrasonic_left);
  Ultrasonic_Update(&ultrasonic_back);

  LD06_Update(&lidar);

  voltage_signal = adc_value[4] * ADC2VOLT * 10;
  voltage_power = adc_value[3] * ADC2VOLT * 10;
}

void MainApp() {
  while (1) {
    GetSensors();
    Drive_Serial();

    // --- 1. 両側壁の距離取得 (左右のセンタリング用) ---
    int left_dist = 0, right_dist = 0;
    int count_l = 0, count_r = 0;

    // 左(90度付近)の平均距離
    for (int i = 45; i <= 90; i++) {
      if (lidar.distances_360[i] > 0) {
        left_dist += lidar.distances_360[i];
        count_l++;
      }
    }
    // 右(270度付近)の平均距離
    for (int i = 270; i <= 315; i++) {
      if (lidar.distances_360[i] > 0) {
        right_dist += lidar.distances_360[i];
        count_r++;
      }
    }
    if (count_l > 0)
      left_dist /= count_l;
    if (count_r > 0)
      right_dist /= count_r;

    // --- 2. 目標ステアリング(steer)の計算 ---
    float error = (float)(left_dist - right_dist);
    float Kp = 0.002f;

    // errorがプラス(左が遠い)の時に、左(-45°側)へ行くか右(45°側)へ行くかは
    // 機体の仕様に合わせて符号を調整してください。
    float steer = Kp * error;
    steer = Constrain(steer, -1.0f, 1.0f);

    // --- 3. 進行方向(タイヤが向いている方向)の距離確認 ---
    // steer (-1.0 〜 1.0) を角度 (-45.0° 〜 45.0°) に変換
    float look_ahead_angle = steer * 45.0f;
    int base_angle = (int)look_ahead_angle;

    int front_dist = 0;
    int count_f = 0;

    // 進行方向の角度を中心に ±5度 の距離を確認する
    for (int i = base_angle - 10; i <= base_angle + 10; i++) {
      // 角度がマイナスになった場合を考慮して360で丸める (-45° -> 315°)
      int idx = (i + 360) % 360;
      if (lidar.distances_360[idx] > 0) {
        front_dist += lidar.distances_360[idx];
        count_f++;
      }
    }
    if (count_f > 0)
      front_dist /= count_f;

    // --- 4. 走行制御 (ブレーキ or 進行) ---
    float acceleration = 0.0f;

    // "進行方向"に壁が近い場合(例: 300mm以内)はブレーキ
    if (front_dist > 0 && front_dist < (Drive_GetSpeed() * 100 + 400)) {
      Drive_Brake(2.0f);
    } else {
      acceleration = 1.5f; // 前進
      Drive_Set(acceleration, steer);
    }

    // 制御周期
    while (Timer_ReadUs(&control_interval_timer) < CONTROL_INTERVAL_US) {
      DigitalOut_Write(&user_led1, 1);
    }
    DigitalOut_Write(&user_led1, 0);
    Timer_Reset(&control_interval_timer);
  }
}
