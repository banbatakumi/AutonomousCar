#include "app.h"

DigitalOut user_led1;
DigitalOut user_led2;
PwmOut user_led3;
PwmOut user_led4;

DigitalIn button1;
DigitalIn button2;

Timer serial_send_interval_timer;

Timer timer;

Ultrasonic ultrasonic_front;
Ultrasonic ultrasonic_right;
Ultrasonic ultrasonic_left;
Ultrasonic ultrasonic_back;

void Setup() {
  printf("Setup started\n");
  DigitalOut_Init(&user_led1, USER_LED1_GPIO_Port, USER_LED1_Pin);
  DigitalOut_Init(&user_led2, USER_LED2_GPIO_Port, USER_LED2_Pin);
  PwmOut_Init(&user_led3, &htim4, TIM_CHANNEL_1);
  PwmOut_Init(&user_led4, &htim4, TIM_CHANNEL_2);
  DigitalIn_Init(&button1, BUTTON1_GPIO_Port, BUTTON1_Pin);
  DigitalIn_Init(&button2, BUTTON2_GPIO_Port, BUTTON2_Pin);

  Ultrasonic_Init(&ultrasonic_front, TRIG1_GPIO_Port, TRIG1_Pin, ECHO1_GPIO_Port, ECHO1_Pin, 0.5);
  Ultrasonic_Init(&ultrasonic_right, TRIG2_GPIO_Port, TRIG2_Pin, ECHO2_GPIO_Port, ECHO2_Pin, 0.5);
  Ultrasonic_Init(&ultrasonic_left, TRIG3_GPIO_Port, TRIG3_Pin, ECHO3_GPIO_Port, ECHO3_Pin, 0.5);
  Ultrasonic_Init(&ultrasonic_back, TRIG4_GPIO_Port, TRIG4_Pin, ECHO4_GPIO_Port, ECHO4_Pin, 0.5);

  Drive_Init();

  Timer_Init(&serial_send_interval_timer);
  Timer_Init(&timer);
  printf("Setup finished\n");
}

void MainApp() {
  while (1) {
    Ultrasonic_Read(&ultrasonic_front);
    Ultrasonic_Read(&ultrasonic_right);
    Ultrasonic_Read(&ultrasonic_left);
    Ultrasonic_Read(&ultrasonic_back);
    uint16_t front_distance = Ultrasonic_Get(&ultrasonic_front);
    uint16_t right_distance = Ultrasonic_Get(&ultrasonic_right);
    uint16_t left_distance = Ultrasonic_Get(&ultrasonic_left);
    uint16_t back_distance = Ultrasonic_Get(&ultrasonic_back);

    float acceleration = 0.0f;
    float steer = 0.0f;

    if (front_distance < 50) {
      acceleration = -1.5f;
      steer = 1.3f;
    } else {
      acceleration = 1.5f;
      steer = Constrain((200 - left_distance) * 0.01, 0.1f, 1.8f);
    }

    if (Timer_ReadMs(&serial_send_interval_timer) >= 10) {
      Drive_SetAcceleration(acceleration);
      Drive_SetSteer(steer);
      Timer_Reset(&serial_send_interval_timer);
    }
  }
}