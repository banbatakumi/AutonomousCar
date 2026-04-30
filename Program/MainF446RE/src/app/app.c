#include "app.h"

DigitalOut user_led1;
DigitalOut user_led2;
PwmOut user_led3;
PwmOut user_led4;

DigitalIn button1;
DigitalIn button2;

void Setup() {
  DigitalOut_Init(&user_led1, USER_LED1_GPIO_Port, USER_LED1_Pin);
  DigitalOut_Init(&user_led2, USER_LED2_GPIO_Port, USER_LED2_Pin);
  PwmOut_Init(&user_led3, &htim4, TIM_CHANNEL_1);
  PwmOut_Init(&user_led4, &htim4, TIM_CHANNEL_2);
  DigitalIn_Init(&button1, BUTTON1_GPIO_Port, BUTTON1_Pin);
  DigitalIn_Init(&button2, BUTTON2_GPIO_Port, BUTTON2_Pin);
}

void MainApp() {
  while (1) {
  }
}