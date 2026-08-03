#include "app.h"

DigitalOut led1;
DigitalOut led2;
PwmOut led3;
PwmOut led4;

DigitalIn button1;
DigitalIn button2;

Buzzer buzzer;

Timer control_interval_timer;

void Setup() {
  printf("Setup started\n");
  DigitalOut_Init(&led1, USER_LED1_GPIO_Port, USER_LED1_Pin);
  DigitalOut_Init(&led2, USER_LED2_GPIO_Port, USER_LED2_Pin);
  PwmOut_Init(&led3, &htim4, TIM_CHANNEL_1);
  PwmOut_Init(&led4, &htim4, TIM_CHANNEL_2);

  // TIM2 CH3: BUZZER, APB1 タイマクロック 90MHz, Prescaler=0 (tim.c の MX_TIM2_Init と一致させる)
  Buzzer_Init(&buzzer, &htim2, TIM_CHANNEL_3, 90000000U, 0U);

  Buzzer_PlayStartupMelody(&buzzer);

  DigitalIn_Init(&button1, BUTTON1_GPIO_Port, BUTTON1_Pin);
  DigitalIn_Init(&button2, BUTTON2_GPIO_Port, BUTTON2_Pin);

  Timer_Init(&control_interval_timer);
  printf("Setup finished\n");
}

void MainApp() {
  while (1) {
    // 制御周期 (STANDBY は低速)
    uint32_t interval_us = 1000;
    DigitalOut_Write(&led2, 1);
    while (Timer_ReadUs(&control_interval_timer) < interval_us);
    DigitalOut_Write(&led2, 0);
    Timer_Reset(&control_interval_timer);
  }
}
