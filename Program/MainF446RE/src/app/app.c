#include "app.h"

DigitalOut user_led1;
DigitalOut user_led2;
PwmOut user_led3;
PwmOut user_led4;

DigitalIn button1;
DigitalIn button2;

Serial serial2;
Serial serial4;
Serial serial5;

Timer serial_send_interval_timer;

Timer timer;

void Setup() {
  printf("Setup started\n");
  DigitalOut_Init(&user_led1, USER_LED1_GPIO_Port, USER_LED1_Pin);
  DigitalOut_Init(&user_led2, USER_LED2_GPIO_Port, USER_LED2_Pin);
  PwmOut_Init(&user_led3, &htim4, TIM_CHANNEL_1);
  PwmOut_Init(&user_led4, &htim4, TIM_CHANNEL_2);
  DigitalIn_Init(&button1, BUTTON1_GPIO_Port, BUTTON1_Pin);
  DigitalIn_Init(&button2, BUTTON2_GPIO_Port, BUTTON2_Pin);

  Serial_Init(&serial2, &huart2, 128);
  Serial_Init(&serial4, &huart4, 128);
  Serial_Init(&serial5, &huart5, 128);

  Timer_Init(&serial_send_interval_timer);
  Timer_Init(&timer);
  printf("Setup finished\n");
}

void MainApp() {
  while (1) {
    int16_t torque = 100;
    static int16_t prev_torque = 0;
    if (Timer_ReadMs(&timer) >= 4000) {
      Timer_Reset(&timer);
    } else if (Timer_ReadMs(&timer) >= 2000) {
      torque = -200;
    } else {
      torque = 200;
    }
    torque = prev_torque * 0.95 + torque * 0.05;
    prev_torque = torque;

    if (Timer_ReadMs(&serial_send_interval_timer) >= 10) {
      const uint8_t HEADER = 0xFF;
      const uint8_t FOOTER = 0xAA;

      const uint8_t DATA_SIZE = 5;
      uint8_t data1[DATA_SIZE];

      data1[0] = HEADER;
      data1[1] = 0xFC;                           // TORQUE_HEADER
      data1[2] = (int16_t)(torque >> 8) & 0xFF;  // 上位8ビット
      data1[3] = (int16_t)(torque) & 0xFF;
      data1[4] = FOOTER;
      Serial_Write(&serial2, data1, sizeof(data1));  // シリアル送信

      uint8_t data2[DATA_SIZE];
      data2[0] = HEADER;
      data2[1] = 0xFC;                                // TORQUE_HEADER
      data2[2] = (int16_t)(torque * -1 >> 8) & 0xFF;  // 上位8ビット
      data2[3] = (int16_t)(torque * -1) & 0xFF;
      data2[4] = FOOTER;
      Serial_Write(&serial4, data2, sizeof(data2));  // シリアル送信
      // Serial_Write(&serial5, data, sizeof(data));       // シリアル送信
      Timer_Reset(&serial_send_interval_timer);
    }
  }
}