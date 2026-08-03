#include "app.h"

static uint16_t adc1_buffer[POWER_ADC1_NUM_CH];
static uint16_t adc2_buffer[ENCODER_ADC2_NUM_CH];

AdcDma adc1;
AdcDma adc2;

Power power;
Encoder encoder;
Ultrasonic ultrasonic_front;
Ultrasonic ultrasonic_rear;

DigitalOut led1;
DigitalOut led2;
PwmOut led3;
PwmOut led4;

Lighting lighting;

DigitalIn button1;
DigitalIn button2;

DigitalOut drive_power;
DigitalOut lidar_power;

Buzzer buzzer;

Timer control_interval_timer;

void Setup() {
  printf("Setup started\n");
  DigitalOut_Init(&led1, LED1_GPIO_Port, LED1_Pin);
  DigitalOut_Init(&led2, LED2_GPIO_Port, LED2_Pin);
  PwmOut_Init(&led3, &htim4, TIM_CHANNEL_1);
  PwmOut_Init(&led4, &htim4, TIM_CHANNEL_2);

  Lighting_Init(&lighting, &htim3, TIM_CHANNEL_1, &htim3, TIM_CHANNEL_3,
                &htim3, TIM_CHANNEL_2, &htim3, TIM_CHANNEL_4);

  DigitalOut_Init(&drive_power, DRIVE_POWER_GPIO_Port, DRIVE_POWER_Pin);
  DigitalOut_Init(&lidar_power, LIDAR_POWER_GPIO_Port, LIDAR_POWER_Pin);
  DigitalOut_Write(&drive_power, 1);

  AdcDma_Init(&adc1, &hadc1, adc1_buffer, POWER_ADC1_NUM_CH);
  AdcDma_Init(&adc2, &hadc2, adc2_buffer, ENCODER_ADC2_NUM_CH);
  Power_Init(&power, &adc1);
  Encoder_Init(&encoder, &adc2);

  Ultrasonic_Init(&ultrasonic_front, TRIG_FRONT_GPIO_Port, TRIG_FRONT_Pin, ECHO_FRONT_GPIO_Port, ECHO_FRONT_Pin);
  Ultrasonic_Init(&ultrasonic_rear, TRIG_REAR_GPIO_Port, TRIG_REAR_Pin, ECHO_REAR_GPIO_Port, ECHO_REAR_Pin);

  // TIM2 CH3: BUZZER, APB1 タイマクロック 90MHz, Prescaler=0 (tim.c の MX_TIM2_Init と一致させる)
  Buzzer_Init(&buzzer, &htim2, TIM_CHANNEL_3, 90000000U, 0U);

  Buzzer_PlayStartupMelody(&buzzer);

  DigitalIn_Init(&button1, BUTTON1_GPIO_Port, BUTTON1_Pin);
  DigitalIn_Init(&button2, BUTTON2_GPIO_Port, BUTTON2_Pin);

  Timer_Init(&control_interval_timer);
  printf("Setup finished\n");
}

// ECHOピンの変化割り込み (stm32f4xx_it.c の EXTI9_5_IRQHandler/EXTI15_10_IRQHandler 経由) から呼ばれる
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ECHO_FRONT_Pin) {
    Ultrasonic_OnEchoEdge(&ultrasonic_front);
  } else if (GPIO_Pin == ECHO_REAR_Pin) {
    Ultrasonic_OnEchoEdge(&ultrasonic_rear);
  }
}

void MainApp() {
  Lighting_SetHeadlight(&lighting, LIGHTING_HEADLIGHT_DAYTIME);
  Lighting_SetWinker(&lighting, LIGHTING_WINKER_HAZARD);

  while (1) {
    Lighting_Update(&lighting);
    Encoder_Update(&encoder);
    Ultrasonic_Update(&ultrasonic_front);
    Ultrasonic_Update(&ultrasonic_rear);

    uint32_t interval_us = 1000;
    DigitalOut_Write(&led2, 1);
    while (Timer_ReadUs(&control_interval_timer) < interval_us);
    DigitalOut_Write(&led2, 0);
    Timer_Reset(&control_interval_timer);
  }
}
