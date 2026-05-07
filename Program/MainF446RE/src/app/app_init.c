#include "app.h"
#include "mode.h"

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
  
  // Initialize LiDAR motor control (汎用化: LD06に渡す)
  DigitalOut_Init(&lidar_motor, LIDAR_GPIO_Port, LIDAR_Pin);
  LD06_Init(&lidar, &serial3, &lidar_motor);

  // IMU_Manager_Init(&imu_manager, &hi2c1, DigitalIn_Read(&button2));

  Timer_Init(&control_interval_timer);
  Timer_Init(&voltage_signal_led_timer);
  Timer_Init(&voltage_power_led_timer);

  // Initialize LPF for voltage measurements (k_lpf=0.8 for smooth filtering)
  LPF_Init(&voltage_signal_lpf, 0.8, 0.0);
  LPF_Init(&voltage_power_lpf, 0.8, 0.0);

  DigitalOut_Write(&user_led2, voltage_power_led_state);
  DigitalOut_Write(&user_led3, voltage_signal_led_state);
  Mode_Init(DigitalIn_Read(&button1), DigitalIn_Read(&button2));
  printf("Setup finished\n");
}
