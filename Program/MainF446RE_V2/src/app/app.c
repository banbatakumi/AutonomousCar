#include "app.h"

#include "algo_forward.h"
#include "algorithm.h"
#include "buzzer.h"
#include "drive.h"
#include "lighting.h"
#include "mode.h"
#include "remote.h"
#include "sensor.h"

Buzzer buzzer;

DigitalOut user_led1;
DigitalOut user_led2;
PwmOut user_led3;
PwmOut user_led4;

DigitalIn button1;
DigitalIn button2;

Timer control_interval_timer;

Serial serial6;

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  Sensor_OnI2CRxComplete(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
  Sensor_OnI2CError(hi2c);
}

void Setup() {
  printf("Setup started\n");
  DigitalOut_Init(&user_led1, USER_LED1_GPIO_Port, USER_LED1_Pin);
  DigitalOut_Init(&user_led2, USER_LED2_GPIO_Port, USER_LED2_Pin);
  PwmOut_Init(&user_led3, &htim4, TIM_CHANNEL_1);
  PwmOut_Init(&user_led4, &htim4, TIM_CHANNEL_2);

  // TIM2 CH2: BUZZER, APB1 タイマクロック 90MHz, Prescaler=9
  Buzzer_Init(&buzzer, &htim2, TIM_CHANNEL_2, 90000000U, 9U);

  Lighting_Init();
  Buzzer_PlayStartupMelody(&buzzer);

  DigitalIn_Init(&button1, BUTTON1_GPIO_Port, BUTTON1_Pin);
  DigitalIn_Init(&button2, BUTTON2_GPIO_Port, BUTTON2_Pin);

  // button2 を押したまま起動 → IMU 再キャリブレーション
  Sensor_Init(DigitalIn_Read(&button2));

  Drive_Init(DigitalIn_Read(&button1));

  Serial_Init(&serial6, &huart6, 2048);

  Timer_Init(&control_interval_timer);

  Mode_Init(DigitalIn_Read(&button1), DigitalIn_Read(&button2));
  printf("Setup finished\n");
}

void MainApp() {
  while (1) {
    Sensor_Update(Mode_Get() != MODE_STANDBY);

    const MPU6050_Data* imu_data = Sensor_GetImuData();
    if (imu_data != NULL) {
      Drive_SetImuData(imu_data->accel_x, imu_data->accel_y, imu_data->pitch, imu_data->roll, imu_data->gyro_z);
    }

    bool tilted = (imu_data != NULL) &&
                  (imu_data->pitch >  IMU_TILT_LIMIT_DEG || imu_data->pitch < -IMU_TILT_LIMIT_DEG ||
                   imu_data->roll  >  IMU_TILT_LIMIT_DEG || imu_data->roll  < -IMU_TILT_LIMIT_DEG);

    bool battery_err = Sensor_GetBatteryError() &&
                       (Sensor_GetVoltageSignal() > RASPBERRY_POWER_MAX_VOLTAGE ||
                        Sensor_GetVoltagePower() > RASPBERRY_POWER_MAX_VOLTAGE);

    if (battery_err || tilted) {
      Drive_Free();
    }

    Drive_Update();
    Buzzer_Update(&buzzer);
    Lighting_Update();
    Sensor_UpdateVoltageLeds(&user_led3, &user_led4);
    DigitalOut_Write(&user_led1, (Drive_HasError() || battery_err || tilted) ? 1 : 0);

    if (Mode_Update(DigitalIn_Read(&button1), DigitalIn_Read(&button2))) {
      Buzzer_Beep(&buzzer, 1000, 80);
    }

    switch (Mode_Get()) {
      case MODE_STANDBY:
        Remote_Update();
        break;

      case MODE_RUN:
        Algorithm_Run(Sensor_GetLidar(), Sensor_GetUltrasonicFront());
        Lighting_SetHeadlight(0.5f);
        break;

      case MODE_FORWARD_ONLY:
        Algorithm_ForwardOnly(Sensor_GetLidar());
        Lighting_SetHeadlight(0.5f);
        break;

      default:
        break;
    }

    // 制御周期 (STANDBY は低速)
    uint32_t interval_us = (Mode_Get() == MODE_STANDBY) ? STANDBY_CONTROL_INTERVAL_US : CONTROL_INTERVAL_US;
    DigitalOut_Write(&user_led2, 1);
    while (Timer_ReadUs(&control_interval_timer) < interval_us);
    DigitalOut_Write(&user_led2, 0);
    Timer_Reset(&control_interval_timer);
  }
}
