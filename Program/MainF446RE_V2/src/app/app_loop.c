#include <math.h>
#include <stdint.h>

#include "algo_forward.h"

// 車速 1 m/s あたりの音程 [Hz]。値を上げると高い音になる。
#define MOTOR_SOUND_FREQ_PER_MPS 1500.0f
// 最高音程 [Hz]
#define MOTOR_SOUND_MAX_FREQ_HZ 5000U
// この速度未満は無音 [m/s]
#define MOTOR_SOUND_MIN_SPEED_MPS 0.05f
#include "algorithm.h"
#include "app.h"
#include "buzzer.h"
#include "drive.h"
#include "lighting.h"
#include "mode.h"

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

// 電圧に応じた周期のSin波でPWM輝度を更新する。高電圧=長周期(ゆっくり)、低電圧=短周期(速い)。
static void UpdateVoltageSinLed(PwmOut* led, Timer* timer, double voltage) {
  uint32_t period_us = VoltageToBlinkPeriodUs(voltage);
  uint32_t elapsed_us = Timer_ReadUs(timer);

  if (elapsed_us >= period_us) {
    Timer_Reset(timer);
    elapsed_us = 0U;
  }

  float phase = 6.28318530f * (float)elapsed_us / (float)period_us;
  float brightness = 0.5f * (1.0f + sinf(phase));
  PwmOut_Write(led, brightness);
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

void GetSensors() {
  LD06_Update(&lidar);

  Ultrasonic_Update(&ultrasonic_front);
  Ultrasonic_Update(&ultrasonic_right);
  Ultrasonic_Update(&ultrasonic_left);
  Ultrasonic_Update(&ultrasonic_back);

  uint16_t front_dist = Ultrasonic_Get(&ultrasonic_front);
  uint16_t right_dist = Ultrasonic_Get(&ultrasonic_right);
  uint16_t left_dist = Ultrasonic_Get(&ultrasonic_left);
  uint16_t back_dist = Ultrasonic_Get(&ultrasonic_back);

  // Read raw ADC values and apply LPF
  double raw_voltage_signal = adc_value[4] * ADC2VOLT * VOLTAGE_DIVIDER_RATIO;
  double raw_voltage_power = adc_value[3] * ADC2VOLT * VOLTAGE_DIVIDER_RATIO;

  voltage_signal = LPF_Update(&voltage_signal_lpf, raw_voltage_signal);
  voltage_power = LPF_Update(&voltage_power_lpf, raw_voltage_power);

  // MPU6050 (姿勢/角速度/角加速度) 更新
  Imu_Update(&imu);
  const MPU6050_Data* imu_data = Imu_GetData(&imu);
  if (imu_data != NULL) {
    Drive_SetImuData(imu_data->accel_x, imu_data->accel_y, imu_data->pitch, imu_data->roll);
  }
}

void MainApp() {
  while (1) {
    GetSensors();
    CheckBatteryVoltage();  // バッテリー電圧チェック
    Drive_Update();
    Buzzer_Update(&buzzer);
    Lighting_Update();
    UpdateVoltageSinLed(&user_led3, &voltage_signal_led_timer, voltage_signal);
    UpdateVoltageSinLed(&user_led4, &voltage_power_led_timer, voltage_power);
    DigitalOut_Write(&user_led1, (Drive_HasError() || battery_error) ? 1 : 0);

    // モード切替の処理（button1, button2を使用）
    Mode_Update(DigitalIn_Read(&button1), DigitalIn_Read(&button2));

    switch (Mode_Get()) {
      case MODE_STANDBY:
        // 待機モード：モータを停止
        Drive_Free();
        Lighting_SetHeadlight(0.01f);
        break;

      case MODE_RUN:
        // 走行モード：従来の自動走行ロジック
        Algorithm_Run(&lidar, Ultrasonic_Get(&ultrasonic_front));
        Lighting_SetHeadlight(0.5f);
        break;

      case MODE_FORWARD_ONLY:
        // 一旦前進だけのモード
        Algorithm_ForwardOnly(&lidar);
        Lighting_SetHeadlight(0.5f);
        break;

      default:
        break;
    }

    // 制御周期
    while (Timer_ReadUs(&control_interval_timer) < CONTROL_INTERVAL_US) {
      DigitalOut_Write(&user_led2, 1);
    }
    DigitalOut_Write(&user_led2, 0);
    Timer_Reset(&control_interval_timer);
  }
}
