#include "algo_forward.h"
#include "algorithm.h"
#include "app.h"
#include "drive.h"
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

static void UpdateVoltageBlinkLed(DigitalOut* led, Timer* timer, bool* state, double voltage) {
  uint32_t period_us = VoltageToBlinkPeriodUs(voltage);
  uint32_t half_period_us = period_us / 2U;

  if (half_period_us == 0U) {
    half_period_us = 1U;
  }

  if (Timer_ReadUs(timer) >= half_period_us) {
    *state = !*state;
    DigitalOut_Write(led, *state);
    Timer_Reset(timer);
  }
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
  Ultrasonic_Update(&ultrasonic_front);
  Ultrasonic_Update(&ultrasonic_right);
  Ultrasonic_Update(&ultrasonic_left);
  Ultrasonic_Update(&ultrasonic_back);

  if (LD06_Update(&lidar)) {
  }

  // Read raw ADC values and apply LPF
  double raw_voltage_signal = adc_value[4] * ADC2VOLT * VOLTAGE_DIVIDER_RATIO;
  double raw_voltage_power = adc_value[3] * ADC2VOLT * VOLTAGE_DIVIDER_RATIO;

  voltage_signal = LPF_Update(&voltage_signal_lpf, raw_voltage_signal);
  voltage_power = LPF_Update(&voltage_power_lpf, raw_voltage_power);

  // MPU6050 (姿勢/角速度/角加速度) 更新
  Imu_Update(&imu);
  const MPU6050_Data* d = Imu_GetData(&imu);
  printf("IMU: Yaw=%.1f, Pitch=%.1f, Roll=%.1f\n",
         d->yaw, d->pitch, d->roll);
}

void MainApp() {
  while (1) {
    GetSensors();
    CheckBatteryVoltage();  // バッテリー電圧チェック
    Drive_Update();
    UpdateVoltageBlinkLed(&user_led3, &voltage_signal_led_timer, &voltage_signal_led_state, voltage_signal);
    UpdateVoltageBlinkLed(&user_led2, &voltage_power_led_timer, &voltage_power_led_state, voltage_power);
    PwmOut_Write(&user_led4, (Drive_HasError() || battery_error) ? 1.0f : 0.0f);

    // モード切替の処理（button1, button2を使用）
    Mode_Update(DigitalIn_Read(&button1), DigitalIn_Read(&button2));

    switch (Mode_Get()) {
      case MODE_STANDBY:
        // 待機モード：モータを停止
        Drive_Free();
        PwmOut_Write(&front_led, 0.01);
        break;

      case MODE_RUN:
        // 走行モード：従来の自動走行ロジック
        Algorithm_Run(&lidar);
        PwmOut_Write(&front_led, 0.5);
        break;

      case MODE_FORWARD_ONLY:
        // 一旦前進だけのモード
        Algorithm_ForwardOnly(&lidar);
        PwmOut_Write(&front_led, 0.5);
        break;

      default:
        break;
    }

    // 制御周期
    while (Timer_ReadUs(&control_interval_timer) < CONTROL_INTERVAL_US) {
      DigitalOut_Write(&user_led1, 1);
    }
    DigitalOut_Write(&user_led1, 0);
    Timer_Reset(&control_interval_timer);
  }
}
