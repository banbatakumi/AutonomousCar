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
    if (Mode_Update(DigitalIn_Read(&button1), DigitalIn_Read(&button2))) {
      Buzzer_Beep(&buzzer, 1000, 80);
    }

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
        const uint8_t HEADER = 0xFF;
        const uint8_t FOOTER = 0xAA;
        const uint8_t DATA_SIZE = 4;
        static uint8_t index = 0;
        static uint8_t recv_buf[4];

        static float move_speed = 0;
        static float acceleration = 0;
        static float steer = 0;
        static bool do_stop = 0;
        static bool do_remote_control = 0;
        static bool do_brake = 0;
        static bool on_headlight = 0;
        static bool on_hazard = 0;

        while (Serial_Available(&serial6)) {
          uint8_t byte = Serial_Read(&serial6);
          if (index == 0) {
            if (byte == HEADER) index++;
          } else if (index == DATA_SIZE + 1) {
            if (byte == FOOTER) {
              do_stop = recv_buf[0] & 0b00000001;
              do_remote_control = (recv_buf[0] >> 1) & 0b00000001;
              do_brake = (recv_buf[0] >> 2) & 0b00000001;
              on_headlight = (recv_buf[0] >> 3) & 0b00000001;
              on_hazard = (recv_buf[0] >> 4) & 0b00000001;
              move_speed = (int8_t)recv_buf[1] * 0.1f;    // -12.8 m/s 〜 +12.7 m/s
              acceleration = (int8_t)recv_buf[2] * 0.1f;  // -12.8 m/s² 〜 +12.7 m/s²
              steer = ((int8_t)recv_buf[3]) / 127.0f;     // -1.0 〜 +1.0
            }
            index = 0;
          } else {
            recv_buf[index - 1] = byte;
            index++;
          }
        }

        Lighting_SetHeadlight(on_headlight ? 1.0f : 0.01f);
        Lighting_SetHazard(on_hazard);

        if (do_stop) {
          Drive_Free();
        } else if (do_remote_control) {
          if (do_brake) {
            Drive_Brake(acceleration, steer);
          } else {
            Drive_SetVelocity(move_speed, acceleration, steer);
          }
        } else {
          Drive_Free();
        }

        uint8_t front_dis = Ultrasonic_Get(&ultrasonic_front) * 10;  // 仮
        uint8_t left_dis = Ultrasonic_Get(&ultrasonic_left) * 10;    // 仮
        uint8_t right_dis = Ultrasonic_Get(&ultrasonic_right) * 10;  // 仮
        uint8_t back_dis = Ultrasonic_Get(&ultrasonic_back) * 10;    // 仮

        if (Timer_ReadUs(&serial_send_interval_timer) >= 100000) {  // 100msごとに送信
          static uint8_t buf[11];
          buf[0] = HEADER;
          buf[1] = (int8_t)(Drive_GetSpeed() * 10);  // 現在速度を -128 〜 +127 の範囲で送信
          buf[2] = (int8_t)(Drive_GetAccel() * 10);
          buf[3] = front_dis;
          buf[4] = left_dis;
          buf[5] = right_dis;
          buf[6] = back_dis;
          buf[7] = (uint8_t)(voltage_signal * 10);  // 電圧を 0 〜 25.5V の範囲で送信
          buf[8] = (uint8_t)(voltage_power * 10);   // 電圧を 0 〜 25.5V の範囲で送信
          buf[9] = Drive_HasError() ? 1 : 0;        // エラー状態を送信
          buf[10] = FOOTER;
          Serial_Write(&serial6, buf, sizeof(buf));
          Timer_Reset(&serial_send_interval_timer);
        }

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
