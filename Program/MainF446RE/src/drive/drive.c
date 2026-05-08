#include "drive.h"

#include "flash.h"

#define SPEED_HEADER 0xFE
#define POSITION_HEADER 0xFD
#define TORQUE_HEADER 0xFC
#define BRAKE_HEADER 0xFB

#define WHEEL_DIAMETER 0.055f  // m

#define SERIAL_SEND_FREQUENCY_HZ 100
#define SERIAL_SEND_INTERVAL_MS (1000 / SERIAL_SEND_FREQUENCY_HZ)

Serial serial_left;
Serial serial_right;
Serial serial_steer;

PwmOut brake_led;
PwmOut winker_left_led;
PwmOut winker_right_led;

Timer steer_setup_timer;

Timer serial_send_interval_timer;

SteerConfig steer_config;

typedef struct {
  RecvData data;
  uint8_t recv_buf[6];
  uint8_t index;
} Protocol;

Protocol left_protocol;
Protocol right_protocol;
Protocol steer_protocol;

SendData send_data;

Drive drive;

static void Drive_RecvSerial(Serial* serial, Protocol* protocol) {
  const uint8_t HEADER = 0xFF;
  const uint8_t FOOTER = 0xAA;
  const uint8_t DATA_SIZE = 6;

  while (Serial_Available(serial)) {
    uint8_t recv_byte = Serial_Read(serial);
    if (protocol->index == 0) {
      if (recv_byte == HEADER) {
        protocol->index++;
      }
    } else if (protocol->index == (DATA_SIZE + 1)) {
      if (recv_byte == FOOTER) {
        RecvData* data = &protocol->data;
        data->flags = protocol->recv_buf[0];
        data->is_enable = data->flags & 0b00000001;
        data->is_voltage_out_of_range = (data->flags >> 1) & 0b00000001;
        data->is_overheat = (data->flags >> 2) & 0b00000001;
        data->speed =
            (int16_t)((protocol->recv_buf[1] << 8) | protocol->recv_buf[2]) *
            0.01f;
        data->mech_theta =
            (int16_t)((protocol->recv_buf[3] << 8) | protocol->recv_buf[4]) *
            0.001f;
        data->amp_volt = protocol->recv_buf[5] * 0.1f;
      }
      protocol->index = 0;
    } else {
      protocol->recv_buf[protocol->index - 1] = recv_byte;
      protocol->index++;
    }
  }
}

static void Drive_SendSerialSteer(uint8_t header, int16_t value) {
  const uint8_t FOOTER = 0xAA;
  static uint8_t tx_buf[5];

  tx_buf[0] = 0xFF;
  tx_buf[1] = header;
  tx_buf[2] = ((int16_t)value >> 8) & 0xFF;
  tx_buf[3] = (int16_t)value & 0xFF;
  tx_buf[4] = FOOTER;

  Serial_Write(&serial_steer, tx_buf, sizeof(tx_buf));
}

static void Drive_SendSerialAcceleration(uint8_t header, int16_t value_left,
                                         int16_t value_right) {
  const uint8_t HEADER = 0xFF;
  const uint8_t FOOTER = 0xAA;
  static uint8_t tx_buf_left[5];
  static uint8_t tx_buf_right[5];

  tx_buf_left[0] = HEADER;
  tx_buf_left[1] = header;
  tx_buf_left[2] = ((int16_t)value_left >> 8) & 0xFF;
  tx_buf_left[3] = (int16_t)value_left & 0xFF;
  tx_buf_left[4] = FOOTER;

  tx_buf_right[0] = HEADER;
  tx_buf_right[1] = header;
  tx_buf_right[2] = ((int16_t)value_right >> 8) & 0xFF;
  tx_buf_right[3] = (int16_t)value_right & 0xFF;
  tx_buf_right[4] = FOOTER;

  Serial_Write(&serial_left, tx_buf_left, sizeof(tx_buf_left));
  Serial_Write(&serial_right, tx_buf_right, sizeof(tx_buf_right));
}

void Drive_Serial() {
  Drive_RecvSerial(&serial_left, &left_protocol);
  Drive_RecvSerial(&serial_right, &right_protocol);
  Drive_RecvSerial(&serial_steer, &steer_protocol);

  drive.speed = (right_protocol.data.speed - left_protocol.data.speed) / 2.0f *
                WHEEL_DIAMETER;
  drive.speed = MAF_Update(&drive.maf_speed, drive.speed);

  // ブレーキランプの制御
  if (send_data.do_brake) {
    // 急ブレーキの場合: 高速で点滅
    if (send_data.brake_strength >= 1.0f && drive.speed > 0.5) {
      float elapsed_ms = Timer_ReadMs(&drive.brake_led_timer);
      // 200ms周期で点灯/消灯（100msごと）
      if (elapsed_ms >= 200) {
        Timer_Reset(&drive.brake_led_timer);
        elapsed_ms = 0;
      }
      PwmOut_Write(&brake_led, elapsed_ms < 100 ? 1.0f : 0.0f);
    } else {
      // 通常ブレーキ: 常に点灯
      PwmOut_Write(&brake_led, 1.0f);
    }
  } else {
    // ブレーキなし: 薄く光っている
    PwmOut_Write(&brake_led, 0.1);
  }

  if (Timer_ReadMs(&serial_send_interval_timer) > SERIAL_SEND_INTERVAL_MS) {
    Timer_Reset(&serial_send_interval_timer);
    if (drive.is_free) {
      Serial_Reset(&serial_left);
      Serial_Reset(&serial_right);
      Serial_Reset(&serial_steer);
      return;  // フリー状態: 送信停止でモーター待機
    }
    Drive_SendSerialSteer(POSITION_HEADER, (int16_t)(send_data.steer * 1000));
    if (send_data.do_brake) {
      Drive_SendSerialAcceleration(BRAKE_HEADER,
                                   (int16_t)(send_data.brake_strength * 100),
                                   (int16_t)(send_data.brake_strength * 100));
    } else {
      Drive_SendSerialAcceleration(
          TORQUE_HEADER, (int16_t)(send_data.acceleration_left * 100),
          (int16_t)(send_data.acceleration_right * -100));
    }
  }
}

void Drive_Init(bool do_steer_setup) {
  Serial_Init(&serial_left, &huart4, 256);
  Serial_Init(&serial_right, &huart2, 256);
  Serial_Init(&serial_steer, &huart5, 256);
  Timer_Init(&steer_setup_timer);
  Timer_Init(&serial_send_interval_timer);
  MAF_Init(&drive.maf_speed, 10);
  drive.current_acceleration = 0.0f;
  Timer_Init(&drive.accel_timer);
  drive.current_target_velocity = 0.0f;
  Timer_Init(&drive.velocity_timer);
  PID_Init(&drive.pid_velocity, 1.0f, 1.0f, 0.0f, -MAX_ACCELERATION,
           MAX_ACCELERATION);
  Timer_Init(&drive.steer_timer);
  Timer_Init(&drive.brake_led_timer);
  drive.is_free = true;

  PwmOut_Init(&brake_led, &htim3, TIM_CHANNEL_4);
  PwmOut_Init(&winker_left_led, &htim3, TIM_CHANNEL_3);
  PwmOut_Init(&winker_right_led, &htim3, TIM_CHANNEL_2);

  for (int i = 0; i < 3; i++) {
    PwmOut_Write(&winker_left_led, 0.5);
    PwmOut_Write(&winker_right_led, 0.5);
    HAL_Delay(200);
    PwmOut_Write(&winker_left_led, 0);
    PwmOut_Write(&winker_right_led, 0);
    HAL_Delay(200);
  }

  if (do_steer_setup) {
    // ボタン1押下起動: ステアセットアップ実行 → Flashに保存
    printf("Steer setup: calibrating...\n");
    Timer_Reset(&steer_setup_timer);
    while (Drive_SetupSteer() == false);
    Flash_WriteData(FLASH_USER_START_ADDR, &steer_config,
                    sizeof(SteerConfig));
    printf("Steer config saved to flash\n");
  } else {
    // 通常起動: Flashから読み出し
    Flash_ReadData(FLASH_USER_START_ADDR, &steer_config,
                   sizeof(SteerConfig));
    printf("Steer config loaded from flash: min_rad=%.3f, max_rad=%.3f\n",
           steer_config.min_rad, steer_config.max_rad);
  }
}

bool Drive_SetupSteer() {
  Drive_RecvSerial(&serial_steer, &steer_protocol);
  int16_t torque = 0;
  if (Timer_ReadMs(&steer_setup_timer) < 1000) {
    torque = 250;
    steer_config.min_rad = steer_protocol.data.mech_theta;
  } else if (Timer_ReadMs(&steer_setup_timer) < 2000) {
    torque = -250;
    steer_config.max_rad = steer_protocol.data.mech_theta;
  }

  if (Timer_ReadMs(&steer_setup_timer) < 2000) {
    Drive_SendSerialSteer(TORQUE_HEADER, torque);
  } else {
    printf("Steer setup completed: min_rad=%.3f, max_rad=%.3f\n",
           steer_config.min_rad, steer_config.max_rad);
    return true;
  }
  return false;
}

static void Drive_ApplyAccelerationAndSteer(float acceleration, float steer) {
  // 電子ディファレンシャル制御
  if (steer > 0) {
    send_data.acceleration_left =
        Constrain(acceleration, -MAX_ACCELERATION, MAX_ACCELERATION);
    send_data.acceleration_right =
        Constrain(acceleration * (1.0f - steer * DIFFERENTIAL),
                  -MAX_ACCELERATION, MAX_ACCELERATION);
  } else {
    send_data.acceleration_left =
        Constrain(acceleration * (1.0f + steer * DIFFERENTIAL),
                  -MAX_ACCELERATION, MAX_ACCELERATION);
    send_data.acceleration_right =
        Constrain(acceleration, -MAX_ACCELERATION, MAX_ACCELERATION);
  }

  steer = -Constrain(steer, -1.0f, 1.0f);
  double mapped_rad = steer_config.min_rad + (steer + 1.0) / 2.0 * (steer_config.max_rad - steer_config.min_rad);

  // ステアリング回転速度制限 [rad/s]
  float steer_dt = Timer_Read(&drive.steer_timer);
  Timer_Reset(&drive.steer_timer);
  if (steer_dt > 0.0f && steer_dt < 0.5f) {
    double max_delta = MAX_STEER_SPEED * steer_dt;
    double delta = mapped_rad - send_data.steer;
    if (delta > max_delta) delta = max_delta;
    if (delta < -max_delta) delta = -max_delta;
    send_data.steer += delta;
  } else {
    send_data.steer = mapped_rad;
  }
}

void Drive_Set(float max_acceleration, float acceleration_rate, float steer) {
  send_data.do_brake = false;
  drive.is_free = false;

  // dt計算（加速度ランプ用）
  float dt = Timer_Read(&drive.accel_timer);
  Timer_Reset(&drive.accel_timer);
  if (dt > 0.5f)
    dt = 0.0f;  // 初回 or 長時間停止後のガード

  // 現在のアクセルを目標値に向けてランプアップ/ダウン
  if (drive.current_acceleration < max_acceleration) {
    drive.current_acceleration += acceleration_rate * dt;
    if (drive.current_acceleration > max_acceleration)
      drive.current_acceleration = max_acceleration;
  } else if (drive.current_acceleration > max_acceleration) {
    drive.current_acceleration -= acceleration_rate * dt;
    if (drive.current_acceleration < max_acceleration)
      drive.current_acceleration = max_acceleration;
  }

  Drive_ApplyAccelerationAndSteer(drive.current_acceleration, steer);
}

void Drive_SetVelocity(float target_velocity, float acceleration, float steer) {
  send_data.do_brake = false;
  drive.is_free = false;

  // dt計算（速度ランプ用）
  float dt = Timer_Read(&drive.velocity_timer);
  Timer_Reset(&drive.velocity_timer);
  if (dt > 0.1f)
    dt = 0.0f;  // 初回 or 長時間停止後のガード

  // 現在の目標速度をtarget_velocityに向けてacceleration [m/s²] でランプ
  if (drive.current_target_velocity < target_velocity) {
    drive.current_target_velocity += acceleration * dt;
    if (drive.current_target_velocity > target_velocity)
      drive.current_target_velocity = target_velocity;
  } else if (drive.current_target_velocity > target_velocity) {
    drive.current_target_velocity -= acceleration * dt;
    if (drive.current_target_velocity < target_velocity)
      drive.current_target_velocity = target_velocity;
  }

  // PIDでランプ後の目標速度に追従
  float accel_output = PID_Update(&drive.pid_velocity,
                                  drive.current_target_velocity, drive.speed);
  Drive_ApplyAccelerationAndSteer(accel_output, steer);
}

void Drive_Brake(float deceleration) {
  send_data.do_brake = true;
  send_data.brake_strength = Constrain(deceleration, 0, MAX_ACCELERATION);
  drive.current_acceleration = 0.0f;
  drive.current_target_velocity = 0.0f;
  drive.is_free = false;
  PID_Reset(&drive.pid_velocity);
}

void Drive_Free() {
  drive.is_free = true;
  drive.current_acceleration = 0.0f;
  drive.current_target_velocity = 0.0f;
  PID_Reset(&drive.pid_velocity);
}

float Drive_GetSpeed() { return drive.speed; }

static bool Drive_RecvDataHasError(const RecvData* data) {
  return data->is_voltage_out_of_range || data->is_overheat;
}

bool Drive_HasError() {
  return Drive_RecvDataHasError(&left_protocol.data) ||
         Drive_RecvDataHasError(&right_protocol.data) ||
         Drive_RecvDataHasError(&steer_protocol.data);
}
