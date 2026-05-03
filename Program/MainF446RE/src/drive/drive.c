#include "drive.h"

#define SPEED_HEADER 0xFE
#define POSITION_HEADER 0xFD
#define TORQUE_HEADER 0xFC
#define BRAKE_HEADER 0xFB

#define DIFFERENTIAL 0.5f
#define MAX_ACCELERATION 4.0f

#define SERIAL_SEND_FREQUENCY_HZ 100
#define SERIAL_SEND_INTERVAL_MS (1000 / SERIAL_SEND_FREQUENCY_HZ)

Serial serial_left;
Serial serial_right;
Serial serial_steer;

Timer steer_setup_timer;

Timer serial_send_interval_timer;

SteerConfig steer_config;

MotorRecvData left_recv_data;
MotorRecvData right_recv_data;
MotorRecvData steer_recv_data;

Drive drive;

static inline void Drive_RecvSerial(Serial* serial, MotorRecvData* recv_data) {
  const uint8_t HEADER = 0xFF;
  const uint8_t FOOTER = 0xAA;
  const uint8_t DATA_SIZE = 6;
  static uint8_t recv_buf[6];
  static uint8_t index = 0;

  while (Serial_Available(serial)) {
    uint8_t recv_byte = Serial_Read(serial);
    if (index == 0) {
      if (recv_byte == HEADER) {
        index++;
      } else {
        index = 0;
      }
    } else if (index == (DATA_SIZE + 1)) {
      if (recv_byte == FOOTER) {
        recv_data->is_enable = recv_data->flags & 0b00000001;
        recv_data->is_voltage_out_of_range = (recv_data->flags >> 1) & 0b00000001;
        recv_data->is_overheat = (recv_data->flags >> 2) & 0b00000001;
        recv_data->speed = (int16_t)((recv_buf[1] << 8) | recv_buf[2]) * 0.01f;
        recv_data->mech_theta = (int16_t)((recv_buf[3] << 8) | recv_buf[4]) * 0.001f;
        recv_data->amp_volt = recv_buf[5] * 0.1f;
      }
      index = 0;
    } else {
      recv_buf[index - 1] = recv_byte;
      index++;
    }
  }
}

static inline void Drive_SendSerialSteer(uint8_t header, int16_t value) {
  const uint8_t FOOTER = 0xAA;
  static uint8_t tx_buf[5];

  tx_buf[0] = 0xFF;
  tx_buf[1] = header;
  tx_buf[2] = ((int16_t)value >> 8) & 0xFF;
  tx_buf[3] = (int16_t)value & 0xFF;
  tx_buf[4] = FOOTER;

  Serial_Write(&serial_steer, tx_buf, sizeof(tx_buf));
}

static inline void Drive_SendSerialAcceleration(uint8_t header, int16_t value_left, int16_t value_right) {
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
  Drive_RecvSerial(&serial_left, &left_recv_data);
  Drive_RecvSerial(&serial_right, &right_recv_data);
  Drive_RecvSerial(&serial_steer, &steer_recv_data);

  if (Timer_ReadMs(&serial_send_interval_timer) > SERIAL_SEND_INTERVAL_MS) {
    Timer_Reset(&serial_send_interval_timer);
    Drive_SendSerialSteer(POSITION_HEADER, (int16_t)(drive.steer * 1000));
    if (drive.do_brake) {
      Drive_SendSerialAcceleration(BRAKE_HEADER, (int16_t)(drive.brake_strength * 100), (int16_t)(drive.brake_strength * 100));
    } else {
      Drive_SendSerialAcceleration(TORQUE_HEADER, (int16_t)(drive.acceleration_left * 100), (int16_t)(drive.acceleration_right * -100));
    }
  }
}

void Drive_Init() {
  Serial_Init(&serial_left, &huart2, 256);
  Serial_Init(&serial_right, &huart4, 256);
  Serial_Init(&serial_steer, &huart5, 256);
  Timer_Init(&steer_setup_timer);
  Timer_Init(&serial_send_interval_timer);
  while (Drive_SetupSteer() == false);
}

bool Drive_SetupSteer() {
  Drive_RecvSerial(&serial_steer, &steer_recv_data);
  int16_t torque = 0;
  if (Timer_ReadMs(&steer_setup_timer) < 1000) {
    torque = 250;
    steer_config.min_rad = steer_recv_data.mech_theta;
  } else if (Timer_ReadMs(&steer_setup_timer) < 2000) {
    torque = -250;
    steer_config.max_rad = steer_recv_data.mech_theta;
  }

  if (Timer_ReadMs(&steer_setup_timer) < 2000) {
    Drive_SendSerialSteer(TORQUE_HEADER, torque);
  } else {
    printf("Steer setup completed: min_rad=%.3f, max_rad=%.3f\n", steer_config.min_rad, steer_config.max_rad);
    return true;
  }
  return false;
}

void Drive_Set(float acceleration, float steer) {
  drive.do_brake = false;
  // 電子ディファレンシャル制御
  if (steer > 0) {
    drive.acceleration_left = Constrain(acceleration, -MAX_ACCELERATION, MAX_ACCELERATION);
    drive.acceleration_right = Constrain(acceleration * (1.0f - steer * DIFFERENTIAL), -MAX_ACCELERATION, MAX_ACCELERATION);
  } else {
    drive.acceleration_left = Constrain(acceleration * (1.0f + steer * DIFFERENTIAL), -MAX_ACCELERATION, MAX_ACCELERATION);
    drive.acceleration_right = Constrain(acceleration, -MAX_ACCELERATION, MAX_ACCELERATION);
  }

  steer = -Constrain(steer, -1.0f, 1.0f);
  double mapped_rad = steer_config.min_rad + (steer + 1.0) / 2.0 * (steer_config.max_rad - steer_config.min_rad);
  drive.steer = mapped_rad;
}

void Drive_Brake(float deceleration) {
  drive.do_brake = true;
  drive.brake_strength = Constrain(deceleration, 0, MAX_ACCELERATION);
}