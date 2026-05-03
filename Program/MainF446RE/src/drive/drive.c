#include "drive.h"

Serial serial_left;
Serial serial_right;
Serial serial_steer;

Timer steer_setup_timer;

SteerConfig steer_config;

static inline void Drive_SendSerialSteer(uint8_t header, int16_t value) {
  const uint8_t FOOTER = 0xAA;
  static uint8_t tx_buf[5];

  tx_buf[0] = 0xFF;
  tx_buf[1] = header;
  tx_buf[2] = ((int16_t)(value) >> 8) & 0xFF;
  tx_buf[3] = (int16_t)(value) & 0xFF;
  tx_buf[4] = FOOTER;

  Serial_Write(&serial_steer, tx_buf, sizeof(tx_buf));
}

void Drive_Init() {
  Serial_Init(&serial_left, &huart2, 256);
  Serial_Init(&serial_right, &huart4, 256);
  Serial_Init(&serial_steer, &huart5, 256);
  Timer_Init(&steer_setup_timer);
  Timer_Reset(&steer_setup_timer);
  while (Drive_SetupSteer() == false);
}

void Drive_SetAcceleration(float acceleration_left, float acceleration_right) {
  const uint8_t HEADER = 0xFF;
  const uint8_t FOOTER = 0xAA;
  static uint8_t tx_buf_left[5];
  static uint8_t tx_buf_right[5];

  tx_buf_left[0] = HEADER;
  tx_buf_left[1] = TORQUE_HEADER;
  tx_buf_left[2] = ((int16_t)(acceleration_left * 100) >> 8) & 0xFF;
  tx_buf_left[3] = (int16_t)(acceleration_left * 100) & 0xFF;
  tx_buf_left[4] = FOOTER;

  tx_buf_right[0] = HEADER;
  tx_buf_right[1] = TORQUE_HEADER;
  tx_buf_right[2] = ((int16_t)(-acceleration_right * 100) >> 8) & 0xFF;
  tx_buf_right[3] = (int16_t)(-acceleration_right * 100) & 0xFF;
  tx_buf_right[4] = FOOTER;

  Serial_Write(&serial_left, tx_buf_left, sizeof(tx_buf_left));
  Serial_Write(&serial_right, tx_buf_right, sizeof(tx_buf_right));
}

void Drive_SetSteer(float steer) {
  steer = -Constrain(steer, -1.0f, 1.0f);
  double mapped_rad = steer_config.min_rad + (steer + 1.0) / 2.0 * (steer_config.max_rad - steer_config.min_rad);
  Drive_SendSerialSteer(POSITION_HEADER, (int16_t)(mapped_rad * 1000));
}

void Drive(float acceleration, float steer) {
  // 電子ディファレンシャル制御
  if (steer > 0) {
    Drive_SetAcceleration(acceleration, acceleration * (1.0f - steer * DIFFERENTIAL));
  } else {
    Drive_SetAcceleration(acceleration * (1.0f + steer * DIFFERENTIAL), acceleration);
  }
  Drive_SetSteer(steer);
}

bool Drive_SetupSteer() {
  static float mech_theta = 0.0;
  static float amp_volt = 0.0;
  static float speed = 0.0;
  static uint8_t flags = 0;

  const static uint8_t HEADER = 0xFF;
  const static uint8_t FOOTER = 0xAA;
  const static uint8_t DATA_SIZE = 6;
  static uint8_t recv_data[6];
  static uint8_t index = 0;

  while (Serial_Available(&serial_steer)) {
    uint8_t recv_byte = Serial_Read(&serial_steer);
    if (index == 0) {
      if (recv_byte == HEADER) {
        index++;
      } else {
        index = 0;
      }
    } else if (index == (DATA_SIZE + 1)) {
      if (recv_byte == FOOTER) {
        flags = recv_data[0];
        speed = (int16_t)((recv_data[1] << 8) | recv_data[2]) * 0.01f;
        mech_theta = (int16_t)((recv_data[3] << 8) | recv_data[4]) * 0.001f;
        amp_volt = recv_data[5] * 0.1f;
      }
      index = 0;
    } else {
      recv_data[index - 1] = recv_byte;
      index++;
    }
  }
  int16_t a;
  if (Timer_ReadMs(&steer_setup_timer) < 1000) {
    a = 200;
    steer_config.min_rad = mech_theta;
  } else if (Timer_ReadMs(&steer_setup_timer) < 2000) {
    a = -200;
    steer_config.max_rad = mech_theta;
  }
  if (Timer_ReadMs(&steer_setup_timer) < 2000) {
    Drive_SendSerialSteer(TORQUE_HEADER, a);
  } else {
    printf("Steer setup completed: min_rad=%.3f, max_rad=%.3f\n", steer_config.min_rad, steer_config.max_rad);
    return true;
  }
  return false;
}