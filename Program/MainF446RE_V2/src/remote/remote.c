#include "remote.h"

#include "app.h"
#include "buzzer.h"
#include "drive.h"
#include "lidar_utils.h"
#include "lighting.h"
#include "mymath.h"
#include "remote_auto1.h"
#include "remote_auto2.h"
#include "remote_manual.h"
#include "sensor.h"
#include "timer.h"

#define HEADER 0xFF
#define FOOTER 0xAA
#define RECV_DATA_SIZE 4
#define SEND_INTERVAL_US 100000  // 100 ms
#define LIDAR_SECTORS 36

static RemoteCommand cmd = {0};
static Timer send_timer;
static bool initialized = false;

static void ParseSerial(void) {
  static uint8_t index = 0;
  static uint8_t recv_buf[RECV_DATA_SIZE];

  while (Serial_Available(&serial6)) {
    uint8_t byte = Serial_Read(&serial6);
    if (index == 0) {
      if (byte == HEADER) index++;
    } else if (index == RECV_DATA_SIZE + 1) {
      if (byte == FOOTER) {
        cmd.do_stop = recv_buf[0] & 0x01;
        cmd.do_brake = (recv_buf[0] >> 1) & 0x01;
        cmd.on_headlight = (recv_buf[0] >> 2) & 0x01;
        cmd.on_hazard = (recv_buf[0] >> 3) & 0x01;
        cmd.play_sound = (recv_buf[0] >> 4) & 0x01;
        cmd.enable_auto_brake = (recv_buf[0] >> 5) & 0x01;
        cmd.mode = (recv_buf[0] >> 6) & 0x03;
        cmd.move_speed = (int8_t)recv_buf[1] * 0.1f;
        cmd.acceleration = (int8_t)recv_buf[2] * 0.1f;
        cmd.steer = ((int8_t)recv_buf[3]) / 127.0f;
      }
      index = 0;
    } else {
      recv_buf[index - 1] = byte;
      index++;
    }
  }
}

static void SendTelemetry(void) {
  if (Timer_ReadUs(&send_timer) < SEND_INTERVAL_US) return;

  const LD06* lidar = Sensor_GetLidar();
  uint8_t dis[LIDAR_SECTORS];
  for (uint8_t i = 0; i < LIDAR_SECTORS; i++) {
    dis[i] = Constrain(Lidar_GetSector(lidar, i * 10, 5).avg * 0.005f, 0, 15);
  }

  const MPU6050_Data* imu_data = Sensor_GetImuData();

  static uint8_t buf[31];
  buf[0] = HEADER;
  buf[1] = (int8_t)(Drive_GetSpeed() * 10);
  buf[2] = (int8_t)(Drive_GetAccel() * 10);
  buf[3] = dis[0] << 4 | dis[1];
  buf[4] = dis[2] << 4 | dis[3];
  buf[5] = dis[4] << 4 | dis[5];
  buf[6] = dis[6] << 4 | dis[7];
  buf[7] = dis[8] << 4 | dis[9];
  buf[8] = dis[10] << 4 | dis[11];
  buf[9] = dis[12] << 4 | dis[13];
  buf[10] = dis[14] << 4 | dis[15];
  buf[11] = dis[16] << 4 | dis[17];
  buf[12] = dis[18] << 4 | dis[19];
  buf[13] = dis[20] << 4 | dis[21];
  buf[14] = dis[22] << 4 | dis[23];
  buf[15] = dis[24] << 4 | dis[25];
  buf[16] = dis[26] << 4 | dis[27];
  buf[17] = dis[28] << 4 | dis[29];
  buf[18] = dis[30] << 4 | dis[31];
  buf[19] = dis[32] << 4 | dis[33];
  buf[20] = dis[34] << 4 | dis[35];
  buf[21] = (uint8_t)(Sensor_GetVoltageSignal() * 10);
  buf[22] = (uint8_t)(Sensor_GetVoltagePower() * 10);
  buf[23] = Drive_HasError() ? 1 : 0;
  buf[24] = (int8_t)(imu_data->accel_x * 2) << 4 | (int8_t)(imu_data->accel_y * 2);
  buf[25] = (int8_t)imu_data->pitch;
  buf[26] = (int8_t)imu_data->roll;
  buf[27] = Drive_GetLeftMotorTemperature();
  buf[28] = Drive_GetRightMotorTemperature();
  buf[29] = Drive_GetSteerMotorTemperature();
  buf[30] = FOOTER;
  Serial_Write(&serial6, buf, sizeof(buf));
  Timer_Reset(&send_timer);
}

void Remote_Update(void) {
  if (!initialized) {
    Timer_Init(&send_timer);
    initialized = true;
  }

  ParseSerial();

  Lighting_SetHeadlight(cmd.on_headlight ? 1.0f : 0.01f);
  Lighting_SetHazard(cmd.on_hazard);
  Buzzer_SetTone(&buzzer, cmd.play_sound ? 1000 : 0);

  if (cmd.do_stop) {
    Drive_Free();
  } else {
    const LD06* lidar = Sensor_GetLidar();
    switch (cmd.mode) {
      case 1:
        RemoteManual_Run(&cmd, lidar);
        break;
      case 2:
        RemoteAuto1_Run(&cmd, lidar);
        break;
      case 3:
        RemoteAuto2_Run(&cmd, lidar);
        break;
      default:
        Drive_Free();
        break;
    }
  }

  SendTelemetry();
}

const RemoteCommand* Remote_GetCommand(void) {
  return &cmd;
}
