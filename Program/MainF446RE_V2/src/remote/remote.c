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
#define SEND_INTERVAL_US 100000   // 100 ms
#define WATCHDOG_TIMEOUT_US 500000  // 500 ms

static RemoteCommand cmd = {0};
static Timer send_timer;
static Timer watchdog_timer;
static bool watchdog_stop = false;
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
        Timer_Reset(&watchdog_timer);
        watchdog_stop = false;
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

  const MPU6050_Data* imu_data = Sensor_GetImuData();

  static uint8_t buf[12 + 720 + 1];  // ヘッダ+基本データ+360点分のLiDAR距離+フッタ
  buf[0] = HEADER;
  buf[1] = Drive_HasError() ? 1 : 0;
  buf[2] = (int8_t)(Drive_GetSpeed() * 10);
  buf[3] = (int8_t)(Drive_GetAccel() * 10);
  buf[4] = (uint8_t)(Sensor_GetVoltageSignal() * 10);
  buf[5] = (uint8_t)(Sensor_GetVoltagePower() * 10);
  buf[6] = (int8_t)(imu_data->accel_x * 2) << 4 | (int8_t)(imu_data->accel_y * 2);
  buf[7] = (int8_t)imu_data->pitch;
  buf[8] = (int8_t)imu_data->roll;
  buf[9] = Drive_GetLeftMotorTemperature();
  buf[10] = Drive_GetRightMotorTemperature();
  buf[11] = Drive_GetSteerMotorTemperature();
  const LD06* lidar = Sensor_GetLidar();
  uint16_t distances_360[360];
  Lidar_BuildFilledPoints(lidar, distances_360, 200);  // confidence_threshold = 100
  Lidar_FilterSpikes(distances_360, 10, 0.3f);         // window_deg = 10, threshold_ratio = 0.5
  for (int i = 0; i < 360; i++) {
    buf[12 + i * 2] = (distances_360[i] >> 8) & 0xFF;
    buf[12 + i * 2 + 1] = distances_360[i] & 0xFF;
  }
  buf[732] = FOOTER;
  Serial_Write(&serial6, buf, sizeof(buf));
  Timer_Reset(&send_timer);
}

void Remote_Update(void) {
  if (!initialized) {
    Timer_Init(&send_timer);
    Timer_Init(&watchdog_timer);
    initialized = true;
  }

  ParseSerial();

  if (Timer_ReadUs(&watchdog_timer) >= WATCHDOG_TIMEOUT_US) {
    watchdog_stop = true;
  }

  Lighting_SetHeadlight(cmd.on_headlight ? 0.5f : 0.01f);
  Lighting_SetHazard(cmd.on_hazard);
  Buzzer_SetTone(&buzzer, cmd.play_sound ? 500 : 0);

  if (cmd.do_stop || watchdog_stop) {
    if (Abs(Drive_GetSpeed()) >= 0.5f) {
      Drive_Brake(MAX_TORQUE, 0.0f);
    } else {
      Drive_Free();
    }
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
