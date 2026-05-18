#include "drive.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "flash.h"
#include "lighting.h"

// モータコントローラへのコマンドヘッダ
#define POSITION_HEADER 0xFD
#define VOLTAGE_HEADER 0xFC
#define BRAKE_HEADER 0xFB

#define WHEEL_RADIUS 0.0275f  // [m]

#define TRACTION_SLIP_VEL_THRESHOLD 0.25f    // [m/s] スリップと判定する車輪速とIMU推定速度の差
#define TRACTION_BIAS_SPEED_THRESHOLD 0.05f  // [m/s] バイアス学習を行う最大速度
#define TRACTION_BIAS_ALPHA 0.01f            // バイアスの学習レート
#define TRACTION_IMU_BLEND_ALPHA 0.005f      // 非スリップ時にIMU速度を車輪速へ引き寄せるレート
#define TRACTION_VEL_LIMIT_FLOOR 0.0f        // [m/s] スリップ時の速度上限の最低値

#define SERIAL_SEND_FREQUENCY_HZ 1000
#define SERIAL_SEND_INTERVAL_MS 1  // 1000 / SERIAL_SEND_FREQUENCY_HZ

#define WINKER_STEER_THRESHOLD 0.4f  // ウィンカーを点滅させる最小ステア量

static Serial serial_left;
static Serial serial_right;
static Serial serial_steer;

static Timer steer_setup_timer;
static Timer serial_send_interval_timer;

SteerConfig steer_config;

// 受信パーサの状態管理
typedef struct {
  RecvData data;
  uint8_t recv_buf[8];
  uint8_t index;
} Protocol;

Protocol left_protocol;
Protocol right_protocol;
Protocol steer_protocol;

SendData send_data;
Drive drive;

// シリアル受信プロトコル: [0xFF][flags][angular_speed_hi][angular_speed_lo][theta_hi][theta_lo][amp_volt][0xAA]
static void Drive_RecvSerial(Serial* serial, Protocol* protocol) {
  const uint8_t HEADER = 0xFF;
  const uint8_t FOOTER = 0xAA;
  const uint8_t DATA_SIZE = 8;

  while (Serial_Available(serial)) {
    uint8_t byte = Serial_Read(serial);
    if (protocol->index == 0) {
      if (byte == HEADER) protocol->index++;
    } else if (protocol->index == DATA_SIZE + 1) {
      if (byte == FOOTER) {
        RecvData* d = &protocol->data;
        d->flags = protocol->recv_buf[0];
        d->is_enable = d->flags & 0x01;
        d->is_voltage_out_of_range = (d->flags >> 1) & 0x01;
        d->is_overheat = (d->flags >> 2) & 0x01;
        d->temperature = protocol->recv_buf[1];
        d->mech_theta = (uint16_t)((protocol->recv_buf[2] << 8) | protocol->recv_buf[3]) * 0.0001f;
        d->angular_speed = (int16_t)((protocol->recv_buf[4] << 8) | protocol->recv_buf[5]) * 0.01f;
        d->angular_accel = (int16_t)((protocol->recv_buf[6] << 8) | protocol->recv_buf[7]) * 0.1f;
      }
      protocol->index = 0;
    } else {
      protocol->recv_buf[protocol->index - 1] = byte;
      protocol->index++;
    }
  }
}

// 送信パケット形式: [0xFF][cmd][value_hi][value_lo][0xAA]
// DMA 転送中もバッファが有効であるよう、呼び出し元で static バッファを用意すること。
static void BuildPacket(uint8_t* buf, uint8_t cmd, int16_t value) {
  buf[0] = 0xFF;
  buf[1] = cmd;
  buf[2] = (uint8_t)((value >> 8) & 0xFF);
  buf[3] = (uint8_t)(value & 0xFF);
  buf[4] = 0xAA;
}

static void Drive_SendSerialSteer(uint8_t cmd, int16_t value) {
  // DMA 転送完了までバッファが必要なため static
  static uint8_t buf[5];
  BuildPacket(buf, cmd, value);
  Serial_Write(&serial_steer, buf, sizeof(buf));
}

static void Drive_SendSerialAcceleration(uint8_t cmd, int16_t left,
                                         int16_t right) {
  // 左右 DMA が並行するため別バッファが必要
  static uint8_t buf_left[5];
  static uint8_t buf_right[5];
  BuildPacket(buf_left, cmd, left);
  BuildPacket(buf_right, cmd, right);
  Serial_Write(&serial_left, buf_left, sizeof(buf_left));
  Serial_Write(&serial_right, buf_right, sizeof(buf_right));
}

static void Drive_UpdateTractionEstimator(void) {
  if (!drive.imu_valid) return;

  float dt = Timer_Read(&drive.traction_timer);
  Timer_Reset(&drive.traction_timer);
  if (dt <= 0.0f || dt > 0.1f) return;

  float imu_long_accel = MAF_Update(&drive.maf_imu_long, drive.imu_accel_x);
  float imu_lat_accel = MAF_Update(&drive.maf_imu_lat, drive.imu_accel_y);

  // 低速・低加速時にIMUのオフセットを学習する
  if (Abs(drive.speed) < 0.01) {
    drive.imu_long_bias += TRACTION_BIAS_ALPHA * (imu_long_accel - drive.imu_long_bias);
    drive.imu_lat_bias += TRACTION_BIAS_ALPHA * (imu_lat_accel - drive.imu_lat_bias);
  }

  float corrected_long_accel = imu_long_accel - drive.imu_long_bias;

  // IMU加速度を積分して車体速度を推定する
  drive.imu_velocity += corrected_long_accel * dt;

  // 車輪速がIMU推定速度を大きく上回ったらスリップと判定する
  float slip = drive.speed - drive.imu_velocity;

  drive.is_slipping = Abs(slip) > TRACTION_SLIP_VEL_THRESHOLD &&
                      drive.speed > TRACTION_BIAS_SPEED_THRESHOLD;

  if (drive.is_slipping) {
    // スリップ中は目標速度をIMU推定速度に制限する
    drive.traction_vel_limit = fmaxf(drive.imu_velocity, TRACTION_VEL_LIMIT_FLOOR);
  } else {
    // 非スリップ時は制限を解除し、imu_velocityを車輪速に緩やかに引き寄せてドリフトを防ぐ
    drive.traction_vel_limit = 1000.0f;
    drive.imu_velocity += TRACTION_IMU_BLEND_ALPHA * (drive.speed - drive.imu_velocity);
  }
}

void Drive_Update() {
  Drive_RecvSerial(&serial_left, &left_protocol);
  Drive_RecvSerial(&serial_right, &right_protocol);
  Drive_RecvSerial(&serial_steer, &steer_protocol);

  // 右モータは車体に対して逆向きに搭載されているため符号を反転して平均する
  drive.speed = (right_protocol.data.angular_speed - left_protocol.data.angular_speed) * 0.5 * WHEEL_RADIUS;
  drive.speed = MAF_Update(&drive.maf_speed, drive.speed);

  drive.accel = (right_protocol.data.angular_accel - left_protocol.data.angular_accel) * 0.5 * WHEEL_RADIUS;
  drive.accel = MAF_Update(&drive.maf_acccel, drive.accel);
  Drive_UpdateTractionEstimator();

  Lighting_SetBrake(send_data.do_brake, send_data.brake_strength, drive.speed);

  WinkerDirection winker_dir = WINKER_OFF;
  if (!send_data.do_brake && !drive.is_free && drive.speed >= 0.0f) {
    if (drive.steer_logical > WINKER_STEER_THRESHOLD) {
      winker_dir = WINKER_RIGHT;
    } else if (drive.steer_logical < -WINKER_STEER_THRESHOLD) {
      winker_dir = WINKER_LEFT;
    }
  }
  Lighting_SetWinker(winker_dir);

  if (Timer_ReadMs(&serial_send_interval_timer) <= SERIAL_SEND_INTERVAL_MS) return;
  Timer_Reset(&serial_send_interval_timer);

  if (drive.is_free) {
    // フリー状態ではバッファをリセットして送信を止める
    return;
  }

  Drive_SendSerialSteer(POSITION_HEADER, (int16_t)(send_data.steer * 1000));

  if (send_data.do_brake) {
    int16_t strength = (int16_t)(send_data.brake_strength * 10000);
    Drive_SendSerialAcceleration(BRAKE_HEADER, strength, strength);
  } else {
    Drive_SendSerialAcceleration(
        VOLTAGE_HEADER,
        (int16_t)(send_data.voltage_left * -10000),
        (int16_t)(send_data.voltage_right * 10000));  // 右モータは逆転方向が前進
  }
}

void Drive_Init(bool do_steer_setup) {
  Serial_Init(&serial_left, &huart4, 256);
  Serial_Init(&serial_right, &huart2, 256);
  Serial_Init(&serial_steer, &huart5, 256);

  Timer_Init(&steer_setup_timer);
  Timer_Init(&serial_send_interval_timer);

  MAF_Init(&drive.maf_speed, 25);
  MAF_Init(&drive.maf_acccel, 50);
  MAF_Init(&drive.maf_imu_long, 25);
  MAF_Init(&drive.maf_imu_lat, 25);
  drive.current_voltage = 0.0f;
  drive.current_target_velocity = 0.0f;
  drive.is_free = true;
  drive.is_slipping = false;
  drive.steer_logical = 0.0f;
  drive.imu_accel_x = 0.0f;
  drive.imu_accel_y = 0.0f;
  drive.imu_pitch_deg = 0.0f;
  drive.imu_roll_deg = 0.0f;
  drive.imu_valid = false;
  drive.imu_velocity = 0.0f;
  drive.traction_vel_limit = 1000.0f;
  drive.imu_long_bias = 0.0f;
  drive.imu_lat_bias = 0.0f;
  drive.traction_enabled = true;
  Timer_Init(&drive.voltage_timer);
  Timer_Init(&drive.velocity_timer);
  Timer_Init(&drive.steer_timer);
  Timer_Init(&drive.traction_timer);
  PID_Init(&drive.pid_velocity, 0.5f, 1.0f, 0.0f, -MAX_VOLTAGE, MAX_VOLTAGE);

  if (do_steer_setup) {
    printf("Steer setup: calibrating...\n");
    Timer_Reset(&steer_setup_timer);
    while (!Drive_SetupSteer());
    Flash_WriteData(FLASH_USER_START_ADDR, &steer_config, sizeof(SteerConfig));
    printf("Steer config saved to flash\n");
  } else {
    Flash_ReadData(FLASH_USER_START_ADDR, &steer_config, sizeof(SteerConfig));
    printf("Steer config loaded: min_rad=%.3f, max_rad=%.3f\n",
           steer_config.min_rad, steer_config.max_rad);
  }
}

bool Drive_SetupSteer() {
  Drive_RecvSerial(&serial_steer, &steer_protocol);

  printf("Steer setup: mech_theta=%.3f rad\n", steer_protocol.data.mech_theta);

  int16_t voltage = 0;
  uint32_t elapsed = Timer_ReadMs(&steer_setup_timer);

  // 最初の 1 秒: 正電圧で物理的な最小角を探る
  if (elapsed < 1000) {
    voltage = 5000;
    steer_config.min_rad = steer_protocol.data.mech_theta;
    // 次の 1 秒: 逆電圧で最大角を探る
  } else if (elapsed < 2000) {
    voltage = -5000;
    steer_config.max_rad = steer_protocol.data.mech_theta;
  }

  if (elapsed < 2000) {
    Drive_SendSerialSteer(VOLTAGE_HEADER, voltage);
    return false;
  }

  // キャリブレーション後、min_rad と max_rad の大小関係を確認・正規化する。
  // モータ極性がキャリブレーション時と異なる場合に対応。
  if (steer_config.min_rad > steer_config.max_rad) {
    printf("Steer setup: swapping min/max (min_rad > max_rad detected)\n");
    double tmp = steer_config.min_rad;
    steer_config.min_rad = steer_config.max_rad;
    steer_config.max_rad = tmp;
  }

  printf("Steer setup done: min_rad=%.3f, max_rad=%.3f\n",
         steer_config.min_rad, steer_config.max_rad);
  return true;
}

static void Drive_ApplyVoltageAndSteer(float voltage, float steer) {
  float voltage_left = voltage;
  float voltage_right = voltage;

  voltage_left *= (1.0f + steer * TORQUE_VECTORING_GAIN);
  voltage_right *= (1.0f - steer * TORQUE_VECTORING_GAIN);

  // 最終的なモータ出力をクリップ
  send_data.voltage_left = Constrain(voltage_left, -MAX_VOLTAGE, MAX_VOLTAGE);
  send_data.voltage_right = Constrain(voltage_right, -MAX_VOLTAGE, MAX_VOLTAGE);

  // steer を物理角度 [rad] に変換（-1=右最小, +1=左最大）
  drive.steer_logical = Constrain(steer, -1.0f, 1.0f);
  double target_rad = steer_config.min_rad +
                      (-drive.steer_logical + 1.0) / 2.0 * (steer_config.max_rad - steer_config.min_rad);

  // 急激な舵角変化を MAX_STEER_SPEED [rad/s] で制限する
  float dt_steer = Timer_Read(&drive.steer_timer);
  Timer_Reset(&drive.steer_timer);
  if (dt_steer > 0.0f && dt_steer < 0.5f) {
    double max_delta = MAX_STEER_SPEED * dt_steer;
    double delta = Constrain(target_rad - send_data.steer, -max_delta, max_delta);
    send_data.steer += delta;
  } else {
    send_data.steer = target_rad;
  }
}

void Drive_Set(float target_voltage, float voltage_rate, float steer) {
  send_data.do_brake = false;
  drive.is_free = false;

  float dt = Timer_Read(&drive.voltage_timer);
  Timer_Reset(&drive.voltage_timer);
  if (dt > 0.1f) dt = 0.0f;  // 長時間停止後の初回スパイクを防ぐ

  if (drive.current_voltage < target_voltage) {
    drive.current_voltage = drive.current_voltage + voltage_rate * dt;
  } else if (drive.current_voltage > target_voltage) {
    drive.current_voltage = drive.current_voltage - voltage_rate * dt;
  }
  float voltage_limit = Abs(target_voltage);
  drive.current_voltage = Constrain(drive.current_voltage, -voltage_limit, voltage_limit);

  Drive_ApplyVoltageAndSteer(drive.current_voltage, steer);
}

void Drive_SetVelocity(float target_velocity, float acceleration, float steer) {
  send_data.do_brake = false;
  drive.is_free = false;

  float dt = Timer_Read(&drive.velocity_timer);
  Timer_Reset(&drive.velocity_timer);
  if (dt > 0.1f) dt = 0.0f;  // 長時間停止後の初回スパイクを防ぐ

  float vel_limit = drive.traction_enabled ? drive.traction_vel_limit : target_velocity;
  float clamped_target = fminf(target_velocity, vel_limit);

  float abs_accel = Abs(acceleration);
  if (drive.current_target_velocity < clamped_target) {
    drive.current_target_velocity += abs_accel * dt;
  } else if (drive.current_target_velocity > clamped_target) {
    drive.current_target_velocity -= abs_accel * dt;
  }
  drive.current_target_velocity = Constrain(drive.current_target_velocity,
                                            -Abs(clamped_target), Abs(clamped_target));

  float voltage_output = PID_Update(&drive.pid_velocity,
                                   drive.current_target_velocity, drive.speed);
  Drive_ApplyVoltageAndSteer(voltage_output, steer);
}

void Drive_Brake(float deceleration, float steer) {
  Drive_ApplyVoltageAndSteer(0.0f, steer);
  send_data.do_brake = true;
  send_data.brake_strength = Constrain(deceleration, 0.0f, MAX_VOLTAGE);
  drive.current_voltage = 0.0f;
  drive.current_target_velocity = 0.0f;
  drive.is_free = false;
  PID_Reset(&drive.pid_velocity);
}

void Drive_Free() {
  drive.is_free = true;
  send_data.do_brake = false;
  drive.current_voltage = 0.0f;
  drive.current_target_velocity = 0.0f;
  PID_Reset(&drive.pid_velocity);
}

float Drive_GetSpeed() { return drive.speed; }

float Drive_GetAccel() { return drive.accel; }

float Drive_GetSteer() { return drive.steer_logical; }

uint8_t Drive_GetLeftMotorTemperature() { return left_protocol.data.temperature; }
uint8_t Drive_GetRightMotorTemperature() { return right_protocol.data.temperature; }
uint8_t Drive_GetSteerMotorTemperature() { return steer_protocol.data.temperature; }

void Drive_SetImuData(float accel_x, float accel_y, float pitch_deg, float roll_deg) {
  // app 層から渡された IMU 値を保持し、推定器は drive 層の中だけで回す。
  drive.imu_accel_x = accel_x;
  drive.imu_accel_y = accel_y;
  drive.imu_pitch_deg = pitch_deg;
  drive.imu_roll_deg = roll_deg;
  drive.imu_valid = true;
}

static bool Drive_RecvDataHasError(const RecvData* data) {
  return data->is_voltage_out_of_range || data->is_overheat;
}

bool Drive_HasError() {
  return Drive_RecvDataHasError(&left_protocol.data) ||
         Drive_RecvDataHasError(&right_protocol.data) ||
         Drive_RecvDataHasError(&steer_protocol.data);
}

void Drive_SetTractionEnabled(bool enabled) { drive.traction_enabled = enabled; }
