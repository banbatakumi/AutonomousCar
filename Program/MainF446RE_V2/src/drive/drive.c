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

#define TRACTION_BIAS_ALPHA 0.01f           // 静止時IMUバイアス学習レート
#define TRACTION_BIAS_SPEED_THRESHOLD 0.5f  // [m/s] スリップ判定を行う最低速度
#define TRACTION_VOLT_REDUCE_RATE 4.0f      // [V/s] スリップ中の電圧上限削減レート
#define TRACTION_VOLT_RECOVER_RATE 2.0f     // [V/s] 非スリップ時の電圧上限回復レート

// カルマンフィルタ速度推定器のパラメータ
// Q: プロセスノイズ分散 [m²/s²/s]。未モデル化加速度・IMUドリフトの大きさで調整。
// R: 観測ノイズ分散 [m²/s²]。オドメトリの計測誤差の大きさで調整。
#define KV_PROCESS_NOISE_Q 0.5f
#define KV_MEAS_NOISE_R 1.0f
#define KV_SLIP_THRESHOLD 0.15f      // [m/s] イノベーション閾値（空転スリップ検知）
#define KV_SLIP_DEBOUNCE_SAMPLES 50  // スリップ確定サンプル数 (≈5ms @ 10kHz)

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

  // 静止時のみIMUバイアスを学習する（重力補正後の残留オフセット吸収）
  if (Abs(drive.speed) < 0.05f) {
    drive.imu_long_bias += TRACTION_BIAS_ALPHA * (drive.imu_accel_x - drive.imu_long_bias);
  }

  float body_accel = drive.imu_accel_x - drive.imu_long_bias;

  // カルマンフィルタ: IMU加速度を入力、オドメトリを観測として速度融合。
  // ゲーティングにより、|innovation| > slip_threshold のとき観測更新をスキップするため
  // 持続スリップ中も innovation が大きいまま維持される。
  drive.kf_velocity = KalmanVelocity_Update(&drive.kalman_vel, body_accel, dt, drive.speed);

  // innovation > 0 かつ閾値超え → 車輪が車体より速く回転 → 空転スリップ
  bool slip_candidate = drive.kalman_vel.innovation > KV_SLIP_THRESHOLD &&
                        Abs(drive.speed) > TRACTION_BIAS_SPEED_THRESHOLD;
  if (slip_candidate) {
    if (drive.slip_debounce_count < KV_SLIP_DEBOUNCE_SAMPLES) {
      drive.slip_debounce_count++;
    }
  } else {
    drive.slip_debounce_count = 0;
  }
  drive.is_slipping = drive.slip_debounce_count >= KV_SLIP_DEBOUNCE_SAMPLES;

  if (drive.is_slipping) {
    drive.traction_volt_limit -= TRACTION_VOLT_REDUCE_RATE * dt;
    if (drive.traction_volt_limit < 0.0f) drive.traction_volt_limit = 0.0f;
  } else {
    drive.traction_volt_limit += TRACTION_VOLT_RECOVER_RATE * dt;
    if (drive.traction_volt_limit > MAX_VOLTAGE) drive.traction_volt_limit = MAX_VOLTAGE;
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
  if (!send_data.do_brake && !drive.is_free) {
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
  drive.imu_long_bias = 0.0f;
  drive.traction_volt_limit = MAX_VOLTAGE;
  drive.slip_debounce_count = 0;
  drive.imu_gyro_z = 0.0f;
  drive.sc_yaw_rate_error = 0.0f;
  drive.traction_enabled = true;
  drive.stability_enabled = true;
  KalmanVelocity_Init(&drive.kalman_vel, KV_PROCESS_NOISE_Q, KV_MEAS_NOISE_R, KV_SLIP_THRESHOLD);
  drive.kf_velocity = 0.0f;
  Timer_Init(&drive.voltage_timer);
  Timer_Init(&drive.velocity_timer);
  Timer_Init(&drive.steer_timer);
  Timer_Init(&drive.traction_timer);
  PID_Init(&drive.pid_velocity, 1.0f, 1.5f, 0.0f, -MAX_VOLTAGE, MAX_VOLTAGE);

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

// スタビリティコントロール: 実ヨーレートと自転車モデル目標ヨーレートの差を
// 差動電圧で打ち消す。低速 (< SC_MIN_SPEED) では非作動。
static void Drive_UpdateStabilityControl(void) {
  if (!drive.stability_enabled || !drive.imu_valid) return;
  if (Abs(drive.kf_velocity) < SC_MIN_SPEED) {
    drive.sc_yaw_rate_error = 0.0f;
    return;
  }

  float delta_rad = drive.steer_logical * MAX_STEER_ANGLE_RAD;
  float desired_yaw_rate = drive.kf_velocity * tanf(delta_rad) / WHEEL_BASE;
  float actual_yaw_rate = drive.imu_gyro_z * (float)DEG_TO_RAD;
  drive.sc_yaw_rate_error = actual_yaw_rate - desired_yaw_rate;

  float correction = Constrain(drive.sc_yaw_rate_error * SC_YAW_RATE_GAIN,
                               -SC_MAX_CORRECTION, SC_MAX_CORRECTION);

  if (drive.kf_velocity < 0) correction = -correction;  // 後退時は補正方向を逆転
  send_data.voltage_left = Constrain(send_data.voltage_left - correction, -MAX_VOLTAGE, MAX_VOLTAGE);
  send_data.voltage_right = Constrain(send_data.voltage_right + correction, -MAX_VOLTAGE, MAX_VOLTAGE);
}

static void Drive_ApplyVoltageAndSteer(float voltage, float steer) {
  if (drive.traction_enabled && voltage > drive.traction_volt_limit) {
    voltage = drive.traction_volt_limit;
  }

  float voltage_left = voltage;
  float voltage_right = voltage;

  voltage_left *= (1.0f + steer * TORQUE_VECTORING_GAIN);
  voltage_right *= (1.0f - steer * TORQUE_VECTORING_GAIN);

  // 最終的なモータ出力をクリップ
  send_data.voltage_left = Constrain(voltage_left, -MAX_VOLTAGE, MAX_VOLTAGE);
  send_data.voltage_right = Constrain(voltage_right, -MAX_VOLTAGE, MAX_VOLTAGE);

  Drive_UpdateStabilityControl();

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

  float abs_accel = Abs(acceleration);
  if (drive.current_target_velocity < target_velocity) {
    drive.current_target_velocity += abs_accel * dt;
  } else if (drive.current_target_velocity > target_velocity) {
    drive.current_target_velocity -= abs_accel * dt;
  }
  drive.current_target_velocity = Constrain(drive.current_target_velocity,
                                            -Abs(target_velocity), Abs(target_velocity));

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
  drive.traction_volt_limit = MAX_VOLTAGE;
  drive.slip_debounce_count = 0;
  drive.is_slipping = false;
  KalmanVelocity_Reset(&drive.kalman_vel, 0.0f);
  drive.kf_velocity = 0.0f;
  PID_Reset(&drive.pid_velocity);
}

float Drive_GetSpeed() { return drive.speed; }

float Drive_GetAccel() { return drive.accel; }

float Drive_GetSteer() { return drive.steer_logical; }

uint8_t Drive_GetLeftMotorTemperature() { return left_protocol.data.temperature; }
uint8_t Drive_GetRightMotorTemperature() { return right_protocol.data.temperature; }
uint8_t Drive_GetSteerMotorTemperature() { return steer_protocol.data.temperature; }

void Drive_SetImuData(float accel_x, float accel_y, float pitch_deg, float roll_deg, float gyro_z_dps) {
  // app 層から渡された IMU 値を保持し、推定器は drive 層の中だけで回す。
  drive.imu_accel_x = accel_x;
  drive.imu_accel_y = accel_y;
  drive.imu_pitch_deg = pitch_deg;
  drive.imu_roll_deg = roll_deg;
  drive.imu_gyro_z = gyro_z_dps;
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

bool Drive_IsSlipping() { return drive.is_slipping; }

float Drive_GetKfVelocity() { return drive.kf_velocity; }

float Drive_GetKfInnovation() { return drive.kalman_vel.innovation; }

void Drive_SetStabilityEnabled(bool enabled) { drive.stability_enabled = enabled; }

float Drive_GetYawRateError(void) { return drive.sc_yaw_rate_error; }
