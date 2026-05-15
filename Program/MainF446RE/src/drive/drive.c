#include "drive.h"

#include <math.h>
#include <stdio.h>

#include "flash.h"
#include "lighting.h"

// モータコントローラへのコマンドヘッダ
#define POSITION_HEADER 0xFD
#define TORQUE_HEADER 0xFC
#define BRAKE_HEADER 0xFB

#define WHEEL_RADIUS 0.0275f  // [m]

#define TRACTION_GRAVITY 9.80665f
#define TRACTION_DEFAULT_MU 0.35f
#define TRACTION_MIN_MU 0.20f
#define TRACTION_MAX_MU 1.20f
#define TRACTION_SAFETY_FACTOR 0.85f
#define TRACTION_BIAS_SPEED_THRESHOLD 0.05f
#define TRACTION_BIAS_ALPHA 0.02f
#define TRACTION_MU_RISE_ALPHA 0.02f
#define TRACTION_MU_FALL_ALPHA 0.20f
#define TRACTION_SLIP_ACCEL_MARGIN 0.60f
#define TRACTION_LIMIT_FLOOR 0.20f

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
  uint8_t recv_buf[7];
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
  const uint8_t DATA_SIZE = 7;

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
        d->mech_theta = (uint16_t)((protocol->recv_buf[1] << 8) | protocol->recv_buf[2]) * 0.0001f;
        d->angular_speed = (int16_t)((protocol->recv_buf[3] << 8) | protocol->recv_buf[4]) * 0.01f;
        d->angular_accel = (int16_t)((protocol->recv_buf[5] << 8) | protocol->recv_buf[6]) * 0.1f;
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
  if (!drive.imu_valid) {
    // IMU がまだ入っていない間は、車輪由来の情報だけでは推定を更新しない。
    return;
  }

  // IMU の前後・横加速度を軽く平滑化して、瞬間ノイズの影響を減らす。
  float imu_long_accel = MAF_Update(&drive.maf_imu_long, drive.imu_accel_x);
  float imu_lat_accel = MAF_Update(&drive.maf_imu_lat, drive.imu_accel_y);

  // 低速かつほぼ無加速のときは、IMU の残留オフセットを少しずつ学習する。
  if (Abs(drive.speed) < TRACTION_BIAS_SPEED_THRESHOLD && Abs(drive.accel) < TRACTION_BIAS_SPEED_THRESHOLD) {
    drive.imu_long_bias += TRACTION_BIAS_ALPHA * (imu_long_accel - drive.imu_long_bias);
    drive.imu_lat_bias += TRACTION_BIAS_ALPHA * (imu_lat_accel - drive.imu_lat_bias);
  }

  // 静止時に学習したバイアスを引いて、実際の車体加速度だけを残す。
  float corrected_long_accel = imu_long_accel - drive.imu_long_bias;
  float corrected_lat_accel = imu_lat_accel - drive.imu_lat_bias;

  // IMU から見た摩擦の使われ方を、前後・横加速度の合成から推定する。
  float observed_mu = sqrtf(corrected_long_accel * corrected_long_accel +
                            corrected_lat_accel * corrected_lat_accel) /
                      TRACTION_GRAVITY;
  observed_mu = Constrain(observed_mu, TRACTION_MIN_MU, TRACTION_MAX_MU);

  // 車輪側の加速度と IMU 側の加速度が大きくずれたら、空転/スリップの疑いがある。
  float wheel_mu = Abs(drive.accel) / TRACTION_GRAVITY;
  bool is_slipping = Abs(drive.accel - corrected_long_accel) > TRACTION_SLIP_ACCEL_MARGIN &&
                     Abs(drive.speed) > TRACTION_BIAS_SPEED_THRESHOLD;

  printf("drive.accel=%.2f, corrected_long_accel=%.2f, wheel_mu=%.2f, observed_mu=%.2f, is_slipping=%d\n",
         drive.accel, corrected_long_accel, wheel_mu, observed_mu, is_slipping);

  // スリップ時は安全側に下げ、通常時は観測値を少しずつ追従する。
  float candidate_mu = is_slipping ? fminf(observed_mu, wheel_mu)
                                   : fmaxf(observed_mu, wheel_mu);

  if (is_slipping) {
    drive.traction_mu += TRACTION_MU_FALL_ALPHA * (candidate_mu - drive.traction_mu);
  } else {
    drive.traction_mu += TRACTION_MU_RISE_ALPHA * (candidate_mu - drive.traction_mu);
  }
  drive.traction_mu = MAF_Update(&drive.maf_mu_estimate,
                                 Constrain(drive.traction_mu, TRACTION_MIN_MU, TRACTION_MAX_MU));

  // 推定した μ から、今その瞬間に使える縦方向加速度の上限を決める。
  float available_accel = drive.traction_mu * TRACTION_GRAVITY;
  float lateral_abs = Abs(corrected_lat_accel);
  float longitudinal_limit = sqrtf(fmaxf(0.0f, available_accel * available_accel -
                                                   lateral_abs * lateral_abs));
  drive.traction_accel_limit = Constrain(longitudinal_limit * TRACTION_SAFETY_FACTOR,
                                         TRACTION_LIMIT_FLOOR,
                                         available_accel * TRACTION_SAFETY_FACTOR);
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
    Serial_Reset(&serial_left);
    Serial_Reset(&serial_right);
    Serial_Reset(&serial_steer);
    return;
  }

  Drive_SendSerialSteer(POSITION_HEADER, (int16_t)(send_data.steer * 1000));

  if (send_data.do_brake) {
    int16_t strength = (int16_t)(send_data.brake_strength * 10000);
    Drive_SendSerialAcceleration(BRAKE_HEADER, strength, strength);
  } else {
    Drive_SendSerialAcceleration(
        TORQUE_HEADER,
        (int16_t)(send_data.torque_left * -10000),
        (int16_t)(send_data.torque_right * 10000));  // 右モータは逆転方向が前進
  }
}

void Drive_Init(bool do_steer_setup) {
  Serial_Init(&serial_left, &huart4, 256);
  Serial_Init(&serial_right, &huart2, 256);
  Serial_Init(&serial_steer, &huart5, 256);

  Timer_Init(&steer_setup_timer);
  Timer_Init(&serial_send_interval_timer);

  MAF_Init(&drive.maf_speed, 10);
  MAF_Init(&drive.maf_acccel, 10);
  MAF_Init(&drive.maf_imu_long, 10);
  MAF_Init(&drive.maf_imu_lat, 10);
  MAF_Init(&drive.maf_mu_estimate, 10);
  drive.current_torque = 0.0f;
  drive.current_target_velocity = 0.0f;
  drive.is_free = true;
  drive.steer_logical = 0.0f;
  drive.imu_accel_x = 0.0f;
  drive.imu_accel_y = 0.0f;
  drive.imu_pitch_deg = 0.0f;
  drive.imu_roll_deg = 0.0f;
  drive.imu_valid = false;
  drive.traction_mu = TRACTION_DEFAULT_MU;
  drive.traction_accel_limit = TRACTION_DEFAULT_MU * TRACTION_GRAVITY * TRACTION_SAFETY_FACTOR;
  drive.imu_long_bias = 0.0f;
  drive.imu_lat_bias = 0.0f;
  Timer_Init(&drive.torque_timer);
  Timer_Init(&drive.velocity_timer);
  Timer_Init(&drive.steer_timer);
  PID_Init(&drive.pid_velocity, 0.5f, 1.0f, 0.0f, -MAX_TORQUE, MAX_TORQUE);

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

  int16_t torque = 0;
  uint32_t elapsed = Timer_ReadMs(&steer_setup_timer);

  // 最初の 1 秒: 正トルクで物理的な最小角を探る
  if (elapsed < 1000) {
    torque = 5000;
    steer_config.min_rad = steer_protocol.data.mech_theta;
    // 次の 1 秒: 逆トルクで最大角を探る
  } else if (elapsed < 2000) {
    torque = -5000;
    steer_config.max_rad = steer_protocol.data.mech_theta;
  }

  if (elapsed < 2000) {
    Drive_SendSerialSteer(TORQUE_HEADER, torque);
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

static void Drive_ApplyTorqueAndSteer(float torque, float steer) {
  // 左右のトルク配分によるヨーモーメント制御
  float torque_left = torque;
  float torque_right = torque;

  if (steer > 0.0f) {
    // 左旋回: 右を減速
    torque_left *= (1.0f + steer * DIFFERENTIAL);
    torque_right *= (1.0f - steer * DIFFERENTIAL);
  } else {
    // 右旋回: 左を減速
    torque_left *= (1.0f + steer * DIFFERENTIAL);
    torque_right *= (1.0f - steer * DIFFERENTIAL);
  }

  // 最終的なモータ出力をクリップ
  send_data.torque_left = Constrain(torque_left, -MAX_TORQUE, MAX_TORQUE);
  send_data.torque_right = Constrain(torque_right, -MAX_TORQUE, MAX_TORQUE);

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

void Drive_Set(float target_torque, float torque_rate, float steer) {
  send_data.do_brake = false;
  drive.is_free = false;

  float dt = Timer_Read(&drive.torque_timer);
  Timer_Reset(&drive.torque_timer);
  if (dt > 0.1f) dt = 0.0f;  // 長時間停止後の初回スパイクを防ぐ

  if (drive.current_torque < target_torque) {
    drive.current_torque = drive.current_torque + torque_rate * dt;
  } else if (drive.current_torque > target_torque) {
    drive.current_torque = drive.current_torque - torque_rate * dt;
  }
  float torque_limit = Abs(target_torque);
  drive.current_torque = Constrain(drive.current_torque, -torque_limit, torque_limit);

  Drive_ApplyTorqueAndSteer(drive.current_torque, steer);
}

void Drive_SetVelocity(float target_velocity, float acceleration, float steer) {
  send_data.do_brake = false;
  drive.is_free = false;

  float dt = Timer_Read(&drive.velocity_timer);
  Timer_Reset(&drive.velocity_timer);
  if (dt > 0.1f) dt = 0.0f;  // 長時間停止後の初回スパイクを防ぐ

  float accel_limit = drive.traction_accel_limit;
  if (accel_limit <= 0.0f) {
    accel_limit = acceleration;
  }
  float effective_acceleration = Constrain(Abs(acceleration), 0.0f, accel_limit);

  if (drive.current_target_velocity < target_velocity) {
    drive.current_target_velocity = drive.current_target_velocity + effective_acceleration * dt;
  } else if (drive.current_target_velocity > target_velocity) {
    drive.current_target_velocity = drive.current_target_velocity - effective_acceleration * dt;
  }
  float target_velocity_limit = Abs(target_velocity);
  drive.current_target_velocity = Constrain(drive.current_target_velocity, -target_velocity_limit,
                                            target_velocity_limit);

  float torque_output = PID_Update(&drive.pid_velocity,
                                   drive.current_target_velocity, drive.speed);
  Drive_ApplyTorqueAndSteer(torque_output, steer);
}

void Drive_Brake(float deceleration, float steer) {
  Drive_ApplyTorqueAndSteer(0.0f, steer);
  send_data.do_brake = true;
  send_data.brake_strength = Constrain(deceleration, 0.0f, MAX_TORQUE);
  drive.current_torque = 0.0f;
  drive.current_target_velocity = 0.0f;
  drive.is_free = false;
  PID_Reset(&drive.pid_velocity);
}

void Drive_Free() {
  drive.is_free = true;
  send_data.do_brake = false;
  drive.current_torque = 0.0f;
  drive.current_target_velocity = 0.0f;
  PID_Reset(&drive.pid_velocity);
}

float Drive_GetSpeed() { return drive.speed; }

void Drive_SetImuData(float accel_x, float accel_y, float pitch_deg, float roll_deg) {
  // app 層から渡された IMU 値を保持し、推定器は drive 層の中だけで回す。
  drive.imu_accel_x = accel_x;
  drive.imu_accel_y = accel_y;
  drive.imu_pitch_deg = pitch_deg;
  drive.imu_roll_deg = roll_deg;
  drive.imu_valid = true;
}

float Drive_GetTractionMu() { return drive.traction_mu; }

static bool Drive_RecvDataHasError(const RecvData* data) {
  return data->is_voltage_out_of_range || data->is_overheat;
}

bool Drive_HasError() {
  return Drive_RecvDataHasError(&left_protocol.data) ||
         Drive_RecvDataHasError(&right_protocol.data) ||
         Drive_RecvDataHasError(&steer_protocol.data);
}
