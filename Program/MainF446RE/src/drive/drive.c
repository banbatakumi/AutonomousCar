#include "drive.h"

#include <math.h>

#include "flash.h"

// モータコントローラへのコマンドヘッダ
#define POSITION_HEADER 0xFD
#define TORQUE_HEADER 0xFC
#define BRAKE_HEADER 0xFB

#define WHEEL_RADIUS 0.0275f  // [m]

#define SERIAL_SEND_FREQUENCY_HZ 1000
#define SERIAL_SEND_INTERVAL_MS 1  // 1000 / SERIAL_SEND_FREQUENCY_HZ

// ブレーキ点滅パラメータ（急ブレーキ時）
#define BRAKE_BLINK_PERIOD_MS 200
#define BRAKE_BLINK_HALF_PERIOD_MS 100  // BRAKE_BLINK_PERIOD_MS / 2

// 急ブレーキ点滅を開始する速度閾値 [m/s]
#define BRAKE_BLINK_SPEED_THRESHOLD 0.5f

// ウィンカー点滅パラメータ
#define WINKER_BLINK_HALF_PERIOD_MS 200  // 400ms 周期の半分 [ms]
#define WINKER_STEER_THRESHOLD 0.4f      // 点滅を開始する最小ステア量

Serial serial_left;
Serial serial_right;
Serial serial_steer;

PwmOut brake_led;
PwmOut winker_left_led;
PwmOut winker_right_led;

Timer steer_setup_timer;
Timer serial_send_interval_timer;

SteerConfig steer_config;

// 受信パーサの状態管理
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

// シリアル受信プロトコル: [0xFF][flags][angular_speed_hi][angular_speed_lo][theta_hi][theta_lo][amp_volt][0xAA]
static void Drive_RecvSerial(Serial* serial, Protocol* protocol) {
  const uint8_t HEADER = 0xFF;
  const uint8_t FOOTER = 0xAA;
  const uint8_t DATA_SIZE = 6;

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
        d->angular_speed = (int16_t)((protocol->recv_buf[1] << 8) | protocol->recv_buf[2]) * 0.01f;
        d->mech_theta = (int16_t)((protocol->recv_buf[3] << 8) | protocol->recv_buf[4]) * 0.001f;
        d->amp_volt = protocol->recv_buf[5] * 0.1f;
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

static void UpdateBrakeLed(void) {
  if (!send_data.do_brake) {
    PwmOut_Write(&brake_led, 0.05f);  // 非ブレーキ時は薄く点灯
    return;
  }

  // 急ブレーキかつ走行中: 200ms 周期で点滅
  if (send_data.brake_strength >= 1.0f &&
      drive.speed > BRAKE_BLINK_SPEED_THRESHOLD) {
    float elapsed_ms = Timer_ReadMs(&drive.brake_led_timer);
    if (elapsed_ms >= BRAKE_BLINK_PERIOD_MS) {
      Timer_Reset(&drive.brake_led_timer);
      elapsed_ms = 0;
    }
    PwmOut_Write(&brake_led, elapsed_ms < BRAKE_BLINK_HALF_PERIOD_MS ? 1.0f : 0.0f);
  } else {
    PwmOut_Write(&brake_led, 1.0f);
  }
}

static void UpdateWinkerLed(void) {
  // ブレーキ中・フリー状態・直進時は消灯
  if (send_data.do_brake || drive.is_free ||
      Abs(drive.steer_logical) < WINKER_STEER_THRESHOLD || drive.speed < 0) {
    PwmOut_Write(&winker_left_led, 0.0f);
    PwmOut_Write(&winker_right_led, 0.0f);
    return;
  }

  if (Timer_ReadMs(&drive.winker_timer) >= WINKER_BLINK_HALF_PERIOD_MS) {
    drive.winker_state = !drive.winker_state;
    Timer_Reset(&drive.winker_timer);
  }

  float brightness = drive.winker_state ? 1.0f : 0.0f;
  if (drive.steer_logical > 0.0f) {
    PwmOut_Write(&winker_left_led, 0.0f);
    PwmOut_Write(&winker_right_led, brightness);
  } else {
    PwmOut_Write(&winker_left_led, brightness);
    PwmOut_Write(&winker_right_led, 0.0f);
  }
}

void Drive_Update() {
  Drive_RecvSerial(&serial_left, &left_protocol);
  Drive_RecvSerial(&serial_right, &right_protocol);
  Drive_RecvSerial(&serial_steer, &steer_protocol);

  // 右モータは車体に対して逆向きに搭載されているため符号を反転して平均する
  drive.speed = (right_protocol.data.angular_speed - left_protocol.data.angular_speed) * 0.5 *
                WHEEL_RADIUS;
  drive.speed = MAF_Update(&drive.maf_speed, drive.speed);

  UpdateBrakeLed();
  UpdateWinkerLed();

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
    int16_t strength = (int16_t)(send_data.brake_strength * 100);
    Drive_SendSerialAcceleration(BRAKE_HEADER, strength, strength);
  } else {
    Drive_SendSerialAcceleration(
        TORQUE_HEADER,
        (int16_t)(send_data.acceleration_left * 100),
        (int16_t)(send_data.acceleration_right * -100));  // 右モータは逆転方向が前進
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
  drive.current_target_velocity = 0.0f;
  drive.is_free = true;
  drive.steer_logical = 0.0f;
  drive.winker_state = false;
  Timer_Init(&drive.accel_timer);
  Timer_Init(&drive.velocity_timer);
  Timer_Init(&drive.steer_timer);
  Timer_Init(&drive.brake_led_timer);
  Timer_Init(&drive.winker_timer);
  PID_Init(&drive.pid_velocity, 1.0f, 2.0f, 0.0f, -MAX_ACCELERATION, MAX_ACCELERATION);

  PwmOut_Init(&brake_led, &htim3, TIM_CHANNEL_4);
  PwmOut_Init(&winker_left_led, &htim3, TIM_CHANNEL_3);
  PwmOut_Init(&winker_right_led, &htim3, TIM_CHANNEL_2);

  // 起動確認用のウィンカー点滅
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 50; j++) {
      PwmOut_Write(&winker_left_led, j * 0.01f);
      PwmOut_Write(&winker_right_led, j * 0.01f);
      HAL_Delay(1);
    }
    HAL_Delay(150);
    PwmOut_Write(&winker_left_led, 0.0f);
    PwmOut_Write(&winker_right_led, 0.0f);
    HAL_Delay(200);
  }

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

  int16_t torque = 0;
  uint32_t elapsed = Timer_ReadMs(&steer_setup_timer);

  // 最初の 1 秒: 正トルクで物理的な最小角を探る
  if (elapsed < 1000) {
    torque = 250;
    steer_config.min_rad = steer_protocol.data.mech_theta;
    // 次の 1 秒: 逆トルクで最大角を探る
  } else if (elapsed < 2000) {
    torque = -250;
    steer_config.max_rad = steer_protocol.data.mech_theta;
  }

  if (elapsed < 2000) {
    Drive_SendSerialSteer(TORQUE_HEADER, torque);
    return false;
  }

  printf("Steer setup done: min_rad=%.3f, max_rad=%.3f\n",
         steer_config.min_rad, steer_config.max_rad);
  return true;
}

// 電子ディファレンシャル: steer > 0（左旋回）なら右を減速、steer < 0 なら左を減速。
static void Drive_ApplyAccelerationAndSteer(float acceleration, float steer) {
  if (steer > 0.0f) {
    send_data.acceleration_left = Constrain(acceleration, -MAX_ACCELERATION, MAX_ACCELERATION);
    send_data.acceleration_right = Constrain(acceleration * (1.0f - steer * DIFFERENTIAL),
                                             -MAX_ACCELERATION, MAX_ACCELERATION);
  } else {
    send_data.acceleration_left = Constrain(acceleration * (1.0f + steer * DIFFERENTIAL),
                                            -MAX_ACCELERATION, MAX_ACCELERATION);
    send_data.acceleration_right = Constrain(acceleration, -MAX_ACCELERATION, MAX_ACCELERATION);
  }

  // steer を物理角度 [rad] に変換（-1=右最大, +1=左最大）
  drive.steer_logical = Constrain(steer, -1.0f, 1.0f);
  steer = -drive.steer_logical;
  double target_rad = steer_config.min_rad +
                      (steer + 1.0) / 2.0 * (steer_config.max_rad - steer_config.min_rad);

  // 急激な舵角変化を MAX_STEER_SPEED [rad/s] で制限する
  float dt = Timer_Read(&drive.steer_timer);
  Timer_Reset(&drive.steer_timer);
  if (dt > 0.0f && dt < 0.5f) {
    double max_delta = MAX_STEER_SPEED * dt;
    double delta = Constrain(target_rad - send_data.steer, -max_delta, max_delta);
    send_data.steer += delta;
  } else {
    send_data.steer = target_rad;
  }
}

void Drive_Set(float max_acceleration, float acceleration_rate, float steer) {
  send_data.do_brake = false;
  drive.is_free = false;

  float dt = Timer_Read(&drive.accel_timer);
  Timer_Reset(&drive.accel_timer);
  if (dt > 0.5f) dt = 0.0f;  // 長時間停止後の初回スパイクを防ぐ

  if (drive.current_acceleration < max_acceleration) {
    drive.current_acceleration =
        fminf(drive.current_acceleration + acceleration_rate * dt, max_acceleration);
  } else if (drive.current_acceleration > max_acceleration) {
    drive.current_acceleration =
        fmaxf(drive.current_acceleration - acceleration_rate * dt, max_acceleration);
  }

  Drive_ApplyAccelerationAndSteer(drive.current_acceleration, steer);
}

void Drive_SetVelocity(float target_velocity, float acceleration, float steer) {
  send_data.do_brake = false;
  drive.is_free = false;

  float dt = Timer_Read(&drive.velocity_timer);
  Timer_Reset(&drive.velocity_timer);
  if (dt > 0.1f) dt = 0.0f;  // 長時間停止後の初回スパイクを防ぐ

  if (drive.current_target_velocity < target_velocity) {
    drive.current_target_velocity =
        fminf(drive.current_target_velocity + acceleration * dt, target_velocity);
  } else if (drive.current_target_velocity > target_velocity) {
    drive.current_target_velocity =
        fmaxf(drive.current_target_velocity - acceleration * dt, target_velocity);
  }

  float accel_output = PID_Update(&drive.pid_velocity,
                                  drive.current_target_velocity, drive.speed);
  Drive_ApplyAccelerationAndSteer(accel_output, steer);
}

void Drive_Brake(float deceleration) {
  send_data.do_brake = true;
  send_data.brake_strength = Constrain(deceleration, 0.0f, MAX_ACCELERATION);
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
