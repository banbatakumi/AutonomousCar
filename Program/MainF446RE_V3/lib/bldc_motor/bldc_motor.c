#include "bldc_motor.h"

#include "crc8.h"

#define BLDC_MOTOR_HEADER 0xAA

// 指令値のスケール。MD側のSERIAL_COMMANDSテーブルのscaleと一致させること
#define BLDC_MOTOR_SCALE_SPEED 0.01f
#define BLDC_MOTOR_SCALE_POSITION 0.001f
#define BLDC_MOTOR_SCALE_TORQUE 0.001f
#define BLDC_MOTOR_SCALE_TORQUE_NM 0.0001f
#define BLDC_MOTOR_SCALE_BRAKE 0.001f
#define BLDC_MOTOR_SCALE_BRAKE_NM 0.0001f

// 状態フレームのスケール。MD側のSendSerialと一致させること (theta: 0.1mrad, speed: 0.01rad/s, iq: 1mA)
#define BLDC_MOTOR_RX_SCALE_THETA 0.0001f
#define BLDC_MOTOR_RX_SCALE_SPEED 0.01f
#define BLDC_MOTOR_RX_SCALE_IQ 0.001f

// トルク上限のスケール。MD側の SERIAL_TORQUE_LIMIT_SCALE と必ず一致させること。
// 上限値なので符号を持たせず uint8 のレンジをすべて正側に使う (0〜0.255 N・m)。
// モータ最大 0.1557 N・m に対してレンジを詰めてあり、分解能 1mN・m で 155段階を取れる
// (電流センサの量子化 0.0081A/LSB = 0.158mN・m と釣り合う粒度)。
#define BLDC_MOTOR_SCALE_TORQUE_LIMIT 0.001f

static void SetCommand(BldcMotor* obj, uint8_t header, float value, float scale) {
  obj->tx_enabled = true;
  obj->tx_header = header;
  obj->tx_raw = (int16_t)(value / scale);
}

// 上限値として使う量なので、切り捨て (安全側) で量子化し、レンジ外は飽和させる。
// uint8へのキャスト任せだと 0.3 N・m が 0.045 N・m に化けるなど、意図と無関係な値になる
static uint8_t EncodeLimit(float value, float scale) {
  if (value <= 0.0f) {
    return 0;
  }
  float raw = value / scale;
  if (raw >= 255.0f) {
    return 255;
  }
  return (uint8_t)raw;
}

static void Transmit(BldcMotor* obj) {
  if (Timer_ReadUs(&obj->tx_timer) < BLDC_MOTOR_TX_INTERVAL_US) {
    return;
  }
  Timer_Reset(&obj->tx_timer);

  if (!obj->tx_enabled) {
    return;
  }

  obj->tx_frame[0] = BLDC_MOTOR_HEADER;
  obj->tx_frame[1] = obj->tx_header;
  obj->tx_frame[2] = (obj->tx_raw >> 8) & 0xFF;
  obj->tx_frame[3] = obj->tx_raw & 0xFF;
  obj->tx_frame[4] = obj->torque_limit_raw;
  // CRCの対象はヘッダを除く [1]〜[4]。MD側と範囲を必ず一致させること
  obj->tx_frame[5] = Crc8(&obj->tx_frame[1], 4);

  Serial_Write(obj->serial, obj->tx_frame, BLDC_MOTOR_TX_FRAME_SIZE);
}

static void ParseStatusFrame(BldcMotor* obj) {
  uint8_t status = obj->rx_body[0];
  obj->is_overcurrent = (status >> 3) & 0x01;
  obj->is_overheat = (status >> 2) & 0x01;
  obj->is_voltage_out_of_range = (status >> 1) & 0x01;
  obj->is_running = status & 0x01;
  obj->temperature_c = obj->rx_body[1];

  uint16_t theta_raw = ((uint16_t)obj->rx_body[2] << 8) | obj->rx_body[3];
  int16_t speed_raw = (int16_t)(((uint16_t)obj->rx_body[4] << 8) | obj->rx_body[5]);
  int16_t iq_raw = (int16_t)(((uint16_t)obj->rx_body[6] << 8) | obj->rx_body[7]);

  obj->mech_angle_rad = theta_raw * BLDC_MOTOR_RX_SCALE_THETA;
  obj->angular_speed_rad_s = speed_raw * BLDC_MOTOR_RX_SCALE_SPEED;
  obj->iq_a = iq_raw * BLDC_MOTOR_RX_SCALE_IQ;

  obj->applied_torque_limit_raw = obj->rx_body[8];
}

static void Receive(BldcMotor* obj) {
  while (Serial_Available(obj->serial)) {
    uint8_t b = Serial_Read(obj->serial);

    if (obj->rx_index == 0) {
      // データ部にも 0xAA は現れうるので、ここで同期できたとは限らない。
      // 誤って同期した場合はCRCが一致しないため、結果として弾かれる
      obj->rx_index = (b == BLDC_MOTOR_HEADER) ? 1 : 0;
      continue;
    }

    obj->rx_body[obj->rx_index - 1] = b;
    obj->rx_index++;
    if (obj->rx_index <= BLDC_MOTOR_RX_BODY_SIZE + 1) {
      continue;  // 本体+CRCがまだ揃っていない
    }
    obj->rx_index = 0;

    if (Crc8(obj->rx_body, BLDC_MOTOR_RX_BODY_SIZE) != obj->rx_body[BLDC_MOTOR_RX_BODY_SIZE]) {
      // 化けたフレームは捨てて前回値を保持する。次のフレームで復帰するため、
      // 誤った角度・速度で制御するより保持する方が安全
      obj->rx_error_count++;
      continue;
    }

    ParseStatusFrame(obj);
    obj->rx_count++;
  }
}

void BldcMotor_Init(BldcMotor* obj, Serial* serial) {
  obj->serial = serial;

  obj->tx_enabled = false;
  obj->tx_header = 0;
  obj->tx_raw = 0;
  obj->torque_limit_raw = 0;
  Timer_Init(&obj->tx_timer);

  obj->rx_index = 0;
  obj->rx_count = 0;
  obj->rx_error_count = 0;

  obj->is_running = false;
  obj->is_overcurrent = false;
  obj->is_overheat = false;
  obj->is_voltage_out_of_range = false;
  obj->temperature_c = 0;
  obj->mech_angle_rad = 0.0f;
  obj->angular_speed_rad_s = 0.0f;
  obj->iq_a = 0.0f;
  obj->applied_torque_limit_raw = 0;
}

void BldcMotor_Update(BldcMotor* obj) {
  // 受信は間引かない。Serialのリングバッファは64バイトしかなく、溢れると状態フレームを失う
  Receive(obj);
  Transmit(obj);
}

void BldcMotor_SetAngularSpeed(BldcMotor* obj, float rad_s) {
  SetCommand(obj, BLDC_MOTOR_MODE_SPEED, rad_s, BLDC_MOTOR_SCALE_SPEED);
}

void BldcMotor_SetPosition(BldcMotor* obj, float rad) {
  SetCommand(obj, BLDC_MOTOR_MODE_POSITION, rad, BLDC_MOTOR_SCALE_POSITION);
}

void BldcMotor_SetTorqueCurrent(BldcMotor* obj, float amp) {
  SetCommand(obj, BLDC_MOTOR_MODE_TORQUE, amp, BLDC_MOTOR_SCALE_TORQUE);
}

void BldcMotor_SetTorqueNm(BldcMotor* obj, float nm) {
  SetCommand(obj, BLDC_MOTOR_MODE_TORQUE_NM, nm, BLDC_MOTOR_SCALE_TORQUE_NM);
}

void BldcMotor_SetBrakeCurrent(BldcMotor* obj, float amp) {
  SetCommand(obj, BLDC_MOTOR_MODE_BRAKE, amp, BLDC_MOTOR_SCALE_BRAKE);
}

void BldcMotor_SetBrakeNm(BldcMotor* obj, float nm) {
  SetCommand(obj, BLDC_MOTOR_MODE_BRAKE_NM, nm, BLDC_MOTOR_SCALE_BRAKE_NM);
}

void BldcMotor_SetTorqueLimitNm(BldcMotor* obj, float nm) {
  obj->torque_limit_raw = EncodeLimit(nm, BLDC_MOTOR_SCALE_TORQUE_LIMIT);
}

void BldcMotor_Stop(BldcMotor* obj) {
  obj->tx_enabled = false;
}

bool BldcMotor_IsRunning(const BldcMotor* obj) {
  return obj->is_running;
}

bool BldcMotor_IsOvercurrent(const BldcMotor* obj) {
  return obj->is_overcurrent;
}

bool BldcMotor_IsOverheat(const BldcMotor* obj) {
  return obj->is_overheat;
}

bool BldcMotor_IsVoltageOutOfRange(const BldcMotor* obj) {
  return obj->is_voltage_out_of_range;
}

uint8_t BldcMotor_GetTemperatureC(const BldcMotor* obj) {
  return obj->temperature_c;
}

float BldcMotor_GetMechAngle(const BldcMotor* obj) {
  return obj->mech_angle_rad;
}

float BldcMotor_GetAngularSpeed(const BldcMotor* obj) {
  return obj->angular_speed_rad_s;
}

float BldcMotor_GetIq(const BldcMotor* obj) {
  return obj->iq_a;
}

float BldcMotor_GetAppliedTorqueLimitNm(const BldcMotor* obj) {
  return obj->applied_torque_limit_raw * BLDC_MOTOR_SCALE_TORQUE_LIMIT;
}

bool BldcMotor_IsLimitSynced(const BldcMotor* obj) {
  if (obj->rx_count == 0) {
    return false;
  }
  return obj->applied_torque_limit_raw == obj->torque_limit_raw;
}

uint32_t BldcMotor_GetRxCount(const BldcMotor* obj) {
  return obj->rx_count;
}

uint32_t BldcMotor_GetRxErrorCount(const BldcMotor* obj) {
  return obj->rx_error_count;
}
