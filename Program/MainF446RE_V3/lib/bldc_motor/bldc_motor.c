#include "bldc_motor.h"

#define BLDC_MOTOR_HEADER 0xAA
#define BLDC_MOTOR_FOOTER 0xFF
#define BLDC_MOTOR_RX_BODY_SIZE 8  // status(1) + temp(1) + theta(2) + speed(2) + iq(2)

// MD側のSERIAL_COMMANDSテーブルのscaleと一致させること
#define BLDC_MOTOR_SCALE_SPEED 0.01f
#define BLDC_MOTOR_SCALE_POSITION 0.001f
#define BLDC_MOTOR_SCALE_TORQUE 0.001f
#define BLDC_MOTOR_SCALE_TORQUE_NM 0.0001f
#define BLDC_MOTOR_SCALE_BRAKE 0.001f
#define BLDC_MOTOR_SCALE_BRAKE_NM 0.0001f

// MD側のSendSerialが送ってくる状態フレームのscale (theta: 0.1mrad, speed: 0.01rad/s, iq: 1mA)
#define BLDC_MOTOR_RX_SCALE_THETA 0.0001f
#define BLDC_MOTOR_RX_SCALE_SPEED 0.01f
#define BLDC_MOTOR_RX_SCALE_IQ 0.001f

static void SetCommand(BldcMotor* obj, uint8_t header, float value, float scale) {
  obj->tx_enabled = true;
  obj->tx_header = header;
  obj->tx_raw = (int16_t)(value / scale);
}

void BldcMotor_Init(BldcMotor* obj, Serial* serial) {
  obj->serial = serial;

  obj->tx_enabled = false;
  obj->tx_header = 0;
  obj->tx_raw = 0;

  obj->rx_index = 0;
  obj->rx_count = 0;

  obj->is_running = false;
  obj->is_overcurrent = false;
  obj->is_overheat = false;
  obj->is_voltage_out_of_range = false;
  obj->temperature_c = 0;
  obj->mech_angle_rad = 0.0f;
  obj->angular_speed_rad_s = 0.0f;
  obj->iq_a = 0.0f;
}

void BldcMotor_Update(BldcMotor* obj) {
  if (obj->tx_enabled) {
    obj->tx_frame[0] = BLDC_MOTOR_HEADER;
    obj->tx_frame[1] = obj->tx_header;
    obj->tx_frame[2] = (obj->tx_raw >> 8) & 0xFF;
    obj->tx_frame[3] = obj->tx_raw & 0xFF;
    obj->tx_frame[4] = BLDC_MOTOR_FOOTER;
    Serial_Write(obj->serial, obj->tx_frame, sizeof(obj->tx_frame));
  }

  while (Serial_Available(obj->serial)) {
    uint8_t b = Serial_Read(obj->serial);

    if (obj->rx_index == 0) {
      obj->rx_index = (b == BLDC_MOTOR_HEADER) ? 1 : 0;
    } else if (obj->rx_index <= BLDC_MOTOR_RX_BODY_SIZE) {
      obj->rx_body[obj->rx_index - 1] = b;
      obj->rx_index++;
    } else {
      if (b == BLDC_MOTOR_FOOTER) {
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
        obj->rx_count++;
      }
      obj->rx_index = 0;
    }
  }
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

void BldcMotor_Stop(BldcMotor* obj) {
  obj->tx_enabled = false;
}

bool BldcMotor_IsRunning(BldcMotor* obj) {
  return obj->is_running;
}

bool BldcMotor_IsOvercurrent(BldcMotor* obj) {
  return obj->is_overcurrent;
}

bool BldcMotor_IsOverheat(BldcMotor* obj) {
  return obj->is_overheat;
}

bool BldcMotor_IsVoltageOutOfRange(BldcMotor* obj) {
  return obj->is_voltage_out_of_range;
}

uint8_t BldcMotor_GetTemperatureC(BldcMotor* obj) {
  return obj->temperature_c;
}

float BldcMotor_GetMechAngle(BldcMotor* obj) {
  return obj->mech_angle_rad;
}

float BldcMotor_GetAngularSpeed(BldcMotor* obj) {
  return obj->angular_speed_rad_s;
}

float BldcMotor_GetIq(BldcMotor* obj) {
  return obj->iq_a;
}

uint32_t BldcMotor_GetRxCount(BldcMotor* obj) {
  return obj->rx_count;
}
