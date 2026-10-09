#include "motors.h"

#include <stddef.h>

static const BldcMotor* MotorAt(const Motors* obj, int index) {
  switch (index) {
    case MOTORS_INDEX_REAR_LEFT:
      return &obj->rear_left;
    case MOTORS_INDEX_REAR_RIGHT:
      return &obj->rear_right;
    default:
      return &obj->steering;
  }
}

static void ResetCommWatch(MotorsCommWatch* watch, uint32_t rx_count) {
  watch->last_rx_count = rx_count;
  watch->idle_polls = 0;
  watch->ok = false;
  Timer_Reset(&watch->timer);
}

static void UpdateCommWatch(MotorsCommWatch* watch, uint32_t rx_count) {
  if (rx_count != watch->last_rx_count) {
    watch->last_rx_count = rx_count;
    watch->idle_polls = 0;
    watch->ok = true;
    Timer_Reset(&watch->timer);
    return;
  }
  if (watch->idle_polls < UINT16_MAX) watch->idle_polls++;
  if (Timer_ReadMs(&watch->timer) > MOTORS_COMM_TIMEOUT_MS &&
      watch->idle_polls > MOTORS_COMM_MIN_IDLE_POLLS) {
    watch->ok = false;
  }
}

void Motors_Init(Motors* obj, Serial* steering_serial, Serial* rear_left_serial, Serial* rear_right_serial) {
  BldcMotor_Init(&obj->steering, steering_serial);
  BldcMotor_Init(&obj->rear_left, rear_left_serial);
  BldcMotor_Init(&obj->rear_right, rear_right_serial);
  for (int i = 0; i < MOTORS_COUNT; i++) {
    Timer_Init(&obj->comm_watch[i].timer);
    ResetCommWatch(&obj->comm_watch[i], 0);
  }
}

void Motors_Receive(Motors* obj, bool powered) {
  if (!powered) {
    BldcMotor_Reset(&obj->steering);
    BldcMotor_Reset(&obj->rear_left);
    BldcMotor_Reset(&obj->rear_right);
    for (int i = 0; i < MOTORS_COUNT; i++) {
      ResetCommWatch(&obj->comm_watch[i], BldcMotor_GetRxCount(MotorAt(obj, i)));
    }
    return;
  }
  BldcMotor_Receive(&obj->steering);
  BldcMotor_Receive(&obj->rear_left);
  BldcMotor_Receive(&obj->rear_right);
  for (int i = 0; i < MOTORS_COUNT; i++) {
    UpdateCommWatch(&obj->comm_watch[i], BldcMotor_GetRxCount(MotorAt(obj, i)));
  }
}

void Motors_Transmit(Motors* obj, bool powered) {
  if (!powered) return;
  BldcMotor_Transmit(&obj->steering);
  BldcMotor_Transmit(&obj->rear_left);
  BldcMotor_Transmit(&obj->rear_right);
}

BldcMotor* Motors_Get(Motors* obj, int index) {
  return (BldcMotor*)MotorAt(obj, index);
}

bool Motors_IsCommOk(const Motors* obj, int index) {
  return obj->comm_watch[index].ok;
}

const char* Motors_NotReadyReason(const Motors* obj, int index) {
  const BldcMotor* motor = MotorAt(obj, index);
  if (!obj->comm_watch[index].ok) return "no response";
  if (BldcMotor_IsOvercurrent(motor)) return "overcurrent";
  if (BldcMotor_IsOverheat(motor)) return "overheat";
  if (BldcMotor_IsVoltageOutOfRange(motor)) return "supply voltage out of range";
  if (BldcMotor_IsUncalibrated(motor)) return "encoder not calibrated";
  // MD が無通信タイムアウト・異常で停止モードに入ると上限を 0 に戻す。MD 側の絶対上限
  // (Kt × 最大電流。校正値で決まる) がこちらの指令より小さい個体でもここに来る
  if (!BldcMotor_IsLimitSynced(motor)) return "torque limit not applied";
  return NULL;
}

bool Motors_IsReady(const Motors* obj) {
  for (int i = 0; i < MOTORS_COUNT; i++) {
    if (Motors_NotReadyReason(obj, i) != NULL) return false;
  }
  return true;
}
