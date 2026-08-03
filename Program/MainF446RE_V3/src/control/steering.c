#include "steering.h"

#include <stdio.h>

#include "flash.h"
#include "mymath.h"
#include "timer.h"

#define STEERING_CALIB_MAGIC 0x53544545u  // "STEE"
#define STEERING_CALIB_TIMEOUT_S 2.0f     // MD起動直後は状態フレーム送信開始まで時間がかかりうるため、待つ上限

typedef struct {
  uint32_t magic;
  float center_rad;
} SteeringCalibData;

static void LoadFromFlash(Steering* obj) {
  SteeringCalibData calib;
  Flash_ReadData(FLASH_USER_START_ADDR, &calib, sizeof(calib));
  if (calib.magic == STEERING_CALIB_MAGIC) {
    obj->center_rad = calib.center_rad;
    printf("Steering: loaded center_rad=%.3f\n", obj->center_rad);
  } else {
    obj->center_rad = 0.0f;
    printf("Steering: no calibration data found, using center_rad=0\n");
  }
}

// MDから状態フレームを実際に受信できるまで待ってから中心点を記録する。MD起動直後は送信開始まで
// 時間がかかる場合があるため、固定時間ではなく最初の1フレームが届くまで待つ。タイムアウトまでに
// 一度も受信できなければ配線・ボーレート不一致等で受信できていないと判断し、trueを返さない。
static bool Calibrate(Steering* obj) {
  uint32_t rx_count_before = BldcMotor_GetRxCount(obj->motor);

  Timer timer;
  Timer_Init(&timer);
  while (BldcMotor_GetRxCount(obj->motor) == rx_count_before) {
    BldcMotor_Update(obj->motor);
    if (Timer_Read(&timer) > STEERING_CALIB_TIMEOUT_S) {
      printf("Steering: calibration failed (no data received from motor)\n");
      return false;
    }
  }

  obj->center_rad = BldcMotor_GetMechAngle(obj->motor);

  SteeringCalibData calib = {STEERING_CALIB_MAGIC, obj->center_rad};
  Flash_WriteData(FLASH_USER_START_ADDR, &calib, sizeof(calib));
  printf("Steering calibrated: center_rad=%.3f\n", obj->center_rad);
  return true;
}

void Steering_Init(Steering* obj, BldcMotor* motor, bool do_calibrate) {
  obj->motor = motor;

  if (do_calibrate && Calibrate(obj)) {
    return;
  }

  LoadFromFlash(obj);
}

void Steering_SetAngleRad(Steering* obj, float angle_rad) {
  angle_rad = Constrain(angle_rad, -STEERING_MAX_ANGLE_RAD, STEERING_MAX_ANGLE_RAD);
  float target_rad = NormalizeRadians(obj->center_rad + angle_rad);
  BldcMotor_SetPosition(obj->motor, target_rad);
}

float Steering_GetAngleRad(Steering* obj) {
  return GapRadians(BldcMotor_GetMechAngle(obj->motor), obj->center_rad);
}
