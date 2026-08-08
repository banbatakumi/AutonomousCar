#include "encoder.h"

#include "mymath.h"

#define ENCODER_VREF 3.3f  // エンコーダ電源電圧 (+3V3系、フルスケール=1回転分の電圧に相当)

static float VoltageToAngleRad(float voltage) {
  return NormalizeRadians(voltage / ENCODER_VREF * TWO_PI);
}

void Encoder_Init(Encoder* obj, AdcDma* adc2) {
  obj->adc2 = adc2;
  obj->angle_left_rad = VoltageToAngleRad(AdcDma_GetVoltage(adc2, ENCODER_ADC2_CH_LEFT, ENCODER_VREF));
  obj->angle_right_rad = VoltageToAngleRad(AdcDma_GetVoltage(adc2, ENCODER_ADC2_CH_RIGHT, ENCODER_VREF));
  obj->angular_velocity_left_rad_s = 0.0f;
  obj->angular_velocity_right_rad_s = 0.0f;
  obj->accum_angle_left_mrad = 0;
  obj->accum_angle_right_mrad = 0;
  obj->accum_carry_left_mrad = 0.0f;
  obj->accum_carry_right_mrad = 0.0f;
  Timer_Init(&obj->timer);
}

// 角度差を累積値へ足し込む。1e-3 rad に満たない端数を毎回切り捨てると、微小な回転が
// いくら続いても累積が進まない (低速走行で距離が出ない) ため、端数は繰り越す。
static void Accumulate(int32_t* accum_mrad, float* carry_mrad, float gap_rad) {
  *carry_mrad += gap_rad * 1000.0f;
  int32_t whole = (int32_t)(*carry_mrad);
  *accum_mrad += whole;
  *carry_mrad -= (float)whole;
}

void Encoder_Update(Encoder* obj) {
  float dt_s = (float)Timer_ReadUs(&obj->timer) / 1000000.0f;
  if (dt_s <= 0.0f) return;

  float new_angle_left_rad = VoltageToAngleRad(AdcDma_GetVoltage(obj->adc2, ENCODER_ADC2_CH_LEFT, ENCODER_VREF));
  float new_angle_right_rad = VoltageToAngleRad(AdcDma_GetVoltage(obj->adc2, ENCODER_ADC2_CH_RIGHT, ENCODER_VREF));

  float gap_left_rad = GapRadians(new_angle_left_rad, obj->angle_left_rad);
  float gap_right_rad = GapRadians(new_angle_right_rad, obj->angle_right_rad);

  obj->angular_velocity_left_rad_s = gap_left_rad / dt_s;
  obj->angular_velocity_right_rad_s = gap_right_rad / dt_s;

  Accumulate(&obj->accum_angle_left_mrad, &obj->accum_carry_left_mrad, gap_left_rad);
  Accumulate(&obj->accum_angle_right_mrad, &obj->accum_carry_right_mrad, gap_right_rad);

  obj->angle_left_rad = new_angle_left_rad;
  obj->angle_right_rad = new_angle_right_rad;
  Timer_Reset(&obj->timer);
}

float Encoder_GetAngleLeft(Encoder* obj) {
  return obj->angle_left_rad;
}

float Encoder_GetAngleRight(Encoder* obj) {
  return obj->angle_right_rad;
}

float Encoder_GetAngularVelocityLeft(Encoder* obj) {
  return obj->angular_velocity_left_rad_s;
}

float Encoder_GetAngularVelocityRight(Encoder* obj) {
  return obj->angular_velocity_right_rad_s;
}

int32_t Encoder_GetAccumAngleLeftMrad(const Encoder* obj) {
  return obj->accum_angle_left_mrad;
}

int32_t Encoder_GetAccumAngleRightMrad(const Encoder* obj) {
  return obj->accum_angle_right_mrad;
}
