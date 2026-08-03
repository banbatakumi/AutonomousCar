#include "encoder.h"

#include "mymath.h"

#define ENCODER_VREF 3.3f  // エンコーダ電源電圧 (+3V3系、フルスケール=1回転分の電圧に相当)

static float VoltageToAngleRad(float voltage) {
  return NormalizeRadians(voltage / ENCODER_VREF * TWO_PI);
}

void Encoder_Init(Encoder *obj, AdcDma *adc2) {
  obj->adc2 = adc2;
  obj->angle_left_rad = VoltageToAngleRad(AdcDma_GetVoltage(adc2, ENCODER_ADC2_CH_LEFT, ENCODER_VREF));
  obj->angle_right_rad = VoltageToAngleRad(AdcDma_GetVoltage(adc2, ENCODER_ADC2_CH_RIGHT, ENCODER_VREF));
  obj->angular_velocity_left_rad_s = 0.0f;
  obj->angular_velocity_right_rad_s = 0.0f;
  Timer_Init(&obj->timer);
}

void Encoder_Update(Encoder *obj) {
  float dt_s = (float)Timer_ReadUs(&obj->timer) / 1000000.0f;
  if (dt_s <= 0.0f) return;

  float new_angle_left_rad = VoltageToAngleRad(AdcDma_GetVoltage(obj->adc2, ENCODER_ADC2_CH_LEFT, ENCODER_VREF));
  float new_angle_right_rad = VoltageToAngleRad(AdcDma_GetVoltage(obj->adc2, ENCODER_ADC2_CH_RIGHT, ENCODER_VREF));

  obj->angular_velocity_left_rad_s = GapRadians(new_angle_left_rad, obj->angle_left_rad) / dt_s;
  obj->angular_velocity_right_rad_s = GapRadians(new_angle_right_rad, obj->angle_right_rad) / dt_s;

  obj->angle_left_rad = new_angle_left_rad;
  obj->angle_right_rad = new_angle_right_rad;
  Timer_Reset(&obj->timer);
}

float Encoder_GetAngleLeft(Encoder *obj) {
  return obj->angle_left_rad;
}

float Encoder_GetAngleRight(Encoder *obj) {
  return obj->angle_right_rad;
}

float Encoder_GetAngularVelocityLeft(Encoder *obj) {
  return obj->angular_velocity_left_rad_s;
}

float Encoder_GetAngularVelocityRight(Encoder *obj) {
  return obj->angular_velocity_right_rad_s;
}
