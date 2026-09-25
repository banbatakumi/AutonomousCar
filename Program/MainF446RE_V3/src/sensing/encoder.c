#include "encoder.h"

#include "mymath.h"

#define ENCODER_VREF 3.3f  // エンコーダ電源電圧 (+3V3系、フルスケール=1回転分の電圧に相当)

#define ENCODER_ADC_FULL_SCALE 4095.0f
#define ENCODER_ADC_HALF_TURN_COUNTS 2048

static float VoltageToAngleRad(float voltage) {
  return NormalizeRadians(voltage / ENCODER_VREF * TWO_PI);
}

// 循環バッファ内の全サンプルを平均した角度 [rad]。生のカウントをそのまま平均すると、
// 窓の途中で 0/2π の境界 (4095→0) を跨いだときに半周ずれた値になるため、
// 先頭サンプルからの差分を ±半周に畳んでから平均する。1窓 (約444us) で半周も回ることはない
static float ReadAveragedAngleRad(AdcDma* adc, uint32_t channel) {
  int32_t reference = AdcDma_GetRawAt(adc, 0, channel);
  int32_t sum_diff = 0;
  for (uint32_t i = 0; i < adc->num_samples; i++) {
    int32_t diff = (int32_t)AdcDma_GetRawAt(adc, i, channel) - reference;
    if (diff > ENCODER_ADC_HALF_TURN_COUNTS) diff -= 2 * ENCODER_ADC_HALF_TURN_COUNTS;
    if (diff < -ENCODER_ADC_HALF_TURN_COUNTS) diff += 2 * ENCODER_ADC_HALF_TURN_COUNTS;
    sum_diff += diff;
  }
  float counts = (float)reference + (float)sum_diff / (float)adc->num_samples;
  return VoltageToAngleRad(counts * ENCODER_VREF / ENCODER_ADC_FULL_SCALE);
}

void Encoder_Init(Encoder* obj, AdcDma* adc2) {
  obj->adc2 = adc2;
  // DMA 開始直後はバッファの大半が未書き込み (0) で、平均が 0 側へ引っ張られる。そのまま初期角度にすると
  // 初回 Update で偽の角度差が累積回転角 (オドメトリ) に入るため、一巡するまで待つ
  HAL_Delay(2);
  obj->angle_left_rad = ReadAveragedAngleRad(adc2, ENCODER_ADC2_CH_LEFT);
  obj->angle_right_rad = ReadAveragedAngleRad(adc2, ENCODER_ADC2_CH_RIGHT);
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

  float new_angle_left_rad = ReadAveragedAngleRad(obj->adc2, ENCODER_ADC2_CH_LEFT);
  float new_angle_right_rad = ReadAveragedAngleRad(obj->adc2, ENCODER_ADC2_CH_RIGHT);

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
