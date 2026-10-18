#include "encoder.h"

#include <stdbool.h>

#include "mymath.h"

#define ENCODER_ADC_FULL_SCALE 4095.0f

// 循環バッファ内の全サンプルを平均した、ADC のフルスケールに対する比 (1回転ぶんで 0〜1 を一周する)。
// 生のカウントをそのまま平均すると、窓の途中で 0/2π の境界を跨いだときに半周ずれた値になるため、
// 先頭サンプルからの差分を ±半周に畳んでから平均する。1回転に当たるカウントは校正値の範囲 (hi − lo) で、
// 既定はフルスケール。1窓 (約444us) で半周も回ることはない
static float ReadAveragedRaw(AdcDma* adc, uint32_t channel, const EncoderCal* cal) {
  const float turn = EncoderCal_Span(cal) * ENCODER_ADC_FULL_SCALE;
  int32_t reference = AdcDma_GetRawAt(adc, 0, channel);
  float sum_diff = 0.0f;
  for (uint32_t i = 0; i < adc->num_samples; i++) {
    float diff = (float)((int32_t)AdcDma_GetRawAt(adc, i, channel) - reference);
    if (diff > 0.5f * turn) diff -= turn;
    if (diff < -0.5f * turn) diff += turn;
    sum_diff += diff;
  }
  return ((float)reference + sum_diff / (float)adc->num_samples) / ENCODER_ADC_FULL_SCALE;
}

void Encoder_Init(Encoder* obj, AdcDma* adc2) {
  obj->adc2 = adc2;
  EncoderCal_SetIdentity(&obj->cal_left);
  EncoderCal_SetIdentity(&obj->cal_right);
  // DMA 開始直後はバッファの大半が未書き込み (0) で、平均が 0 側へ引っ張られる。そのまま初期角度にすると
  // 初回 Update で偽の角度差が累積回転角 (オドメトリ) に入るため、一巡するまで待つ
  HAL_Delay(2);
  obj->raw_left = ReadAveragedRaw(adc2, ENCODER_ADC2_CH_LEFT, &obj->cal_left);
  obj->raw_right = ReadAveragedRaw(adc2, ENCODER_ADC2_CH_RIGHT, &obj->cal_right);
  obj->angle_left_rad = EncoderCal_Angle(&obj->cal_left, obj->raw_left);
  obj->angle_right_rad = EncoderCal_Angle(&obj->cal_right, obj->raw_right);
  obj->angular_velocity_left_rad_s = 0.0f;
  obj->angular_velocity_right_rad_s = 0.0f;
  obj->accum_angle_left_mrad = 0;
  obj->accum_angle_right_mrad = 0;
  obj->accum_carry_left_mrad = 0.0f;
  obj->accum_carry_right_mrad = 0.0f;
  obj->last_gap_left_rad = 0.0f;
  obj->last_gap_right_rad = 0.0f;
  obj->last_dt_s = 0.0f;
  obj->update_seq = 0;
  Timer enable_cycle_counter;
  Timer_Init(&enable_cycle_counter);  // DWT のサイクルカウンタを有効にする
  obj->last_cycles = DWT->CYCCNT;
}

// 角度差を累積値へ足し込む。1e-3 rad に満たない端数を毎回切り捨てると、微小な回転が
// いくら続いても累積が進まない (低速走行で距離が出ない) ため、端数は繰り越す。
static void Accumulate(int32_t* accum_mrad, float* carry_mrad, float gap_rad) {
  *carry_mrad += gap_rad * 1000.0f;
  int32_t whole = (int32_t)(*carry_mrad);
  *accum_mrad += whole;
  *carry_mrad -= (float)whole;
}

// 前回の読み取りからこれ以上空いたら、角度差から回転量を決められない [s]。角度差は ±半周に
// 畳んで求めるので、1回の間に半周を超えて回ると折り返して逆向き・過小に読む。
// 15ms なら半周 (π) を超えるのは 209rad/s (前輪の周速で約6m/s) 以上のとき
#define ENCODER_MAX_DT_S 0.015f

void Encoder_Update(Encoder* obj) {
  // 角度を読む瞬間の時刻。dt は前回の「角度を読んだ瞬間」との差で、関数の処理時間も含めて正しい間隔になる。
  // 以前は Timer を関数の最後でリセットしていたため、dt が処理時間 (約9us) ぶん短く、車速が約1.8% (500/491.5)
  // 速く出ていた (記録の odom_dist・後輪周速との比、SLAM との比で確認)。us への切り捨ても無くした
  uint32_t now_cycles = DWT->CYCCNT;
  float dt_s = (float)(now_cycles - obj->last_cycles) / (float)(Timer_GetCyclesPerUs() * 1000000U);
  if (dt_s <= 0.0f) return;

  obj->raw_left = ReadAveragedRaw(obj->adc2, ENCODER_ADC2_CH_LEFT, &obj->cal_left);
  obj->raw_right = ReadAveragedRaw(obj->adc2, ENCODER_ADC2_CH_RIGHT, &obj->cal_right);
  float new_angle_left_rad = EncoderCal_Angle(&obj->cal_left, obj->raw_left);
  float new_angle_right_rad = EncoderCal_Angle(&obj->cal_right, obj->raw_right);

  float gap_left_rad = GapRadians(new_angle_left_rad, obj->angle_left_rad);
  float gap_right_rad = GapRadians(new_angle_right_rad, obj->angle_right_rad);

  // 長く空いたとき (メインループが止まっていた) は、角速度を更新せず直前の値を保つ
  // (折り返しを誤った速度を出さない)。累積は、半周以内の回転なら正しく足せるのでそのまま足す
  if (dt_s <= ENCODER_MAX_DT_S) {
    obj->angular_velocity_left_rad_s = gap_left_rad / dt_s;
    obj->angular_velocity_right_rad_s = gap_right_rad / dt_s;
  }

  Accumulate(&obj->accum_angle_left_mrad, &obj->accum_carry_left_mrad, gap_left_rad);
  Accumulate(&obj->accum_angle_right_mrad, &obj->accum_carry_right_mrad, gap_right_rad);

  obj->last_gap_left_rad = gap_left_rad;
  obj->last_gap_right_rad = gap_right_rad;
  obj->last_dt_s = dt_s;
  obj->update_seq++;

  obj->angle_left_rad = new_angle_left_rad;
  obj->angle_right_rad = new_angle_right_rad;
  obj->last_cycles = now_cycles;
}

static bool CalEqual(const EncoderCal* a, const EncoderCal* b) {
  return a->lo == b->lo && a->hi == b->hi && a->h1s == b->h1s && a->h1c == b->h1c && a->h2s == b->h2s &&
         a->h2c == b->h2c;
}

void Encoder_SetCalibration(Encoder* obj, const EncoderCal* left, const EncoderCal* right) {
  if (!CalEqual(&obj->cal_left, left)) {
    obj->cal_left = *left;
    obj->angle_left_rad = EncoderCal_Angle(&obj->cal_left, obj->raw_left);
  }
  if (!CalEqual(&obj->cal_right, right)) {
    obj->cal_right = *right;
    obj->angle_right_rad = EncoderCal_Angle(&obj->cal_right, obj->raw_right);
  }
}

float Encoder_GetRawAngleLeft(const Encoder* obj) {
  return EncoderCal_Wrap(obj->raw_left * ENCODER_CAL_TWO_PI);
}

float Encoder_GetRawAngleRight(const Encoder* obj) {
  return EncoderCal_Wrap(obj->raw_right * ENCODER_CAL_TWO_PI);
}

float Encoder_GetLastGapLeft(const Encoder* obj) {
  return obj->last_gap_left_rad;
}

float Encoder_GetLastGapRight(const Encoder* obj) {
  return obj->last_gap_right_rad;
}

float Encoder_GetLastDt(const Encoder* obj) {
  return obj->last_dt_s;
}

uint32_t Encoder_GetUpdateSeq(const Encoder* obj) {
  return obj->update_seq;
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
