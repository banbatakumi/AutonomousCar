#include "encoder_direction.h"

#include <math.h>

static void ResetWindow(EncoderDirection* obj) {
  obj->front_m = 0.0f;
  obj->rear_m = 0.0f;
  obj->window_s = 0.0f;
}

static void ResetSlowWindow(EncoderDirection* obj) {
  obj->slow_front_m = 0.0f;
  obj->slow_rear_m = 0.0f;
  obj->slow_window_s = 0.0f;
}

static void Flip(EncoderDirection* obj) {
  obj->sign = -obj->sign;
  obj->flips++;
  // 反転前の向きで溜めた量を、反転後の判定に持ち越さない
  ResetWindow(obj);
  ResetSlowWindow(obj);
}

void EncoderDirection_Init(EncoderDirection* obj) {
  obj->sign = 1.0f;
  obj->flips = 0;
  obj->jumps = 0;
  ResetWindow(obj);
  ResetSlowWindow(obj);
}

float EncoderDirection_Apply(EncoderDirection* obj, float forward_m, float rear_speed_m_s, float vehicle_speed_m_s,
                             float dt_s) {
  const float corrected = forward_m * obj->sign;
  if (dt_s <= 0.0f) return corrected;
  const float rear_step_m = rear_speed_m_s * dt_s;
  const float rear_step_abs = fabsf(rear_step_m);
  // 基準: 後輪と車速の推定が同じ向きに十分速い
  const bool reference = fabsf(rear_speed_m_s) >= ENCODER_DIRECTION_MIN_SPEED_M_S &&
                         fabsf(vehicle_speed_m_s) >= ENCODER_DIRECTION_MIN_SPEED_M_S &&
                         rear_speed_m_s * vehicle_speed_m_s > 0.0f;

  // 1. 毎回の直し
  float out = corrected;
  bool jump = false;
  if (reference) {
    if (fabsf(corrected) > ENCODER_DIRECTION_JUMP_RATIO * rear_step_abs + ENCODER_DIRECTION_JUMP_FLOOR_M) {
      out = rear_step_m;
      jump = true;
      obj->jumps++;
    } else if (corrected * rear_step_m < 0.0f && fabsf(corrected) >= ENCODER_DIRECTION_MIN_RATIO * rear_step_abs) {
      out = -corrected;
    }
  }

  // 2. 向きの判定 (基準があった回だけ。飛びは量も向きも当てにならないので入れない)
  if (reference && !jump) {
    obj->front_m += corrected;
    obj->rear_m += rear_step_m;
  }
  obj->window_s += dt_s;
  if (obj->window_s >= ENCODER_DIRECTION_WINDOW_S) {
    const float rear_abs = fabsf(obj->rear_m);
    const bool reversed = rear_abs >= ENCODER_DIRECTION_MIN_SPEED_M_S * obj->window_s &&
                          obj->front_m * obj->rear_m < 0.0f &&
                          fabsf(obj->front_m) >= ENCODER_DIRECTION_MIN_RATIO * rear_abs;
    ResetWindow(obj);
    if (reversed) {
      Flip(obj);
      return out;
    }
  }

  // 3. 立て直し (推定に依らない。飛びは入れない)
  if (!jump) {
    obj->slow_front_m += corrected;
    obj->slow_rear_m += rear_step_m;
  }
  obj->slow_window_s += dt_s;
  if (obj->slow_window_s >= ENCODER_DIRECTION_SLOW_WINDOW_S) {
    const float rear_abs = fabsf(obj->slow_rear_m);
    const float front_abs = fabsf(obj->slow_front_m);
    const bool reversed = rear_abs >= ENCODER_DIRECTION_MIN_SPEED_M_S * obj->slow_window_s &&
                          obj->slow_front_m * obj->slow_rear_m < 0.0f &&
                          front_abs >= ENCODER_DIRECTION_SLOW_RATIO_MIN * rear_abs &&
                          front_abs <= ENCODER_DIRECTION_SLOW_RATIO_MAX * rear_abs;
    ResetSlowWindow(obj);
    if (reversed) Flip(obj);
  }
  return out;
}
