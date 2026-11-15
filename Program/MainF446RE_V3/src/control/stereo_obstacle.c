#include "stereo_obstacle.h"

#include <string.h>

void StereoObstacles_Init(StereoObstacles* obj) { memset(obj, 0, sizeof(*obj)); }

void StereoObstacles_Push(StereoObstacles* obj, uint32_t now_us, uint16_t age_ms, uint8_t count,
                          const int16_t* xy_mm) {
  for (uint8_t i = STEREO_OBS_MAX_CONFIRM - 1; i > 0; i--) obj->frames[i] = obj->frames[i - 1];
  StereoFrame* f = &obj->frames[0];
  if (count > STEREO_OBS_MAX_POINTS) count = STEREO_OBS_MAX_POINTS;
  f->valid = true;
  f->count = count;
  f->rx_us = now_us;
  f->age_rx_us = (uint32_t)age_ms * 1000u;
  for (uint8_t i = 0; i < count; i++) {
    f->x_m[i] = (float)xy_mm[2 * i] * 0.001f;
    f->y_m[i] = (float)xy_mm[2 * i + 1] * 0.001f;
  }
}

bool StereoObstacles_IsFresh(const StereoObstacles* obj, uint32_t now_us, float max_age_s) {
  if (!obj->frames[0].valid) return false;
  return (float)(uint32_t)(now_us - obj->frames[0].rx_us) * 1e-6f <= max_age_s;
}

// 撮影からの経過 [s]
static float FrameAgeS(const StereoFrame* f, uint32_t now_us) {
  return ((float)(uint32_t)(now_us - f->rx_us) + (float)f->age_rx_us) * 1e-6f;
}

static uint8_t CountInPath(const StereoFrame* f, uint32_t now_us, const StereoEvalParams* p,
                           uint8_t confirm_frames) {
  // 撮影からの間に車が進んだ分だけ予想進路を延ばす。確認のフレーム分と通信の遅れも
  float extra_s = FrameAgeS(f, now_us) + p->comm_delay_s +
                  p->confirm_period_s * (float)(confirm_frames - 1);
  AutoStopPath path;
  AutoStopPath_Init(&path, true, p->curvature_1_m, p->body_front_x_m, p->body_rear_x_m,
                    p->path_half_width_m, p->d_stop_m + p->speed_abs_m_s * extra_s);
  // 自車除外は車体の外形を前・横へ広げた範囲
  float ex_front = p->body_front_x_m + p->self_ext_front_m;
  float ex_half = p->body_half_width_m + p->self_ext_side_m;
  uint8_t hits = 0;
  for (uint8_t i = 0; i < f->count; i++) {
    if (AutoStopPath_IsInsideBody(f->x_m[i], f->y_m[i], ex_front, p->body_rear_x_m, ex_half)) {
      continue;
    }
    if (AutoStopPath_Contains(&path, f->x_m[i], f->y_m[i])) hits++;
  }
  return hits;
}

bool StereoObstacles_Evaluate(const StereoObstacles* obj, uint32_t now_us, bool forward,
                              const StereoEvalParams* p) {
  uint8_t n = p->confirm_frames;
  if (n < 1) n = 1;
  if (n > STEREO_OBS_MAX_CONFIRM) n = STEREO_OBS_MAX_CONFIRM;
  if (!forward || !StereoObstacles_IsFresh(obj, now_us, p->max_age_s)) return false;
  for (uint8_t i = 0; i < n; i++) {
    if (!obj->frames[i].valid) return false;
    // 隣のフレームが離れすぎていたら「連続」と見ない (途絶の後の 1 枚目で止めない)
    if (i > 0) {
      uint32_t gap_us = (uint32_t)(obj->frames[i - 1].rx_us - obj->frames[i].rx_us);
      if ((float)gap_us * 1e-6f > p->max_age_s) return false;
    }
    if (CountInPath(&obj->frames[i], now_us, p, n) < p->min_points) return false;
  }
  return true;
}
