#include "auto_stop_path.h"

#include <math.h>

#define AUTO_STOP_PATH_PI 3.14159265f

void AutoStopPath_Init(AutoStopPath* obj, bool forward, float curvature_1_m, float body_front_x_m,
                       float body_rear_x_m, float half_width_m, float length_m) {
  // x を進む向きに揃えたときの、先端と後端までの距離 (どちらも正)
  float lead_m = forward ? body_front_x_m : -body_rear_x_m;
  float trail_m = forward ? -body_rear_x_m : body_front_x_m;
  float k = fabsf(curvature_1_m);

  obj->forward = forward;
  obj->half_width_m = half_width_m;
  obj->limit_x_m = lead_m + length_m;
  obj->straight = k < AUTO_STOP_PATH_STRAIGHT_CURVATURE_1_M;
  obj->turn_sign = (curvature_1_m >= 0.0f) ? 1.0f : -1.0f;
  if (obj->straight) return;

  float r = 1.0f / k;
  float r_in = r - half_width_m;
  float r_far = r + half_width_m;
  float end_m = (lead_m > trail_m) ? lead_m : trail_m;
  float sweep_rad = length_m / sqrtf(lead_m * lead_m + r * r);
  obj->radius_m = r;
  obj->lead_m = lead_m;
  obj->inner_m = r_in;
  obj->outer_m = r_far;
  obj->r_in_sq = (r_in > 0.0f) ? r_in * r_in : 0.0f;
  obj->r_out_sq = end_m * end_m + r_far * r_far;
  obj->r_corner_sq = lead_m * lead_m + r_in * r_in;
  obj->r_lead_out_sq = lead_m * lead_m + r_far * r_far;
  obj->cos_sweep = cosf(sweep_rad);
  obj->sin_sweep = sinf(sweep_rad);
  obj->sweep_all = sweep_rad >= AUTO_STOP_PATH_PI;
}

bool AutoStopPath_Contains(const AutoStopPath* obj, float x_m, float y_m) {
  float x = obj->forward ? x_m : -x_m;
  if (obj->straight) {
    return x <= obj->limit_x_m && fabsf(y_m) <= obj->half_width_m;
  }
  // C から点へのベクトルの、後車軸の向き (C → 後車軸) の成分。cos α = toward / |C→点|
  float toward = obj->radius_m - y_m * obj->turn_sign;
  float r_sq = x * x + toward * toward;
  if (r_sq < obj->r_in_sq || r_sq > obj->r_out_sq) return false;
  if (obj->sweep_all) return true;
  // 半径 r で最も先にある車体の点の角度 α_lead を、r・cos α_lead と r・sin α_lead で持つ
  float lead_cos_r;
  float lead_sin_r;
  if (r_sq > obj->r_lead_out_sq) {
    lead_cos_r = obj->outer_m;
    lead_sin_r = -sqrtf(r_sq - obj->outer_m * obj->outer_m);
  } else if (r_sq >= obj->r_corner_sq) {
    lead_sin_r = obj->lead_m;
    lead_cos_r = sqrtf(r_sq - obj->lead_m * obj->lead_m);
  } else {
    float side_sq = r_sq - obj->inner_m * obj->inner_m;
    lead_cos_r = obj->inner_m;
    lead_sin_r = (side_sq > 0.0f) ? sqrtf(side_sq) : 0.0f;
  }
  // 上限の角度 b = α_lead + Φ を r・cos b、r・sin b で持つ (加法定理)
  float limit_cos_r = lead_cos_r * obj->cos_sweep - lead_sin_r * obj->sin_sweep;
  float limit_sin_r = lead_sin_r * obj->cos_sweep + lead_cos_r * obj->sin_sweep;
  if (limit_sin_r < 0.0f) {
    // α_lead ≥ 0 なら b は半周を超えた: この半径では角度を問わず当たる
    if (lead_sin_r >= 0.0f) return true;
    // α_lead < 0 (外側の側面が先頭) で b < 0: α ≤ b ⇔ sin(b − α) ≥ 0。両辺に r² を掛けた形
    return limit_sin_r * toward - limit_cos_r * x >= 0.0f;
  }
  // b ∈ [0, π]: α < 0 (x < 0。後退で後車軸より前の真横) の点は b より手前なので当たる。
  // α ≥ 0 は cos α ≥ cos b。両辺に r を掛けた形
  return x < 0.0f || toward >= limit_cos_r;
}

bool AutoStopPath_IsInsideBody(float x_m, float y_m, float body_front_x_m, float body_rear_x_m,
                               float body_half_width_m) {
  return x_m > body_rear_x_m && x_m < body_front_x_m && fabsf(y_m) < body_half_width_m;
}

float AutoStopPath_Curvature(float road_wheel_angle_rad, float wheelbase_m) {
  return tanf(road_wheel_angle_rad) / wheelbase_m;
}
