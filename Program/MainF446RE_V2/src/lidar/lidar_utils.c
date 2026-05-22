#include "lidar_utils.h"

// distances_360[idx] == 0 は「未計測」を意味する。
// LD06 は有効測定で距離 0mm を返さないため、0 を欠損値として扱う。

// 1 点分の距離を統計に加算するヘルパー。dist == 0 のとき何もしない。
static void AccumulateDist(uint16_t dist, uint32_t* sum, LidarSector* s) {
  if (dist == 0) return;
  *sum += dist;
  s->count++;
  if (s->count == 1 || dist < s->min) s->min = dist;
  if (dist > s->max) s->max = dist;
}

LidarSector Lidar_GetSector(const LD06* lidar, int center_deg,
                            int half_width_deg) {
  LidarSector s = {0.0f, 0, 0, 0};
  uint32_t sum = 0;

  for (int d = center_deg - half_width_deg; d <= center_deg + half_width_deg;
       d++) {
    // center_deg < half_width_deg のとき d が負になるため +360 でラップ
    AccumulateDist(lidar->distances_360[(d + 360) % 360], &sum, &s);
  }

  if (s.count > 0) s.avg = (float)sum / (float)s.count;
  return s;
}

int Lidar_FindNearestSector(const LD06* lidar, int center_deg,
                            int half_width_deg, int sector_half_deg,
                            float min_valid_mm, int min_count,
                            float* out_avg_mm) {
  int best_deg = -1;
  float best_avg = 0.0f;

  for (int d = -half_width_deg; d <= half_width_deg; d++) {
    int center = (center_deg + d + 360) % 360;
    LidarSector s = Lidar_GetSector(lidar, center, sector_half_deg);
    if (s.count < min_count) continue;
    if (s.avg < min_valid_mm) continue;
    if (best_deg == -1 || s.avg < best_avg) {
      best_avg = s.avg;
      best_deg = center;
    }
  }

  if (best_deg != -1) *out_avg_mm = best_avg;
  return best_deg;
}

int Lidar_FindClearestDirection(const LD06* lidar, int center_deg,
                                int half_width_deg, int sector_half_width,
                                float* out_avg_mm) {
  int best_angle = -1;
  float best_avg = -1.0f;

  for (int d = -half_width_deg; d <= half_width_deg; d++) {
    int center = (center_deg + d + 360) % 360;
    LidarSector s = Lidar_GetSector(lidar, center, sector_half_width);
    if (s.count == 0) continue;
    if (s.avg > best_avg) {
      best_avg = s.avg;
      best_angle = center;
    }
  }

  if (out_avg_mm) *out_avg_mm = best_avg;
  return best_angle;
}

void Lidar_FilterSpikes(uint16_t points_360[360], int window_deg,
                        float threshold_ratio) {
  // 比較は変更前のデータに対して行うためコピーを取る
  uint16_t orig[360];
  for (int i = 0; i < 360; i++) orig[i] = points_360[i];

  for (int i = 0; i < 360; i++) {
    if (orig[i] == 0) continue;

    // ±window_deg の有効点（自点除く）の平均を計算
    uint32_t sum = 0;
    int count = 0;
    for (int d = 1; d <= window_deg; d++) {
      uint16_t left = orig[(i - d + 360) % 360];
      uint16_t right = orig[(i + d) % 360];
      if (left > 0) {
        sum += left;
        count++;
      }
      if (right > 0) {
        sum += right;
        count++;
      }
    }

    if (count == 0) continue;

    float avg = (float)sum / (float)count;
    float diff = (float)orig[i] - avg;
    if (diff < 0.0f) diff = -diff;

    if (diff > avg * threshold_ratio) {
      points_360[i] = (uint16_t)avg;
    }
  }
}

void Lidar_BuildFilledPoints(const LD06* lidar, uint16_t out_360[360],
                             uint8_t confidence_threshold) {
  uint8_t valid[360];
  for (int i = 0; i < 360; i++) {
    valid[i] = lidar->distances_360[i] > 0 &&
               lidar->confidences_360[i] >= confidence_threshold;
    out_360[i] = valid[i] ? lidar->distances_360[i] : 0;
  }

  for (int i = 0; i < 360; i++) {
    if (valid[i]) continue;

    // 両隣の有効点を最大 180° の範囲で探す
    int left_idx = -1, right_idx = -1;
    for (int d = 1; d <= 180; d++) {
      if (left_idx == -1 && valid[(i - d + 360) % 360]) left_idx = (i - d + 360) % 360;
      if (right_idx == -1 && valid[(i + d) % 360]) right_idx = (i + d) % 360;
      if (left_idx != -1 && right_idx != -1) break;
    }

    if (left_idx == -1 && right_idx == -1) continue;

    if (left_idx == -1) {
      out_360[i] = lidar->distances_360[right_idx];
    } else if (right_idx == -1) {
      out_360[i] = lidar->distances_360[left_idx];
    } else {
      // 角度距離で重み付けした線形補間
      // 左が l ステップ、右が r ステップ離れているとき:
      //   out = left_dist * (r/(l+r)) + right_dist * (l/(l+r))
      int l = (i - left_idx + 360) % 360;
      int r = (right_idx - i + 360) % 360;
      out_360[i] = (uint16_t)(((uint32_t)lidar->distances_360[left_idx] * r +
                               (uint32_t)lidar->distances_360[right_idx] * l) /
                              (l + r));
    }
  }
}
