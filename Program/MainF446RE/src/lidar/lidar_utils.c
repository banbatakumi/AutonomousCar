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

int Lidar_FindClearestDirection(const LD06* lidar, int start_deg, int end_deg,
                                int sector_half_width) {
  int best_angle = -1;
  float best_avg = -1.0f;

  // 全周のとき start_deg を 2 回スキャンしないよう 359 にする
  int span = (end_deg - start_deg + 360) % 360;
  if (span == 0) span = 359;

  for (int i = 0; i <= span; i++) {
    int center = (start_deg + i) % 360;
    LidarSector s = Lidar_GetSector(lidar, center, sector_half_width);
    if (s.count == 0) continue;
    if (s.avg > best_avg) {
      best_avg = s.avg;
      best_angle = center;
    }
  }

  return best_angle;
}
