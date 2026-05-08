#include "lidar_utils.h"

LidarSector Lidar_GetSector(const LD06* lidar, int center_deg,
                            int half_width_deg) {
  LidarSector s = {0.0f, 0, 0, 0};
  uint32_t sum = 0;

  for (int d = center_deg - half_width_deg; d <= center_deg + half_width_deg;
       d++) {
    int idx = (d + 360) % 360;
    uint16_t dist = lidar->distances_360[idx];
    if (dist == 0) continue;

    sum += dist;
    s.count++;
    if (s.count == 1 || dist < s.min) s.min = dist;
    if (dist > s.max) s.max = dist;
  }

  if (s.count > 0) s.avg = (float)sum / (float)s.count;
  return s;
}

LidarSector Lidar_GetSectorRange(const LD06* lidar, int start_deg,
                                 int end_deg) {
  LidarSector s = {0.0f, 0, 0, 0};
  uint32_t sum = 0;

  // 0°をまたぐ場合は total span を計算して回す
  int span = (end_deg - start_deg + 360) % 360;
  if (span == 0) span = 360;

  for (int i = 0; i <= span; i++) {
    int idx = (start_deg + i) % 360;
    uint16_t dist = lidar->distances_360[idx];
    if (dist == 0) continue;

    sum += dist;
    s.count++;
    if (s.count == 1 || dist < s.min) s.min = dist;
    if (dist > s.max) s.max = dist;
  }

  if (s.count > 0) s.avg = (float)sum / (float)s.count;
  return s;
}

int Lidar_FindNearestAngle(const LD06* lidar, int center_deg,
                           int half_width_deg) {
  int nearest_angle = -1;
  uint16_t nearest_dist = 0;

  for (int d = center_deg - half_width_deg; d <= center_deg + half_width_deg;
       d++) {
    int idx = (d + 360) % 360;
    uint16_t dist = lidar->distances_360[idx];
    if (dist == 0) continue;

    if (nearest_angle == -1 || dist < nearest_dist) {
      nearest_dist = dist;
      nearest_angle = idx;
    }
  }

  return nearest_angle;
}

void Lidar_UpdateScore(int* score, bool raw, int max_score) {
  if (raw) {
    if (*score < max_score) (*score)++;
  } else {
    if (*score > 0) (*score)--;
  }
}

bool Lidar_IsBlocked(const LidarSector* s, float threshold_mm) {
  return (s->count > 0 && s->avg < threshold_mm);
}

bool Lidar_IsClear(const LidarSector* s, float threshold_mm) {
  return (s->count > 0 && s->avg > threshold_mm);
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

int Lidar_FindGaps(const LD06* lidar, int center_deg, int half_width_deg,
                   float min_dist_mm, int min_width_deg,
                   LidarGap* gaps, int max_gaps) {
  int found = 0;
  bool in_gap = false;
  int gap_start = 0;
  uint32_t gap_sum = 0;
  int gap_count = 0;

  int total = half_width_deg * 2 + 1;

  for (int i = 0; i < total; i++) {
    int deg = (center_deg - half_width_deg + i + 360) % 360;
    uint16_t dist = lidar->distances_360[deg];
    bool is_far = (dist > 0 && (float)dist >= min_dist_mm);

    if (is_far) {
      if (!in_gap) {
        in_gap = true;
        gap_start = deg;
        gap_sum = 0;
        gap_count = 0;
      }
      gap_sum += dist;
      gap_count++;
    } else {
      if (in_gap) {
        in_gap = false;
        int gap_end = (deg - 1 + 360) % 360;
        int width = (gap_end - gap_start + 360) % 360 + 1;

        if (width >= min_width_deg && found < max_gaps) {
          gaps[found].start_deg  = gap_start;
          gaps[found].end_deg    = gap_end;
          gaps[found].center_deg = (gap_start + width / 2) % 360;
          gaps[found].width_deg  = width;
          gaps[found].avg_dist   = (gap_count > 0)
                                     ? (float)gap_sum / (float)gap_count
                                     : 0.0f;
          found++;
        }
      }
    }
  }

  // 走査終端でギャップが続いていた場合を閉じる
  if (in_gap) {
    int gap_end = (center_deg + half_width_deg + 360) % 360;
    int width = (gap_end - gap_start + 360) % 360 + 1;

    if (width >= min_width_deg && found < max_gaps) {
      gaps[found].start_deg  = gap_start;
      gaps[found].end_deg    = gap_end;
      gaps[found].center_deg = (gap_start + width / 2) % 360;
      gaps[found].width_deg  = width;
      gaps[found].avg_dist   = (gap_count > 0)
                                 ? (float)gap_sum / (float)gap_count
                                 : 0.0f;
      found++;
    }
  }

  return found;
}
