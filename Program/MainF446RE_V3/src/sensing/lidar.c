#include "lidar.h"

#include <string.h>

// ビン中心からのずれの初期値。実際のずれは必ず 0.5度以下なので、最初の点は必ず採用される
#define BIN_ERROR_UNSET 999.0f

static void StartSector(Lidar* obj, uint8_t sector_idx, uint32_t t_us) {
  memset(obj->building.distance_mm, 0, sizeof(obj->building.distance_mm));
  memset(obj->building.intensity, 0, sizeof(obj->building.intensity));
  for (int i = 0; i < LIDAR_POINTS_PER_SECTOR; i++) obj->bin_error_deg[i] = BIN_ERROR_UNSET;

  obj->building.sector_idx = sector_idx;
  obj->building.t_start_us = t_us;
  obj->building.duration_us = 0;
  obj->building.rot_speed_dps = 0;
  obj->building_last_us = t_us;
  obj->building_has_point = false;
}

static void FinishSector(Lidar* obj) {
  if (!obj->building_has_point) return;

  uint32_t elapsed = (uint32_t)(obj->building_last_us - obj->building.t_start_us);
  // セクタ1つは 10Hz 回転なら 8.3ms。u16 に収まらない値は回転が止まりかけているので飽和させる
  obj->building.duration_us = (elapsed > 65535u) ? 65535u : (uint16_t)elapsed;
  obj->building.rot_speed_dps = (obj->ld06.speed_dps > 65535.0f)
                                    ? 65535u
                                    : (uint16_t)(obj->ld06.speed_dps + 0.5f);

  if (obj->has_ready) obj->sector_drop_count++;
  obj->ready = obj->building;
  obj->has_ready = true;
  obj->sector_count++;

  uint8_t sector_idx = obj->building.sector_idx;
  for (int i = 0; i < LIDAR_POINTS_PER_SECTOR; i++) {
    obj->point_distance_mm[sector_idx * LIDAR_POINTS_PER_SECTOR + i] = obj->building.distance_mm[i];
  }
  obj->sector_update_us[sector_idx] = obj->building_last_us;
}

// 1点をビンへ置く。同じビンに既に点があれば、ビン中心に近い方を残す
static void PlacePoint(Lidar* obj, float angle_deg, uint16_t distance_mm, uint8_t intensity,
                       uint32_t t_us) {
  float bin_f = angle_deg;
  int bin = (int)(bin_f + 0.5f) % 360;
  if (bin < 0) bin += 360;

  uint8_t sector_idx = (uint8_t)(bin / LIDAR_DEG_PER_SECTOR);
  if (sector_idx != obj->building.sector_idx) {
    FinishSector(obj);
    StartSector(obj, sector_idx, t_us);
  }

  int index = bin % LIDAR_POINTS_PER_SECTOR;
  float error = bin_f - (float)bin;
  if (error < 0.0f) error = -error;
  if (error > 180.0f) error = 360.0f - error;  // 359.7度 → ビン0 のような折り返し

  if (error <= obj->bin_error_deg[index]) {
    obj->bin_error_deg[index] = error;
    obj->building.distance_mm[index] = distance_mm;
    obj->building.intensity[index] = intensity;
  }

  obj->building_last_us = t_us;
  obj->building_has_point = true;
}

void Lidar_Init(Lidar* obj, Serial* serial, TIM_HandleTypeDef* htim, uint32_t channel) {
  memset(obj, 0, sizeof(*obj));

  // memset で sector_update_us[] は0になるが、Micros() は起動直後で0付近のため
  // 「0 = 起動時刻」と「0 = 一度も更新されていない」を区別できず、LIDAR_TIMEOUT_US
  // (300ms) が経過するまで未受信セクタを誤って fresh と判定してしまう
  // (Lidar_QueryRoi 参照)。uint32_t の差分は周回しても正しく評価される (Lidar_IsOk と
  // 同じ手法) ため、あらかじめ「十分過去」の時刻で埋めておき起動直後から確実に
  // stale 扱いにする
  uint32_t stale_us = Micros() - LIDAR_TIMEOUT_US - 1u;
  for (int i = 0; i < LIDAR_SECTOR_NUM; i++) obj->sector_update_us[i] = stale_us;

  // PwmOut は htim->Init.Period を分解能として読むため、ARR と併せて張り替える
  htim->Init.Period = LIDAR_PWM_PERIOD;
  __HAL_TIM_SET_AUTORELOAD(htim, LIDAR_PWM_PERIOD);
  PwmOut_Init(&obj->motor, htim, channel);
  PwmOut_Write(&obj->motor, LIDAR_PWM_DUTY);

  LD06_Init(&obj->ld06, serial, &obj->motor);
  StartSector(obj, 0, Micros());
}

void Lidar_Update(Lidar* obj) {
  // LD06_Update は1パケットずつしか返さない (points[] が上書きされるため)。
  // 溜まっている分を取りこぼさないよう、無くなるまで繰り返す
  while (LD06_Update(&obj->ld06)) {
    obj->last_packet_us = obj->ld06.rx_time_us;

    // パケット末尾を受け取った時刻から、回転速度で各点の取得時刻を遡って求める。
    // 12点は 10Hz 回転で約2.7msにわたって取得されているため、まとめて同じ時刻として
    // 扱うと走行中の点群の歪み補正がその分ずれる
    float span_us = 0.0f;
    if (obj->ld06.speed_dps > 1.0f) {
      span_us = obj->ld06.span_deg / obj->ld06.speed_dps * 1000000.0f;
    }

    for (int i = 0; i < LD06_POINT_PER_PACK; i++) {
      const LD06_Point* point = &obj->ld06.points[i];
      float back_us =
          span_us * (float)(LD06_POINT_PER_PACK - 1 - i) / (float)(LD06_POINT_PER_PACK - 1);
      uint32_t t_us = obj->ld06.rx_time_us - (uint32_t)back_us;

      // 角度は LD06 側で 0.0-360.0 に正規化済み、センサ基準のまま渡す。実際にはセンサを
      // 裏向きに取り付けているため左右が鏡像になっているが、その補正はここでは行わない
      // (詳細は lidar.h のモジュール解説コメントを参照。上位側で座標変換すること)
      PlacePoint(obj, point->angle, point->distance, point->confidence, t_us);
    }
  }
}

const LidarSector* Lidar_TakeReadySector(Lidar* obj) {
  if (!obj->has_ready) return NULL;
  obj->has_ready = false;
  return &obj->ready;
}

bool Lidar_IsOk(const Lidar* obj) {
  if (obj->ld06.rx_count == 0) return false;
  return (uint32_t)(Micros() - obj->last_packet_us) < LIDAR_TIMEOUT_US;
}

LidarRoiResult Lidar_QueryRoi(const Lidar* obj, float center_deg, float half_width_deg,
                              float max_distance_cm) {
  LidarRoiResult result = {0, 0};
  uint32_t now = Micros();
  int half = (int)(half_width_deg + 0.5f);
  int center = (int)(center_deg + 0.5f);
  for (int offset = -half; offset <= half; offset++) {
    int bin = (center + offset + 360) % 360;
    uint8_t sector_idx = (uint8_t)(bin / LIDAR_DEG_PER_SECTOR);
    if ((uint32_t)(now - obj->sector_update_us[sector_idx]) >= LIDAR_TIMEOUT_US) continue;
    result.fresh_count++;
    uint16_t mm = obj->point_distance_mm[bin];
    if (mm != 0 && (float)mm / 10.0f <= max_distance_cm) result.hit_count++;
  }
  return result;
}

void Lidar_SetMotorDuty(Lidar* obj, float duty) {
  PwmOut_Write(&obj->motor, duty);
}
