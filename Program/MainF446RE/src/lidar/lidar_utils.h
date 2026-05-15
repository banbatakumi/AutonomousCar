#ifndef LIDAR_UTILS_H_
#define LIDAR_UTILS_H_

#include <stdbool.h>
#include <stdint.h>

#include "ld06.h"

// LD06 の distances_360[] から計算した扇形領域の統計。
// count == 0 のとき avg/min/max は未定義。
typedef struct {
  float avg;     // 有効点の平均距離 [mm]
  uint16_t min;  // 有効点の最小距離 [mm]
  uint16_t max;  // 有効点の最大距離 [mm]
  int count;     // 有効点数
} LidarSector;

// 指定した中心角を基準とした扇形領域の距離統計を取得する。
// center_deg を中心に ±half_width_deg の範囲にある有効点（距離 > 0）の
// 平均・最小・最大・個数を計算して返す。0° またぎに対応。
LidarSector Lidar_GetSector(const LD06* lidar, int center_deg,
                            int half_width_deg);

// 指定範囲内で最も近い有効点の角度を返す。
// center_deg ± half_width_deg の範囲を走査し、距離が最小の点の角度（0〜359）を返す。
// 有効点がなければ -1。
int Lidar_FindNearestAngle(const LD06* lidar, int center_deg,
                           int half_width_deg);

// セクタ単位の評価で最も近い障害物の方向を返す。
// 指定範囲を sector_half_deg 幅のセクタに分割し、各セクタの平均距離で評価する。
// min_valid_mm 未満または有効点数が min_count 未満のセクタはノイズとして除外する。
// 戻り値は最近傍セクタの中心角度（0〜359）。有効セクタがなければ -1。
// out_avg_mm は戻り値 != -1 のときのみ書き込まれる。
int Lidar_FindNearestSector(const LD06* lidar, int center_deg,
                            int half_width_deg, int sector_half_deg,
                            float min_valid_mm, int min_count,
                            float* out_avg_mm);

// 指定範囲内で最も開けた（平均距離が最大の）方向を返す。
// start_deg から end_deg（時計回り）の範囲を sector_half_width 幅のセクタで評価し、
// 平均距離が最大のセクタの中心角度を返す。有効点が全くなければ -1。
// 全周探索は start_deg=0, end_deg=359 で指定する。
int Lidar_FindClearestDirection(const LD06* lidar, int start_deg, int end_deg,
                                int sector_half_width);

#endif  // LIDAR_UTILS_H_
