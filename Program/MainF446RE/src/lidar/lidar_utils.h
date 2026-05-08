#ifndef LIDAR_UTILS_H_
#define LIDAR_UTILS_H_

#include <stdbool.h>
#include <stdint.h>

#include "ld06.h"

// セクタの統計情報
typedef struct {
  float avg;    // 有効点の平均距離 [mm] (count==0 なら 0.0f)
  uint16_t min; // 有効点の最小距離 [mm] (count==0 なら 0)
  uint16_t max; // 有効点の最大距離 [mm] (count==0 なら 0)
  int count;    // 有効点数
} LidarSector;

// center_deg を中心に ±half_width_deg の扇形の統計を返す
// center_deg, half_width_deg はどちらも度単位 (0-359)
LidarSector Lidar_GetSector(const LD06* lidar, int center_deg,
                            int half_width_deg);

// start_deg から end_deg (両端含む, 時計回りに走査) の統計を返す
// 0°をまたいで指定可能 (例: 350 → 10)
LidarSector Lidar_GetSectorRange(const LD06* lidar, int start_deg,
                                 int end_deg);

// セクタ内で最も近い障害物の角度を返す (有効点がなければ -1)
int Lidar_FindNearestAngle(const LD06* lidar, int center_deg,
                           int half_width_deg);

// ヒステリシス付きスコアカウンタの更新
// raw が true なら score を +1 (max_score 上限), false なら -1 (0 下限)
void Lidar_UpdateScore(int* score, bool raw, int max_score);

// 判定ヘルパー
// count > 0 かつ avg < threshold のとき true
bool Lidar_IsBlocked(const LidarSector* s, float threshold_mm);
// count > 0 かつ avg > threshold のとき true
bool Lidar_IsClear(const LidarSector* s, float threshold_mm);

// -----------------------------------------------------------------------
// 最大空間方向の探索
// -----------------------------------------------------------------------

// start_deg から end_deg の範囲 (0°またぎ対応) で最も平均距離が大きい方向を返す
// sector_half_width: 各候補角度を評価するセクタ幅 [deg]
// 戻り値: 0-359 の角度。有効点が全くなければ -1
// 全周探索: start_deg=0, end_deg=359
int Lidar_FindClearestDirection(const LD06* lidar, int start_deg, int end_deg,
                                int sector_half_width);

// -----------------------------------------------------------------------
// ギャップ（開口部）検出
// -----------------------------------------------------------------------

typedef struct {
  int start_deg;  // ギャップ開始角度 [deg]
  int end_deg;    // ギャップ終了角度 [deg]
  int center_deg; // ギャップ中心角度 [deg]
  int width_deg;  // ギャップ幅 [deg]
  float avg_dist; // ギャップ内の平均距離 [mm]
} LidarGap;

// center_deg ± half_width_deg の範囲でギャップを探す
// min_dist_mm:   この距離より遠い角度をギャップとみなす
// min_width_deg: この角度幅未満のギャップは無視する
// gaps:          結果格納配列 (呼び出し側で確保)
// max_gaps:      gaps 配列の最大要素数
// 戻り値: 検出したギャップ数
int Lidar_FindGaps(const LD06* lidar, int center_deg, int half_width_deg,
                   float min_dist_mm, int min_width_deg,
                   LidarGap* gaps, int max_gaps);

#endif  // LIDAR_UTILS_H_
