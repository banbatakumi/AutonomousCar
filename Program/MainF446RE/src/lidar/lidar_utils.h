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

// 任意の角度範囲を指定して距離統計を取得する。
// start_deg から end_deg まで時計回りに走査した有効点の統計を返す。
// start_deg == end_deg のとき全周（360°）とみなす。
LidarSector Lidar_GetSectorRange(const LD06* lidar, int start_deg,
                                 int end_deg);

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

// ヒステリシス付きスコアカウンタを更新する。
// raw が true なら score を +1（上限 max_score）、false なら -1（下限 0）する。
// 毎フレーム呼び出し、score が閾値を超えたときだけ状態変化とみなすことで
// 1 フレームのノイズによる誤判定を防ぐ。
void Lidar_UpdateScore(int* score, bool raw, int max_score);

// セクタが指定距離以内に障害物を検出しているか判定する。
// count > 0 かつ avg < threshold_mm のとき true を返す。
bool Lidar_IsBlocked(const LidarSector* s, float threshold_mm);

// セクタが指定距離より遠くまで開けているか判定する。
// count > 0 かつ avg > threshold_mm のとき true を返す。
bool Lidar_IsClear(const LidarSector* s, float threshold_mm);

// 指定範囲内で最も開けた（平均距離が最大の）方向を返す。
// start_deg から end_deg（時計回り）の範囲を sector_half_width 幅のセクタで評価し、
// 平均距離が最大のセクタの中心角度を返す。有効点が全くなければ -1。
// 全周探索は start_deg=0, end_deg=359 で指定する。
int Lidar_FindClearestDirection(const LD06* lidar, int start_deg, int end_deg,
                                int sector_half_width);

// ギャップ（障害物がなく開けた連続領域）の情報。
typedef struct {
  int start_deg;   // ギャップ開始角度 [deg]
  int end_deg;     // ギャップ終了角度 [deg]
  int center_deg;  // ギャップ中心角度 [deg]
  int width_deg;   // ギャップ幅 [deg]
  float avg_dist;  // ギャップ内の平均距離 [mm]
} LidarGap;

// 指定範囲内のギャップ（開口部）をすべて検出する。
// center_deg ± half_width_deg の範囲を線形走査し、
// min_dist_mm 以上の距離が min_width_deg 以上連続する領域をギャップとみなす。
// 検出結果を gaps 配列に格納し、検出数を返す（最大 max_gaps 個）。
int Lidar_FindGaps(const LD06* lidar, int center_deg, int half_width_deg,
                   float min_dist_mm, int min_width_deg, LidarGap* gaps,
                   int max_gaps);

#endif  // LIDAR_UTILS_H_
