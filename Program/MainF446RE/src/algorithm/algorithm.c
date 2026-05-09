#include "algorithm.h"

#include <stdio.h>

#include "drive.h"
#include "lidar_utils.h"
#include "mymath.h"

// 探索・走行パラメータ
#define MIN_VELOCITY 0.25f        // 最低速度 [m/s]
#define MAX_VELOCITY 1.0f         // 障害物なし時の最大速度 [m/s]
#define ACCELERATION 1.0f         // 速度ランプ [m/s²]
#define EMERGENCY_DIST_MM 350.0f  // この距離未満で緊急停止 [mm]
#define FAST_DIST_MM 1000.0f      // この距離以上で最大速度 [mm]

// 探索範囲: 前方 ±SEARCH_HALF_DEG
#define SEARCH_HALF_DEG 90
#define SECTOR_HALF_DEG 10  // 各候補方向の評価幅 [deg]
#define FRONT_HALF_DEG 20   // 緊急停止判定の前方扇形幅 [deg]

// ±STEER_SAT_DEG で最大ステア
#define STEER_SAT_DEG 60
#define STEER_GAIN (1.0f / STEER_SAT_DEG)

// 壁接近補正パラメータ
#define WALL_HALF_DEG 20     // 左右壁検出セクタの半幅 [deg]
#define WALL_DIST_MM 300.0f  // この距離未満で補正開始 [mm]
// WALL_DIST_MM のとき補正 1.0 になるゲイン（1.5 で余裕を持たせて飽和させる）
#define WALL_CORRECTION_GAIN (1.5f / WALL_DIST_MM)

void Algorithm_Run(LD06* lidar) {
  const LidarSector front = Lidar_GetSector(lidar, 0, FRONT_HALF_DEG);

  // 前方至近距離での緊急停止
  if (front.count > 0 && front.avg < EMERGENCY_DIST_MM + Drive_GetSpeed() * 100.0f) {
    Drive_Brake(0.75f);
    return;
  }

  // 前方 ±SEARCH_HALF_DEG の範囲で最も開けた方向を探す
  // start = 360 - SEARCH_HALF_DEG、end = SEARCH_HALF_DEG で 0° またぎに対応
  int clear_deg = Lidar_FindClearestDirection(
      lidar, 360 - SEARCH_HALF_DEG, SEARCH_HALF_DEG, SECTOR_HALF_DEG);

  if (clear_deg == -1) {
    // 有効点が全くない場合は停止して待機
    Drive_Brake(0.5f);
    return;
  }

  // 最も開けた方向のセクタ統計を速度計算に使う
  const LidarSector best = Lidar_GetSector(lidar, clear_deg, SECTOR_HALF_DEG);

  // 角度 0〜359 を符号付き -180〜179 に変換（正=左、負=右）してステア値に変換
  int signed_deg = (clear_deg > 180) ? (clear_deg - 360) : clear_deg;
  float steer = Constrain((float)signed_deg * STEER_GAIN, -1.0f, 1.0f);

  // 左右壁への接近補正
  // 左壁が近い → 右へ補正（負）、右壁が近い → 左へ補正（正）
  const LidarSector wall_left = Lidar_GetSector(lidar, 90, WALL_HALF_DEG);
  const LidarSector wall_right = Lidar_GetSector(lidar, 270, WALL_HALF_DEG);

  float wall_correction = 0.0f;
  if (wall_left.count > 0 && wall_left.avg < WALL_DIST_MM) {
    wall_correction -= (WALL_DIST_MM - wall_left.avg) * WALL_CORRECTION_GAIN;
  }
  if (wall_right.count > 0 && wall_right.avg < WALL_DIST_MM) {
    wall_correction += (WALL_DIST_MM - wall_right.avg) * WALL_CORRECTION_GAIN;
  }
  steer = Constrain(steer + wall_correction, -1.0f, 1.0f);

  // 目標方向の空き距離を [0, FAST_DIST_MM] で正規化して MIN〜MAX 速度にマップ
  float best_dis_avg = 0;
  float min_dir = Lidar_FindNearestSector(lidar, 0, 60, 5, 10, 3, &best_dis_avg);
  float ratio = Constrain((float)best_dis_avg / FAST_DIST_MM, 0.0f, 1.0f);
  float target_vel = MIN_VELOCITY + ratio * (MAX_VELOCITY - MIN_VELOCITY);

  // printf("clear=%d deg, dist=%.0fmm, steer=%.2f, vel=%.2f\n",
  //        signed_deg, best.avg, steer, target_vel);

  Drive_SetVelocity(target_vel, ACCELERATION, steer);
  // Drive_SetVelocity(1, 1, 0);

  // Drive_Set(4, 3, steer);
}
