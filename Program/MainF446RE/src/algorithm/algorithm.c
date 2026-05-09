#include "algorithm.h"

#include <stdio.h>

#include "drive.h"
#include "lidar_utils.h"
#include "mymath.h"

// 追従パラメータ
#define TARGET_DIST_MM 500.0f      // 目標距離 [mm]
#define DEAD_ZONE_MM 10.0f         // 停止維持のデッドゾーン [mm]
#define SPEED_GAIN 0.01f           // 距離誤差→速度ゲイン [m/s per mm]
#define MAX_FOLLOW_SPEED 2.5f      // 前進最大速度 [m/s]
#define MAX_REVERSE_SPEED 1.5f     // 後退最大速度 [m/s]
#define STEER_GAIN (1.0f / 60.0f)  // ±45°で最大ステア
#define SEARCH_HALF_DEG 60         // 探索範囲 ±45°
#define EMERGENCY_DIST_MM 300.0f   // 緊急ブレーキ距離 [mm]
#define SECTOR_HALF_DEG 5          // セクタ評価幅 ±5°
#define MIN_VALID_DIST_MM 50.0f    // この距離未満はノイズとして無視
#define MIN_SECTOR_COUNT 3         // セクタ内の最低有効点数

void Algorithm_Run(LD06* lidar) {
  // float dist_mm = 0.0f;
  // int deg = Lidar_FindNearestSector(lidar, 0, SEARCH_HALF_DEG,
  //                                   SECTOR_HALF_DEG, MIN_VALID_DIST_MM,
  //                                   MIN_SECTOR_COUNT, &dist_mm);

  // if (deg == -1) {
  //   Drive_Brake(1.0f);
  //   printf("Follow: no target in ±%ddeg\n", SEARCH_HALF_DEG);
  //   return;
  // }

  // // 0-359° → -180〜+179° に変換 (正=左, 負=右)
  // int signed_deg = (deg > 180) ? (deg - 360) : deg;

  // printf("Follow: deg=%d, dist=%.0fmm\n", signed_deg, dist_mm);

  // // 緊急ブレーキ: 近すぎる場合
  // if (dist_mm < EMERGENCY_DIST_MM) {
  //   Drive_Brake(1.0f);
  //   return;
  // }

  // // ステアリング: 角度に比例、±45°で飽和
  // float steer = Constrain((float)signed_deg * STEER_GAIN, -1.0f, 1.0f);

  // float dist_error = dist_mm - TARGET_DIST_MM;

  // if (dist_error > DEAD_ZONE_MM) {
  //   // 目標より遠い: 前進
  //   float vel = Constrain(dist_error * SPEED_GAIN, 0.0f, MAX_FOLLOW_SPEED);
  //   Drive_SetVelocity(vel, 2.0f, steer);
  // } else if (dist_error < -DEAD_ZONE_MM) {
  //   // 目標より近い: 後退
  //   float vel = Constrain(dist_error * SPEED_GAIN, -MAX_REVERSE_SPEED, 0.0f);
  //   Drive_SetVelocity(vel, 2.0f, steer);
  // } else {
  //   // デッドゾーン: 停止保持
  //   Drive_Brake(0.5f);
  // }

  static bool is_braking = false;
  int front_dist = lidar->distances_360[0];

  if (front_dist > 0) {
    if (!is_braking) {
      // 走行中：速度に応じてブレーキ開始距離を計算
      float brake_threshold = Drive_GetSpeed() * 200.0f + 300.0f;
      if (front_dist < brake_threshold) {
        is_braking = true;
      }
    } else {
      // ブレーキ中：停止時の閾値(400)より十分離れるまで解除しない
      if (front_dist > 300.0f && Drive_GetSpeed() < 0.1) {
        is_braking = false;
      }
    }
  } else {
    // 障害物が見えなくなった場合は解除
    is_braking = false;
  }

  if (is_braking) {
    Drive_Brake(0.75);
  } else {
    Drive_Set(5, 3, 0.0f);
  }
}
