#include "algorithm.h"

#include "drive.h"
#include "mymath.h"

void Algorithm_Run(LD06* lidar) {
  // --- 1. 両側壁の距離取得 (左右のセンタリング用) ---
  int left_dist = 0, right_dist = 0;
  int count_l = 0, count_r = 0;

  // 左(90度付近)の平均距離
  for (int i = 45; i <= 120; i++) {
    if (lidar->distances_360[i] > 0) {
      left_dist += lidar->distances_360[i];
      count_l++;
    }
  }
  // 右(270度付近)の平均距離
  for (int i = 240; i <= 315; i++) {
    if (lidar->distances_360[i] > 0) {
      right_dist += lidar->distances_360[i];
      count_r++;
    }
  }
  if (count_l > 0)
    left_dist /= count_l;
  if (count_r > 0)
    right_dist /= count_r;

  // --- 2. 目標ステアリング(steer)の計算 ---
  float error = (float)(left_dist - 300);
  float Kp = 0.004f;

  // errorがプラス(左が遠い)の時に、左(-45°側)へ行くか右(45°側)へ行くかは
  // 機体の仕様に合わせて符号を調整してください。
  float steer = Kp * error;
  steer = Constrain(steer, -1.0f, 1.0f);

  // --- 3. 進行方向(タイヤが向いている方向)の距離確認 ---
  // steer (-1.0 〜 1.0) を角度 (-45.0° 〜 45.0°) に変換
  float look_ahead_angle = steer * 45.0f;
  int base_angle = (int)look_ahead_angle;

  int front_dist = 0;
  int count_f = 0;

  // 進行方向の角度を中心に ±5度 の距離を確認する
  for (int i = base_angle - 5; i <= base_angle + 5; i++) {
    // 角度がマイナスになった場合を考慮して360で丸める (-45° -> 315°)
    int idx = (i + 360) % 360;
    if (lidar->distances_360[idx] > 0) {
      front_dist += lidar->distances_360[idx];
      count_f++;
    }
  }
  if (count_f > 0)
    front_dist /= count_f;

  // --- 4. 走行制御 (ブレーキ or 進行) ---

  static bool is_braking = false;

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
    Drive_Brake(2.0f);
  } else {
    Drive_SetVelocity(1.5f, 1.5f, 0);
  }
}
