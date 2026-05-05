#include "algo_forward.h"

#include <stdbool.h>

#include "drive.h"

void Algorithm_ForwardOnly(LD06* lidar) {
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
    Drive_Brake(2.0f);
  } else {
    Drive_Set(2, 2, 0.0f);
  }
}
