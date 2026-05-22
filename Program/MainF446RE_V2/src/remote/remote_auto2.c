#include "remote_auto2.h"

#include <stdio.h>

#include "drive.h"
#include "lidar_utils.h"
#include "mymath.h"
#include "sensor.h"
#include "timer.h"

#define SEARCH_HALF_DEG 60
#define SECTOR_HALF_DEG 10
#define ACCELERATION 1.5f

// 速度パラメータ
#define MIN_VELOCITY 1.0f     // 障害物が近い時の最低速度 [m/s]
#define MAX_VELOCITY 3.0f     // 障害物が遠い時の最大速度 [m/s]
#define STOP_DIST_MM 400.0f   // この距離以下で MIN_VELOCITY [mm]
#define FAST_DIST_MM 1500.0f  // この距離以上で MAX_VELOCITY [mm]

// Pure Pursuit パラメータ
#define LOOKAHEAD_TIME 0.5f        // 先読み時間 [s]: L_d = v × LOOKAHEAD_TIME
#define MIN_LOOKAHEAD_M 0.75f      // 先読み距離の下限 [m]
#define MAX_LOOKAHEAD_M 2.5f       // 先読み距離の上限 [m]
#define LOOKAHEAD_DIST_RATIO 0.5f  // clear_dist に対する先読み距離の上限割合

// スローインファーストアウト
#define CORNER_THRESHOLD_DEG 15  // この角度以上で速度上限を下げ始める
#define CORNER_MAX_DEG 45        // この角度以上で MIN_VELOCITY に制限
#define CORNER_BRAKE_DEG 25      // この角度かつ速度超過でアクティブブレーキ
#define CORNER_BRAKE_STRENGTH 0.05f
#define CORNER_BRAKE_MARGIN 0.2f  // MIN_VELOCITY + この値を超えたらブレーキ

// 壁回避補正
#define WALL_HALF_DEG 80
#define WALL_DIST_MM 600.0f
#define WALL_CENTER_GAIN (0.5f / WALL_DIST_MM)  // 両壁時: 左右差 → 中央寄せゲイン
#define WALL_SINGLE_GAIN (0.4f / WALL_DIST_MM)  // 片壁時: 個別回避ゲイン

static bool initialized = false;
static bool corner_brake_done = false;

void RemoteAuto2_Run(const RemoteCommand* cmd, const LD06* lidar) {
  (void)cmd;

  if (!initialized) {
    initialized = true;
  }

  float clear_dist_mm = 0.0f;
  int clear_deg = Lidar_FindClearestDirection(lidar, 0, SEARCH_HALF_DEG, SECTOR_HALF_DEG, &clear_dist_mm);
  if (clear_deg == -1) {
    Drive_Brake(0.1f, 0.0f);
    return;
  }

  int signed_deg = (clear_deg > 180) ? (clear_deg - 360) : clear_deg;

  // --- Pure Pursuit ステアリング ---
  // 先読み距離 L_d: 速度に比例し、clear_dist の LOOKAHEAD_DIST_RATIO 倍を超えない
  float speed = Drive_GetKfVelocity();
  float L_d = Constrain(speed * LOOKAHEAD_TIME, MIN_LOOKAHEAD_M, MAX_LOOKAHEAD_M);
  float clear_limit_m = clear_dist_mm * 0.001f * LOOKAHEAD_DIST_RATIO;
  if (clear_limit_m > MIN_LOOKAHEAD_M && clear_limit_m < L_d) {
    L_d = clear_limit_m;
  }

  // δ = atan(2L sin(α) / L_d)  自転車モデルの幾何学的操舵角
  float alpha = (float)signed_deg * (float)DEG_TO_RAD;
  float delta = Atan2(2.0f * WHEEL_BASE * Sin(alpha), L_d);
  float steer = Constrain(delta / MAX_STEER_ANGLE_RAD, -1.0f, 1.0f);

  // --- 壁回避補正 ---
  float left_dist_mm = 0.0f, right_dist_mm = 0.0f;
  const int left_nearest = Lidar_FindNearestSector(lidar, 270, WALL_HALF_DEG, 5, 1.0f, 3, &left_dist_mm);
  const int right_nearest = Lidar_FindNearestSector(lidar, 90, WALL_HALF_DEG, 5, 1.0f, 3, &right_dist_mm);

  float wall_correction = 0.0f;
  if (left_nearest != -1 && right_nearest != -1) {
    // 両壁検出: 左右距離の差で中央に寄せる（差がゼロ=中央が平衡点）
    float center_error = Constrain(left_dist_mm, 0.0f, WALL_DIST_MM) - Constrain(right_dist_mm, 0.0f, WALL_DIST_MM);
    wall_correction = -center_error * WALL_CENTER_GAIN;
  } else {
    // 片壁のみ: 個別回避
    if (left_nearest != -1 && left_dist_mm < WALL_DIST_MM) {
      wall_correction -= (WALL_DIST_MM - left_dist_mm) * WALL_SINGLE_GAIN;
    }
    if (right_nearest != -1 && right_dist_mm < WALL_DIST_MM) {
      wall_correction += (WALL_DIST_MM - right_dist_mm) * WALL_SINGLE_GAIN;
    }
  }
  steer = Constrain(steer + wall_correction, -1.0f, 1.0f);

  // --- clear_dist_mm による速度計画 ---
  float t = Constrain((clear_dist_mm - STOP_DIST_MM) / (FAST_DIST_MM - STOP_DIST_MM), 0.0f, 1.0f);
  float base_velocity = MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * t;

  // --- スローインファーストアウト ---
  // signed_deg の絶対値がコーナー度合いを表す
  int abs_deg = Abs(signed_deg);

  // コーナー角度に応じた速度上限 (THRESHOLD〜MAX_DEG で MAX→MIN に線形低下)
  float velocity_limit = MAX_VELOCITY;
  if (abs_deg > CORNER_THRESHOLD_DEG) {
    float ct = Constrain((float)(abs_deg - CORNER_THRESHOLD_DEG) /
                             (float)(CORNER_MAX_DEG - CORNER_THRESHOLD_DEG),
                         0.0f, 1.0f);
    velocity_limit = MAX_VELOCITY - ct * (MAX_VELOCITY - MIN_VELOCITY);
  }
  float velocity = Constrain(base_velocity, MIN_VELOCITY, velocity_limit);

  // スローイン: 速度超過かつ急カーブならブレーキ優先
  if (abs_deg > CORNER_BRAKE_DEG) {
    if (!corner_brake_done &&
        Drive_GetKfVelocity() > MIN_VELOCITY + CORNER_BRAKE_MARGIN) {
      Drive_Brake(CORNER_BRAKE_STRENGTH, steer);
      return;
    }
    corner_brake_done = true;
    Drive_SetVelocity(MIN_VELOCITY, ACCELERATION, steer);
    return;
  }

  // ファーストアウト: コーナーを抜けたらブレーキ解除して加速
  corner_brake_done = false;
  Drive_SetVelocity(velocity, ACCELERATION, steer);
}
