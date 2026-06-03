#include "remote_auto2.h"

#include <stdbool.h>
#include <stdio.h>

#include "drive.h"
#include "lidar_utils.h"
#include "lighting.h"
#include "mymath.h"
#include "sensor.h"
#include "timer.h"

#define SEARCH_HALF_DEG 90
#define SECTOR_HALF_DEG 25
#define ACCELERATION 1.0f

// 速度パラメータ
#define MIN_VELOCITY 0.5f     // 障害物が近い時の最低速度 [m/s]
#define MAX_VELOCITY 1.0f     // 障害物が遠い時の最大速度 [m/s]
#define STOP_DIST_MM 1000.0f  // この距離以下で MIN_VELOCITY [mm]
#define FAST_DIST_MM 2000.0f  // この距離以上で MAX_VELOCITY [mm]

// Pure Pursuit パラメータ
#define LOOKAHEAD_TIME 0.5         // 先読み時間 [s]: L_d = v × LOOKAHEAD_TIME
#define MIN_LOOKAHEAD_M 0.5f       // 先読み距離の下限 [m]
#define MAX_LOOKAHEAD_M 3.0f       // 先読み距離の上限 [m]
#define LOOKAHEAD_DIST_RATIO 0.5f  // clear_dist に対する先読み距離の上限割合

// スローインファーストアウト
#define CORNER_THRESHOLD_DEG 20  // この角度以上で速度上限を下げ始める
#define CORNER_MAX_DEG 60        // この角度以上で MIN_VELOCITY に制限
#define CORNER_BRAKE_DEG 60      // この角度かつ速度超過でアクティブブレーキ
#define CORNER_BRAKE_STRENGTH 0.05f
#define CORNER_BRAKE_MARGIN 0.5f  // MIN_VELOCITY + この値を超えたらブレーキ

// 壁回避補正
#define WALL_HALF_DEG 80
#define WALL_DIST_MM 500.0f
#define WALL_CENTER_GAIN (0.25 / WALL_DIST_MM)

// 障害物緊急停止・切り返し
#define FRONT_HALF_DEG 15
#define EMERGENCY_DIST_MM 300.0f
#define EMERGENCY_SPEED_FACTOR 150.0f  // v[m/s] × この値[mm] を動的マージンに加算
#define REVERSE_VELOCITY -0.5f
#define REVERSE_ACCELERATION 1.0f
#define REVERSE_DURATION_MS 1000
#define POST_CLEAR_REVERSE_MS 250  // is_emergency 解除後もこの時間[ms]だけ後退を継続
#define BRAKE_COMPLETE_SPEED 0.1f
#define REAR_HALF_DEG 15
#define REAR_EMERGENCY_DIST_MM 300.0f

typedef enum {
  ALG_STATE_NORMAL,
  ALG_STATE_BRAKING,
  ALG_STATE_REVERSING,
} AlgState;

static bool initialized = false;
static bool corner_brake_done = false;
static AlgState alg_state = ALG_STATE_NORMAL;
static float reverse_steer = 0.0f;
static Timer reverse_timer;
static Timer post_clear_timer;
static bool post_clear_started = false;

void RemoteAuto2_Run(const RemoteCommand* cmd, const LD06* lidar) {
  (void)cmd;

  if (!initialized) {
    Timer_Init(&reverse_timer);
    Timer_Init(&post_clear_timer);
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
  float speed = Drive_GetSpeed();
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
  const int left_nearest = Lidar_FindNearestSector(lidar, 270, WALL_HALF_DEG, 10, 1.0f, 3, &left_dist_mm);
  const int right_nearest = Lidar_FindNearestSector(lidar, 90, WALL_HALF_DEG, 10, 1.0f, 3, &right_dist_mm);
  right_dist_mm = Constrain(right_dist_mm, 0, WALL_DIST_MM);
  left_dist_mm = Constrain(left_dist_mm, 0, WALL_DIST_MM);

  float wall_correction = 0.0f;
  wall_correction += (WALL_DIST_MM - right_dist_mm) * -WALL_CENTER_GAIN;
  wall_correction -= (WALL_DIST_MM - left_dist_mm) * -WALL_CENTER_GAIN;
  wall_correction = Constrain(wall_correction, -0.3, 0.3);
  steer = Constrain(steer + wall_correction, -1.0f, 1.0f);
  // printf("wall_corection:%f¥n", wall_correction);

  // --- 障害物緊急停止・切り返しステートマシン ---
  // 進行方向(signed_deg の半分)の前方障害物を検出
  float front_nearest_mm = 0.0f;
  Lidar_FindNearestSector(lidar, Drive_GetSteer() * MAX_STEER_ANGLE_DEG, FRONT_HALF_DEG, 5, 1.0f, 3, &front_nearest_mm);

  const bool is_emergency =
      front_nearest_mm > 0.0f &&
      front_nearest_mm < EMERGENCY_DIST_MM + Drive_GetSpeed() * EMERGENCY_SPEED_FACTOR;

  switch (alg_state) {
    case ALG_STATE_BRAKING:
      Lighting_Passing();
      if (front_nearest_mm > EMERGENCY_DIST_MM * 1.1f) {
        // 前方クリア: 切り返しを省略して通常走行に即復帰
        alg_state = ALG_STATE_NORMAL;
        break;
      }
      Drive_Brake(0.1f, 0.0f);
      if (Abs(Drive_GetSpeed()) < BRAKE_COMPLETE_SPEED) {
        // 停止完了: 最開放方向の逆ステアを設定（後退+右ステア → 前部が左へ向く）
        reverse_steer = signed_deg >= 0 ? -1.0f : 1.0f;
        Timer_Reset(&reverse_timer);
        post_clear_started = false;
        alg_state = ALG_STATE_REVERSING;
      }
      return;

    case ALG_STATE_REVERSING: {
      // 前方クリア: POST_CLEAR_REVERSE_MS 経過後に復帰
      if (!is_emergency) {
        if (!post_clear_started) {
          Timer_Reset(&post_clear_timer);
          post_clear_started = true;
        }
        if (Timer_ReadMs(&post_clear_timer) >= POST_CLEAR_REVERSE_MS) {
          alg_state = ALG_STATE_NORMAL;
          corner_brake_done = false;
          post_clear_started = false;
          break;
        }
      } else {
        post_clear_started = false;
      }
      // 後方障害物が近ければ後退中断して通常走行へ復帰
      float rear_nearest_mm = 0.0f;
      Lidar_FindNearestSector(lidar, 180, REAR_HALF_DEG, 5, 1.0f, 3, &rear_nearest_mm);
      if (rear_nearest_mm > 0.0f && rear_nearest_mm < REAR_EMERGENCY_DIST_MM) {
        alg_state = ALG_STATE_NORMAL;
        Drive_Brake(0.1f, 0.0f);
        return;
      }
      Drive_SetVelocity(REVERSE_VELOCITY, REVERSE_ACCELERATION, reverse_steer);
      if (Timer_ReadMs(&reverse_timer) >= REVERSE_DURATION_MS) {
        alg_state = ALG_STATE_NORMAL;
        corner_brake_done = false;
      }
      return;
    }

    default:  // ALG_STATE_NORMAL
      if (is_emergency) {
        alg_state = ALG_STATE_BRAKING;
        Drive_Brake(0.1f, 0.0f);
        return;
      }
      break;
  }

  // --- clear_dist_mm による速度計画 ---
  float t = Constrain((clear_dist_mm - STOP_DIST_MM) / (FAST_DIST_MM - STOP_DIST_MM), 0.0f, 1.0f);
  float base_velocity = MIN_VELOCITY + (MAX_VELOCITY - MIN_VELOCITY) * t;

  if (clear_dist_mm < STOP_DIST_MM && Drive_GetSpeed() > MIN_VELOCITY + 0.5) {
    Drive_Brake(0.05, steer);
    return;
  }

  // --- スローインファーストアウト ---
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
        Drive_GetSpeed() > MIN_VELOCITY + CORNER_BRAKE_MARGIN) {
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
