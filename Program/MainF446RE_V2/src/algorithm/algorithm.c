#include "algorithm.h"

#include <stdio.h>

#include "drive.h"
#include "lidar_utils.h"
#include "mymath.h"

// 探索・走行パラメータ
#define MIN_VELOCITY 1.0f                  // 最低速度 [m/s]
#define MAX_VELOCITY 3.0f                  // 障害物なし時の最大速度 [m/s]
#define ACCELERATION 5.0f                  // 速度ランプ [m/s²]
#define EMERGENCY_DIST_MM 300.0f           // この距離未満で緊急停止 [mm]
#define FAST_DIST_MM 1500.0f               // この距離以上で最大速度 [mm]
#define STEER_SAT 1.0f                     // ステア飽和値（0〜1、1=最大舵角）
#define FRONT_ULTRASONIC_OFFSET_MM 150.0f  // 前方超音波はLiDARより150mm前方に搭載

// 探索範囲: 前方 ±SEARCH_HALF_DEG
#define SEARCH_HALF_DEG 90
#define SECTOR_HALF_DEG 15  // 各候補方向の評価幅 [deg]
#define FRONT_HALF_DEG 20   // 緊急停止判定の前方扇形幅 [deg]

// ±STEER_SAT_DEG で最大ステア
#define STEER_SAT_DEG 60
#define STEER_GAIN (STEER_SAT / STEER_SAT_DEG)

// 壁接近補正パラメータ
#define WALL_HALF_DEG 80     // 左右壁検出セクタの半幅 [deg]
#define WALL_DIST_MM 400.0f  // この距離未満で補正開始 [mm]
// WALL_DIST_MM のとき補正 1.0 になるゲイン（1.5 で余裕を持たせて飽和させる）
#define WALL_CORRECTION_GAIN (0.5f / WALL_DIST_MM)

// カーブ減速パラメータ
#define CORNER_STEER_THRESHOLD 0.2f        // この値以上で緩やかに減速開始（0〜1）
#define CORNER_STEER_BRAKE_THRESHOLD 0.6f  // この値以上でブレーキ開始（0〜1）

// 切り返しパラメータ
#define REVERSE_VELOCITY -0.5f         // 後退速度 [m/s]
#define REVERSE_ACCELERATION 1.0f      // 後退加速度 [m/s²]
#define REVERSE_DURATION_MS 1500       // 後退継続時間 [ms]
#define BRAKE_COMPLETE_SPEED 0.1f      // 停止判定速度 [m/s]
#define REVERSE_STEER_SAT 1.0f         // 切り返しステア飽和値
#define REAR_HALF_DEG 20               // 後方障害物検出セクタの半幅 [deg]
#define REAR_EMERGENCY_DIST_MM 350.0f  // この距離未満で後退中断 [mm]

// 緊急停止→切り返しのステート
typedef enum {
  ALG_STATE_NORMAL,
  ALG_STATE_BRAKING,
  ALG_STATE_REVERSING,
} AlgState;

static AlgState alg_state = ALG_STATE_NORMAL;
static float reverse_steer = 0.0f;
static Timer reverse_timer;
static bool alg_initialized = false;
static bool curve_brake_done = false;  // MIN_VELOCITY 達成後の再ブレーキ抑制フラグ

void Algorithm_Run(const LD06* lidar, uint16_t front_ultrasonic_mm) {
  if (!alg_initialized) {
    Timer_Init(&reverse_timer);
    alg_initialized = true;
  }
  // 通常走行ロジック
  // 前方 ±SEARCH_HALF_DEG の範囲で最も開けた方向を探す
  int clear_deg = Lidar_FindClearestDirection(
      lidar, 0, SEARCH_HALF_DEG, SECTOR_HALF_DEG, NULL);

  if (clear_deg == -1) {
    Drive_Brake(0.3f, 0.0f);
    return;
  }

  // 角度 0〜359 を符号付き -180〜179 に変換（正=左、負=右）してステア値に変換
  int signed_deg = (clear_deg > 180) ? (clear_deg - 360) : clear_deg;
  float steer = Constrain((float)signed_deg * STEER_GAIN, -STEER_SAT, STEER_SAT);

  // 前方の障害物距離に応じて速度を決定
  float front_nearest_mm = 0.0f;
  const int front_nearest = Lidar_FindNearestSector(lidar, signed_deg * 0.5, FRONT_HALF_DEG, 5, 1.0f, 3,
                                                    &front_nearest_mm);

  const bool is_emergency =
      (front_nearest != -1 && front_nearest_mm < EMERGENCY_DIST_MM + Drive_GetSpeed() * 150.0f) || (front_ultrasonic_mm > 0 && front_ultrasonic_mm < EMERGENCY_DIST_MM - FRONT_ULTRASONIC_OFFSET_MM);

  // 緊急停止・切り返しステートマシン
  switch (alg_state) {
    case ALG_STATE_BRAKING:
      if (front_nearest_mm > EMERGENCY_DIST_MM * 1.5f && front_ultrasonic_mm > EMERGENCY_DIST_MM - FRONT_ULTRASONIC_OFFSET_MM) {
        // 前方クリア: 切り返しを省略して通常走行に即復帰
        alg_state = ALG_STATE_NORMAL;
        break;
      }
      Drive_Brake(0.3f, 0.0f);
      if (Abs(Drive_GetSpeed()) < BRAKE_COMPLETE_SPEED) {
        // 停止完了: 前方の最も開けた方向を検出し、後退中に前部がその方向を向くよう
        // 逆符号のステアを設定する（後退 + 右ステア → 前部が左へ向く）
        int clear_deg = Lidar_FindClearestDirection(
            lidar, 0, SEARCH_HALF_DEG, SECTOR_HALF_DEG, NULL);
        if (clear_deg != -1) {
          int signed_deg = (clear_deg > 180) ? (clear_deg - 360) : clear_deg;
          reverse_steer = -Constrain((float)signed_deg * STEER_GAIN,
                                     -REVERSE_STEER_SAT, REVERSE_STEER_SAT);
        } else {
          reverse_steer = 0.0f;
        }
        Timer_Reset(&reverse_timer);
        alg_state = ALG_STATE_REVERSING;
      }
      return;

    case ALG_STATE_REVERSING: {
      // 後方障害物が近い場合は後退を中断して通常走行へ復帰
      const LidarSector rear = Lidar_GetSector(lidar, 180, REAR_HALF_DEG);
      if (rear.count > 0 && rear.avg < REAR_EMERGENCY_DIST_MM) {
        alg_state = ALG_STATE_NORMAL;
        Drive_Brake(0.3f, 0.0f);
        return;
      }
      Drive_SetVelocity(REVERSE_VELOCITY, REVERSE_ACCELERATION, reverse_steer);
      if (Timer_ReadMs(&reverse_timer) >= REVERSE_DURATION_MS) {
        alg_state = ALG_STATE_NORMAL;
      }
      return;
    }

    default:  // ALG_STATE_NORMAL
      if (is_emergency) {
        alg_state = ALG_STATE_BRAKING;
        Drive_Brake(0.3f, 0.0f);
        return;
      }
      break;
  }

  // 左右壁への接近補正
  // 左壁が近い → 右へ補正（負）、右壁が近い → 左へ補正（正）
  float left_dist_mm = 0.0f, right_dist_mm = 0.0f;
  const int left_nearest = Lidar_FindNearestSector(lidar, 90, WALL_HALF_DEG, 5, 1.0f, 3,
                                                   &left_dist_mm);
  const int right_nearest = Lidar_FindNearestSector(lidar, 270, WALL_HALF_DEG, 5, 1.0f, 3,
                                                    &right_dist_mm);

  float wall_correction = 0.0f;
  if (left_nearest != -1 && left_dist_mm < WALL_DIST_MM) {
    wall_correction -= (WALL_DIST_MM - left_dist_mm) * WALL_CORRECTION_GAIN;
  }
  if (right_nearest != -1 && right_dist_mm < WALL_DIST_MM) {
    wall_correction += (WALL_DIST_MM - right_dist_mm) * WALL_CORRECTION_GAIN;
  }
  steer = Constrain(steer + wall_correction, -STEER_SAT, STEER_SAT);

  // 目標方向の空き距離を [0, FAST_DIST_MM] で正規化して MIN〜MAX 速度にマップ
  float best_dis_avg = 0;
  Lidar_FindNearestSector(lidar, 0, 45, 5, 10, 3, &best_dis_avg);
  float ratio = Constrain((float)best_dis_avg / FAST_DIST_MM, 0.0f, 1.0f);
  float target_vel = MIN_VELOCITY + ratio * (MAX_VELOCITY - MIN_VELOCITY);

  // カーブ減速・ブレーキ
  float steer_abs = Abs(steer);
  if (steer_abs > CORNER_STEER_THRESHOLD) {
    // 緩いカーブ: steer=THRESHOLD で上限 MAX、steer=BRAKE_THRESHOLD で上限 MIN に線形補間
    float t = Constrain((steer_abs - CORNER_STEER_THRESHOLD) /
                            (CORNER_STEER_BRAKE_THRESHOLD - CORNER_STEER_THRESHOLD),
                        0.0f, 1.0f);
    float corner_vel_limit = MAX_VELOCITY - t * (MAX_VELOCITY - MIN_VELOCITY);
    target_vel = Constrain(target_vel, MIN_VELOCITY, corner_vel_limit);
  }
  if (steer_abs > CORNER_STEER_THRESHOLD) {
    if (!curve_brake_done && Drive_GetSpeed() > MIN_VELOCITY + 0.2f) {
      // ブレーキ未完了かつ速度超過: ブレーキで MIN_VELOCITY まで減速しながらステア
      Drive_Brake(0.1f, steer);
      return;
    }
    // MIN_VELOCITY 到達済み: フラグを立てて再ブレーキを抑制し、前方距離ベースで加速
    curve_brake_done = true;
    Drive_SetVelocity(MIN_VELOCITY, ACCELERATION, steer);
    return;
  }
  // ステアが閾値以下に戻ったらフラグリセット（次のカーブで再びブレーキ可能に）
  curve_brake_done = false;

  Drive_SetVelocity(target_vel, ACCELERATION, steer);
}
