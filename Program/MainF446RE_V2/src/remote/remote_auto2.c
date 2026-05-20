#include "remote_auto2.h"

#include <stdio.h>

#include "drive.h"
#include "lidar_utils.h"
#include "mymath.h"
#include "sensor.h"
#include "timer.h"

// 速度
#define MIN_VELOCITY 1.5f
#define MAX_VELOCITY 4.0f
#define ACCELERATION 2.5f

// 障害物
#define EMERGENCY_DIST_MM 300.0f
#define FAST_DIST_MM 1500.0f
#define FRONT_ULTRASONIC_OFFSET_MM 150.0f
#define STEER_SAT 1.0f

// Pure Pursuit
// 先読み距離を速度に応じて動的に変化させる（速いほど遠くを見る）
#define LOOKAHEAD_MIN_MM 400.0f
#define LOOKAHEAD_MAX_MM 1500.0f
#define LOOKAHEAD_VEL_GAIN 200.0f  // [mm / (m/s)]
// 最大舵角 [rad]。steer=1.0 に対応する物理舵角。要チューニング。
#define DELTA_MAX_RAD 0.5236f  // 30 deg

// LiDAR 探索
#define SEARCH_HALF_DEG 90
#define SECTOR_HALF_DEG 15
#define FRONT_HALF_DEG 20

// コーナリング速度制限
#define CORNER_STEER_THRESHOLD 0.25f
#define CORNER_STEER_BRAKE_THRESHOLD 0.65f

// 壁センタリング補正
#define WALL_HALF_DEG 80
#define WALL_DIST_MM 500.0f
#define WALL_CORRECTION_GAIN (0.3f / WALL_DIST_MM)

// 後退
#define REVERSE_VELOCITY -0.5f
#define REVERSE_ACCELERATION 1.0f
#define REVERSE_DURATION_MS 1500
#define BRAKE_COMPLETE_SPEED 0.1f
#define REVERSE_STEER_SAT 1.0f
#define REAR_HALF_DEG 20
#define REAR_EMERGENCY_DIST_MM 350.0f

typedef enum {
  ALG_STATE_NORMAL,
  ALG_STATE_BRAKING,
  ALG_STATE_REVERSING,
} AlgState;

static AlgState alg_state = ALG_STATE_NORMAL;
static float reverse_steer = 0.0f;
static Timer reverse_timer;
static bool alg_initialized = false;

// 自転車モデル Pure Pursuit ステア計算。
// alpha_rad: 目標方向と車体前方のなす角 [rad]（左正）
// Ld_m: 先読み距離 [m]
// 戻り値: ステア [-1.0=右最大, +1.0=左最大]
static float PurePursuit_CalcSteer(float alpha_rad, float Ld_m) {
  float delta_rad = Atan2(2.0f * WHEEL_BASE * Sin(alpha_rad), Ld_m);
  return Constrain(delta_rad / DELTA_MAX_RAD, -STEER_SAT, STEER_SAT);
}

void RemoteAuto2_Run(const RemoteCommand* cmd, const LD06* lidar) {
  (void)cmd;

  if (!alg_initialized) {
    Timer_Init(&reverse_timer);
    alg_initialized = true;
  }

  uint16_t front_ultrasonic_mm = (uint16_t)Sensor_GetUltrasonicFront();

  int clear_deg = Lidar_FindClearestDirection(
      lidar, 360 - SEARCH_HALF_DEG, SEARCH_HALF_DEG, SECTOR_HALF_DEG);

  if (clear_deg == -1) {
    Drive_Brake(0.3f, 0.0f);
    return;
  }

  int signed_deg = (clear_deg > 180) ? (clear_deg - 360) : clear_deg;
  float alpha_rad = Radians((float)signed_deg);

  float speed = Drive_GetSpeed();
  float lookahead_mm = Constrain(
      LOOKAHEAD_MIN_MM + speed * LOOKAHEAD_VEL_GAIN,
      LOOKAHEAD_MIN_MM, LOOKAHEAD_MAX_MM);
  float steer = PurePursuit_CalcSteer(alpha_rad, lookahead_mm / 1000.0f);

  float front_nearest_mm = 0.0f;
  const int front_nearest = Lidar_FindNearestSector(
      lidar, (int)((float)signed_deg * 0.5f), FRONT_HALF_DEG, 5, 1.0f, 3, &front_nearest_mm);

  const bool is_emergency =
      (front_nearest != -1 && front_nearest_mm < EMERGENCY_DIST_MM + speed * 150.0f) ||
      (front_ultrasonic_mm > 0 && front_ultrasonic_mm < EMERGENCY_DIST_MM - FRONT_ULTRASONIC_OFFSET_MM);

  switch (alg_state) {
    case ALG_STATE_BRAKING:
      if (front_nearest_mm > EMERGENCY_DIST_MM * 1.5f &&
          front_ultrasonic_mm > EMERGENCY_DIST_MM - FRONT_ULTRASONIC_OFFSET_MM) {
        alg_state = ALG_STATE_NORMAL;
        break;
      }
      Drive_Brake(0.3f, 0.0f);
      if (Abs(speed) < BRAKE_COMPLETE_SPEED) {
        int cd = Lidar_FindClearestDirection(
            lidar, 360 - SEARCH_HALF_DEG, SEARCH_HALF_DEG, SECTOR_HALF_DEG);
        if (cd != -1) {
          int sd = (cd > 180) ? (cd - 360) : cd;
          reverse_steer = -Constrain((float)sd * (STEER_SAT / 60.0f),
                                     -REVERSE_STEER_SAT, REVERSE_STEER_SAT);
        } else {
          reverse_steer = 0.0f;
        }
        Timer_Reset(&reverse_timer);
        alg_state = ALG_STATE_REVERSING;
      }
      return;

    case ALG_STATE_REVERSING: {
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

  // 左右壁から等距離を保つ補正
  float left_dist_mm = 0.0f, right_dist_mm = 0.0f;
  const int left_nearest =
      Lidar_FindNearestSector(lidar, 90, WALL_HALF_DEG, 5, 1.0f, 3, &left_dist_mm);
  const int right_nearest =
      Lidar_FindNearestSector(lidar, 270, WALL_HALF_DEG, 5, 1.0f, 3, &right_dist_mm);

  float wall_correction = 0.0f;
  if (left_nearest != -1 && left_dist_mm < WALL_DIST_MM) {
    wall_correction -= (WALL_DIST_MM - left_dist_mm) * WALL_CORRECTION_GAIN;
  }
  if (right_nearest != -1 && right_dist_mm < WALL_DIST_MM) {
    wall_correction += (WALL_DIST_MM - right_dist_mm) * WALL_CORRECTION_GAIN;
  }
  steer = Constrain(steer + wall_correction, -STEER_SAT, STEER_SAT);

  // 前方の開放度で目標速度を決定
  float front_avg_mm = 0.0f;
  Lidar_FindNearestSector(lidar, 0, 45, 5, 10, 3, &front_avg_mm);
  float ratio = Constrain(front_avg_mm / FAST_DIST_MM, 0.0f, 1.0f);
  float target_vel = MIN_VELOCITY + ratio * (MAX_VELOCITY - MIN_VELOCITY);

  // コーナリング時の速度上限（steer が大きいほど速度を絞る）
  float steer_abs = Abs(steer);
  if (steer_abs > CORNER_STEER_THRESHOLD) {
    float t = Constrain((steer_abs - CORNER_STEER_THRESHOLD) /
                            (CORNER_STEER_BRAKE_THRESHOLD - CORNER_STEER_THRESHOLD),
                        0.0f, 1.0f);
    float corner_vel_limit = MAX_VELOCITY - t * (MAX_VELOCITY - MIN_VELOCITY);
    target_vel = Constrain(target_vel, MIN_VELOCITY, corner_vel_limit);
  }

  Drive_SetVelocity(target_vel, ACCELERATION, steer);
}
