#include "algo_forward.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drive.h"
#include "lidar_utils.h"
#include "mymath.h"

typedef enum {
  NAV_CRUISE = 0,
  NAV_CORNER_TURN,
  NAV_ESCAPE_REVERSE,
  NAV_ESCAPE_FORWARD_TURN,
} ForwardNavState;

static int ChooseTurnDirection(const LidarSector* front_left,
                               const LidarSector* front_right,
                               const LidarSector* left,
                               const LidarSector* right) {
  float left_score = 0.0f;
  float right_score = 0.0f;

  if (front_left->count > 0)  left_score  += front_left->avg * 1.4f;
  if (left->count > 0)        left_score  += left->avg;
  if (front_right->count > 0) right_score += front_right->avg * 1.4f;
  if (right->count > 0)       right_score += right->avg;

  return (left_score >= right_score) ? 1 : -1;
}

void Algorithm_ForwardOnly(LD06* lidar) {
  static ForwardNavState nav_state = NAV_CRUISE;
  static int turn_dir = 1;  // +1: 左, -1: 右
  static int front_block_score = 0;
  static int hard_front_block_score = 0;
  static int dead_end_score = 0;
  static Timer state_timer;
  static Timer corner_cooldown_timer;
  static Timer escape_cooldown_timer;
  static bool corner_cooldown_active = false;
  static bool escape_cooldown_active = false;
  static bool timers_initialized = false;

  if (!timers_initialized) {
    Timer_Init(&state_timer);
    Timer_Init(&corner_cooldown_timer);
    Timer_Init(&escape_cooldown_timer);
    timers_initialized = true;
  }

  uint32_t state_elapsed_ms    = Timer_ReadMs(&state_timer);
  uint32_t corner_cooldown_ms  = Timer_ReadMs(&corner_cooldown_timer);
  uint32_t escape_cooldown_ms  = Timer_ReadMs(&escape_cooldown_timer);

  const LidarSector front       = Lidar_GetSector(lidar,   0, 12);
  const LidarSector front_left  = Lidar_GetSector(lidar,  35, 18);
  const LidarSector front_right = Lidar_GetSector(lidar, 325, 18);
  const LidarSector left        = Lidar_GetSector(lidar,  90, 22);
  const LidarSector right       = Lidar_GetSector(lidar, 270, 22);

  const bool left_blocked  = Lidar_IsBlocked(&left,  320.0f);
  const bool right_blocked = Lidar_IsBlocked(&right, 320.0f);

  Lidar_UpdateScore(&front_block_score,      Lidar_IsBlocked(&front, 550.0f), 6);
  Lidar_UpdateScore(&hard_front_block_score, Lidar_IsBlocked(&front, 450.0f), 6);

  const bool front_blocked      = (front_block_score >= 2);
  const bool hard_front_blocked = (hard_front_block_score >= 2);

  const bool dead_end_raw =
      hard_front_blocked && left_blocked && right_blocked &&
      Lidar_IsBlocked(&front_left,  450.0f) &&
      Lidar_IsBlocked(&front_right, 450.0f);

  Lidar_UpdateScore(&dead_end_score, dead_end_raw, 8);
  const bool dead_end = (dead_end_score >= 3);

  // 緊急停止: ただし切り返し中は潰さない
  if (front.count > 0 && front.avg < 300.0f &&
      nav_state != NAV_ESCAPE_REVERSE &&
      nav_state != NAV_ESCAPE_FORWARD_TURN) {
    if (dead_end || (hard_front_blocked && left_blocked && right_blocked)) {
      nav_state = NAV_ESCAPE_REVERSE;
      Timer_Reset(&state_timer);
      turn_dir = ChooseTurnDirection(&front_left, &front_right, &left, &right);
      escape_cooldown_active = true;
      Timer_Reset(&escape_cooldown_timer);
      Timer_Reset(&corner_cooldown_timer);
      Drive_Set(-2.0f, 5.0f, (turn_dir > 0) ? -0.9f : 0.9f);
      return;
    }

    Drive_Brake(1.0f);
    return;
  }

  switch (nav_state) {
    case NAV_CRUISE: {
      if (dead_end && (!escape_cooldown_active || escape_cooldown_ms > 200)) {
        nav_state = NAV_ESCAPE_REVERSE;
        Timer_Reset(&state_timer);
        turn_dir = ChooseTurnDirection(&front_left, &front_right, &left, &right);
        escape_cooldown_active = true;
        Timer_Reset(&escape_cooldown_timer);
        Timer_Reset(&corner_cooldown_timer);
        break;
      }

      if (front_blocked && (!corner_cooldown_active || corner_cooldown_ms > 120)) {
        nav_state = NAV_CORNER_TURN;
        Timer_Reset(&state_timer);
        turn_dir = ChooseTurnDirection(&front_left, &front_right, &left, &right);
        corner_cooldown_active = true;
        break;
      }

      float steer = 0.0f;
      if (left.count > 0 && right.count > 0) {
        steer = Constrain((left.avg - right.avg) * 0.0032f, -0.75f, 0.75f);
      } else if (left.count > 0) {
        steer = Constrain((left.avg - 380.0f) * 0.0040f, -0.7f, 0.7f);
      } else if (right.count > 0) {
        steer = Constrain((380.0f - right.avg) * 0.0040f, -0.7f, 0.7f);
      }

      if (front_left.count > 0 && front_right.count > 0) {
        steer += Constrain((front_right.avg - front_left.avg) * 0.0025f,
                           -0.35f, 0.35f);
      }
      steer = Constrain(steer, -1.0f, 1.0f);

      float max_accel = 1.8f;
      if (Lidar_IsBlocked(&front, 900.0f)) max_accel = 1.2f;
      if (Lidar_IsBlocked(&front, 650.0f)) max_accel = 1.0f;

      if (Drive_GetSpeed() < 0.05f && (!hard_front_blocked || !dead_end)) {
        if (max_accel < 1.35f) max_accel = 1.35f;
      }

      Drive_Set(max_accel, 3.5f, steer);
      break;
    }

    case NAV_CORNER_TURN: {
      state_elapsed_ms = Timer_ReadMs(&state_timer);

      if (!front_blocked && state_elapsed_ms > 120) {
        nav_state = NAV_CRUISE;
        Timer_Reset(&state_timer);
        corner_cooldown_active = true;
        Timer_Reset(&corner_cooldown_timer);
        break;
      }

      if (state_elapsed_ms > 800) {
        if (escape_cooldown_ms > 200) {
          nav_state = NAV_ESCAPE_REVERSE;
          Timer_Reset(&state_timer);
          escape_cooldown_active = true;
          Timer_Reset(&escape_cooldown_timer);
        } else {
          nav_state = NAV_CRUISE;
          Timer_Reset(&state_timer);
          corner_cooldown_active = true;
          Timer_Reset(&corner_cooldown_timer);
        }
        break;
      }

      float steer = (turn_dir > 0) ? 1.0f : -1.0f;
      float max_accel = hard_front_blocked ? 1.5f : 2.0f;
      if (Drive_GetSpeed() < 0.05f && max_accel < 1.4f) max_accel = 1.4f;
      Drive_Set(max_accel, 4.0f, steer);
      break;
    }

    case NAV_ESCAPE_REVERSE: {
      state_elapsed_ms = Timer_ReadMs(&state_timer);

      if (state_elapsed_ms <= 1200) {
        float reverse_steer = (turn_dir > 0) ? -0.9f : 0.9f;
        Drive_Set(-2.0f, 6.0f, reverse_steer);
        break;
      }

      nav_state = NAV_ESCAPE_FORWARD_TURN;
      Timer_Reset(&state_timer);
      break;
    }

    case NAV_ESCAPE_FORWARD_TURN: {
      state_elapsed_ms = Timer_ReadMs(&state_timer);

      if (state_elapsed_ms > 1500) {
        nav_state = NAV_CRUISE;
        Timer_Reset(&state_timer);
        corner_cooldown_active = true;
        Timer_Reset(&corner_cooldown_timer);
        break;
      }

      float steer = (turn_dir > 0) ? 1.0f : -1.0f;
      Drive_Set(2.0f, 5.0f, steer);
      break;
    }

    default:
      nav_state = NAV_CRUISE;
      Timer_Reset(&state_timer);
      Drive_Brake(1.0f);
      break;
  }
}
