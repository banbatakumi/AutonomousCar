#include "algo_forward.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drive.h"
#include "mymath.h"

typedef enum {
  NAV_CRUISE = 0,
  NAV_CORNER_TURN,
  NAV_ESCAPE_REVERSE,
  NAV_ESCAPE_FORWARD_TURN,
} ForwardNavState;

typedef struct {
  float avg;
  int count;
} SectorStat;

static SectorStat GetSectorAverage(const LD06* lidar, int center_deg,
                                   int half_width_deg) {
  int sum = 0;
  int count = 0;

  for (int d = center_deg - half_width_deg; d <= center_deg + half_width_deg;
       d++) {
    int idx = (d + 360) % 360;
    uint16_t dist = lidar->distances_360[idx];
    if (dist > 0) {
      sum += dist;
      count++;
    }
  }

  SectorStat stat = {0.0f, count};
  if (count > 0) {
    stat.avg = (float)sum / (float)count;
  }
  return stat;
}

static int ChooseTurnDirection(const SectorStat* front_left,
                               const SectorStat* front_right,
                               const SectorStat* left,
                               const SectorStat* right) {
  float left_score = 0.0f;
  float right_score = 0.0f;

  if (front_left->count > 0)
    left_score += front_left->avg * 1.4f;
  if (left->count > 0)
    left_score += left->avg;
  if (front_right->count > 0)
    right_score += front_right->avg * 1.4f;
  if (right->count > 0)
    right_score += right->avg;

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

  uint32_t state_elapsed_ms = Timer_ReadMs(&state_timer);
  uint32_t corner_cooldown_ms = Timer_ReadMs(&corner_cooldown_timer);
  uint32_t escape_cooldown_ms = Timer_ReadMs(&escape_cooldown_timer);

  const SectorStat front = GetSectorAverage(lidar, 0, 12);
  const SectorStat front_left = GetSectorAverage(lidar, 35, 18);
  const SectorStat front_right = GetSectorAverage(lidar, 325, 18);
  const SectorStat left = GetSectorAverage(lidar, 90, 22);
  const SectorStat right = GetSectorAverage(lidar, 270, 22);

  const bool left_blocked = (left.count > 0 && left.avg < 320.0f);
  const bool right_blocked = (right.count > 0 && right.avg < 320.0f);
  const bool front_block_raw = (front.count > 0 && front.avg < 550.0f);
  const bool front_clear_raw = (front.count > 0 && front.avg > 650.0f);
  const bool hard_front_block_raw = (front.count > 0 && front.avg < 450.0f);
  const bool hard_front_clear_raw = (front.count > 0 && front.avg > 500.0f);

  if (front_block_raw) {
    if (front_block_score < 6)
      front_block_score++;
  } else if (front_clear_raw) {
    if (front_block_score > 0)
      front_block_score--;
  }

  if (hard_front_block_raw) {
    if (hard_front_block_score < 6)
      hard_front_block_score++;
  } else if (hard_front_clear_raw) {
    if (hard_front_block_score > 0)
      hard_front_block_score--;
  }

  const bool front_blocked = (front_block_score >= 2);
  const bool hard_front_blocked = (hard_front_block_score >= 2);
  const bool dead_end_raw =
      hard_front_blocked && left_blocked && right_blocked &&
      (front_left.count > 0 && front_left.avg < 450.0f) &&
      (front_right.count > 0 && front_right.avg < 450.0f);

  if (dead_end_raw) {
    if (dead_end_score < 8)
      dead_end_score++;
  } else {
    if (dead_end_score > 0)
      dead_end_score--;
  }
  const bool dead_end = (dead_end_score >= 3);

  // 緊急停止: ただし切り返し中は潰さない
  if (front.count > 0 && front.avg < 300.0f &&
      nav_state != NAV_ESCAPE_REVERSE &&
      nav_state != NAV_ESCAPE_FORWARD_TURN) {
    // printf("EMERGENCY BRAKE: front=%.1f, left=%.1f, right=%.1f nav=%d ms=%lu esc_cd=%lu drive_err=%d\n",
    //        front.avg, left.avg, right.avg, (int)nav_state,
    //        (unsigned long)state_elapsed_ms, (unsigned long)escape_cooldown_ms,
    //        Drive_HasError());

    if (dead_end || (hard_front_blocked && left_blocked && right_blocked)) {
      nav_state = NAV_ESCAPE_REVERSE;
      Timer_Reset(&state_timer);
      turn_dir = ChooseTurnDirection(&front_left, &front_right, &left, &right);
      escape_cooldown_active = true;
      Timer_Reset(&escape_cooldown_timer);
      Timer_Reset(&corner_cooldown_timer);
      // printf("ALGDBG: ENTER ESCAPE FROM EMERGENCY (dir=%d)\n", turn_dir);
      Drive_Set(-2.0f, 5.0f, (turn_dir > 0) ? -0.9f : 0.9f);
      return;
    }

    Drive_Brake(1.0f);
    return;
  }

  // デバッグ出力（スコア・判定）
  if (dead_end || front_blocked || hard_front_blocked) {
    // printf("ALGDBG: front=%.1f fscore=%d hscore=%d dscore=%d left=%.1f right=%.1f nav=%d ms=%lu esc_cd=%lu\n",
    //        front.avg, front_block_score, hard_front_block_score, dead_end_score,
    //        left.avg, right.avg, (int)nav_state,
    //        (unsigned long)state_elapsed_ms, (unsigned long)escape_cooldown_ms);
  }

  switch (nav_state) {
    case NAV_CRUISE: {
      // 切り返し開始: dead_end を検出したら入る（ただし連続再突入を防止）
      if (dead_end && (!escape_cooldown_active || escape_cooldown_ms > 200)) {
        nav_state = NAV_ESCAPE_REVERSE;
        Timer_Reset(&state_timer);
        turn_dir =
            ChooseTurnDirection(&front_left, &front_right, &left, &right);
        // printf("ALGDBG: ENTER ESCAPE (dir=%d) nav=%d\n", turn_dir,
        //        (int)nav_state);
        // 切り返し直後の早期再突入を防ぐクールダウン
        escape_cooldown_active = true;
        Timer_Reset(&escape_cooldown_timer);
        Timer_Reset(&corner_cooldown_timer);
        break;
      }

      if (front_blocked && (!corner_cooldown_active || corner_cooldown_ms > 120)) {
        nav_state = NAV_CORNER_TURN;
        Timer_Reset(&state_timer);
        turn_dir =
            ChooseTurnDirection(&front_left, &front_right, &left, &right);
        corner_cooldown_active = true;
        break;
      }

      float steer = 0.0f;
      if (left.count > 0 && right.count > 0) {
        // 左右の壁中心へ寄せる
        steer = Constrain((left.avg - right.avg) * 0.0032f, -0.75f, 0.75f);
      } else if (left.count > 0) {
        // 左壁追従
        steer = Constrain((left.avg - 380.0f) * 0.0040f, -0.7f, 0.7f);
      } else if (right.count > 0) {
        // 右壁追従
        steer = Constrain((380.0f - right.avg) * 0.0040f, -0.7f, 0.7f);
      }

      if (front_left.count > 0 && front_right.count > 0) {
        // 斜め前の近い側を避ける
        steer += Constrain((front_right.avg - front_left.avg) * 0.0025f,
                           -0.35f, 0.35f);
      }
      steer = Constrain(steer, -1.0f, 1.0f);

      float max_accel = 1.8f;
      if (front.count > 0 && front.avg < 900.0f)
        max_accel = 1.2f;
      if (front.count > 0 && front.avg < 650.0f)
        max_accel = 1.0f;

      // 壁際で減速しすぎて失速しないように最低クリープを確保
      if (Drive_GetSpeed() < 0.05f && (!hard_front_blocked || !dead_end)) {
        if (max_accel < 1.35f)
          max_accel = 1.35f;
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
        // 長時間旋回で抜けられない場合は切り返しに移行（ただしクールダウンで抑制）
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
      // 旋回中にトルク不足で停止しないよう最低加速度を上げる
      float max_accel = hard_front_blocked ? 1.5f : 2.0f;
      if (Drive_GetSpeed() < 0.05f) {
        if (max_accel < 1.4f)
          max_accel = 1.4f;
      }
      Drive_Set(max_accel, 4.0f, steer);
      break;
    }

    case NAV_ESCAPE_REVERSE: {
      state_elapsed_ms = Timer_ReadMs(&state_timer);

      if (state_elapsed_ms <= 1200) {
        // 切り返し1段目: 後退しながら向きを作る
        float reverse_steer = (turn_dir > 0) ? -0.9f : 0.9f;
        // 最初のティックではランプを飛ばして即時逆進を試行
        if (state_elapsed_ms < 20) {
          // printf("ALGDBG: ESCAPE_REV IMMEDIATE accel=%.2f steer=%.2f ticks=%d nav=%d\n",
          //        -2.0f, reverse_steer, (int)state_elapsed_ms, (int)nav_state);
          Drive_Set(-2.0f, 6.0f, reverse_steer);
        } else {
          // printf("ALGDBG: ESCAPE_REV CMD accel=%.2f ramp=%.2f steer=%.2f ticks=%d nav=%d\n",
          //        -1.2f, 6.0f, reverse_steer, (int)state_elapsed_ms,
          //        (int)nav_state);
          Drive_Set(-2.0f, 6.0f, reverse_steer);
        }
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

      // 切り返し2段目: 前進しながら脱出方向へ旋回
      float steer = (turn_dir > 0) ? 1.0f : -1.0f;
      // printf("ALGDBG: ESCAPE_FWD CMD accel=%.2f ramp=%.2f steer=%.2f ticks=%d nav=%d\n",
      //  1.2f, 5.0f, steer, (int)state_elapsed_ms, (int)nav_state);
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
