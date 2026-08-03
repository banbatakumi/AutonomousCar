#ifndef LIGHTING_H_
#define LIGHTING_H_

#include <stdbool.h>

#include "pwm_out.h"
#include "timer.h"

// ウィンカー・ハザードの状態
typedef enum {
  LIGHTING_WINKER_OFF = 0,
  LIGHTING_WINKER_LEFT,
  LIGHTING_WINKER_RIGHT,
  LIGHTING_WINKER_HAZARD,
} LightingWinkerState;

// 前照灯の点灯パターン
typedef enum {
  LIGHTING_HEADLIGHT_OFF = 0,
  LIGHTING_HEADLIGHT_DAYTIME,  // デイライト (減光、日中の被視認性向上用)
  LIGHTING_HEADLIGHT_NORMAL,   // 通常点灯 (夜間走行用、全光量)
} LightingHeadlightMode;

typedef struct {
  PwmOut front_light;
  PwmOut rear_light;
  PwmOut left_winker;
  PwmOut right_winker;

  LightingHeadlightMode headlight_mode;
  bool brake_on;

  LightingWinkerState winker_state;
  Timer winker_timer;
} Lighting;

/**
 * @brief ライティング系統を初期化する。前照灯・尾灯・左右ウィンカーをそれぞれ指定した TIM チャンネルに割り当てる。
 */
void Lighting_Init(Lighting* obj, TIM_HandleTypeDef* front_htim, uint32_t front_channel,
                    TIM_HandleTypeDef* rear_htim, uint32_t rear_channel,
                    TIM_HandleTypeDef* left_htim, uint32_t left_channel,
                    TIM_HandleTypeDef* right_htim, uint32_t right_channel);

/**
 * @brief 前照灯を指定パターンで点灯する。OFF 以外では尾灯も薄暗く連動点灯する。
 */
void Lighting_SetHeadlight(Lighting* obj, LightingHeadlightMode mode);

/**
 * @brief ブレーキランプ (尾灯の増灯) を点灯/消灯する。
 */
void Lighting_SetBrake(Lighting* obj, bool on);

/**
 * @brief ウィンカー/ハザードの状態を設定する。
 */
void Lighting_SetWinker(Lighting* obj, LightingWinkerState state);

/**
 * @brief ウィンカーの点滅処理を更新する。メインループで毎ティック呼ぶこと。
 */
void Lighting_Update(Lighting* obj);

#endif  // LIGHTING_H_
