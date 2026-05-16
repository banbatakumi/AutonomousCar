#include "lighting.h"

#include <stdbool.h>

#include "main.h"
#include "pwm_out.h"
#include "timer.h"

// ブレーキ点滅パラメータ（急ブレーキ時）
#define BRAKE_BLINK_PERIOD_MS 200
#define BRAKE_BLINK_HALF_PERIOD_MS 100
#define BRAKE_DIM_BRIGHTNESS 0.05f
#define BRAKE_BLINK_SPEED_THRESHOLD 0.5f  // [m/s]

// ウィンカー点滅パラメータ
#define WINKER_HALF_PERIOD_MS 200  // 400ms 周期の半分

// パッシングパラメータ
#define PASSING_HALF_PERIOD_MS 100  // 200ms 周期の半分
#define PASSING_FLASH_COUNT 3       // 点滅回数

typedef struct {
  bool brake_active;
  float brake_strength;
  float brake_speed;
  Timer brake_timer;

  WinkerDirection winker_dir;
  bool winker_blink_state;
  Timer winker_timer;

  bool hazard_active;

  float headlight_brightness;

  bool passing_active;
  Timer passing_timer;
} LightingState;

static PwmOut front_led;
static PwmOut winker_left_led;
static PwmOut winker_right_led;
static PwmOut brake_led;

static LightingState s;

static void UpdateBrakeLed(void) {
  if (!s.brake_active) {
    PwmOut_Write(&brake_led, BRAKE_DIM_BRIGHTNESS);
    return;
  }

  // 急ブレーキかつ走行中: 200ms 周期で点滅
  if (s.brake_strength >= 1.0f && s.brake_speed > BRAKE_BLINK_SPEED_THRESHOLD) {
    float elapsed = Timer_ReadMs(&s.brake_timer);
    if (elapsed >= BRAKE_BLINK_PERIOD_MS) {
      Timer_Reset(&s.brake_timer);
      elapsed = 0.0f;
    }
    PwmOut_Write(&brake_led, elapsed < BRAKE_BLINK_HALF_PERIOD_MS ? 1.0f : 0.0f);
  } else {
    PwmOut_Write(&brake_led, 1.0f);
  }
}

static void UpdateWinkerLed(void) {
  bool blink_active = s.hazard_active || (s.winker_dir != WINKER_OFF);
  if (!blink_active) {
    PwmOut_Write(&winker_left_led, 0.0f);
    PwmOut_Write(&winker_right_led, 0.0f);
    return;
  }

  if (Timer_ReadMs(&s.winker_timer) >= WINKER_HALF_PERIOD_MS) {
    s.winker_blink_state = !s.winker_blink_state;
    Timer_Reset(&s.winker_timer);
  }

  float brightness = s.winker_blink_state ? 1.0f : 0.0f;
  if (s.hazard_active) {
    PwmOut_Write(&winker_left_led, brightness);
    PwmOut_Write(&winker_right_led, brightness);
  } else if (s.winker_dir == WINKER_LEFT) {
    PwmOut_Write(&winker_left_led, brightness);
    PwmOut_Write(&winker_right_led, 0.0f);
  } else {
    PwmOut_Write(&winker_left_led, 0.0f);
    PwmOut_Write(&winker_right_led, brightness);
  }
}

static void UpdateFrontLed(void) {
  if (!s.passing_active) {
    PwmOut_Write(&front_led, s.headlight_brightness);
    return;
  }

  // パッシング: PASSING_FLASH_COUNT 回だけ高速点滅（偶数インデックスで点灯）
  int flash_index = (int)(Timer_ReadMs(&s.passing_timer) / PASSING_HALF_PERIOD_MS);
  if (flash_index >= PASSING_FLASH_COUNT * 2) {
    s.passing_active = false;
    PwmOut_Write(&front_led, s.headlight_brightness);
    return;
  }
  PwmOut_Write(&front_led, (flash_index % 2 == 0) ? 1.0f : s.headlight_brightness);
}

void Lighting_Init(void) {
  PwmOut_Init(&front_led, &htim3, TIM_CHANNEL_1);
  PwmOut_Init(&winker_left_led, &htim3, TIM_CHANNEL_3);
  PwmOut_Init(&winker_right_led, &htim3, TIM_CHANNEL_2);
  PwmOut_Init(&brake_led, &htim3, TIM_CHANNEL_4);

  Timer_Init(&s.brake_timer);
  Timer_Init(&s.winker_timer);
  Timer_Init(&s.passing_timer);

  s.brake_active = false;
  s.brake_strength = 0.0f;
  s.brake_speed = 0.0f;
  s.winker_dir = WINKER_OFF;
  s.winker_blink_state = true;
  s.hazard_active = false;
  s.headlight_brightness = 0.0f;
  s.passing_active = false;

  // 起動確認用のウィンカー点滅
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 50; j++) {
      PwmOut_Write(&winker_left_led, j * 0.01f);
      PwmOut_Write(&winker_right_led, j * 0.01f);
      HAL_Delay(1);
    }
    HAL_Delay(150);
    PwmOut_Write(&winker_left_led, 0.0f);
    PwmOut_Write(&winker_right_led, 0.0f);
    HAL_Delay(200);
  }
}

void Lighting_Update(void) {
  UpdateBrakeLed();
  UpdateWinkerLed();
  UpdateFrontLed();
}

void Lighting_SetBrake(bool active, float strength, float speed) {
  // ブレーキが新たに有効になったときにタイマをリセットして点滅位相を揃える
  if (active && !s.brake_active) {
    Timer_Reset(&s.brake_timer);
  }
  s.brake_active = active;
  s.brake_strength = strength;
  s.brake_speed = speed;
}

void Lighting_SetWinker(WinkerDirection direction) {
  // 方向が切り替わったとき点灯フェーズから始める
  if (direction != s.winker_dir) {
    s.winker_blink_state = true;
    Timer_Reset(&s.winker_timer);
  }
  s.winker_dir = direction;
}

void Lighting_SetHeadlight(float brightness) {
  s.headlight_brightness = brightness;
}

void Lighting_SetHazard(bool active) {
  if (active && !s.hazard_active) {
    s.winker_blink_state = true;
    Timer_Reset(&s.winker_timer);
  }
  s.hazard_active = active;
}

void Lighting_Passing(void) {
  s.passing_active = true;
  Timer_Reset(&s.passing_timer);
}
