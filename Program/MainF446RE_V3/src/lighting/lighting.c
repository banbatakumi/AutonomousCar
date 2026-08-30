#include "lighting.h"

#define LIGHTING_DAYTIME_DUTY 0.1f
#define LIGHTING_TAILLIGHT_DUTY 0.1f

// マツダ車のような上品な点滅を再現するため、単純な on/off ではなく
// 「素早く点灯 → 一定時間保持 → ゆっくり消灯 → 間隔を空ける」の
// 4 フェーズで duty を連続的に変化させる。
#define LIGHTING_WINKER_FADE_IN_MS 50u
#define LIGHTING_WINKER_HOLD_MS 300u
#define LIGHTING_WINKER_FADE_OUT_MS 100u
#define LIGHTING_WINKER_GAP_MS 500u
#define LIGHTING_WINKER_PERIOD_MS \
  (LIGHTING_WINKER_FADE_IN_MS + LIGHTING_WINKER_HOLD_MS + LIGHTING_WINKER_FADE_OUT_MS + LIGHTING_WINKER_GAP_MS)

#define LIGHTING_BRAKE_FLASH_ON_MS 125u
#define LIGHTING_BRAKE_FLASH_OFF_MS 125u
#define LIGHTING_BRAKE_FLASH_PERIOD_MS (LIGHTING_BRAKE_FLASH_ON_MS + LIGHTING_BRAKE_FLASH_OFF_MS)

// elapsed_ms (1周期内, 0 <= elapsed_ms < PERIOD_MS) における duty を返す。
static float Lighting_WinkerDutyAt(uint32_t elapsed_ms) {
  if (elapsed_ms < LIGHTING_WINKER_FADE_IN_MS) {
    return (float)elapsed_ms / LIGHTING_WINKER_FADE_IN_MS;
  }
  elapsed_ms -= LIGHTING_WINKER_FADE_IN_MS;

  if (elapsed_ms < LIGHTING_WINKER_HOLD_MS) {
    return 1.0f;
  }
  elapsed_ms -= LIGHTING_WINKER_HOLD_MS;

  if (elapsed_ms < LIGHTING_WINKER_FADE_OUT_MS) {
    return 1.0f - (float)elapsed_ms / LIGHTING_WINKER_FADE_OUT_MS;
  }

  return 0.0f;
}

// パッシング中は前照灯だけを全光量にする。尾灯を連動させないのは、消灯状態でパッシングした
// ときに尾灯まで一緒に瞬くと後続車から見て制動と紛らわしいため
static void Lighting_ApplyFrontLight(Lighting* obj) {
  float duty = 0.0f;
  if (obj->passing_on || obj->headlight_mode == LIGHTING_HEADLIGHT_NORMAL) {
    duty = 1.0f;
  } else if (obj->headlight_mode == LIGHTING_HEADLIGHT_DAYTIME) {
    duty = LIGHTING_DAYTIME_DUTY;
  }
  PwmOut_Write(&obj->front_light, duty);
}

static void Lighting_ApplyRearLight(Lighting* obj) {
  float duty = 0.0f;
  if (obj->brake_on) {
    if (obj->brake_flashing) {
      uint32_t elapsed_ms = Timer_ReadMs(&obj->brake_flash_timer);
      if (elapsed_ms >= LIGHTING_BRAKE_FLASH_PERIOD_MS) {
        Timer_Reset(&obj->brake_flash_timer);
        elapsed_ms = 0;
      }
      duty = elapsed_ms < LIGHTING_BRAKE_FLASH_ON_MS ? 1.0f : 0.0f;
      if (duty == 0.0f && obj->headlight_mode != LIGHTING_HEADLIGHT_OFF) {
        duty = LIGHTING_TAILLIGHT_DUTY;
      }
    } else {
      duty = 1.0f;
    }
  } else if (obj->headlight_mode != LIGHTING_HEADLIGHT_OFF) {
    duty = LIGHTING_TAILLIGHT_DUTY;
  }
  PwmOut_Write(&obj->rear_light, duty);
}

void Lighting_Init(Lighting* obj, TIM_HandleTypeDef* front_htim, uint32_t front_channel,
                   TIM_HandleTypeDef* rear_htim, uint32_t rear_channel,
                   TIM_HandleTypeDef* left_htim, uint32_t left_channel,
                   TIM_HandleTypeDef* right_htim, uint32_t right_channel) {
  PwmOut_Init(&obj->front_light, front_htim, front_channel);
  PwmOut_Init(&obj->rear_light, rear_htim, rear_channel);
  PwmOut_Init(&obj->left_winker, left_htim, left_channel);
  PwmOut_Init(&obj->right_winker, right_htim, right_channel);

  obj->headlight_mode = LIGHTING_HEADLIGHT_OFF;
  obj->passing_on = false;
  obj->brake_on = false;
  obj->brake_flashing = false;
  obj->winker_state = LIGHTING_WINKER_OFF;

  PwmOut_Write(&obj->front_light, 0.0f);
  PwmOut_Write(&obj->rear_light, 0.0f);
  PwmOut_Write(&obj->left_winker, 0.0f);
  PwmOut_Write(&obj->right_winker, 0.0f);

  Timer_Init(&obj->winker_timer);
  Timer_Init(&obj->brake_flash_timer);
}

void Lighting_SetHeadlight(Lighting* obj, LightingHeadlightMode mode) {
  obj->headlight_mode = mode;
  Lighting_ApplyFrontLight(obj);
  Lighting_ApplyRearLight(obj);
}

void Lighting_SetPassing(Lighting* obj, bool on) {
  obj->passing_on = on;
  Lighting_ApplyFrontLight(obj);
}

void Lighting_SetBrake(Lighting* obj, bool on) {
  Lighting_SetBrakeMode(obj, on, false);
}

void Lighting_SetBrakeMode(Lighting* obj, bool on, bool flashing) {
  flashing = on && flashing;
  if (flashing && !obj->brake_flashing) {
    Timer_Reset(&obj->brake_flash_timer);
  }
  obj->brake_on = on;
  obj->brake_flashing = flashing;
  Lighting_ApplyRearLight(obj);
}

void Lighting_SetBrakeFlashing(Lighting* obj, bool on) {
  Lighting_SetBrakeMode(obj, obj->brake_on || on, on);
}

void Lighting_SetWinker(Lighting* obj, LightingWinkerState state) {
  if (obj->winker_state == state) {
    return;
  }
  obj->winker_state = state;
  Timer_Reset(&obj->winker_timer);

  if (state == LIGHTING_WINKER_OFF) {
    PwmOut_Write(&obj->left_winker, 0.0f);
    PwmOut_Write(&obj->right_winker, 0.0f);
  }
}

void Lighting_Update(Lighting* obj) {
  if (obj->brake_on && obj->brake_flashing) {
    Lighting_ApplyRearLight(obj);
  }

  if (obj->winker_state == LIGHTING_WINKER_OFF) {
    return;
  }

  uint32_t elapsed_ms = Timer_ReadMs(&obj->winker_timer);
  if (elapsed_ms >= LIGHTING_WINKER_PERIOD_MS) {
    Timer_Reset(&obj->winker_timer);
    elapsed_ms = 0;
  }
  float duty = Lighting_WinkerDutyAt(elapsed_ms);

  float left_duty = 0.0f;
  float right_duty = 0.0f;
  if (obj->winker_state == LIGHTING_WINKER_LEFT || obj->winker_state == LIGHTING_WINKER_HAZARD) {
    left_duty = duty;
  }
  if (obj->winker_state == LIGHTING_WINKER_RIGHT || obj->winker_state == LIGHTING_WINKER_HAZARD) {
    right_duty = duty;
  }
  PwmOut_Write(&obj->left_winker, left_duty);
  PwmOut_Write(&obj->right_winker, right_duty);
}

LightingWinkerState Lighting_GetWinkerState(const Lighting* obj) { return obj->winker_state; }

uint32_t Lighting_GetWinkerPeriodMs(void) {
  return LIGHTING_WINKER_PERIOD_MS;
}
