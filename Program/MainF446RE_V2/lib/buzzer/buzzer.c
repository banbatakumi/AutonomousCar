#include "buzzer.h"

#include <stdint.h>

#include "main.h"

// 起動メロディのノート定義（周波数 [Hz], 発音時間 [ms], 次音までの無音時間 [ms]）
typedef struct {
  uint32_t freq_hz;
  uint32_t on_ms;
  uint32_t gap_ms;
} MelodyNote;

// ARR と比較値（50% duty）を計算して音程を設定する。
// freq_hz == 0 の場合は無音（compare を 0 に設定）とする。
static void SetTone(Buzzer* obj, uint32_t freq_hz) {
  uint32_t arr;
  if (freq_hz == 0) {
    __HAL_TIM_SET_COMPARE(obj->htim, obj->channel, 0);
    return;
  }
  arr = (obj->timer_clock_hz / ((obj->prescaler + 1) * freq_hz)) - 1;
  __HAL_TIM_SET_AUTORELOAD(obj->htim, arr);
  __HAL_TIM_SET_COMPARE(obj->htim, obj->channel, arr / 2);
}

static void BuzzerOn(Buzzer* obj) {
  SetTone(obj, obj->freq_hz);
  obj->buzzer_on = true;
  Timer_Reset(&obj->timer);
}

static void BuzzerOff(Buzzer* obj) {
  __HAL_TIM_SET_COMPARE(obj->htim, obj->channel, 0);
  obj->buzzer_on = false;
  Timer_Reset(&obj->timer);
}

void Buzzer_Init(Buzzer* obj, TIM_HandleTypeDef* htim, uint32_t channel,
                 uint32_t timer_clock_hz, uint32_t prescaler) {
  obj->htim = htim;
  obj->channel = channel;
  obj->timer_clock_hz = timer_clock_hz;
  obj->prescaler = prescaler;
  obj->pattern = BUZZER_PATTERN_NONE;
  obj->freq_hz = 1000;
  obj->on_ms = 0;
  obj->off_ms = 0;
  obj->repeat_count = 0;
  obj->buzzer_on = false;

  HAL_TIM_PWM_Start(obj->htim, obj->channel);
  __HAL_TIM_SET_COMPARE(obj->htim, obj->channel, 0);
  Timer_Init(&obj->timer);
}

void Buzzer_Beep(Buzzer* obj, uint32_t freq_hz, uint32_t duration_ms) {
  obj->pattern = BUZZER_PATTERN_SINGLE;
  obj->freq_hz = freq_hz;
  obj->on_ms = duration_ms;
  BuzzerOn(obj);
}

void Buzzer_BeepPattern(Buzzer* obj, uint32_t freq_hz, uint32_t on_ms,
                        uint32_t off_ms, int32_t count) {
  obj->pattern = BUZZER_PATTERN_REPEAT;
  obj->freq_hz = freq_hz;
  obj->on_ms = on_ms;
  obj->off_ms = off_ms;
  obj->repeat_count = count;
  BuzzerOn(obj);
}

void Buzzer_Stop(Buzzer* obj) {
  obj->pattern = BUZZER_PATTERN_NONE;
  BuzzerOff(obj);
}

void Buzzer_SetTone(Buzzer* obj, uint32_t freq_hz) {
  obj->pattern = BUZZER_PATTERN_NONE;
  SetTone(obj, freq_hz);
}

void Buzzer_PlayStartupMelody(Buzzer* obj) {
  // ド(C5)→ミ(E5)→ソ(G5) の上昇アルペジオで起動を通知
  static const MelodyNote kMelody[] = {
      {523, 100, 40},  // C5
      {659, 100, 40},  // E5
      {784, 300, 0},   // G5
  };
  uint32_t i;
  for (i = 0; i < sizeof(kMelody) / sizeof(kMelody[0]); i++) {
    SetTone(obj, kMelody[i].freq_hz);
    HAL_Delay(kMelody[i].on_ms);
    __HAL_TIM_SET_COMPARE(obj->htim, obj->channel, 0);
    if (kMelody[i].gap_ms > 0) {
      HAL_Delay(kMelody[i].gap_ms);
    }
  }
}

void Buzzer_Update(Buzzer* obj) {
  uint32_t elapsed = Timer_ReadMs(&obj->timer);

  switch (obj->pattern) {
    case BUZZER_PATTERN_NONE:
      break;

    case BUZZER_PATTERN_SINGLE:
      if (obj->buzzer_on && elapsed >= obj->on_ms) {
        obj->pattern = BUZZER_PATTERN_NONE;
        BuzzerOff(obj);
      }
      break;

    case BUZZER_PATTERN_REPEAT:
      if (obj->buzzer_on && elapsed >= obj->on_ms) {
        BuzzerOff(obj);
        // repeat_count == 0 は残り 0 回 → 停止
        if (obj->repeat_count > 0) {
          obj->repeat_count--;
        }
        if (obj->repeat_count == 0) {
          obj->pattern = BUZZER_PATTERN_NONE;
        }
      } else if (!obj->buzzer_on && elapsed >= obj->off_ms &&
                 obj->pattern == BUZZER_PATTERN_REPEAT) {
        BuzzerOn(obj);
      }
      break;
  }
}
