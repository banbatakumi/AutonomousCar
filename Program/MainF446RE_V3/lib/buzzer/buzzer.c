#include "buzzer.h"

#include <stdint.h>

#include "main.h"

// 起動メロディのノート定義（周波数 [Hz], 発音時間 [ms], 次音までの無音時間 [ms]）
typedef struct {
  uint32_t freq_hz;
  uint32_t on_ms;
  uint32_t gap_ms;
} MelodyNote;

// ブザー配線はラズパイと共有しているため、STM側が鳴らさない間はピンを
// フローティング入力にしてバスを明け渡す。TIM の AF プッシュプル出力のままだと
// duty 0 でも Low を出し続け、ラズパイが High を出そうとすると貫通してしまうため。
static void ReleaseGpio(Buzzer* obj) {
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = obj->gpio_pin;
  gpio_init.Mode = GPIO_MODE_INPUT;
  gpio_init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(obj->gpio_port, &gpio_init);
}

// STM 側で音を出す直前にピンをタイマの AF プッシュプル出力へ戻す。
static void DriveGpio(Buzzer* obj) {
  GPIO_InitTypeDef gpio_init = {0};
  gpio_init.Pin = obj->gpio_pin;
  gpio_init.Mode = GPIO_MODE_AF_PP;
  gpio_init.Pull = GPIO_NOPULL;
  gpio_init.Speed = GPIO_SPEED_FREQ_LOW;
  gpio_init.Alternate = obj->gpio_af;
  HAL_GPIO_Init(obj->gpio_port, &gpio_init);
}

// ARR と比較値（50% duty）を計算して音程を設定する。
// freq_hz == 0 の場合は無音（compare を 0 に設定）とし、ピンをラズパイへ明け渡す。
static void SetTone(Buzzer* obj, uint32_t freq_hz) {
  uint32_t arr;
  if (freq_hz == 0) {
    __HAL_TIM_SET_COMPARE(obj->htim, obj->channel, 0);
    ReleaseGpio(obj);
    return;
  }
  DriveGpio(obj);
  arr = (obj->timer_clock_hz / ((obj->prescaler + 1) * freq_hz)) - 1;
  __HAL_TIM_SET_AUTORELOAD(obj->htim, arr);
  __HAL_TIM_SET_COMPARE(obj->htim, obj->channel, arr / 2);
  // ARR はプリロード無効で即時反映されるため、CNT が新 ARR を超えたままだと
  // 次の一致まで最大値までカウントし続けてしまう。CNT を 0 に戻して回避する。
  __HAL_TIM_SET_COUNTER(obj->htim, 0);
}

static void BuzzerOn(Buzzer* obj) {
  SetTone(obj, obj->freq_hz);
  obj->buzzer_on = true;
  Timer_Reset(&obj->timer);
}

static void BuzzerOff(Buzzer* obj) {
  // REPEAT パターンの off 区間もここを通るため、ビープの合間もラズパイへ明け渡される
  SetTone(obj, 0);
  obj->buzzer_on = false;
  Timer_Reset(&obj->timer);
}

void Buzzer_Init(Buzzer* obj, TIM_HandleTypeDef* htim, uint32_t channel,
                 uint32_t timer_clock_hz, uint32_t prescaler,
                 GPIO_TypeDef* gpio_port, uint16_t gpio_pin, uint32_t gpio_af) {
  obj->htim = htim;
  obj->channel = channel;
  obj->timer_clock_hz = timer_clock_hz;
  obj->prescaler = prescaler;
  obj->gpio_port = gpio_port;
  obj->gpio_pin = gpio_pin;
  obj->gpio_af = gpio_af;
  obj->pattern = BUZZER_PATTERN_NONE;
  obj->freq_hz = 1000;
  obj->on_ms = 0;
  obj->off_ms = 0;
  obj->repeat_count = 0;
  obj->buzzer_on = false;

  HAL_TIM_PWM_Start(obj->htim, obj->channel);
  __HAL_TIM_SET_COMPARE(obj->htim, obj->channel, 0);
  ReleaseGpio(obj);
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
    SetTone(obj, 0);
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
