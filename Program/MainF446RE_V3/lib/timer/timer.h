#ifndef TIMER_H_
#define TIMER_H_

#include "main.h"

typedef struct {
  uint32_t start_time;
} Timer;

static inline void Timer_Init(Timer* timer) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  timer->start_time = DWT->CYCCNT;
}

static inline void Timer_Reset(Timer* timer) {
  timer->start_time = DWT->CYCCNT;
}

/**
 * @brief Micros_Init() が実クロックから求めた 1us あたりのサイクル数を取得する。
 * HAL_RCC_GetSysClockFreq() は PLL 分周比からの再計算 (switch + 複数除算) を毎回行うため、
 * 500us ループで何十回も呼ぶと無視できないコストになる。こちらは定数を返すだけ。
 */
uint32_t Timer_GetCyclesPerUs(void);

static inline float Timer_Read(Timer* timer) {
  return (float)(DWT->CYCCNT - timer->start_time) / (float)(Timer_GetCyclesPerUs() * 1000000U);
}

static inline uint32_t Timer_ReadMs(Timer* timer) {
  return (DWT->CYCCNT - timer->start_time) / (Timer_GetCyclesPerUs() * 1000U);
}

static inline uint32_t Timer_ReadUs(Timer* timer) {
  return (DWT->CYCCNT - timer->start_time) / Timer_GetCyclesPerUs();
}

/**
 * @brief 開始時刻を us だけ過去へずらす (経過時間を即座に us だけ進める)。
 * 周期処理の初期位相をずらす用途 (例: 複数センサのトリガタイミングを分散させる) に使う。
 */
static inline void Timer_RewindUs(Timer* timer, uint32_t us) {
  timer->start_time -= us * Timer_GetCyclesPerUs();
}

static inline void WaitUs(uint32_t micros) {
  uint32_t startTick = DWT->CYCCNT;
  uint32_t requiredTicks = micros * (SystemCoreClock / 1000000);
  while ((DWT->CYCCNT - startTick) < requiredTicks) {
  }
}

// DWT->CYCCNT は32bitカウンタで、180MHzでは約23.9秒 (2^32/180e6) でラップする。
// requiredTicks = millis * (SystemCoreClock/1000) をそのまま uint32_t で計算すると、
// 引数がそれより大きいときに乗算自体がオーバーフローし、要求より大幅に短い時間で
// 返ってしまう (現状呼び出し箇所は無いが、将来の呼び出しで無音のバグにならないよう
// 安全な範囲ずつに分割して繰り返す)
#define TIMER_MAX_SAFE_WAIT_MS 1000u  // 180MHzでも十分な安全マージンを残せる長さ

static inline void WaitMs(uint32_t millis) {
  while (millis > TIMER_MAX_SAFE_WAIT_MS) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t requiredTicks = TIMER_MAX_SAFE_WAIT_MS * (SystemCoreClock / 1000);
    while ((DWT->CYCCNT - startTick) < requiredTicks) {
    }
    millis -= TIMER_MAX_SAFE_WAIT_MS;
  }
  uint32_t startTick = DWT->CYCCNT;
  uint32_t requiredTicks = millis * (SystemCoreClock / 1000);
  while ((DWT->CYCCNT - startTick) < requiredTicks) {
  }
}

static inline void Wait(uint32_t seconds) {
  // seconds * SystemCoreClock を直接計算せず、オーバーフローしない安全な単位
  // (WaitMs 1回分) の呼び出しへ分解する
  for (uint32_t i = 0; i < seconds; i++) {
    WaitMs(1000);
  }
}

/**
 * @brief 単調増加するマイクロ秒カウンタを初期化する。他のどの初期化よりも先に呼ぶこと。
 */
void Micros_Init(void);

/**
 * @brief 起動からの経過時間 [us] を返す。約71.6分でラップするため、時刻の差分は
 * 必ず符号なし演算 ((uint32_t)(a - b)) で取ること。割り込みからも呼んでよい。
 * ラップ周期より長く呼ばれない期間があると時刻が飛ぶため、制御周期ごとに呼ぶこと。
 */
uint32_t Micros(void);

#endif  // TIMER_H_