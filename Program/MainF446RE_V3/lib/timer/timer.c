#include "timer.h"

// ---------------------------------------------------------------------------
// 単調増加する 32bit マイクロ秒カウンタ
//
// Timer 構造体は「ある時点からの経過時間」しか測れないうえ、DWT のサイクルカウンタは
// 180MHz では約23.9秒でラップするため、上位との通信プロトコルが要求する
// 「起動からの単調な us 時刻」には使えない。
//
// ここでは呼ばれるたびに前回からの経過サイクルを us へ換算して積算する。
// ラップ周期 (23.9秒) より短い間隔で呼ばれている限り、CYCCNT が何周しても正しい。
// 実際には制御周期ごとに呼ばれるため条件は常に満たされる。
// ---------------------------------------------------------------------------

static volatile uint32_t g_micros;      // 積算した経過時間 [us]
static volatile uint32_t g_last_cycle;  // 最後に換算した時点の CYCCNT
static uint32_t g_cycles_per_us = 180;  // Micros_Init() で実クロックから求める

void Micros_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  g_cycles_per_us = SystemCoreClock / 1000000U;
  if (g_cycles_per_us == 0U) g_cycles_per_us = 1U;

  g_last_cycle = DWT->CYCCNT;
  g_micros = 0;
}

uint32_t Micros(void) {
  // 割り込みからも呼ばれるため、読み出しと基準の更新は不可分に行う
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint32_t now = DWT->CYCCNT;
  uint32_t elapsed_us = (now - g_last_cycle) / g_cycles_per_us;
  // 1us に満たない端数のサイクルは繰り越す。切り捨てたまま基準を now に進めると
  // 呼び出しのたびに端数を捨てることになり、時計が系統的に遅れていく
  g_last_cycle += elapsed_us * g_cycles_per_us;
  g_micros += elapsed_us;

  uint32_t result = g_micros;
  if (primask == 0U) __enable_irq();
  return result;
}
