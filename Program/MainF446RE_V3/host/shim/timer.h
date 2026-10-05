#ifndef TIMER_H_
#define TIMER_H_

// ホスト用の差し替え (host/README.md)。時刻はシミュレーションが進める g_host_now_s
#include <stdint.h>

extern double g_host_now_s;

typedef struct {
  double start_s;
} Timer;

static inline void Timer_Init(Timer* timer) { timer->start_s = g_host_now_s; }
static inline void Timer_Reset(Timer* timer) { timer->start_s = g_host_now_s; }
static inline float Timer_Read(Timer* timer) { return (float)(g_host_now_s - timer->start_s); }

#endif  // TIMER_H_
