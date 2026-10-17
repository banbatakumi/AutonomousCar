#ifndef LPH_H_
#define LPH_H_

#include <math.h>

// Cortex-M4F の FPU は単精度のみのため float で持つ (double だとソフトウェア
// エミュレーションになり、毎周期何度も呼ばれるこのフィルタでは無視できないコストになる)
typedef struct {
      float current_val;
      float prev_val;
      float k_lpf;  // ローパスフィルタ係数
} LPF;

static inline void LPF_Init(LPF *lpf, float k_lpf, float initial_val) {
      lpf->k_lpf = k_lpf;
      lpf->prev_val = initial_val;
      // LPF_Update() を呼ぶ前に current_val を読むゲッター (Power_GetVoltage*Filtered() 等) が
      // 未初期化値を返さないよう、prev_val と揃えておく
      lpf->current_val = initial_val;
}

static inline float LPF_Update(LPF *lpf, float new_val) {
      // ローパスフィルタの更新
      lpf->current_val = lpf->k_lpf * lpf->prev_val + (1.0f - lpf->k_lpf) * new_val;
      lpf->prev_val = lpf->current_val;
      return lpf->current_val;
}

// 時定数 tau_s の1次遅れを、前回からの経過時間 dt_s で1歩進める (係数 k = exp(-dt/τ) を毎回求める。
// 連続時間の1次遅れの厳密な離散化)。呼び出し周期が変わっても時定数が変わらない。dt_s <= 0 なら値を保つ
static inline float LPF_UpdateTau(LPF *lpf, float new_val, float tau_s, float dt_s) {
      if (dt_s <= 0.0f) return lpf->current_val;
      lpf->k_lpf = (tau_s > 0.0f) ? expf(-dt_s / tau_s) : 0.0f;
      return LPF_Update(lpf, new_val);
}

#endif  // LPH_H_