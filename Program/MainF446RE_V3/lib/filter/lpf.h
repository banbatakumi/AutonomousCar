#ifndef LPH_H_
#define LPH_H_

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
}

static inline float LPF_Update(LPF *lpf, float new_val) {
      // ローパスフィルタの更新
      lpf->current_val = lpf->k_lpf * lpf->prev_val + (1.0f - lpf->k_lpf) * new_val;
      lpf->prev_val = lpf->current_val;
      return lpf->current_val;
}

#endif  // LPH_H_