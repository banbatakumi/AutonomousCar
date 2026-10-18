#ifndef ENCODER_CAL_H_
#define ENCODER_CAL_H_

#include <math.h>

// アナログ出力の絶対角エンコーダ (AS5600 等) の読みを、校正値で真の角度へ直す。HAL に依存しない
// (ホストのテストも同じ式を使う)。
//
// 1. 出力電圧の範囲: ADC のフルスケールに対する比 u (0〜1) のうち、角度 0 と 2π に当たるのが lo と hi。
//    θm = (u − lo) / (hi − lo) × 2π。出力の範囲が電源電圧の全域でない (AS5600 の OUTS 設定・電源と
//    ADC の基準電圧の差) と、1回転の角度が 2π より小さく読めて走行距離の倍率と 0/2π の境目の段差になる
// 2. 1回転の周期誤差 (磁石の偏心・INL): 読みは θm = θ + e(θ)、
//    e(θ) = h1s·sin θ + h1c·cos θ + h2s·sin 2θ + h2c·cos 2θ [rad] (θ は真の角度)。
//    θ = θm − e(θ) を不動点反復で解く (2回で誤差は |e|·|e'|² 程度。振幅 2.3° で 0.002° 未満)
//
// 既定 (lo=0, hi=1, 係数 0) では何もしない。値は上位のシステム同定 (エンコーダ校正試験) が決める。
typedef struct {
  float lo;
  float hi;
  float h1s, h1c, h2s, h2c;
} EncoderCal;

#define ENCODER_CAL_TWO_PI 6.28318530717958647692f

static inline void EncoderCal_SetIdentity(EncoderCal* cal) {
  cal->lo = 0.0f;
  cal->hi = 1.0f;
  cal->h1s = cal->h1c = cal->h2s = cal->h2c = 0.0f;
}

static inline float EncoderCal_Wrap(float a) {
  a = fmodf(a, ENCODER_CAL_TWO_PI);
  if (a < 0.0f) a += ENCODER_CAL_TWO_PI;
  return a;
}

// 1回転に当たる ADC の比の幅 (hi − lo)。平均の折り返しの判定にも使う
static inline float EncoderCal_Span(const EncoderCal* cal) {
  float span = cal->hi - cal->lo;
  return span > 0.5f ? span : 1.0f;  // 壊れた値 (範囲が逆・狭すぎ) は無視して恒等にする
}

static inline float EncoderCal_Ripple(const EncoderCal* cal, float theta) {
  float s1 = sinf(theta), c1 = cosf(theta);
  float s2 = 2.0f * s1 * c1, c2 = c1 * c1 - s1 * s1;
  return cal->h1s * s1 + cal->h1c * c1 + cal->h2s * s2 + cal->h2c * c2;
}

// ADC のフルスケールに対する比 u → 真の角度 [rad] (0〜2π)
static inline float EncoderCal_Angle(const EncoderCal* cal, float u) {
  float theta_m = EncoderCal_Wrap((u - cal->lo) / EncoderCal_Span(cal) * ENCODER_CAL_TWO_PI);
  float theta = theta_m;
  for (int i = 0; i < 2; i++) theta = theta_m - EncoderCal_Ripple(cal, theta);
  return EncoderCal_Wrap(theta);
}

#endif  // ENCODER_CAL_H_
