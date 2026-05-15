#ifndef MAF_H_
#define MAF_H_

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

// 移動平均フィルタ (Moving Average Filter)

// 最大窓幅（必要に応じて変更）
#ifndef MAF_MAX_WINDOW
#define MAF_MAX_WINDOW 100
#endif

typedef struct {
  float buffer[MAF_MAX_WINDOW];  // 固定長リングバッファ（malloc不要）
  uint16_t window_size;          // 窓幅（平均を取るサンプル数、<= MAF_MAX_WINDOW）
  uint16_t index;                // 現在書き込み位置
  uint16_t count;                // これまでに蓄積したサンプル数（window_size まで増加）
  float sum;                     // バッファ内サンプルの合計値
} MAF;

// 移動平均フィルタの初期化
// window_size で指定したサンプル数分のバッファを動的確保し、0 クリアする
static inline void MAF_Init(MAF* maf, uint16_t window_size) {
  if (!maf) return;
  if (window_size == 0) {
    maf->window_size = 0;
    maf->index = 0;
    maf->count = 0;
    maf->sum = 0.0f;
    return;
  }
  if (window_size > MAF_MAX_WINDOW) {
    window_size = MAF_MAX_WINDOW;
  }
  maf->window_size = window_size;
  maf->index = 0;
  maf->count = 0;
  maf->sum = 0.0f;
  // clear buffer
  memset(maf->buffer, 0, sizeof(float) * maf->window_size);
}

// 新しいサンプルを追加し、移動平均値を返す
static inline float MAF_Update(MAF* maf, float new_val) {
  // バッファ未確保や窓幅 0 の場合は入力値をそのまま返す
  if (maf->window_size == 0) {
    return new_val;
  }

  // サンプル数が窓幅に達するまでは単純にカウントアップ
  if (maf->count < maf->window_size) {
    maf->count++;
  } else {
    // 古いサンプルを合計値から減算
    maf->sum -= maf->buffer[maf->index];
  }

  // 新しいサンプルを追加して合計値を更新
  maf->buffer[maf->index] = new_val;
  maf->sum += new_val;

  // 書き込みインデックスをリングバッファとして更新
  maf->index++;
  if (maf->index >= maf->window_size) {
    maf->index = 0;
  }

  // 現在の平均値を返す
  return maf->sum / (float)maf->count;
}

#endif  // MAF_H_
