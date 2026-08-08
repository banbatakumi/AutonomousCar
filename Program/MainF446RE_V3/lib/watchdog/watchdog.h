#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "main.h"

// 独立ウォッチドッグ (IWDG)。LSI で動くため、システムクロックが止まっても、HAL や
// アプリケーションがハングしても効く。**マイコン自身の暴走を検出できる唯一の手段**で、
// 上位からの指令途絶の検出も自動ブレーキも「マイコンが動いていること」が前提なので、
// この層が無いと最後の砦が存在しないことになる。
//
// HAL のドライバではなくレジスタを直接叩いている。IWDG は .ioc で有効化されておらず、
// HAL モジュールを足すと CubeMX の再生成と衝突するため (手順は RM0390 の 21.3 のとおり)。
//
// **一度起動すると停止できない** (リセットするまで動き続ける)。ブロッキングする初期化が
// すべて終わってから起動すること。

// レジスタのキー
#define WATCHDOG_KEY_ENABLE 0x0000CCCCu
#define WATCHDOG_KEY_WRITE_ACCESS 0x00005555u
#define WATCHDOG_KEY_RELOAD 0x0000AAAAu

// プリスケーラ 64分周 (PR=4)。LSI 32kHz なら 1カウント 2ms
#define WATCHDOG_PRESCALER 4u
#define WATCHDOG_TICK_US 2000u
#define WATCHDOG_RELOAD_MAX 0xFFFu  // RLR は12bit

/**
 * @brief 独立ウォッチドッグを起動する。以降 timeout_ms 以内に Watchdog_Refresh() を
 * 呼び続けないとマイコンがリセットされる。
 *
 * LSI の周波数は 32kHz が標準値だが、STM32F446 のデータシートでは 17〜47kHz と幅がある。
 * 実際のタイムアウトは指定値の約 0.68〜1.9 倍に振れるため、リフレッシュ周期より桁で
 * 大きい値を指定すること (500ms 指定なら最悪 340ms まで縮む)。
 */
static inline void Watchdog_Start(uint32_t timeout_ms) {
  uint32_t reload = timeout_ms * 1000u / WATCHDOG_TICK_US;
  if (reload > WATCHDOG_RELOAD_MAX) reload = WATCHDOG_RELOAD_MAX;
  if (reload == 0u) reload = 1u;

  // デバッガでコアを止めている間はカウントも止める。これが無いとブレークポイントで
  // 止めるたびにリセットがかかり、デバッグそのものができなくなる
  __HAL_DBGMCU_FREEZE_IWDG();

  IWDG->KR = WATCHDOG_KEY_ENABLE;
  IWDG->KR = WATCHDOG_KEY_WRITE_ACCESS;
  IWDG->PR = WATCHDOG_PRESCALER;
  IWDG->RLR = reload;
  // PR/RLR は LSI 側のクロックで反映されるため、完了するまで次の操作をしない
  while (IWDG->SR != 0u) {
  }
  IWDG->KR = WATCHDOG_KEY_RELOAD;
}

/**
 * @brief ウォッチドッグのカウンタを再読み込みする。メインループごとに呼ぶこと。
 */
static inline void Watchdog_Refresh(void) {
  IWDG->KR = WATCHDOG_KEY_RELOAD;
}

#endif  // WATCHDOG_H_
