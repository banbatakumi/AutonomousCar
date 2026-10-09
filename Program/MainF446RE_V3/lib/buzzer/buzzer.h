#ifndef BUZZER_H_
#define BUZZER_H_

#include <stdbool.h>
#include <stdint.h>

#include "tim.h"
#include "timer.h"

// ビープパターンの種別
typedef enum {
  BUZZER_PATTERN_NONE = 0,  // 停止中
  BUZZER_PATTERN_SINGLE,    // 単発ビープ（on_ms 後に停止）
  BUZZER_PATTERN_REPEAT,    // on/off を繰り返す（count=-1 で無限）
  BUZZER_PATTERN_SWEEP,     // 周波数を連続的に変化させる（完了後に停止）
} BuzzerPattern;

typedef struct {
  TIM_HandleTypeDef *htim;
  uint32_t channel;
  uint32_t timer_clock_hz;  // タイマペリフェラルのクロック周波数 [Hz]
  uint32_t prescaler;       // CubeMX で設定したプリスケーラ値（分周比 = prescaler+1）

  // ブザー配線をラズパイと共有しているため、鳴らしていない間はピンを
  // フローティング入力へ切り替えてラズパイ側から駆動できるようにする
  GPIO_TypeDef *gpio_port;
  uint16_t gpio_pin;
  uint32_t gpio_af;

  BuzzerPattern pattern;
  uint32_t freq_hz;          // 音程 [Hz]
  uint32_t on_ms;            // ON 持続時間 [ms]
  uint32_t off_ms;           // OFF 持続時間 [ms]（REPEAT のみ使用）
  int32_t repeat_count;      // 残り繰り返し回数（-1 で無限）
  uint32_t sweep_start_hz;   // SWEEP の開始周波数 [Hz]
  uint32_t sweep_end_hz;     // SWEEP の終了周波数 [Hz]
  uint32_t sweep_ms;         // SWEEP 全体の長さ [ms]
  uint32_t sweep_last_hz;    // 直近に設定した周波数 [Hz]（変化したときだけ書くため）
  bool buzzer_on;            // 現在 ON 状態か
  Timer timer;
} Buzzer;

/**
 * @brief ブザーを初期化する。
 * @param obj             Buzzer インスタンス
 * @param htim            タイマハンドル（CubeMX で PWM 設定済みのもの）
 * @param channel         TIM_CHANNEL_x
 * @param timer_clock_hz  タイマペリフェラルのクロック周波数（APB タイマクロック） [Hz]
 * @param prescaler       CubeMX で設定したプリスケーラ値（tim.c の Init.Prescaler と同値）
 * @param gpio_port       ブザーピンのポート（例: BUZZER_GPIO_Port）
 * @param gpio_pin        ブザーピン番号（例: BUZZER_Pin）
 * @param gpio_af         ブザーピンに割り当てるタイマのオルタネート機能（例: GPIO_AF1_TIM2）
 */
void Buzzer_Init(Buzzer *obj, TIM_HandleTypeDef *htim, uint32_t channel,
                 uint32_t timer_clock_hz, uint32_t prescaler,
                 GPIO_TypeDef *gpio_port, uint16_t gpio_pin, uint32_t gpio_af);

/**
 * @brief 指定周波数で duration_ms [ms] 間鳴らし、その後自動停止する。
 */
void Buzzer_Beep(Buzzer *obj, uint32_t freq_hz, uint32_t duration_ms);

/**
 * @brief on_ms 鳴らして off_ms 止める、を count 回繰り返す。count=-1 で無限。
 */
void Buzzer_BeepPattern(Buzzer *obj, uint32_t freq_hz, uint32_t on_ms,
                        uint32_t off_ms, int32_t count);

/**
 * @brief start_hz から end_hz まで duration_ms [ms] かけて周波数を連続的に変え、終了後に停止する。
 *        ノンブロッキング。Buzzer_Update で進行する。補間は対数（音程が等間隔に聞こえる）。
 *        end_hz < start_hz で下降音になる。
 */
void Buzzer_Sweep(Buzzer *obj, uint32_t start_hz, uint32_t end_hz, uint32_t duration_ms);

/**
 * @brief ブザーを即時停止する。
 */
void Buzzer_Stop(Buzzer *obj);

/**
 * @brief パターン再生を管理する。メインループで毎ティック呼ぶこと。
 */
void Buzzer_Update(Buzzer *obj);

/**
 * @brief 指定周波数で連続トーンを設定する。freq_hz=0 で無音。
 *        パターン再生を中断して直接ハードウェアに適用する。
 */
void Buzzer_SetTone(Buzzer *obj, uint32_t freq_hz);

/**
 * @brief 起動確認メロディを同期再生する（HAL_Delay を使用するブロッキング関数）。
 *        Setup() 内で一度だけ呼ぶこと。
 */
void Buzzer_PlayStartupMelody(Buzzer *obj);

#endif  // BUZZER_H_
