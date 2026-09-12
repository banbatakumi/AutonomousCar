#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include "digitalinout.h"
#include "timer.h"

// トリガパルスの送出間隔 [us]
// (HC-SR04 系はセンサ同士の残響干渉を避けるため 60ms 以上の間隔が推奨される)
#define ULTRASONIC_TRIGGER_INTERVAL_US 60000

// 計測不能時（無反射・センサ未接続・タイムアウトなど）に返る距離値
#define ULTRASONIC_NO_ECHO -1.0f

// HC-SR04系は測定範囲外だとECHOが約38ms出続け、これをそのまま距離換算すると
// 651cmという「有効値」になって上位へ渡ってしまう。実用上あり得ない距離は
// 無反射として扱うための上限 [cm] (一般的なHC-SR04系の仕様上限は400cm程度)
#define ULTRASONIC_MAX_DISTANCE_CM 400.0f

typedef struct {
  DigitalOut trig;
  DigitalIn echo;
  Timer trigger_timer;                  // トリガ送出間隔の管理用 (Ultrasonic_Update() が使う)
  volatile uint32_t rise_cycle_count;   // ECHO立ち上がり時のDWTサイクルカウント (ISRが更新)
  volatile uint8_t waiting_fall;        // 立ち上がり検知済み・立ち下がり待ち中かどうか (ISRが更新)
  volatile float distance_cm;           // Ultrasonic_OnEchoEdge() がISRから更新する最新の計測結果
  // 新しい計測が確定するたびに ISR が++する。呼び出し側 (RangeSensor) はこれの変化を見て
  // 「本当に新しい値か」を判別する (トリガ間隔60msの間、Ultrasonic_GetDistanceCm()は
  // ずっと同じ値を返すため、値そのものの比較では区別できない)
  volatile uint32_t seq;
} Ultrasonic;

static inline void Ultrasonic_Init(Ultrasonic *obj, GPIO_TypeDef *trig_port, uint16_t trig_pin,
                                    GPIO_TypeDef *echo_port, uint16_t echo_pin) {
  DigitalOut_Init(&obj->trig, trig_port, trig_pin);
  DigitalIn_Init(&obj->echo, echo_port, echo_pin);
  Timer_Init(&obj->trigger_timer);
  obj->waiting_fall = 0;
  obj->distance_cm = ULTRASONIC_NO_ECHO;
  obj->seq = 0;
}

// 一定間隔でトリガパルスを送出する。ブロッキングしないので制御ループの毎ティック呼ぶこと。
// ECHOの立ち上がり/立ち下がり検知はメインループの周期に依存せず、ピン変化割り込み経由で
// Ultrasonic_OnEchoEdge() を呼ぶことで行う (HAL_GPIO_EXTI_Callback() から呼び出すこと)。
static inline void Ultrasonic_Update(Ultrasonic *obj) {
  if (Timer_ReadUs(&obj->trigger_timer) >= ULTRASONIC_TRIGGER_INTERVAL_US) {
    Timer_Reset(&obj->trigger_timer);

    // 前回計測の立ち下がりを検知できないままタイムアウトした場合の後始末。
    // waiting_fall の読み取りとその後の書き込みの間に本物の立ち下がりがISR
    // (Ultrasonic_OnEchoEdge) で処理されると、ISRが確定させた新しい距離を
    // ここでの書き込みが上書きしてしまう (waiting_fall/distance_cm の2フィールド
    // 更新がアトミックでないため)。対象ECHOピンのEXTIラインだけ一時マスクして
    // この区間をISRに対して排他にする (src/sensing/imu.c のDATA_RDYマスクと同じ手法。
    // マスク中に来たエッジはEXTIのペンディングビットに残るため、解除した瞬間に
    // 取りこぼさず処理される)
    uint32_t exti_line = obj->echo.pin;
    EXTI->IMR &= ~exti_line;
    if (obj->waiting_fall) {
      obj->waiting_fall = 0;
      obj->distance_cm = ULTRASONIC_NO_ECHO;
    }
    EXTI->IMR |= exti_line;

    DigitalOut_Write(&obj->trig, 1);
    WaitUs(10);
    DigitalOut_Write(&obj->trig, 0);
  }
}

// ECHOピンの変化割り込みから呼ぶこと (HAL_GPIO_EXTI_Callback 内で GPIO_Pin を見て振り分ける)。
// 立ち上がり/立ち下がりを判定し、パルス幅(DWTサイクルカウント差)から距離を計算する。
static inline void Ultrasonic_OnEchoEdge(Ultrasonic *obj) {
  if (DigitalIn_Read(&obj->echo)) {
    obj->rise_cycle_count = DWT->CYCCNT;
    obj->waiting_fall = 1;
  } else if (obj->waiting_fall) {
    uint32_t elapsed_cycles = DWT->CYCCNT - obj->rise_cycle_count;
    float elapsed_us = (float)elapsed_cycles * 1000000.0f / (float)SystemCoreClock;
    // 距離[cm] = 往復時間[us] * 音速(0.0343 cm/us) / 2
    float distance_cm = elapsed_us * 0.0343f / 2.0f;
    // 測定範囲外 (無反射) をそのまま「651cmの実測値」として報告しないよう、
    // 実用上あり得ない距離は無反射として扱う
    obj->distance_cm = distance_cm <= ULTRASONIC_MAX_DISTANCE_CM ? distance_cm : ULTRASONIC_NO_ECHO;
    obj->seq++;
    obj->waiting_fall = 0;
  }
}

// 直近の計測結果 [cm] を取得する（非ブロッキング）。未計測・計測不能時は ULTRASONIC_NO_ECHO。
static inline float Ultrasonic_GetDistanceCm(Ultrasonic *obj) {
  return obj->distance_cm;
}

// 直近の計測が確定した回数を取得する。値そのものが変化しなくても新しい計測が来たことを
// 判別するために使う (RangeSensor のLPFがトリガ間隔60msの間ずっと同じ値を通してしまう問題対策)。
static inline uint32_t Ultrasonic_GetSeq(Ultrasonic *obj) {
  return obj->seq;
}

#endif  // ULTRASONIC_H_
