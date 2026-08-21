#ifndef RANGE_SENSOR_H_
#define RANGE_SENSOR_H_

#include <stdbool.h>

#include "lpf.h"
#include "main.h"
#include "ultrasonic.h"

// 前後の超音波測距 (lib/ultrasonic) をまとめ、LPFで平滑化した距離を提供する。
// 反射面の角度やマルチパスで単発の測距値が飛ぶことがあるため、生値をそのまま上位へ
// 送るとノイズが乗る。ただし ULTRASONIC_NO_ECHO (未検知) は物理量のノイズではなく
// 「対象が検知範囲にいない」という離散的な状態なので、フィルタを介さず即座に反映する
// (フィルタ越しだと対象が消えたことに気づくのが遅れ、逆に対象が現れた直後の1発目も
// 古い値と混ざって鈍る)。

typedef struct {
  Ultrasonic front;
  Ultrasonic rear;
  LPF front_lpf;
  LPF rear_lpf;
  bool front_lpf_seeded;
  bool rear_lpf_seeded;
  // 直近にLPFへ反映した Ultrasonic_GetSeq() の値。トリガ間隔60msの間 Ultrasonic の生値は
  // 変化しないため、値ではなくこのseqの変化で「新しい計測が来たか」を判別する
  uint32_t front_lpf_seq;
  uint32_t rear_lpf_seq;
} RangeSensor;

/**
 * @brief 前後の超音波センサを初期化する。
 */
void RangeSensor_Init(RangeSensor* obj, GPIO_TypeDef* front_trig_port, uint16_t front_trig_pin,
                       GPIO_TypeDef* front_echo_port, uint16_t front_echo_pin,
                       GPIO_TypeDef* rear_trig_port, uint16_t rear_trig_pin,
                       GPIO_TypeDef* rear_echo_port, uint16_t rear_echo_pin);

/**
 * @brief トリガ送出とフィルタの更新。ブロッキングしないので制御周期ごとに呼ぶこと。
 */
void RangeSensor_Update(RangeSensor* obj);

/**
 * @brief 前方 ECHO ピンの変化割り込みから呼ぶこと (HAL_GPIO_EXTI_Callback 内で振り分ける)。
 */
void RangeSensor_OnFrontEchoEdge(RangeSensor* obj);

/**
 * @brief 後方 ECHO ピンの変化割り込みから呼ぶこと。
 */
void RangeSensor_OnRearEchoEdge(RangeSensor* obj);

/**
 * @brief 前方の距離 [cm] の生値を取得する。未検知・計測不能時は ULTRASONIC_NO_ECHO。
 */
float RangeSensor_GetFrontDistanceCm(RangeSensor* obj);

/**
 * @brief 後方の距離 [cm] の生値を取得する。未検知・計測不能時は ULTRASONIC_NO_ECHO。
 */
float RangeSensor_GetRearDistanceCm(RangeSensor* obj);

/**
 * @brief LPF を通した前方の距離 [cm] を取得する。未検知時は ULTRASONIC_NO_ECHO
 * (フィルタを介さない、上の説明を参照)。
 */
float RangeSensor_GetFrontDistanceFilteredCm(RangeSensor* obj);

/**
 * @brief LPF を通した後方の距離 [cm] を取得する。未検知時は ULTRASONIC_NO_ECHO。
 */
float RangeSensor_GetRearDistanceFilteredCm(RangeSensor* obj);

#endif  // RANGE_SENSOR_H_
