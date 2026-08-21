#include "range_sensor.h"

// トリガ間隔60ms (ULTRASONIC_TRIGGER_INTERVAL_US) に対して、単発の反射角ノイズを
// 抑えつつ実際の接近には遅れず追従できる程度の係数
#define RANGE_SENSOR_LPF_K 0.6f

// メインループ (2kHz) から毎周期呼ばれるが、Ultrasonic_GetDistanceCm() の値自体は
// トリガ間隔 (60ms) の間ずっと同じなので、値の比較では「新しい計測か」を判別できない
// (0.6^120 ≒ 1e-27 となり、フィルタが実質無効化されていた)。Ultrasonic_GetSeq() の
// 変化で新しい計測が確定した周期だけを見つけてフィルタを回す
static void UpdateOne(Ultrasonic* sensor, LPF* lpf, bool* seeded, uint32_t* last_seq) {
  Ultrasonic_Update(sensor);
  float raw = Ultrasonic_GetDistanceCm(sensor);
  if (raw == ULTRASONIC_NO_ECHO) {
    // 次に検知が戻ったとき、消える前の古い値へ引きずられて収束が遅れないよう
    // フィルタを作り直す
    *seeded = false;
    return;
  }
  uint32_t seq = Ultrasonic_GetSeq(sensor);
  if (*seeded && seq == *last_seq) return;  // まだ新しい計測が来ていない
  *last_seq = seq;
  if (!*seeded) {
    LPF_Init(lpf, RANGE_SENSOR_LPF_K, raw);
    *seeded = true;
  }
  LPF_Update(lpf, raw);
}

void RangeSensor_Init(RangeSensor* obj, GPIO_TypeDef* front_trig_port, uint16_t front_trig_pin,
                       GPIO_TypeDef* front_echo_port, uint16_t front_echo_pin,
                       GPIO_TypeDef* rear_trig_port, uint16_t rear_trig_pin,
                       GPIO_TypeDef* rear_echo_port, uint16_t rear_echo_pin) {
  Ultrasonic_Init(&obj->front, front_trig_port, front_trig_pin, front_echo_port, front_echo_pin);
  Ultrasonic_Init(&obj->rear, rear_trig_port, rear_trig_pin, rear_echo_port, rear_echo_pin);
  // 前後を同時初期化すると常に同時刻にトリガされ、壁際で反射波が回り込むことがあるため、
  // 後方の位相を半周期 (30ms) ずらす
  Timer_RewindUs(&obj->rear.trigger_timer, ULTRASONIC_TRIGGER_INTERVAL_US / 2);
  obj->front_lpf_seeded = false;
  obj->rear_lpf_seeded = false;
  obj->front_lpf_seq = 0;
  obj->rear_lpf_seq = 0;
}

void RangeSensor_Update(RangeSensor* obj) {
  UpdateOne(&obj->front, &obj->front_lpf, &obj->front_lpf_seeded, &obj->front_lpf_seq);
  UpdateOne(&obj->rear, &obj->rear_lpf, &obj->rear_lpf_seeded, &obj->rear_lpf_seq);
}

void RangeSensor_OnFrontEchoEdge(RangeSensor* obj) {
  Ultrasonic_OnEchoEdge(&obj->front);
}

void RangeSensor_OnRearEchoEdge(RangeSensor* obj) {
  Ultrasonic_OnEchoEdge(&obj->rear);
}

float RangeSensor_GetFrontDistanceCm(RangeSensor* obj) {
  return Ultrasonic_GetDistanceCm(&obj->front);
}

float RangeSensor_GetRearDistanceCm(RangeSensor* obj) {
  return Ultrasonic_GetDistanceCm(&obj->rear);
}

float RangeSensor_GetFrontDistanceFilteredCm(RangeSensor* obj) {
  return obj->front_lpf_seeded ? (float)obj->front_lpf.current_val : ULTRASONIC_NO_ECHO;
}

float RangeSensor_GetRearDistanceFilteredCm(RangeSensor* obj) {
  return obj->rear_lpf_seeded ? (float)obj->rear_lpf.current_val : ULTRASONIC_NO_ECHO;
}
