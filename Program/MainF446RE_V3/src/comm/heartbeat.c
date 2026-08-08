#include "heartbeat.h"

void Heartbeat_Init(Heartbeat* obj, GPIO_TypeDef* port, uint16_t pin) {
  // ピンは CubeMX の設定 (入力・プルアップ無し) のまま使う。基板側に外付けの
  // プルアップがあるため、Pi がオープンドレインで駆動しても High 側は確定する
  DigitalIn_Init(&obj->pin, port, pin);

  obj->last_level = DigitalIn_Read(&obj->pin);
  obj->last_edge_us = Micros();
  obj->edge_count = 0;
}

void Heartbeat_Update(Heartbeat* obj) {
  int level = DigitalIn_Read(&obj->pin);
  if (level == obj->last_level) return;

  obj->last_level = level;
  obj->last_edge_us = Micros();
  if (obj->edge_count < UINT32_MAX) obj->edge_count++;
}

bool Heartbeat_IsAlive(const Heartbeat* obj) {
  if (obj->edge_count < HEARTBEAT_MIN_EDGES) return false;
  return (uint32_t)(Micros() - obj->last_edge_us) < HEARTBEAT_TIMEOUT_US;
}

bool Heartbeat_HasEverBeenSeen(const Heartbeat* obj) {
  return obj->edge_count >= HEARTBEAT_MIN_EDGES;
}

uint32_t Heartbeat_GetEdgeCount(const Heartbeat* obj) {
  return obj->edge_count;
}
