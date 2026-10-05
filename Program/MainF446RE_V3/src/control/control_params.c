#include "control_params.h"

#include <stddef.h>

typedef struct {
  uint16_t id;
  uint16_t offset;
  float def;
  float lo;
  float hi;
} ParamEntry;

static const ParamEntry kTable[] = {
#define CONTROL_PARAM_ENTRY(name, id, def, lo, hi) {id, (uint16_t)offsetof(ControlParams, name), def, lo, hi},
    CONTROL_PARAM_TABLE(CONTROL_PARAM_ENTRY)
#undef CONTROL_PARAM_ENTRY
};

#define PARAM_COUNT (sizeof(kTable) / sizeof(kTable[0]))

static float* Field(ControlParams* obj, const ParamEntry* e) {
  return (float*)((uint8_t*)obj + e->offset);
}

static const ParamEntry* Find(uint16_t param_id) {
  for (uint32_t i = 0; i < PARAM_COUNT; i++) {
    if (kTable[i].id == param_id) return &kTable[i];
  }
  return NULL;
}

void ControlParams_SetDefaults(ControlParams* obj) {
  for (uint32_t i = 0; i < PARAM_COUNT; i++) *Field(obj, &kTable[i]) = kTable[i].def;
}

ControlParamResult ControlParams_Set(ControlParams* obj, uint16_t param_id, float value, float* applied) {
  const ParamEntry* e = Find(param_id);
  if (e == NULL) {
    *applied = 0.0f;
    return CONTROL_PARAM_UNKNOWN;
  }
  float clamped = value;
  if (!(value == value)) {
    clamped = e->def;  // 非数は比較がすべて偽になり、下の丸めをすり抜ける
  } else if (value < e->lo) {
    clamped = e->lo;
  } else if (value > e->hi) {
    clamped = e->hi;
  }
  *Field(obj, e) = clamped;
  *applied = clamped;
  return (clamped == value) ? CONTROL_PARAM_OK : CONTROL_PARAM_CLAMPED;
}

bool ControlParams_Get(const ControlParams* obj, uint16_t param_id, float* value) {
  const ParamEntry* e = Find(param_id);
  if (e == NULL) return false;
  *value = *Field((ControlParams*)obj, e);
  return true;
}
