#include "indicator.h"

#include "mymath.h"

static void BreathLed_Init(BreathLed* obj, PwmOut* led) {
  obj->led = led;
  Timer_Init(&obj->timer);
}

// voltage には Power 側で LPF を通した値を渡すこと。生値だと周期がガタついて読めない
static void BreathLed_Update(BreathLed* obj, float voltage, bool fault) {
  float level = (voltage - POWER_UNDERVOLTAGE_THRESHOLD_V) /
                (POWER_BATTERY_FULL_V - POWER_UNDERVOLTAGE_THRESHOLD_V);
  level = Constrain(level, 0.0f, 1.0f);
  float period_ms = INDICATOR_BREATH_PERIOD_EMPTY_MS +
                    level * (INDICATOR_BREATH_PERIOD_FULL_MS - INDICATOR_BREATH_PERIOD_EMPTY_MS);

  uint32_t elapsed_ms = Timer_ReadMs(&obj->timer);
  if ((float)elapsed_ms >= period_ms) {
    Timer_Reset(&obj->timer);
    elapsed_ms = 0;
  }

  // 波形全体を周期に比例させることで、周期が変わっても点灯時間の割合 (=平均輝度) が変わらない。
  // 点灯時間を固定したまま周期だけ縮めると明るさまで変化し、輝度の1bit情報と混ざってしまう
  float shape = (1.0f - CosDeg((int)(360.0f * elapsed_ms / period_ms))) * 0.5f;
  // 人間の輝度知覚は対数的なので、shape をそのまま duty にすると暗い側が潰れる
  float peak = fault ? INDICATOR_BREATH_DUTY_FAULT : INDICATOR_BREATH_DUTY_NORMAL;
  PwmOut_Write(obj->led, peak * shape * shape);
}

static void UpdatePowerIndication(Indicator* obj) {
  uint32_t faults = Power_GetFaults(obj->power);
  BreathLed_Update(&obj->breath_signal, Power_GetVoltageSignalFiltered(obj->power),
                   (faults & POWER_FAULT_SIGNAL_ANY) != 0);
  BreathLed_Update(&obj->breath_drive, Power_GetVoltageDriveFiltered(obj->power),
                   (faults & POWER_FAULT_DRIVE_ANY) != 0);
}

// 何らかの異常が出ていればハザードを点滅させ、車外から異常だと分かるようにする。
// 復帰しうる異常 (電圧低下) もあるため、消えたらハザードも消す。
// 電源系以外のモジュールのエラーも、実装したらここに集約する。
//
// 上位 (Raspberry Pi) からの方向指示要求 (winker_request) はフォールト時のハザードより
// 優先度が低い。ウィンカーとハザードは Lighting 上で表現を共有しているため、この
// 調停を一箇所 (ここ) に集約する。
static void UpdateFaultIndication(Indicator* obj, bool estop_active,
                                  LightingWinkerState winker_request) {
  bool fault = Power_GetFaults(obj->power) != POWER_FAULT_NONE || estop_active;
  Lighting_SetWinker(obj->lighting, fault ? LIGHTING_WINKER_HAZARD : winker_request);
}

void Indicator_Init(Indicator* obj, Power* power, Lighting* lighting,
                    PwmOut* signal_led, PwmOut* drive_led) {
  obj->power = power;
  obj->lighting = lighting;
  BreathLed_Init(&obj->breath_signal, signal_led);
  BreathLed_Init(&obj->breath_drive, drive_led);
}

void Indicator_Update(Indicator* obj, bool estop_active, LightingWinkerState winker_request) {
  UpdateFaultIndication(obj, estop_active, winker_request);
  UpdatePowerIndication(obj);
}
