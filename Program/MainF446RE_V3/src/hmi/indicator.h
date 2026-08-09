#ifndef INDICATOR_H_
#define INDICATOR_H_

#include <stdbool.h>

#include "lighting.h"
#include "power.h"
#include "pwm_out.h"
#include "timer.h"

// ===========================================================================
// 機体の状態を人間へ見せる表示をまとめたモジュール。
//
// 上位からの指令で点く灯火 (ブレーキ灯・前照灯・パッシング) は車両制御側 (Vehicle) が
// 扱い、こちらは「機体が今どういう状態か」から導かれる表示だけを担当する。
// ハザードはウィンカーと灯火を共有するため、方向指示を実装するときはここで優先度を
// 調停すること。
// ===========================================================================

// --- 電源状態の表示 (LED3: シグナル系, LED4: 駆動系) ---
//
// 電圧を「呼吸」の周期にマップして脈打たせる。周期から電圧の目安が読めるうえ、脈が
// 止まれば制御ループが回っていないことも同時に分かる。輝度に連続量を載せても人間には
// 読めないため、輝度は1bitの情報として使い、その系統にフォールトが出たときだけ最大にする。
#define INDICATOR_BREATH_PERIOD_FULL_MS 2000.0f  // 満充電時はゆったり脈打つ
#define INDICATOR_BREATH_PERIOD_EMPTY_MS 300.0f  // 終止電圧に近いほど速く脈打つ (速い=危険、警報の慣習に合わせる)
#define INDICATOR_BREATH_DUTY_NORMAL 0.1f        // 正常時のピーク輝度
#define INDICATOR_BREATH_DUTY_FAULT 1.0f         // フォールト時のピーク輝度

typedef struct {
  PwmOut* led;
  Timer timer;
} BreathLed;

typedef struct {
  Power* power;
  Lighting* lighting;
  BreathLed breath_signal;
  BreathLed breath_drive;
} Indicator;

/**
 * @brief 状態表示を初期化する。signal_led / drive_led には PwmOut_Init 済みの
 * LED3 / LED4 を渡すこと。
 */
void Indicator_Init(Indicator* obj, Power* power, Lighting* lighting,
                    PwmOut* signal_led, PwmOut* drive_led);

/**
 * @brief 電源表示と異常表示を1周期分更新する。制御周期ごとに呼ぶこと。
 * estop_active には緊急停止のラッチ状態を渡す (電源フォールトと同じくハザードで表示する)。
 * Lighting_Update より前に呼ぶこと。
 */
void Indicator_Update(Indicator* obj, bool estop_active);

#endif  // INDICATOR_H_
