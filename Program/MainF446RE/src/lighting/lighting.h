#ifndef LIGHTING_H_
#define LIGHTING_H_

#include <stdbool.h>

// ウィンカー方向
typedef enum {
  WINKER_OFF = 0,
  WINKER_LEFT,
  WINKER_RIGHT,
} WinkerDirection;

// LED ペリフェラルを初期化し、起動確認アニメーションを再生する。
void Lighting_Init(void);

// 点滅タイマを更新し、各 LED の PWM を出力する。メインループで毎ティック呼ぶこと。
void Lighting_Update(void);

// ブレーキランプの状態を設定する。
// active=false のとき薄く点灯。active=true かつ strength>=1.0 かつ speed が閾値以上のとき点滅。
void Lighting_SetBrake(bool active, float strength, float speed);

// ウィンカーの点滅方向を設定する。WINKER_OFF で消灯。
void Lighting_SetWinker(WinkerDirection direction);

// フロントライトの輝度を設定する [0.0, 1.0]。パッシング中は上書きされる。
void Lighting_SetHeadlight(float brightness);

// パッシング（フロントライト短点滅）をトリガする。
void Lighting_Passing(void);

#endif  // LIGHTING_H_
