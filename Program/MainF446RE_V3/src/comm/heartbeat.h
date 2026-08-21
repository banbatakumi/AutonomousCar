#ifndef HEARTBEAT_H_
#define HEARTBEAT_H_

#include <stdbool.h>
#include <stdint.h>

#include "digitalinout.h"
#include "timer.h"

// Raspberry Pi (上位) の生存を、UART とは独立した信号線 1本 (RAS_SIG = PB12) で監視する。
//
// Pi は 100Hz の矩形波を出し続け、こちらはエッジが途切れたことで異常を検出する。
// 「Lowで停止」のような単純なレベル方式にしないのは、断線するとプルアップで High
// (= 正常) 側へ戻ってしまいフェイルセーフにならないため。矩形波なら
// Pi のフリーズ・電源断・配線の断線・プロセスのクラッシュを 1つの機構でまとめて検出できる。
//
// **外部割り込みは使えない。** RAS_SIG(PB12) と ECHO_FRONT(PA12) は同じ EXTI ライン12を
// 使い、STM32 の EXTI はライン番号ごとに 1ポートしか選べないため。100Hz 矩形波の
// エッジ間隔 5ms に対して制御ループは 2kHz なので、ポーリングで十分間に合う。

// エッジがこの時間途切れたら異常とみなす [us]
#define HEARTBEAT_TIMEOUT_US 50000u

// これだけエッジを数えて初めて「Pi が繋がっている」とみなす。
// 単発のノイズで接続済みと判定してしまうと、その直後に途絶と見なして緊急停止に落ちる
#define HEARTBEAT_MIN_EDGES 10u

typedef struct {
  DigitalIn pin;
  int last_level;
  uint32_t last_edge_us;
  uint32_t edge_count;
} Heartbeat;

/**
 * @brief ハートビート監視を初期化する。
 * 基板に外付けプルアップがあるため、Pi はオープンドレインで駆動してよい。
 */
void Heartbeat_Init(Heartbeat* obj, GPIO_TypeDef* port, uint16_t pin);

/**
 * @brief 信号線を読んでエッジを検出する。制御周期ごとに呼ぶこと。
 */
void Heartbeat_Update(Heartbeat* obj);

/**
 * @brief 上位が生きているか (直近 HEARTBEAT_TIMEOUT_US 以内にエッジがあったか) を取得する。
 */
bool Heartbeat_IsAlive(const Heartbeat* obj);

/**
 * @brief 起動後に一度でも上位の生存を確認したかを取得する。
 * 偽の間はハートビートが配線されていない (単体でのベンチ確認中など) とみなし、
 * 緊急停止を発動させないこと。
 */
bool Heartbeat_HasEverBeenSeen(const Heartbeat* obj);

/**
 * @brief 検出したエッジの累積数を取得する。配線確認用。
 */
uint32_t Heartbeat_GetEdgeCount(const Heartbeat* obj);

/**
 * @brief 直近のエッジ時刻を現在時刻へ強制的に更新する (エッジ検出済みとしては数えない)。
 * 他モジュールの長時間ブロッキング (例: IMU の I2C 復旧、docs/code_review_2026-08-21.md A-3)
 * でエッジのポーリング自体ができなかった期間を、通信断とみなさないようにする応急処置用。
 * 本来のエッジ検出を置き換えるものではないため、多用しないこと。
 */
void Heartbeat_ResetBaseline(Heartbeat* obj);

#endif  // HEARTBEAT_H_
