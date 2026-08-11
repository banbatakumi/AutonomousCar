#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <stdbool.h>
#include <stdint.h>

#include "buzzer.h"
#include "digitalinout.h"
#include "drive.h"
#include "heartbeat.h"
#include "lighting.h"
#include "power.h"
#include "ras_link.h"
#include "steering.h"
#include "timer.h"

// ===========================================================================
// 車両統括 (上位指令と安全層の調停)
//
// 上位 (Raspberry Pi) の指令・緊急停止・フェイルセーフのどれを車両へ適用するかを毎周期
// 決める層。個々のアクチュエータ制御 (Drive / Steering / Lighting) には踏み込まず、
// 「今どの指令源に従うか」と「上位の指令を車両が受け付けられる形に均す」ことだけを担う。
//
// 適用の優先順位:
//   1. 緊急停止がラッチ中          → フェイルセーフ
//   2. COMMAND が生きている        → 上位の指令
//   3. それ以外 (未接続・途絶)     → フェイルセーフ
// ===========================================================================

// クラクションの音程 [Hz]。押している間だけ鳴らすため、単発ビープではなく連続トーンで出す
#define VEHICLE_HORN_FREQ_HZ 2000
// ARM解除からLiDAR電源を落とすまでの遅延 [s]。信号待ちなど短い停止のたびに電源を入り切り
// すると、LD06は起動から安定したスキャンが出るまで数秒かかるため、再ARM直後にセンサが
// 使えない空白ができてしまう。この遅延の間はARM解除中もLiDARを点けたままにする
#define VEHICLE_LIDAR_IDLE_OFF_DELAY_S 5.0f

typedef struct {
  RasLink* ras_link;
  Drive* drive;
  Steering* steering;
  Lighting* lighting;
  Power* power;
  Heartbeat* heartbeat;
  Buzzer* buzzer;
  DigitalIn* estop_reset_button;

  // 上位から指令された目標値を accel_limit / steer_rate_limit でレート制限したあとの値で、
  // 実際に Drive / Steering へ渡している量。テレメトリの steer_cmd もこれを返す
  float applied_speed_m_s;
  float applied_steer_rad;
  Timer command_rate_timer;

  uint8_t mode;  // RasMode
  bool estop_latched;
  bool horn_on;

  // ARMが外れている(=走らせる予定がない)時間を計り、LiDAR電源の遅延OFFに使う
  Timer lidar_idle_timer;
  bool lidar_on;
} Vehicle;

/**
 * @brief 車両統括を初期化する。estop_reset_button には緊急停止の解除に使うボタン
 * (ボタン2) を渡すこと。
 */
void Vehicle_Init(Vehicle* obj, RasLink* ras_link, Drive* drive, Steering* steering,
                  Lighting* lighting, Power* power, Heartbeat* heartbeat, Buzzer* buzzer,
                  DigitalIn* estop_reset_button);

/**
 * @brief ハートビート監視・緊急停止の判定と、指令の車両への適用を1周期分行う。
 * Drive_Update / Motors_Update より前に、制御周期ごとに呼ぶこと。
 */
void Vehicle_Update(Vehicle* obj);

/**
 * @brief 緊急停止がラッチ中かを取得する。
 */
bool Vehicle_IsEstopLatched(const Vehicle* obj);

/**
 * @brief 現在の走行モード (RasMode) を取得する。
 */
uint8_t Vehicle_GetMode(const Vehicle* obj);

/**
 * @brief 直近に Steering へ渡した路面舵角 [rad] を取得する (レート制限後の値)。
 */
float Vehicle_GetAppliedSteerRad(const Vehicle* obj);

#endif  // VEHICLE_H_
