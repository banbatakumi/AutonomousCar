#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include <stdint.h>

#include "drive.h"
#include "encoder.h"
#include "imu.h"
#include "lidar.h"
#include "motors.h"
#include "power.h"
#include "range_sensor.h"
#include "ras_link.h"
#include "steering.h"
#include "timer.h"
#include "vehicle.h"

// ===========================================================================
// 各モジュールの観測量を上位 (Raspberry Pi) へ報告する形に組み立てる層。
//
// RasLink がフレーミングと量子化を担うのに対し、こちらは「どのモジュールの値を
// プロトコルのどのフィールドへ入れるか」だけを担う。制御には一切関与しない。
// ===========================================================================

typedef struct {
  RasLink* ras_link;
  const Vehicle* vehicle;
  Power* power;
  Encoder* encoder;
  Imu* imu;
  Lidar* lidar;
  Motors* motors;
  Steering* steering;
  Drive* drive;
  RangeSensor* range_sensor;

  // 直近に RasLink_Log で通知済みの PowerFault ビット。新規に立ったビットだけを通知するのに使う
  uint32_t logged_faults;

  // Telemetry_Update() は2kHzで呼ばれるが、組み立ては送信周期 (RAS_TELEMETRY_INTERVAL_US) に
  // 合わせて内部で間引く (ADC読み・MD状態集計などを送信されない周期に回さないため)
  Timer update_timer;
} Telemetry;

/**
 * @brief テレメトリの組み立てを初期化する。
 */
void Telemetry_Init(Telemetry* obj, RasLink* ras_link, const Vehicle* vehicle, Power* power,
                    Encoder* encoder, Imu* imu, Lidar* lidar, Motors* motors, Steering* steering,
                    Drive* drive, RangeSensor* range_sensor);

/**
 * @brief 次に送る TELEMETRY / STATS の内容を最新値に差し替える。
 * 制御周期ごとに呼んでよい (実際の組み立て・送信は内部で RAS_TELEMETRY_INTERVAL_US = 100Hz に
 * 間引かれるため、毎周期呼んでも2kHzで重い処理が走るわけではない)。
 */
void Telemetry_Update(Telemetry* obj);

/**
 * @brief 組み上がった LiDAR のセクタがあれば送信キューへ積む。
 * 1周を12分割して送るため、制御周期ごとに呼べば 120Hz で送られる。
 */
void Telemetry_PublishLidarSector(Telemetry* obj);

#endif  // TELEMETRY_H_
