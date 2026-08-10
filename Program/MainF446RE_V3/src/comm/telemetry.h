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

// MDからの状態フレームがこの時間更新されなければ通信断とみなす [ms]。
// MDが無言になっても保持している値は最後の正常値のまま固まるため、これが無いと
// 上位は「古い正常値」を現在値だと信じ続けることになる
#define TELEMETRY_MD_COMM_TIMEOUT_MS 100

typedef struct {
  uint32_t last_rx_count;
  Timer timer;
  bool ok;
} MdCommWatch;

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

  // [左後輪, 右後輪, ステアリング] の順 (プロトコルの配列インデックス規約に合わせる)
  MdCommWatch md_comm_watch[3];
} Telemetry;

/**
 * @brief テレメトリの組み立てを初期化する。
 */
void Telemetry_Init(Telemetry* obj, RasLink* ras_link, const Vehicle* vehicle, Power* power,
                    Encoder* encoder, Imu* imu, Lidar* lidar, Motors* motors, Steering* steering,
                    Drive* drive, RangeSensor* range_sensor);

/**
 * @brief MD の通信監視を進め、次に送る TELEMETRY / STATS の内容を最新値に差し替える。
 * 制御周期ごとに呼ぶこと (実際の送信は RasLink 側で間引かれる)。
 */
void Telemetry_Update(Telemetry* obj);

/**
 * @brief 組み上がった LiDAR のセクタがあれば送信キューへ積む。
 * 1周を12分割して送るため、制御周期ごとに呼べば 120Hz で送られる。
 */
void Telemetry_PublishLidarSector(Telemetry* obj);

#endif  // TELEMETRY_H_
