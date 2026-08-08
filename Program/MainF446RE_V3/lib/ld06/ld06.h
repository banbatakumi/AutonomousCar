#ifndef LD06_H_
#define LD06_H_

#include <stdbool.h>
#include <stdint.h>

#include "pwm_out.h"
#include "serial.h"
#include "timer.h"

// LD06 LiDAR のパケットパース。1パケット47バイト固定で12点を含む。
//
// このモジュールが持つのは「センサが送ってきたものをそのまま物理量に直す」ところまでで、
// 取り付け方向の補正・ビニング・セクタ組み立てといった車両側の都合は src/sensing/lidar が担う。
// 角度はセンサ基準のまま返す。

#define LD06_POINT_PER_PACK 12
#define LD06_PACKET_SIZE 47

typedef struct {
  uint16_t distance;   // 距離 [mm] (0 = 無効)
  uint8_t confidence;  // 信頼度 (0-255)
  float angle;         // 角度 [deg] センサ基準 0.0-360.0
} LD06_Point;

typedef struct {
  Serial* serial;
  PwmOut* motor;
  uint8_t rx_buf[LD06_PACKET_SIZE];
  uint8_t rx_state;

  // 直近で受信したパケットのパース結果
  float speed_dps;      // 回転速度 [deg/s] (10Hz 回転なら 3600)
  float span_deg;       // このパケットの12点が覆う角度幅 [deg]
  uint32_t rx_time_us;  // パケットの最終バイトを処理した時刻 (Micros())
  uint16_t timestamp_ms;  // センサ内部のタイムスタンプ [ms]
  LD06_Point points[LD06_POINT_PER_PACK];

  uint32_t rx_count;        // CRC が一致したパケットの累積数
  uint32_t rx_error_count;  // CRC 不一致で破棄したパケット数 (配線・ノイズ環境の確認用)
} LD06;

/**
 * @brief LD06 を初期化する。motor_control には回転モータの PWM 出力を渡す
 * (デューティの設定は呼び出し側が行う。このモジュールは触らない)。
 */
void LD06_Init(LD06* lidar, Serial* serial, PwmOut* motor_control);

/**
 * @brief 溜まっている受信データを処理する。メインループごとに呼ぶこと。
 * パケットを1つ以上正常にパースできた場合 true を返す (points/speed_dps が更新されている)。
 * CRC 不一致のパケットは破棄してカウンタだけ進める。
 */
bool LD06_Update(LD06* lidar);

#endif  // LD06_H_
