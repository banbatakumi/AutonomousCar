#ifndef LD06_H_
#define LD06_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "digitalinout.h"
#include "serial.h"

#define LD06_POINT_PER_PACK 12
#define LD06_PACKET_SIZE 47

// LiDARの1点のデータ構造
typedef struct {
  uint16_t distance;   // 距離 [mm]
  uint8_t confidence;  // 信頼度 (0-255)
  float angle;         // 角度 [degree]
} LD06_Point;

// LD06の制御・データ保持用インスタンス構造体
typedef struct {
  Serial* serial;
  DigitalOut* motor;   // LiDAR回転モーター制御ポインタ（外部から提供）
  uint8_t rx_buf[LD06_PACKET_SIZE];
  uint8_t rx_state;

  // 直近で受信したパケットのパース結果
  float speed;         // 回転速度 [degrees per second]
  uint16_t timestamp;  // タイムスタンプ [ms]
  LD06_Point points[LD06_POINT_PER_PACK];

  // 360度分の最新距離・信頼度データ (配列インデックス = 角度 0~359度)
  uint16_t distances_360[360];
  uint8_t confidences_360[360];
} LD06;

// 初期化関数（motor_controlは呼び出し側で初期化済みのDigitalOut*ポインタ）
void LD06_Init(LD06* lidar, Serial* serial, DigitalOut* motor_control);

// 受信処理とデータ更新 (メインループで定期的に呼び出す)
// 新しいパケットが受信され、正常にパース完了した場合は true を返す
bool LD06_Update(LD06* lidar);

#endif  // LD06_H_
