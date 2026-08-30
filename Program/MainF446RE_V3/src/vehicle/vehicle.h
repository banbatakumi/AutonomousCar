#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <stdbool.h>
#include <stdint.h>

#include "buzzer.h"
#include "digitalinout.h"
#include "drive.h"
#include "heartbeat.h"
#include "lidar.h"
#include "lighting.h"
#include "power.h"
#include "range_sensor.h"
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
// 自動停止 (RAS_CMD_FLAG_AUTO_STOP) の動的停止距離
// d_stop = v・VEHICLE_AUTO_STOP_DELAY_S + v²/(2・DRIVE_MAX_ACCEL_M_S2) + マージン
// のうち、通信・処理・制動応答の遅延見積り [s] (未実測プレースホルダー。実測後に更新すること)
#define VEHICLE_AUTO_STOP_DELAY_S 0.1f
// 上式の a_max には車体質量・実測減速度が無いため、既存の駆動加速上限 (DRIVE_MAX_ACCEL_M_S2,
// src/control/drive.h) を制動側の近似としてそのまま流用する。
// 安全マージンは上位が RasConfig.auto_stop_margin_cm (CONFIG_SET param_id = 0x0060) で
// cm単位の連続値として直接指定する (段階的なレベルではない。既定値・範囲は ras_link.h の
// RAS_AUTO_STOP_MARGIN_* 参照)
// 実車速がこの絶対値未満 (ほぼ静止) のときは、前後判定に実車速ではなく上位が指令した方向を
// 使う。静止中は実車速の符号が定まらず常に前方判定に固定されてしまうため
#define VEHICLE_AUTO_STOP_DIRECTION_DEADBAND_M_S 0.05f
// 前方0度・後方180度を中心に、LiDARのセーフティゾーンとして見る片側角度幅 [deg]
#define VEHICLE_AUTO_STOP_LIDAR_HALF_WIDTH_DEG 20.0f
// 上の角度窓内で、停止距離以内の点がこの数以上あれば障害物ありと確定する。単発ノイズは
// 1点しか占めないため、時間方向のデバウンスではなく空間方向の点密度で誤検知を弾く
#define VEHICLE_AUTO_STOP_LIDAR_MIN_POINTS 3
// LiDARの結果に関わらず、超音波がこの距離未満を検知したら常に停止する (至近距離の保険。
// LiDARの最短測距距離や取付高さの死角を補う) [cm]
#define VEHICLE_AUTO_STOP_ULTRASONIC_NEAR_CM 5.0f

// 各センサの車体先端(バンパー)からの後退量 [cm]。上位 (Raspberry Pi) 側の
// raspi/config/vehicle.toml (base_link = 後輪車軸中心、実測確定 2026-08-20) の
// footprint (前端 x=+30cm、後端 x=-7cm) とセンサ取付座標から算出した固定値。
// センサはいずれも車体先端より内側に付いているため、実測距離はそのままだと
// 「車体先端から障害物まで」より長く出る。しきい値側にこの分を足して補正する
// (実測距離 - オフセット = 車体先端から障害物までの真の距離、と同値の変形)
#define VEHICLE_AUTO_STOP_LIDAR_FRONT_OFFSET_CM 23.0f       // LiDAR(x=7cm) → 前端(x=30cm)
#define VEHICLE_AUTO_STOP_LIDAR_REAR_OFFSET_CM 14.0f        // LiDAR(x=7cm) → 後端(x=-7cm)
#define VEHICLE_AUTO_STOP_ULTRASONIC_FRONT_OFFSET_CM 4.0f   // 前方超音波(x=26cm) → 前端(x=30cm)
#define VEHICLE_AUTO_STOP_ULTRASONIC_REAR_OFFSET_CM 3.0f    // 後方超音波(x=-4cm) → 後端(x=-7cm)
// この制動トルク以上なら、実車の急制動警告のようにブレーキランプを高速点滅させる [Nm/輪]
#define VEHICLE_EMERGENCY_BRAKE_FLASH_THRESHOLD_NM 0.1f

typedef struct {
  RasLink* ras_link;
  Drive* drive;
  Steering* steering;
  Lighting* lighting;
  Power* power;
  Heartbeat* heartbeat;
  Buzzer* buzzer;
  DigitalIn* estop_reset_button;
  RangeSensor* range_sensor;
  Lidar* lidar;

  // 上位から指令された目標舵角を steer_rate_limit でレート制限したあとの値で、
  // 実際に Steering へ渡している量。テレメトリの steer_cmd もこれを返す
  // (目標車速の加速度レート制限は Drive 側 (Drive_SetTargetSpeed) が持つ)
  float applied_steer_rad;
  Timer command_rate_timer;

  uint8_t mode;  // RasMode
  bool estop_latched;
  bool horn_on;

  // 上位から要求されたウィンカー/ハザード状態 (flags2 の WINKER_LEFT/RIGHT をデコードしたもの)。
  // Lighting へ直接反映せず、フォールト時のハザード優先の調停は Indicator (src/hmi/indicator.c)
  // に委ねる (ハザードは Indicator が既に Lighting_SetWinker の唯一の呼び出し元だったため)
  LightingWinkerState winker_request;

  // ARMが外れている(=走らせる予定がない)時間を計り、LiDAR電源の遅延OFFに使う
  Timer lidar_idle_timer;
  bool lidar_on;

  // 直近の周期で自動停止 (RAS_CMD_FLAG_AUTO_STOP) が実際に制動へ介入したか。
  // テレメトリの RAS_FLAG_AUTO_STOP_ACTIVE に使う
  bool auto_stop_active;
} Vehicle;

/**
 * @brief 車両統括を初期化する。estop_reset_button には緊急停止の解除に使うボタン
 * (ボタン2) を渡すこと。range_sensor と lidar には自動停止 (RAS_CMD_FLAG_AUTO_STOP) が
 * 参照する前後超音波センサ・LiDARを渡すこと。
 */
void Vehicle_Init(Vehicle* obj, RasLink* ras_link, Drive* drive, Steering* steering,
                  Lighting* lighting, Power* power, Heartbeat* heartbeat, Buzzer* buzzer,
                  DigitalIn* estop_reset_button, RangeSensor* range_sensor, Lidar* lidar);

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

/**
 * @brief 直近の周期で自動停止 (RAS_CMD_FLAG_AUTO_STOP) が実際に制動へ介入したかを取得する。
 */
bool Vehicle_IsAutoStopActive(const Vehicle* obj);

/**
 * @brief 上位から要求されたウィンカー/ハザード状態を取得する。フォールト時のハザード優先など
 * 実際に Lighting へ反映する際の調停は Indicator (src/hmi/indicator.c) が行うため、この値は
 * あくまで上位の「要求」であって実際の点灯状態と一致するとは限らない。
 */
LightingWinkerState Vehicle_GetWinkerRequest(const Vehicle* obj);

/**
 * @brief 左/右ウィンカーが実際に点滅中かどうかを取得する (TELEMETRY.flags の
 * WINKER_LEFT/RIGHT_ACTIVE 用)。Lighting の実際の状態を見るため、フォールトによる
 * ハザード優先で上位の要求と異なる状態になっている場合もそれを反映する。
 */
bool Vehicle_IsWinkerLeftActive(const Vehicle* obj);
bool Vehicle_IsWinkerRightActive(const Vehicle* obj);

#endif  // VEHICLE_H_
