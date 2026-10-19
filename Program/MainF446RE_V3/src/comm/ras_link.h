#ifndef RAS_LINK_H_
#define RAS_LINK_H_

#include <stdbool.h>
#include <stdint.h>

#include "control_params.h"
#include "serial.h"
#include "timer.h"

// ===========================================================================
// Raspberry Pi (上位) との UART 通信。レイアウトは ras_link.c、近年の変更点は
// docs/pi_uart_protocol_v0.19_delta.md 〜 v0.23_delta.md (旧版は docs/archive/)。
//
// 物理層: USART1, 1000000bps 8N1 (分周誤差0%。2026-09-26 に 250000bps から変更)。
//
// このモジュールが持つのはフレーミング (SYNC/TYPE/SEQ/LEN/CRC16) とパケットの
// 解釈・組み立てだけで、走行制御そのものには関与しない。受信した指令は
// RasLink_GetCommand() で取り出し、送るテレメトリは RasLink_SetTelemetry() で
// 物理量のまま渡す (量子化はこのモジュールが行う)。
//
// 送信は優先度つきキューを持ち、PONG > TELEMETRY > LIDAR > CONFIG_ACK等 > LOG の
// 順に流す。Serial_Write は進行中の DMA を中断してしまうため使わず、
// Serial_WriteAsync でフレーム単位に送出する。
// ===========================================================================

#define RAS_PROTOCOL_VERSION 0x001Cu
// COMMAND.accel_limit (u16 × 0.001 m/s^2) で表せる最大。LIMITS.max_accel_m_s2 はこの値を返す
// (STM32 側に加速度上限は無い。「上限なし」をワイヤ上の最大で伝える)
#define RAS_ACCEL_LIMIT_WIRE_MAX_M_S2 65.535f
// "MF3" + 版数。Pi 側のログで機体を識別するための任意値。0x04 (2026-09-27): TELEMETRY の t_us を
// 送信時刻からスナップショットの時刻に変えた (ワイヤ形式は同じなので protocol_version は据え置き)。
// Pi の同定の解析はこれより古い記録の遅延に警告を出す (tools/sysid/fit.py)
#define RAS_FIRMWARE_ID 0x4D463304u

#define RAS_SYNC1 0xAAu
#define RAS_SYNC2 0x55u
#define RAS_FRAME_OVERHEAD 7  // SYNC(2) + TYPE + SEQ + LEN + CRC16(2)
#define RAS_PAYLOAD_MAX 255
// 受信で受け付ける最長のフレーム (SYNC2 + TYPE/SEQ/LEN 3 + 最長ペイロード + CRC2) に余裕を足した長さ
#define RAS_RX_RAW_MAX 32
#define RAS_FRAME_MAX (RAS_FRAME_OVERHEAD + RAS_PAYLOAD_MAX)

// LiDAR 1セクタあたりの点数 (1周360点を12分割)
#define RAS_LIDAR_POINTS_PER_SECTOR 30
#define RAS_LIDAR_SECTOR_NUM 12

// COMMAND がこの時間途絶したら自動ブレーキへ落とす [us]
#define RAS_COMMAND_TIMEOUT_US 100000u

// 定期送信の周期 [us]
// 100Hz (2026-09-26 に 50Hz から変更。上位が受け取る値の古さ (次のスナップショットまでの待ち)
// を平均10ms→5msに縮める。1Mbps で上りの使用率は約21%)
#define RAS_TELEMETRY_INTERVAL_US 10000u   // 100Hz
#define RAS_STATS_INTERVAL_US 1000000u     // 1Hz
#define RAS_VERSION_BURST_INTERVAL_US 100000u
#define RAS_VERSION_BURST_COUNT 3

typedef enum {
  RAS_TYPE_LIDAR_SECTOR = 0x01,
  RAS_TYPE_TELEMETRY = 0x02,
  RAS_TYPE_CONFIG_ACK = 0x03,
  RAS_TYPE_LOG = 0x04,
  RAS_TYPE_LIDAR_SECTOR_I = 0x05,
  RAS_TYPE_PONG = 0x06,
  RAS_TYPE_VERSION = 0x07,
  RAS_TYPE_STATS = 0x08,
  RAS_TYPE_LIDAR_SECTOR_C = 0x09,
  RAS_TYPE_LIMITS = 0x0A,  // 車両の固定上限値
  RAS_TYPE_COMMAND = 0x10,
  RAS_TYPE_CONFIG_SET = 0x11,
  RAS_TYPE_PING = 0x12,
  RAS_TYPE_CONFIG_GET = 0x13,
  RAS_TYPE_VERSION_REQ = 0x14,
  RAS_TYPE_LIMITS_REQ = 0x15,  //
} RasPacketType;

// COMMAND.mode / TELEMETRY.flags bit0-1
typedef enum {
  RAS_MODE_DISARM = 0,
  RAS_MODE_MANUAL = 1,
  RAS_MODE_AUTO = 2,
  RAS_MODE_RESERVED = 3,  // 旧 CALIB (廃止)。受信しても現在のモードを維持する
} RasMode;

// COMMAND.flags
#define RAS_CMD_FLAG_ARM (1u << 0)
#define RAS_CMD_FLAG_BRAKE (1u << 1)
#define RAS_CMD_FLAG_HORN (1u << 2)
// bit3-4 は前照灯モード (RasLightMode)。1ビットでは「前後とも消灯」を表現できないため 2ビット
#define RAS_CMD_LIGHT_MODE_SHIFT 3
#define RAS_CMD_LIGHT_MODE_MASK (0x03u << RAS_CMD_LIGHT_MODE_SHIFT)
#define RAS_CMD_FLAG_PASSING (1u << 5)
// 立っている間は車速PIを迂回し、target_torque を後輪へ直接指令する。brake と同時に
// 立っていたら brake を優先する (Drive_Update 側の優先順位で解決する)
#define RAS_CMD_FLAG_TORQUE_MODE (1u << 6)
// 立っている間、進行方向の障害物までの距離が車速から計算した動的停止距離
// (d_stop = v・t_delay + v²/2a_max + マージン。マージンは RAS_PARAM_AUTO_STOP_MARGIN_CM で
// cm単位で直接指定) を下回ったら最大制動トルクで自動停止する (src/vehicle/vehicle.c)。判定は前後LiDARの
// セーフティゾーン(角度×距離のROI内点数)を主に用い、超音波は LiDAR が死角・欠測のときの
// フォールバックと至近距離の独立トリガーの2役を担う。一度作動したら止まるまで制動を続け、
// 止まってから判定し直す。下りている間は従来通り上位の指令のみに従う
#define RAS_CMD_FLAG_AUTO_STOP (1u << 7)

// COMMAND.flags2 (flags が8bit使い切ったための拡張バイト)
// 立っている間、指令を受けた瞬間の後輪モータ機械角度を目標値としてラッチし、位置制御へ
// 切り替えて機械的に切り替える (パーキングロック)。速度に関わらず即座に切り替わり、
// brake (bit1) より優先する (Drive_Update 側の優先順位で解決する)。
#define RAS_CMD_FLAG2_SIDE_BRAKE (1u << 0)
// 立っている間、左/右ウィンカーを点滅させる (自律走行中の右左折・車線変更の意思表示用)。
// 両方同時に立っていればハザード (左右同時点滅) として扱う。brake/side_brake と違い
// 走行系には関与しないため優先順位の調停は不要。フォールト表示のハザード点滅
// (src/hmi/indicator.c) の方が優先し、フォールト中はこのビットの状態に関わらずハザードになる。
// COMMAND 途絶・緊急停止では horn/passing と同様に強制的に消灯する
#define RAS_CMD_FLAG2_WINKER_LEFT (1u << 1)
#define RAS_CMD_FLAG2_WINKER_RIGHT (1u << 2)
// ローンチコントロール。立っている間、車速指令の目標が実車速より大きく離れた加速 (発進) で、目標車速の
// ランプと車速PIを迂回し、全開のトルク + TC で目標車速まで加速する (始める・終える条件は src/control/drive.h の
// 「ローンチコントロール」の節)。brake・torque_mode・side_brake の間は意味を持たない。
// 実際に発進中かは TELEMETRY.flags の RAS_FLAG_LAUNCH_ACTIVE
#define RAS_CMD_FLAG2_LAUNCH (1u << 3)

// COMMAND.flags bit3-4 (前照灯モード)
typedef enum {
  RAS_LIGHT_OFF = 0,      // 前照灯・尾灯とも消灯
  RAS_LIGHT_DAYTIME = 1,  // デイライト (前照灯減光 + 尾灯薄点灯)
  RAS_LIGHT_NORMAL = 2,   // 通常点灯 (前照灯全光量 + 尾灯薄点灯)
  // 3 は予約。受信したら被視認性の高い NORMAL として扱う
} RasLightMode;

// TELEMETRY.flags
#define RAS_FLAG_MODE_MASK 0x0003u
// 上位の指令で走れる状態 (Vehicle_IsArmed)。「駆動電源が入っている」とは別
// (ステア未較正・MD の起動待ち・ARM が外れたあとの停止待ちでは立たない)。
// 駆動電源の状態は RAS_FLAG_DRIVE_POWER_ON
#define RAS_FLAG_ARMED (1u << 2)
#define RAS_FLAG_ESTOP_ACTIVE (1u << 3)
#define RAS_FLAG_UART_TIMEOUT (1u << 4)
#define RAS_FLAG_TC_ACTIVE (1u << 5)
#define RAS_FLAG_TV_ACTIVE (1u << 6)
#define RAS_FLAG_IMU_OK (1u << 7)
#define RAS_FLAG_LIDAR_OK (1u << 8)
// bit9 は旧 calib_running。CALIB 廃止に伴い予約
#define RAS_FLAG_STEER_CENTER_VALID (1u << 10)
#define RAS_FLAG_FAULT_DRIVE_OVERCURRENT (1u << 11)
#define RAS_FLAG_FAULT_SIGNAL_OVERCURRENT (1u << 12)
#define RAS_FLAG_FAULT_DRIVE_UNDERVOLTAGE (1u << 13)
#define RAS_FLAG_FAULT_SIGNAL_UNDERVOLTAGE (1u << 14)
#define RAS_FLAG_DRIVE_POWER_LOCKED (1u << 15)
// 障害物により自動停止が実際に作動中 (flags.auto_stop が有効かつ検知中)
#define RAS_FLAG_AUTO_STOP_ACTIVE (1u << 16)
// サイドブレーキが実際に位置制御へ切り替わって固定中かどうか (要求中でも
// MDのデータ無効でトルク制動へフォールバックしている間は偽)
#define RAS_FLAG_SIDE_BRAKE_ACTIVE (1u << 17)
// 左/右ウィンカーが実際に点滅中かどうか。フォールトによるハザード表示 (src/hmi/indicator.c)
// が上位のウィンカー要求より優先するため、要求どおりとは限らない (フォールト中は要求に
// 関わらず両方立つ)
#define RAS_FLAG_WINKER_LEFT_ACTIVE (1u << 18)
#define RAS_FLAG_WINKER_RIGHT_ACTIVE (1u << 19)
// ABSが制動トルクを要求より削っている最中かどうか (フォールバック中は偽)
#define RAS_FLAG_ABS_ACTIVE (1u << 20)
// 片輪浮き対策 (後輪左右の速度差・後輪周速の絶対上限) がトルクを削っている最中かどうか。
// このとき RAS_FLAG_TC_ACTIVE は立たない (どちらの偏差が効いたかを分ける)
#define RAS_FLAG_WHEEL_LIFT_ACTIVE (1u << 21)
// 駆動電源 (DRIVE_POWER) が入っている。ARM 要求中のほか、ARM が外れたあと停車を待って
// 制動している間も立つ
#define RAS_FLAG_DRIVE_POWER_ON (1u << 22)
// MD (3台のいずれか) が使えないため走行を止めている (Vehicle_IsMdFault)。どの MD かは
// md_status、理由は LOG で分かる。駆動電源が切れる (ARM を外す) まで立ち続ける
#define RAS_FLAG_MD_FAULT (1u << 23)
// ローンチコントロールで発進している最中 (Drive_IsLaunchActive)。上位が要求 (RAS_CMD_FLAG2_LAUNCH)
// していても、条件を満たさず始まらなかった・もう終わった、なら立たない
#define RAS_FLAG_LAUNCH_ACTIVE (1u << 24)
// ハートビート (RAS_SIG) を一度も検出していない。緊急停止の層が働いていないので、上位は ARM を
// 通さないこと (Pi 側の --no-heartbeat で動かすときを除く)
#define RAS_FLAG_NO_HEARTBEAT (1u << 25)
// ARM 中に LiDAR も前後の超音波も使えない状態が続いている。自動停止が何も検知できない
#define RAS_FLAG_AUTO_STOP_BLIND (1u << 26)

// TELEMETRY.md_status[i]
// bit0 は MD が「受理済みの指令を実行中」(停止モードでない) で、回転中かどうかではない
#define RAS_MD_STATUS_RUNNING (1u << 0)
#define RAS_MD_STATUS_VOLTAGE_OUT_OF_RANGE (1u << 1)
#define RAS_MD_STATUS_OVERHEAT (1u << 2)
#define RAS_MD_STATUS_OVERCURRENT (1u << 3)
#define RAS_MD_STATUS_COMM_OK (1u << 4)
#define RAS_MD_STATUS_LIMIT_SYNCED (1u << 5)
// MD のエンコーダが未校正 (MD は動かない)
#define RAS_MD_STATUS_UNCALIBRATED (1u << 6)

typedef enum {
  RAS_LOG_DEBUG = 0,
  RAS_LOG_INFO = 1,
  RAS_LOG_WARN = 2,
  RAS_LOG_ERROR = 3,
} RasLogSeverity;

// CONFIG_ACK.result
typedef enum {
  RAS_CONFIG_OK = 0,
  RAS_CONFIG_UNKNOWN_PARAM = 1,
  RAS_CONFIG_OUT_OF_RANGE = 2,  // applied にクランプ後の値が入る
  RAS_CONFIG_MODE_LOCKED = 3,
} RasConfigResult;

// 実装済みのパラメータのみを定義する。
// 足回りの制御 (TC・ABS・片輪浮き対策・TV) の調整パラメータは param_id と
// 範囲・既定値は src/control/control_params.h の表が唯一の定義で、ここには並べない
// (0x0011-0x0015, 0x0021-0x002C, 0x0030-0x0031 = 車速PI, 0x0051, 0x0071-0x0073)。Flash へは保存しない。
// 値は持たない: 起動直後は仮置き (下限 = 何もしない値) で、上位が全項目を送るまで ARM を通さない運用。
// TC/TV の ON/OFF (0x0010/0x0020) はゲインではなく機能そのものの有効/無効なので対応している。
// 片輪浮き対策 (0x0050) の ON/OFF は TC本体 (0x0010) とは独立に切替できる。
// 最大速度・最大加速度・最大舵角 (旧 0x0001-0x0003) は廃止した。上位から変更する
// 実用上の必要が無かったため、Drive/Steering 側の固定定数 (DRIVE_MAX_SPEED_M_S /
// Steering_GetMaxRoadWheelAngleRad()) に一本化してある。最大加速度は上限そのものを
// 無くしてある (上位が COMMAND.accel_limit で毎回決める。drive.h の DRIVE_DEFAULT_ACCEL_M_S2 の注記)
// (最大舵角だけはリンクの換算 steer_link_* で変わる。変わるたびに LIMITS を送り直す)
typedef enum {
  RAS_PARAM_TC_ENABLE = 0x0010,     // 0.0=無効, 非0=有効 (既定は無効。上位が接続のたびに送る)
  RAS_PARAM_TV_ENABLE = 0x0020,     // 0.0=無効, 非0=有効 (既定は無効。上位が接続のたびに送る)
  RAS_PARAM_LIDAR_FORMAT = 0x0040,  // RasLidarFormat
  RAS_PARAM_WHEEL_LIFT_GUARD_ENABLE = 0x0050,  // 0.0=無効, 非0=有効 (既定は有効)
  RAS_PARAM_AUTO_STOP_MARGIN_CM = 0x0060,  // 安全マージン [cm] を直接指定
  RAS_PARAM_ABS_ENABLE = 0x0070,           // 0.0=無効, 非0=有効 (既定は有効)
  // ブレーキホールド (制動して止まったら後輪の位置保持へ移る)。0.0=無効, 非0=有効 (既定は有効)
  RAS_PARAM_BRAKE_HOLD_ENABLE = 0x0090,
  // ステアのリンクの換算。値・範囲は control_params.h の表。入れた瞬間に Steering へ反映し
  // LIMITS を送り直すので、ID だけここにも置く
  RAS_PARAM_STEER_LINK_GAIN = 0x0081,
  RAS_PARAM_STEER_LINK_CUBIC = 0x0082,
} RasParamId;

typedef enum {
  RAS_LIDAR_FORMAT_STANDARD = 0,   // 0x01 LIDAR_SECTOR   (u16 mm)
  RAS_LIDAR_FORMAT_INTENSITY = 1,  // 0x05 LIDAR_SECTOR_I (u16 mm + 強度)
  RAS_LIDAR_FORMAT_COMPACT = 2,    // 0x09 LIDAR_SECTOR_C (u8 2cm/LSB)
} RasLidarFormat;

// 自動停止の動的停止距離 (d_stop = v・t_delay + v²/2a_max + マージン) に加える安全マージン
// [cm] の範囲。物理項 (車速由来の制動距離) は車両特性として固定し、上位は margin のみを
// 任意の値で直接指定できる (段階的なレベルではなく連続値)。範囲外はクランプする
#define RAS_AUTO_STOP_MARGIN_MIN_CM 0.0f
#define RAS_AUTO_STOP_MARGIN_MAX_CM 100.0f
#define RAS_AUTO_STOP_MARGIN_DEFAULT_CM 15.0f

typedef struct {
  bool tc_enabled;  // 既定は無効 (上位が送る)
  bool tv_enabled;  // 既定は無効 (上位が送る)
  bool wheel_lift_guard_enabled;  // 既定は有効
  uint8_t lidar_format;
  float auto_stop_margin_cm;  // 既定は RAS_AUTO_STOP_MARGIN_DEFAULT_CM
  bool abs_enabled;           // 既定は有効
  bool brake_hold_enabled;    // 既定は有効
  ControlParams control;      // 足回りの制御の調整パラメータ
} RasConfig;

// 受信した COMMAND をデコードしたもの
typedef struct {
  uint8_t mode;    // RasMode
  uint8_t flags;   // RAS_CMD_FLAG_*
  uint8_t flags2;  // RAS_CMD_FLAG2_*
  float target_speed_m_s;
  float target_steer_rad;  // 路面舵角 (反時計回り = 左旋回が正)
  float accel_limit_m_s2;
  float steer_rate_limit_rad_s;
  // 制動トルク [Nm] (常に正)。0 は「無制動」ではなく「未指定」を意味するので、
  // 呼び出し側が既定値へ読み替えること (accel_limit と同じ約束)
  float brake_torque_nm;
  // 駆動トルク直接指令 [Nm] (負=後退方向)。brake_torque と違い 0 は素直に「トルク0」を
  // 意味する (未指定への読み替えはしない)。flags の torque_mode が立っているときのみ使う
  float target_torque_nm;
  uint8_t light_mode;  // RasLightMode (flags bit3-4 をデコードしたもの)
  uint8_t seq;
  uint32_t rx_time_us;
} RasCommand;

// 送信するテレメトリ。呼び出し側は物理量のまま埋める (量子化は RasLink が行う)
typedef struct {
  // このスナップショットを取った時刻 (Micros())。TELEMETRY の t_us にはこれを載せる。
  // 以前は送信時の Micros() を載せていたが、スナップショットは Telemetry_Update の、送信は
  // RasLink_Update の別々の 50Hz タイマーで動いていたため、中身が最大 20ms 古いのに送信時刻が
  // 付き、上位の時刻合わせ (制御遅延の同定など) が最大 20ms ずれていた
  uint32_t t_us;
  uint32_t flags;  // RAS_FLAG_*

  float speed_m_s;         // 車体中心線方向の車速 (前輪速を舵角で射影したもの)
  float yaw_rate_rad_s;    // 左旋回が正
  float steer_actual_rad;  // 路面舵角
  float steer_cmd_rad;     // 直近に指令した路面舵角

  // 車輪周速 [FL, FR, RL, RR]。4輪ともローパス後の値を渡すこと (帯域およそ 1.6Hz)。
  // 前輪はアナログ絶対角の微分なので、生値は静止中でも ±12m/s 振れて使い物にならない。
  // 低速域では分解能が足りないため、輪ごとの走行距離が要る用途では odom_dist の差分を使うこと
  float wheel_speed_m_s[4];
  int32_t odom_dist_0p1mm[2];   // 前輪の累積走行距離 [FL, FR] (0.1mm 単位)

  float accel_m_s2[3];  // 機体座標系の加速度 [x, y, z] ★重力成分を含む
  float pitch_rad;
  float roll_rad;

  float motor_current_a[3];  // q軸電流 [RL, RR, ST] 制動時は負
  float torque_cmd_nm[2];    // TC 適用後の最終指令トルク [RL, RR]

  // TCのチューニング用。slip は正=空転・負=ロック傾向。tc_limit は
  // DRIVE_MAX_TORQUE_NM が「制限なし」、それより小さければ介入中を意味する
  float slip[2];         // 後輪スリップ率 [RL, RR] (無次元)
  float tc_limit_nm[2];  // TCが動的に決めたトルク上限 [RL, RR]
  // --- 制御の介入を上位で見るため (src/comm/telemetry.c) ---
  float torque_req_nm[2];        // スリップ制限が絞る前に掛けたかったトルク [RL, RR] (制動は負)
  float abs_limit_nm;            // ABS が決めた制動トルクの上限
  float tv_ratio;                // TV の配分の比率 (左旋回で正 = 右輪が多い)
  float tv_moment_nm;            // TV が要求した左右差のヨーモーメント換算 (左旋回が正)

  uint8_t temp_c[4];       // [MD後左, MD後右, MDステア, STM32内蔵]
  float batt_voltage_v[2];  // [駆動系, シグナル系]
  float batt_current_a[2];  // [駆動系, シグナル系]
  float us_distance_m[2];   // [前方, 後方] 計測不能は負値を渡すこと (0 として送る)

  uint8_t md_status[3];  // RAS_MD_STATUS_* [RL, RR, ST]

  // --- 車速のカルマンフィルタの診断 (v0.24 で追加、v0.25 で前輪 LPF を廃止。src/control/speed_kf.h)。speed_m_s がこのフィルタの車速 ---
  float accel_kf_m_s2;          // カルマンフィルタの加速度
  float accel_bias_kf_m_s2;     // カルマンフィルタの加速度計バイアス (重力の傾き成分を含む)
  uint16_t kf_gated_count;      // 測定を弱めた累積回数 (16bit で巻き戻る。前輪のロック・衝撃の目安)
  // 前回のスナップショットからの一貫性の診断 (v0.26。src/control/speed_kf.h)。
  // NIS (平均 z²。前提が合っていれば 1) は静止で 1e-3 まで小さくなり、そのまま量子化すると潰れるので、
  // 平方根 (z の RMS) と、隣り合う残差の相関係数 ρ = (平均 z_k z_{k-1}) / NIS で送る。0 = その間に測定が無かった
  float kf_z_rms_dist;          // 走行距離の残差の z の RMS (= √NIS)。1 が理想
  float kf_z_rms_accel;         // 加速度の残差の z の RMS
  float kf_rho_dist;            // 走行距離の残差の隣り合う相関係数 (-1〜1)。0 が理想
  float kf_rho_accel;           // 加速度の残差の相関係数
  // 前輪エンコーダの絶対角 [rad、0〜2π。取付向きの符号のまま = 速度の符号を直す前]。[FL, FR] (v0.26)。
  // 周期誤差 (角度の関数) の同定に使う (上位の tools/sysid)。Encoder_Update が読んだ最新の値
  float front_angle_rad[2];
} RasTelemetry;

// 送信キューの優先度 (小さいほど優先)
typedef enum {
  RAS_TXQ_PONG = 0,
  RAS_TXQ_TELEMETRY,
  RAS_TXQ_LIDAR,
  RAS_TXQ_INFO,  // CONFIG_ACK / VERSION / STATS
  RAS_TXQ_LOG,
  RAS_TXQ_NUM,
} RasTxPriority;

typedef struct {
  uint8_t* buf;
  uint16_t size;
  uint16_t head;
  uint16_t tail;
  uint32_t drop_count;
} RasTxQueue;

typedef struct {
  Serial* serial;

  // --- 送信 ---
  RasTxQueue tx_queue[RAS_TXQ_NUM];
  uint8_t tx_pong_storage[64];
  uint8_t tx_telemetry_storage[256];
  uint8_t tx_lidar_storage[512];
  // 上位は接続のたびに調整パラメータ (control_params.h の表) をまとめて CONFIG_SET してくる。
  // CONFIG_ACK は1つ 14 バイトで、256 だと約 18 個で溢れていた (上位の再送で回復はする)。
  // 調整パラメータと校正値で約 50 項目あるので、まとめて届いても溢れないだけ持つ
  uint8_t tx_info_storage[1024];
  uint8_t tx_log_storage[512];
  uint8_t tx_frame[RAS_FRAME_MAX];  // 送信中のフレーム (DMA が読むので完了まで触らない)
  uint8_t build_buf[RAS_FRAME_MAX];
  uint8_t tx_seq;

  // --- 受信 ---
  // 読み進めているフレームの生バイト (先頭は SYNC1)。ヘッダ不正・CRC 不一致で捨てるとき、
  // 消費済みのバイトの中に次のフレームの SYNC があれば拾い直すために持つ
  uint8_t rx_raw[RAS_RX_RAW_MAX];
  uint8_t rx_raw_len;
  uint8_t rx_state;
  uint8_t rx_type;
  uint8_t rx_seq;
  uint8_t rx_len;
  uint16_t rx_index;
  uint16_t rx_crc;
  uint16_t rx_crc_received;
  bool rx_seq_valid;
  uint8_t rx_last_seq;
  uint8_t rx_payload[RAS_PAYLOAD_MAX];

  // UART の IDLE 割り込みが記録する「最後にバースト受信が途切れた時刻と位置」。
  // PING の最終バイト到着時刻 (時刻同期の T2) をメインループの周期に依存せず取るために使う
  volatile uint32_t rx_idle_time_us;
  volatile uint16_t rx_idle_head;

  // --- 状態 ---
  RasCommand command;
  bool has_command;
  uint32_t last_command_us;
  // COMMAND が途絶していないか。途絶を一度検出したら、次の COMMAND を受けるまで偽のまま保つ
  // (経過時間だけで判定すると、Micros() が一周する約71.6分後に古い COMMAND が 100ms 生き返る)
  bool command_alive;
  RasConfig config;
  RasTelemetry telemetry;

  bool telemetry_ready;  // RasLink_SetTelemetry で新しいスナップショットが届き、まだ送っていない
  uint32_t last_stats_us;
  uint32_t last_version_us;
  uint8_t version_burst_left;

  // --- 統計 (STATS で送る累積値) ---
  // 増分ではなく累積で送る。増分だと STATS が1つ落ちたときにその区間のエラー件数が
  // 永久に失われるが、累積なら次のパケットに正しい総数が入っているためロスに強い。
  // u32 は 1000件/s でも 49日かかるため実質ラップしない。
  uint32_t rx_frame_ok;
  uint32_t rx_crc_error;
  uint32_t rx_len_error;
  uint32_t rx_unknown_type;
  uint32_t rx_packet_loss;  // SEQ の欠番から数えたロス数 (STATS には載せず下位側の確認用)
  uint32_t md_rx_count[3];
  uint32_t md_rx_error[3];
} RasLink;

/**
 * @brief Raspberry Pi との通信を初期化する。serial には Serial_Init 済みの USART1 を渡すこと。
 * UART の TC 割り込みが無いと1回しか送信できないため、この関数の中で USART1 の NVIC を
 * 有効化し、IDLE 割り込みも有効にする。
 */
void RasLink_Init(RasLink* obj, Serial* serial);

/**
 * @brief 受信済みのバイト列を解釈し、COMMAND / CONFIG_SET などを取り込む。
 * 制御周期ごとに、指令を使う Vehicle_Update より前で呼ぶこと (後に呼ぶと、届いた COMMAND が
 * 効くのが1周期遅れる)。応答 (PONG / CONFIG_ACK など) は送信キューへ積むだけで、送り出すのは
 * RasLink_Update。
 */
void RasLink_Receive(RasLink* obj);

/**
 * @brief 定期送信と送信キューの吐き出しを1周期分行う。
 * PONG の応答性と時刻同期の精度が呼び出し周期で決まるため、制御周期ごと (2kHz 目安) に呼ぶこと。
 * 受信は RasLink_Receive。
 */
void RasLink_Update(RasLink* obj);

/**
 * @brief 次の TELEMETRY で送る内容を差し替える。制御周期ごとに最新値で呼ぶこと
 * (実際の送信は RAS_TELEMETRY_INTERVAL_US ごとに間引かれる)。
 */
void RasLink_SetTelemetry(RasLink* obj, const RasTelemetry* telemetry);

/**
 * @brief 直近に受信した COMMAND を取得する。一度も受信していない場合の内容は不定なので、
 * RasLink_HasCommand() が真であることを確認してから使うこと。
 */
const RasCommand* RasLink_GetCommand(const RasLink* obj);

/**
 * @brief 起動後に一度でも COMMAND を受信したかを取得する。
 * 「一度も繋がっていない」のか「繋がった後に途絶した」のかを区別するために使う
 * (RasLink_IsCommandAlive() と組み合わせて、後者のみ RAS_FLAG_UART_TIMEOUT を立てるなど)。
 */
bool RasLink_HasCommand(const RasLink* obj);

/**
 * @brief COMMAND が途絶せず生きているか (最後の受信から RAS_COMMAND_TIMEOUT_US 以内か) を取得する。
 * 偽になったら自動ブレーキへ落とすこと。
 */
bool RasLink_IsCommandAlive(const RasLink* obj);

/**
 * @brief LIMITS を送り直す。可動範囲 (max_steer_rad) が変わったとき (ステアのリンクの換算を適用したとき)
 * に呼ぶ。上位が古い上限で指令を丸め続けないようにするため。
 */
void RasLink_SendLimits(RasLink* obj);

/**
 * @brief 現在の設定値を取得する。CONFIG_SET / CONFIG_GET で更新される。
 */
const RasConfig* RasLink_GetConfig(const RasLink* obj);

/**
 * @brief STATS に載せる MD 通信の累積カウンタを差し替える ([RL, RR, ST] の順)。
 * 上下どちらの配線が悪いかを Pi 側で切り分けるために送る。制御周期ごとに呼んでよい。
 */
void RasLink_SetMdStats(RasLink* obj, const uint32_t* rx_count, const uint32_t* rx_error);

/**
 * @brief LiDAR の1セクタ (30点) を送信キューへ積む。
 * intensity は RAS_LIDAR_FORMAT_INTENSITY のときのみ参照する (それ以外は NULL 可)。
 * 送信される TYPE は設定 (RAS_PARAM_LIDAR_FORMAT) で 0x01 / 0x05 / 0x09 に切り替わる。
 */
void RasLink_PublishLidarSector(RasLink* obj, uint8_t sector_idx, uint32_t t_start_us,
                                uint16_t duration_us, uint16_t rot_speed_dps,
                                const uint16_t* dist_mm, const uint8_t* intensity);

/**
 * @brief デバッグ文字列を送る (最低優先度。帯域が逼迫すると破棄される)。
 */
void RasLink_Log(RasLink* obj, RasLogSeverity severity, const char* text);

/**
 * @brief USART1 の割り込みから呼ぶこと。IDLE を検出して受信バーストの終了時刻を記録する。
 * (ras_link.c が USART1_IRQHandler を定義しているため、通常は呼び出し不要)
 */
void RasLink_OnUartIdle(RasLink* obj);

#endif  // RAS_LINK_H_
