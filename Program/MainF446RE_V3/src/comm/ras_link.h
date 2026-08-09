#ifndef RAS_LINK_H_
#define RAS_LINK_H_

#include <stdbool.h>
#include <stdint.h>

#include "serial.h"
#include "timer.h"

// ===========================================================================
// Raspberry Pi (上位) との UART 通信。プロトコル仕様は
// docs/pi_uart_protocol_v0.4_request.md (v0.4) と、そこからの差分を書いた
// docs/pi_uart_protocol_v0.5_delta.md (v0.5) に対応する。
//
// 物理層: USART1, 250000bps 8N1 (分周誤差0%)。
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

#define RAS_PROTOCOL_VERSION 0x0005u
#define RAS_FIRMWARE_ID 0x4D463303u  // "MF3" + 版数。Pi 側のログで機体を識別するための任意値

#define RAS_SYNC1 0xAAu
#define RAS_SYNC2 0x55u
#define RAS_FRAME_OVERHEAD 7  // SYNC(2) + TYPE + SEQ + LEN + CRC16(2)
#define RAS_PAYLOAD_MAX 255
#define RAS_FRAME_MAX (RAS_FRAME_OVERHEAD + RAS_PAYLOAD_MAX)

// LiDAR 1セクタあたりの点数 (1周360点を12分割)
#define RAS_LIDAR_POINTS_PER_SECTOR 30
#define RAS_LIDAR_SECTOR_NUM 12

// COMMAND がこの時間途絶したら自動ブレーキへ落とす [us]
#define RAS_COMMAND_TIMEOUT_US 100000u

// 定期送信の周期 [us]
#define RAS_TELEMETRY_INTERVAL_US 20000u   // 50Hz
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
  RAS_TYPE_COMMAND = 0x10,
  RAS_TYPE_CONFIG_SET = 0x11,
  RAS_TYPE_PING = 0x12,
  RAS_TYPE_CONFIG_GET = 0x13,
  RAS_TYPE_VERSION_REQ = 0x14,
} RasPacketType;

// COMMAND.mode / TELEMETRY.flags bit0-1
typedef enum {
  RAS_MODE_DISARM = 0,
  RAS_MODE_MANUAL = 1,
  RAS_MODE_AUTO = 2,
  RAS_MODE_RESERVED = 3,  // 旧 CALIB。v0.4 で廃止したため受信しても現在のモードを維持する
} RasMode;

// COMMAND.flags
#define RAS_CMD_FLAG_ARM (1u << 0)
#define RAS_CMD_FLAG_BRAKE (1u << 1)
#define RAS_CMD_FLAG_HORN (1u << 2)
// bit3-4 は前照灯モード (RasLightMode)。v0.4 の1ビット (RAS_CMD_FLAG_LIGHT) では
// 「前後とも消灯」を表現できなかったため 2ビットへ拡張した
#define RAS_CMD_LIGHT_MODE_SHIFT 3
#define RAS_CMD_LIGHT_MODE_MASK (0x03u << RAS_CMD_LIGHT_MODE_SHIFT)
#define RAS_CMD_FLAG_PASSING (1u << 5)

// COMMAND.flags bit3-4 (前照灯モード)
typedef enum {
  RAS_LIGHT_OFF = 0,      // 前照灯・尾灯とも消灯
  RAS_LIGHT_DAYTIME = 1,  // デイライト (前照灯減光 + 尾灯薄点灯)
  RAS_LIGHT_NORMAL = 2,   // 通常点灯 (前照灯全光量 + 尾灯薄点灯)
  // 3 は予約。受信したら被視認性の高い NORMAL として扱う
} RasLightMode;

// TELEMETRY.flags
#define RAS_FLAG_MODE_MASK 0x0003u
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

// TELEMETRY.md_status[i]
#define RAS_MD_STATUS_RUNNING (1u << 0)
#define RAS_MD_STATUS_VOLTAGE_OUT_OF_RANGE (1u << 1)
#define RAS_MD_STATUS_OVERHEAT (1u << 2)
#define RAS_MD_STATUS_OVERCURRENT (1u << 3)
#define RAS_MD_STATUS_COMM_OK (1u << 4)
#define RAS_MD_STATUS_LIMIT_SYNCED (1u << 5)

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
// TC/TV/速度PIゲインの実行時変更 (0x0010,0x0011,0x0020,0x0021,0x0030,0x0031) は
// Drive 側が定数で持っているため未対応で、受信すると RAS_CONFIG_UNKNOWN_PARAM を返す。
// 「OK を返すが何も変わらない」より、対応していないことを Pi 側に伝える方が安全。
typedef enum {
  RAS_PARAM_MAX_SPEED = 0x0001,     // [m/s]
  RAS_PARAM_MAX_ACCEL = 0x0002,     // [m/s^2]
  RAS_PARAM_MAX_STEER = 0x0003,     // [rad] 路面舵角
  RAS_PARAM_LIDAR_FORMAT = 0x0040,  // RasLidarFormat
} RasParamId;

typedef enum {
  RAS_LIDAR_FORMAT_STANDARD = 0,   // 0x01 LIDAR_SECTOR   (u16 mm)
  RAS_LIDAR_FORMAT_INTENSITY = 1,  // 0x05 LIDAR_SECTOR_I (u16 mm + 強度)
  RAS_LIDAR_FORMAT_COMPACT = 2,    // 0x09 LIDAR_SECTOR_C (u8 2cm/LSB)
} RasLidarFormat;

typedef struct {
  float max_speed_m_s;
  float max_accel_m_s2;
  float max_steer_rad;
  uint8_t lidar_format;
} RasConfig;

// 受信した COMMAND をデコードしたもの
typedef struct {
  uint8_t mode;   // RasMode
  uint8_t flags;  // RAS_CMD_FLAG_*
  float target_speed_m_s;
  float target_steer_rad;  // 路面舵角 (反時計回り = 左旋回が正)
  float accel_limit_m_s2;
  float steer_rate_limit_rad_s;
  // 制動トルク [Nm] (常に正)。0 は「無制動」ではなく「未指定」を意味するので、
  // 呼び出し側が既定値へ読み替えること (accel_limit と同じ約束)
  float brake_torque_nm;
  uint8_t light_mode;  // RasLightMode (flags bit3-4 をデコードしたもの)
  uint8_t seq;
  uint32_t rx_time_us;
} RasCommand;

// 送信するテレメトリ。呼び出し側は物理量のまま埋める (量子化は RasLink が行う)
typedef struct {
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

  uint8_t temp_c[4];       // [MD後左, MD後右, MDステア, STM32内蔵]
  float batt_voltage_v[2];  // [駆動系, シグナル系]
  float batt_current_a[2];  // [駆動系, シグナル系]
  float us_distance_m[2];   // [前方, 後方] 計測不能は負値を渡すこと (0 として送る)

  uint8_t md_status[3];  // RAS_MD_STATUS_* [RL, RR, ST]
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
  uint8_t tx_info_storage[256];
  uint8_t tx_log_storage[512];
  uint8_t tx_frame[RAS_FRAME_MAX];  // 送信中のフレーム (DMA が読むので完了まで触らない)
  uint8_t build_buf[RAS_FRAME_MAX];
  uint8_t tx_seq;

  // --- 受信 ---
  uint8_t rx_state;
  uint8_t rx_type;
  uint8_t rx_seq;
  uint8_t rx_len;
  uint16_t rx_index;
  uint16_t rx_crc;
  uint16_t rx_crc_received;
  bool rx_discard;
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
  RasConfig config;
  RasTelemetry telemetry;

  uint32_t last_telemetry_us;
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
 * @brief 受信処理・定期送信・送信キューの吐き出しを1周期分行う。
 * PONG の応答性と時刻同期の精度が呼び出し周期で決まるため、制御周期ごと (2kHz 目安) に呼ぶこと。
 */
void RasLink_Update(RasLink* obj);

/**
 * @brief 次の TELEMETRY で送る内容を差し替える。制御周期ごとに最新値で呼ぶこと
 * (実際の送信は 50Hz に間引かれる)。
 */
void RasLink_SetTelemetry(RasLink* obj, const RasTelemetry* telemetry);

/**
 * @brief 直近に受信した COMMAND を取得する。一度も受信していない場合の内容は不定なので、
 * RasLink_HasCommand() が真であることを確認してから使うこと。
 */
const RasCommand* RasLink_GetCommand(const RasLink* obj);

/**
 * @brief 起動後に一度でも COMMAND を受信したかを取得する。
 * 偽の間は上位が繋がっていないため、下位単独の動作 (走行テスト等) をしてよい。
 */
bool RasLink_HasCommand(const RasLink* obj);

/**
 * @brief COMMAND が途絶せず生きているか (最後の受信から RAS_COMMAND_TIMEOUT_US 以内か) を取得する。
 * 偽になったら自動ブレーキへ落とすこと。
 */
bool RasLink_IsCommandAlive(const RasLink* obj);

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
