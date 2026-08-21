#include "ras_link.h"

#include <string.h>

#include "crc16.h"
// 設定値のクランプ範囲を車両の物理的な限界に合わせるため、車速・舵角の上限だけ参照する。
// 制御そのものには関与しない
#include "drive.h"
#include "steering.h"

// PONG の t_pong_tx_us は「実際に1バイト目が線に出る直前」に埋めないと、送信キューでの
// 待ち時間がそのまま時刻同期のオフセット推定にバイアスとして乗る (LiDAR セクタ1つで
// 最大3ms)。キューに積む時点では 0 を入れておき、送出直前にこのフラグを見て書き換える
#define RAS_TXFLAG_PATCH_PONG_TIME 0x01u

// PONG ペイロード内の t_pong_tx_us の位置 → フレーム先頭からは +5 (ヘッダ長)
#define RAS_PONG_TX_TIME_OFFSET 8

// 各パケットのペイロード長
#define RAS_LEN_TELEMETRY 66
#define RAS_LEN_PONG 12
#define RAS_LEN_VERSION 10
#define RAS_LEN_STATS 48
#define RAS_LEN_CONFIG_ACK 7
#define RAS_LEN_LIDAR_SECTOR 69
#define RAS_LEN_LIDAR_SECTOR_I 99
#define RAS_LEN_LIDAR_SECTOR_C 39

// LOG のペイロード上限 (severity 1バイト + 本文)。プロトコル上は255まで載るが、
// スタックが 1KB しかないためここで頭打ちにする
#define RAS_LOG_MAX_LEN 128

// Pi → STM32 の各 TYPE が取るべきペイロード長 (これ以外は破棄して len_error に数える)
#define RAS_LEN_COMMAND 14
#define RAS_LEN_CONFIG_SET 6
#define RAS_LEN_PING 4
#define RAS_LEN_CONFIG_GET 2
#define RAS_LEN_VERSION_REQ 0

// 設定値の既定値と可動範囲
#define RAS_DEFAULT_MAX_SPEED_M_S 3.0f
#define RAS_DEFAULT_MAX_ACCEL_M_S2 3.0f
#define RAS_LIMIT_MAX_SPEED_M_S DRIVE_MAX_SPEED_M_S
#define RAS_LIMIT_MAX_ACCEL_M_S2 20.0f

typedef enum {
  RX_SYNC1 = 0,
  RX_SYNC2,
  RX_TYPE,
  RX_SEQ,
  RX_LEN,
  RX_PAYLOAD,
  RX_CRC_LO,
  RX_CRC_HI,
} RasRxState;

// USART1 の IDLE 割り込みからインスタンスを引くための参照。上位との通信は1系統しかない
static RasLink* g_link;

// ---------------------------------------------------------------------------
// バイト列の読み書き (すべてリトルエンディアン)
// ---------------------------------------------------------------------------

static void PutU8(uint8_t* buf, uint16_t* pos, uint8_t value) {
  buf[(*pos)++] = value;
}

static void PutU16(uint8_t* buf, uint16_t* pos, uint16_t value) {
  buf[(*pos)++] = (uint8_t)(value & 0xFF);
  buf[(*pos)++] = (uint8_t)(value >> 8);
}

static void PutU32(uint8_t* buf, uint16_t* pos, uint32_t value) {
  buf[(*pos)++] = (uint8_t)(value & 0xFF);
  buf[(*pos)++] = (uint8_t)((value >> 8) & 0xFF);
  buf[(*pos)++] = (uint8_t)((value >> 16) & 0xFF);
  buf[(*pos)++] = (uint8_t)((value >> 24) & 0xFF);
}

static void PutI16(uint8_t* buf, uint16_t* pos, int16_t value) {
  PutU16(buf, pos, (uint16_t)value);
}

static void PutI32(uint8_t* buf, uint16_t* pos, int32_t value) {
  PutU32(buf, pos, (uint32_t)value);
}

static uint16_t GetU16(const uint8_t* buf) {
  return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static int16_t GetI16(const uint8_t* buf) {
  return (int16_t)GetU16(buf);
}

static uint32_t GetU32(const uint8_t* buf) {
  return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}

static float GetF32(const uint8_t* buf) {
  float value;
  // 型を跨いだ読み替えを memcpy で行う。ポインタのキャストはアライメント違反になりうる
  // (ペイロードはフレーム先頭から5バイト目に始まるため4バイト境界に乗らない)
  memcpy(&value, buf, sizeof(value));
  return value;
}

// ---------------------------------------------------------------------------
// 物理量 → 固定小数点への量子化。値域外は飽和させる (ラップさせると符号が反転して
// 制御量の意味が壊れるため、飽和の方が上位から見て解釈しやすい)
// ---------------------------------------------------------------------------

static bool IsFinite(float value) {
  return (value == value) && (value < 3.0e38f) && (value > -3.0e38f);
}

static int16_t QuantizeI16(float value, float scale) {
  if (!IsFinite(value)) return 0;
  float raw = value / scale;
  if (raw >= 32767.0f) return 32767;
  if (raw <= -32768.0f) return -32768;
  return (int16_t)(raw >= 0.0f ? raw + 0.5f : raw - 0.5f);
}

static uint8_t QuantizeU8(float value, float scale) {
  if (!IsFinite(value) || value <= 0.0f) return 0;
  float raw = value / scale;
  if (raw >= 255.0f) return 255;
  return (uint8_t)(raw + 0.5f);
}

static float Clampf(float value, float min, float max) {
  if (!IsFinite(value)) return min;
  if (value < min) return min;
  if (value > max) return max;
  return value;
}

// ---------------------------------------------------------------------------
// 送信キュー
//
// 1レコード = [len_lo][len_hi][flags][フレーム本体]。生産者・消費者ともメインループ
// (RasLink_Update) なので排他は不要
// ---------------------------------------------------------------------------

static void QueueInit(RasTxQueue* queue, uint8_t* buf, uint16_t size) {
  queue->buf = buf;
  queue->size = size;
  queue->head = 0;
  queue->tail = 0;
  queue->drop_count = 0;
}

static uint16_t QueueUsed(const RasTxQueue* queue) {
  return (uint16_t)((queue->head + queue->size - queue->tail) % queue->size);
}

static bool QueueIsEmpty(const RasTxQueue* queue) {
  return queue->head == queue->tail;
}

static bool QueuePush(RasTxQueue* queue, const uint8_t* frame, uint16_t len, uint8_t flags) {
  uint16_t free_bytes = (uint16_t)(queue->size - 1 - QueueUsed(queue));
  if (free_bytes < len + 3) {
    queue->drop_count++;
    return false;
  }
  uint8_t header[3] = {(uint8_t)(len & 0xFF), (uint8_t)(len >> 8), flags};
  for (uint8_t i = 0; i < 3; i++) {
    queue->buf[queue->head] = header[i];
    queue->head = (uint16_t)((queue->head + 1) % queue->size);
  }
  for (uint16_t i = 0; i < len; i++) {
    queue->buf[queue->head] = frame[i];
    queue->head = (uint16_t)((queue->head + 1) % queue->size);
  }
  return true;
}

static uint16_t QueuePop(RasTxQueue* queue, uint8_t* out, uint8_t* flags) {
  if (QueueIsEmpty(queue)) return 0;
  uint8_t header[3];
  for (uint8_t i = 0; i < 3; i++) {
    header[i] = queue->buf[queue->tail];
    queue->tail = (uint16_t)((queue->tail + 1) % queue->size);
  }
  uint16_t len = (uint16_t)header[0] | ((uint16_t)header[1] << 8);
  *flags = header[2];
  for (uint16_t i = 0; i < len; i++) {
    out[i] = queue->buf[queue->tail];
    queue->tail = (uint16_t)((queue->tail + 1) % queue->size);
  }
  return len;
}

// ---------------------------------------------------------------------------
// フレームの組み立てと送出
// ---------------------------------------------------------------------------

// CRC の対象は TYPE から PAYLOAD 末尾まで (= LEN + 3 バイト)
static void WriteFrameCrc(uint8_t* frame, uint8_t len) {
  uint16_t crc = Crc16(&frame[2], (uint16_t)len + 3);
  frame[5 + len] = (uint8_t)(crc & 0xFF);
  frame[6 + len] = (uint8_t)(crc >> 8);
}

static bool SendFrame(RasLink* obj, RasTxPriority priority, uint8_t type,
                      const uint8_t* payload, uint8_t len, uint8_t tx_flags) {
  uint8_t* frame = obj->build_buf;
  frame[0] = RAS_SYNC1;
  frame[1] = RAS_SYNC2;
  frame[2] = type;
  frame[3] = obj->tx_seq++;
  frame[4] = len;
  if (len > 0) memcpy(&frame[5], payload, len);
  WriteFrameCrc(frame, len);
  return QueuePush(&obj->tx_queue[priority], frame, (uint16_t)(len + RAS_FRAME_OVERHEAD), tx_flags);
}

// 優先度の高いキューから1フレームだけ取り出して送出する。DMA が動いている間は何もしない
static void PumpTx(RasLink* obj) {
  if (Serial_IsTxBusy(obj->serial)) return;

  uint16_t len = 0;
  uint8_t flags = 0;
  for (int i = 0; i < RAS_TXQ_NUM; i++) {
    len = QueuePop(&obj->tx_queue[i], obj->tx_frame, &flags);
    if (len > 0) break;
  }
  if (len == 0) return;

  if (flags & RAS_TXFLAG_PATCH_PONG_TIME) {
    uint16_t pos = 5 + RAS_PONG_TX_TIME_OFFSET;
    PutU32(obj->tx_frame, &pos, Micros());
    WriteFrameCrc(obj->tx_frame, obj->tx_frame[4]);
  }

  Serial_WriteAsync(obj->serial, obj->tx_frame, len);
}

// ---------------------------------------------------------------------------
// 送信パケットの組み立て
// ---------------------------------------------------------------------------

static void SendTelemetry(RasLink* obj) {
  const RasTelemetry* t = &obj->telemetry;
  uint8_t p[RAS_LEN_TELEMETRY];
  uint16_t pos = 0;

  PutU32(p, &pos, Micros());
  PutU32(p, &pos, t->flags);
  PutI16(p, &pos, QuantizeI16(t->speed_m_s, 0.001f));
  PutI16(p, &pos, QuantizeI16(t->yaw_rate_rad_s, 0.001f));
  PutI16(p, &pos, QuantizeI16(t->steer_actual_rad, 0.0001f));
  PutI16(p, &pos, QuantizeI16(t->steer_cmd_rad, 0.0001f));
  for (int i = 0; i < 4; i++) PutI16(p, &pos, QuantizeI16(t->wheel_speed_m_s[i], 0.001f));
  for (int i = 0; i < 2; i++) PutI32(p, &pos, t->odom_dist_0p1mm[i]);
  for (int i = 0; i < 3; i++) PutI16(p, &pos, QuantizeI16(t->accel_m_s2[i], 0.001f));
  PutI16(p, &pos, QuantizeI16(t->pitch_rad, 0.0001f));
  PutI16(p, &pos, QuantizeI16(t->roll_rad, 0.0001f));
  for (int i = 0; i < 3; i++) PutI16(p, &pos, QuantizeI16(t->motor_current_a[i], 0.001f));
  for (int i = 0; i < 2; i++) PutI16(p, &pos, QuantizeI16(t->torque_cmd_nm[i], 0.0001f));
  for (int i = 0; i < 4; i++) PutU8(p, &pos, t->temp_c[i]);
  PutU8(p, &pos, QuantizeU8(t->batt_voltage_v[0], 0.05f));
  PutU8(p, &pos, QuantizeU8(t->batt_voltage_v[1], 0.05f));
  PutU8(p, &pos, QuantizeU8(t->batt_current_a[0], 0.05f));
  // シグナル系は待機電流が数百mAと小さいため、駆動系より細かいスケールを使う
  PutU8(p, &pos, QuantizeU8(t->batt_current_a[1], 0.02f));
  // 超音波は最小測距が2cmあり「距離0」が physically 出ないため、0 を無効値に使える
  PutU8(p, &pos, QuantizeU8(t->us_distance_m[0], 0.02f));
  PutU8(p, &pos, QuantizeU8(t->us_distance_m[1], 0.02f));
  for (int i = 0; i < 3; i++) PutU8(p, &pos, t->md_status[i]);
  PutU8(p, &pos, obj->has_command ? obj->command.seq : 0);

  SendFrame(obj, RAS_TXQ_TELEMETRY, RAS_TYPE_TELEMETRY, p, RAS_LEN_TELEMETRY, 0);
}

static void SendVersion(RasLink* obj) {
  uint8_t p[RAS_LEN_VERSION];
  uint16_t pos = 0;
  PutU16(p, &pos, RAS_PROTOCOL_VERSION);
  PutU32(p, &pos, RAS_FIRMWARE_ID);
  // ビルド時刻は持っていないため0。必要になったらビルドシステムから埋め込むこと
  PutU32(p, &pos, 0);
  SendFrame(obj, RAS_TXQ_INFO, RAS_TYPE_VERSION, p, RAS_LEN_VERSION, 0);
}

static void SendStats(RasLink* obj) {
  uint8_t p[RAS_LEN_STATS];
  uint16_t pos = 0;
  uint32_t tx_drop = 0;
  for (int i = 0; i < RAS_TXQ_NUM; i++) tx_drop += obj->tx_queue[i].drop_count;

  PutU32(p, &pos, Micros());
  PutU32(p, &pos, obj->rx_frame_ok);
  PutU32(p, &pos, obj->rx_crc_error);
  PutU32(p, &pos, obj->rx_len_error);
  PutU32(p, &pos, obj->rx_unknown_type);
  PutU32(p, &pos, tx_drop);
  for (int i = 0; i < 3; i++) PutU32(p, &pos, obj->md_rx_count[i]);
  for (int i = 0; i < 3; i++) PutU32(p, &pos, obj->md_rx_error[i]);

  SendFrame(obj, RAS_TXQ_INFO, RAS_TYPE_STATS, p, RAS_LEN_STATS, 0);
}

static void SendConfigAck(RasLink* obj, uint16_t param_id, float applied, uint8_t result) {
  uint8_t p[RAS_LEN_CONFIG_ACK];
  uint16_t pos = 0;
  PutU16(p, &pos, param_id);
  memcpy(&p[pos], &applied, sizeof(applied));
  pos += sizeof(applied);
  PutU8(p, &pos, result);
  SendFrame(obj, RAS_TXQ_INFO, RAS_TYPE_CONFIG_ACK, p, RAS_LEN_CONFIG_ACK, 0);
}

void RasLink_PublishLidarSector(RasLink* obj, uint8_t sector_idx, uint32_t t_start_us,
                                uint16_t duration_us, uint16_t rot_speed_dps,
                                const uint16_t* dist_mm, const uint8_t* intensity) {
  uint8_t p[RAS_LEN_LIDAR_SECTOR_I];
  uint16_t pos = 0;
  PutU8(p, &pos, sector_idx);
  PutU32(p, &pos, t_start_us);
  PutU16(p, &pos, duration_us);
  PutU16(p, &pos, rot_speed_dps);

  uint8_t type = RAS_TYPE_LIDAR_SECTOR;
  uint8_t len = RAS_LEN_LIDAR_SECTOR;

  if (obj->config.lidar_format == RAS_LIDAR_FORMAT_COMPACT) {
    type = RAS_TYPE_LIDAR_SECTOR_C;
    len = RAS_LEN_LIDAR_SECTOR_C;
    for (int i = 0; i < RAS_LIDAR_POINTS_PER_SECTOR; i++) {
      uint16_t mm = dist_mm[i];
      // 0 は無効、255 は「5.10m 以上 (飽和)」。飽和値を実測点として地図に打たないこと
      uint8_t value;
      if (mm == 0) {
        value = 0;
      } else if (mm >= 5100) {
        value = 255;
      } else {
        value = (uint8_t)(mm / 20);
        if (value == 0) value = 1;  // 2cm未満を無効値に潰さない
      }
      PutU8(p, &pos, value);
    }
  } else {
    for (int i = 0; i < RAS_LIDAR_POINTS_PER_SECTOR; i++) PutU16(p, &pos, dist_mm[i]);
    if (obj->config.lidar_format == RAS_LIDAR_FORMAT_INTENSITY && intensity != NULL) {
      type = RAS_TYPE_LIDAR_SECTOR_I;
      len = RAS_LEN_LIDAR_SECTOR_I;
      for (int i = 0; i < RAS_LIDAR_POINTS_PER_SECTOR; i++) PutU8(p, &pos, intensity[i]);
    }
  }

  SendFrame(obj, RAS_TXQ_LIDAR, type, p, len, 0);
}

void RasLink_Log(RasLink* obj, RasLogSeverity severity, const char* text) {
  uint8_t p[RAS_LOG_MAX_LEN];
  p[0] = (uint8_t)severity;
  uint16_t len = 1;
  while (text[len - 1] != '\0' && len < RAS_LOG_MAX_LEN) {
    p[len] = (uint8_t)text[len - 1];
    len++;
  }
  SendFrame(obj, RAS_TXQ_LOG, RAS_TYPE_LOG, p, (uint8_t)len, 0);
}

// ---------------------------------------------------------------------------
// 受信パケットの処理
// ---------------------------------------------------------------------------

static void HandleCommand(RasLink* obj, const uint8_t* p, uint32_t now_us) {
  obj->command.mode = p[0];
  obj->command.flags = p[1];
  obj->command.target_speed_m_s = (float)GetI16(&p[2]) * 0.001f;
  obj->command.target_steer_rad = (float)GetI16(&p[4]) * 0.0001f;
  obj->command.accel_limit_m_s2 = (float)GetU16(&p[6]) * 0.001f;
  obj->command.steer_rate_limit_rad_s = (float)GetU16(&p[8]) * 0.001f;
  obj->command.brake_torque_nm = (float)GetU16(&p[10]) * 0.0001f;
  obj->command.target_torque_nm = (float)GetI16(&p[12]) * 0.0001f;

  uint8_t light_mode = (uint8_t)((p[1] & RAS_CMD_LIGHT_MODE_MASK) >> RAS_CMD_LIGHT_MODE_SHIFT);
  // 予約値 (3) は最も明るい NORMAL に倒す。灯火は消えるより点く方が安全側
  obj->command.light_mode = (light_mode > RAS_LIGHT_NORMAL) ? RAS_LIGHT_NORMAL : light_mode;

  obj->command.seq = obj->rx_seq;
  obj->command.rx_time_us = now_us;
  obj->has_command = true;
  obj->last_command_us = now_us;
}

static void HandlePing(RasLink* obj, const uint8_t* p, uint32_t rx_time_us) {
  uint8_t payload[RAS_LEN_PONG];
  uint16_t pos = 0;
  PutU32(payload, &pos, GetU32(p));  // ping_id をそのままエコー
  PutU32(payload, &pos, rx_time_us);
  PutU32(payload, &pos, 0);  // t_pong_tx_us は送出直前に埋める
  SendFrame(obj, RAS_TXQ_PONG, RAS_TYPE_PONG, payload, RAS_LEN_PONG, RAS_TXFLAG_PATCH_PONG_TIME);
}

// 設定値の適用。範囲外はクランプしたうえで RAS_CONFIG_OUT_OF_RANGE を返し、
// applied には実際に適用された値を入れる (GUI が常に実機の真の値を表示できるようにするため)
static uint8_t ApplyConfig(RasLink* obj, uint16_t param_id, float value, float* applied) {
  float clamped;
  switch (param_id) {
    case RAS_PARAM_MAX_SPEED:
      clamped = Clampf(value, 0.0f, RAS_LIMIT_MAX_SPEED_M_S);
      obj->config.max_speed_m_s = clamped;
      break;
    case RAS_PARAM_MAX_ACCEL:
      clamped = Clampf(value, 0.01f, RAS_LIMIT_MAX_ACCEL_M_S2);
      obj->config.max_accel_m_s2 = clamped;
      break;
    case RAS_PARAM_MAX_STEER:
      clamped = Clampf(value, 0.0f, Steering_GetMaxRoadWheelAngleRad());
      obj->config.max_steer_rad = clamped;
      break;
    case RAS_PARAM_TC_ENABLE:
      obj->config.tc_enabled = (value != 0.0f);
      clamped = obj->config.tc_enabled ? 1.0f : 0.0f;
      break;
    case RAS_PARAM_TV_ENABLE:
      obj->config.tv_enabled = (value != 0.0f);
      clamped = obj->config.tv_enabled ? 1.0f : 0.0f;
      break;
    case RAS_PARAM_WHEEL_LIFT_GUARD_ENABLE:
      obj->config.wheel_lift_guard_enabled = (value != 0.0f);
      clamped = obj->config.wheel_lift_guard_enabled ? 1.0f : 0.0f;
      break;
    case RAS_PARAM_LIDAR_FORMAT:
      clamped = Clampf(value, 0.0f, (float)RAS_LIDAR_FORMAT_COMPACT);
      obj->config.lidar_format = (uint8_t)(clamped + 0.5f);
      clamped = (float)obj->config.lidar_format;
      break;
    default:
      *applied = 0.0f;
      return RAS_CONFIG_UNKNOWN_PARAM;
  }
  *applied = clamped;
  return (clamped == value) ? RAS_CONFIG_OK : RAS_CONFIG_OUT_OF_RANGE;
}

static float ReadConfig(const RasLink* obj, uint16_t param_id, bool* known) {
  *known = true;
  switch (param_id) {
    case RAS_PARAM_MAX_SPEED:
      return obj->config.max_speed_m_s;
    case RAS_PARAM_MAX_ACCEL:
      return obj->config.max_accel_m_s2;
    case RAS_PARAM_MAX_STEER:
      return obj->config.max_steer_rad;
    case RAS_PARAM_TC_ENABLE:
      return obj->config.tc_enabled ? 1.0f : 0.0f;
    case RAS_PARAM_TV_ENABLE:
      return obj->config.tv_enabled ? 1.0f : 0.0f;
    case RAS_PARAM_WHEEL_LIFT_GUARD_ENABLE:
      return obj->config.wheel_lift_guard_enabled ? 1.0f : 0.0f;
    case RAS_PARAM_LIDAR_FORMAT:
      return (float)obj->config.lidar_format;
    default:
      break;
  }
  *known = false;
  return 0.0f;
}

static void DispatchPacket(RasLink* obj, uint32_t frame_end_us) {
  const uint8_t* p = obj->rx_payload;
  obj->rx_frame_ok++;

  switch (obj->rx_type) {
    case RAS_TYPE_COMMAND:
      HandleCommand(obj, p, frame_end_us);
      break;
    case RAS_TYPE_PING:
      HandlePing(obj, p, frame_end_us);
      break;
    case RAS_TYPE_CONFIG_SET: {
      uint16_t param_id = GetU16(&p[0]);
      float applied = 0.0f;
      uint8_t result = ApplyConfig(obj, param_id, GetF32(&p[2]), &applied);
      SendConfigAck(obj, param_id, applied, result);
      break;
    }
    case RAS_TYPE_CONFIG_GET: {
      uint16_t param_id = GetU16(&p[0]);
      bool known = false;
      float value = ReadConfig(obj, param_id, &known);
      SendConfigAck(obj, param_id, value, known ? RAS_CONFIG_OK : RAS_CONFIG_UNKNOWN_PARAM);
      break;
    }
    case RAS_TYPE_VERSION_REQ:
      SendVersion(obj);
      break;
    default:
      break;
  }
}

// TYPE ごとの期待ペイロード長。未知の TYPE は -1 を返し、LEN を信用して読み飛ばす
static int ExpectedPayloadLen(uint8_t type) {
  switch (type) {
    case RAS_TYPE_COMMAND:
      return RAS_LEN_COMMAND;
    case RAS_TYPE_CONFIG_SET:
      return RAS_LEN_CONFIG_SET;
    case RAS_TYPE_PING:
      return RAS_LEN_PING;
    case RAS_TYPE_CONFIG_GET:
      return RAS_LEN_CONFIG_GET;
    case RAS_TYPE_VERSION_REQ:
      return RAS_LEN_VERSION_REQ;
    default:
      return -1;
  }
}

static void ParseByte(RasLink* obj, uint8_t byte) {
  switch (obj->rx_state) {
    case RX_SYNC1:
      if (byte == RAS_SYNC1) obj->rx_state = RX_SYNC2;
      break;

    case RX_SYNC2:
      if (byte == RAS_SYNC2) {
        obj->rx_state = RX_TYPE;
      } else if (byte != RAS_SYNC1) {
        // 0xAA が連続する場合は最後の 0xAA を同期ワードの先頭として扱い続ける
        obj->rx_state = RX_SYNC1;
      }
      break;

    case RX_TYPE:
      obj->rx_type = byte;
      obj->rx_crc = Crc16Update(CRC16_INIT, byte);
      obj->rx_state = RX_SEQ;
      break;

    case RX_SEQ:
      obj->rx_seq = byte;
      obj->rx_crc = Crc16Update(obj->rx_crc, byte);
      obj->rx_state = RX_LEN;
      break;

    case RX_LEN: {
      obj->rx_len = byte;
      obj->rx_crc = Crc16Update(obj->rx_crc, byte);
      obj->rx_index = 0;

      int expected = ExpectedPayloadLen(obj->rx_type);
      if (expected < 0) {
        obj->rx_unknown_type++;
        obj->rx_discard = true;
      } else if (expected != (int)obj->rx_len) {
        obj->rx_len_error++;
        obj->rx_discard = true;
      } else {
        obj->rx_discard = false;
      }
      // 破棄する場合でも同期を保つためペイロードと CRC は読み飛ばす
      obj->rx_state = (obj->rx_len > 0) ? RX_PAYLOAD : RX_CRC_LO;
      break;
    }

    case RX_PAYLOAD:
      if (obj->rx_index < RAS_PAYLOAD_MAX) obj->rx_payload[obj->rx_index] = byte;
      obj->rx_crc = Crc16Update(obj->rx_crc, byte);
      obj->rx_index++;
      if (obj->rx_index >= obj->rx_len) obj->rx_state = RX_CRC_LO;
      break;

    case RX_CRC_LO:
      obj->rx_crc_received = byte;
      obj->rx_state = RX_CRC_HI;
      break;

    case RX_CRC_HI: {
      obj->rx_crc_received |= (uint16_t)byte << 8;
      obj->rx_state = RX_SYNC1;

      if (obj->rx_crc_received != obj->rx_crc) {
        obj->rx_crc_error++;
        break;
      }
      if (obj->rx_discard) break;

      // SEQ の欠番からロス数を数える (方向ごとの通し番号なのでリンク全体の指標になる)
      if (obj->rx_seq_valid) {
        obj->rx_packet_loss += (uint8_t)(obj->rx_seq - obj->rx_last_seq - 1);
      }
      obj->rx_last_seq = obj->rx_seq;
      obj->rx_seq_valid = true;

      // フレーム末尾の到着時刻。DMA の書き込み位置が IDLE 割り込みの記録位置と一致して
      // いれば、そのフレームでバーストが途切れた = 割り込みで取った時刻が最終バイトの
      // 到着時刻そのものになる。一致しなければ後続バイトが続いているので現在時刻で代用する
      uint32_t frame_end_us =
          (obj->serial->rxBtm == obj->rx_idle_head) ? obj->rx_idle_time_us : Micros();
      DispatchPacket(obj, frame_end_us);
      break;
    }

    default:
      obj->rx_state = RX_SYNC1;
      break;
  }
}

// ---------------------------------------------------------------------------
// 公開 API
// ---------------------------------------------------------------------------

void RasLink_Init(RasLink* obj, Serial* serial) {
  memset(obj, 0, sizeof(*obj));
  obj->serial = serial;

  QueueInit(&obj->tx_queue[RAS_TXQ_PONG], obj->tx_pong_storage, sizeof(obj->tx_pong_storage));
  QueueInit(&obj->tx_queue[RAS_TXQ_TELEMETRY], obj->tx_telemetry_storage, sizeof(obj->tx_telemetry_storage));
  QueueInit(&obj->tx_queue[RAS_TXQ_LIDAR], obj->tx_lidar_storage, sizeof(obj->tx_lidar_storage));
  QueueInit(&obj->tx_queue[RAS_TXQ_INFO], obj->tx_info_storage, sizeof(obj->tx_info_storage));
  QueueInit(&obj->tx_queue[RAS_TXQ_LOG], obj->tx_log_storage, sizeof(obj->tx_log_storage));

  obj->config.max_speed_m_s = RAS_DEFAULT_MAX_SPEED_M_S;
  obj->config.max_accel_m_s2 = RAS_DEFAULT_MAX_ACCEL_M_S2;
  obj->config.max_steer_rad = Steering_GetMaxRoadWheelAngleRad();
  obj->config.tc_enabled = true;
  obj->config.tv_enabled = true;
  obj->config.wheel_lift_guard_enabled = true;
  obj->config.lidar_format = RAS_LIDAR_FORMAT_STANDARD;

  uint32_t now = Micros();
  obj->last_telemetry_us = now;
  obj->last_stats_us = now;
  obj->last_version_us = now;
  obj->version_burst_left = RAS_VERSION_BURST_COUNT;

  g_link = obj;

  // 送信 DMA の完了は UART の TC 割り込みで検出されるため、NVIC を有効にしないと
  // gState が BUSY_TX のままになり2フレーム目以降を送れない。
  // 一方 HAL_UART_Receive_DMA が有効にするパリティ/エラー割り込みは無効化しておく。
  // 有効なままだとオーバーランのたびに HAL が受信 DMA を中断して復帰しなくなるが、
  // 化けたフレームは CRC16 で確実に落とせるので、受信を止めない方が安全側になる
  CLEAR_BIT(serial->huart->Instance->CR1, USART_CR1_PEIE);
  CLEAR_BIT(serial->huart->Instance->CR3, USART_CR3_EIE);
  __HAL_UART_CLEAR_IDLEFLAG(serial->huart);
  __HAL_UART_ENABLE_IT(serial->huart, UART_IT_IDLE);
  HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void RasLink_OnUartIdle(RasLink* obj) {
  obj->rx_idle_time_us = Micros();
  obj->rx_idle_head =
      (uint16_t)(obj->serial->rxBufSize - obj->serial->huart->hdmarx->Instance->NDTR);
}

void RasLink_Update(RasLink* obj) {
  while (Serial_Available(obj->serial)) {
    ParseByte(obj, Serial_Read(obj->serial));
  }

  uint32_t now = Micros();

  if (obj->version_burst_left > 0 &&
      (uint32_t)(now - obj->last_version_us) >= RAS_VERSION_BURST_INTERVAL_US) {
    SendVersion(obj);
    obj->last_version_us = now;
    obj->version_burst_left--;
  }
  if ((uint32_t)(now - obj->last_telemetry_us) >= RAS_TELEMETRY_INTERVAL_US) {
    obj->last_telemetry_us = now;
    SendTelemetry(obj);
  }
  if ((uint32_t)(now - obj->last_stats_us) >= RAS_STATS_INTERVAL_US) {
    obj->last_stats_us = now;
    SendStats(obj);
  }

  PumpTx(obj);
}

void RasLink_SetTelemetry(RasLink* obj, const RasTelemetry* telemetry) {
  obj->telemetry = *telemetry;
}

void RasLink_SetMdStats(RasLink* obj, const uint32_t* rx_count, const uint32_t* rx_error) {
  for (int i = 0; i < 3; i++) {
    obj->md_rx_count[i] = rx_count[i];
    obj->md_rx_error[i] = rx_error[i];
  }
}

const RasCommand* RasLink_GetCommand(const RasLink* obj) {
  return &obj->command;
}

bool RasLink_HasCommand(const RasLink* obj) {
  return obj->has_command;
}

bool RasLink_IsCommandAlive(const RasLink* obj) {
  if (!obj->has_command) return false;
  return (uint32_t)(Micros() - obj->last_command_us) < RAS_COMMAND_TIMEOUT_US;
}

const RasConfig* RasLink_GetConfig(const RasLink* obj) {
  return &obj->config;
}

// USART1 の割り込みハンドラ。CubeMX は USART1 のグローバル割り込みを生成していないため
// ここで定義する (startup の弱いシンボルを上書きする)。.ioc で USART1 の割り込みを
// 有効にすると stm32f4xx_it.c 側にも生成されて多重定義になるので注意。
void USART1_IRQHandler(void) {
  UART_HandleTypeDef* huart = &huart1;
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) && __HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if (g_link != NULL) RasLink_OnUartIdle(g_link);
  }
  // 送信完了 (TC) の処理は HAL に任せる。これが無いと gState が READY に戻らない
  HAL_UART_IRQHandler(huart);
}
