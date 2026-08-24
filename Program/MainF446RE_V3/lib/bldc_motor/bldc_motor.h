#ifndef BLDC_MOTOR_H_
#define BLDC_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "serial.h"
#include "timer.h"

// ---------------------------------------------------------------------------
// MDへ送る指令フレーム (6バイト固定長)。MD側のパーサと必ず一致させること。
//
//   [0] 0xAA          ヘッダ
//   [1] mode          BldcMotorMode
//   [2] setpoint hi   int16 ビッグエンディアン。スケールは mode ごと
//   [3] setpoint lo
//   [4] torque_limit  uint8 × BLDC_MOTOR_SCALE_TORQUE_LIMIT [N・m]
//   [5] crc8          [1]〜[4] に対する CRC-8/AUTOSAR
//
// トルク上限を毎フレームに載せることで、フレーム1つが常に完結した状態を表す。取りこぼしても
// MDがリセットされても次のフレームで復元されるため、設定の同期という問題自体が発生しない。
// フッタは置かない。固定値のフッタはペイロードの情報を1ビットも含まないためデータ化けを
// 検出できず、CRCがあれば同期確認としても冗長になる。
// ---------------------------------------------------------------------------
#define BLDC_MOTOR_TX_FRAME_SIZE 6

// ---------------------------------------------------------------------------
// MDから受け取る状態フレーム (11バイト固定長)。指令フレームと同じ方針で、末尾は
// CRC-8/AUTOSAR、フッタは無し。
//
//   [0]  0xAA          ヘッダ
//   [1]  status        bit3:過電流 bit2:過熱 bit1:電源電圧異常 bit0:動作中
//   [2]  temperature   uint8 [degC]
//   [3-4]  theta       uint16 BE × 0.1mrad
//   [5-6]  speed       int16 BE × 0.01rad/s
//   [7-8]  iq          int16 BE × 1mA
//   [9]  torque_limit  MDが実際に適用しているトルク上限 (指令フレームのエコーバック)
//   [10] crc8          [1]〜[9] に対する CRC-8/AUTOSAR
//
// 制限値をエコーバックさせることで、MDが本当に意図した上限で動いているかを
// BldcMotor_IsLimitSynced() で確認できる。エコーが無いと、指令が届いていないのか
// MD側が別の値で動いているのかを切り分ける手段が無い。
// ---------------------------------------------------------------------------
#define BLDC_MOTOR_RX_BODY_SIZE 9  // status+temp+theta+speed+iq+torque_limit (CRCとヘッダを除く)

// 指令フレームの送信周期 [us]。制御ループより遅い周期で送るため、BldcMotor_Update() は
// これより短い間隔で呼ばれても送信をスキップする (受信の取りこぼしを防ぐため受信は毎回行う)
#define BLDC_MOTOR_TX_INTERVAL_US 1000

// BLDCモータドライバ (MD) との通信ヘッダ。MD側の SERIAL_COMMANDS テーブルの header と対応させること。
typedef enum {
  BLDC_MOTOR_MODE_SPEED = 0xBA,       // 角速度指令 [rad/s]
  BLDC_MOTOR_MODE_POSITION = 0xBB,    // 位置指令 [rad]
  BLDC_MOTOR_MODE_TORQUE = 0xBC,      // トルク指令 (Iq指令) [A]
  BLDC_MOTOR_MODE_TORQUE_NM = 0xBD,   // トルク指令 [N・m]
  BLDC_MOTOR_MODE_BRAKE = 0xBE,       // 制動電流指令 [A]
  BLDC_MOTOR_MODE_BRAKE_NM = 0xBF,    // 制動トルク指令 [N・m]
} BldcMotorMode;

typedef struct {
  Serial* serial;

  bool tx_enabled;
  uint8_t tx_header;
  int16_t tx_raw;
  uint8_t torque_limit_raw;  // 未設定なら0 = トルク上限0。設定し忘れたら動かないことで気付ける
                             // (ゼロ値が「無制限」に化けないよう、0は最も安全な側に倒してある)
  Timer tx_timer;
  uint8_t tx_frame[BLDC_MOTOR_TX_FRAME_SIZE];  // DMA送信中もSerial_Writeに渡したポインタ先が
                                               // 有効である必要があるため、インスタンスごとに保持する

  uint8_t rx_body[BLDC_MOTOR_RX_BODY_SIZE + 1];  // 本体 + 末尾のCRC
  uint8_t rx_index;
  uint32_t rx_count;        // 正常パースできた状態フレームの累積数 (受信できているかの確認用)
  uint32_t rx_error_count;  // CRC不一致で破棄したフレーム数 (配線・ノイズ環境の確認用)

  bool is_running;               // MDが停止モードでない (指令を送信していれば真)
  bool is_overcurrent;           // MD側で過電流を検知
  bool is_overheat;              // MD側で過熱を検知
  bool is_voltage_out_of_range;  // MD側で電源電圧異常を検知
  uint8_t temperature_c;         // MDの基板温度 [degC]
  float mech_angle_rad;          // モータ機械角 [rad]
  float angular_speed_rad_s;     // モータ角速度 [rad/s]
  float iq_a;                    // q軸電流 [A]
  uint8_t applied_torque_limit_raw;  // MDが適用中のトルク上限 (エコーバック)
  bool data_valid;  // 状態フレームを一度でも受信済みか。BldcMotor_Reset() で偽に落ちる
                    // (受信由来フィールドが「ゼロという実測値」なのか「無効値」なのかを
                    // 区別できないと、ゼロ点からの相対値を計算する上位側 (例:Steering) が
                    // 誤った値を作ってしまうため)
} BldcMotor;

/**
 * @brief BLDCモータドライバ通信モジュールを初期化する。MDと接続されたUARTのSerialインスタンスを渡す。
 */
void BldcMotor_Init(BldcMotor* obj, Serial* serial);

/**
 * @brief 指令の送信と状態フレームの受信・パースを行う。MDが無通信0.5秒で停止モードに移行するため、
 * 制御周期ごと (目安 10ms 以内) に呼び続けること。
 * 呼び出し間隔が BLDC_MOTOR_TX_INTERVAL_US より短い場合、送信のみが間引かれる
 * (受信は毎回処理する)。制御ループの周期を気にせず毎周期呼んでよい。
 */
void BldcMotor_Update(BldcMotor* obj);

/**
 * @brief 角速度指令 [rad/s] を設定する (APP_MODE_SPEED)。
 */
void BldcMotor_SetAngularSpeed(BldcMotor* obj, float rad_s);

/**
 * @brief 位置指令 [rad] を設定する (APP_MODE_POSITION)。
 */
void BldcMotor_SetPosition(BldcMotor* obj, float rad);

/**
 * @brief トルク指令をq軸電流 [A] で設定する (APP_MODE_TORQUE)。
 */
void BldcMotor_SetTorqueCurrent(BldcMotor* obj, float amp);

/**
 * @brief トルク指令 [N・m] を設定する (APP_MODE_TORQUE_NM)。
 */
void BldcMotor_SetTorqueNm(BldcMotor* obj, float nm);

/**
 * @brief 制動電流指令 [A] を設定する (APP_MODE_BRAKE)。
 */
void BldcMotor_SetBrakeCurrent(BldcMotor* obj, float amp);

/**
 * @brief 制動トルク指令 [N・m] を設定する (APP_MODE_BRAKE_NM)。
 */
void BldcMotor_SetBrakeNm(BldcMotor* obj, float nm);

/**
 * @brief MD側で効かせるトルク上限 [N・m] を設定する。以降すべての指令フレームに載るため
 * 呼び出しは1回でよく、取りこぼしやMDのリセットがあっても次のフレームで復元される。
 * 上限値なので量子化は切り捨て (安全側)、レンジ外は 0〜0.255 N・m に飽和させる。
 * 0.255 を超える値を渡すと飽和し、MD側の定数上限がそのまま効く = 実質的に無制限になるため、
 * モータ最大より小さい値を指定すること。
 * 初期値は0 (=トルク上限0) のため、モータを動かす前に必ず設定すること。
 */
void BldcMotor_SetTorqueLimitNm(BldcMotor* obj, float nm);

/**
 * @brief 指令の送信を停止する。MD側は無通信0.5秒後に停止モードへ自動的に移行する
 * (MDのプロトコルに明示的な停止コマンドが無いため)。設定値の再送も併せて止まる。
 */
void BldcMotor_Stop(BldcMotor* obj);

/**
 * @brief MDからの状態フレーム由来の情報 (status/temperature/theta/speed/iq/applied_torque_limit)
 * を初期値へリセットし、data_valid を偽にする。MDへの給電が切れている (DRIVE_POWER OFF) 間、
 * MDが状態フレームを送ってこないため何もしなければ最後に受信した値を保持し続け、給電が
 * 切れていることに気付かず古い角度・電流値を現在値だと誤認しうる (幽霊値)。
 * 呼び出し側が給電状態を把握して都度呼ぶこと。指令 (tx_*) と受信統計
 * (rx_count/rx_error_count) はここではリセットしない。
 */
void BldcMotor_Reset(BldcMotor* obj);

/**
 * @brief 状態フレームを一度でも受信済み (＝各種取得値が実測値) かを取得する。
 * BldcMotor_Reset() を呼んだ直後は偽になり、次に状態フレームを受信するまで偽のまま。
 * ゼロ点からの相対値を計算する上位側 (例: Steering) は、これが偽の間は生値をそのまま
 * 使わず無効値として扱うこと (でないとリセットされた0とセンター点との差分が実際の
 * 角度であるかのように計算されてしまう)。
 */
bool BldcMotor_IsDataValid(const BldcMotor* obj);

/**
 * @brief MDが停止モードでないか (直近の状態フレームより) を取得する。
 */
bool BldcMotor_IsRunning(const BldcMotor* obj);

/**
 * @brief MD側で過電流異常が検知されているかを取得する。
 */
bool BldcMotor_IsOvercurrent(const BldcMotor* obj);

/**
 * @brief MD側で過熱異常が検知されているかを取得する。
 */
bool BldcMotor_IsOverheat(const BldcMotor* obj);

/**
 * @brief MD側で電源電圧異常が検知されているかを取得する。
 */
bool BldcMotor_IsVoltageOutOfRange(const BldcMotor* obj);

/**
 * @brief MDの基板温度 [degC] を取得する。
 */
uint8_t BldcMotor_GetTemperatureC(const BldcMotor* obj);

/**
 * @brief モータ機械角 [rad] を取得する。
 */
float BldcMotor_GetMechAngle(const BldcMotor* obj);

/**
 * @brief モータ角速度 [rad/s] を取得する。
 */
float BldcMotor_GetAngularSpeed(const BldcMotor* obj);

/**
 * @brief q軸電流 [A] を取得する。
 */
float BldcMotor_GetIq(const BldcMotor* obj);

/**
 * @brief MDが実際に適用しているトルク上限 [N・m] を取得する (状態フレームのエコーバック)。
 */
float BldcMotor_GetAppliedTorqueLimitNm(const BldcMotor* obj);

/**
 * @brief MDが適用中のトルク上限が、こちらが指令している値と一致しているかを取得する。
 * 状態フレームを一度も受信できていない間は偽を返す。偽のまま変わらない場合、MDが
 * 制限値を解釈していない (プロトコル不一致) か、指令フレームが届いていない。
 */
bool BldcMotor_IsLimitSynced(const BldcMotor* obj);

/**
 * @brief 正常にパースできた状態フレームの累積受信数を取得する。呼び出し間隔で値が
 * 増えていなければ、MDからの受信ができていない (配線・ボーレート不一致など) ことを示す。
 */
uint32_t BldcMotor_GetRxCount(const BldcMotor* obj);

/**
 * @brief CRC不一致で破棄した状態フレームの累積数を取得する。BldcMotor_GetRxCount との比が
 * 通信品質の目安になる (継続的に増えるならノイズ・配線・ボーレート誤差を疑う)。
 */
uint32_t BldcMotor_GetRxErrorCount(const BldcMotor* obj);

#endif  // BLDC_MOTOR_H_
