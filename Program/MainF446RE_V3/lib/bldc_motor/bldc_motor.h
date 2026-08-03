#ifndef BLDC_MOTOR_H_
#define BLDC_MOTOR_H_

#include <stdbool.h>
#include <stdint.h>

#include "serial.h"

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
  uint8_t tx_frame[5];  // DMA送信中もSerial_Writeに渡したポインタ先が有効である必要があるため、インスタンスごとに保持する

  uint8_t rx_body[8];  // status(1) + temp(1) + theta(2) + speed(2) + iq(2)
  uint8_t rx_index;
  uint32_t rx_count;  // 正常パースできた状態フレームの累積数 (受信できているかの確認用)

  bool is_running;               // MDが停止モードでない (指令を送信していれば真)
  bool is_overcurrent;           // MD側で過電流を検知
  bool is_overheat;              // MD側で過熱を検知
  bool is_voltage_out_of_range;  // MD側で電源電圧異常を検知
  uint8_t temperature_c;         // MDの基板温度 [degC]
  float mech_angle_rad;          // モータ機械角 [rad]
  float angular_speed_rad_s;     // モータ角速度 [rad/s]
  float iq_a;                    // q軸電流 [A]
} BldcMotor;

/**
 * @brief BLDCモータドライバ通信モジュールを初期化する。MDと接続されたUARTのSerialインスタンスを渡す。
 */
void BldcMotor_Init(BldcMotor* obj, Serial* serial);

/**
 * @brief 指令の送信と状態フレームの受信・パースを行う。MDが無通信0.5秒で停止モードに移行するため、
 * 制御周期ごと (目安 10ms 以内) に呼び続けること。
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
 * @brief 指令の送信を停止する。MD側は無通信0.5秒後に停止モードへ自動的に移行する
 * (MDのプロトコルに明示的な停止コマンドが無いため)。
 */
void BldcMotor_Stop(BldcMotor* obj);

/**
 * @brief MDが停止モードでないか (直近の状態フレームより) を取得する。
 */
bool BldcMotor_IsRunning(BldcMotor* obj);

/**
 * @brief MD側で過電流異常が検知されているかを取得する。
 */
bool BldcMotor_IsOvercurrent(BldcMotor* obj);

/**
 * @brief MD側で過熱異常が検知されているかを取得する。
 */
bool BldcMotor_IsOverheat(BldcMotor* obj);

/**
 * @brief MD側で電源電圧異常が検知されているかを取得する。
 */
bool BldcMotor_IsVoltageOutOfRange(BldcMotor* obj);

/**
 * @brief MDの基板温度 [degC] を取得する。
 */
uint8_t BldcMotor_GetTemperatureC(BldcMotor* obj);

/**
 * @brief モータ機械角 [rad] を取得する。
 */
float BldcMotor_GetMechAngle(BldcMotor* obj);

/**
 * @brief モータ角速度 [rad/s] を取得する。
 */
float BldcMotor_GetAngularSpeed(BldcMotor* obj);

/**
 * @brief q軸電流 [A] を取得する。
 */
float BldcMotor_GetIq(BldcMotor* obj);

/**
 * @brief 正常にパースできた状態フレームの累積受信数を取得する。呼び出し間隔で値が
 * 増えていなければ、MDからの受信ができていない (配線・ボーレート不一致など) ことを示す。
 */
uint32_t BldcMotor_GetRxCount(BldcMotor* obj);

#endif  // BLDC_MOTOR_H_
