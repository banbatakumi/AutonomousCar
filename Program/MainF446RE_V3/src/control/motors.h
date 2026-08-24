#ifndef MOTORS_H_
#define MOTORS_H_

#include "bldc_motor.h"
#include "serial.h"

// 走行系3モータ (ステアリング/左後輪/右後輪) のBLDCモータドライバ通信をまとめて保持する
typedef struct {
  BldcMotor steering;
  BldcMotor rear_left;
  BldcMotor rear_right;
} Motors;

/**
 * @brief 走行系3モータを初期化する。各モータのMDが接続されたUARTのSerialインスタンスを渡す
 * (ステアリング: USART2, 左後輪: USART3, 右後輪: UART4)。
 */
void Motors_Init(Motors* obj, Serial* steering_serial, Serial* rear_left_serial, Serial* rear_right_serial);

/**
 * @brief 3モータ分の指令送信・状態受信をまとめて行う。制御周期ごとに呼ぶこと。
 * 送信レートは BldcMotor 側で BLDC_MOTOR_TX_INTERVAL_US に間引かれるため、
 * 制御ループが速くてもそのまま毎周期呼んでよい。
 * powered は3モータ共通の給電状態 (DRIVE_POWER の実際の状態) を渡す。偽の間は3モータとも
 * MDが給電されておらず状態フレームが届かないため、送受信を行わず BldcMotor_Reset() で
 * 受信由来の情報を初期値へ落とす (給電が切れたことに気付かず古い値を保持し続けないため)。
 */
void Motors_Update(Motors* obj, bool powered);

#endif  // MOTORS_H_
