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
 * @brief 3モータ分の状態フレームを受信する。制御周期ごとに、MDの状態 (舵角・後輪速度等) を
 * 使うモジュール (Vehicle/Drive) の更新より前に呼ぶこと。後に呼ぶと1周期 (500us) 古い値で
 * 制御することになる。
 * powered は3モータ共通の給電状態 (DRIVE_POWER の実際の状態) を渡す。偽の間は3モータとも
 * MDが給電されておらず状態フレームが届かないため、受信を行わず BldcMotor_Reset() で
 * 受信由来の情報を初期値へ落とす (給電が切れたことに気付かず古い値を保持し続けないため)。
 */
void Motors_Receive(Motors* obj, bool powered);

/**
 * @brief 3モータ分の指令を送信する。制御周期ごとに、指令を決めるモジュール (Drive) の
 * 更新の直後に呼ぶこと。間引かずに毎周期送るので、決めた指令がその周期のうちにMDへ届く。
 * powered が偽の間 (MDに給電されていない) は送信しない。
 */
void Motors_Transmit(Motors* obj, bool powered);

#endif  // MOTORS_H_
