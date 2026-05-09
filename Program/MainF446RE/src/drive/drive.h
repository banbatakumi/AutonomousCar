#ifndef DRIVE_H_
#define DRIVE_H_

#define DIFFERENTIAL 0.5f
#define MAX_ACCELERATION 5.0f
#define MAX_STEER_SPEED 2.0f  // ステアリングの最大回転速度 [rad/s]

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "lpf.h"
#include "maf.h"
#include "main.h"
#include "mymath.h"
#include "pid.h"
#include "pwm_out.h"
#include "serial.h"
#include "timer.h"

typedef struct {
  double min_rad;
  double max_rad;
} SteerConfig;

typedef struct {
  float mech_theta;
  float amp_volt;
  float angular_speed;
  uint8_t flags;
  bool is_enable;
  bool is_voltage_out_of_range;
  bool is_overheat;
} RecvData;

typedef struct {
  float acceleration_left;
  float acceleration_right;
  float steer;
  LPF lpf_steer;
  bool do_brake;
  float brake_strength;
} SendData;

typedef struct {
  float speed;
  MAF maf_speed;
  float current_acceleration;
  Timer accel_timer;
  float current_target_velocity;
  Timer velocity_timer;
  PID pid_velocity;
  Timer steer_timer;
  Timer brake_led_timer;
  bool is_free;
} Drive;

// ペリフェラルを初期化する。
// do_steer_setup が true のとき、ステアリングの機械的限界を測定して Flash に保存する。
// false のとき Flash から前回のキャリブレーション値を読み出す。
void Drive_Init(bool do_steer_setup);

// ステアリングキャリブレーションを 1 ステップ進める。
// Drive_Init(true) の内部から完了まで繰り返し呼び出す。完了時に true を返す。
bool Drive_SetupSteer();

// モータコントローラとのシリアル通信を処理する。
// 各コントローラから受信データを読み出し、車速を更新する。
// 100 Hz の送信レートでコマンドパケットを送出する。
// メインループで毎ティック呼び出すこと。
void Drive_Update();

// 加速度指令値を設定して走行する。
// max_acceleration [m/s²] を目標値として acceleration_rate [m/s³] でランプアップする。
// steer は -1.0（右最大）〜 +1.0（左最大）。
void Drive_Set(float max_acceleration, float acceleration_rate, float steer);

// 目標速度を指定して PID 制御で走行する。
// target_velocity [m/s] に向けて acceleration [m/s²] でランプし、PID で追従する。
// steer は -1.0（右最大）〜 +1.0（左最大）。
void Drive_SetVelocity(float target_velocity, float acceleration, float steer);

// ブレーキコマンドをモータコントローラに送信する。
// deceleration は制動強度（0.0〜MAX_ACCELERATION）。
void Drive_Brake(float deceleration);

// モータコントローラへの送信を停止してモータをフリー状態にする。
// バッテリーエラーや待機モード時に呼び出す。
void Drive_Free();

// 現在の車速を返す [m/s]。移動平均フィルタ済みの値。
float Drive_GetSpeed();

// いずれかのモータコントローラが電圧異常または過熱を報告している場合に true を返す。
bool Drive_HasError();

#endif  // DRIVE_H_
