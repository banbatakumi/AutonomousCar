#ifndef DRIVE_H_
#define DRIVE_H_

#define DIFFERENTIAL 0.25f
#define MAX_TORQUE 2.0f
#define MAX_STEER_SPEED 2.0f  // ステアリングの最大回転速度 [rad/s]

// 車体の物理パラメータ
#define WHEEL_BASE 0.220f   // ホイールベース (前後軸間距離) [m]
#define TREAD_WIDTH 0.143f  // トレッド幅 (左右車輪中心間距離) [m]

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "lpf.h"
#include "maf.h"
#include "main.h"
#include "mymath.h"
#include "pid.h"
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
  float angular_accel;
  uint8_t temperature;
  uint8_t flags;
  bool is_enable;
  bool is_voltage_out_of_range;
  bool is_overheat;
} RecvData;

typedef struct {
  float torque_left;
  float torque_right;
  float steer;
  LPF lpf_steer;
  bool do_brake;
  float brake_strength;
} SendData;

typedef struct {
  float speed;
  float accel;
  MAF maf_speed;
  MAF maf_acccel;
  MAF maf_imu_long;
  MAF maf_imu_lat;
  float current_torque;
  Timer torque_timer;
  float current_target_velocity;
  Timer velocity_timer;
  PID pid_velocity;
  Timer steer_timer;
  Timer traction_timer;
  bool is_free;
  bool is_slipping;
  float steer_logical;  // 直近のステア値 [-1, +1]（正=左、負=右）。ウィンカー判定に使用。
  float imu_accel_x;
  float imu_accel_y;
  float imu_pitch_deg;
  float imu_roll_deg;
  bool imu_valid;
  float imu_velocity;        // IMU積分による推定速度 [m/s]
  float traction_vel_limit;  // スリップ時の目標速度上限 [m/s]
  float imu_long_bias;
  float imu_lat_bias;
  bool traction_enabled;
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

// モータ出力を指定して走行する。
// target_torque を目標値として torque_rate [/s] でランプアップする。
// steer は -1.0（右最大）〜 +1.0（左最大）。
void Drive_Set(float target_torque, float torque_rate, float steer);

// 目標速度を指定して PID 制御で走行する。
// target_velocity [m/s] に向けて acceleration [m/s²] でランプし、PID で追従する。
// steer は -1.0（右最大）〜 +1.0（左最大）。
void Drive_SetVelocity(float target_velocity, float acceleration, float steer);

// ブレーキコマンドをモータコントローラに送信する。
// deceleration は制動強度（0.0〜MAX_TORQUE）。steer は -1.0〜+1.0。
void Drive_Brake(float deceleration, float steer);

// モータコントローラへの送信を停止してモータをフリー状態にする。
// バッテリーエラーや待機モード時に呼び出す。
void Drive_Free();

// 現在の車速を返す [m/s]。移動平均フィルタ済みの値。
float Drive_GetSpeed();

// 現在の車加速度を返す [m/s²]。移動平均フィルタ済みの値。
float Drive_GetAccel();

float Drive_GetSteer();

uint8_t Drive_GetLeftMotorTemperature();
uint8_t Drive_GetRightMotorTemperature();
uint8_t Drive_GetSteerMotorTemperature();

// IMU の加速度・姿勢を渡す。Drive 側で摩擦推定と加速度上限に利用する。
void Drive_SetImuData(float accel_x, float accel_y, float pitch_deg, float roll_deg);

// いずれかのモータコントローラが電圧異常または過熱を報告している場合に true を返す。
bool Drive_HasError();

// トラクション制御の有効/無効を切り替える。デフォルトは有効。
void Drive_SetTractionEnabled(bool enabled);

#endif  // DRIVE_H_
