#ifndef STEERING_H_
#define STEERING_H_

#include <stdbool.h>

#include "bldc_motor.h"

#define STEERING_MAX_ANGLE_RAD 1.04719755119659774615f  // ±60度 (PI/3)

// MD側の位置制御ループに効かせるトルク上限。据え切りやラックエンド当てで位置偏差が残り続けると
// MD内の速度ループがワインドアップして電流を流しっぱなしにするため、これは必須。
// モータ最大 (0.1557 N・m) より小さくすること。これを上回る値を指定すると
// MD側の定数上限が効くだけになり、ここで絞る意味が無くなる。
// 速度上限は持たせていない。位置制御の速度指令は Kp × 位置偏差 で決まり、舵角は
// ±60度に有界なので、位置ゲイン自体が速度リミッタとして働くため。
#define STEERING_MAX_TORQUE_NM 0.1f

typedef struct {
  BldcMotor* motor;
  float center_rad;  // 直進時のモータ機械角 [rad] (0~2piの生角度系におけるオフセット)
} Steering;

/**
 * @brief ステアリング制御を初期化する。
 * do_calibrate が真の場合、その場のモータ角度を直進中心点として記録しflashへ保存する
 * (ステアリングを手で真っ直ぐな位置に合わせた状態でボタン1を押しながら起動することを想定)。
 * 偽の場合はflashに保存済みの中心点を読み込む (未キャリブレーション時は0とする)。
 */
void Steering_Init(Steering* obj, BldcMotor* motor, bool do_calibrate);

/**
 * @brief 中心点からの相対角度 [rad] (-60度~+60度の範囲にクランプ) を指令する。
 */
void Steering_SetAngleRad(Steering* obj, float angle_rad);

/**
 * @brief 中心点からの現在の相対角度 [rad] を取得する。
 * 生角度(0~2pi)が中心点をまたいで切り替わる場合でも、最短角度差により連続な値になる。
 */
float Steering_GetAngleRad(const Steering* obj);

#endif  // STEERING_H_
