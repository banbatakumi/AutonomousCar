#ifndef STEERING_H_
#define STEERING_H_

#include <stdbool.h>

#include "bldc_motor.h"

#define STEERING_MAX_ANGLE_RAD 1.0471975512f  // ±60度 (PI/3)

// MD側の位置制御ループに効かせるトルク上限。据え切りやラックエンド当てで位置偏差が残り続けると
// MD内の速度ループがワインドアップして電流を流しっぱなしにするため、これは必須。
// モータ最大 (0.1557 N・m) より小さくすること。これを上回る値を指定すると
// MD側の定数上限が効くだけになり、ここで絞る意味が無くなる。
// 速度上限は持たせていない。位置制御の速度指令は Kp × 位置偏差 で決まり、舵角は
// ±60度に有界なので、位置ゲイン自体が速度リミッタとして働くため。
#define STEERING_MAX_TORQUE_NM 0.075f

// モータ機械角の増加方向と舵角の正方向 (反時計回り = 左旋回) の対応。モータの取付向きや
// リンクの組み方で反転するため、実機に合わせて +1 / -1 を切り替える。指令と実測の両方に
// 同じ符号を掛けるので、ここを反転しても報告される舵角と実際の向きは一致したままになる。
#define STEERING_DIRECTION_SIGN (-1.0f)

// モータ機械角 [rad] → 路面舵角 [rad] の換算比 ★実機で実測して差し替えること★
// 上位 (Raspberry Pi) の自転車モデル・Pure Pursuit が必要とするのは路面舵角であり、
// モータ角ではない。リンク比が入るまで路面舵角の報告値は正しくない。
#define STEERING_LINKAGE_RATIO 0.5f

typedef struct {
  BldcMotor* motor;
  float center_rad;   // 直進時のモータ機械角 [rad] (0~2piの生角度系におけるオフセット)
  bool center_valid;  // 有効な中心点を持っているか (較正済み or Flashから読めた)
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

/**
 * @brief 路面舵角 [rad] (反時計回り = 左旋回が正) を指令する。
 * 上位から来る舵角指令はすべてこちらを使うこと。
 */
void Steering_SetRoadWheelAngleRad(Steering* obj, float angle_rad);

/**
 * @brief 現在の路面舵角 [rad] を取得する (モータ機械角に STEERING_LINKAGE_RATIO を掛けた値)。
 */
float Steering_GetRoadWheelAngleRad(const Steering* obj);

/**
 * @brief 路面舵角の可動範囲 [rad] を取得する。
 */
float Steering_GetMaxRoadWheelAngleRad(void);

/**
 * @brief 有効な直進中心点を持っているかを取得する。
 * 偽の場合、舵角の絶対値が信用できないため走行させてはならない。
 */
bool Steering_IsCenterValid(const Steering* obj);

#endif  // STEERING_H_
