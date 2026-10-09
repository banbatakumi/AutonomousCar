#ifndef STEERING_H_
#define STEERING_H_

#include <stdbool.h>

#include "bldc_motor.h"

#define STEERING_MAX_ANGLE_RAD 1.0471975512f  // ±60度 (PI/3)

// MD側の位置制御ループに効かせるトルク上限。MDの位置制御は位置PIDの出力をそのままIq指令にする
// 構成 (速度ループは挟まない) で、この上限がPIDの出力飽和になる。据え切りやラックエンド当てで
// 位置偏差が残り続けると、保持電流はこの上限そのものになる (焼かない電流を選ぶ責任は上位側)。
// モータ最大 (MD側 Kt × MAX_CURRENT = 約0.195 N・m) より小さくすること。これを上回る値を
// 指定するとMD側の定数上限が効くだけになり、ここで絞る意味が無くなる。
// 速度上限は持たせていない。位置モードの移動速度は、加速側がこのトルク上限、減速側が
// MDの位置PIDの kp/kd で決まる (速すぎるときはMD側の kd を上げる)。
#define STEERING_MAX_TORQUE_NM 0.1f

// モータ機械角の増加方向と舵角の正方向 (反時計回り = 左旋回) の対応。モータの取付向きや
// リンクの組み方で反転するため、実機に合わせて +1 / -1 を切り替える。指令と実測の両方に
// 同じ符号を掛けるので、ここを反転しても報告される舵角と実際の向きは一致したままになる。
#define STEERING_DIRECTION_SIGN (-1.0f)

// モータ機械角 [rad] → 路面舵角 [rad] の換算。リンク機構は線形ではなく、大舵角ほど路面舵角が
// 付いてこない (2026-10-08 の実測: 比 0.5 のままだと 29.1° と報告しているとき前輪は約 26.5°)。
//   x        = モータ角 × STEERING_LINKAGE_RATIO   (中立付近の比で直した「見かけの舵角」)
//   路面舵角 = gain*x + cubic*x^3                   (Steering_SetLinkage。既定は 1 と 0)
// gain・cubic は上位がシステム同定 (舵角ごとの曲率) で求めて CONFIG_SET で送る (control_params.h の
// steer_link_gain / steer_link_cubic)。リンクが複雑で設計から式を出せないため。
// 上位 (Raspberry Pi) の自転車モデル・Pure Pursuit が必要とするのは路面舵角であり、モータ角では
// ない。このファーム自身も、前輪の速度を車体前後方向へ射影するのに路面舵角を使う (Drive の車速 →
// TC のスリップ・TV の横加速度)。換算がずれると車速が cos のぶんだけずれる
#define STEERING_LINKAGE_RATIO 0.5f

// 換算の傾き (d路面舵角/dx) の下限。可動範囲の端でこれを下回る cubic は、下回らない値へ丸める
// (傾きが 0 以下になると路面舵角 → モータ角の逆変換が一意に決まらない)
#define STEERING_LINKAGE_MIN_SLOPE 0.3f

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
 * @brief モータ角 → 路面舵角の換算 (路面舵角 = gain*x + cubic*x^3、x = モータ角×リンク比) を設定する。
 * 以後の指令・報告・可動範囲のすべてに効く。ステアは1つしか無いのでモジュールで1組だけ持つ。
 */
void Steering_SetLinkage(float gain, float cubic);

/**
 * @brief 路面舵角 [rad] (反時計回り = 左旋回が正) を指令する。
 * 上位から来る舵角指令はすべてこちらを使うこと。
 */
void Steering_SetRoadWheelAngleRad(Steering* obj, float angle_rad);

/**
 * @brief 現在の路面舵角 [rad] を取得する (モータ機械角をリンクの換算に通した値)。
 */
float Steering_GetRoadWheelAngleRad(const Steering* obj);

/**
 * @brief 路面舵角の可動範囲 [rad] を取得する (モータの可動範囲をリンクの換算に通した値)。
 */
float Steering_GetMaxRoadWheelAngleRad(void);

/**
 * @brief 有効な直進中心点を持っているかを取得する。
 * 偽の場合、舵角の絶対値が信用できないため走行させてはならない。
 */
bool Steering_IsCenterValid(const Steering* obj);

#endif  // STEERING_H_
