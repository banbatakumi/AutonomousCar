#ifndef KALMAN_VELOCITY_H_
#define KALMAN_VELOCITY_H_

// 1次元カルマンフィルタによる車体速度推定器（イノベーション・ゲーティング付き）。
//
// 状態変数: v_body (車体速度 [m/s])
// 予測モデル: v_k = v_{k-1} + a_imu * dt  (IMU加速度を入力)
// 観測モデル: z = v_odometry               (車輪速度 = スリップなしなら車体速度)
//
// スリップ検知の原理:
//   イノベーション innovation = v_odometry - v_predicted
//     正方向に大きい → 車輪が車体より速く回転 → 空転スリップ
//     負方向に大きい → 車輪が車体より遅く回転 → 制動ロック
//
// イノベーション・ゲーティング (slip_threshold > 0 のとき有効):
//   |innovation| > slip_threshold のとき観測更新をスキップし、IMU積分のみで伝播。
//   これにより持続スリップ中も innovation が大きいまま維持される。
//   スリップ解消後: P が大きくなっているため K が大きくなり、オドメトリへ素早く復帰。

typedef struct {
  float v;              // 推定車体速度 [m/s]
  float P;              // 誤差共分散
  float Q;              // プロセスノイズ分散 [m²/s² per s]
  float R;              // 観測ノイズ分散 [m²/s²]
  float P_max;          // P の上限（ゲーティング中の発散防止）
  float slip_threshold; // ゲーティング閾値 [m/s]（0 で無効 = 常に観測更新）
  float innovation;     // イノベーション: v_odometry - v_predicted [m/s]
} KalmanVelocity;

// Q: プロセスノイズ分散 (1秒あたり)。IMU誤差・未モデル化加速度の大きさに応じて調整。
// R: 観測ノイズ分散。オドメトリの計測誤差の大きさに応じて調整。
// slip_threshold: ゲーティング閾値 [m/s]。|innovation| がこの値を超えると観測更新をスキップ。
void KalmanVelocity_Init(KalmanVelocity* kv, float Q, float R, float slip_threshold);

// 速度推定器を既知の速度にリセットする。Drive_Free() 後などに使用。
void KalmanVelocity_Reset(KalmanVelocity* kv, float init_vel);

// IMU加速度 accel [m/s²]、経過時間 dt [s]、オドメトリ速度 v_meas [m/s] で更新。
// 推定速度を返し、kv->innovation にイノベーションを格納する。
// ゲーティングが有効かつ |innovation| > slip_threshold のとき観測更新をスキップする。
float KalmanVelocity_Update(KalmanVelocity* kv, float accel, float dt, float v_meas);

#endif  // KALMAN_VELOCITY_H_
