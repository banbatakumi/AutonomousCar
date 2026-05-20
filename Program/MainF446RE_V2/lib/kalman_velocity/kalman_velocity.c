#include "kalman_velocity.h"

void KalmanVelocity_Init(KalmanVelocity* kv, float Q, float R, float slip_threshold) {
  kv->v = 0.0f;
  kv->P = 1.0f;
  kv->Q = Q;
  kv->R = R;
  kv->P_max = 100.0f;
  kv->slip_threshold = slip_threshold;
  kv->innovation = 0.0f;
}

void KalmanVelocity_Reset(KalmanVelocity* kv, float init_vel) {
  kv->v = init_vel;
  kv->P = 1.0f;
  kv->innovation = 0.0f;
}

float KalmanVelocity_Update(KalmanVelocity* kv, float accel, float dt, float v_meas) {
  // 予測ステップ: IMU加速度で状態を時間伝播
  float v_pred = kv->v + accel * dt;
  float P_pred = kv->P + kv->Q * dt;
  if (P_pred > kv->P_max) P_pred = kv->P_max;

  // イノベーション: 観測値と予測値の差
  kv->innovation = v_meas - v_pred;

  // イノベーション・ゲーティング:
  //   |innovation| > slip_threshold のとき観測をスキップし IMU 積分のみで伝播する。
  //   これにより持続スリップ中も innovation が大きいまま維持される。
  //   ゲーティング中は P が増加し続けるため、スリップ解消後に大きな K でオドメトリへ素早く復帰する。
  if (kv->slip_threshold > 0.0f &&
      (kv->innovation > kv->slip_threshold || kv->innovation < -kv->slip_threshold)) {
    kv->v = v_pred;
    kv->P = P_pred;
  } else {
    float K = P_pred / (P_pred + kv->R);
    kv->v = v_pred + K * kv->innovation;
    kv->P = (1.0f - K) * P_pred;
  }

  return kv->v;
}
