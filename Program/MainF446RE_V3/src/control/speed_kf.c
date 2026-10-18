#include "speed_kf.h"

#include <math.h>
#include <string.h>

#define N 4

static void ResetDiagPrev(SpeedKf* kf) {
  // 初期化直後の残差は前の残差と続いていないので、自己相関の対にしない
  for (int c = 0; c < SPEED_KF_DIAG_COUNT; c++) kf->diag[c].has_prev = false;
}

static void ResetState(SpeedKf* kf) {
  memset(kf->x, 0, sizeof(kf->x));
  memset(kf->p, 0, sizeof(kf->p));
  // 初期の不確かさ: 走行距離は測定でほぼ決まる、速度は止まっている想定、加速度とバイアスは大きめ
  kf->p[0][0] = kf->cfg.sigma_s_m * kf->cfg.sigma_s_m;
  kf->p[1][1] = 1.0f;
  kf->p[2][2] = 10.0f;
  kf->p[3][3] = 25.0f;
  kf->s_meas = 0.0f;
  ResetDiagPrev(kf);
}

void SpeedKf_Init(SpeedKf* kf) {
  kf->cfg.sigma_s_m = SPEED_KF_SIGMA_S_M;
  kf->cfg.sigma_a_m_s2 = SPEED_KF_SIGMA_A_M_S2;
  kf->cfg.sigma_a_per_m_s = 0.0f;
  kf->cfg.jerk_psd = SPEED_KF_JERK_PSD;
  kf->cfg.bias_walk_psd = SPEED_KF_BIAS_WALK_PSD;
  ResetState(kf);
  kf->resync = false;
  memset(kf->diag, 0, sizeof(kf->diag));
  kf->gated_count = 0;
  kf->initialized = true;
}

void SpeedKf_SetConfig(SpeedKf* kf, const SpeedKfConfig* cfg) { kf->cfg = *cfg; }

// P を対称に保ち、対角が負・非有限にならないようにする (float で更新を重ねると崩れるため)
static void Condition(SpeedKf* kf) {
  for (int i = 0; i < N; i++) {
    for (int j = i + 1; j < N; j++) {
      float m = 0.5f * (kf->p[i][j] + kf->p[j][i]);
      kf->p[i][j] = m;
      kf->p[j][i] = m;
    }
    if (!(kf->p[i][i] > 1e-14f)) kf->p[i][i] = 1e-14f;
  }
}

void SpeedKf_Predict(SpeedKf* kf, float dt) {
  if (!kf->initialized) return;
  if (!(dt > 0.0f)) return;
  if (dt > SPEED_KF_MAX_DT_S) {
    // 制御ループが長く止まっていた。その間の動きは分からないので速度を捨てて測り直す
    uint32_t gated = kf->gated_count;
    ResetState(kf);
    kf->gated_count = gated;
    kf->resync = true;
    return;
  }

  // x' = F x : s += v dt + a dt²/2、v += a dt
  const float h2 = 0.5f * dt * dt;
  kf->x[0] += kf->x[1] * dt + kf->x[2] * h2;
  kf->x[1] += kf->x[2] * dt;

  // P' = F P Fᵀ + Q。F は単位行列に F01=dt, F02=dt²/2, F12=dt を足した形
  float fp[N][N];  // F P
  for (int j = 0; j < N; j++) {
    fp[0][j] = kf->p[0][j] + dt * kf->p[1][j] + h2 * kf->p[2][j];
    fp[1][j] = kf->p[1][j] + dt * kf->p[2][j];
    fp[2][j] = kf->p[2][j];
    fp[3][j] = kf->p[3][j];
  }
  for (int i = 0; i < N; i++) {
    kf->p[i][0] = fp[i][0] + dt * fp[i][1] + h2 * fp[i][2];
    kf->p[i][1] = fp[i][1] + dt * fp[i][2];
    kf->p[i][2] = fp[i][2];
    kf->p[i][3] = fp[i][3];
  }

  // Q: jerk の白色雑音 (連続時間の PSD q) を離散化したもの + バイアスのランダムウォーク
  const float q = kf->cfg.jerk_psd;
  const float dt2 = dt * dt, dt3 = dt2 * dt, dt4 = dt3 * dt, dt5 = dt4 * dt;
  kf->p[0][0] += q * dt5 / 20.0f;
  kf->p[0][1] += q * dt4 / 8.0f;
  kf->p[1][0] += q * dt4 / 8.0f;
  kf->p[0][2] += q * dt3 / 6.0f;
  kf->p[2][0] += q * dt3 / 6.0f;
  kf->p[1][1] += q * dt3 / 3.0f;
  kf->p[1][2] += q * dt2 / 2.0f;
  kf->p[2][1] += q * dt2 / 2.0f;
  kf->p[2][2] += q * dt;
  kf->p[3][3] += kf->cfg.bias_walk_psd * dt;

  Condition(kf);
}

// スカラー測定の更新。h は測定行列 (1×4)、innovation は z − h x、r は測定ノイズの分散
static void ScalarUpdate(SpeedKf* kf, SpeedKfDiagChannel channel, const float h[N], float innovation, float r) {
  float ph[N];  // P hᵀ
  for (int i = 0; i < N; i++) {
    ph[i] = 0.0f;
    for (int j = 0; j < N; j++) ph[i] += kf->p[i][j] * h[j];
  }
  float hph = 0.0f;
  for (int i = 0; i < N; i++) hph += h[i] * ph[i];

  // 一貫性の診断。ゲートで膨らませる前の S (= フィルタが主張する残差の分散) で正規化する
  const float s_nominal = hph + r;
  if (s_nominal > 0.0f) {
    SpeedKfDiag* d = &kf->diag[channel];
    float z = innovation / sqrtf(s_nominal);
    if (z > SPEED_KF_DIAG_Z_CLIP) z = SPEED_KF_DIAG_Z_CLIP;
    if (z < -SPEED_KF_DIAG_Z_CLIP) z = -SPEED_KF_DIAG_Z_CLIP;
    d->nis_sum += z * z;
    d->n++;
    if (d->has_prev) {
      d->ac_sum += z * d->z_prev;
      d->n_ac++;
    }
    d->z_prev = z;
    d->has_prev = true;
  }

  // 予測との差が大きすぎる測定は、分散を膨らませて効きを弱める (外れ値に引きずられない)
  const float gate2 = SPEED_KF_GATE_SIGMA * SPEED_KF_GATE_SIGMA;
  const float y2 = innovation * innovation;
  if (y2 > gate2 * (hph + r)) {
    r = y2 / gate2 - hph;
    kf->gated_count++;
  }

  const float s = hph + r;
  if (!(s > 0.0f)) return;
  const float inv_s = 1.0f / s;

  float k[N];
  for (int i = 0; i < N; i++) k[i] = ph[i] * inv_s;
  for (int i = 0; i < N; i++) kf->x[i] += k[i] * innovation;
  // P' = P − K (P hᵀ)ᵀ
  for (int i = 0; i < N; i++) {
    for (int j = 0; j < N; j++) kf->p[i][j] -= k[i] * ph[j];
  }
  Condition(kf);
}

void SpeedKf_UpdateDistance(SpeedKf* kf, float delta_m) {
  if (!kf->initialized) return;
  if (kf->resync) {
    // 初期化した直後の増分は、止まっていた間 (半周を超えて回りうる) の回転で、距離として使えない
    kf->resync = false;
    return;
  }
  kf->s_meas += delta_m;
  static const float h[N] = {1.0f, 0.0f, 0.0f, 0.0f};
  ScalarUpdate(kf, SPEED_KF_DIAG_DISTANCE, h, kf->s_meas - kf->x[0], kf->cfg.sigma_s_m * kf->cfg.sigma_s_m);

  // s と s_meas は差だけが意味を持つので、同じ量を引いて小さく保つ (float の仮数は 24bit)
  if (fabsf(kf->s_meas) > SPEED_KF_REBASE_M) {
    kf->x[0] -= kf->s_meas;
    kf->s_meas = 0.0f;
  }

  if (!isfinite(kf->x[1]) || !isfinite(kf->x[2]) || !isfinite(kf->x[3])) {
    uint32_t gated = kf->gated_count;
    ResetState(kf);
    kf->gated_count = gated;
  }
}

void SpeedKf_UpdateAccel(SpeedKf* kf, float accel_x) {
  if (!kf->initialized) return;
  static const float h[N] = {0.0f, 0.0f, 1.0f, 1.0f};
  // 走行中の振動は車速とともに増える (路面の凹凸・駆動の脈動の周波数と強さが車速に比例する)
  const float sigma = kf->cfg.sigma_a_m_s2 + kf->cfg.sigma_a_per_m_s * fabsf(kf->x[1]);
  ScalarUpdate(kf, SPEED_KF_DIAG_ACCEL, h, accel_x - (kf->x[2] + kf->x[3]), sigma * sigma);
}

void SpeedKf_TakeDiag(SpeedKf* kf, SpeedKfDiagChannel channel, float* nis, float* autocorr) {
  SpeedKfDiag* d = &kf->diag[channel];
  *nis = (d->n > 0) ? d->nis_sum / (float)d->n : 0.0f;
  *autocorr = (d->n_ac > 0) ? d->ac_sum / (float)d->n_ac : 0.0f;
  d->n = 0;
  d->n_ac = 0;
  d->nis_sum = 0.0f;
  d->ac_sum = 0.0f;
}
