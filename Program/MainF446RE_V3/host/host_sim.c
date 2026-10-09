// src/control (Drive・TorqueVectoring) をホストでそのままコンパイルし、後輪+タイヤ+車体の
// モデルと閉ループで回す。使い方・モデルの中身は host/README.md。
//
// 呼び出すのは Pi 側リポジトリの tools/ctrl_tune (ctypes)。構造体・列の並びを変えたら
// HOST_SIM_ABI を上げ、tools/ctrl_tune/fw.py も揃えること。

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "drive.h"

#define HOST_SIM_ABI 3

// 入力 (1制御周期ごと) の列
enum {
  IN_MODE = 0,     // 0=車速指令 1=トルク指令 2=制動 3=DISARM (惰行)
  IN_VALUE,        // 目標車速 [m/s] / 1輪あたりのトルク [Nm] / 1輪あたりの制動トルク [Nm]
  IN_ACCEL_LIMIT,  // [m/s^2] (車速指令のとき。0 = ファームの上限)
  IN_STEER,        // 路面舵角 [rad]
  IN_MU_LEFT,      // 左後輪の路面μの倍率 (1 = 通常。片輪浮きは 0 に近い値)
  IN_MU_RIGHT,
  IN_TEST_MOMENT,  // tv_test_moment_nm へ入れる値 (NaN = 触らない)
  IN_FRONT_SCALE,  // 前輪エンコーダの読みの倍率 (1 = 正常、0 = 断線)
  IN_COUNT,
};

// 出力 (1制御周期ごと) の列
enum {
  OUT_T = 0,
  OUT_SPEED,        // 車速の真値 [m/s]
  OUT_WHEEL_LEFT,   // 後輪の周速の真値 [m/s]
  OUT_WHEEL_RIGHT,
  OUT_YAW_RATE,     // ヨーレートの真値 [rad/s]
  OUT_CMD_LEFT,     // ファームが MD へ送ったトルク指令 [Nm] (制動は負)
  OUT_CMD_RIGHT,
  OUT_FORCE_LEFT,   // タイヤの前後力 [N]
  OUT_FORCE_RIGHT,
  OUT_KAPPA_LEFT,   // スリップ率の真値
  OUT_KAPPA_RIGHT,
  OUT_TC_LIMIT_LEFT,
  OUT_TC_LIMIT_RIGHT,
  OUT_ABS_LIMIT,
  OUT_YAW_TARGET,   // 車両モデルの「舵で決まる定常のヨーレート」(左右の力の差が無いときの値)
  OUT_FW_SPEED,     // ファームが推定した車速
  OUT_FW_SLIP_LEFT,
  OUT_FW_SLIP_RIGHT,
  OUT_FLAGS,        // bit0=TC bit1=ABS bit2=TV bit3=片輪浮き対策 が介入中
  OUT_DISTANCE,     // 走行距離 [m]
  OUT_TORQUE_LEFT,  // モータが実際に出したトルク [Nm]
  OUT_TORQUE_RIGHT,
  OUT_ACCEL,        // 前後加速度 [m/s^2]
  OUT_FW_REAR_LEFT,   // ファームが見ている後輪の周速 (ローパス後。TELEMETRY の wheel_speed)
  OUT_FW_REAR_RIGHT,
  OUT_FW_YAW_RATE,    // ファームが見ているヨーレート (TELEMETRY の yaw_rate)
  OUT_FRONT_ODOM,     // 前輪の累積走行距離 (TELEMETRY の odom_dist。射影なし)
  OUT_COUNT,
};

typedef struct {
  // --- 車体・車輪 ---
  double mass_kg;
  double wheel_inertia_kgm2;    // 後輪1つ (ホイール+タイヤ+ロータ)
  double wheel_friction_nm;     // 後輪の回転の摩擦 (一定)
  double wheel_damping_nms;     // 同 (角速度に比例)
  double rolling_decel_m_s2;    // 転がり抵抗による減速度
  // --- タイヤ (後輪の前後力 F = D*sin(C*atan(B*κ))、D = μ*Fz) ---
  double mu;
  double fz_static_n;           // 静止時の後輪1つの荷重
  double load_transfer;         // 重心高/ホイールベース (加速で後輪に乗る荷重)
  double lateral_transfer;      // 重心高/トレッド × 後軸の分担 (旋回で外輪に乗る荷重)
  double rear_lateral_share;    // 横力のうち後軸が受け持つ割合
  double tyre_b;
  double tyre_c;
  double slip_speed_floor_m_s;  // スリップ率の分母の下限 (モデル側)
  // --- モータドライバ ---
  double md_delay_s;            // 指令→トルクのむだ時間
  double md_tau_s;              // 同 1次遅れ
  double md_brake_fade_rad_s;   // 制動は -tanh(ω/これ) で抜ける
  double md_speed_tau_s;        // MD が報告する角速度の遅れ
  // --- ヨー ---
  double yaw_inertia_kgm2;
  double yaw_damping;           // Iz*dr/dt = Mz - yaw_damping*(r - r_ss)/|v| [N m^2/rad]
  double steer_gain;
  double steer_gain_cubic;
  double lateral_accel_max_m_s2;
  // --- センサ ---
  double front_noise_rad_s;     // 前輪の角速度のノイズ (2kHz の1サンプルごと、標準偏差)
  double rear_noise_rad_s;
  double gyro_noise_rad_s;
  double gyro_tau_s;
  double gyro_bias_rad_s;
  double front_scale_error;     // 前輪の速度の倍率誤差 (タイヤ径の誤差。-0.03 = 3%遅く読む)
  double front_ripple;          // 前輪エンコーダの1回転周期の誤差 (速度に対する振幅の割合)
  uint32_t seed;
  uint32_t reserved;
} HostPlant;

typedef struct {
  double dt_s;          // 制御周期 (ファームは 0.0005)
  int32_t substeps;     // 1制御周期あたりの車両モデルの刻み数
  int32_t tc_enabled;
  int32_t abs_enabled;
  int32_t wheel_lift_guard_enabled;
  int32_t tv_enabled;
  int32_t imu_ready;
  double initial_speed_m_s;
} HostConfig;

typedef struct {
  uint16_t id;
  float value;
} HostParam;

double g_host_now_s = 0.0;
static float g_steer_rad = 0.0f;

float Steering_GetRoadWheelAngleRad(const Steering* obj) {
  (void)obj;
  return g_steer_rad;
}

int host_sim_abi(void) { return HOST_SIM_ABI; }
int host_sim_in_count(void) { return IN_COUNT; }
int host_sim_out_count(void) { return OUT_COUNT; }
int host_sim_plant_size(void) { return (int)sizeof(HostPlant); }
int host_sim_has_params(void) {
#ifdef HOST_FW_HAS_CONTROL_PARAMS
  return 1;
#else
  return 0;
#endif
}

// --- 乱数 (xorshift64* + Box-Muller) ---
typedef struct {
  uint64_t s;
  int has_spare;
  double spare;
} Rng;

static double RngUniform(Rng* r) {
  r->s ^= r->s >> 12;
  r->s ^= r->s << 25;
  r->s ^= r->s >> 27;
  return (double)((r->s * 2685821657736338717ULL) >> 11) / 9007199254740992.0;
}

static double RngNormal(Rng* r) {
  if (r->has_spare) {
    r->has_spare = 0;
    return r->spare;
  }
  double u = RngUniform(r);
  double v = RngUniform(r);
  if (u < 1e-300) u = 1e-300;
  double m = sqrt(-2.0 * log(u));
  r->spare = m * sin(6.283185307179586 * v);
  r->has_spare = 1;
  return m * cos(6.283185307179586 * v);
}

static double Max(double a, double b) { return a > b ? a : b; }

#define MD_DELAY_MAX 256

typedef struct {
  HostMdMode mode[MD_DELAY_MAX];
  double command[MD_DELAY_MAX];
  int head;
  double torque_nm;  // 実際に出ているトルク (車体前進方向が正)
} MdModel;

// 後輪1つのタイヤの前後力。kappa (スリップ率の真値) も返す
static double TyreForce(const HostPlant* p, double wheel_rim_m_s, double centre_m_s, double fz_n,
                        double mu_scale, double fy_n, double* kappa_out) {
  double denom = Max(fabs(centre_m_s), p->slip_speed_floor_m_s);
  double kappa = (wheel_rim_m_s - centre_m_s) / denom;
  *kappa_out = kappa;
  double d = p->mu * mu_scale * Max(fz_n, 0.0);
  // 摩擦円: 横力に使っている分だけ前後力の上限が下がる (最低でも5%は残す)
  double fx_max2 = d * d - fy_n * fy_n;
  double floor2 = 0.0025 * d * d;
  double fx_max = sqrt(fx_max2 > floor2 ? fx_max2 : floor2);
  return fx_max * sin(p->tyre_c * atan(p->tyre_b * kappa));
}

// シミュレーション1本ぶんの状態。host_sim_create() で作り、host_sim_step() で続きから進める
// (Pi 側の planner を 10Hz で閉ループにするとき、区切りごとに呼ぶ)
typedef struct {
  HostPlant plant;
  HostConfig cfg;
  Motors motors;
  Encoder encoder;
  Steering steering;
  Imu imu;
  Drive drive;
#ifdef HOST_FW_HAS_CONTROL_PARAMS
  ControlParams control;
#endif
  Rng rng;
  double now_s;
  long step;          // 進めた制御周期の数
  int delay_steps;
  double v;
  double omega[2];    // [左, 右]
  double yaw;
  double distance;
  double front_odom;
  double accel_x;     // 荷重移動に使う前後加速度 (代数ループを避けるため少し遅らせる)
  double md_speed[2];
  double gyro;
  double front_angle; // 前輪の回転角 (周期誤差の位相)
  MdModel md[2];
} HostSim;

static void ApplyEnables(HostSim* s) {
  Drive_SetTractionControlEnabled(&s->drive, s->cfg.tc_enabled != 0);
  Drive_SetAbsEnabled(&s->drive, s->cfg.abs_enabled != 0);
  Drive_SetWheelLiftGuardEnabled(&s->drive, s->cfg.wheel_lift_guard_enabled != 0);
  Drive_SetTorqueVectoringEnabled(&s->drive, s->cfg.tv_enabled != 0);
}

// 1制御周期。u = 入力の1行、o = 出力の1行 (NULL なら捨てる)。advance が偽なら車両を進めない
// (初速にファームのローパスを馴染ませる助走)
static void StepOnce(HostSim* s, const double* u, double* o, int advance) {
  const HostPlant* p = &s->plant;
  const HostConfig* cfg = &s->cfg;
  const double r_wheel = DRIVE_REAR_WHEEL_RADIUS_M;
  const double half_track = DRIVE_REAR_TRACK_M * 0.5;
  const double h = cfg->dt_s / cfg->substeps;

  double steer = u[IN_STEER];
  double front_scale = u[IN_FRONT_SCALE];
  double cos_steer = cos(steer);
  if (fabs(cos_steer) < 0.1) cos_steer = 0.1;

  // --- センサ ---
  double front_true = s->v / cos_steer / DRIVE_FRONT_WHEEL_RADIUS_M;
  if (advance) {
    s->front_angle += front_true * cfg->dt_s;
    s->front_odom += s->v / cos_steer * front_scale * (1.0 + p->front_scale_error) * cfg->dt_s;
  }
  double front_omega = front_true * front_scale * (1.0 + p->front_scale_error);
  // 左右で位相の違う周期誤差 (絶対角エンコーダの取付偏心)。左右の平均には半分ほど残る
  s->encoder.angular_velocity_left =
      (float)((front_omega * (1.0 + p->front_ripple * sin(s->front_angle)) +
               p->front_noise_rad_s * RngNormal(&s->rng)) * DRIVE_FRONT_LEFT_DIR);
  s->encoder.angular_velocity_right =
      (float)((front_omega * (1.0 + p->front_ripple * sin(s->front_angle + 1.3)) +
               p->front_noise_rad_s * RngNormal(&s->rng)) * DRIVE_FRONT_RIGHT_DIR);
  s->motors.rear_left.speed_rad_s =
      (float)((s->md_speed[0] + p->rear_noise_rad_s * RngNormal(&s->rng)) * DRIVE_REAR_LEFT_DIR);
  s->motors.rear_right.speed_rad_s =
      (float)((s->md_speed[1] + p->rear_noise_rad_s * RngNormal(&s->rng)) * DRIVE_REAR_RIGHT_DIR);
  s->imu.data.gyro_z =
      (float)((s->gyro + p->gyro_bias_rad_s + p->gyro_noise_rad_s * RngNormal(&s->rng)) * 57.29577951308232);

  // --- ファーム ---
  // Timer・舵角はファイル内の共有変数なので、呼ぶたびにこのシミュレーションのものへ差し替える
  s->now_s += cfg->dt_s;
  g_host_now_s = s->now_s;
  g_steer_rad = (float)steer;
  int mode = advance ? (int)(u[IN_MODE] + 0.5) : 3;
  if (mode == 3) {
    if (Drive_IsEnabled(&s->drive)) Drive_Disable(&s->drive);
  } else {
    if (!Drive_IsEnabled(&s->drive)) {
      Drive_Enable(&s->drive);
      // Drive_Enable は目標車速を 0 から始める (停止から ARM する前提)。初速のある場面では
      // 「その速度で走っていた」ことにする。そうしないと最初に目標 0 へ向けて制動してしまう
      if (s->step == 0 && mode == 0) s->drive.target_speed_m_s = (float)cfg->initial_speed_m_s;
    }
    Drive_SetTargetSpeed(&s->drive, mode == 0 ? (float)u[IN_VALUE] : 0.0f, (float)u[IN_ACCEL_LIMIT]);
    Drive_SetTorque(&s->drive, mode == 1, mode == 1 ? (float)u[IN_VALUE] : 0.0f);
    Drive_SetBrake(&s->drive, mode == 2, mode == 2 ? (float)u[IN_VALUE] : 0.0f);
  }
#ifdef HOST_FW_HAS_CONTROL_PARAMS
  if (u[IN_TEST_MOMENT] == u[IN_TEST_MOMENT]) {
    s->control.tv_test_moment_nm = (float)u[IN_TEST_MOMENT];
    Drive_SetParams(&s->drive, &s->control);
  }
#endif
  Drive_Update(&s->drive);
  if (!advance) return;

  // --- MD: 指令をむだ時間ぶん遅らせる ---
  BldcMotor* motor[2] = {&s->motors.rear_left, &s->motors.rear_right};
  const double dir[2] = {DRIVE_REAR_LEFT_DIR, DRIVE_REAR_RIGHT_DIR};
  HostMdMode md_mode[2];
  double md_command[2];
  for (int i = 0; i < 2; i++) {
    MdModel* md = &s->md[i];
    md->mode[md->head] = motor[i]->mode;
    md->command[md->head] = motor[i]->command;
    int tail = (md->head + MD_DELAY_MAX - s->delay_steps) % MD_DELAY_MAX;
    md_mode[i] = md->mode[tail];
    md_command[i] = md->command[tail];
    md->head = (md->head + 1) % MD_DELAY_MAX;
  }

  // --- 車両 ---
  double mu_scale[2] = {u[IN_MU_LEFT], u[IN_MU_RIGHT]};
  double force[2] = {0.0, 0.0};
  double kappa[2] = {0.0, 0.0};
  double yaw_ss = 0.0;
  for (int k = 0; k < cfg->substeps; k++) {
    double a_y = s->v * s->yaw;
    double fy = p->mass_kg * a_y * p->rear_lateral_share * 0.5;
    double fz_long = p->fz_static_n + p->load_transfer * p->mass_kg * s->accel_x * 0.5;
    double fz_lat = p->lateral_transfer * p->mass_kg * a_y;  // 左旋回 (a_y>0) で右輪に乗る
    double fz[2] = {fz_long - fz_lat, fz_long + fz_lat};
    double centre[2] = {s->v - s->yaw * half_track, s->v + s->yaw * half_track};

    for (int i = 0; i < 2; i++) {
      double target = 0.0;
      if (md_mode[i] == HOST_MD_TORQUE) {
        target = md_command[i] * dir[i];
        if (target > motor[i]->limit_nm) target = motor[i]->limit_nm;
        if (target < -motor[i]->limit_nm) target = -motor[i]->limit_nm;
      } else if (md_mode[i] == HOST_MD_BRAKE) {
        target = -md_command[i] * tanh(s->omega[i] / p->md_brake_fade_rad_s);
      } else if (md_mode[i] == HOST_MD_POSITION) {
        target = -motor[i]->limit_nm * tanh(s->omega[i] / p->md_brake_fade_rad_s);
      }
      s->md[i].torque_nm += (target - s->md[i].torque_nm) * (h / (p->md_tau_s + h));

      force[i] = TyreForce(p, s->omega[i] * r_wheel, centre[i], fz[i], mu_scale[i], fy, &kappa[i]);
      double friction = p->wheel_friction_nm * tanh(s->omega[i] / 1.0) + p->wheel_damping_nms * s->omega[i];
      s->omega[i] += (s->md[i].torque_nm - r_wheel * force[i] - friction) / p->wheel_inertia_kgm2 * h;
      s->md_speed[i] += (s->omega[i] - s->md_speed[i]) * (h / (p->md_speed_tau_s + h));
    }

    double drive_accel = (force[0] + force[1]) / p->mass_kg;
    double rolling = p->rolling_decel_m_s2 * tanh(s->v / 0.02);
    double a = drive_accel - rolling;
    s->v += a * h;
    s->distance += s->v * h;
    s->accel_x += (a - s->accel_x) * (h / (0.005 + h));

    // ヨー: 舵で決まる定常のヨーレートへ戻ろうとする減衰 + 左右の駆動力差のモーメント
    double eff = p->steer_gain * steer + p->steer_gain_cubic * steer * steer * steer;
    yaw_ss = s->v * tan(eff) / DRIVE_WHEELBASE_M;
    double speed_abs = Max(fabs(s->v), 0.3);
    double yaw_cap = p->lateral_accel_max_m_s2 / speed_abs;
    if (yaw_ss > yaw_cap) yaw_ss = yaw_cap;
    if (yaw_ss < -yaw_cap) yaw_ss = -yaw_cap;
    double moment = (force[1] - force[0]) * half_track;
    s->yaw += (moment - p->yaw_damping * (s->yaw - yaw_ss) / speed_abs) / p->yaw_inertia_kgm2 * h;
    s->gyro += (s->yaw - s->gyro) * (h / (p->gyro_tau_s + h));
  }
  s->step++;
  if (o == NULL) return;

  const Drive* d = &s->drive;
  o[OUT_T] = s->step * cfg->dt_s;
  o[OUT_SPEED] = s->v;
  o[OUT_WHEEL_LEFT] = s->omega[0] * r_wheel;
  o[OUT_WHEEL_RIGHT] = s->omega[1] * r_wheel;
  o[OUT_YAW_RATE] = s->yaw;
  o[OUT_CMD_LEFT] = Drive_GetTorqueLeft(d);
  o[OUT_CMD_RIGHT] = Drive_GetTorqueRight(d);
  o[OUT_FORCE_LEFT] = force[0];
  o[OUT_FORCE_RIGHT] = force[1];
  o[OUT_KAPPA_LEFT] = kappa[0];
  o[OUT_KAPPA_RIGHT] = kappa[1];
  o[OUT_TC_LIMIT_LEFT] = Drive_GetTcLimitLeft(d);
  o[OUT_TC_LIMIT_RIGHT] = Drive_GetTcLimitRight(d);
  o[OUT_ABS_LIMIT] = d->abs_limit_nm;
  o[OUT_YAW_TARGET] = yaw_ss;
  o[OUT_FW_SPEED] = Drive_GetVehicleSpeed(d);
  o[OUT_FW_SLIP_LEFT] = Drive_GetSlipLeft(d);
  o[OUT_FW_SLIP_RIGHT] = Drive_GetSlipRight(d);
  o[OUT_FLAGS] = (Drive_IsTractionControlActive(d) ? 1 : 0) | (Drive_IsAbsActive(d) ? 2 : 0) |
                 (Drive_IsTorqueVectoringActive(d) ? 4 : 0) | (Drive_IsWheelLiftGuardActive(d) ? 8 : 0);
  o[OUT_DISTANCE] = s->distance;
  o[OUT_TORQUE_LEFT] = s->md[0].torque_nm;
  o[OUT_TORQUE_RIGHT] = s->md[1].torque_nm;
  o[OUT_ACCEL] = s->accel_x;
  o[OUT_FW_REAR_LEFT] = d->rear_speed_left_m_s;
  o[OUT_FW_REAR_RIGHT] = d->rear_speed_right_m_s;
  o[OUT_FW_YAW_RATE] = d->yaw_rate_rad_s;
  o[OUT_FRONT_ODOM] = s->front_odom;
}

/**
 * シミュレーションを1本作る。params は調整パラメータ (param_id と値。古いファームでは無視)。
 * 失敗 (設定の誤り・知らない param_id) なら NULL。
 */
HostSim* host_sim_create(const HostPlant* p, const HostConfig* cfg, const HostParam* params,
                         int param_count) {
  if (cfg->substeps < 1 || cfg->dt_s <= 0.0) return NULL;
  int delay_steps = (int)(p->md_delay_s / cfg->dt_s + 0.5);
  if (delay_steps >= MD_DELAY_MAX) return NULL;

  HostSim* s = (HostSim*)calloc(1, sizeof(HostSim));
  if (s == NULL) return NULL;
  s->plant = *p;
  s->cfg = *cfg;
  s->delay_steps = delay_steps;
  s->motors.rear_left.data_valid = true;
  s->motors.rear_right.data_valid = true;
  s->imu.ready = cfg->imu_ready != 0;

  g_host_now_s = 0.0;
  g_steer_rad = 0.0f;
  Drive_Init(&s->drive, &s->motors, &s->encoder, &s->steering, &s->imu);
#ifdef HOST_FW_HAS_CONTROL_PARAMS
  ControlParams_SetDefaults(&s->control);
  for (int i = 0; i < param_count; i++) {
    float applied;
    if (ControlParams_Set(&s->control, params[i].id, params[i].value, &applied) == CONTROL_PARAM_UNKNOWN) {
      free(s);
      return NULL;
    }
  }
  Drive_SetParams(&s->drive, &s->control);
#else
  (void)params;
  (void)param_count;
#endif
  ApplyEnables(s);

  s->rng.s = 0x9E3779B97F4A7C15ULL ^ ((uint64_t)p->seed * 0xD1B54A32D192ED03ULL + 1ULL);
  s->v = cfg->initial_speed_m_s;
  s->omega[0] = s->omega[1] = s->v / DRIVE_REAR_WHEEL_RADIUS_M;
  s->md_speed[0] = s->md_speed[1] = s->omega[0];

  // ファームのローパスを初速に馴染ませる (この間は車両を進めない)
  double idle[IN_COUNT] = {0};
  idle[IN_MODE] = 3;
  idle[IN_MU_LEFT] = idle[IN_MU_RIGHT] = 1.0;
  idle[IN_TEST_MOMENT] = NAN;
  idle[IN_FRONT_SCALE] = 1.0;
  for (int k = 0; k < 200; k++) StepOnce(s, idle, NULL, 0);
  return s;
}

void host_sim_free(HostSim* s) { free(s); }

/** n 制御周期ぶん進める。in は n×IN_COUNT、out は n×OUT_COUNT。 */
void host_sim_step(HostSim* s, int n, const double* in, double* out) {
  for (int k = 0; k < n; k++) StepOnce(s, &in[k * IN_COUNT], &out[k * OUT_COUNT], 1);
}

/** 機能の ON/OFF を途中で切り替える (上位の CONFIG_SET に相当)。 */
void host_sim_set_enables(HostSim* s, int tc, int abs, int wheel_lift_guard, int tv) {
  s->cfg.tc_enabled = tc;
  s->cfg.abs_enabled = abs;
  s->cfg.wheel_lift_guard_enabled = wheel_lift_guard;
  s->cfg.tv_enabled = tv;
  ApplyEnables(s);
}

/** 調整パラメータを途中で変える。知らない param_id・古いファームなら -1。 */
int host_sim_set_param(HostSim* s, uint16_t id, float value) {
#ifdef HOST_FW_HAS_CONTROL_PARAMS
  float applied;
  if (ControlParams_Set(&s->control, id, value, &applied) == CONTROL_PARAM_UNKNOWN) return -1;
  Drive_SetParams(&s->drive, &s->control);
  return 0;
#else
  (void)s;
  (void)id;
  (void)value;
  return -1;
#endif
}

/** 作って n 周期回して捨てる。0 = 成功。 */
int host_sim_run(const HostPlant* p, const HostConfig* cfg, int n, const double* in, double* out,
                 const HostParam* params, int param_count) {
  HostSim* s = host_sim_create(p, cfg, params, param_count);
  if (s == NULL) return -1;
  host_sim_step(s, n, in, out);
  host_sim_free(s);
  return 0;
}
