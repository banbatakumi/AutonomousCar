#ifndef AHRS_H_
#define AHRS_H_

// 6軸 (ジャイロ + 加速度) の Mahony 相補フィルタによる姿勢推定。
// ジャイロ積分をベースに、重力方向 (加速度) で roll/pitch のドリフトを補正する。
// 地磁気センサがないため yaw に絶対基準はなく、起動時姿勢からの相対角として
// 少しずつドリフトする (ジャイロのバイアス補正精度がそのまま yaw の精度になる)。
//
// HAL に依存しないので、そのままホスト側でのシミュレーションにも使える。

#define AHRS_GRAVITY_MPS2 9.80665f

// 加速度による姿勢補正を許可する加速度ノルムの範囲 [m/s^2]。
// 車体の加減速や振動が乗ったサンプルは「重力方向の観測」として使えないため、
// この範囲を外れたサンプルではジャイロ積分のみで姿勢を進める。
#define AHRS_ACCEL_GATE_MIN 9.0f
#define AHRS_ACCEL_GATE_MAX 10.6f

// 積分フィードバック (ジャイロ残留バイアス補正量) の飽和上限 [deg/s]。
// 呼び出し側 (imu.c) が静止時ジャイロバイアスを別途追従・除去しているため、ここで
// 吸収すべきは取付誤差やその追従誤差など「除去しきれずに残る小さなバイアス」のみで
// よい。MPU6050データシート上のZRO許容誤差 (最大±20dps程度) よりはるかに絞った値にし、
// センサ取付誤差の残留バイアス等で姿勢誤差が長時間残る異常時でも際限なく蓄積せず、
// 誤差反転時のオーバーシュート・発振を防ぐ
#define AHRS_INTEGRAL_FB_LIMIT_DPS 5.0f

typedef struct {
  float q0, q1, q2, q3;  // 姿勢クォータニオン (機体 -> 慣性)
  float two_kp;          // 比例ゲイン x2 (大きいほど加速度を信用する)
  float two_ki;          // 積分ゲイン x2 (ジャイロ残留バイアスの吸収。0 で無効)
  float integral_fb_x, integral_fb_y, integral_fb_z;
  float yaw;    // [deg] -180..180
  float pitch;  // [deg] -90..90
  float roll;   // [deg] -180..180
} Ahrs;

/**
 * @brief フィルタを初期化する (姿勢は水平・yaw=0 で開始)。
 * @param kp 加速度フィードバックの比例ゲイン。大きいほど収束が速いが加減速の影響を受けやすい。
 * @param ki ジャイロ残留バイアスを吸収する積分ゲイン。0 で無効。
 */
void Ahrs_Init(Ahrs *obj, float kp, float ki);

/**
 * @brief ゲインを変更する。
 */
void Ahrs_SetGains(Ahrs *obj, float kp, float ki);

/**
 * @brief 静止時の加速度ベクトルから初期姿勢 (roll/pitch) を直接与える。yaw は 0 になる。
 * 起動直後に呼ぶと、水平へ収束するまでの数秒を待たずに済む。
 */
void Ahrs_SetFromAccel(Ahrs *obj, float ax, float ay, float az);

/**
 * @brief 1 サンプル分だけ姿勢を更新する。
 * @param gx_dps,gy_dps,gz_dps 角速度 [deg/s] (機体座標系)
 * @param ax,ay,az 加速度 [m/s^2] (機体座標系)。ノルムで重力ゲート判定を行うため必ず物理単位で渡すこと。
 * @param dt 前サンプルからの経過時間 [s]
 */
void Ahrs_Update(Ahrs *obj, float gx_dps, float gy_dps, float gz_dps, float ax,
                 float ay, float az, float dt);

/**
 * @brief 推定姿勢から求めた重力方向の単位ベクトル (機体座標系) を取得する。
 * 加速度から重力成分を除いて並進加速度を取り出すのに使う。
 */
void Ahrs_GetGravityDirection(const Ahrs *obj, float *gx, float *gy, float *gz);

#endif  // AHRS_H_
