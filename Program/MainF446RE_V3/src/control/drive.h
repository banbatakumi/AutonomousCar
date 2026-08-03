#ifndef DRIVE_H_
#define DRIVE_H_

#include <stdbool.h>

#include "encoder.h"
#include "imu.h"
#include "lpf.h"
#include "motors.h"
#include "pid.h"
#include "steering.h"
#include "timer.h"

// 後輪2モータをトルク制御し、車速をこのマイコン側で閉ループ制御する。
//
// 制御構造 (カスケード):
//   [目標車速] → 車速PI (このモジュール, メインループ周期) → 総駆動トルク
//                                                          ↓ 左右配分 + トルクベクタリング項
//                                          左輪トルク ← TCリミッタ → 右輪トルク
//                                                          ↓ UART
//                                          [MDの電流ループ (kHz)] ← 内側ループはMD側
//
// 左右へは等トルクを配分するため、旋回時の内外輪速度差は各輪が自然に見つける (オープンデフ相当)。
// 左右輪へ独立に速度ループを掛ける方式と違い、タイヤ径誤差やアッカーマンモデル誤差による
// 内部トルクの循環 (片輪が押して片輪が引く状態) が原理的に発生しない。
//
// 車速の真値は非駆動輪である前輪エンコーダから得るため、駆動輪速度との比較でスリップ率が
// 直接計算でき、トラクションコントロール (TC) が成立する。

// ===========================================================================
// 車両パラメータ ★実機の実測値に必ず差し替えること★
// ===========================================================================
#define DRIVE_FRONT_WHEEL_RADIUS_M 0.030f  // 前輪 (非駆動輪) の有効転がり半径 [m]
#define DRIVE_REAR_WHEEL_RADIUS_M 0.030f   // 後輪 (駆動輪) の有効転がり半径 [m]
#define DRIVE_GEAR_RATIO 1.0f              // モータ回転数 / 後輪回転数 (ダイレクトドライブなら 1.0)
#define DRIVE_WHEELBASE_M 0.230f           // 前後車軸間距離 [m]
#define DRIVE_REAR_TRACK_M 0.155f          // 後輪左右間距離 (トレッド) [m]

// 各センサ・アクチュエータの符号。左右のモータ/エンコーダは鏡像に取り付くため、
// 「車体前進方向を正」に揃えるための符号をここで吸収する (実機で1輪ずつ回して確認すること)。
#define DRIVE_REAR_LEFT_DIR (-1.0f)
#define DRIVE_REAR_RIGHT_DIR (+1.0f)
#define DRIVE_FRONT_LEFT_DIR (+1.0f)
#define DRIVE_FRONT_RIGHT_DIR (-1.0f)

// ===========================================================================
// 制御パラメータ
// ===========================================================================
#define DRIVE_SPEED_KP 0.1f   // 車速PIの比例ゲイン [Nm / (m/s)]
#define DRIVE_SPEED_KI 0.25f  // 車速PIの積分ゲイン [Nm / (m/s) / s]
#define DRIVE_SPEED_KD 0.0f   // 車速は微分ノイズが乗りやすいため既定では使わない

#define DRIVE_MAX_TORQUE_NM 0.1f     // 1輪あたりの指令トルク上限 [Nm] (プロトコル上の絶対上限は ±3.2767)
#define DRIVE_MAX_SPEED_M_S 3.0f     // これを超えたら正トルクを出さない (暴走時の最終防壁)
#define DRIVE_ANTIWINDUP_TT_S 0.10f  // TC/リミッタで飽和したときに積分を巻き戻す時定数 [s]

// 停車保持: 目標車速がほぼ0かつ実車速もほぼ0のとき、トルク制御では保持剛性が無いため制動モードに切り替える
#define DRIVE_STANDSTILL_SPEED_M_S 0.05f
#define DRIVE_STANDSTILL_BRAKE_NM 0.10f

// ===========================================================================
// トラクションコントロール (TC) パラメータ
// ===========================================================================
#define DRIVE_TC_SLIP_THRESHOLD 0.12f  // これを超えるスリップ率からトルクを削り始める
#define DRIVE_TC_CUT_GAIN 0.5f         // 超過スリップ率あたりのトルク削減速度 [Nm/s]
#define DRIVE_TC_RECOVER_RATE 0.1f     // グリップ回復後にトルク上限を戻す速度 [Nm/s]
#define DRIVE_TC_MIN_TORQUE_NM 0.01f   // 削り切っても完全には0にしない (再加速できなくなるため)
#define DRIVE_TC_MIN_SPEED_M_S 0.20f   // これ以下の車速ではスリップ率が発散するのでTCを効かせない

// ===========================================================================
// フィルタ係数 (Drive_Update の呼び出し周期に依存する。1kHz 前提)
// ===========================================================================
#define DRIVE_LPF_K_FRONT 0.995f  // 前輪エンコーダ速度 (ADC量子化ノイズが大きいので強めに)
#define DRIVE_LPF_K_REAR 0.90f    // 後輪モータ速度 (MD側で既にフィルタ済みのため弱め)

typedef struct {
  Motors* motors;
  Encoder* encoder;
  Steering* steering;
  Imu* imu;  // ヨーレート取得用 (NULL可。その場合は自転車モデルで代用する)

  PID speed_pid;
  LPF lpf_front_left;
  LPF lpf_front_right;
  LPF lpf_rear_left;
  LPF lpf_rear_right;
  Timer timer;

  bool enabled;
  float target_speed_m_s;
  float yaw_moment_torque_nm;  // トルクベクタリング項 (正 = 左旋回方向のヨーモーメント)

  // --- 以下は Drive_Update が更新する観測量 (デバッグ・上位への報告用) ---
  float vehicle_speed_m_s;     // 前輪から推定した車体前後方向の速度
  float yaw_rate_rad_s;        // スリップ率の基準速度を作るのに使ったヨーレート
  float rear_speed_left_m_s;   // 左後輪の周速
  float rear_speed_right_m_s;  // 右後輪の周速
  float slip_left;             // 左後輪のスリップ率 (正 = 空転, 負 = ロック傾向)
  float slip_right;            // 右後輪のスリップ率
  float tc_limit_left_nm;      // TCが動的に決めた左輪のトルク上限
  float tc_limit_right_nm;     // TCが動的に決めた右輪のトルク上限
  float torque_left_nm;        // 実際にMDへ送った左輪トルク指令
  float torque_right_nm;       // 実際にMDへ送った右輪トルク指令
} Drive;

/**
 * @brief 駆動制御を初期化する。imu は NULL でもよい (旋回時のスリップ率補正精度が落ちる)。
 * 初期化直後は無効状態 (トルク指令を出さない)。
 */
void Drive_Init(Drive* obj, Motors* motors, Encoder* encoder, Steering* steering, Imu* imu);

/**
 * @brief 車速制御・TC を1周期分実行し、後輪MDへのトルク指令を更新する。
 * 実際の送信は Motors_Update が行うため、本関数の後に Motors_Update を呼ぶこと。
 * フィルタ係数が周期に依存するため、一定周期 (1kHz 想定) で呼ぶこと。
 */
void Drive_Update(Drive* obj);

/**
 * @brief 目標車速 [m/s] を設定する (負値で後退)。
 */
void Drive_SetTargetSpeed(Drive* obj, float m_s);

/**
 * @brief トルクベクタリングのヨーモーメント指令 [Nm] を設定する。
 * 正で左旋回方向 (右輪のトルクを増やし左輪を減らす)。左右の総和は変えないため車速に影響しない。
 */
void Drive_SetYawMomentTorque(Drive* obj, float nm);

/**
 * @brief 駆動制御を有効化する。積分項とTCのトルク上限をリセットしてから開始する。
 */
void Drive_Enable(Drive* obj);

/**
 * @brief 駆動制御を無効化し、トルク指令の送信を止める (MD側は無通信0.5秒で停止モード = 惰行)。
 */
void Drive_Disable(Drive* obj);

/**
 * @brief 駆動制御が有効かを取得する。
 */
bool Drive_IsEnabled(const Drive* obj);

/**
 * @brief 前輪エンコーダから推定した車速 [m/s] を取得する。
 */
float Drive_GetVehicleSpeed(const Drive* obj);

/**
 * @brief 左後輪のスリップ率を取得する (正 = 空転、負 = ロック傾向)。
 */
float Drive_GetSlipLeft(const Drive* obj);

/**
 * @brief 右後輪のスリップ率を取得する。
 */
float Drive_GetSlipRight(const Drive* obj);

/**
 * @brief いずれかの後輪でTCが介入中かを取得する。
 */
bool Drive_IsTractionControlActive(const Drive* obj);

/**
 * @brief 実際にMDへ送った左輪トルク指令 [Nm] を取得する。
 */
float Drive_GetTorqueLeft(const Drive* obj);

/**
 * @brief 実際にMDへ送った右輪トルク指令 [Nm] を取得する。
 */
float Drive_GetTorqueRight(const Drive* obj);

#endif  // DRIVE_H_
