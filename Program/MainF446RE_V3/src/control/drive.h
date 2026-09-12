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
#include "torque_vectoring.h"

// 後輪2モータをトルク制御し、車速をこのマイコン側で閉ループ制御する。
//
// 制御構造 (カスケード):
//   [目標車速] → 車速PI (このモジュール, メインループ周期) → 総駆動トルク
//                                                          ↓ 左右配分 + トルクベクタリング項
//                                          左輪トルク ← TCリミッタ → 右輪トルク
//                                                          ↓ UART
//                                          [MDの電流ループ (kHz)] ← 内側ループはMD側
//
// 左右の配分は等トルクを基準とし、そこへトルクベクタリング (src/control/torque_vectoring) が
// 決めた左右差だけを重ねる。総和は変わらないので車速制御とは干渉しない。差を付けない限りは
// 旋回時の内外輪速度差を各輪が自然に見つける (オープンデフ相当) 挙動になり、左右輪へ独立に
// 速度ループを掛ける方式と違ってタイヤ径誤差やアッカーマンモデル誤差による内部トルクの循環
// (片輪が押して片輪が引く状態) が原理的に発生しない。
//
// 車速の真値は非駆動輪である前輪エンコーダから得るため、駆動輪速度との比較でスリップ率が
// 直接計算でき、トラクションコントロール (TC) が成立する。

// ===========================================================================
// 車両パラメータ (実測値)
//
// 後輪は減速機を挟まないダイレクトドライブのため、モータ角速度がそのまま車輪角速度になる。
// 減速比の定数を置いていないのはこのため (減速機を入れたら車輪速の換算に比を掛けること)。
// ===========================================================================
#define DRIVE_FRONT_WHEEL_RADIUS_M 0.030f  // 前輪 (非駆動輪) の有効転がり半径 [m] (直径60mm)
#define DRIVE_REAR_WHEEL_RADIUS_M 0.030f   // 後輪 (駆動輪) の有効転がり半径 [m] (直径60mm)
#define DRIVE_WHEELBASE_M 0.230f           // 前後車軸間距離 [m]
#define DRIVE_REAR_TRACK_M 0.155f          // 後輪左右間距離 (トレッド) [m]

// 半径は幾何寸法 (直径60mm) から入れてある。ゴムタイヤは荷重で潰れるぶん実際の転がり半径が
// 数%小さくなり、その誤差はそのまま車速とオドメトリの倍率誤差になる。精度が要るなら
// メジャーで測った距離を転がして Encoder_GetAccumAngleLeftMrad() の累積値から逆算すること。

// 各センサ・アクチュエータの符号。左右のモータ/エンコーダは鏡像に取り付くため、
// 「車体前進方向を正」に揃えるための符号をここで吸収する。
// ★未確認: 実機で1輪ずつ回して確認すること。符号を間違えると車が逆に走るだけでなく、
// TCがスリップ率を符号ごと誤読して、滑っていないのにトルクを削る (逆に足す) side に倒れる★
#define DRIVE_REAR_LEFT_DIR (-1.0f)
#define DRIVE_REAR_RIGHT_DIR (+1.0f)
#define DRIVE_FRONT_LEFT_DIR (-1.0f)
#define DRIVE_FRONT_RIGHT_DIR (+1.0f)

// ===========================================================================
// 制御パラメータ
// ===========================================================================
#define DRIVE_SPEED_KP 0.25f  // 車速PIの比例ゲイン [Nm / (m/s)]
#define DRIVE_SPEED_KI 0.5f   // 車速PIの積分ゲイン [Nm / (m/s) / s]
#define DRIVE_SPEED_KD 0.0f   // 車速は微分ノイズが乗りやすいため既定では使わない

// 1輪あたりのトルク上限 [Nm] (プロトコル上の絶対上限は ±3.2767)。
// この値は指令のクランプに使うと同時に MD 側のトルク上限としても設定するため、
// このマイコンのバグや通信異常で過大な指令が出ても最終段で頭打ちになる。
// 上位が指令できる制動トルク (DRIVE_MAX_BRAKE_TORQUE_NM) もこの上限を超えないこと
#define DRIVE_MAX_TORQUE_NM 0.15f
#define DRIVE_MAX_SPEED_M_S 5.0f  // これを超えたら正トルクを出さない (暴走時の最終防壁)
// 目標車速の変化をこの加速度でレート制限する (急な指令変化でタイヤを滑らせないため)。
// 上位 (Drive_SetTargetSpeed) が指定できる加速度制限の上限もこの値になる
#define DRIVE_MAX_ACCEL_M_S2 3.0f
#define DRIVE_ANTIWINDUP_TT_S 0.10f  // TC/リミッタで飽和したときに積分を巻き戻す時定数 [s]

// 停車: 目標車速がほぼ0かつ実車速もほぼ0のとき、指令を止めて自由回転させる (惰行)。
// 上位が明示的にブレーキ (RAS_CMD_FLAG_BRAKE) を指定しない限り、停車中も車両を押さえ込まない
#define DRIVE_STANDSTILL_SPEED_M_S 0.05f

// 上位から指令できる制動トルクの上限 [Nm]。MD側のトルク上限 (DRIVE_MAX_TORQUE_NM) を
// 超える値を送っても MD 側で頭打ちになるだけなので、指令の時点で同じ値に揃えておく
#define DRIVE_MAX_BRAKE_TORQUE_NM DRIVE_MAX_TORQUE_NM

// ===========================================================================
// トラクションコントロール (TC) パラメータ
// ===========================================================================
#define DRIVE_TC_SLIP_THRESHOLD 0.2f   // これを超えるスリップ率からトルクを削り始める
#define DRIVE_TC_CUT_GAIN 0.1f         // 超過スリップ率あたりのトルク削減速度 [Nm/s]
#define DRIVE_TC_RECOVER_RATE 0.1f     // グリップ回復後にトルク上限を戻す速度 [Nm/s]
#define DRIVE_TC_MIN_TORQUE_NM 0.005f  // 削り切っても完全には0にしない (再加速できなくなるため)
#define DRIVE_TC_MIN_SPEED_M_S 0.25f   // これ以下の車速ではスリップ率が発散するのでTCを効かせない
// スリップ率がしきい値を超えてから、実際にトルクを削り始めるまでの継続時間 [s]。
// DRIVE_LPF_K_FRONT_TC を軽くしてもなお残るノイズ由来の一瞬の超過を無視するためのデバウンス。
// 本物の空転は超過が持続するのでこの遅延はほぼ影響しない
#define DRIVE_TC_SLIP_DEBOUNCE_S 0.03f
// スリップ率がしきい値をわずかに下回ってから、継続時間 (excess_time_s) を実際に
// リセットするまでのホールドダウン [s]。これが無いとしきい値付近のノイズで一瞬でも
// 下回るたびにデバウンスの積み上げが0に戻り、本物の持続的な空転の検出が遅れうる
#define DRIVE_TC_SLIP_HOLD_DOWN_S 0.05f

// ===========================================================================
// 片輪浮き対策 (Wheel Lift Guard) パラメータ
// 上のTC (DRIVE_TC_*) は前輪基準速度に対する後輪個々のスリップ率で判定するため、
// 基準速度が DRIVE_TC_MIN_SPEED_M_S 未満の低速域では機能しない。停止/低速からの
// 片輪浮き急発進を捉えるため、前輪基準速度に依存しない「後輪左右速度差」で判定する
// 経路を独立に追加する。実車のeLSD (電子制御LSD) と同じ役割分担:
// 基準車速比較 (=上のTC) は両輪同時空転を、左右輪速度差 (=本機構) は片輪だけの
// 異常を、それぞれ担当する。TC本体とは独立に上位からON/OFFできる (RasConfig 参照)。
// ===========================================================================

// 後輪左右の速度差 (ヨーレートで期待される差を差し引いた異常成分) がこれを超えたら、
// 速い方 (浮いていると推定される輪) のトルク上限を削り始める [m/s]。
// ★未実測: 正常なコーナリング・段差通過時に生じる残差 (ヨーレート補正の誤差・センサ
// ノイズ) の最大値を実測し、それを上回る値に設定すること。当面は「確実に止める」側の
// 低めの値から始める★
#define DRIVE_WHEEL_LIFT_DIFF_THRESHOLD_M_S 0.35f

#define DRIVE_WHEEL_LIFT_CUT_GAIN 1.2f                       // 超過差分あたりのトルク削減速度 [Nm/s / (m/s)]
#define DRIVE_WHEEL_LIFT_RECOVER_RATE DRIVE_TC_RECOVER_RATE  // 上のTCと同じ回復速度

// 後輪周速がこれを超えたら、基準速度・左右差に関係なく即座にトルク上限を0にする
// (最終防波堤)。SlipRatio() は DRIVE_TC_MIN_SPEED_M_S 未満で無効化されるため、これは
// 左右速度差検知と違うレイヤーの保護として持たせてある。
// ★未実測: 最大舵角・最高速旋回時の外輪速度を実機で確認し、誤介入しない下限まで詰めること★
#define DRIVE_WHEEL_LIFT_MAX_WHEEL_SPEED_M_S (DRIVE_MAX_SPEED_M_S * 1.5f)  // 4.5 m/s

// ===========================================================================
// フィルタ係数 (Drive_Update の呼び出し周期に依存する。実際は 500us = 2kHz で呼ばれるが、
// 係数から逆算した帯域は約1.6Hzで結果的に妥当なため、値自体はそのままにしてある)
// ===========================================================================
#define DRIVE_LPF_K_FRONT 0.995f  // 前輪エンコーダ速度 (ADC量子化ノイズが大きいので強めに)。
                                  // 時定数 τ=-dt/ln(k)≈100ms。PIDフィードバック・上位報告用
#define DRIVE_LPF_K_REAR 0.90f    // 後輪モータ速度 (MD側で既にフィルタ済みのため弱め)。τ≈4.75ms
// TCのスリップ判定専用の前輪速度フィルタ。DRIVE_LPF_K_FRONT (τ≈100ms) をそのまま基準速度に
// 使うと、フル加速のようなランプ入力で τ×加速度 ぶん (最大加速度3.0m/s²なら約0.3m/s) 基準速度が
// 実速度より系統的に遅れ、後輪 (τ≈4.75ms、ほぼ遅れ無し) との差が「常時空転している」という
// 誤ったスリップ率を生む (低速ほど基準速度に対する遅れの比率が大きく致命的)。この遅れバイアスを
// 縮めるため、PID用より軽い τ≈25ms のフィルタをスリップ判定専用に別途持つ
// (ノイズは残りやすくなるが、DRIVE_TC_SLIP_DEBOUNCE_S 側で吸収する)
#define DRIVE_LPF_K_FRONT_TC 0.98f

typedef struct {
  Motors* motors;
  Encoder* encoder;
  Steering* steering;
  Imu* imu;  // ヨーレート取得用 (NULL可。その場合は自転車モデルで代用する)

  PID speed_pid;
  TorqueVectoring tv;
  LPF lpf_front_left;
  LPF lpf_front_right;
  LPF lpf_front_left_tc;   // スリップ判定専用 (DRIVE_LPF_K_FRONT_TC)
  LPF lpf_front_right_tc;  // スリップ判定専用 (DRIVE_LPF_K_FRONT_TC)
  LPF lpf_rear_left;
  LPF lpf_rear_right;
  Timer timer;

  bool enabled;
  bool tc_enabled;                // 既定は有効。無効時は tc_limit_left/right_nm を上限固定にする
  bool wheel_lift_guard_enabled;  // 既定は有効。TC本体とは独立にON/OFF可能
  // 上位から指令された生の目標車速 (レート制限前)。Drive_SetTargetSpeed が更新する
  float speed_setpoint_m_s;
  float accel_limit_m_s2;  // speed_setpoint_m_s へ向かう加速度の上限 [m/s^2]
  // レート制限後、実際にPIへ渡している目標車速。Drive_Update が毎周期 speed_setpoint_m_s
  // へ accel_limit_m_s2 で近づける
  float target_speed_m_s;
  bool brake_active;      // 真の間は車速制御を止めて制動トルクだけを出す
  float brake_torque_nm;  // 制動時に後輪各輪へ掛ける制動トルク [Nm] (常に正)
  // 真の間は車速PIを迂回し、manual_torque_nm を左右等配分の総駆動トルクとして直接使う
  // (TC/TV は掛けたまま。brake_active の方が優先される)
  bool torque_mode_active;
  float manual_torque_nm;  // torque_mode 中に指令された1輪あたりの駆動トルク [Nm] (負=後退方向)
  bool side_brake_active;   // 上位からの要求 (毎周期 Drive_SetSideBrake で更新)
  bool side_brake_engaged;  // 実際に位置保持へ入っているか (角度をラッチ済みか)
  float side_brake_target_left_rad;
  float side_brake_target_right_rad;

  // --- 以下は Drive_Update が更新する観測量 (デバッグ・上位への報告用) ---
  // 周速はすべて LPF 後の値。前輪の生の角速度は使わないこと。12bit ADC で 1回転を測るため
  // 1LSB = 2pi/4096 = 1.53mrad で、これを 500us で微分すると 1LSB あたり 3.07rad/s
  // (= 0.09m/s) に化ける。実測ではノイズが ±100mV 程度あり、静止中でも生値は ±12m/s 振れる
  float vehicle_speed_m_s;      // 前輪から推定した車体前後方向の速度 (舵角で射影済み)
  // 前後方向判定用の応答が速い車速 (EstimateVehicleSpeedForSlip 由来、τ≈25ms)。
  // vehicle_speed_m_s (τ≈100ms) では急減速時の符号反転検出が最大100ms遅れるため、
  // 自動停止の前後判定 (vehicle.c) 向けに分けて保持する
  float vehicle_speed_for_direction_m_s;
  float yaw_rate_rad_s;         // スリップ率の基準速度を作るのに使ったヨーレート
  bool yaw_rate_measured;       // 上がIMUの実測値か (偽 = 舵角からの幾何計算で代用中)
  float front_speed_left_m_s;   // 左前輪の周速 (射影前。車体速度ではなく車輪自身の軌跡上の速度)
  float front_speed_right_m_s;  // 右前輪の周速
  float rear_speed_left_m_s;    // 左後輪の周速
  float rear_speed_right_m_s;   // 右後輪の周速
  float slip_left;              // 左後輪のスリップ率 (正 = 空転, 負 = ロック傾向)
  float slip_right;             // 右後輪のスリップ率
  // slip_left/right がDRIVE_TC_SLIP_THRESHOLDを超えてからの継続時間 [s] (デバウンス用)。
  // 超過が途切れてから DRIVE_TC_SLIP_HOLD_DOWN_S 継続するまでは0に戻さない
  // (tc_slip_below_time_*_s 参照)
  float tc_slip_excess_time_left_s;
  float tc_slip_excess_time_right_s;
  // slip_left/right がDRIVE_TC_SLIP_THRESHOLD以下になってからの継続時間 [s]
  // (tc_slip_excess_time_*_s のホールドダウン用)。超過が再発したら0に戻る
  float tc_slip_below_time_left_s;
  float tc_slip_below_time_right_s;
  float tc_limit_left_nm;   // TCが動的に決めた左輪のトルク上限
  float tc_limit_right_nm;  // TCが動的に決めた右輪のトルク上限
  // 片輪浮き対策が動的に決めた各輪のトルク上限 (tc_limit_*とは独立、最終的にminを取る)
  float wheel_lift_limit_left_nm;
  float wheel_lift_limit_right_nm;
  // 実際にMDへ送った各輪のトルク指令。正 = 駆動、負 = 制動。制動モード (停車保持・
  // Drive_SetBrake) のときは制動トルクを負値として入れるので、符号を見れば駆動しているのか
  // 押さえているのかが上位から区別できる
  float torque_left_nm;
  float torque_right_nm;
} Drive;

/**
 * @brief 駆動制御を初期化する。imu は NULL でもよい (旋回時のスリップ率補正精度が落ちる)。
 * 初期化直後は無効状態 (トルク指令を出さない)。
 */
void Drive_Init(Drive* obj, Motors* motors, Encoder* encoder, Steering* steering, Imu* imu);

/**
 * @brief 車速制御・TC を1周期分実行し、後輪MDへのトルク指令を更新する。
 * 実際の送信は Motors_Update が行うため、本関数の後に Motors_Update を呼ぶこと。
 * フィルタ係数が周期に依存するため、一定周期 (500us = 2kHz 想定) で呼ぶこと。
 */
void Drive_Update(Drive* obj);

/**
 * @brief 目標車速 [m/s] を設定する (負値で後退)。急な指令変化でタイヤを滑らせないよう、
 * Drive_Update が毎周期 accel_limit_m_s2 で実際の目標車速をこの値へ近づける
 * (段階的な変化は Drive_Update 側の責務で、ここでは目的値を差し替えるだけ)。
 * accel_limit_m_s2 が正なら DRIVE_MAX_ACCEL_M_S2 を上限にクランプして使う。
 * 0以下を渡すと DRIVE_MAX_ACCEL_M_S2 で代替する (上限0を「制限なし」と誤解させないため)。
 */
void Drive_SetTargetSpeed(Drive* obj, float m_s, float accel_limit_m_s2);

/**
 * @brief ブレーキを掛ける/離す。on の間は車速制御 (PI) を止め、後輪MDを制動モードに切り替えて
 * torque_nm の制動トルクを各輪へ掛ける。torque_nm は 0〜DRIVE_MAX_BRAKE_TORQUE_NM に
 * クランプされる。0 を渡すと制動トルクが 0 = 惰行になるため、上位の指令をそのまま渡す場合は
 * 「未指定 (0) なら既定値」の解釈を呼び出し側で行うこと。
 */
void Drive_SetBrake(Drive* obj, bool on, float torque_nm);

/**
 * @brief 駆動トルクを直接指令する/止める (torque_mode)。on の間は車速制御 (PI) を止め、
 * torque_nm (1輪あたり、負=後退方向) を左右等配分の総駆動トルクとしてそのまま使う。
 * TC/TV は掛けたままにする (空転抑制のため)。brake_active の方が優先されるので、
 * ブレーキと同時に立っていてもこちらは無視される。torque_nm は ±DRIVE_MAX_TORQUE_NM に
 * クランプされる (上位のクランプに頼らない)。
 */
void Drive_SetTorque(Drive* obj, bool on, float torque_nm);

/**
 * @brief サイドブレーキ (位置制御によるパーキングロック) を掛ける/離す。on の間は
 * 車速制御・通常のブレーキ・torque_mode より優先し、有効化された瞬間の後輪機械角度を
 * ラッチして位置制御へ切り替える。MD からの状態フレームが無効 (BldcMotor_IsDataValid が
 * 偽) な間は角度をラッチできないため、有効な角度が取れるまで通常のトルク制動へ
 * フォールバックする。
 */
void Drive_SetSideBrake(Drive* obj, bool on);

/**
 * @brief サイドブレーキが実際に位置制御へ切り替わって機械角度を固定中かを取得する
 * (要求中でもラッチ待ちでフォールバック中の間は偽)。
 */
bool Drive_IsSideBrakeEngaged(const Drive* obj);

/**
 * @brief トルクベクタリングの有効/無効を切り替える (既定は有効)。
 * 無効にすると左右へ常に等トルクを配分する (オープンデフ相当の挙動になる)。
 * 有効にしていても、IMU の実測ヨーレートが得られない間・低速時は介入しない。
 */
void Drive_SetTorqueVectoringEnabled(Drive* obj, bool enabled);

/**
 * @brief トラクションコントロール (TC本体、前輪基準スリップ率ベース) の有効/無効を
 * 切り替える (既定は有効)。無効にすると各輪のトルク上限を常に DRIVE_MAX_TORQUE_NM に
 * 固定し、スリップ率による削り込みを一切行わない。片輪浮き対策 (Drive_SetWheelLiftGuardEnabled)
 * とは独立に切り替わる。
 */
void Drive_SetTractionControlEnabled(Drive* obj, bool enabled);

/**
 * @brief 片輪浮き対策 (後輪左右速度差の異常検知・絶対車輪速上限) の有効/無効を
 * 切り替える (既定は有効)。トラクションコントロール (TC) 本体とは独立に切替可能。
 */
void Drive_SetWheelLiftGuardEnabled(Drive* obj, bool enabled);

/**
 * @brief トルクベクタリングが今まさに左右へトルク差を付けているかを取得する。
 */
bool Drive_IsTorqueVectoringActive(const Drive* obj);

/**
 * @brief トルクベクタリングの規範モデルが出した目標ヨーレート [rad/s] を取得する
 * (実測値との比較でゲインを詰めるためのデバッグ用)。
 */
float Drive_GetTargetYawRate(const Drive* obj);

/**
 * @brief 適用中の左右トルク差 [Nm] (右輪 − 左輪) を取得する。正 = 左旋回方向。
 */
float Drive_GetYawMomentTorque(const Drive* obj);

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
 * @brief 前後方向判定用の応答が速い車速 [m/s] を取得する。
 *
 * vehicle_speed_m_s (DRIVE_LPF_K_FRONT, τ≈100ms) と同じ前輪エンコーダ由来だが、
 * TCのスリップ判定専用フィルタ (DRIVE_LPF_K_FRONT_TC, τ≈25ms) を共用しており、
 * 急減速時の符号反転への追従が速い。PID/上位報告用の Drive_GetVehicleSpeed とは
 * 用途が異なるため混同しないこと (自動停止の前後方向判定向け)。
 */
float Drive_GetVehicleSpeedForDirection(const Drive* obj);

/**
 * @brief 左後輪のスリップ率を取得する (正 = 空転、負 = ロック傾向)。
 */
float Drive_GetSlipLeft(const Drive* obj);

/**
 * @brief 右後輪のスリップ率を取得する。
 */
float Drive_GetSlipRight(const Drive* obj);

/**
 * @brief TCが動的に決めた左後輪のトルク上限 [Nm] を取得する (デバッグ・上位への報告用)。
 * DRIVE_MAX_TORQUE_NM が「制限なし」、それより小さければ介入中を意味する。
 */
float Drive_GetTcLimitLeft(const Drive* obj);

/**
 * @brief TCが動的に決めた右後輪のトルク上限 [Nm] を取得する。
 */
float Drive_GetTcLimitRight(const Drive* obj);

/**
 * @brief いずれかの後輪でTC(本体)が介入中かを取得する。
 */
bool Drive_IsTractionControlActive(const Drive* obj);

/**
 * @brief いずれかの後輪で片輪浮き対策が介入中かを取得する (デバッグ・上位への報告用)。
 */
bool Drive_IsWheelLiftGuardActive(const Drive* obj);

/**
 * @brief 実際にMDへ送った左輪トルク指令 [Nm] を取得する。
 */
float Drive_GetTorqueLeft(const Drive* obj);

/**
 * @brief 実際にMDへ送った右輪トルク指令 [Nm] を取得する。
 */
float Drive_GetTorqueRight(const Drive* obj);

#endif  // DRIVE_H_
