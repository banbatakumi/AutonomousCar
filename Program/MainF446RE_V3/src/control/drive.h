#ifndef DRIVE_H_
#define DRIVE_H_

#include <stdbool.h>

#include "encoder.h"
#include "imu.h"
#include "lpf.h"
#include "control_params.h"
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
// 決めた左右差 (旋回中の左右の荷重の比) だけを重ねる。片輪が上限に当たらない限り総和は
// 変わらないので車速制御とは干渉しない。差を付けない限りは
// 旋回時の内外輪速度差を各輪が自然に見つける (オープンデフ相当) 挙動になり、左右輪へ独立に
// 速度ループを掛ける方式と違ってタイヤ径誤差やアッカーマンモデル誤差による内部トルクの循環
// (片輪が押して片輪が引く状態) が原理的に発生しない。
//
// 車速の真値は非駆動輪である前輪エンコーダから得るため、駆動輪速度との比較でスリップ率が
// 直接計算でき、トラクションコントロール (TC) が成立する。制動 (Drive_SetBrake) 側も同じ
// スリップ率を負側で見て、ロックしかけたら制動トルクを抜く (ABS)。

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
// スリップ制限 (TC・ABS・片輪浮き対策)
//
// 調整パラメータは control_params.h (ControlParams)。上位が CONFIG_SET で入れる。
//
// ■ スリップの測り方
// 後輪の周速と「その輪が滑っていなければ出るはずの速度」(前輪の車速 ± ヨーレート×半トレッド)
// の差をスリップ速度、それを max(|基準速度|, slip_speed_floor_m_s) で割ったものをスリップ率と
// する。分母に下限を置くのは、車速が0に近いと率が発散するため (2026-10-05 までは 0.25m/s 未満で
// TC・ABS ごと無効にしていて、発進時の空転を誰も見ていなかった)。下限より遅い間は「スリップ
// 速度が 下限×目標スリップ率 を超えたか」を見ていることになる。
// 制御が見るのは**トルクを掛けている向きのスリップ速度**: 駆動なら空転、車速PIの減速・制動なら
// ロック傾向が正になる。前進・後退・加速・減速を1つの式で扱える。
//
// ■ 制限のしかた (TC・ABS 共通、SlipLimiter)
// スリップ率を目標 (tc_slip_target / abs_slip_target) に保つ連続の PI。出力はトルクの上限。
//   偏差 e = 目標スリップ率×分母 − スリップ速度 [m/s] (正 = 余裕がある)
//   ①e が負になったら働き始める。積分の初期値はそのとき実際に掛けていたトルク
//     (上限を最大値から削り始めると、掛けているトルクへ届くまで何もしないのと同じになる)
//   ②働いている間、上限 = 積分 + kp×e。積分は ki×e で進む
//   ③上限が要求トルク以上に戻ったら (= 要求どおり掛けても滑らない) 止める
// 2026-10-05 までは実車の油圧 ABS の「減→保持→増」を写したルールベース (滑り始めのトルクを
// 覚えて0.7倍へ下げ、25ms 戻らなければもう一段、戻ったら0.9倍へ戻して0.2Nm/s で上げる) だった。
// 油圧の弁は開閉しかできないのでこの形になるが、モータはトルクを連続に出せるので段を刻む理由が
// 無い。実機でも ABS は最初の一撃で深くロックし、25ms ごとの再カットで必要以上に抜いていた
// (ロックなしの約85%)。比較は Pi 側リポジトリの tools/ctrl_tune/bench.py と PROGRESS.md。
//
// ■ TC (各輪独立)
// 駆動 (車速PI・torque_mode) と車速PIの減速の両方に掛かる。後者は以前 ABS の対象外だったが、
// 目標車速のランプ (3.0m/s^2) は後輪だけの制動で出せる減速度を超えるので、旋回しながら減速すると
// 後輪がロックしてスピンし得た。
// 片輪を絞っても、その分を反対の輪へ載せない (総駆動力が減る)。2026-10-05 までは総和を保つために
// 反対の輪を押し増していて、要求していないヨーモーメントが出ていた。
//
// ■ 片輪浮き対策
// TC の基準は前輪なので、前輪エンコーダが壊れると働かない (または絞り続ける)。前輪に依らない
// 見張りとして「後輪左右の速度差からヨーレートで説明できる分を引いた残り」が
// wheel_lift_diff_threshold_m_s を超えたら、トルクの向きへ先走っている輪を同じ SlipLimiter で絞る
// (偏差は TC と小さい方を使う)。TC とは独立に ON/OFF できる。
//
// ■ ABS
// 制動モード (Drive_SetBrake) の間だけ働く。上限は左右共通で、ロックしかけている側に合わせる
// (select-low)。左右独立にすると左右で路面μが違うときに後輪側でヨーモーメントが出てスピン方向に
// 振れるため、実車の後輪 ABS と同じく安定性を優先する。
// ===========================================================================

// 後輪周速がこれを超えたら、基準速度・左右差に関係なく即座にトルクを0にする (最終防波堤。
// 片輪浮き対策の一部として ON/OFF される)。7.5m/s
#define DRIVE_WHEEL_LIFT_MAX_WHEEL_SPEED_M_S (DRIVE_MAX_SPEED_M_S * 1.5f)

// ABS のフォールバック: 上限が要求の DRIVE_ABS_FALLBACK_LIMIT_RATIO 以下に張り付いたまま
// DRIVE_ABS_FALLBACK_TIME_S 続いたら、ブレーキを離すまで ABS を止めて要求どおりに制動する。
// 前輪エンコーダの故障・符号ミスで基準車速が高く出るとスリップがロック側に張り付き、緊急停止中も
// 制動を抜き続けてしまうため。本来の ABS はグリップの限界付近 (この車は最大制動の約6割) に
// 留まるので、2割以下に長く張り付くことは少ない、という前提に立つ (非常に滑る路面で本当に
// 抜き続ける場面ではロック側に倒れる)
#define DRIVE_ABS_FALLBACK_LIMIT_RATIO 0.2f
#define DRIVE_ABS_FALLBACK_TIME_S 0.2f

// ===========================================================================
// フィルタ係数 (Drive_Update の呼び出し周期 500us = 2kHz に依存する。τ=-dt/ln(k))
// 前輪の2つは、Encoder 側で ADC を32サンプル平均して角度ノイズを下げたのに合わせ、
// 平均化前 (PID用 k=0.995 τ≈100ms / TC用 k=0.98 τ≈25ms) から縮めたもの。
// 静止時 (駆動電源OFF) の実測で生の前輪角速度の標準偏差は約2.2rad/s (周速0.065m/s) の白色ノイズで、
// 車速のノイズは σ ≈ 0.046 × (1-k) [m/s] でよく予測できる (k=0.951 で約0.002m/s、k=0.905 で約0.004m/s)。
// ★未実測: 駆動電源ON時 (MDのPWMノイズ)・定速走行時 (エンコーダの1回転周期の誤差) のノイズは未確認★
// ===========================================================================
#define DRIVE_LPF_K_FRONT 0.951f  // 前輪エンコーダ速度。τ≈10ms。PIDフィードバック・上位報告用
#define DRIVE_LPF_K_REAR 0.90f    // 後輪モータ速度 (MD側で既にフィルタ済みのため弱め)。τ≈4.75ms
// TCのスリップ判定専用の前輪速度フィルタ。DRIVE_LPF_K_FRONT (τ≈10ms) をそのまま基準速度に
// 使うと、フル加速のようなランプ入力で τ×加速度 ぶん (最大加速度3.0m/s²なら約0.03m/s) 基準速度が
// 実速度より系統的に遅れ、後輪 (τ≈4.75ms、ほぼ遅れ無し) との差が「常時空転している」という
// 誤ったスリップ率を生む (低速ほど基準速度に対する遅れの比率が大きく致命的)。この遅れバイアスを
// 縮めるため、PID用より軽い τ≈5ms のフィルタをスリップ判定専用に別途持つ
// (ノイズは残りやすくなるが、スリップ制限は偏差に比例して絞るだけなので小さなノイズは効かない)
#define DRIVE_LPF_K_FRONT_TC 0.905f

// スリップ制限1つぶんの状態 (TC は各輪、ABS は左右共通で1つ)
typedef struct {
  bool active;        // スリップが目標を超えて働き始めてから、上限が要求以上へ戻るまで
  float integral_nm;  // PI の積分 = 「このトルクなら滑らない」という見積もり
} SlipLimiter;

typedef struct {
  Motors* motors;
  Encoder* encoder;
  Steering* steering;
  Imu* imu;  // ヨーレート取得用 (NULL可。その場合は自転車モデルで代用する)

  ControlParams params;  // 調整パラメータ (Drive_SetParams で上位の設定を写す)

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
  bool tc_enabled;                // 既定は有効
  bool wheel_lift_guard_enabled;  // 既定は有効。TC本体とは独立にON/OFF可能
  bool abs_enabled;               // 既定は有効
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
  float manual_torque_nm;   // torque_mode 中に指令された1輪あたりの駆動トルク [Nm] (負=後退方向)
  bool side_brake_active;   // 上位からの要求 (毎周期 Drive_SetSideBrake で更新)
  bool side_brake_engaged;  // 実際に位置保持へ入っているか (角度をラッチ済みか)
  float side_brake_target_left_rad;
  float side_brake_target_right_rad;

  // --- 以下は Drive_Update が更新する観測量 (デバッグ・上位への報告用) ---
  // 周速はすべて LPF 後の値。前輪の生の角速度は使わないこと。12bit ADC で 1回転を測るため
  // 1LSB = 2pi/4096 = 1.53mrad で、これを 500us で微分すると 1LSB あたり 3.07rad/s
  // (= 0.09m/s) に化ける。実測ではノイズが ±100mV 程度あり、静止中でも生値は ±12m/s 振れる
  float vehicle_speed_m_s;  // 前輪から推定した車体前後方向の速度 (舵角で射影済み)
  // 前後方向判定用の応答が速い車速 (EstimateVehicleSpeedForSlip 由来、τ≈5ms)。
  // vehicle_speed_m_s (τ≈10ms) では急減速時の符号反転検出が遅れるため、
  // 自動停止の前後判定 (vehicle.c) 向けに分けて保持する
  float vehicle_speed_for_direction_m_s;
  float yaw_rate_rad_s;         // スリップ率の基準速度を作るのに使ったヨーレート
  bool yaw_rate_measured;       // 上がIMUの実測値か (偽 = 舵角からの幾何計算で代用中)
  float front_speed_left_m_s;   // 左前輪の周速 (射影前。車体速度ではなく車輪自身の軌跡上の速度)
  float front_speed_right_m_s;  // 右前輪の周速
  float rear_speed_left_m_s;    // 左後輪の周速
  float rear_speed_right_m_s;   // 右後輪の周速
  // スリップ率は進行方向が基準 (正 = 空転, 負 = ロック傾向)。上位への報告用で、制御は
  // トルクの向きを基準にしたスリップ速度 (drive.c の SlipSpeedAlongTorque) を見る
  float slip_left;
  float slip_right;
  float slip_ref_left_m_s;   // 左後輪が滑っていなければ出るはずの周速 (スリップの基準)
  float slip_ref_right_m_s;
  SlipLimiter tc_left;
  SlipLimiter tc_right;
  float tc_limit_left_nm;   // 左輪のトルク上限 (働いていなければ DRIVE_MAX_TORQUE_NM)
  float tc_limit_right_nm;
  bool tc_limiting;          // 前周期、いずれかの輪で上限が実際にトルクを削った
  bool wheel_lift_limiting;  // 同 片輪浮き対策の偏差の方が効いていた (または絶対上限で切った)
  SlipLimiter abs;
  float abs_limit_nm;       // 制動トルクの上限 (左右共通。働いていなければ DRIVE_MAX_BRAKE_TORQUE_NM)
  float abs_floor_time_s;   // 上限が DRIVE_ABS_FALLBACK_LIMIT_RATIO 以下に張り付いている継続時間
  bool abs_fallback_latched;  // 真の間はABSを止めて要求どおりに制動する (ブレーキ解除で戻る)
  // 実際にMDへ送った各輪のトルク指令。正 = 駆動、負 = 制動。制動モード (停車保持・
  // Drive_SetBrake) のときは制動トルクを負値として入れるので、符号を見れば駆動しているのか
  // 押さえているのかが上位から区別できる
  float torque_left_nm;
  float torque_right_nm;
  // スリップ制限 (TC・片輪浮き対策・ABS) が絞る前に各輪へ掛けたかったトルク。torque_*_nm との差が
  // 「どれだけ絞ったか」。駆動は TV の左右差を載せた後の値、制動は要求の制動トルク (負値)
  float torque_request_left_nm;
  float torque_request_right_nm;
} Drive;

/**
 * @brief 駆動制御を初期化する。imu は NULL でもよい (旋回時のスリップ率補正精度が落ちる)。
 * 初期化直後は無効状態 (トルク指令を出さない)。
 */
void Drive_Init(Drive* obj, Motors* motors, Encoder* encoder, Steering* steering, Imu* imu);

/**
 * @brief 車速制御・TC を1周期分実行し、後輪MDへのトルク指令を更新する。
 * 実際の送信は Motors_Transmit が行うため、本関数の後に Motors_Transmit を呼ぶこと。
 * フィルタ係数が周期に依存するため、一定周期 (500us = 2kHz 想定) で呼ぶこと。
 */
void Drive_Update(Drive* obj);

/**
 * @brief 調整パラメータを差し替える (上位の CONFIG_SET を毎周期ここへ写す)。
 */
void Drive_SetParams(Drive* obj, const ControlParams* params);

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
 * 有効にしていても、IMU の実測ヨーレートが得られない間は介入しない。
 */
void Drive_SetTorqueVectoringEnabled(Drive* obj, bool enabled);

/**
 * @brief トラクションコントロール (TC本体、前輪基準のスリップ) の有効/無効を切り替える
 * (既定は有効)。片輪浮き対策 (Drive_SetWheelLiftGuardEnabled) とは独立に切り替わり、両方を
 * 無効にすると各輪のトルク上限は常に DRIVE_MAX_TORQUE_NM になる。
 */
void Drive_SetTractionControlEnabled(Drive* obj, bool enabled);

/**
 * @brief 片輪浮き対策 (後輪左右速度差の異常検知・絶対車輪速上限) の有効/無効を
 * 切り替える (既定は有効)。トラクションコントロール (TC) 本体とは独立に切替可能。
 */
void Drive_SetWheelLiftGuardEnabled(Drive* obj, bool enabled);

/**
 * @brief ABS (制動時の後輪ロック防止) の有効/無効を切り替える (既定は有効)。
 * 無効にすると Drive_SetBrake で指定した制動トルクをそのまま掛ける。
 */
void Drive_SetAbsEnabled(Drive* obj, bool enabled);

/**
 * @brief ABSが今まさに制動トルクを要求より削っているかを取得する (デバッグ・上位への報告用)。
 * フォールバック中 (ABSを止めて要求どおりに制動している間) は偽。
 */
bool Drive_IsAbsActive(const Drive* obj);

/**
 * @brief トルクベクタリングが今まさに左右へトルク差を付けているかを取得する。
 */
bool Drive_IsTorqueVectoringActive(const Drive* obj);

/**
 * @brief トルクベクタリングの配分の比率 (左右の荷重差 / 荷重和 の見積り。左旋回で正 = 右輪が多い)。
 */
float Drive_GetTvRatio(const Drive* obj);

/**
 * @brief トルクベクタリングが要求した左右トルク差をヨーモーメント [Nm] に換算した値
 * (左旋回方向が正)。実際に付いた左右差は各輪のトルク (Drive_GetTorqueLeft/Right) から分かる。
 */
float Drive_GetTvYawMoment(const Drive* obj);

/**
 * @brief ABS が決めた制動トルクの上限 [Nm] (働いていなければ DRIVE_MAX_BRAKE_TORQUE_NM)。
 */
float Drive_GetAbsLimit(const Drive* obj);

/**
 * @brief スリップ制限が絞る前に左輪へ掛けたかったトルク [Nm] (Drive.torque_request_left_nm 参照)。
 */
float Drive_GetTorqueRequestLeft(const Drive* obj);

/**
 * @brief 同 右輪。
 */
float Drive_GetTorqueRequestRight(const Drive* obj);

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
 * vehicle_speed_m_s (DRIVE_LPF_K_FRONT, τ≈10ms) と同じ前輪エンコーダ由来だが、
 * TCのスリップ判定専用フィルタ (DRIVE_LPF_K_FRONT_TC, τ≈5ms) を共用しており、
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
