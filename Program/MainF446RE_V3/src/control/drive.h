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
// トラクションコントロール (TC) パラメータ
// 各輪独立。ABS (下) と同じ「滑り始めたときのトルクを覚えて、そこを基準に上下させる」型:
//   ①空転し始めを検知したら、その輪に掛けていたトルク (tc_*.lock_nm) を覚え、上限を
//     DRIVE_TC_CUT_RATIO 倍へ一度だけ下げる
//   ②DRIVE_TC_RELEASE_TIMEOUT_S 経ってもグリップが戻らなければもう一段下げる
//   ③戻ったら覚えたトルクの DRIVE_TC_REAPPLY_RATIO 倍へすぐ戻し、DRIVE_TC_RECOVER_RATE で上げる
// 2026-09-27 までは超過スリップに比例して 0.3Nm/s で削る型で、実機の全開加速で右後輪が
// 滑り率0.3〜0.8を約0.4s続けても上限は 0.15→0.118Nm しか下がらず (実トルク約0.10Nm に
// 届かない)、一度も絞っていなかった。上限を 0.15 から削り始める (掛けているトルクから削らない)
// ことも効かない理由だった。実機記録で較正した後輪+タイヤのモデルで、空転している時間が
// 77%→13% に減ることを確認 (Pi側リポジトリの PROGRESS.md)。
// ★しきい値 0.2 は横グリップ優先の選択。この路面は滑らせても駆動力がほぼ落ちない可能性があり
// (モデルの較正結果)、その場合の直線の加速は約0.4m/s²下がる。直線を優先するなら0.3へ★
// ===========================================================================
#define DRIVE_TC_SLIP_THRESHOLD 0.2f   // スリップ率がこれを超えたら空転し始めとみなす
#define DRIVE_TC_RECOVER_SLIP 0.1f     // スリップ率がこれより小さくなったらグリップが戻ったとみなす
#define DRIVE_TC_SLIP_DEBOUNCE_S 0.005f  // ABS と同じ。本物の空転は数十msで深くなるので応答を優先
#define DRIVE_TC_CUT_RATIO 0.7f          // 空転し始めで上限を掛けていたトルクの何倍へ下げるか
#define DRIVE_TC_RELEASE_TIMEOUT_S 0.025f  // 下げてもこの時間グリップが戻らなければもう一段下げる
#define DRIVE_TC_REAPPLY_RATIO 0.9f      // グリップが戻ったら覚えたトルクの何倍へすぐ戻すか
#define DRIVE_TC_RECOVER_RATE 0.2f       // そこから上限を上げる速度 [Nm/s]
#define DRIVE_TC_MIN_TORQUE_NM 0.005f  // 削り切っても完全には0にしない (再加速できなくなるため)
#define DRIVE_TC_MIN_SPEED_M_S 0.25f   // これ以下の車速ではスリップ率が発散するのでTCを効かせない

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
// ABS (アンチロックブレーキ) パラメータ
// 制動モード (Drive_SetBrake) の間だけ働く。制動トルクの上限を左右共通の1つだけ持つ
// (select-low: ロックしかけている側に合わせる)。左右独立にすると左右で路面μが違うときに
// 後輪側でヨーモーメントが出てスピン方向に振れるため、実車の後輪ABSと同じく安定性を優先する。
// 車速PIの減速・torque_mode の逆向きトルクは対象外 (ユーザー判断)。
//
// 実車のABSの定石どおり「滑り始めたときのトルクを覚えて、そこを基準に上下させる」:
//   ①滑り始めを検知したら、そのとき掛けていたトルク (abs_lock_nm) を覚え、上限を
//     DRIVE_ABS_CUT_RATIO 倍へ一度だけ下げる (削り続けない)
//   ②DRIVE_ABS_RELEASE_TIMEOUT_S 経ってもグリップが戻らなければ、もう一段同じ比で下げる
//     (低μ路。覚えるトルクも下げた値に更新する)
//   ③グリップが戻ったら (スリップが DRIVE_ABS_RECOVER_SLIP より浅い) 覚えたトルクの
//     DRIVE_ABS_REAPPLY_RATIO 倍へすぐ戻し、そこから DRIVE_ABS_RECOVER_RATE でゆっくり上げる
//     (グリップの限界の少し手前に長く留まる)
// 2026-09-27 の初版は超過スリップに比例して削り続ける TC と同じ型だったが、実機では検知の遅れの
// 間に上限がほぼ0まで削られ、1.0Nm/s でゆっくり戻すので平均の制動トルクが約0.06Nm (限界は約
// 0.09Nm) に留まり、要求0.12・0.15Nm の方が0.08Nm より減速が弱かった (2.3〜2.5m/s²、ロックなし
// だと3.67m/s²)。実機の記録で較正した後輪+タイヤのモデルで比べ、この方式で平均の減速度が約25%
// 上がり (ロックなしの約97%)、後輪がロック気味の時間も減ることを確認した (低μ・ノイズ・遅れでも同様)。
// ★ゲインはモデルでの机上値。実機で制動中の torque_cmd と slip を見て詰めること★
// ===========================================================================
#define DRIVE_ABS_SLIP_THRESHOLD 0.2f  // スリップ率がこれより負になったら滑り始めとみなす
#define DRIVE_ABS_RECOVER_SLIP 0.1f    // スリップ率がこれより浅くなったらグリップが戻ったとみなす (ヒステリシス)
#define DRIVE_ABS_SLIP_DEBOUNCE_S 0.005f  // TCより短い。制動中はノイズ耐性より応答を優先する
#define DRIVE_ABS_CUT_RATIO 0.7f          // 滑り始めで上限を掛けていたトルクの何倍へ下げるか
#define DRIVE_ABS_RELEASE_TIMEOUT_S 0.025f  // 下げてもこの時間グリップが戻らなければもう一段下げる
#define DRIVE_ABS_REAPPLY_RATIO 0.9f      // グリップが戻ったら覚えたトルクの何倍へすぐ戻すか
#define DRIVE_ABS_RECOVER_RATE 0.2f       // そこから上限を上げる速度 [Nm/s] (限界の手前に長く留まる)
// フォールバック: 上限が要求の DRIVE_ABS_FALLBACK_LIMIT_RATIO 以下に張り付いたまま
// DRIVE_ABS_FALLBACK_TIME_S 続いたら、ブレーキを離すまでABSを止めて要求どおりに制動する。
// 前輪エンコーダの故障・符号ミスで基準車速が高く出るとスリップが負に張り付き、緊急停止中も
// 制動を抜き続けてしまうため。本来のABSは上限を上下させるので下限付近に長く張り付くことは
// 少ない、という前提に立つ (非常に滑る路面で本当に抜き続ける場面ではロック側に倒れる)。
// 上限は DRIVE_ABS_RELEASE_TIMEOUT_S ごとに CUT_RATIO 倍ずつ下がるので、最大制動から20%以下へ
// 届くまで約0.125s掛かる。そこから0.1sで、初版 (ほぼ即座に0へ削って0.2s) と同じ約0.2sで戻る
#define DRIVE_ABS_FALLBACK_LIMIT_RATIO 0.2f
#define DRIVE_ABS_FALLBACK_TIME_S 0.1f

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
// (ノイズは残りやすくなるが、DRIVE_TC_SLIP_DEBOUNCE_S 側で吸収する)
#define DRIVE_LPF_K_FRONT_TC 0.905f

// TC の1輪ぶんの状態 (ABS の abs_* と同じ役割)
typedef struct {
  float lock_nm;         // 直近に空転し始めたときに掛けていたトルク (戻す基準)
  bool releasing;        // 空転を検知して上限を下げ、グリップが戻るのを待っている間
  float release_time_s;  // releasing の継続時間 (DRIVE_TC_RELEASE_TIMEOUT_S ごとに下げ直す)
  float excess_time_s;   // スリップ率がしきい値を超えてからの継続時間 (デバウンス用)
} TractionState;

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
  float slip_left;              // 左後輪のスリップ率 (正 = 空転, 負 = ロック傾向)
  float slip_right;             // 右後輪のスリップ率
  // TC の各輪の状態 (TractionState 参照)
  TractionState tc_left;
  TractionState tc_right;
  float tc_limit_left_nm;   // TCが動的に決めた左輪のトルク上限
  float tc_limit_right_nm;  // TCが動的に決めた右輪のトルク上限
  // 片輪浮き対策が動的に決めた各輪のトルク上限 (tc_limit_*とは独立、最終的にminを取る)
  float wheel_lift_limit_left_nm;
  float wheel_lift_limit_right_nm;
  // ABSが動的に決めた制動トルク上限 (左右共通、select-low)。制動していない間は
  // DRIVE_MAX_BRAKE_TORQUE_NM に戻しておき、次の制動を全量から始める
  float abs_limit_nm;
  float abs_lock_nm;        // 直近に滑り始めたときに掛けていた制動トルク (戻す基準)
  bool abs_releasing;       // 滑り始めを検知して上限を下げ、グリップが戻るのを待っている間
  float abs_release_time_s; // abs_releasing の継続時間 (DRIVE_ABS_RELEASE_TIMEOUT_S ごとに下げ直す)
  float abs_excess_time_s;  // TractionState.excess_time_s と同じデバウンス用
  float abs_floor_time_s;   // 上限が DRIVE_ABS_FALLBACK_LIMIT_RATIO 以下に張り付いている継続時間
  bool abs_fallback_latched;  // 真の間はABSを止めて要求どおりに制動する (ブレーキ解除で戻る)
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
 * 実際の送信は Motors_Transmit が行うため、本関数の後に Motors_Transmit を呼ぶこと。
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
