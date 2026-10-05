#ifndef TORQUE_VECTORING_H_
#define TORQUE_VECTORING_H_

#include <stdbool.h>

#include "control_params.h"

// ===========================================================================
// トルクベクタリング (直接ヨーモーメント制御, DYC)
//
// 「舵角と車速から本来出るはずのヨーレート」(規範モデル) と「IMU が実測したヨーレート」の
// 差を、左右後輪の駆動トルク差で埋める。回り足りない (アンダーステア) なら外輪のトルクを
// 足して内輪を引き、回り過ぎ (オーバーステア) ならその逆に配分する。
//
//   [舵角, 車速] → 規範モデル (自転車モデル + 安定係数) → 目標ヨーレート
//                                                             ↓ −
//                                        IMU 実測ヨーレート ──┤
//                                                             ↓
//                                                       ヨーレート偏差
//                                                             ↓ PI + 不感帯
//                                                    ヨーモーメント Mz [Nm]
//                                                             ↓ ×2r/track
//                                              左右トルク差 ΔT = T_right − T_left [Nm]
//
// **左右の総和は変えない** ため、Drive の車速PI (総駆動トルクを決める側) とは干渉しない。
// トルク差を出せる余力があるかどうか (TC が削っている最中かどうか) は呼び出し側にしか
// 分からないため、実際に適用できた量は TorqueVectoring_ReportApplied() で返してもらう。
//
// 舵角の切り増しではなく駆動力の左右差で曲げるので、ステアリングの応答 (MD の位置ループ +
// リンク機構) を待たずにヨーを立ち上げられる。逆に言えばタイヤのグリップ余力を使い切って
// いる場面では効かない (前輪が既に滑っているならヨーモーメントを足しても曲がらない)。
// ===========================================================================

// ===========================================================================
// 調整パラメータは control_params.h (ControlParams の tv_*)。上位が CONFIG_SET で入れる。
//
// 規範モデル (目標ヨーレートの作り方):
//   δ_eff = tv_steer_gain*δ + tv_steer_gain_cubic*δ^3
//   r_ref = v*tan(δ_eff) / (L*(1 + tv_stability_factor*v^2))、|r_ref| <= tv_max_lateral_accel/|v|
//   tv_ref_lag_s > 0 ならさらに1次遅れを掛ける
// - 実効舵角: 報告される路面舵角は大舵角ほど実際の曲がり方より大きい (28°で幾何のヨーレートが
//   実測より約1割大きい。タイヤではなく舵の非線形)。幾何のままだと TV が旋回中ずっと「回り
//   足りない」と判断して上限に張り付く。値は Pi 側のシステム同定 (vehicle.toml の steer_gain /
//   steer_gain_cubic、シムの曲率と同じ式) で、Pi が起動時に同じ値を送ってくる。規範とシムの
//   曲率が一致するので、TV は外乱 (左右の路面差・TC の片輪絞り) で規範から外れた分だけを直す
// - 横加速度の頭打ち: これが無いと、タイヤのグリップが足りずに曲がれない領域でも「回り足り
//   ない」と判定してトルク差を出し続ける。この車は後輪が先に滑る (オーバーステア寄り) ので、
//   限界で外輪を押し続けると後輪をさらに滑らせる。実測の限界 (mu*g) の約0.92倍にする
// - 1次遅れ: 車のヨーは舵に遅れて付いてくる。遅れの無い規範だと舵を切った瞬間に必ず偏差が
//   出て、過渡のたびに TV が押す (応答は速くなるが行き過ぎる)
//
// 制御 (ヨーレート偏差の PI + 不感帯):
// - tv_kp: ヨー慣性 Iz に対して Kp = Iz/τ でヨーレートループの時定数 τ が決まる
// - tv_max_yaw_moment: ハードの限界は「両輪を逆向きに全開」= ΔT 0.30Nm → Mz 0.78Nm、タイヤの
//   グリップで決まる限界は約0.35Nm。TV が駆動トルクの配分を占有しないよう抑えてある。符号
//   (DRIVE_REAR_*_DIR や IMU の yaw 向き) を間違えるとヨーレート偏差を増やす向きへ正帰還する
//   ため、上限を絞っておくこと自体が安全策になっている
// - tv_deadband: ジャイロのノイズと規範モデル自体の誤差で偏差は決して 0 にならないため、
//   これが無いと直進中も微小なトルク差を出し続け、左右の駆動輪が押し合って電力を捨てる
// - tv_test_moment: 同定用。0 以外の間は PI を止めてこのヨーモーメントだけを出す
// ===========================================================================

// この車速 [m/s] 以下では介入しない。規範モデルの目標ヨーレートは車速に比例するので
// 低速では効果が無いうえ、据え切りに近い状態で左右差を付けても前輪を引きずるだけになる
#define TV_MIN_SPEED_M_S 0.30f

// 適用できなかった分だけ積分を巻き戻す時定数 [s] (back-calculation)
#define TV_ANTIWINDUP_TT_S 0.10f

// 不感帯の中にいる間に積分を0へ抜く時定数 [s]。
// 不感帯は偏差を0に潰すため、旋回中に溜まった積分がそのままでは永久に抜けず、直進に戻った
// あとも一定のトルク差を出し続ける (2026-08-10 のベンチで 0.01Nm ほど張り付いた)。
// 偏差の側から戻ってこない以上こちらから漏らすしかない
#define TV_INTEGRAL_LEAK_TT_S 0.50f

// 上位へ「介入中」と報告する left/right トルク差のしきい値 [Nm]
#define TV_ACTIVE_TORQUE_NM 0.002f

typedef struct {
  // 車体寸法 (Drive 側の実測値を Init で受け取る。TV から drive.h を include すると
  // 依存が循環するため定数を共有せず引数で渡している)
  float wheelbase_m;
  float track_m;
  float wheel_radius_m;

  const ControlParams* params;

  bool enabled;
  float integral_nm;  // ヨーモーメントの積分項 [Nm]
  float ref_lagged_rad_s;  // 1次遅れを掛けた規範ヨーレート (tv_ref_lag_s が 0 なら遅れ前と同じ)
  bool ref_valid;          // 偽ならリセット直後 (次の Update で遅れの状態を今の値から始める)

  // --- 以下は Update が更新する観測量 (チューニング・デバッグ用) ---
  float target_yaw_rate_rad_s;  // 規範モデルが出した目標ヨーレート (左旋回が正)
  float yaw_rate_error_rad_s;   // 目標 − 実測 (不感帯を引く前の生の偏差)
  float yaw_moment_nm;          // PI が要求したヨーモーメント
  float diff_torque_nm;         // 実際に適用された左右トルク差 (T_right − T_left)
} TorqueVectoring;

/**
 * @brief トルクベクタリングを初期化する。車体寸法 [m] は駆動輪 (後輪) 側の値を渡すこと。
 * 初期状態は有効 (enabled)。
 */
void TorqueVectoring_Init(TorqueVectoring* obj, const ControlParams* params, float wheelbase_m,
                          float track_m, float wheel_radius_m);

/**
 * @brief ヨーレート偏差から左右トルク差 [Nm] (正 = 右輪を足して左輪を引く = 左旋回方向) を
 * 計算して返す。yaw_rate_rad_s には**実測値**を渡すこと (舵角からの幾何計算値を渡すと
 * 規範モデルとほぼ同じ式になり偏差が常に 0 付近になって制御が成立しない)。
 * 無効時・低速時は 0 を返し積分もリセットする。
 * @param speed_m_s 車体前後方向の車速 (後退時は負)
 * @param steer_rad 路面舵角 (反時計回り = 左旋回が正)
 * @param dt_s 前回呼び出しからの経過時間 (0 なら積分を進めない)
 */
float TorqueVectoring_Update(TorqueVectoring* obj, float speed_m_s, float steer_rad,
                             float yaw_rate_rad_s, float dt_s);

/**
 * @brief 要求した左右トルク差がどれだけ実現できたかを返す。TorqueVectoring_Update() の直後に必ず呼ぶこと。
 * @param achieved_diff_nm 要求した差のうち実際に左右差として効いた分 = (送った左右差) −
 *   (TV の要求が 0 だったら送っていたはずの左右差)。要求より小さければその差分だけ積分項を
 *   巻き戻す (back-calculation)。後ろの項を引くのは、TC が片輪を絞って生じた「要求していない差」を
 *   TV が自分の出力と取り違えて、積分をその向きへ育ててしまうため。こうしておけば、片輪が
 *   絞られている間も反対の輪を下げる向きの要求はそのまま通り (ヨーを優先して駆動力を譲る)、
 *   絞られた輪を上げる向きの要求だけが巻き戻される
 * @param applied_diff_nm 最終的に MD へ送った左右差 (報告用)
 */
void TorqueVectoring_ReportApplied(TorqueVectoring* obj, float achieved_diff_nm,
                                   float applied_diff_nm, float dt_s);

/**
 * @brief 積分項と出力をリセットする (走行の開始・停止時に呼ぶ)。
 */
void TorqueVectoring_Reset(TorqueVectoring* obj);

/**
 * @brief 機能そのものの有効/無効を切り替える。無効にすると常に 0 を返す。
 */
void TorqueVectoring_SetEnabled(TorqueVectoring* obj, bool enabled);

/**
 * @brief 機能が有効かを取得する (今まさに介入中かは TorqueVectoring_IsActive())。
 */
bool TorqueVectoring_IsEnabled(const TorqueVectoring* obj);

/**
 * @brief 今まさに左右へトルク差を付けているかを取得する。
 */
bool TorqueVectoring_IsActive(const TorqueVectoring* obj);

/**
 * @brief 規範モデルが出した目標ヨーレート [rad/s] を取得する。
 */
float TorqueVectoring_GetTargetYawRate(const TorqueVectoring* obj);

/**
 * @brief 適用中の左右トルク差 [Nm] (T_right − T_left) を取得する。
 */
float TorqueVectoring_GetDiffTorque(const TorqueVectoring* obj);

#endif  // TORQUE_VECTORING_H_
