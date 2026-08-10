#ifndef TORQUE_VECTORING_H_
#define TORQUE_VECTORING_H_

#include <stdbool.h>

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
// 規範モデル (目標ヨーレートの作り方)
// ===========================================================================

// 安定係数 [s^2/m^2]。0 なら幾何 (アッカーマン) どおりのヨーレートをそのまま目標にする。
// 実車はタイヤの横滑り角のぶんだけ幾何より曲がらない (弱アンダーステアに設計するのが普通)
// ため、正の値を入れると高速側で目標が下がる。0 のままだと高速コーナーで「回り足りない」と
// 判定し続け、TV が常に外輪を足す側へ張り付く。
// ★未実測: 一定舵角で速度を変えながら定常円旋回させ、実測ヨーレート r から
// K = (v*δ / (L*r) - 1) / v^2 として求めること。当面は 0 (幾何どおり) で運用する★
#define TV_STABILITY_FACTOR 0.0f

// タイヤが出せる横加速度の上限 [m/s^2]。目標ヨーレートは |r| <= a_y_max / |v| で頭打ちにする。
// これが無いと、そもそもタイヤのグリップが足りずに曲がれない領域でも「回り足りない」と
// 判定してトルク差を出し続け、内輪を無駄に削るだけの介入になる
// ★未実測: μ=0.6 相当の暫定値★
#define TV_MAX_LATERAL_ACCEL_M_S2 6.0f

// ===========================================================================
// 制御パラメータ
// ===========================================================================

// ヨーレート偏差に対する比例ゲイン [Nm / (rad/s)]。
// ヨー慣性 Iz ≈ m*(L^2+W^2)/12 ≈ 1.5*(0.30^2+0.15^2)/12 ≈ 0.014 kg・m^2 に対して
// Kp = Iz / τ の関係でヨーレートループの時定数 τ が決まる。0.08 は τ ≈ 0.18s 相当。
// ★Iz は概算なので実機で振動が出るなら下げること★
#define TV_KP_NM_PER_RAD_S 0.08f
#define TV_KI_NM_PER_RAD 0.16f  // 積分ゲイン (積分時間 Ti = Kp/Ki = 0.5s 相当)

// ヨーモーメント指令の上限 [Nm]。ハードの限界は「両輪を逆向きに全開」= ΔT 0.15Nm →
// Mz 0.39Nm だが、TV が駆動トルクの配分を占有しないよう 4割程度に抑えてある。
// 符号 (DRIVE_REAR_*_DIR や IMU の yaw 向き) を間違えるとヨーレート偏差を増やす向きへ
// 正帰還するため、上限を絞っておくこと自体が実機確認前の安全策になっている
#define TV_MAX_YAW_MOMENT_NM 0.15f

// ヨーレート偏差の不感帯 [rad/s]。ジャイロのノイズと規範モデル自体の誤差で偏差は決して
// 0 にならないため、これが無いと直進中も微小なトルク差を出し続け、左右の駆動輪が
// 押し合って電力を捨てる状態になる
#define TV_DEADBAND_RAD_S 0.05f

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

  bool enabled;
  float integral_nm;  // ヨーモーメントの積分項 [Nm]

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
void TorqueVectoring_Init(TorqueVectoring* obj, float wheelbase_m, float track_m,
                          float wheel_radius_m);

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
 * @brief 実際に適用できた左右トルク差 [Nm] を返す。TC の介入などで要求どおりの差を
 * 出せなかった場合、その差分だけ積分項を巻き戻す (これを呼ばないとワインドアップし、
 * グリップが戻った瞬間に過大なトルク差が出る)。TorqueVectoring_Update() の直後に、
 * クランプ後の値で必ず呼ぶこと。
 */
void TorqueVectoring_ReportApplied(TorqueVectoring* obj, float applied_diff_nm, float dt_s);

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
