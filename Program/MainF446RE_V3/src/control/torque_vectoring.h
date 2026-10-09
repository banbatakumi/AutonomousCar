#ifndef TORQUE_VECTORING_H_
#define TORQUE_VECTORING_H_

#include <stdbool.h>

#include "control_params.h"

// ===========================================================================
// トルクベクタリング (後輪左右の荷重に比例したトルク配分。フィードフォワードのみ)
//
// 旋回中は横加速度で荷重が外輪へ移る。左右へ等トルクを配ると、荷重の抜けた内輪が先に空転して
// TC に絞られ、荷重の乗った外輪のグリップは余る。そこで総駆動トルクを左右の荷重の比で配る:
//
//   a_y   = 車速 × 実測ヨーレート                    (左旋回が正)
//   比率  = clamp(tv_load_gain × a_y, ±tv_max_ratio)  (= 左右の荷重差 / 荷重和 の見積り)
//   ΔT    = T_right − T_left = 総駆動トルク × 比率
//
// - 駆動 (総トルク > 0) では外輪が多くなり、旋回方向のヨーモーメントも付く
// - 車速PIの減速 (総トルク < 0) では外輪が多く制動し、旋回を戻す向き (安定側) になる
// - 総トルクが 0 (定速の惰行に近い旋回) なら何もしない
// - 後退は車速・ヨーレート・総トルクの符号がそろって反転するので、同じ式で外輪が多くなる
//
// 2026-10-08 まではヨーレートの PI (規範モデルとの偏差を左右差で埋める) だった。やめた理由:
// - 規範に実機の同定結果 (舵の効き) を入れていたので偏差が最初からほぼ 0 で、不感帯の中にいた
// - ヨーモーメント→ヨーレートの効きが小さく (0.1Nm で 0.056rad/s)、ループゲインが 0.05 しか
//   ない。1 にするゲインでは偏差のノイズだけで上限に張り付く
// - 定常のヨーレートは舵 (上位が直接指令する) の方が30倍動かせる。TV が稼げるのは
//   「旋回しながらの加速で内輪を空転させない」ことで、それには偏差の制御は要らない
// - 回り過ぎの修正は内輪へトルクを足す向きになり、空転している内輪を TC が絞って打ち消した
//
// 横加速度を舵角からではなく実測ヨーレートから作るのは、車体が滑り出したら自然に頭打ちに
// なるため (舵角から作ると、曲がれていないのに外輪を押し続ける)。「ヨーが増える→外輪が増える→
// ヨーモーメントが増える」の正帰還になるが、全開でもループゲインは 0.05 未満で発散しない。
// 状態 (積分) を持たないので、TC が後段で片輪を絞っても巻き戻すものが無い。
// ===========================================================================

// ===========================================================================
// 調整パラメータは control_params.h (ControlParams の tv_*)。上位が CONFIG_SET で入れる。
// - tv_load_gain_s2_per_m: 横加速度 → 配分の比率。理論値は
//   2 × 重心高 × (後軸のロール分担) / (トレッド × g) で、後輪1つの静止荷重 Fz と
//   「横加速度あたりに外輪へ移る荷重」k [N/(m/s^2)] からは k / Fz。0 で TV なし
// - tv_max_ratio: 比率の上限。1 で内輪のトルクが 0 になる (それを超えると内輪が逆向きに回す)
// - tv_test_moment_nm: 同定用。0 以外の間は配分を止めてこのヨーモーメントだけを出す
// ===========================================================================

// 上位へ「介入中」と報告する left/right トルク差のしきい値 [Nm]
#define TV_ACTIVE_TORQUE_NM 0.002f

typedef struct {
  // 車体寸法 (Drive 側の実測値を Init で受け取る。TV から drive.h を include すると
  // 依存が循環するため定数を共有せず引数で渡している)
  float track_m;
  float wheel_radius_m;

  const ControlParams* params;

  bool enabled;

  // --- 以下は Update が更新する観測量 (チューニング・デバッグ用) ---
  float ratio;           // 配分の比率 (上限で丸めた後。左旋回で正 = 右輪が多い)
  float diff_torque_nm;  // 要求した左右トルク差 (T_right − T_left)
} TorqueVectoring;

/**
 * @brief トルクベクタリングを初期化する。車体寸法 [m] は駆動輪 (後輪) 側の値を渡すこと。
 * 初期状態は有効 (enabled)。
 */
void TorqueVectoring_Init(TorqueVectoring* obj, const ControlParams* params, float track_m,
                          float wheel_radius_m);

/**
 * @brief 総駆動トルクを左右の荷重の比で配るための左右トルク差 [Nm] (T_right − T_left) を返す。
 * 各輪のトルクは (総トルク ∓ 差) / 2。無効時は 0 を返す。
 * @param total_torque_nm 左右の総駆動トルク (負 = 減速方向)
 * @param speed_m_s 車体前後方向の車速 (後退時は負)
 * @param yaw_rate_rad_s **実測**のヨーレート (左旋回が正)
 */
float TorqueVectoring_Update(TorqueVectoring* obj, float total_torque_nm, float speed_m_s,
                             float yaw_rate_rad_s);

/**
 * @brief 出力をリセットする (駆動していない間に呼ぶ。上位へ古い値を報告しないため)。
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
 * @brief 配分の比率 (左旋回で正 = 右輪が多い) を取得する。
 */
float TorqueVectoring_GetRatio(const TorqueVectoring* obj);

/**
 * @brief 要求した左右トルク差 [Nm] (T_right − T_left) を取得する。
 */
float TorqueVectoring_GetDiffTorque(const TorqueVectoring* obj);

/**
 * @brief 要求した左右トルク差をヨーモーメント [Nm] (左旋回方向が正) に換算して取得する。
 */
float TorqueVectoring_GetYawMoment(const TorqueVectoring* obj);

#endif  // TORQUE_VECTORING_H_
