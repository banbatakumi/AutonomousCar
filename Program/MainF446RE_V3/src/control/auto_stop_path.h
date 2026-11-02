#ifndef AUTO_STOP_PATH_H_
#define AUTO_STOP_PATH_H_

#include <stdbool.h>

// ===========================================================================
// 自動停止の予想進路 (止まるまでに車体が掃く範囲)
//
// 舵角から自転車モデルの曲率 κ = tan(路面舵角) / ホイールベース を出し、後車軸の中心が半径 R = 1/|κ| の
// 円弧を進むとして、車体の矩形 (前後 body_rear_x〜body_front_x、左右 ±half_width) が掃く範囲に点が入るかを見る。
// 座標は base_link (後車軸の中心、x 前・y 左) [m]。舵角は左が正 (κ > 0 で左旋回)。
//
//   車体は回転の中心 C = (0, R) (左旋回に揃えたとき) のまわりを回る剛体なので、掃く範囲は C を中心とする円環
//     内径 = R − half_width          (内側の後車軸の位置が最も C に近い)
//     外径 = |(max(先端, 後端), R + half_width)| (外側の角が最も遠い。後退では前端が外へ振り出す)
//   のうち、C から見た角度 (後車軸の向きから進む向きに測る) が
//     α ≤ α_lead(r) + Φ、Φ = length / |(先端, R)|
//   のもの。Φ は先端の中心が length 進むまでに回る角度、α_lead(r) は半径 r で最も先にある車体の点の角度
//     r > |(先端, R + half_width)| なら外側の側面の上:  cos α_lead = (R + half_width) / r、α_lead < 0
//                                                       (後端の方が長いとき = 後退で前端が外へ振り出す分)
//     r ≥ |(先端, R − half_width)| なら前端の上:      sin α_lead = 先端 / r
//     それより内側なら内側の側面の上:                  cos α_lead = (R − half_width) / r
//   (半径ごとに先頭を取らずに内側の先端の角で一律に見ると、外側の終わりで最大 20cm 余分に拾う)。
//   角度は cos・sin の加法定理で比べる (点ごとに atan2 を呼ばない。2kHz で 179 点を見るため)。
//   α_lead + Φ が半周を超えるなら角度は見ない。負 (後退で前端が振り出す半径) なら sin(b − α) の符号で比べる。
//
//   - |κ| が AUTO_STOP_PATH_STRAIGHT_CURVATURE_1_M 未満は矩形 (x ≤ 先端 + length、|y| ≤ half_width) で見る
//     (R が大きいと円環の式は浮動小数の桁落ちを起こす。R = 50m なら 1.4m 進んで横へ 2cm しかずれない)
//   - 後退は x を反転して同じ式で見る (舵が同じなら後退でも回転の中心は同じ)。先端は後端になる
//   - 車体の横・後ろ (角度の下限) は見ない。呼び出し側が進行方向の半面の点だけを渡すこと
//     (車体の真横で幅の内側にある点は「当たる」に数える。曲げずに置いた矩形のときと同じ)
//   - 車体の外形の内側の点も「当たる」になる。呼び出し側が AutoStopPath_IsInsideBody で先に除くこと
// ===========================================================================

// これ未満の |曲率| [1/m] は直進の矩形で見る
#define AUTO_STOP_PATH_STRAIGHT_CURVATURE_1_M 0.02f

typedef struct {
  bool forward;
  bool straight;
  float turn_sign;     // 左旋回なら +1、右旋回なら -1。y に掛けて左旋回に揃える
  float radius_m;      // R
  float limit_x_m;     // 直進のとき: 先端 + length
  float half_width_m;
  float lead_m;        // 進む向きの先端までの距離
  float inner_m;       // R − half_width (内側の側面と C の距離)
  float outer_m;       // R + half_width (外側の側面と C の距離)
  float r_in_sq;       // 円環の内径の2乗
  float r_out_sq;      // 円環の外径の2乗
  float r_corner_sq;   // 内側の先端の角の半径の2乗 (これより外は前端が先頭)
  float r_lead_out_sq; // 外側の先端の角の半径の2乗 (これより外は外側の側面が先頭)
  float cos_sweep;     // cos Φ
  float sin_sweep;     // sin Φ
  bool sweep_all;      // Φ が半周以上 (円環の中は全部当たる)
} AutoStopPath;

/**
 * @brief 予想進路を作る。length_m は車体の先端 (後退なら後端) が止まるまでに進む距離。
 * body_front_x_m > 0 > body_rear_x_m (base_link 基準)。
 */
void AutoStopPath_Init(AutoStopPath* obj, bool forward, float curvature_1_m, float body_front_x_m,
                       float body_rear_x_m, float half_width_m, float length_m);

/**
 * @brief base_link 座標の点 (x_m, y_m) が予想進路に入るか。
 */
bool AutoStopPath_Contains(const AutoStopPath* obj, float x_m, float y_m);

/**
 * @brief base_link 座標の点 (x_m, y_m) が車体の外形 (前後 body_rear_x〜body_front_x、左右 ±body_half_width) の
 * 内側か。境界ちょうどは外に数える。車体の内側に障害物は無い (あれば既にぶつかっている) ので、内側の点は
 * 車体に載せた物 (支柱・配線など) の反射として予想進路の判定から除く。
 */
bool AutoStopPath_IsInsideBody(float x_m, float y_m, float body_front_x_m, float body_rear_x_m,
                               float body_half_width_m);

/**
 * @brief 路面舵角 [rad] とホイールベース [m] から曲率 [1/m] を出す (左が正)。
 */
float AutoStopPath_Curvature(float road_wheel_angle_rad, float wheelbase_m);

#endif  // AUTO_STOP_PATH_H_
