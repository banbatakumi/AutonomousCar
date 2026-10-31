#ifndef ENCODER_DIRECTION_H_
#define ENCODER_DIRECTION_H_

#include <stdbool.h>
#include <stdint.h>

// ===========================================================================
// 前輪エンコーダの読みの向きの反転 (鏡像) を、後輪の速度と車速の推定と突き合わせて直す
//
// 2026-10-31 の会場の試走 (surge_20261031_044620) で、毎周同じ場所で前輪の左右の生の角度が
// **「2π − 本来の角度」に入れ替わり**、約1秒後に戻った (量はそのまま、向きだけが逆。左右は約60ms ずれて同時期)。
// 車速 (KF) が -1.5〜-2.0m/s になり、TC がトルクを切る・上位の MPC が舵を逆に振る・SLAM が崩れる、
// でコースから外れていた。LiDAR を地図に合わせた車体の速度では、前輪の中心は車輪の向きに +1.2〜1.5m/s で
// 進んでおり (横滑りほぼ 0)、後輪の周速はそれと一致していた = 前輪が実際に逆転したのではなく、読みが鏡像になった。
// AS5600 の DIR ピン (出力の増える向き) が切り替わったときと同じ出方。
//
// 直し方: 鏡像の間も回転量の大きさは正しいので、捨てずに向きだけを戻す。
//   基準の向き: 後輪 (ENCODER_DIRECTION_MIN_SPEED_M_S 以上) と車速の推定 (前周期の KF) が**同じ向きのときだけ**使う。
//     後輪だけを基準にすると、低μの旋回中の回生で後輪が後ろへ回る (シムの低μ×0.6 で後輪 -0.4m/s・車 +1.4m/s) のを
//     正として前輪を誤って反転させ、車速が 2.7m/s 狂った。車速の推定は前輪から作るが、鏡像は一瞬で入るので、
//     入った直後の推定はまだ正しい (以後はここで直すので正しいまま保たれる)
//   1. 基準があるとき、毎回:
//      - 向きが基準と食い違っていれば、量はそのまま向きを基準に合わせて返す (判定の窓を待たない)
//      - 鏡像に入る・戻る瞬間の1回は、角度が θ → 2π−θ と飛ぶので量も向きも当てにならない。
//        後輪の距離から見てありえない量 (ENCODER_DIRECTION_JUMP_*) なら、後輪の距離で置き換えて返す
//   2. 基準があるとき、ENCODER_DIRECTION_WINDOW_S ごとに、前輪が基準と逆向きに十分回っていたらその輪の向き (sign) を反転する。
//      基準が無い間 (低速・後輪のロック) と上位への周速の報告はこの向きで直す。鏡像が戻ったときも同じ判定で元へ戻る
//   3. 立て直し: 推定が既に狂っている (停止中に鏡像になって走り出した等) と 1・2 は働かない。ENCODER_DIRECTION_SLOW_WINDOW_S
//      ごとに、前輪と後輪が逆向きで**大きさが近い** (鏡像は量が変わらない。比 ENCODER_DIRECTION_SLOW_RATIO_*) なら推定に依らず反転する。
//      後輪の逆回転 (上の低μの例は比 4) や空転・ロックは大きさが合わないので拾わない
// ===========================================================================

// 調整値。ホストのテストが -D で上書きする。

// 判定の窓 [s]。10ms で 1.4m/s なら 14mm 進み、前輪の角度の雑音 (0.02mm) より十分大きい
#ifndef ENCODER_DIRECTION_WINDOW_S
#define ENCODER_DIRECTION_WINDOW_S 0.010f
#endif
// 後輪・車速の推定がこれ未満なら基準にしない [m/s] (停止・低速では向きが当てにならない)
#ifndef ENCODER_DIRECTION_MIN_SPEED_M_S
#define ENCODER_DIRECTION_MIN_SPEED_M_S 0.3f
#endif
// 前輪の逆向きの回転量が、後輪の距離のこの割合以上なら反転とみなす (旋回の内外輪差・後輪の空転で
// 前輪が後輪より遅く見えても拾えるよう小さめ。読みの雑音・ロックした前輪では超えない)
#ifndef ENCODER_DIRECTION_MIN_RATIO
#define ENCODER_DIRECTION_MIN_RATIO 0.25f
#endif

// 1回の回転量が「後輪の距離 × これ + FLOOR」を超えたら、鏡像の切り替わりの飛びとみなす。2kHz で 1.4m/s なら
// 1回 0.7mm に対して 4.8mm。飛びは 2θ × 半径 (最大 94mm) なので、θ がほぼ 0・π のときを除いて拾える
// (拾えない小さな飛びは誤差も小さい)。FLOOR の 2mm は 2kHz の1回で 4m/s に当たるので、ABS の制動で後輪が
// 遅く見えても (後輪 0.3m/s なら前輪 4.6m/s まで) 本物の回転を飛びと取り違えない
#ifndef ENCODER_DIRECTION_JUMP_RATIO
#define ENCODER_DIRECTION_JUMP_RATIO 4.0f
#endif
#ifndef ENCODER_DIRECTION_JUMP_FLOOR_M
#define ENCODER_DIRECTION_JUMP_FLOOR_M 0.002f
#endif

// 立て直しの窓 [s] と、前輪 ÷ 後輪の大きさの比の範囲。記録の鏡像中は 0.85〜1.2 (旋回の内外輪差を含む)。
// 窓が長いほど一時的な逆回転を拾いにくいが、狂った推定が長く残る
#ifndef ENCODER_DIRECTION_SLOW_WINDOW_S
#define ENCODER_DIRECTION_SLOW_WINDOW_S 0.100f
#endif
#ifndef ENCODER_DIRECTION_SLOW_RATIO_MIN
#define ENCODER_DIRECTION_SLOW_RATIO_MIN 0.6f
#endif
#ifndef ENCODER_DIRECTION_SLOW_RATIO_MAX
#define ENCODER_DIRECTION_SLOW_RATIO_MAX 1.7f
#endif

typedef struct {
  float sign;            // 生の回転量に掛ける向き (+1 / -1)
  float front_m;         // 2 の窓の中の、向きを直した前輪の回転量 [m] (前進が正。基準があった回だけ)
  float rear_m;          // 2 の窓の中の後輪の距離 [m] (前進が正。基準があった回だけ)
  float window_s;        // 2 の窓に溜めた時間
  float slow_front_m;    // 3 の窓の前輪の回転量 [m]
  float slow_rear_m;     // 3 の窓の後輪の距離 [m]
  float slow_window_s;   // 3 の窓に溜めた時間
  uint32_t flips;        // sign を反転した回数 (上位への報告用)
  uint32_t jumps;        // 切り替わりの飛びを後輪の距離で置き換えた回数
} EncoderDirection;

void EncoderDirection_Init(EncoderDirection* obj);

/**
 * @brief 前輪 1輪の回転量を1回ぶん取り込み、直した値を返す。Encoder の更新ごとに1回呼ぶ。
 * @param forward_m 前輪の回転量 [m] (取付向きの符号を直した、前進が正の値。舵角の射影はしない)
 * @param rear_speed_m_s 後輪の速度 [m/s] (左右平均、前進が正)
 * @param vehicle_speed_m_s 車速の推定 [m/s] (前周期の KF)
 * @param dt_s 前回の取り込みからの時間
 * @return 直した回転量 [m] (上の 1。車速の推定に使う)。窓の終わりで反転したときは、次の呼び出しから新しい向きになる
 */
float EncoderDirection_Apply(EncoderDirection* obj, float forward_m, float rear_speed_m_s, float vehicle_speed_m_s,
                             float dt_s);

static inline float EncoderDirection_GetSign(const EncoderDirection* obj) { return obj->sign; }
static inline uint32_t EncoderDirection_GetFlips(const EncoderDirection* obj) { return obj->flips; }

#endif /* ENCODER_DIRECTION_H_ */
