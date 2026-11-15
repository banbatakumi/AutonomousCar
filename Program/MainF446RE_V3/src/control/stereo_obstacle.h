#ifndef STEREO_OBSTACLE_H_
#define STEREO_OBSTACLE_H_

#include <stdbool.h>
#include <stdint.h>

#include "auto_stop_path.h"

// ===========================================================================
// ステレオ (Pi が照合した障害物の点) を自動停止の入力に足す。
//
// 判定の責任は STM32 に置く (最終防壁が Pi・UART の生死に依存しないため)。Pi は base_link 座標の
// 点列 (RAS_TYPE_STEREO_OBS) を 10Hz で送るだけで、停止の判断はしない。
//
//   - ステレオは「止める条件を足す」だけ。古い (途絶した) ときは何も言わず、LiDAR・超音波だけで判定する
//   - 誤検知を避けるため、直近 confirm_frames (2〜3) フレームの全部で予想進路内に min_points 点以上あるときだけ
//     当たりにする (床の取り違えは単発のことが多い)。確認で待つフレームは遅れとして停止距離に足す
//   - 車体の外形を前・横へ広げた範囲 (自車除外) の点は数えない (車体の外に出ている前輪などを拾うため)
//   - 点は撮影から (Pi の処理 + 通信 + 受信してからの経過) だけ古い。その間に車は動いたので、
//     予想進路を v × (点の古さ) だけ長く取る (障害物は静止していると見て、今の距離は点の距離より短い)
//   - 前進のみ。ステレオは前方にしか付いていないので、後退では見ない
//   - 点が 0 個のフレームも「ステレオは生きていて何も見ていない」として受ける (途絶と区別する)
// ===========================================================================

#define STEREO_OBS_MAX_POINTS 40

typedef struct {
  bool valid;
  uint8_t count;
  uint32_t rx_us;      // 受信時刻 (Micros)
  uint32_t age_rx_us;  // 受信した時点で、撮影からすでに経っていた時間
  float x_m[STEREO_OBS_MAX_POINTS];  // base_link
  float y_m[STEREO_OBS_MAX_POINTS];
} StereoFrame;

#define STEREO_OBS_MAX_CONFIRM 3

typedef struct {
  StereoFrame frames[STEREO_OBS_MAX_CONFIRM];  // [0] が最新、[1] がその一つ前、[2] がさらに前
} StereoObstacles;

typedef struct {
  float body_front_x_m;
  float body_rear_x_m;
  float body_half_width_m;  // 車体の外形 (この内側の点は数えない)
  float path_half_width_m;  // 予想進路の半幅
  float curvature_1_m;
  float d_stop_m;           // 車体の先端が止まるまでに進む距離 (LiDAR と同じ式)
  float speed_abs_m_s;
  float confirm_period_s;   // 確認フレームで足す遅れ (フレーム間隔 × (confirm_frames - 1))
  float comm_delay_s;       // 撮影時刻に含まれない、Pi → STM32 の送出・UART の遅れ
  float max_age_s;          // これより古いフレームは使わない
  uint8_t min_points;       // 1 フレームあたりの当たり点数のしきい値
  uint8_t confirm_frames;   // 連続して当たりのフレーム数 (2〜STEREO_OBS_MAX_CONFIRM)
  // 自車除外: 車体の外形を前へ・横へ広げた範囲の点は数えない。ステレオは近距離の測距誤差があり、
  // 車体の外に出ている前輪・ナックルを障害物として拾う (実走行ログで再現)。広げた分の手前は
  // LiDAR・超音波の近接 (5cm) が受け持つ
  float self_ext_front_m;
  float self_ext_side_m;
} StereoEvalParams;

void StereoObstacles_Init(StereoObstacles* obj);

/**
 * @brief 受信したフレームを入れる。count は STEREO_OBS_MAX_POINTS に切る。xy_mm は x, y の順の i16 [mm]
 * (base_link)。age_ms は Pi が送る直前に測った「撮影からの時間」。
 */
void StereoObstacles_Push(StereoObstacles* obj, uint32_t now_us, uint16_t age_ms, uint8_t count,
                          const int16_t* xy_mm);

/**
 * @brief 最新フレームが max_age_s 以内か (ステレオが生きているか)。
 */
bool StereoObstacles_IsFresh(const StereoObstacles* obj, uint32_t now_us, float max_age_s);

/**
 * @brief 前進中、予想進路に障害物があるか (最新 2 フレームの両方で min_points 点以上)。
 * 後退・フレームが古い・前のフレームが無いときは false。
 */
bool StereoObstacles_Evaluate(const StereoObstacles* obj, uint32_t now_us, bool forward,
                              const StereoEvalParams* p);

#endif  // STEREO_OBSTACLE_H_
