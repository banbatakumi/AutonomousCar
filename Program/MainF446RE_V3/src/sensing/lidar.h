#ifndef LIDAR_H_
#define LIDAR_H_

#include <stdbool.h>
#include <stdint.h>

#include "ld06.h"
#include "pwm_out.h"
#include "serial.h"
#include "timer.h"

// LD06 の生パケットを、上位 (Raspberry Pi) へ送るセクタ単位の点群へ組み立てる。
//
// 1周を1度ビン×360点に整形し、30点 (=30度) ずつのセクタとして切り出す。1周まとめて
// 送らないのは、貯めるだけで100msの遅延が乗るのと、セクタごとにタイムスタンプが入って
// 走行中の点群の歪み補正ができるため (プロトコル仕様 docs/pi_uart_protocol_v0.4_request.md)。
//
// LD06 は約450点/周を出すので1度ビンに複数点が入ることがある。そのときは
// **ビン中心に最も近い点**を採用する。最短距離を採る方が安全側に見えるが、地図生成用の
// データを系統的に近距離側へ歪ませるとスキャンマッチングの精度が落ちるため、
// 安全側の判断 (最短距離での緊急停止など) を実装するときは別途この下に
// 360点の最小距離配列を持たせること。

#define LIDAR_SECTOR_NUM 12
#define LIDAR_POINTS_PER_SECTOR 30
#define LIDAR_DEG_PER_SECTOR (360 / LIDAR_SECTOR_NUM)

// **この機体はセンサの0度が機体前方を向くよう取り付けてある**が、センサを裏向き (PCB面が下)
// に取り付けているため角度の増加方向が上から見て機体と逆周りになっており、左右が鏡像になる
// (0度=前方・180度=後方は反転軸上なので影響を受けない)。
//
// この鏡像補正はあえて本モジュール (このセクタ組み立てのステートマシン) では行わない。
// 一度 PlacePoint() に渡す角度を 360-angle に置き換えて試したところ、計算コスト自体は
// 無視できる差のはずなのに上位 (Raspberry Pi) 側で通信エラーが増発し原因不明のまま再現した
// (メカニズムは未解明。詳しい調査はしていない)。0度・180度がちょうどセクタ境界に乗るため
// 正確な鏡像補正は1セクタの30点が出力側の隣接2セクタにまたがって分裂し、複数セクタを
// またいだバッファの組み替えが要る点も踏まえ、実装が確実に安定している「センサ基準の
// 角度をそのまま送る」側を維持する。左右の解釈は受信側 (Raspberry Pi) で
// real_angle_deg = (360 - (sector_idx*30 + point_index)) % 360 として行うこと。
//
// 回転モータの PWM。LD06 のデータシート指定は 30kHz で、デューティで回転数が決まる。
// TIM1 は APB2 のタイマなのでクロックは 180MHz (CubeMX の Period 65535 では 2.7kHz に
// なってしまうため、Lidar_Init で ARR を張り替える)
#define LIDAR_PWM_FREQ_HZ 30000U
#define LIDAR_PWM_PERIOD (180000000U / LIDAR_PWM_FREQ_HZ - 1U)
#define LIDAR_PWM_DUTY 0.4f  // ★実機で回転数を見ながら詰めること (10Hz 回転が目標)

// この時間パケットを受信できなければ異常とみなす [us]
#define LIDAR_TIMEOUT_US 300000u

typedef struct {
  uint8_t sector_idx;                              // 0-11
  uint16_t distance_mm[LIDAR_POINTS_PER_SECTOR];   // 0 = 無効
  uint8_t intensity[LIDAR_POINTS_PER_SECTOR];
  uint32_t t_start_us;    // このセクタの先頭点を取得した時刻
  uint16_t duration_us;   // 先頭点から末尾点までの所要時間
  uint16_t rot_speed_dps; // 回転速度 [1 deg/s] (10Hz 回転なら 3600)
} LidarSector;

typedef struct {
  LD06 ld06;
  PwmOut motor;

  LidarSector building;  // 組み立て中のセクタ
  // 各点がビン中心からどれだけずれているか [deg]。同じビンに複数点が来たとき、
  // より中心に近い方で置き換えるために保持する
  float bin_error_deg[LIDAR_POINTS_PER_SECTOR];
  bool building_has_point;
  uint32_t building_last_us;

  LidarSector ready;  // 送信待ちのセクタ (1個)
  bool has_ready;

  uint32_t last_packet_us;
  uint32_t sector_count;       // 組み立てが完了したセクタの累積数
  uint32_t sector_drop_count;  // 取り出される前に上書きしてしまったセクタ数

  // Lidar_TakeReadySector() による取り出しとは独立に保持する、1度ビンごとの直近距離。
  // 自動停止などの安全判定がテレメトリ送信の消費タイミングに左右されないようにするために持つ
  uint16_t point_distance_mm[360];              // 0 = 無効
  uint32_t sector_update_us[LIDAR_SECTOR_NUM];  // 各セクタの最終更新時刻
} Lidar;

typedef struct {
  uint16_t fresh_count;  // 窓内で直近 LIDAR_TIMEOUT_US 以内に更新されたビンの数
  uint16_t hit_count;    // さらに max_distance_cm 以内だったビンの数
} LidarRoiResult;

/**
 * @brief LiDAR を初期化する。serial には LD06 が繋がった USART6 (230400bps) を、
 * htim/channel には回転モータ PWM (LIDAR_OUT = PA8, TIM1 CH1) を渡す。
 * PWM の周期を 30kHz へ張り替えてからモータを回し始める。
 * ここではMCU側のペリフェラル (PWM/シリアル) を組み立てるだけで LD06 本体の給電は
 * 前提にしないため、LIDAR_POWER が未投入 (Setup() 時点など) のまま呼んでよい。
 * 実際の給電は上位の ARM 要求に応じて Vehicle 層が行う (UpdateLidarPower(), vehicle.c)。
 */
void Lidar_Init(Lidar* obj, Serial* serial, TIM_HandleTypeDef* htim, uint32_t channel);

/**
 * @brief 受信データを処理してセクタを組み立てる。メインループごとに呼ぶこと。
 */
void Lidar_Update(Lidar* obj);

/**
 * @brief 組み上がったセクタを1つ取り出す。無ければ NULL。
 * 返したポインタの中身は次に Lidar_Update() を呼ぶまで有効。
 */
const LidarSector* Lidar_TakeReadySector(Lidar* obj);

/**
 * @brief LiDAR から正常にパケットを受信できているかを取得する。
 */
bool Lidar_IsOk(const Lidar* obj);

/**
 * @brief center_deg ± half_width_deg の角度窓を、距離 max_distance_cm 以内かどうかで走査する。
 * fresh_count が0なら窓内に新しいデータが無い(死角・センサ不調)ことを示し、呼び出し側は
 * 他のセンサへフォールバックすること。
 * 注意: このモジュールはセンサ基準角度をそのまま保持しており(冒頭コメント参照)、0度(前方)・
 * 180度(後方)以外は左右が鏡像になる。center_deg は 0 または 180 付近でのみ安全に使えること。
 */
LidarRoiResult Lidar_QueryRoi(const Lidar* obj, float center_deg, float half_width_deg,
                              float max_distance_cm);

/**
 * @brief 回転モータのデューティを設定する (0.0-1.0)。回転数の調整用。
 */
void Lidar_SetMotorDuty(Lidar* obj, float duty);

#endif  // LIDAR_H_
