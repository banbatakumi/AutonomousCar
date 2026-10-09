#ifndef MOTORS_H_
#define MOTORS_H_

#include "bldc_motor.h"
#include "serial.h"
#include "timer.h"

// MD の状態フレームがこの時間 [ms] 来なければ通信断とみなす (MD は 500us ごとに返すので 200 フレームぶん)
#define MOTORS_COMM_TIMEOUT_MS 100
// 通信断の判定に、上の時間に加えて要求する「受信を試みたが1フレームも来なかった」連続回数。
// メインループ自体が止まっていた (受信を試みていない) 時間を MD の無応答と数えないため
#define MOTORS_COMM_MIN_IDLE_POLLS 20

typedef enum {
  MOTORS_INDEX_REAR_LEFT = 0,
  MOTORS_INDEX_REAR_RIGHT = 1,
  MOTORS_INDEX_STEERING = 2,
  MOTORS_COUNT = 3,
} MotorsIndex;  // TELEMETRY の motor_current / md_status と同じ並び [RL, RR, ST]

// MD 1台ぶんの通信監視。MD が無言になっても保持値は最後の正常値のまま固まるため、
// これが無いと古い値を現在値だと信じ続ける
typedef struct {
  uint32_t last_rx_count;
  uint16_t idle_polls;
  Timer timer;
  bool ok;
} MotorsCommWatch;

// 走行系3モータ (ステアリング/左後輪/右後輪) のBLDCモータドライバ通信をまとめて保持する
typedef struct {
  BldcMotor steering;
  BldcMotor rear_left;
  BldcMotor rear_right;
  MotorsCommWatch comm_watch[MOTORS_COUNT];
} Motors;

/**
 * @brief 走行系3モータを初期化する。各モータのMDが接続されたUARTのSerialインスタンスを渡す
 * (ステアリング: USART2, 左後輪: USART3, 右後輪: UART4)。
 */
void Motors_Init(Motors* obj, Serial* steering_serial, Serial* rear_left_serial, Serial* rear_right_serial);

/**
 * @brief 3モータ分の状態フレームを受信する。制御周期ごとに、MDの状態 (舵角・後輪速度等) を
 * 使うモジュール (Vehicle/Drive) の更新より前に呼ぶこと。後に呼ぶと1周期 (500us) 古い値で
 * 制御することになる。
 * powered は3モータ共通の給電状態 (DRIVE_POWER の実際の状態) を渡す。偽の間は3モータとも
 * MDが給電されておらず状態フレームが届かないため、受信を行わず BldcMotor_Reset() で
 * 受信由来の情報を初期値へ落とす (給電が切れたことに気付かず古い値を保持し続けないため)。
 */
void Motors_Receive(Motors* obj, bool powered);

/**
 * @brief 3モータ分の指令を送信する。制御周期ごとに、指令を決めるモジュール (Drive) の
 * 更新の直後に呼ぶこと。間引かずに毎周期送るので、決めた指令がその周期のうちにMDへ届く。
 * powered が偽の間 (MDに給電されていない) は送信しない。
 */
void Motors_Transmit(Motors* obj, bool powered);

/**
 * @brief index (MotorsIndex) の BldcMotor を取得する。
 */
BldcMotor* Motors_Get(Motors* obj, int index);

/**
 * @brief index (MotorsIndex) の MD から状態フレームが届き続けているかを取得する。
 * 給電していない間、および MOTORS_COMM_TIMEOUT_MS 届いていなければ偽。
 */
bool Motors_IsCommOk(const Motors* obj, int index);

/**
 * @brief index (MotorsIndex) の MD が走行に使える状態でない理由を返す (使えるなら NULL)。
 * 「使える」= 状態フレームが届いていて、異常ビット (過電流・過熱・電源電圧・エンコーダ未校正) が
 * 立っておらず、こちらが指令したトルク上限を MD が適用している。
 * 給電直後は MD の起動 (約1秒) が終わるまで "no response" か "torque limit not applied" を返す。
 */
const char* Motors_NotReadyReason(const Motors* obj, int index);

/**
 * @brief 3台の MD がすべて走行に使える状態か (Motors_NotReadyReason がすべて NULL か)。
 */
bool Motors_IsReady(const Motors* obj);

#endif  // MOTORS_H_
