#ifndef VEHICLE_H_
#define VEHICLE_H_

#include <stdbool.h>
#include <stdint.h>

#include "buzzer.h"
#include "digitalinout.h"
#include "drive.h"
#include "heartbeat.h"
#include "lidar.h"
#include "lighting.h"
#include "motors.h"
#include "power.h"
#include "range_sensor.h"
#include "ras_link.h"
#include "steering.h"
#include "timer.h"

// ===========================================================================
// 車両統括 (上位指令と安全層の調停)
//
// 上位 (Raspberry Pi) の指令・緊急停止・フェイルセーフのどれを車両へ適用するかを毎周期
// 決める層。個々のアクチュエータ制御 (Drive / Steering / Lighting) には踏み込まず、
// 「今どの指令源に従うか」と「上位の指令を車両が受け付けられる形に均す」ことだけを担う。
//
// 適用の優先順位:
//   1. 緊急停止がラッチ中          → フェイルセーフ
//   2. COMMAND が生きている        → 上位の指令
//   3. それ以外 (未接続・途絶)     → フェイルセーフ
// フェイルセーフを一度通ったら、上位が ARM を下ろして立て直すまで走らせない (rearm_required)。
//
// 「止める」の約束 (上位・MD と意味を揃えてある):
//   - 走行中に上位の ARM が外れても、すぐには駆動電源を切らない。電源を切ると MD ごと落ちて
//     制動できず惰行するので、停車するまで最大制動を続けてから切る (VEHICLE_DISARM_BRAKE_*)
//   - ARM されても、3台の MD が使える状態になるまでは走らせない (VEHICLE_MD_BOOT_TIMEOUT_S)。
//     走行中にどれかが使えなくなったら最大制動で止め、ARM を外すまで再開しない
//   - 制動して止まったら、後輪の位置保持へ移る (ブレーキホールド。制動トルクは車輪が止まると
//     抜けるので、それだけでは坂で保持できない)。上位の brake・自動停止・フェイルセーフの
//     どの制動でも同じ。上位から ON/OFF できる (RasConfig.brake_hold_enabled、既定は有効)
// ===========================================================================

// 走行中に ARM が外れたとき、停車を待って駆動電源を切るまでの時間の上限 [s]。最高速
// (DRIVE_MAX_SPEED_M_S) からの最大制動でも止まり切れる長さにしてある (減速度 2.5m/s² で 2秒)。
// 前輪エンコーダの異常で車速が 0 に落ちなくても、これを過ぎたら電源を切る
#define VEHICLE_DISARM_BRAKE_TIMEOUT_S 3.0f
// 緊急停止・COMMAND 途絶で停車を保持しているとき、止まってからこの時間が経ったら駆動電源を切る [s]。
// 電源を入れたまま位置保持を続けると、据え切りと同様にモータが過熱し、電池も減り続ける。
// 切ると坂では転がりうるが、30秒止まっていれば人が気づく時間は十分にあるとみなす。
// 戻すには、ラッチ (緊急停止) を解除して ARM を一度下ろしてから入れ直す (ARM で電源が入る)
#define VEHICLE_FAILSAFE_POWER_OFF_S 30.0f
// 自動停止の検知手段が失われている状態 (ARM 中に LiDAR も前後の超音波も使えない) が、この時間
// 続いたら TELEMETRY の AUTO_STOP_BLIND を立てる [s]。ARM 直後の LiDAR の起動待ちで立たないため
#define VEHICLE_AUTO_STOP_BLIND_DELAY_S 3.0f
// 駆動電源を入れてから、3台の MD が使える状態 (Motors_IsReady) になるのを待つ時間の上限 [s]。
// MD の起動は約1秒 (電流ゼロ点の校正を含む。惰行中の再 ARM ではやり直しで最大約4秒)
#define VEHICLE_MD_BOOT_TIMEOUT_S 5.0f
// 走行中に MD が「使えない」と読めてから md_fault をラッチするまでの継続時間 [s]。MD は状態を
// 500us ごとに返すので 40 フレームぶん。状態フレームの CRC は 8bit で、化けたフレームの 1/256 は
// 通る。1フレームで決めると、化けて異常ビットが立って見えただけ・電圧のサグが一瞬掛かっただけで
// 走行中に最大制動へ入り、ARM を外すまで戻らない (通信断の判定は Motors 側が 100ms 掛けている)
#define VEHICLE_MD_FAULT_DEBOUNCE_S 0.02f
// COMMAND.steer_rate_limit が 0 (指定なし) のときの舵角速度の上限 [rad/s]
#define VEHICLE_DEFAULT_STEER_RATE_RAD_S 0.5f

// クラクションの音程 [Hz]。押している間だけ鳴らすため、単発ビープではなく連続トーンで出す
#define VEHICLE_HORN_FREQ_HZ 2000
// ARM/DISARM の通知音。周波数を連続的に変える (ARM は上昇、DISARM は下降) ことで、
// 他の固定周波数の音と区別する。いずれも未実測の机上値
#define VEHICLE_ARM_SWEEP_LOW_HZ 300
#define VEHICLE_ARM_SWEEP_HIGH_HZ 4000
#define VEHICLE_ARM_SWEEP_DURATION_MS 500
// ARM解除からLiDAR電源を落とすまでの遅延 [s]。信号待ちなど短い停止のたびに電源を入り切り
// すると、LD06は起動から安定したスキャンが出るまで数秒かかるため、再ARM直後にセンサが
// 使えない空白ができてしまう。この遅延の間はARM解除中もLiDARを点けたままにする
#define VEHICLE_LIDAR_IDLE_OFF_DELAY_S 5.0f
// ブレーキホールド: 制動中に |車速| がこの値未満の状態が VEHICLE_BRAKE_HOLD_DELAY_S 続いたら、
// 後輪の位置保持 (サイドブレーキと同じ仕組み) へ移る。実車のブレーキホールドに当たる。
// 車速のしきい値を「ほぼ 0」にしないのは、MD の制動が低速で抜ける (20rad/s = 0.6m/s 以下で
// tanh で弱まり、2rad/s 未満で 0) ため。坂ではトルクの釣り合う速度でずるずる動き続け、
// しきい値が低いと永久に保持へ移れない (0.2m/s なら勾配 約12° まで拾える。0.1m/s だと 約6°)。
// 保持中にこの車速を超えたら (押された・保持し切れない) 制動トルクからやり直す。
// フェイルセーフでは、保持へ移る条件が成り立った時点でブレーキ灯も減光する (省電力)。
// どちらも机上値
#define VEHICLE_BRAKE_HOLD_SPEED_M_S 0.2f
#define VEHICLE_BRAKE_HOLD_DELAY_S 0.3f
// 自動停止 (RAS_CMD_FLAG_AUTO_STOP) の動的停止距離
// d_stop = v・VEHICLE_AUTO_STOP_DELAY_S + v²/(2・VEHICLE_AUTO_STOP_DECEL_M_S2) + マージン
// のうち、通信・処理・制動応答の遅延見積り [s] (未実測プレースホルダー。実測後に更新すること)
#define VEHICLE_AUTO_STOP_DELAY_S 0.1f
// 上式の減速度 [m/s^2]。★実測の制動は ABS 込みで約2.5m/s² (上位側の同定、フローリング) なので
// 少し楽観的。測り直したら更新すること
#define VEHICLE_AUTO_STOP_DECEL_M_S2 3.0f
// 安全マージンは上位が RasConfig.auto_stop_margin_cm (CONFIG_SET param_id = 0x0060) で
// cm単位の連続値として直接指定する (段階的なレベルではない。既定値・範囲は ras_link.h の
// RAS_AUTO_STOP_MARGIN_* 参照)
// 実車速がこの絶対値未満 (ほぼ静止) のときは、前後判定に実車速ではなく上位が指令した方向を
// 使う。静止中は実車速の符号が定まらず常に前方判定に固定されてしまうため
#define VEHICLE_AUTO_STOP_DIRECTION_DEADBAND_M_S 0.05f
// LiDAR のセーフティゾーンは、前方 (0度) か後方 (180度) へ伸ばした矩形で見る。
// 奥行きは動的停止距離 (+ LiDAR の取付オフセット)、片側の幅は次の値。扇形にすると遠方ほど
// 幅が広がり、コースの壁を拾って走行中に誤って止まるため、車体が通る幅だけを見る。
// 半車幅 9cm (surge_mk2/config/vehicle.toml の footprint、y=±0.09m) に余裕 6cm を足した値。
// (LiDAR の y 取付オフセットは 0)
#define VEHICLE_AUTO_STOP_LIDAR_HALF_WIDTH_CM 15.0f
// 上の矩形内で、停止距離以内の点がこの数以上あれば障害物ありと確定する。単発ノイズは
// 1点しか占めないため、時間方向のデバウンスではなく空間方向の点密度で誤検知を弾く
#define VEHICLE_AUTO_STOP_LIDAR_MIN_POINTS 3
// LiDARの結果に関わらず、超音波がこの距離未満を検知したら常に停止する (至近距離の保険。
// LiDARの最短測距距離や取付高さの死角を補う) [cm]
#define VEHICLE_AUTO_STOP_ULTRASONIC_NEAR_CM 5.0f

// 各センサの車体先端(バンパー)からの後退量 [cm]。上位 (Raspberry Pi) 側の
// raspi/config/vehicle.toml (base_link = 後輪車軸中心、実測確定) の
// footprint (前端 x=+30cm、後端 x=-7cm) とセンサ取付座標から算出した固定値。
// センサはいずれも車体先端より内側に付いているため、実測距離はそのままだと
// 「車体先端から障害物まで」より長く出る。しきい値側にこの分を足して補正する
// (実測距離 - オフセット = 車体先端から障害物までの真の距離、と同値の変形)
#define VEHICLE_AUTO_STOP_LIDAR_FRONT_OFFSET_CM 23.0f       // LiDAR(x=7cm) → 前端(x=30cm)
#define VEHICLE_AUTO_STOP_LIDAR_REAR_OFFSET_CM 14.0f        // LiDAR(x=7cm) → 後端(x=-7cm)
#define VEHICLE_AUTO_STOP_ULTRASONIC_FRONT_OFFSET_CM 4.0f   // 前方超音波(x=26cm) → 前端(x=30cm)
#define VEHICLE_AUTO_STOP_ULTRASONIC_REAR_OFFSET_CM 3.0f    // 後方超音波(x=-4cm) → 後端(x=-7cm)
// 緊急制動表示 (ESS, Emergency Stop Signal)。実車 (UN-ECE R48 / 保安基準) と同じく、
// 制動トルクの指令値ではなく「実際に一定車速以上から強く減速しているか」で点滅させる。
// 実車の規定は 50km/h 超かつ減速度 6m/s² 超で作動、2.5m/s² 未満で解除。ミニカーの制動性能に
// 合わせて縮小した値で、いずれも未実測プレースホルダー (実測後に更新すること)
#define VEHICLE_ESS_MIN_SPEED_M_S 1.0f       // 作動を許す最低車速
#define VEHICLE_ESS_ON_DECEL_M_S2 2.0f       // 作動する減速度
#define VEHICLE_ESS_OFF_DECEL_M_S2 0.8f      // 解除する減速度 (作動と差を付けてチャタリングを防ぐ)
#define VEHICLE_ESS_STOP_SPEED_M_S 0.1f      // これ未満になったら停止とみなして解除する
// 減速度を求める車速差分の窓幅 [s]。制御周期 (500us) ごとに差分を取ると前輪速度の
// 量子化ノイズ (σ≈2mm/s) が 2000倍されて数m/s² になりしきい値を跨いでしまうため、
// 窓を広げてノイズを 1/40 に落とす (遅れは窓幅ぶんで、表示用途なら問題にならない)
#define VEHICLE_ESS_DECEL_WINDOW_S 0.02f
// 減速度によるブレーキ灯の点灯 (制動モードでなくても、実際に減速していれば点ける)。
// 車速の窓差分 (VEHICLE_ESS_DECEL_WINDOW_S) は実機で σ≈2.3m/s² も揺れる (前輪エンコーダの
// 回転に同期したうねり) ので、1次遅れでならしてからしきい値に掛ける。値は実機の周回の記録
// (最高 2.8m/s・262s) を当てて決めた: これより短い時定数は減速の途中で
// 消えてまた点き、長いと点くのが遅れて加速に移ってからも残る
#define VEHICLE_BRAKE_LIGHT_DECEL_TAU_S 0.1f
#define VEHICLE_BRAKE_LIGHT_ON_DECEL_M_S2 1.5f   // 点ける減速度 (下げると減速していないのに点く回数が増える)
#define VEHICLE_BRAKE_LIGHT_OFF_DECEL_M_S2 0.5f  // 消す減速度
#define VEHICLE_BRAKE_LIGHT_MIN_ON_S 0.3f        // 一度点けたら最低これだけ点け続ける (ちらつき防止)

typedef struct {
  RasLink* ras_link;
  Drive* drive;
  Motors* motors;
  Steering* steering;
  Lighting* lighting;
  Power* power;
  Heartbeat* heartbeat;
  Buzzer* buzzer;
  DigitalIn* estop_reset_button;
  RangeSensor* range_sensor;
  Lidar* lidar;

  // 上位から指令された目標舵角を steer_rate_limit でレート制限したあとの値で、
  // 実際に Steering へ渡している量。テレメトリの steer_cmd もこれを返す
  // (目標車速の加速度レート制限は Drive 側 (Drive_SetTargetSpeed) が持つ)
  float applied_steer_rad;
  Timer command_rate_timer;

  uint8_t mode;  // RasMode
  bool estop_latched;
  bool horn_on;

  // 上位から要求されたウィンカー/ハザード状態 (flags2 の WINKER_LEFT/RIGHT をデコードしたもの)。
  // Lighting へ直接反映せず、フォールト時のハザード優先の調停は Indicator (src/hmi/indicator.c)
  // に委ねる (ハザードは Indicator が既に Lighting_SetWinker の唯一の呼び出し元だったため)
  LightingWinkerState winker_request;

  // ARMが外れている(=走らせる予定がない)時間を計り、LiDAR電源の遅延OFFに使う
  Timer lidar_idle_timer;
  bool lidar_on;
  // 回転 PWM を出しているか。給電が過電流で拒否・遮断されている間は lidar_on でも止める
  bool lidar_pwm_on;

  // 直近の周期で上位指令が ARM として適用されたか (フェイルセーフ・緊急停止中は false)。
  // 超音波のトリガ送出を止める判定に使う (Vehicle_IsArmed)
  bool armed;
  // 前周期の armed。ARM/DISARM の通知音をエッジで鳴らすのに使う
  bool prev_armed;

  // 制動中に停止が続いている時間と、ブレーキホールドの条件が成り立ったか (UpdateBrakeHold)。
  // Timer は約24秒で一周するので、成り立ったことはフラグで覚える (時間で判定し続けると
  // 周期的に保持が外れる)。実際に位置保持へ移るかは RasConfig.brake_hold_enabled しだい
  Timer brake_hold_timer;
  bool brake_hold;

  // フェイルセーフ (緊急停止・COMMAND 途絶・未接続) を通ったあと、上位が ARM を一度下ろすまで
  // ARM を受け付けない。ARM を立てたままの COMMAND が残っていても、緊急停止を解除した瞬間や
  // 通信が戻った瞬間にそのまま走り出さないようにする (上位が明示的に ARM し直すまで動かさない)
  bool rearm_required;
  bool rearm_logged;  // 受け付けなかったことを上位へ知らせたか (1回の保留につき1回)

  // 走行中に ARM が外れたあと、停車を待って最大制動を続けているか (この間は駆動電源を切らない)
  bool arm_requested_prev;
  bool disarm_braking;
  Timer disarm_brake_timer;

  // MD の起動待ちと健全性。md_started は駆動電源を入れてから3台が一度使える状態になったか、
  // md_fault は「起動待ちが時間切れ」か「走行中に使えなくなった」(駆動電源を切るまでラッチ)
  bool drive_power_prev;
  bool md_started;
  bool md_fault;
  Timer md_boot_timer;
  // 起動後に「使えない」と読め始めてからの時間 (VEHICLE_MD_FAULT_DEBOUNCE_S)。md_unready が偽の間は無効
  bool md_unready;
  Timer md_unready_timer;

  // 直近の周期で自動停止 (RAS_CMD_FLAG_AUTO_STOP) が実際に制動へ介入したか。
  // テレメトリの RAS_FLAG_AUTO_STOP_ACTIVE に使う
  bool auto_stop_active;
  // Steering へ適用済みのリンクの換算 (RasConfig の値は UpdateSteeringLinkage が停車・DISARM のときに適用)
  float linkage_gain;
  float linkage_cubic;

  // 自動停止の検知手段が使えない状態の継続 (VEHICLE_AUTO_STOP_BLIND_DELAY_S)
  bool auto_stop_blind;
  bool auto_stop_blind_pending;
  uint32_t auto_stop_blind_start_us;

  // フェイルセーフで停車を保持している間の経過 (Timer は約24秒で一周するので Micros で数える)。
  // VEHICLE_FAILSAFE_POWER_OFF_S を過ぎたら駆動電源を切る
  bool failsafe_idle;
  uint32_t failsafe_idle_start_us;

  // 緊急制動表示 (ESS) の判定用。decel_timer ごとに |車速| の差分から減速度を求める
  Timer ess_decel_timer;
  float ess_prev_abs_speed_m_s;
  float ess_decel_m_s2;  // 正=減速
  bool ess_active;

  // 減速度によるブレーキ灯 (VEHICLE_BRAKE_LIGHT_*)。ならした減速度 (正=減速) と点灯の状態
  float brake_light_decel_m_s2;
  bool brake_light_decel_on;
  Timer brake_light_hold_timer;
} Vehicle;

/**
 * @brief 車両統括を初期化する。estop_reset_button には緊急停止の解除に使うボタン
 * (ボタン2) を渡すこと。range_sensor と lidar には自動停止 (RAS_CMD_FLAG_AUTO_STOP) が
 * 参照する前後超音波センサ・LiDARを渡すこと。
 */
void Vehicle_Init(Vehicle* obj, RasLink* ras_link, Drive* drive, Motors* motors, Steering* steering,
                  Lighting* lighting, Power* power, Heartbeat* heartbeat, Buzzer* buzzer,
                  DigitalIn* estop_reset_button, RangeSensor* range_sensor, Lidar* lidar);

/**
 * @brief ハートビート監視・緊急停止の判定と、指令の車両への適用を1周期分行う。
 * Drive_Update / Motors_Transmit より前に、制御周期ごとに呼ぶこと。
 */
void Vehicle_Update(Vehicle* obj);

/**
 * @brief 緊急停止がラッチ中かを取得する。
 */
bool Vehicle_IsEstopLatched(const Vehicle* obj);

/**
 * @brief 直近の周期で上位指令が ARM として適用されたか (= 上位の指令で走れる状態か) を取得する。
 * COMMAND 途絶・緊急停止のフェイルセーフ中、Pi 未接続・起動中、MD の起動待ち、ARM が外れた
 * あとの停止待ち、フェイルセーフのあと上位が ARM を下ろすまでの間は false。駆動電源が入っているかどうかとは別 (Power_IsDriveOn)。
 */
bool Vehicle_IsArmed(const Vehicle* obj);

/**
 * @brief MD (3台のいずれか) が使えないために走行を止めているかを取得する。
 * 駆動電源を入れても時間内に揃わなかったか、走行中に無応答・異常になった。駆動電源が
 * 切れる (上位が ARM を外す) まで解除しない。
 */
bool Vehicle_IsMdFault(const Vehicle* obj);

/**
 * @brief 現在の走行モード (RasMode) を取得する。
 */
uint8_t Vehicle_GetMode(const Vehicle* obj);

/**
 * @brief 直近に Steering へ渡した路面舵角 [rad] を取得する (レート制限後の値)。
 */
float Vehicle_GetAppliedSteerRad(const Vehicle* obj);

/**
 * @brief 直近の周期で自動停止 (RAS_CMD_FLAG_AUTO_STOP) が実際に制動へ介入したかを取得する。
 */
bool Vehicle_IsAutoStopActive(const Vehicle* obj);

/**
 * @brief ハートビート (RAS_SIG) を一度も検出していない (緊急停止の層が働いていない) か。
 * 上位は ARM を通す前に確認すること。TELEMETRY の RAS_FLAG_NO_HEARTBEAT に使う。
 */
bool Vehicle_IsHeartbeatMissing(const Vehicle* obj);

/**
 * @brief ARM 中に LiDAR も前後の超音波も使えず、自動停止が何も検知できない状態が続いているか。
 * TELEMETRY の RAS_FLAG_AUTO_STOP_BLIND に使う。
 */
bool Vehicle_IsAutoStopBlind(const Vehicle* obj);

/**
 * @brief 上位から要求されたウィンカー/ハザード状態を取得する。フォールト時のハザード優先など
 * 実際に Lighting へ反映する際の調停は Indicator (src/hmi/indicator.c) が行うため、この値は
 * あくまで上位の「要求」であって実際の点灯状態と一致するとは限らない。
 */
LightingWinkerState Vehicle_GetWinkerRequest(const Vehicle* obj);

/**
 * @brief 左/右ウィンカーが実際に点滅中かどうかを取得する (TELEMETRY.flags の
 * WINKER_LEFT/RIGHT_ACTIVE 用)。Lighting の実際の状態を見るため、フォールトによる
 * ハザード優先で上位の要求と異なる状態になっている場合もそれを反映する。
 */
bool Vehicle_IsWinkerLeftActive(const Vehicle* obj);
bool Vehicle_IsWinkerRightActive(const Vehicle* obj);

#endif  // VEHICLE_H_
