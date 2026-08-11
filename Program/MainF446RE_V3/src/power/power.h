#ifndef POWER_H_
#define POWER_H_

#include "adc_dma.h"
#include "digitalinout.h"
#include "lpf.h"
#include "timer.h"

// ADC1 の Rank 順 (Core/Src/adc.c の MX_ADC1_Init() のチャンネル設定順と一致させること)
typedef enum {
  POWER_ADC1_CH_VOLTAGE_S = 0,  // シグナル系(ロジック電源)電圧
  POWER_ADC1_CH_VOLTAGE_P,      // 駆動系(モーター電源)電圧
  POWER_ADC1_CH_CURRENT_S,      // シグナル系電流
  POWER_ADC1_CH_CURRENT_P,      // 駆動系電流
  POWER_ADC1_CH_TEMP,           // マイコン内蔵温度センサ
  POWER_ADC1_NUM_CH,
} PowerAdc1Channel;

// 電源系のフォールト種別 (ビットマスク)。
// 過電流は電源を遮断するという不可逆な処置を伴うためラッチし、リセットするまで復帰しない。
// 電圧低下は警告するだけで何も遮断しないため、電圧が戻れば自動的にクリアされる。
typedef enum {
  POWER_FAULT_NONE = 0,
  POWER_FAULT_DRIVE_OVERCURRENT = 1u << 0,    // 駆動系過電流 → DRIVE_POWER 遮断 (ラッチ)
  POWER_FAULT_SIGNAL_OVERCURRENT = 1u << 1,   // シグナル系過電流 → LIDAR_POWER/DRIVE_POWER 遮断 (ラッチ)
  POWER_FAULT_DRIVE_UNDERVOLTAGE = 1u << 2,   // 駆動系バッテリー電圧低下 (警告のみ、自動復帰)
  POWER_FAULT_SIGNAL_UNDERVOLTAGE = 1u << 3,  // シグナル系バッテリー電圧低下 (警告のみ、自動復帰)

  // 系統ごとにまとめて判定するためのマスク
  POWER_FAULT_DRIVE_ANY = POWER_FAULT_DRIVE_OVERCURRENT | POWER_FAULT_DRIVE_UNDERVOLTAGE,
  POWER_FAULT_SIGNAL_ANY = POWER_FAULT_SIGNAL_OVERCURRENT | POWER_FAULT_SIGNAL_UNDERVOLTAGE,
} PowerFault;

// 駆動系の過電流保護閾値 [A]。超えたら DRIVE_POWER を遮断する。
#define POWER_DRIVE_OVERCURRENT_THRESHOLD_A 12.0f

// シグナル系の過電流保護閾値 [A] (仮値、実機の定常消費電流を測ってから決めること)。
// シグナル系はマイコン・Raspberry Pi・LiDAR・ライト類をまとめて賄うため、通常時のピークより
// 十分上に置く。超えたら切れる負荷 (LiDAR・駆動系) を落とす。
#define POWER_SIGNAL_OVERCURRENT_THRESHOLD_A 6.0f

// 電圧低下の判定閾値 [V]。駆動系・シグナル系とも 8セルのニッケル水素電池を電源とするため共通。
// ニッケル水素の放電終止電圧は 1セルあたり約1.0V で、これを下回ると過放電でセルを傷める。
#define POWER_BATTERY_CELL_COUNT 8
#define POWER_UNDERVOLTAGE_PER_CELL_V 1.0f
#define POWER_UNDERVOLTAGE_THRESHOLD_V (POWER_BATTERY_CELL_COUNT * POWER_UNDERVOLTAGE_PER_CELL_V)

// 電圧低下から復帰したと判定する閾値 [V] (1.1V/cell)。検出閾値と同じ値にすると、
// 閾値ぎりぎりで検出と復帰を繰り返して表示がばたつくため、ヒステリシスを持たせる。
#define POWER_UNDERVOLTAGE_RECOVERY_V (POWER_BATTERY_CELL_COUNT * 1.1f)

// ニッケル水素の満充電電圧 [V] (1.4V/cell)。残量表示のレンジ上端として使う。
#define POWER_BATTERY_FULL_V (POWER_BATTERY_CELL_COUNT * 1.4f)

// 異常の継続時間がこれを超えたときだけフォールトと判定する [s]。
// 過電流は電源投入時の突入電流を、電圧低下は加速時の一時的な電圧降下を無視するための時間。
// ニッケル水素は内部抵抗が低くないため負荷時のサグが大きく、電圧側は長めに取ってある。
#define POWER_OVERCURRENT_DEBOUNCE_S 0.05f
#define POWER_UNDERVOLTAGE_DEBOUNCE_S 1.0f

// 電圧のローパスフィルタ係数。Power_Update() を制御周期 500us で呼ぶ前提で時定数およそ0.5s。
// 生値のままだとモーター電流のリプルで閾値を跨いで上下し、継続時間の計測がそのたびにリセット
// されて電圧低下を検出できなくなるため、平均値で判定する。
// 過電流保護の判定 (Power_GetCurrentSignal/Drive) には掛けない。遮断が遅れて保護にならないため。
#define POWER_VOLTAGE_LPF_K 0.999

// 電流のローパスフィルタ係数 (上位への報告専用、時定数およそ0.1s)。PWM リプルやADCノイズを
// 均して読みやすくする一方、加減速に伴う実際の電流変化までは鈍らせたくないため電圧より短めに
// している。過電流保護は上記の通りこの値を使わず生値のまま判定する。
#define POWER_CURRENT_LPF_K 0.995

// 異常の検出・復帰を継続時間で判定するためのデバウンス状態
typedef struct {
  int state;    // デバウンス後の判定結果 (1 = 異常)
  int pending;  // 判定待ちの生の値。state と食い違っている間だけ timer が意味を持つ
  Timer timer;  // 生の値が state と食い違い始めてからの経過時間
} PowerDetector;

typedef struct {
  AdcDma* adc1;
  DigitalOut drive_power;  // DRIVE_POWER: モータードライバ電源スイッチ
  DigitalOut lidar_power;  // LIDAR_POWER: LiDAR 電源スイッチ

  int drive_power_request;  // 上位から要求されている駆動電源の状態 (待機中は 0)
  uint32_t faults;          // 発生した PowerFault のビットマスク

  LPF voltage_signal_lpf;
  LPF voltage_drive_lpf;
  int voltage_lpf_seeded;  // 初回の Power_Update() で実測値を初期値として与えたか

  LPF current_signal_lpf;
  LPF current_drive_lpf;
  int current_lpf_seeded;  // 初回の Power_Update() で実測値を初期値として与えたか

  PowerDetector drive_overcurrent;
  PowerDetector signal_overcurrent;
  PowerDetector drive_undervoltage;
  PowerDetector signal_undervoltage;
} Power;

/**
 * @brief 電源管理モジュールを初期化する。ADC1 (VOLTAGE_S/P, CURRENT_S/P, TEMP) を DMA で読む AdcDma と、
 * 駆動電源 (DRIVE_POWER) / LiDAR 電源 (LIDAR_POWER) スイッチのポート・ピンを渡す。
 * 初期化時は駆動電源・LiDAR 電源とも OFF。駆動電源は起動処理が終わってから Power_SetDrivePower() で入れる。
 */
void Power_Init(Power* obj, AdcDma* adc1,
                GPIO_TypeDef* drive_power_port, uint16_t drive_power_pin,
                GPIO_TypeDef* lidar_power_port, uint16_t lidar_power_pin);

/**
 * @brief 電源の監視とフォールト時の保護動作を更新する。制御周期ごとに呼ぶこと。
 * - 駆動系過電流: DRIVE_POWER を遮断する。
 * - シグナル系過電流: マイコン自身の電源なので切れないため、LIDAR_POWER と DRIVE_POWER を落として負荷を減らす。
 * - 電圧低下: 走行を止めると復帰できなくなるため遮断はせず、フォールトを立てるだけ (上位が停止を判断する)。
 * 過電流のフォールトはラッチされリセットするまで復帰しないが、電圧低下は電圧が戻れば自動でクリアされる。
 */
void Power_Update(Power* obj);

/**
 * @brief 発生中のフォールトを PowerFault のビットマスクで取得する。異常がなければ POWER_FAULT_NONE。
 * 過電流のビットは一度立つと消えないが、電圧低下のビットは電圧が復帰すると消える。
 */
uint32_t Power_GetFaults(Power* obj);

/**
 * @brief 駆動電源 (DRIVE_POWER) の ON/OFF を要求する。待機状態では OFF にしてモーターを無力化する。
 * 過電流でトリップ済みの場合は ON を要求しても投入されない。
 */
void Power_SetDrivePower(Power* obj, int on);

/**
 * @brief 駆動電源が実際に投入されているかを取得する。
 */
int Power_IsDriveOn(Power* obj);

/**
 * @brief LiDAR 電源 (LIDAR_POWER) の ON/OFF を切り替える。
 * シグナル系が過電流でトリップ済みの場合は ON を要求しても投入されない。
 */
void Power_SetLidarPower(Power* obj, int on);

/**
 * @brief シグナル系(ロジック電源)の入力電圧 [V] を取得する (R17=10k/R18=1k 分圧、MainBoard_V3_2 回路図実測)。
 * ADC の瞬時値。負荷変動によるリプルがそのまま乗るため、判定や表示には Filtered 版を使うこと。
 */
float Power_GetVoltageSignal(Power* obj);

/**
 * @brief 駆動系(モーター電源)の入力電圧 [V] を取得する (R19=10k/R20=1k 分圧、MainBoard_V3_2 回路図実測)。
 * ADC の瞬時値。負荷変動によるリプルがそのまま乗るため、判定や表示には Filtered 版を使うこと。
 */
float Power_GetVoltageDrive(Power* obj);

/**
 * @brief LPF を通したシグナル系の入力電圧 [V] を取得する。値は Power_Update() で更新される。
 */
float Power_GetVoltageSignalFiltered(Power* obj);

/**
 * @brief LPF を通した駆動系の入力電圧 [V] を取得する。値は Power_Update() で更新される。
 */
float Power_GetVoltageDriveFiltered(Power* obj);

/**
 * @brief シグナル系の消費電流 [A] を取得する (INA180A2 ゲイン50V/V, シャント R3=5mΩ)。
 * ADC の瞬時値。過電流保護の判定に使うため意図的に未フィルタ。上位への報告には Filtered 版を使うこと。
 */
float Power_GetCurrentSignal(Power* obj);

/**
 * @brief 駆動系の消費電流 [A] を取得する (INA180A2 ゲイン50V/V, シャント R28=5mΩ)。
 * ADC の瞬時値。過電流保護の判定に使うため意図的に未フィルタ。上位への報告には Filtered 版を使うこと。
 */
float Power_GetCurrentDrive(Power* obj);

/**
 * @brief LPF を通したシグナル系の消費電流 [A] を取得する。値は Power_Update() で更新される。
 */
float Power_GetCurrentSignalFiltered(Power* obj);

/**
 * @brief LPF を通した駆動系の消費電流 [A] を取得する。値は Power_Update() で更新される。
 */
float Power_GetCurrentDriveFiltered(Power* obj);

/**
 * @brief マイコン内蔵温度センサの温度 [degC] を取得する (工場較正なし、データシート標準値による概算)。
 */
float Power_GetTemperatureC(Power* obj);

#endif  // POWER_H_
