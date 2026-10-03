#include "app.h"

#include <stdbool.h>
#include <stdio.h>

#include "adc.h"
#include "adc_dma.h"
#include "buzzer.h"
#include "digitalinout.h"
#include "drive.h"
#include "encoder.h"
#include "heartbeat.h"
#include "i2c.h"
#include "imu.h"
#include "indicator.h"
#include "lidar.h"
#include "lighting.h"
#include "main.h"
#include "motors.h"
#include "power.h"
#include "pwm_out.h"
#include "range_sensor.h"
#include "ras_link.h"
#include "serial.h"
#include "steering.h"
#include "telemetry.h"
#include "timer.h"
#include "vehicle.h"
#include "watchdog.h"

// メインループの周期 [us]。Drive のフィルタ係数がこの周期を前提にしているため、
// 変更したら DRIVE_LPF_K_* も見直すこと
#define CONTROL_INTERVAL_US 500

// 独立ウォッチドッグのタイムアウト [ms]。メインループは 500us 周期なので3桁の余裕がある。
// LSI のばらつき (17〜47kHz) で実際は 340ms〜940ms に振れる
#define WATCHDOG_TIMEOUT_MS 500

// Raspberry Pi (上位) との通信。USART1、1000000bps (2026-09-26 に 250000bps から変更。
// APB2 45MHz (HCLK 180MHz の 1/4) で USARTDIV = 45M/(16×1M) = 2.8125 = 2+13/16 となり誤差 0%)。
// 受信リングバッファは制御ループが一度でも詰まると取りこぼすため大きめに取る。
// Serial_Available() はバッファが線速で満杯になる時間 (サイズ×10bit÷ボーレート) より長く
// ポーリングが空くと上書きとみなして読み捨てるので、ボーレートを4倍にした分サイズも4倍にし、
// その猶予を 250000bps・512バイトのときと同じ約20msに保つ
#define RAS_SERIAL_RX_BUF_SIZE 2048

// LD06 LiDAR (USART6, 230400bps)。約17.6kB/s 流れ込むため、制御周期 500us の間に
// 溜まる分 (約9バイト) に対して十分な余裕を取る
#define LIDAR_SERIAL_RX_BUF_SIZE 256

// ステアリング/左後輪/右後輪 各MDとのシリアル受信バッファサイズ [B]。
// MD とは USART2/USART3/UART4 とも 1000000bps (2026-09-27 に 250000bps から変更。APB1 45MHz で
// USARTDIV = 45M/(16×1M) = 2.8125 = 2+13/16 となり誤差 0%。MD側は PCLK1 36MHz ÷ 1M = 36 で誤差 0%)。
// MDは 500us ごとに11バイトの状態フレームを返すので1周期あたり約11バイト。Serial_Available() は
// バッファが線速で満杯になる時間より長くポーリングが空くと読み捨てるため、1Mbps では
// 64バイトだと 640us しか猶予が無く制御周期の揺らぎで誤判定しうる。256バイト (2.56ms) 取る
#define MD_SERIAL_RX_BUF_SIZE 256

// ---------------------------------------------------------------------------
// モジュールのインスタンス
// ---------------------------------------------------------------------------
static uint16_t adc1_buffer[POWER_ADC1_NUM_CH * POWER_ADC1_NUM_SAMPLES];
static uint16_t adc2_buffer[ENCODER_ADC2_NUM_CH * ENCODER_ADC2_NUM_SAMPLES];
static AdcDma adc1;
static AdcDma adc2;

static Power power;
static Encoder encoder;
static Imu imu;
static RangeSensor range_sensor;

static DigitalOut led1;
static DigitalOut led2;
static PwmOut led3;
static PwmOut led4;
static Lighting lighting;
static Buzzer buzzer;
static Indicator indicator;

static DigitalIn button1;
static DigitalIn button2;

static uint8_t steering_serial_rx_buf[MD_SERIAL_RX_BUF_SIZE];
static uint8_t rear_left_serial_rx_buf[MD_SERIAL_RX_BUF_SIZE];
static uint8_t rear_right_serial_rx_buf[MD_SERIAL_RX_BUF_SIZE];
static Serial steering_serial;
static Serial rear_left_serial;
static Serial rear_right_serial;
static Motors motors;
static Steering steering;
static Drive drive;

static uint8_t lidar_serial_rx_buf[LIDAR_SERIAL_RX_BUF_SIZE];
static Serial lidar_serial;
static Lidar lidar;

static uint8_t ras_serial_rx_buf[RAS_SERIAL_RX_BUF_SIZE];
static Serial ras_serial;
static RasLink ras_link;
static Heartbeat heartbeat;  // Raspberry Pi の生存監視 (RAS_SIG = PB12 の 100Hz 矩形波)

static Vehicle vehicle;
static Telemetry telemetry;

static Timer control_interval_timer;

// 起動を知らせるハザード2回点滅。Lighting_Update を回し続ける必要があるためブロッキングする
// (ウォッチドッグを起動する前に済ませること)
static void PlayStartupIndication() {
  Lighting_SetHeadlight(&lighting, LIGHTING_HEADLIGHT_OFF);
  Lighting_SetWinker(&lighting, LIGHTING_WINKER_HAZARD);
  Timer startup_hazard_timer;
  Timer_Init(&startup_hazard_timer);
  while (Timer_ReadMs(&startup_hazard_timer) < Lighting_GetWinkerPeriodMs() * 2) {
    Lighting_Update(&lighting);
  }
  Lighting_SetWinker(&lighting, LIGHTING_WINKER_OFF);
}

void Setup() {
  // 上位との通信が使う単調なマイクロ秒時計。他のどの初期化よりも先に立ち上げる
  Micros_Init();
  printf("Setup started\n");
  DigitalOut_Init(&led1, LED1_GPIO_Port, LED1_Pin);
  DigitalOut_Init(&led2, LED2_GPIO_Port, LED2_Pin);
  PwmOut_Init(&led3, &htim4, TIM_CHANNEL_1);
  PwmOut_Init(&led4, &htim4, TIM_CHANNEL_2);

  Lighting_Init(&lighting, &htim3, TIM_CHANNEL_1, &htim3, TIM_CHANNEL_3,
                &htim3, TIM_CHANNEL_2, &htim3, TIM_CHANNEL_4);

  AdcDma_Init(&adc1, &hadc1, adc1_buffer, POWER_ADC1_NUM_CH, POWER_ADC1_NUM_SAMPLES);
  AdcDma_Init(&adc2, &hadc2, adc2_buffer, ENCODER_ADC2_NUM_CH, ENCODER_ADC2_NUM_SAMPLES);
  Power_Init(&power, &adc1,
             DRIVE_POWER_GPIO_Port, DRIVE_POWER_Pin,
             LIDAR_POWER_GPIO_Port, LIDAR_POWER_Pin);
  Encoder_Init(&encoder, &adc2);
  Indicator_Init(&indicator, &power, &lighting, &led3, &led4);

  RangeSensor_Init(&range_sensor, TRIG_FRONT_GPIO_Port, TRIG_FRONT_Pin, ECHO_FRONT_GPIO_Port, ECHO_FRONT_Pin,
                   TRIG_REAR_GPIO_Port, TRIG_REAR_Pin, ECHO_REAR_GPIO_Port, ECHO_REAR_Pin);

  DigitalIn_Init(&button1, BUTTON1_GPIO_Port, BUTTON1_Pin);
  DigitalIn_Init(&button2, BUTTON2_GPIO_Port, BUTTON2_Pin);
  // 起動後のキャリブレーション要求はここでラッチする。以降の初期化に時間がかかるため、
  // 押しっぱなしを要求すると使いづらい
  bool calibrate_steering = DigitalIn_Read(&button1);
  bool calibrate_imu = DigitalIn_Read(&button2);
  // 較正モードで起動したことを即座に返す。特にIMUの静止較正は数秒かかるため、表示がないと
  // ボタンを認識したのか判断できない。各較正が終わった時点で消灯する
  DigitalOut_Write(&led2, calibrate_steering || calibrate_imu);

  // ステアリングモータ(USART2)・左後輪モータ(USART3)・右後輪モータ(UART4) のBLDC MDと通信する
  Serial_Init(&steering_serial, &huart2, steering_serial_rx_buf, MD_SERIAL_RX_BUF_SIZE);
  Serial_Init(&rear_left_serial, &huart3, rear_left_serial_rx_buf, MD_SERIAL_RX_BUF_SIZE);
  Serial_Init(&rear_right_serial, &huart4, rear_right_serial_rx_buf, MD_SERIAL_RX_BUF_SIZE);
  Motors_Init(&motors, &steering_serial, &rear_left_serial, &rear_right_serial);

  // ボタン2を押しながら起動 → 静止キャリブレーションをやり直して Flash に保存する (数秒かかる)
  // モーターに給電する前に済ませることで、車体が動かない状態で確実に測れる
  Imu_Init(&imu, &hi2c1, calibrate_imu);
  DigitalOut_Write(&led2, 0);

  // TIM2 CH3: BUZZER, APB1 タイマクロック 90MHz, Prescaler=0 (tim.c の MX_TIM2_Init と一致させる)。
  // ブザー配線はラズパイと共有しているため、鳴らしていない間はピンをラズパイへ明け渡す
  Buzzer_Init(&buzzer, &htim2, TIM_CHANNEL_3, 90000000U, 0U,
              BUZZER_GPIO_Port, BUZZER_Pin, GPIO_AF1_TIM2);
  Buzzer_PlayStartupMelody(&buzzer);
  PlayStartupIndication();

  // ボタン1を押しながら起動 → 現在のステアリング角度を直進中心点として記録・保存する。
  // 較正は MD からの状態フレーム受信が要るため、このときだけ駆動電源を入れて較正後に落とす。
  // 較正しない起動では Flash から読むだけで MD と話す必要がないので、駆動電源には一切
  // 触れない (毎回一瞬でも投入すると、その間だけMDが指令待ちで通電された状態になる)。
  // 上位が arm するまで駆動電源を入れないことで、Pi が未接続/DISARM の間は駆動系が
  // 無力化された状態を既定にする
  if (calibrate_steering) Power_SetDrivePower(&power, 1);
  Steering_Init(&steering, &motors.steering, calibrate_steering);
  if (calibrate_steering) Power_SetDrivePower(&power, 0);
  DigitalOut_Write(&led2, 0);

  Drive_Init(&drive, &motors, &encoder, &steering, &imu);

  // LD06 LiDAR (USART6, 230400bps)。
  // LIDAR_POWER は起動時点では意図的に投入しない (Pi 未接続/DISARM の間は駆動系だけでなく
  // LiDAR も無給電にする既定のため)。Lidar_Init() 自体はMCU側のペリフェラル (PWM/シリアル)
  // を組み立てるだけで LD06 本体の給電は前提にしないため、ここで呼んでも問題ない。
  // 実際の給電は Vehicle 層が上位の ARM 要求に応じて行う (UpdateLidarPower(), vehicle.c)
  Serial_Init(&lidar_serial, &huart6, lidar_serial_rx_buf, LIDAR_SERIAL_RX_BUF_SIZE);
  Lidar_Init(&lidar, &lidar_serial, &htim1, TIM_CHANNEL_1);

  // Raspberry Pi (USART1)。MD3系統の初期化が終わってから立ち上げることで、
  // 最初のテレメトリを送る時点でモータの状態が揃っている
  Serial_Init(&ras_serial, &huart1, ras_serial_rx_buf, RAS_SERIAL_RX_BUF_SIZE);
  RasLink_Init(&ras_link, &ras_serial);
  Heartbeat_Init(&heartbeat, RAS_SIG_GPIO_Port, RAS_SIG_Pin);

  Vehicle_Init(&vehicle, &ras_link, &drive, &steering, &lighting, &power, &heartbeat,
               &buzzer, &button2, &range_sensor, &lidar);
  Telemetry_Init(&telemetry, &ras_link, &vehicle, &power, &encoder, &imu, &lidar, &motors,
                 &steering, &drive, &range_sensor);

  Timer_Init(&control_interval_timer);
  printf("Setup finished\n");

  // ウォッチドッグは一度起動すると止められないため、ブロッキングする初期化 (IMU の静止
  // 較正・起動演出・ステアリング中心点の記録) がすべて終わってから最後に起動する。
  // 裏を返すと Setup 中のハングは検出できない
  if (!Watchdog_Start(WATCHDOG_TIMEOUT_MS)) {
    // RLRが12bitしかなく約8.19秒が上限のため、要求値がそれを超えるとクランプされる。
    // WATCHDOG_TIMEOUT_MS の変更でこれを踏んだことに気づけるよう警告を出す
    printf("Watchdog: requested timeout clamped to max (~8.19s)\n");
  }
}

// ECHOピンの変化割り込み (stm32f4xx_it.c の EXTI9_5_IRQHandler/EXTI15_10_IRQHandler 経由) から呼ばれる
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ECHO_FRONT_Pin) {
    RangeSensor_OnFrontEchoEdge(&range_sensor);
  } else if (GPIO_Pin == ECHO_REAR_Pin) {
    RangeSensor_OnRearEchoEdge(&range_sensor);
  } else if (GPIO_Pin == INT_Pin) {
    // MPU6050 の新しいサンプルが揃った → 非同期I2C読み出しを開始する
    Imu_OnDataReady(&imu);
  }
}

// MPU6050 の非同期読み出し完了 (HAL_I2C_Mem_Read_IT の完了コールバック)
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef* hi2c) {
  Imu_OnI2cRxComplete(&imu, hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef* hi2c) {
  Imu_OnI2cError(&imu, hi2c);
}

void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef* hi2c) {
  Imu_OnI2cError(&imu, hi2c);
}

static void UpdateSensors() {
  Encoder_Update(&encoder);
  Imu_Update(&imu);
  // DISARM中は自動停止判定 (vehicle.c) が超音波値を参照しないため、トリガ送出自体を
  // 止めて消費電力を抑える。自動停止が超音波を見るのは上位指令を ARM として適用している
  // 周期だけなので、それと同じ Vehicle_IsArmed で判定する。以前の Drive_IsEnabled は
  // MainApp() 冒頭の Drive_Enable で真から始まり、ApplyFailsafe() が Drive_Disable を
  // 呼ばないため、Pi 未接続・起動中・通信途絶・緊急停止の間はトリガが出続けていた
  // (2026-10-04 修正)。直前周期の Vehicle_Update が決めた値なので 1周期 (500us) 遅れるが、
  // 判定を止めるだけなので許容する
  if (Vehicle_IsArmed(&vehicle)) RangeSensor_Update(&range_sensor);
  Lidar_Update(&lidar);

  // IMU の I2C 復旧処理 (imu.c の Recover()) は ~190ms メインループをブロッキングする
  // (docs/code_review_2026-08-21.md A-3)。この間ハートビートのポーリングも止まるため、
  // 復旧直後にエッジを偶然取りこぼすと誤って緊急停止がラッチされうる。根本対策
  // (Recover のステートマシン化) は工数が大きいため、応急処置として基準時刻だけ
  // リセットしておく (実際の断線・Pi側の異常を見逃す窓ができるわけではない。
  // ブロッキングしていた間はそもそも判定できていなかった時間なので、その分を
  // 「途絶していない」として扱うだけ)
  if (Imu_ConsumeRecoveryRan(&imu)) {
    Heartbeat_ResetBaseline(&heartbeat);
  }
}

void MainApp() {
  // 上位 (Raspberry Pi) と繋がっていない/armされていない間は Vehicle 側のフェイルセーフが
  // ブレーキを掛け続けるので、Drive は制動モードで後輪を押さえる
  Drive_Enable(&drive);

  while (1) {
    // ループが回っていること自体が生存の証拠なので、先頭で無条件に叩く。
    // 個々のモジュールの異常はフォールトとして別に扱う (ウォッチドッグを異常時の
    // 停止手段に流用すると、リセットで状態が消えて原因が追えなくなる)
    Watchdog_Refresh();

    Power_Update(&power);
    Indicator_Update(&indicator, Vehicle_IsEstopLatched(&vehicle), Vehicle_GetWinkerRequest(&vehicle));
    Lighting_Update(&lighting);
    Buzzer_Update(&buzzer);
    UpdateSensors();

    // MDの状態 (舵角・後輪速度) は Vehicle/Drive が使うので先に受信し、Drive が決めた指令は
    // その周期のうちに送る (受信を後に回すと1周期古い値で制御することになる)
    Motors_Receive(&motors, Power_IsDriveOn(&power));
    Vehicle_Update(&vehicle);
    Drive_Update(&drive);
    Motors_Transmit(&motors, Power_IsDriveOn(&power));

    // Telemetry_Update 自体が内部で 100Hz (RAS_TELEMETRY_INTERVAL_US) に間引かれるため、
    // 毎周期呼んでも問題ない。実際の送信は RasLink 側でさらにキューイングされる
    Telemetry_Update(&telemetry);
    Telemetry_PublishLidarSector(&telemetry);
    RasLink_Update(&ras_link);

    // LED1 (赤): Raspberry Pi との COMMAND 通信が途絶している間だけ点灯 (Vehicle が
    // フェイルセーフへ落ちる基準 RasLink_IsCommandAlive() と同じ判定を流用)
    DigitalOut_Write(&led1, !RasLink_IsCommandAlive(&ras_link));

    // LED2 の点灯幅が「処理を終えてから次の周期まで空いているアイドル待ち時間」になる
    while (Timer_ReadUs(&control_interval_timer) < CONTROL_INTERVAL_US);
    Timer_Reset(&control_interval_timer);
  }
}
