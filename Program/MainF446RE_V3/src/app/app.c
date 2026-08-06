#include "app.h"

// メインループの周期 [us]。Drive のフィルタ係数がこの周期を前提にしているため、
// 変更したら DRIVE_LPF_K_* も見直すこと
#define CONTROL_INTERVAL_US 500

static uint16_t adc1_buffer[POWER_ADC1_NUM_CH];
static uint16_t adc2_buffer[ENCODER_ADC2_NUM_CH];

AdcDma adc1;
AdcDma adc2;

Power power;
Encoder encoder;
Imu imu;
Ultrasonic ultrasonic_front;
Ultrasonic ultrasonic_rear;

DigitalOut led1;
DigitalOut led2;
PwmOut led3;
PwmOut led4;

Lighting lighting;

Serial steering_serial;
Serial rear_left_serial;
Serial rear_right_serial;
Motors motors;
Steering steering;
Drive drive;

DigitalIn button1;
DigitalIn button2;

Buzzer buzzer;

Timer control_interval_timer;

// ---------------------------------------------------------------------------
// 電源状態の表示 (LED3: シグナル系, LED4: 駆動系)
//
// 電圧を「呼吸」の周期にマップして脈打たせる。周期から電圧の目安が読めるうえ、脈が
// 止まれば制御ループが回っていないことも同時に分かる。輝度に連続量を載せても人間には
// 読めないため、輝度は1bitの情報として使い、その系統にフォールトが出たときだけ最大にする。
// ---------------------------------------------------------------------------
#define BREATH_PERIOD_FULL_MS 2000.0f  // 満充電時はゆったり脈打つ
#define BREATH_PERIOD_EMPTY_MS 300.0f  // 終止電圧に近いほど速く脈打つ (速い=危険、警報の慣習に合わせる)
#define BREATH_DUTY_NORMAL 0.1f        // 正常時のピーク輝度
#define BREATH_DUTY_FAULT 1.0f         // フォールト時のピーク輝度

typedef struct {
  PwmOut* led;
  Timer timer;
} BreathLed;

static BreathLed breath_signal;
static BreathLed breath_drive;

static void BreathLed_Init(BreathLed* obj, PwmOut* led) {
  obj->led = led;
  Timer_Init(&obj->timer);
}

// voltage には Power 側で LPF を通した値を渡すこと。生値だと周期がガタついて読めない
static void BreathLed_Update(BreathLed* obj, float voltage, bool fault) {
  float level = (voltage - POWER_UNDERVOLTAGE_THRESHOLD_V) /
                (POWER_BATTERY_FULL_V - POWER_UNDERVOLTAGE_THRESHOLD_V);
  level = Constrain(level, 0.0f, 1.0f);
  float period_ms = BREATH_PERIOD_EMPTY_MS + level * (BREATH_PERIOD_FULL_MS - BREATH_PERIOD_EMPTY_MS);

  uint32_t elapsed_ms = Timer_ReadMs(&obj->timer);
  if ((float)elapsed_ms >= period_ms) {
    Timer_Reset(&obj->timer);
    elapsed_ms = 0;
  }

  // 波形全体を周期に比例させることで、周期が変わっても点灯時間の割合 (=平均輝度) が変わらない。
  // 点灯時間を固定したまま周期だけ縮めると明るさまで変化し、輝度の1bit情報と混ざってしまう
  float shape = (1.0f - CosDeg((int)(360.0f * elapsed_ms / period_ms))) * 0.5f;
  // 人間の輝度知覚は対数的なので、shape をそのまま duty にすると暗い側が潰れる
  float peak = fault ? BREATH_DUTY_FAULT : BREATH_DUTY_NORMAL;
  PwmOut_Write(obj->led, peak * shape * shape);
}

static void UpdatePowerIndication() {
  uint32_t faults = Power_GetFaults(&power);
  BreathLed_Update(&breath_signal, Power_GetVoltageSignalFiltered(&power),
                   (faults & POWER_FAULT_SIGNAL_ANY) != 0);
  BreathLed_Update(&breath_drive, Power_GetVoltageDriveFiltered(&power),
                   (faults & POWER_FAULT_DRIVE_ANY) != 0);
}

// 何らかの異常が出ていればハザードを点滅させ、車外から異常だと分かるようにする。
// 復帰しうる異常 (電圧低下) もあるため、消えたらハザードも消す。
// 電源系以外のモジュールのエラーも、実装したらここに集約する。
// ハザードはウィンカーと灯火を共有するので、方向指示を実装したらここで優先度を調停すること。
static void UpdateFaultIndication() {
  bool fault = Power_GetFaults(&power) != POWER_FAULT_NONE;
  Lighting_SetWinker(&lighting, fault ? LIGHTING_WINKER_HAZARD : LIGHTING_WINKER_OFF);
}

// ---------------------------------------------------------------------------
// 走行テスト (ボタン1で開始 → 最大加速 → 前方の障害物を検知したら急停止)
//
// Raspberry Pi からの走行指令が未実装のため、駆動系と前方超音波センサの動作を
// 車両単体で確認するための仮ロジック。上位通信を実装したらここを差し替える。
// ---------------------------------------------------------------------------

// 到達し得ない目標を与えて車速PIを飽和させ、常に最大トルク = 最大加速にする
#define TEST_TARGET_SPEED_M_S DRIVE_MAX_SPEED_M_S
// 制動距離に加え、超音波の更新間隔 (60ms) の間に進む距離が空走になるため大きめに取る
#define TEST_STOP_DISTANCE_CM 100.0f
// 障害物を検知できないまま走り続けないための保険
#define TEST_TIMEOUT_MS 3000
// ボタンから手を離してテスト車両から離れるための猶予
#define TEST_COUNTDOWN_MS 2000
#define TEST_COUNTDOWN_BEEP_ON_MS 100
#define TEST_COUNTDOWN_BEEP_OFF_MS 400
#define TEST_BUTTON_DEBOUNCE_MS 20

typedef enum {
  TEST_STATE_IDLE = 0,   // ボタン待ち (Drive は目標車速0 = 停車保持)
  TEST_STATE_COUNTDOWN,  // 発進前のカウントダウン
  TEST_STATE_ACCEL,      // 最大加速中
  TEST_STATE_BRAKE,      // 急停止中
} TestState;

static TestState test_state = TEST_STATE_IDLE;
static Timer test_timer;

static Timer button1_debounce_timer;
// 起動時のボタン1押下 (ステアリング較正) を押しっぱなしのままメインループへ入っても
// テスト開始と誤認しないよう、押されている状態から始める
static int button1_stable = 1;

// チャタリングを除いたうえで「押された瞬間」の1回だけ真を返す
static bool IsButton1Pressed() {
  int raw = DigitalIn_Read(&button1);
  if (raw == button1_stable) {
    Timer_Reset(&button1_debounce_timer);
    return false;
  }
  if (Timer_ReadMs(&button1_debounce_timer) < TEST_BUTTON_DEBOUNCE_MS) return false;
  button1_stable = raw;
  return raw != 0;
}

static void StopTest(bool beep) {
  Drive_SetTargetSpeed(&drive, 0.0f);
  Lighting_SetBrake(&lighting, true);
  if (beep) Buzzer_Beep(&buzzer, 3000, 200);
  Timer_Reset(&test_timer);
  test_state = TEST_STATE_BRAKE;
}

static void UpdateDriveTest() {
  bool pressed = IsButton1Pressed();
  bool fault = Power_GetFaults(&power) != POWER_FAULT_NONE;
  float front_cm = Ultrasonic_GetDistanceCm(&ultrasonic_front);
  // 計測不能 (ULTRASONIC_NO_ECHO = -1) を至近距離と読み違えないよう、正の値だけを障害物とみなす
  bool obstacle = front_cm > 0.0f && front_cm < TEST_STOP_DISTANCE_CM;

  switch (test_state) {
    case TEST_STATE_IDLE:
      // 障害物の目の前や電源異常の状態では発進させない
      if (pressed && !obstacle && !fault) {
        Buzzer_BeepPattern(&buzzer, 2000, TEST_COUNTDOWN_BEEP_ON_MS, TEST_COUNTDOWN_BEEP_OFF_MS,
                           TEST_COUNTDOWN_MS / (TEST_COUNTDOWN_BEEP_ON_MS + TEST_COUNTDOWN_BEEP_OFF_MS));
        Timer_Reset(&test_timer);
        test_state = TEST_STATE_COUNTDOWN;
      }
      break;

    case TEST_STATE_COUNTDOWN:
      // カウントダウン中のもう一度の押下は中止 (発進前なので制動もブザーも不要)
      if (pressed || fault) {
        Buzzer_Stop(&buzzer);
        test_state = TEST_STATE_IDLE;
        break;
      }
      if (Timer_ReadMs(&test_timer) >= TEST_COUNTDOWN_MS) {
        Steering_SetAngleRad(&steering, 0.0f);  // 直進で走らせる
        Lighting_SetHeadlight(&lighting, LIGHTING_HEADLIGHT_NORMAL);
        Drive_SetTargetSpeed(&drive, TEST_TARGET_SPEED_M_S);
        Timer_Reset(&test_timer);
        test_state = TEST_STATE_ACCEL;
      }
      break;

    case TEST_STATE_ACCEL:
      // 障害物のほか、手動停止 (ボタン)・電源異常・時間切れでも同じ急停止へ落とす
      if (obstacle || pressed || fault || Timer_ReadMs(&test_timer) >= TEST_TIMEOUT_MS) {
        StopTest(obstacle);
      }
      break;

    case TEST_STATE_BRAKE:
      // 目標車速0に対する車速PIが負トルク (回生制動) を出し、車速がほぼ0になった時点で
      // Drive 側が自動的に制動モード (停車保持) へ移る。そこまで見届けてから待機に戻す
      if (Abs(Drive_GetVehicleSpeed(&drive)) < DRIVE_STANDSTILL_SPEED_M_S) {
        Lighting_SetBrake(&lighting, false);
        Lighting_SetHeadlight(&lighting, LIGHTING_HEADLIGHT_DAYTIME);
        test_state = TEST_STATE_IDLE;
      }
      break;
  }

  DigitalOut_Write(&led1, test_state != TEST_STATE_IDLE);
}

void Setup() {
  printf("Setup started\n");
  DigitalOut_Init(&led1, LED1_GPIO_Port, LED1_Pin);
  DigitalOut_Init(&led2, LED2_GPIO_Port, LED2_Pin);
  PwmOut_Init(&led3, &htim4, TIM_CHANNEL_1);
  PwmOut_Init(&led4, &htim4, TIM_CHANNEL_2);
  BreathLed_Init(&breath_signal, &led3);
  BreathLed_Init(&breath_drive, &led4);

  Lighting_Init(&lighting, &htim3, TIM_CHANNEL_1, &htim3, TIM_CHANNEL_3,
                &htim3, TIM_CHANNEL_2, &htim3, TIM_CHANNEL_4);

  AdcDma_Init(&adc1, &hadc1, adc1_buffer, POWER_ADC1_NUM_CH);
  AdcDma_Init(&adc2, &hadc2, adc2_buffer, ENCODER_ADC2_NUM_CH);
  Power_Init(&power, &adc1,
             DRIVE_POWER_GPIO_Port, DRIVE_POWER_Pin,
             LIDAR_POWER_GPIO_Port, LIDAR_POWER_Pin);
  Encoder_Init(&encoder, &adc2);

  Ultrasonic_Init(&ultrasonic_front, TRIG_FRONT_GPIO_Port, TRIG_FRONT_Pin, ECHO_FRONT_GPIO_Port, ECHO_FRONT_Pin);
  Ultrasonic_Init(&ultrasonic_rear, TRIG_REAR_GPIO_Port, TRIG_REAR_Pin, ECHO_REAR_GPIO_Port, ECHO_REAR_Pin);

  DigitalIn_Init(&button1, BUTTON1_GPIO_Port, BUTTON1_Pin);
  DigitalIn_Init(&button2, BUTTON2_GPIO_Port, BUTTON2_Pin);
  // 起動後のキャリブレーション要求はここでラッチする。以降の初期化に時間がかかるため、
  // 押しっぱなしを要求すると使いづらい
  bool calibrate_steering = DigitalIn_Read(&button1);
  bool calibrate_imu = DigitalIn_Read(&button2);
  // 較正モードで起動したことを即座に返す。特にIMUの静止較正は数秒かかるため、表示がないと
  // ボタンを認識したのか判断できない。各較正が終わった時点で消灯する
  DigitalOut_Write(&led1, calibrate_steering);
  DigitalOut_Write(&led2, calibrate_imu);

  // ステアリングモータ(USART2)・左後輪モータ(USART3)・右後輪モータ(UART4) のBLDC MDと通信する
  Serial_Init(&steering_serial, &huart2, 64);
  Serial_Init(&rear_left_serial, &huart3, 64);
  Serial_Init(&rear_right_serial, &huart4, 64);
  Motors_Init(&motors, &steering_serial, &rear_left_serial, &rear_right_serial);

  // ボタン2を押しながら起動 → 静止キャリブレーションをやり直して Flash に保存する (数秒かかる)
  // モーターに給電する前に済ませることで、車体が動かない状態で確実に測れる
  Imu_Init(&imu, &hi2c1, calibrate_imu);
  DigitalOut_Write(&led2, 0);

  // TIM2 CH3: BUZZER, APB1 タイマクロック 90MHz, Prescaler=0 (tim.c の MX_TIM2_Init と一致させる)
  Buzzer_Init(&buzzer, &htim2, TIM_CHANNEL_3, 90000000U, 0U);

  Buzzer_PlayStartupMelody(&buzzer);

  // 起動を知らせるハザード2回点滅
  Lighting_SetHeadlight(&lighting, LIGHTING_HEADLIGHT_DAYTIME);
  Lighting_SetWinker(&lighting, LIGHTING_WINKER_HAZARD);
  Timer startup_hazard_timer;
  Timer_Init(&startup_hazard_timer);
  while (Timer_ReadMs(&startup_hazard_timer) < Lighting_GetWinkerPeriodMs() * 2) {
    Lighting_Update(&lighting);
  }
  Lighting_SetWinker(&lighting, LIGHTING_WINKER_OFF);

  // 起動シーケンスがすべて終わってからモータードライバに給電する。給電中に初期化や
  // 起動演出をしていると、指令を出せる状態になる前にMDが動き出す危険がある
  Power_SetDrivePower(&power, 1);

  // ボタン1を押しながら起動 → 現在のステアリング角度を直進中心点として記録・保存する。
  // MDからの状態フレーム受信が要るため駆動電源投入後に行う
  Steering_Init(&steering, &motors.steering, calibrate_steering);
  DigitalOut_Write(&led1, 0);

  Drive_Init(&drive, &motors, &encoder, &steering, &imu);

  Timer_Init(&test_timer);
  Timer_Init(&button1_debounce_timer);

  Timer_Init(&control_interval_timer);
  printf("Setup finished\n");
}

// ECHOピンの変化割り込み (stm32f4xx_it.c の EXTI9_5_IRQHandler/EXTI15_10_IRQHandler 経由) から呼ばれる
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == ECHO_FRONT_Pin) {
    Ultrasonic_OnEchoEdge(&ultrasonic_front);
  } else if (GPIO_Pin == ECHO_REAR_Pin) {
    Ultrasonic_OnEchoEdge(&ultrasonic_rear);
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

void MainApp() {
  // 目標車速・舵角を与える上位ロジック (Raspberry Pi 通信) は未実装。
  // 目標車速は UpdateDriveTest() が与え、待機中は0のままなので Drive は停車保持
  // (制動モード) で後輪を押さえる
  Drive_Enable(&drive);

  while (1) {
    Power_Update(&power);
    UpdateFaultIndication();
    UpdatePowerIndication();
    Lighting_Update(&lighting);
    Buzzer_Update(&buzzer);
    Encoder_Update(&encoder);
    Imu_Update(&imu);
    Ultrasonic_Update(&ultrasonic_front);
    Ultrasonic_Update(&ultrasonic_rear);
    UpdateDriveTest();
    Drive_Update(&drive);
    Motors_Update(&motors);

    // LED2 の点灯幅がループ1周の処理時間になる (オシロで余裕を見るため)
    DigitalOut_Write(&led2, 1);
    while (Timer_ReadUs(&control_interval_timer) < CONTROL_INTERVAL_US);
    DigitalOut_Write(&led2, 0);
    Timer_Reset(&control_interval_timer);
  }
}
