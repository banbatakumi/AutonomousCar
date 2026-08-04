#include "app.h"

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

void Setup() {
  printf("Setup started\n");
  DigitalOut_Init(&led1, LED1_GPIO_Port, LED1_Pin);
  DigitalOut_Init(&led2, LED2_GPIO_Port, LED2_Pin);
  PwmOut_Init(&led3, &htim4, TIM_CHANNEL_1);
  PwmOut_Init(&led4, &htim4, TIM_CHANNEL_2);

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

// 何らかの異常が出ていればハザードを点滅させ、車外から異常だと分かるようにする。
// 復帰しうる異常 (電圧低下) もあるため、消えたらハザードも消す。
// 電源系以外のモジュールのエラーも、実装したらここに集約する。
// ハザードはウィンカーと灯火を共有するので、方向指示を実装したらここで優先度を調停すること。
static void UpdateFaultIndication() {
  bool fault = Power_GetFaults(&power) != POWER_FAULT_NONE;
  Lighting_SetWinker(&lighting, fault ? LIGHTING_WINKER_HAZARD : LIGHTING_WINKER_OFF);
}

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

void MainApp() {
  BreathLed_Init(&breath_signal, &led3);
  BreathLed_Init(&breath_drive, &led4);

  Drive_Enable(&drive);  // 現状は無効のまま。Drive_Update() は観測量だけ更新する
  while (1) {
    Power_Update(&power);
    UpdateFaultIndication();
    UpdatePowerIndication();
    Lighting_Update(&lighting);
    Encoder_Update(&encoder);
    Imu_Update(&imu);
    Ultrasonic_Update(&ultrasonic_front);
    Ultrasonic_Update(&ultrasonic_rear);
    // Drive_Enable(&drive);                // 現状は無効のまま。Drive_Update() は観測量だけ更新する
    // Drive_SetTargetSpeed(&drive, 0.2f);  // 現状
    Steering_SetAngleRad(&steering, 0);  // 現状

    // Drive_SetTargetSpeed() / Drive_Enable() を呼ぶ上位ロジック (Raspberry Pi 通信) は未実装のため、
    // 現状は無効のまま = トルク指令を出さない。Drive_Update はセンサ由来の観測量だけ更新する
    Drive_Update(&drive);
    Motors_Update(&motors);

    uint32_t interval_us = 500;
    DigitalOut_Write(&led2, 1);
    while (Timer_ReadUs(&control_interval_timer) < interval_us);
    DigitalOut_Write(&led2, 0);
    Timer_Reset(&control_interval_timer);
  }
}
