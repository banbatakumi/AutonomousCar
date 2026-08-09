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

// Raspberry Pi (上位) との通信。USART1、250000bps。
// 受信リングバッファは 25kB/s に対して 64バイトでは 2.5ms 分しかなく、制御ループが
// 一度でも詰まると取りこぼすため大きめに取る
#define RAS_SERIAL_RX_BUF_SIZE 512
Serial ras_serial;
RasLink ras_link;

// LD06 LiDAR (USART6, 230400bps)。約17.6kB/s 流れ込むため、制御周期 500us の間に
// 溜まる分 (約9バイト) に対して十分な余裕を取る
#define LIDAR_SERIAL_RX_BUF_SIZE 256
Serial lidar_serial;
Lidar lidar;

// Raspberry Pi の生存監視 (RAS_SIG = PB12 の 100Hz 矩形波)
Heartbeat heartbeat;

// 独立ウォッチドッグのタイムアウト [ms]。メインループは 500us 周期なので3桁の余裕がある。
// LSI のばらつき (17〜47kHz) で実際は 340ms〜940ms に振れる
#define WATCHDOG_TIMEOUT_MS 500

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
static bool estop_latched;

static void UpdateFaultIndication() {
  bool fault = Power_GetFaults(&power) != POWER_FAULT_NONE || estop_latched;
  Lighting_SetWinker(&lighting, fault ? LIGHTING_WINKER_HAZARD : LIGHTING_WINKER_OFF);
}

// ---------------------------------------------------------------------------
// Raspberry Pi (上位) との連携
// ---------------------------------------------------------------------------

// MDからの状態フレームがこの時間更新されなければ通信断とみなす [ms]。
// MDが無言になっても保持している値は最後の正常値のまま固まるため、これが無いと
// 上位は「古い正常値」を現在値だと信じ続けることになる
#define MD_COMM_TIMEOUT_MS 100

typedef struct {
  uint32_t last_rx_count;
  Timer timer;
  bool ok;
} MdCommWatch;

// [左後輪, 右後輪, ステアリング] の順 (プロトコルの配列インデックス規約に合わせる)
static MdCommWatch md_comm_watch[3];

// 上位から指令された目標値。accel_limit / steer_rate_limit でレート制限したあとの値で、
// 実際に Drive / Steering へ渡している量。テレメトリの steer_cmd_echo もこれを返す
static float applied_speed_m_s;
static float applied_steer_rad;
static Timer command_rate_timer;
static uint8_t vehicle_mode = RAS_MODE_DISARM;

// クラクションの音程 [Hz]。押している間だけ鳴らすため、単発ビープではなく連続トーンで出す
#define HORN_FREQ_HZ 2500
static bool horn_was_on;

static void SetHorn(bool on) {
  if (on == horn_was_on) return;
  Buzzer_SetTone(&buzzer, on ? HORN_FREQ_HZ : 0);
  horn_was_on = on;
}

static LightingHeadlightMode HeadlightModeFromCommand(uint8_t light_mode) {
  switch (light_mode) {
    case RAS_LIGHT_NORMAL:
      return LIGHTING_HEADLIGHT_NORMAL;
    case RAS_LIGHT_DAYTIME:
      return LIGHTING_HEADLIGHT_DAYTIME;
    default:
      return LIGHTING_HEADLIGHT_OFF;
  }
}

static BldcMotor* MotorByIndex(int index) {
  switch (index) {
    case 0:
      return &motors.rear_left;
    case 1:
      return &motors.rear_right;
    default:
      return &motors.steering;
  }
}

static void UpdateMdCommWatch() {
  for (int i = 0; i < 3; i++) {
    uint32_t count = BldcMotor_GetRxCount(MotorByIndex(i));
    if (count != md_comm_watch[i].last_rx_count) {
      md_comm_watch[i].last_rx_count = count;
      Timer_Reset(&md_comm_watch[i].timer);
      md_comm_watch[i].ok = true;
    } else if (Timer_ReadMs(&md_comm_watch[i].timer) > MD_COMM_TIMEOUT_MS) {
      md_comm_watch[i].ok = false;
    }
  }
}

static uint8_t BuildMdStatus(int index) {
  const BldcMotor* motor = MotorByIndex(index);
  uint8_t status = 0;
  if (BldcMotor_IsRunning(motor)) status |= RAS_MD_STATUS_RUNNING;
  if (BldcMotor_IsVoltageOutOfRange(motor)) status |= RAS_MD_STATUS_VOLTAGE_OUT_OF_RANGE;
  if (BldcMotor_IsOverheat(motor)) status |= RAS_MD_STATUS_OVERHEAT;
  if (BldcMotor_IsOvercurrent(motor)) status |= RAS_MD_STATUS_OVERCURRENT;
  if (md_comm_watch[index].ok) status |= RAS_MD_STATUS_COMM_OK;
  if (BldcMotor_IsLimitSynced(motor)) status |= RAS_MD_STATUS_LIMIT_SYNCED;
  return status;
}

static uint32_t BuildTelemetryFlags() {
  uint32_t faults = Power_GetFaults(&power);
  uint32_t flags = (uint32_t)vehicle_mode & RAS_FLAG_MODE_MASK;

  if (Power_IsDriveOn(&power)) flags |= RAS_FLAG_ARMED;
  if (estop_latched) flags |= RAS_FLAG_ESTOP_ACTIVE;
  if (RasLink_HasCommand(&ras_link) && !RasLink_IsCommandAlive(&ras_link)) flags |= RAS_FLAG_UART_TIMEOUT;
  if (Drive_IsTractionControlActive(&drive)) flags |= RAS_FLAG_TC_ACTIVE;
  if (Imu_IsReady(&imu)) flags |= RAS_FLAG_IMU_OK;
  if (Lidar_IsOk(&lidar)) flags |= RAS_FLAG_LIDAR_OK;
  if (Steering_IsCenterValid(&steering)) flags |= RAS_FLAG_STEER_CENTER_VALID;

  if (faults & POWER_FAULT_DRIVE_OVERCURRENT) flags |= RAS_FLAG_FAULT_DRIVE_OVERCURRENT;
  if (faults & POWER_FAULT_SIGNAL_OVERCURRENT) flags |= RAS_FLAG_FAULT_SIGNAL_OVERCURRENT;
  if (faults & POWER_FAULT_DRIVE_UNDERVOLTAGE) flags |= RAS_FLAG_FAULT_DRIVE_UNDERVOLTAGE;
  if (faults & POWER_FAULT_SIGNAL_UNDERVOLTAGE) flags |= RAS_FLAG_FAULT_SIGNAL_UNDERVOLTAGE;
  // 過電流はラッチするため、以降は arm を要求されても駆動電源が入らない。
  // 上位が「arm したのに armed が立たない」を異常と誤認しないよう明示する
  if (faults & (POWER_FAULT_DRIVE_OVERCURRENT | POWER_FAULT_SIGNAL_OVERCURRENT)) {
    flags |= RAS_FLAG_DRIVE_POWER_LOCKED;
  }
  return flags;
}

// 組み上がった LiDAR のセクタを送信キューへ積む。1周を12分割して送るため 120Hz で呼ばれる
static void PublishLidarSector() {
  const LidarSector* sector = Lidar_TakeReadySector(&lidar);
  if (sector == NULL) return;
  RasLink_PublishLidarSector(&ras_link, sector->sector_idx, sector->t_start_us,
                             sector->duration_us, sector->rot_speed_dps,
                             sector->distance_mm, sector->intensity);
}

// 前輪の累積回転角 [1e-3 rad] を走行距離 [0.1mm] へ換算する。
// 上位は舵角で射影してから使うこと (操舵輪なので車輪の軌跡長 = 車体中心線距離ではない)
static int32_t AccumAngleToOdom(int32_t mrad, float direction) {
  return (int32_t)((float)mrad * direction * DRIVE_FRONT_WHEEL_RADIUS_M * 10.0f);
}

static void PublishTelemetry() {
  const ImuData* imu_data = Imu_GetData(&imu);
  RasTelemetry telemetry;

  telemetry.flags = BuildTelemetryFlags();
  telemetry.speed_m_s = Drive_GetVehicleSpeed(&drive);  // Drive 側で車体中心線方向へ射影済み
  telemetry.yaw_rate_rad_s = Radians(imu_data->gyro_z);
  telemetry.steer_actual_rad = Steering_GetRoadWheelAngleRad(&steering);
  telemetry.steer_cmd_rad = applied_steer_rad;

  // 4輪ともフィルタ後の値で揃える。以前は前輪だけ生の角速度を渡しており、静止中でも
  // ±12m/s 振れて上位が使えなかった (12bit ADC の 1LSB を 500us で微分すると 0.09m/s に
  // 化けるため。生値をそのまま出すと配列の中で前輪だけ意味が違うことになる)
  telemetry.wheel_speed_m_s[0] = drive.front_speed_left_m_s;
  telemetry.wheel_speed_m_s[1] = drive.front_speed_right_m_s;
  telemetry.wheel_speed_m_s[2] = drive.rear_speed_left_m_s;
  telemetry.wheel_speed_m_s[3] = drive.rear_speed_right_m_s;

  telemetry.odom_dist_0p1mm[0] =
      AccumAngleToOdom(Encoder_GetAccumAngleLeftMrad(&encoder), DRIVE_FRONT_LEFT_DIR);
  telemetry.odom_dist_0p1mm[1] =
      AccumAngleToOdom(Encoder_GetAccumAngleRightMrad(&encoder), DRIVE_FRONT_RIGHT_DIR);

  telemetry.accel_m_s2[0] = imu_data->accel_x;
  telemetry.accel_m_s2[1] = imu_data->accel_y;
  telemetry.accel_m_s2[2] = imu_data->accel_z;
  telemetry.pitch_rad = Radians(imu_data->pitch);
  telemetry.roll_rad = Radians(imu_data->roll);

  for (int i = 0; i < 3; i++) {
    telemetry.motor_current_a[i] = BldcMotor_GetIq(MotorByIndex(i));
    telemetry.temp_c[i] = BldcMotor_GetTemperatureC(MotorByIndex(i));
    telemetry.md_status[i] = BuildMdStatus(i);
  }
  telemetry.temp_c[3] = (uint8_t)Constrain(Power_GetTemperatureC(&power), 0.0f, 255.0f);

  telemetry.torque_cmd_nm[0] = Drive_GetTorqueLeft(&drive);
  telemetry.torque_cmd_nm[1] = Drive_GetTorqueRight(&drive);

  telemetry.batt_voltage_v[0] = Power_GetVoltageDriveFiltered(&power);
  telemetry.batt_voltage_v[1] = Power_GetVoltageSignalFiltered(&power);
  telemetry.batt_current_a[0] = Power_GetCurrentDrive(&power);
  telemetry.batt_current_a[1] = Power_GetCurrentSignal(&power);

  // ULTRASONIC_NO_ECHO (負値) はそのまま渡す。RasLink 側で無効値 0 に落ちる
  telemetry.us_distance_m[0] = Ultrasonic_GetDistanceCm(&ultrasonic_front) / 100.0f;
  telemetry.us_distance_m[1] = Ultrasonic_GetDistanceCm(&ultrasonic_rear) / 100.0f;

  RasLink_SetTelemetry(&ras_link, &telemetry);

  uint32_t md_rx_count[3];
  uint32_t md_rx_error[3];
  for (int i = 0; i < 3; i++) {
    md_rx_count[i] = BldcMotor_GetRxCount(MotorByIndex(i));
    md_rx_error[i] = BldcMotor_GetRxErrorCount(MotorByIndex(i));
  }
  RasLink_SetMdStats(&ras_link, md_rx_count, md_rx_error);
}

// 上位の指令を車両へ適用する。目標値そのものではなく、加速度・舵角速度の上限で
// レート制限した値を渡す (急な指令変化でタイヤを滑らせたり据え切りでラックを痛めないため)
static void ApplyRasCommand() {
  const RasCommand* command = RasLink_GetCommand(&ras_link);
  const RasConfig* config = RasLink_GetConfig(&ras_link);
  float dt_s = Timer_Read(&command_rate_timer);
  Timer_Reset(&command_rate_timer);

  // mode = 3 は v0.4 で予約になったため、受信しても現在のモードを維持する
  if (command->mode != RAS_MODE_RESERVED) vehicle_mode = command->mode;

  bool arm_requested = (command->flags & RAS_CMD_FLAG_ARM) != 0;
  Power_SetDrivePower(&power, arm_requested);

  // 中心点が未較正だと舵角の絶対値が信用できないため走行させない
  bool armed = arm_requested && Steering_IsCenterValid(&steering) &&
               (vehicle_mode == RAS_MODE_MANUAL || vehicle_mode == RAS_MODE_AUTO);
  bool braking = (command->flags & RAS_CMD_FLAG_BRAKE) != 0;

  float target_speed_m_s = 0.0f;
  if (armed && !braking) {
    target_speed_m_s = Constrain(command->target_speed_m_s, -config->max_speed_m_s, config->max_speed_m_s);
  }
  // 制動トルクの指定が無い (0) ときは最大で掛ける。0 をそのまま「制動トルク0」と解釈すると、
  // 上位がフィールドを埋め忘れただけでブレーキが効かなくなる
  float brake_torque_nm =
      command->brake_torque_nm > 0.0f ? command->brake_torque_nm : DRIVE_MAX_BRAKE_TORQUE_NM;

  float target_steer_rad =
      Constrain(command->target_steer_rad, -config->max_steer_rad, config->max_steer_rad);

  // 上限0は「制限なし」ではなく「動かない」になってしまうため、0 のときは設定値で代替する
  float accel_limit = command->accel_limit_m_s2 > 0.0f ? command->accel_limit_m_s2 : config->max_accel_m_s2;
  accel_limit = Constrain(accel_limit, 0.01f, config->max_accel_m_s2);
  float steer_rate_limit = command->steer_rate_limit_rad_s > 0.0f
                               ? command->steer_rate_limit_rad_s
                               : Steering_GetMaxRoadWheelAngleRad();

  // ブレーキ中は Drive 側が車速制御ごと迂回するので目標車速をレート制限で下げる意味が無い。
  // ここで0に落としておかないと、ブレーキを離した瞬間に減速前の目標車速へ復帰してしまう
  if (braking) applied_speed_m_s = 0.0f;

  float speed_step = accel_limit * dt_s;
  applied_speed_m_s += Constrain(target_speed_m_s - applied_speed_m_s, -speed_step, speed_step);
  float steer_step = steer_rate_limit * dt_s;
  applied_steer_rad += Constrain(target_steer_rad - applied_steer_rad, -steer_step, steer_step);

  // Drive_Enable は積分項とTCの上限をリセットするため、状態が変わったときだけ呼ぶ
  if (armed && !Drive_IsEnabled(&drive)) Drive_Enable(&drive);
  if (!armed && Drive_IsEnabled(&drive)) Drive_Disable(&drive);

  Drive_SetTargetSpeed(&drive, applied_speed_m_s);
  Drive_SetBrake(&drive, braking, brake_torque_nm);
  Steering_SetRoadWheelAngleRad(&steering, applied_steer_rad);

  Lighting_SetBrake(&lighting, braking);
  Lighting_SetHeadlight(&lighting, HeadlightModeFromCommand(command->light_mode));
  Lighting_SetPassing(&lighting, (command->flags & RAS_CMD_FLAG_PASSING) != 0);

  SetHorn((command->flags & RAS_CMD_FLAG_HORN) != 0);
}

// ---------------------------------------------------------------------------
// 緊急停止 (第1安全層)
//
// Raspberry Pi が出す 100Hz 矩形波が途切れたら発動する。UART の COMMAND 途絶検出とは
// 別経路であることに意味があり、こちらの方が速い (50ms 対 100ms)。
//
// **発動しても駆動電源は切らない。** 電源を切るとMDが制動をかけられなくなり惰行に入るため、
// 止まるまでの距離がかえって伸びる。電源を落とすのは過電流のように「流し続けること自体が
// 危険」なときの処置で、緊急停止でやるべきなのは最短で止めることの方。
//
// 一度発動したらラッチし、人間が明示的に解除するまで復帰しない (原因が解消しないまま
// 走り出さないため)。解除はハートビートが戻っている状態でボタン2を押す。
// ---------------------------------------------------------------------------

static void UpdateEstop() {
  Heartbeat_Update(&heartbeat);

  // ハートビートが配線されていない (単体でのベンチ確認中) 場合まで停止させない
  if (Heartbeat_HasEverBeenSeen(&heartbeat) && !Heartbeat_IsAlive(&heartbeat)) {
    estop_latched = true;
  }

  if (estop_latched && Heartbeat_IsAlive(&heartbeat) && DigitalIn_Read(&button2)) {
    estop_latched = false;
    Buzzer_Beep(&buzzer, 2000, 100);
  }
}

// 上位の指令が使えない状況 (緊急停止・COMMAND 途絶) で共通に取る処置。最大の制動トルクで
// 止め、上位の操作で入りっぱなしになりうる出力 (クラクション・パッシング) は解除する。
// 舵角は最後の指令値のまま保持する (直進へ戻すと車体が予期しない方向へ動くため)
static void ApplyFailsafe() {
  applied_speed_m_s = 0.0f;
  Drive_SetTargetSpeed(&drive, 0.0f);
  Drive_SetBrake(&drive, true, DRIVE_MAX_BRAKE_TORQUE_NM);
  Steering_SetRoadWheelAngleRad(&steering, applied_steer_rad);
  Lighting_SetBrake(&lighting, true);
  Lighting_SetPassing(&lighting, false);
  SetHorn(false);
}

static void UpdateVehicleControl() {
  UpdateEstop();

  // 緊急停止は上位の指令より優先する
  if (estop_latched) {
    ApplyFailsafe();
    return;
  }

  if (RasLink_IsCommandAlive(&ras_link)) {
    ApplyRasCommand();
  } else {
    // COMMAND が一度も届いていない間も含め、上位と繋がっていなければ停車保持。
    // 上位が一度でも繋がった後も、通信が復帰するまでこの状態を続ける
    ApplyFailsafe();
  }
}

void Setup() {
  // 上位との通信が使う単調なマイクロ秒時計。他のどの初期化よりも先に立ち上げる
  Micros_Init();
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

  // ボタン1を押しながら起動 → 現在のステアリング角度を直進中心点として記録・保存する。
  // 較正は MD からの状態フレーム受信が要るため、このときだけ駆動電源を入れて較正後に落とす。
  // 較正しない起動では Flash から読むだけで MD と話す必要がないので、駆動電源には一切
  // 触れない (毎回一瞬でも投入すると、その間だけMDが指令待ちで通電された状態になる)。
  // 上位が arm するまで駆動電源を入れないことで、Pi が未接続/DISARM の間は駆動系が
  // 無力化された状態を既定にする
  if (calibrate_steering) Power_SetDrivePower(&power, 1);
  Steering_Init(&steering, &motors.steering, calibrate_steering);
  if (calibrate_steering) Power_SetDrivePower(&power, 0);
  DigitalOut_Write(&led1, 0);

  Drive_Init(&drive, &motors, &encoder, &steering, &imu);

  // LD06 LiDAR (USART6, 230400bps)。給電してから初期化する。
  // 回転が安定するまで数秒かかるが、その間は Lidar_IsOk() が偽になるだけで待つ必要はない
  Power_SetLidarPower(&power, 1);
  Serial_Init(&lidar_serial, &huart6, LIDAR_SERIAL_RX_BUF_SIZE);
  Lidar_Init(&lidar, &lidar_serial, &htim1, TIM_CHANNEL_1);

  // Raspberry Pi (USART1)。MD3系統の初期化が終わってから立ち上げることで、
  // 最初のテレメトリを送る時点でモータの状態が揃っている
  Serial_Init(&ras_serial, &huart1, RAS_SERIAL_RX_BUF_SIZE);
  RasLink_Init(&ras_link, &ras_serial);
  Heartbeat_Init(&heartbeat, RAS_SIG_GPIO_Port, RAS_SIG_Pin);
  for (int i = 0; i < 3; i++) Timer_Init(&md_comm_watch[i].timer);
  Timer_Init(&command_rate_timer);

  Timer_Init(&control_interval_timer);
  printf("Setup finished\n");

  // ウォッチドッグは一度起動すると止められないため、ブロッキングする初期化 (IMU の静止
  // 較正・起動演出・ステアリング中心点の記録) がすべて終わってから最後に起動する。
  // 裏を返すと Setup 中のハングは検出できない
  Watchdog_Start(WATCHDOG_TIMEOUT_MS);
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
  // 上位 (Raspberry Pi) と繋がっていない/armされていない間は ApplyFailsafe() が
  // ブレーキを掛け続けるので、Drive は制動モードで後輪を押さえる
  Drive_Enable(&drive);

  while (1) {
    // ループが回っていること自体が生存の証拠なので、先頭で無条件に叩く。
    // 個々のモジュールの異常はフォールトとして別に扱う (ウォッチドッグを異常時の
    // 停止手段に流用すると、リセットで状態が消えて原因が追えなくなる)
    Watchdog_Refresh();

    Power_Update(&power);
    UpdateFaultIndication();
    UpdatePowerIndication();
    Lighting_Update(&lighting);
    Buzzer_Update(&buzzer);
    Encoder_Update(&encoder);
    Imu_Update(&imu);
    Ultrasonic_Update(&ultrasonic_front);
    Ultrasonic_Update(&ultrasonic_rear);
    Lidar_Update(&lidar);
    UpdateMdCommWatch();
    UpdateVehicleControl();
    Drive_Update(&drive);
    Motors_Update(&motors);

    // テレメトリは毎周期最新値に差し替え、実際の送信は RasLink 側で 50Hz に間引かれる
    PublishTelemetry();
    PublishLidarSector();
    RasLink_Update(&ras_link);

    // LED2 の点灯幅がループ1周の処理時間になる (オシロで余裕を見るため)
    DigitalOut_Write(&led2, 1);
    while (Timer_ReadUs(&control_interval_timer) < CONTROL_INTERVAL_US);
    DigitalOut_Write(&led2, 0);
    Timer_Reset(&control_interval_timer);
  }
}
