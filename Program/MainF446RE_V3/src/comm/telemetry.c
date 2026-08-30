#include "telemetry.h"

#include "mymath.h"

static BldcMotor* MotorByIndex(Telemetry* obj, int index) {
  switch (index) {
    case 0:
      return &obj->motors->rear_left;
    case 1:
      return &obj->motors->rear_right;
    default:
      return &obj->motors->steering;
  }
}

static void UpdateMdCommWatch(Telemetry* obj) {
  for (int i = 0; i < 3; i++) {
    uint32_t count = BldcMotor_GetRxCount(MotorByIndex(obj, i));
    if (count != obj->md_comm_watch[i].last_rx_count) {
      obj->md_comm_watch[i].last_rx_count = count;
      Timer_Reset(&obj->md_comm_watch[i].timer);
      obj->md_comm_watch[i].ok = true;
    } else if (Timer_ReadMs(&obj->md_comm_watch[i].timer) > TELEMETRY_MD_COMM_TIMEOUT_MS) {
      obj->md_comm_watch[i].ok = false;
    }
  }
}

static uint8_t BuildMdStatus(Telemetry* obj, int index) {
  const BldcMotor* motor = MotorByIndex(obj, index);
  uint8_t status = 0;
  if (BldcMotor_IsRunning(motor)) status |= RAS_MD_STATUS_RUNNING;
  if (BldcMotor_IsVoltageOutOfRange(motor)) status |= RAS_MD_STATUS_VOLTAGE_OUT_OF_RANGE;
  if (BldcMotor_IsOverheat(motor)) status |= RAS_MD_STATUS_OVERHEAT;
  if (BldcMotor_IsOvercurrent(motor)) status |= RAS_MD_STATUS_OVERCURRENT;
  if (obj->md_comm_watch[index].ok) status |= RAS_MD_STATUS_COMM_OK;
  if (BldcMotor_IsLimitSynced(motor)) status |= RAS_MD_STATUS_LIMIT_SYNCED;
  return status;
}

// フォールトの新規発生を RasLink_Log で1回だけ上位へ通知する。TELEMETRY.flags のビットだけでは
// 「いつ発生したか」が上位のポーリング頻度に依存してしまうため、エッジで明示的に知らせる
static void LogNewFaults(Telemetry* obj, uint32_t faults) {
  uint32_t new_faults = faults & ~obj->logged_faults;
  if (new_faults & POWER_FAULT_DRIVE_OVERCURRENT) {
    RasLink_Log(obj->ras_link, RAS_LOG_ERROR, "drive overcurrent fault (DRIVE_POWER latched off)");
  }
  if (new_faults & POWER_FAULT_SIGNAL_OVERCURRENT) {
    RasLink_Log(obj->ras_link, RAS_LOG_ERROR, "signal overcurrent fault (power latched off)");
  }
  if (new_faults & POWER_FAULT_DRIVE_UNDERVOLTAGE) {
    RasLink_Log(obj->ras_link, RAS_LOG_WARN, "drive battery undervoltage");
  }
  if (new_faults & POWER_FAULT_SIGNAL_UNDERVOLTAGE) {
    RasLink_Log(obj->ras_link, RAS_LOG_WARN, "signal battery undervoltage");
  }
  obj->logged_faults = faults;
}

static uint32_t BuildFlags(Telemetry* obj) {
  uint32_t faults = Power_GetFaults(obj->power);
  uint32_t flags = (uint32_t)Vehicle_GetMode(obj->vehicle) & RAS_FLAG_MODE_MASK;

  if (Power_IsDriveOn(obj->power)) flags |= RAS_FLAG_ARMED;
  if (Vehicle_IsEstopLatched(obj->vehicle)) flags |= RAS_FLAG_ESTOP_ACTIVE;
  if (RasLink_HasCommand(obj->ras_link) && !RasLink_IsCommandAlive(obj->ras_link)) {
    flags |= RAS_FLAG_UART_TIMEOUT;
  }
  if (Drive_IsTractionControlActive(obj->drive)) flags |= RAS_FLAG_TC_ACTIVE;
  if (Drive_IsTorqueVectoringActive(obj->drive)) flags |= RAS_FLAG_TV_ACTIVE;
  if (Imu_IsReady(obj->imu)) flags |= RAS_FLAG_IMU_OK;
  if (Lidar_IsOk(obj->lidar)) flags |= RAS_FLAG_LIDAR_OK;
  if (Steering_IsCenterValid(obj->steering)) flags |= RAS_FLAG_STEER_CENTER_VALID;
  if (Vehicle_IsAutoStopActive(obj->vehicle)) flags |= RAS_FLAG_AUTO_STOP_ACTIVE;
  if (Drive_IsSideBrakeEngaged(obj->drive)) flags |= RAS_FLAG_SIDE_BRAKE_ACTIVE;
  if (Vehicle_IsWinkerLeftActive(obj->vehicle)) flags |= RAS_FLAG_WINKER_LEFT_ACTIVE;
  if (Vehicle_IsWinkerRightActive(obj->vehicle)) flags |= RAS_FLAG_WINKER_RIGHT_ACTIVE;

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

// 前輪の累積回転角 [1e-3 rad] を走行距離 [0.1mm] へ換算する。
// 上位は舵角で射影してから使うこと (操舵輪なので車輪の軌跡長 = 車体中心線距離ではない)
static int32_t AccumAngleToOdom(int32_t mrad, float direction) {
  return (int32_t)((float)mrad * direction * DRIVE_FRONT_WHEEL_RADIUS_M * 10.0f);
}

void Telemetry_Init(Telemetry* obj, RasLink* ras_link, const Vehicle* vehicle, Power* power,
                    Encoder* encoder, Imu* imu, Lidar* lidar, Motors* motors, Steering* steering,
                    Drive* drive, RangeSensor* range_sensor) {
  obj->ras_link = ras_link;
  obj->vehicle = vehicle;
  obj->power = power;
  obj->encoder = encoder;
  obj->imu = imu;
  obj->lidar = lidar;
  obj->motors = motors;
  obj->steering = steering;
  obj->drive = drive;
  obj->range_sensor = range_sensor;

  for (int i = 0; i < 3; i++) {
    obj->md_comm_watch[i].last_rx_count = 0;
    obj->md_comm_watch[i].ok = false;
    Timer_Init(&obj->md_comm_watch[i].timer);
  }
  obj->logged_faults = POWER_FAULT_NONE;
  Timer_Init(&obj->update_timer);
}

void Telemetry_Update(Telemetry* obj) {
  // RasLink 側の実送信自体が50Hzに間引かれるので、ここでの組み立ても同じ周期に間引く
  // (ADC読み・MD状態集計・BuildFlags()の全モジュール問い合わせを2kHzで回す必要はない)
  if (Timer_ReadUs(&obj->update_timer) < RAS_TELEMETRY_INTERVAL_US) return;
  Timer_Reset(&obj->update_timer);

  UpdateMdCommWatch(obj);
  LogNewFaults(obj, Power_GetFaults(obj->power));

  const ImuData* imu_data = Imu_GetData(obj->imu);
  RasTelemetry telemetry;

  telemetry.flags = BuildFlags(obj);
  telemetry.speed_m_s = Drive_GetVehicleSpeed(obj->drive);  // Drive 側で車体中心線方向へ射影済み
  telemetry.yaw_rate_rad_s = Radians(imu_data->gyro_z);
  telemetry.steer_actual_rad = Steering_GetRoadWheelAngleRad(obj->steering);
  telemetry.steer_cmd_rad = Vehicle_GetAppliedSteerRad(obj->vehicle);

  // 4輪ともフィルタ後の値で揃える。以前は前輪だけ生の角速度を渡しており、静止中でも
  // ±12m/s 振れて上位が使えなかった (12bit ADC の 1LSB を 500us で微分すると 0.09m/s に
  // 化けるため。生値をそのまま出すと配列の中で前輪だけ意味が違うことになる)
  telemetry.wheel_speed_m_s[0] = obj->drive->front_speed_left_m_s;
  telemetry.wheel_speed_m_s[1] = obj->drive->front_speed_right_m_s;
  telemetry.wheel_speed_m_s[2] = obj->drive->rear_speed_left_m_s;
  telemetry.wheel_speed_m_s[3] = obj->drive->rear_speed_right_m_s;

  telemetry.odom_dist_0p1mm[0] =
      AccumAngleToOdom(Encoder_GetAccumAngleLeftMrad(obj->encoder), DRIVE_FRONT_LEFT_DIR);
  telemetry.odom_dist_0p1mm[1] =
      AccumAngleToOdom(Encoder_GetAccumAngleRightMrad(obj->encoder), DRIVE_FRONT_RIGHT_DIR);

  telemetry.accel_m_s2[0] = imu_data->accel_x;
  telemetry.accel_m_s2[1] = imu_data->accel_y;
  telemetry.accel_m_s2[2] = imu_data->accel_z;
  telemetry.pitch_rad = Radians(imu_data->pitch);
  telemetry.roll_rad = Radians(imu_data->roll);

  for (int i = 0; i < 3; i++) {
    telemetry.motor_current_a[i] = BldcMotor_GetIq(MotorByIndex(obj, i));
    telemetry.temp_c[i] = BldcMotor_GetTemperatureC(MotorByIndex(obj, i));
    telemetry.md_status[i] = BuildMdStatus(obj, i);
  }
  // 左後輪MDの電流センサ極性が他2台と逆向きのため、報告直前に符号反転して揃える
  telemetry.motor_current_a[0] = -telemetry.motor_current_a[0];
  telemetry.temp_c[3] = (uint8_t)Constrain(Power_GetTemperatureC(obj->power), 0.0f, 255.0f);

  telemetry.torque_cmd_nm[0] = Drive_GetTorqueLeft(obj->drive);
  telemetry.torque_cmd_nm[1] = Drive_GetTorqueRight(obj->drive);

  telemetry.slip[0] = Drive_GetSlipLeft(obj->drive);
  telemetry.slip[1] = Drive_GetSlipRight(obj->drive);
  telemetry.tc_limit_nm[0] = Drive_GetTcLimitLeft(obj->drive);
  telemetry.tc_limit_nm[1] = Drive_GetTcLimitRight(obj->drive);

  telemetry.batt_voltage_v[0] = Power_GetVoltageDriveFiltered(obj->power);
  telemetry.batt_voltage_v[1] = Power_GetVoltageSignalFiltered(obj->power);
  telemetry.batt_current_a[0] = Power_GetCurrentDriveFiltered(obj->power);
  telemetry.batt_current_a[1] = Power_GetCurrentSignalFiltered(obj->power);

  // LPF済みの距離を送る (src/sensing/range_sensor.c)。ULTRASONIC_NO_ECHO (負値) は
  // フィルタを介さずそのまま渡り、RasLink 側で無効値 0 に落ちる
  telemetry.us_distance_m[0] = RangeSensor_GetFrontDistanceFilteredCm(obj->range_sensor) / 100.0f;
  telemetry.us_distance_m[1] = RangeSensor_GetRearDistanceFilteredCm(obj->range_sensor) / 100.0f;

  RasLink_SetTelemetry(obj->ras_link, &telemetry);

  uint32_t md_rx_count[3];
  uint32_t md_rx_error[3];
  for (int i = 0; i < 3; i++) {
    md_rx_count[i] = BldcMotor_GetRxCount(MotorByIndex(obj, i));
    md_rx_error[i] = BldcMotor_GetRxErrorCount(MotorByIndex(obj, i));
  }
  RasLink_SetMdStats(obj->ras_link, md_rx_count, md_rx_error);
}

void Telemetry_PublishLidarSector(Telemetry* obj) {
  const LidarSector* sector = Lidar_TakeReadySector(obj->lidar);
  if (sector == NULL) return;
  RasLink_PublishLidarSector(obj->ras_link, sector->sector_idx, sector->t_start_us,
                             sector->duration_us, sector->rot_speed_dps,
                             sector->distance_mm, sector->intensity);
}
