#include "vehicle.h"

#include "mymath.h"

static void SetHorn(Vehicle* obj, bool on) {
  if (on == obj->horn_on) return;
  Buzzer_SetTone(obj->buzzer, on ? VEHICLE_HORN_FREQ_HZ : 0);
  obj->horn_on = on;
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

static void ApplyBrakeLight(Vehicle* obj, bool braking, float brake_torque_nm) {
  Lighting_SetBrakeMode(obj->lighting, braking,
                         brake_torque_nm >= VEHICLE_EMERGENCY_BRAKE_FLASH_THRESHOLD_NM);
}

// LiDAR電源を want_on に応じて更新する。ARM中は即座に点け、ARMが外れても
// VEHICLE_LIDAR_IDLE_OFF_DELAY_S が経つまでは点けたままにする (短い停止での
// 頻繁な電源入り切りと、それによる再ARM直後のセンサ空白を避けるため)
static void UpdateLidarPower(Vehicle* obj, bool want_on) {
  if (want_on) {
    Timer_Reset(&obj->lidar_idle_timer);
    if (!obj->lidar_on) {
      Power_SetLidarPower(obj->power, 1);
      obj->lidar_on = true;
    }
    return;
  }
  if (obj->lidar_on && Timer_Read(&obj->lidar_idle_timer) >= VEHICLE_LIDAR_IDLE_OFF_DELAY_S) {
    Power_SetLidarPower(obj->power, 0);
    obj->lidar_on = false;
  }
}

// 車速から物理的に導かれる制動距離 (反応・処理遅延分 + 制動で止まるまでの距離) に、
// 上位が直接指定した安全マージン (RasConfig.auto_stop_margin_cm) を加えた
// 動的停止距離 [cm] を返す
static float AutoStopDistanceCm(float speed_m_s, const RasConfig* config) {
  float v = (speed_m_s >= 0.0f) ? speed_m_s : -speed_m_s;
  float physical_cm =
      (v * VEHICLE_AUTO_STOP_DELAY_S + (v * v) / (2.0f * DRIVE_MAX_ACCEL_M_S2)) * 100.0f;
  return physical_cm + config->auto_stop_margin_cm;
}

// 実際の進行方向 (前輪エンコーダから推定した実車速) について、動的停止距離以内に障害物が
// あるかを見る。逆方向のセンサは見ないため、例えば前方に障害物があっても後退はできる。
// 上位が指令した target_speed/target_torque の符号ではなく実車速を使うのは、
// 「前進中に後退を指令」した瞬間でも実車はまだ前進しており、その一瞬に後方センサへ
// 切り替わって前方の障害物を見失う窓ができるのを避けるため。
//
// ただし実車速が VEHICLE_AUTO_STOP_DIRECTION_DEADBAND_M_S 未満 (ほぼ静止) のときは
// desired_direction (上位が指令した方向) にフォールバックする。停止中は実車速の符号が
// 定まらず常に非負 (>=0.0) と評価されて前方判定に固定されてしまい、速度制御モードでは
// 自動停止が braking=true にすると target_speed が常に0へクランプされて実車速も
// 永久に0のままになる (前方に障害物・後方は空いていても後退できないデッドロック) ため
//
// センサはいずれも車体先端(バンパー)より内側に付いているため、しきい値
// (d_stop / VEHICLE_AUTO_STOP_ULTRASONIC_NEAR_CM) 側に取付オフセットを足して、
// 「車体先端から障害物まで」を基準に判定する (VEHICLE_AUTO_STOP_*_OFFSET_CM 参照)
static bool IsAutoStopObstacleAhead(Vehicle* obj, const RasConfig* config,
                                    float desired_direction) {
  float speed = Drive_GetVehicleSpeed(obj->drive);
  float abs_speed = (speed >= 0.0f) ? speed : -speed;
  bool forward = (abs_speed > VEHICLE_AUTO_STOP_DIRECTION_DEADBAND_M_S)
                     ? (speed >= 0.0f)
                     : (desired_direction >= 0.0f);
  float d_stop_cm = AutoStopDistanceCm(speed, config);
  float lidar_center_deg = forward ? 0.0f : 180.0f;
  float lidar_offset_cm = forward ? VEHICLE_AUTO_STOP_LIDAR_FRONT_OFFSET_CM
                                   : VEHICLE_AUTO_STOP_LIDAR_REAR_OFFSET_CM;
  float us_offset_cm = forward ? VEHICLE_AUTO_STOP_ULTRASONIC_FRONT_OFFSET_CM
                                : VEHICLE_AUTO_STOP_ULTRASONIC_REAR_OFFSET_CM;

  LidarRoiResult roi = Lidar_QueryRoi(obj->lidar, lidar_center_deg,
                                      VEHICLE_AUTO_STOP_LIDAR_HALF_WIDTH_DEG,
                                      d_stop_cm + lidar_offset_cm);
  bool lidar_fresh = roi.fresh_count > 0;
  bool lidar_hit = roi.hit_count >= VEHICLE_AUTO_STOP_LIDAR_MIN_POINTS;

  float us_dist_cm = forward ? RangeSensor_GetFrontDistanceFilteredCm(obj->range_sensor)
                              : RangeSensor_GetRearDistanceFilteredCm(obj->range_sensor);
  bool us_near_hit = us_dist_cm >= 0.0f &&
                     us_dist_cm < (VEHICLE_AUTO_STOP_ULTRASONIC_NEAR_CM + us_offset_cm);
  bool us_fallback_hit = !lidar_fresh && us_dist_cm >= 0.0f &&
                         us_dist_cm < (d_stop_cm + us_offset_cm);

  return lidar_hit || us_near_hit || us_fallback_hit;
}

// 上位の指令を車両へ適用する。目標値そのものではなく、加速度・舵角速度の上限で
// レート制限した値を渡す (急な指令変化でタイヤを滑らせたり据え切りでラックを痛めないため)
static void ApplyRasCommand(Vehicle* obj) {
  const RasCommand* command = RasLink_GetCommand(obj->ras_link);
  const RasConfig* config = RasLink_GetConfig(obj->ras_link);
  float dt_s = Timer_Read(&obj->command_rate_timer);
  Timer_Reset(&obj->command_rate_timer);
  // command_rate_timer は緊急停止・COMMAND途絶で ApplyFailsafe() に分岐している間も
  // 回り続けている (ApplyFailsafe 側でもリセットするが、念のため二重に防御する)。
  // 初回や異常に長い dt (途絶からの復帰直後など) では加速度・舵角速度の制限が
  // 実質無効化されてしまうため、drive.c / pid.h と同じガードでレート制限を無効にする
  if (dt_s <= 0.0f || dt_s > 0.1f) dt_s = 0.0f;

  // mode = 3 は v0.4 で予約になったため、受信しても現在のモードを維持する
  if (command->mode != RAS_MODE_RESERVED) obj->mode = command->mode;

  bool arm_requested = (command->flags & RAS_CMD_FLAG_ARM) != 0;
  Power_SetDrivePower(obj->power, arm_requested);
  UpdateLidarPower(obj, arm_requested);

  // 中心点が未較正だと舵角の絶対値が信用できないため走行させない
  bool armed = arm_requested && Steering_IsCenterValid(obj->steering) &&
               (obj->mode == RAS_MODE_MANUAL || obj->mode == RAS_MODE_AUTO);
  bool cmd_braking = (command->flags & RAS_CMD_FLAG_BRAKE) != 0;
  bool torque_mode = (command->flags & RAS_CMD_FLAG_TORQUE_MODE) != 0;
  bool auto_stop_enabled = (command->flags & RAS_CMD_FLAG_AUTO_STOP) != 0;
  bool side_brake_requested = (command->flags2 & RAS_CMD_FLAG2_SIDE_BRAKE) != 0;

  // 静止時の前後判定フォールバック用。torque_mode なら target_torque、そうでなければ
  // target_speed の符号を「これから進もうとしている方向」として使う
  float desired_direction = torque_mode ? command->target_torque_nm : command->target_speed_m_s;
  obj->auto_stop_active =
      armed && auto_stop_enabled && IsAutoStopObstacleAhead(obj, config, desired_direction);
  bool braking = cmd_braking || obj->auto_stop_active;

  float target_speed_m_s = 0.0f;
  if (armed && !braking && !torque_mode && !side_brake_requested) {
    target_speed_m_s = command->target_speed_m_s;  // 最終クランプは Drive_SetTargetSpeed が行う
  }
  // 制動トルクの指定が無い (0) ときは最大で掛ける。0 をそのまま「制動トルク0」と解釈すると、
  // 上位がフィールドを埋め忘れただけでブレーキが効かなくなる。自動停止はできるだけ強く
  // 止めることが目的なので、上位が明示的にブレーキを踏んでいるとき以外は常に最大で掛ける
  float brake_torque_nm = (cmd_braking && command->brake_torque_nm > 0.0f)
                               ? command->brake_torque_nm
                               : DRIVE_MAX_BRAKE_TORQUE_NM;

  float target_steer_rad = Constrain(command->target_steer_rad, -Steering_GetMaxRoadWheelAngleRad(),
                                     Steering_GetMaxRoadWheelAngleRad());

  float steer_rate_limit = command->steer_rate_limit_rad_s > 0.0f
                               ? command->steer_rate_limit_rad_s
                               : Steering_GetMaxRoadWheelAngleRad();

  float steer_step = steer_rate_limit * dt_s;
  obj->applied_steer_rad +=
      Constrain(target_steer_rad - obj->applied_steer_rad, -steer_step, steer_step);

  // Drive_Enable は積分項とTCの上限をリセットするため、状態が変わったときだけ呼ぶ
  if (armed && !Drive_IsEnabled(obj->drive)) Drive_Enable(obj->drive);
  if (!armed && Drive_IsEnabled(obj->drive)) Drive_Disable(obj->drive);

  // 加速度レート制限 (DRIVE_MAX_ACCEL_M_S2 が上限) は Drive 側が持つため、生の目標車速を
  // そのまま渡す。上限0は「制限なし」ではなく「動かない」になってしまうため、
  // 0 のときは Drive_SetTargetSpeed 側で DRIVE_MAX_ACCEL_M_S2 に読み替える
  Drive_SetTargetSpeed(obj->drive, target_speed_m_s, command->accel_limit_m_s2);
  Drive_SetBrake(obj->drive, braking, brake_torque_nm);
  Drive_SetTorque(obj->drive, torque_mode, command->target_torque_nm);
  // 未アーム中は無効化する (アームが外れた瞬間に位置保持へ入り込まないようにするため)
  Drive_SetSideBrake(obj->drive, armed && side_brake_requested);
  Drive_SetTractionControlEnabled(obj->drive, config->tc_enabled);
  Drive_SetTorqueVectoringEnabled(obj->drive, config->tv_enabled);
  Drive_SetWheelLiftGuardEnabled(obj->drive, config->wheel_lift_guard_enabled);
  Steering_SetRoadWheelAngleRad(obj->steering, obj->applied_steer_rad);

  ApplyBrakeLight(obj, braking, brake_torque_nm);
  Lighting_SetHeadlight(obj->lighting, HeadlightModeFromCommand(command->light_mode));
  Lighting_SetPassing(obj->lighting, (command->flags & RAS_CMD_FLAG_PASSING) != 0);

  SetHorn(obj, (command->flags & RAS_CMD_FLAG_HORN) != 0);
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

static void UpdateEstop(Vehicle* obj) {
  Heartbeat_Update(obj->heartbeat);

  // ハートビートが配線されていない (単体でのベンチ確認中) 場合まで停止させない
  if (Heartbeat_HasEverBeenSeen(obj->heartbeat) && !Heartbeat_IsAlive(obj->heartbeat)) {
    obj->estop_latched = true;
  }

  if (obj->estop_latched && Heartbeat_IsAlive(obj->heartbeat) &&
      DigitalIn_Read(obj->estop_reset_button)) {
    obj->estop_latched = false;
    Buzzer_Beep(obj->buzzer, 2000, 100);
  }
}

// 上位の指令が使えない状況 (緊急停止・COMMAND 途絶) で共通に取る処置。最大の制動トルクで
// 止め、上位の操作で入りっぱなしになりうる出力 (クラクション・パッシング) は解除する。
// 舵角は最後の指令値のまま保持する (直進へ戻すと車体が予期しない方向へ動くため)
static void ApplyFailsafe(Vehicle* obj) {
  // ApplyRasCommand() 側の command_rate_timer をここでもリセットしておく。フェイルセーフ中は
  // 呼ばれないため、これをしないと復帰した瞬間の dt が途絶していた時間分だけ膨らみ、
  // 加速度・舵角速度のレート制限が1周期だけ無効化されたのと同じ状態になる
  Timer_Reset(&obj->command_rate_timer);

  Drive_SetTargetSpeed(obj->drive, 0.0f, 0.0f);
  Drive_SetBrake(obj->drive, true, DRIVE_MAX_BRAKE_TORQUE_NM);
  // brake が torque_mode より優先されるため torque_mode の解除は現状表面化しないが、
  // side_brake は Drive_Update 内で brake より先に判定されるため、これを明示的に解除
  // しないと緊急停止・COMMAND途絶中に古い side_brake 状態が残っていた場合に位置保持へ
  // 入り込み、最大制動トルクでの停止 (Drive_SetBrake) が効かなくなる。安全層は他モジュールの
  // 内部優先順位に依存せず自己完結させるべきという方針からも、ここで明示的に解除しておく
  Drive_SetTorque(obj->drive, false, 0.0f);
  Drive_SetSideBrake(obj->drive, false);
  Steering_SetRoadWheelAngleRad(obj->steering, obj->applied_steer_rad);
  ApplyBrakeLight(obj, true, DRIVE_MAX_BRAKE_TORQUE_NM);
  Lighting_SetPassing(obj->lighting, false);
  SetHorn(obj, false);
  obj->auto_stop_active = false;
}

void Vehicle_Init(Vehicle* obj, RasLink* ras_link, Drive* drive, Steering* steering,
                  Lighting* lighting, Power* power, Heartbeat* heartbeat, Buzzer* buzzer,
                  DigitalIn* estop_reset_button, RangeSensor* range_sensor, Lidar* lidar) {
  obj->ras_link = ras_link;
  obj->drive = drive;
  obj->steering = steering;
  obj->lighting = lighting;
  obj->power = power;
  obj->heartbeat = heartbeat;
  obj->buzzer = buzzer;
  obj->estop_reset_button = estop_reset_button;
  obj->range_sensor = range_sensor;
  obj->lidar = lidar;

  obj->applied_steer_rad = 0.0f;
  Timer_Init(&obj->command_rate_timer);

  obj->mode = RAS_MODE_DISARM;
  obj->estop_latched = false;
  obj->horn_on = false;
  obj->auto_stop_active = false;

  // Setup() が Vehicle_Init より前に Power_SetLidarPower() でLiDARへ給電済みの状態を反映する
  Timer_Init(&obj->lidar_idle_timer);
  obj->lidar_on = true;
}

void Vehicle_Update(Vehicle* obj) {
  UpdateEstop(obj);

  // 緊急停止は上位の指令より優先する
  if (obj->estop_latched) {
    ApplyFailsafe(obj);
    UpdateLidarPower(obj, false);
    return;
  }

  if (RasLink_IsCommandAlive(obj->ras_link)) {
    ApplyRasCommand(obj);
  } else {
    // COMMAND が一度も届いていない間も含め、上位と繋がっていなければ停車保持。
    // 上位が一度でも繋がった後も、通信が復帰するまでこの状態を続ける
    ApplyFailsafe(obj);
    UpdateLidarPower(obj, false);
  }
}

bool Vehicle_IsEstopLatched(const Vehicle* obj) { return obj->estop_latched; }

uint8_t Vehicle_GetMode(const Vehicle* obj) { return obj->mode; }

float Vehicle_GetAppliedSteerRad(const Vehicle* obj) { return obj->applied_steer_rad; }

bool Vehicle_IsAutoStopActive(const Vehicle* obj) { return obj->auto_stop_active; }
