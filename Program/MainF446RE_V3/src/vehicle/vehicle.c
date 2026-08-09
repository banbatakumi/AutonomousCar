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

// 上位の指令を車両へ適用する。目標値そのものではなく、加速度・舵角速度の上限で
// レート制限した値を渡す (急な指令変化でタイヤを滑らせたり据え切りでラックを痛めないため)
static void ApplyRasCommand(Vehicle* obj) {
  const RasCommand* command = RasLink_GetCommand(obj->ras_link);
  const RasConfig* config = RasLink_GetConfig(obj->ras_link);
  float dt_s = Timer_Read(&obj->command_rate_timer);
  Timer_Reset(&obj->command_rate_timer);

  // mode = 3 は v0.4 で予約になったため、受信しても現在のモードを維持する
  if (command->mode != RAS_MODE_RESERVED) obj->mode = command->mode;

  bool arm_requested = (command->flags & RAS_CMD_FLAG_ARM) != 0;
  Power_SetDrivePower(obj->power, arm_requested);

  // 中心点が未較正だと舵角の絶対値が信用できないため走行させない
  bool armed = arm_requested && Steering_IsCenterValid(obj->steering) &&
               (obj->mode == RAS_MODE_MANUAL || obj->mode == RAS_MODE_AUTO);
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
  if (braking) obj->applied_speed_m_s = 0.0f;

  float speed_step = accel_limit * dt_s;
  obj->applied_speed_m_s +=
      Constrain(target_speed_m_s - obj->applied_speed_m_s, -speed_step, speed_step);
  float steer_step = steer_rate_limit * dt_s;
  obj->applied_steer_rad +=
      Constrain(target_steer_rad - obj->applied_steer_rad, -steer_step, steer_step);

  // Drive_Enable は積分項とTCの上限をリセットするため、状態が変わったときだけ呼ぶ
  if (armed && !Drive_IsEnabled(obj->drive)) Drive_Enable(obj->drive);
  if (!armed && Drive_IsEnabled(obj->drive)) Drive_Disable(obj->drive);

  Drive_SetTargetSpeed(obj->drive, obj->applied_speed_m_s);
  Drive_SetBrake(obj->drive, braking, brake_torque_nm);
  Steering_SetRoadWheelAngleRad(obj->steering, obj->applied_steer_rad);

  Lighting_SetBrake(obj->lighting, braking);
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
  obj->applied_speed_m_s = 0.0f;
  Drive_SetTargetSpeed(obj->drive, 0.0f);
  Drive_SetBrake(obj->drive, true, DRIVE_MAX_BRAKE_TORQUE_NM);
  Steering_SetRoadWheelAngleRad(obj->steering, obj->applied_steer_rad);
  Lighting_SetBrake(obj->lighting, true);
  Lighting_SetPassing(obj->lighting, false);
  SetHorn(obj, false);
}

void Vehicle_Init(Vehicle* obj, RasLink* ras_link, Drive* drive, Steering* steering,
                  Lighting* lighting, Power* power, Heartbeat* heartbeat, Buzzer* buzzer,
                  DigitalIn* estop_reset_button) {
  obj->ras_link = ras_link;
  obj->drive = drive;
  obj->steering = steering;
  obj->lighting = lighting;
  obj->power = power;
  obj->heartbeat = heartbeat;
  obj->buzzer = buzzer;
  obj->estop_reset_button = estop_reset_button;

  obj->applied_speed_m_s = 0.0f;
  obj->applied_steer_rad = 0.0f;
  Timer_Init(&obj->command_rate_timer);

  obj->mode = RAS_MODE_DISARM;
  obj->estop_latched = false;
  obj->horn_on = false;
}

void Vehicle_Update(Vehicle* obj) {
  UpdateEstop(obj);

  // 緊急停止は上位の指令より優先する
  if (obj->estop_latched) {
    ApplyFailsafe(obj);
    return;
  }

  if (RasLink_IsCommandAlive(obj->ras_link)) {
    ApplyRasCommand(obj);
  } else {
    // COMMAND が一度も届いていない間も含め、上位と繋がっていなければ停車保持。
    // 上位が一度でも繋がった後も、通信が復帰するまでこの状態を続ける
    ApplyFailsafe(obj);
  }
}

bool Vehicle_IsEstopLatched(const Vehicle* obj) { return obj->estop_latched; }

uint8_t Vehicle_GetMode(const Vehicle* obj) { return obj->mode; }

float Vehicle_GetAppliedSteerRad(const Vehicle* obj) { return obj->applied_steer_rad; }
