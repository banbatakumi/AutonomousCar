#include "vehicle.h"

#include <math.h>
#include <stdio.h>

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

// ブレーキ灯は「制動モード (braking)」か「一定以上の減速度」で点ける。後者は車速指令を
// 下げたときの減速 (車速PIの負トルク)・トルク指令の負トルクなど、制動モードを使わない減速でも
// 点けるためのもので、指令の種類ではなく実際の減速度だけで決める (VEHICLE_BRAKE_LIGHT_* 参照)。
//
// 緊急制動表示 (ESS) は実車と同じく、制動中かつ一定車速以上から強く減速したときだけ
// 点滅させ、減速が弱まるか停止したら常灯に戻す。制動トルクの大きさで決めると、
// 停止中にブレーキを踏み続けているだけ (フェイルセーフの停車保持を含む) でも点滅してしまう
static void ApplyBrakeLight(Vehicle* obj, bool braking) {
  float abs_speed = fabsf(Drive_GetVehicleSpeed(obj->drive));
  float window_s = Timer_Read(&obj->ess_decel_timer);
  if (window_s >= VEHICLE_ESS_DECEL_WINDOW_S) {
    // 異常に長い窓 (初回など) の差分は信用できないため捨てる
    obj->ess_decel_m_s2 = (window_s <= 0.1f)
                              ? (obj->ess_prev_abs_speed_m_s - abs_speed) / window_s
                              : 0.0f;
    obj->ess_prev_abs_speed_m_s = abs_speed;
    Timer_Reset(&obj->ess_decel_timer);
    // ブレーキ灯用の減速度は窓ごとの差分をさらに1次遅れでならす (窓が異常なら 0 へ戻す)
    obj->brake_light_decel_m_s2 =
        (window_s <= 0.1f)
            ? obj->brake_light_decel_m_s2 +
                  (obj->ess_decel_m_s2 - obj->brake_light_decel_m_s2) * window_s /
                      (VEHICLE_BRAKE_LIGHT_DECEL_TAU_S + window_s)
            : 0.0f;
  }

  if (!obj->brake_light_decel_on) {
    if (abs_speed >= VEHICLE_ESS_STOP_SPEED_M_S &&
        obj->brake_light_decel_m_s2 >= VEHICLE_BRAKE_LIGHT_ON_DECEL_M_S2) {
      obj->brake_light_decel_on = true;
      Timer_Reset(&obj->brake_light_hold_timer);
    }
  } else if ((abs_speed < VEHICLE_ESS_STOP_SPEED_M_S ||
              obj->brake_light_decel_m_s2 < VEHICLE_BRAKE_LIGHT_OFF_DECEL_M_S2) &&
             Timer_Read(&obj->brake_light_hold_timer) >= VEHICLE_BRAKE_LIGHT_MIN_ON_S) {
    obj->brake_light_decel_on = false;
  }

  if (!braking || abs_speed < VEHICLE_ESS_STOP_SPEED_M_S ||
      obj->ess_decel_m_s2 < VEHICLE_ESS_OFF_DECEL_M_S2) {
    obj->ess_active = false;
  } else if (abs_speed >= VEHICLE_ESS_MIN_SPEED_M_S &&
             obj->ess_decel_m_s2 >= VEHICLE_ESS_ON_DECEL_M_S2) {
    obj->ess_active = true;
  }

  Lighting_SetBrakeMode(obj->lighting, braking || obj->brake_light_decel_on, obj->ess_active);
}

// LiDAR電源を want_on に応じて更新する。ARM中は即座に点け、ARMが外れても
// VEHICLE_LIDAR_IDLE_OFF_DELAY_S が経つまでは点けたままにする (短い停止での
// 頻繁な電源入り切りと、それによる再ARM直後のセンサ空白を避けるため)
//
// 回転モータの PWM も電源と一緒に止める/戻す (電源を切った後も PA8 から出し続けると、
// 無給電の LD06 へ入力保護ダイオード経由で流れ込む恐れがある)
static void UpdateLidarPower(Vehicle* obj, bool want_on) {
  if (want_on) {
    Timer_Reset(&obj->lidar_idle_timer);
    if (!obj->lidar_on) {
      Power_SetLidarPower(obj->power, 1);
      obj->lidar_on = true;
    }
  } else if (obj->lidar_on && Timer_Read(&obj->lidar_idle_timer) >= VEHICLE_LIDAR_IDLE_OFF_DELAY_S) {
    Power_SetLidarPower(obj->power, 0);
    obj->lidar_on = false;
  }

  // PWM は実際に給電されている間だけ出す。シグナル系過電流では Power 側が給電を拒否・
  // 遮断する (Power_SetLidarPower / Power_Update) ので、lidar_on だけでは判断しない
  bool pwm_on = obj->lidar_on && !(Power_GetFaults(obj->power) & POWER_FAULT_SIGNAL_OVERCURRENT);
  if (pwm_on != obj->lidar_pwm_on) {
    Lidar_SetMotorDuty(obj->lidar, pwm_on ? LIDAR_PWM_DUTY : 0.0f);
    obj->lidar_pwm_on = pwm_on;
  }
}

// 車速から物理的に導かれる制動距離 (反応・処理遅延分 + 制動で止まるまでの距離) に、
// 上位が直接指定した安全マージン (RasConfig.auto_stop_margin_cm) を加えた
// 動的停止距離 [cm] を返す
static float AutoStopDistanceCm(float speed_m_s, const RasConfig* config) {
  float v = (speed_m_s >= 0.0f) ? speed_m_s : -speed_m_s;
  float physical_cm =
      (v * VEHICLE_AUTO_STOP_DELAY_S + (v * v) / (2.0f * VEHICLE_AUTO_STOP_DECEL_M_S2)) * 100.0f;
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
  // 前後判定は符号反転への追従速度が重要なため、PID/上位報告用の重いLPF
  // (Drive_GetVehicleSpeed, τ≈10ms) ではなく応答の速い方 (τ≈5ms) を使う
  float speed = Drive_GetVehicleSpeedForDirection(obj->drive);
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

  // Lidar_IsOk() (rx_count==0 か、300ms以上パケット自体が来ていない) を先に見ておく。
  // 未受信セクタの sector_update_us は起動直後から stale 初期化されているため
  // roi.fresh_count は本来ここが無くても正しく0になるが、センサ未接続時に
  // Lidar_QueryRoi へ触れず即座に超音波フォールバックへ倒すことを明示する
  LidarRoiResult roi = {0, 0};
  if (Lidar_IsOk(obj->lidar)) {
    roi = Lidar_QueryCorridor(obj->lidar, lidar_center_deg, VEHICLE_AUTO_STOP_LIDAR_HALF_WIDTH_CM,
                              d_stop_cm + lidar_offset_cm);
  }
  bool lidar_fresh = roi.fresh_count > 0;
  bool lidar_hit = roi.hit_count >= VEHICLE_AUTO_STOP_LIDAR_MIN_POINTS;

  // 近づいてくる障害物には LPF (計測1〜2回ぶんの遅れ。60ms間隔で 2m/s なら十数cm) が遅れるので、
  // 生値の方が近ければ生値を使う。生値が遠い・無反射のときは LPF の値 (単発の外れ値で止まらない)
  float us_filtered_cm = forward ? RangeSensor_GetFrontDistanceFilteredCm(obj->range_sensor)
                                  : RangeSensor_GetRearDistanceFilteredCm(obj->range_sensor);
  float us_raw_cm = forward ? RangeSensor_GetFrontDistanceCm(obj->range_sensor)
                             : RangeSensor_GetRearDistanceCm(obj->range_sensor);
  float us_dist_cm = us_filtered_cm;
  if (us_raw_cm >= 0.0f && (us_dist_cm < 0.0f || us_raw_cm < us_dist_cm)) us_dist_cm = us_raw_cm;
  bool us_near_hit = us_dist_cm >= 0.0f &&
                     us_dist_cm < (VEHICLE_AUTO_STOP_ULTRASONIC_NEAR_CM + us_offset_cm);
  bool us_fallback_hit = !lidar_fresh && us_dist_cm >= 0.0f &&
                         us_dist_cm < (d_stop_cm + us_offset_cm);

  return lidar_hit || us_near_hit || us_fallback_hit;
}

// ブレーキホールドの条件 (制動中に停止が続いている) を更新して返す。毎周期1回だけ呼ぶこと
static bool UpdateBrakeHold(Vehicle* obj, bool braking) {
  if (!braking || fabsf(Drive_GetVehicleSpeed(obj->drive)) >= VEHICLE_BRAKE_HOLD_SPEED_M_S) {
    Timer_Reset(&obj->brake_hold_timer);
    obj->brake_hold = false;
  } else if (!obj->brake_hold && Timer_Read(&obj->brake_hold_timer) >= VEHICLE_BRAKE_HOLD_DELAY_S) {
    obj->brake_hold = true;
  }
  return obj->brake_hold;
}

// 走行中に上位の ARM が外れたら、停車するまで駆動電源を入れたまま最大制動を続ける。
// ARM ビットでそのまま駆動電源を切ると MD ごと落ちて制動できず、惰行で止まるのを待つことになる
// (上位は通信途絶・デッドマンで「安全側」として DISARM を送ってくる)
static void UpdateDisarmBraking(Vehicle* obj, bool arm_requested) {
  if (arm_requested) {
    obj->disarm_braking = false;
  } else if (obj->arm_requested_prev) {
    // 駆動電源が入っていなければ (ステア未較正・過電流での遮断) 制動する手段が無い
    obj->disarm_braking = Power_IsDriveOn(obj->power);
    Timer_Reset(&obj->disarm_brake_timer);
  }
  obj->arm_requested_prev = arm_requested;

  if (obj->disarm_braking &&
      (fabsf(Drive_GetVehicleSpeed(obj->drive)) < DRIVE_STANDSTILL_SPEED_M_S ||
       Timer_Read(&obj->disarm_brake_timer) >= VEHICLE_DISARM_BRAKE_TIMEOUT_S)) {
    obj->disarm_braking = false;
  }
}

static const char* const kMdNames[MOTORS_COUNT] = {"rear-left", "rear-right", "steering"};

// md_fault を立て、どの MD がなぜ使えないかを上位へ1回だけ知らせる
static void LatchMdFault(Vehicle* obj, const char* when) {
  obj->md_fault = true;
  for (int i = 0; i < MOTORS_COUNT; i++) {
    const char* reason = Motors_NotReadyReason(obj->motors, i);
    if (reason == NULL) continue;
    char text[64];
    snprintf(text, sizeof(text), "MD %s %s: %s", kMdNames[i], when, reason);
    RasLink_Log(obj->ras_link, RAS_LOG_ERROR, text);
  }
}

// MD の起動待ちと健全性の監視。駆動電源が入っている間、毎周期呼ぶ。
// 走行制御は MD の状態を見ずにトルクを指令するので、ここで見ていないと、片側の後輪 MD が
// 過熱・電圧異常で止まっても反対側は駆動を続け (ヨーモーメントが出る)、ステア MD が
// 止まっても舵が効かないまま走り続ける
static void UpdateMdHealth(Vehicle* obj) {
  bool powered = Power_IsDriveOn(obj->power);
  if (!powered) {
    obj->md_started = false;
    obj->md_fault = false;
    obj->md_unready = false;
  } else {
    if (!obj->drive_power_prev) Timer_Reset(&obj->md_boot_timer);
    bool ready = Motors_IsReady(obj->motors);
    if (!obj->md_fault) {
      if (!obj->md_started) {
        if (ready) {
          obj->md_started = true;
        } else if (Timer_Read(&obj->md_boot_timer) >= VEHICLE_MD_BOOT_TIMEOUT_S) {
          LatchMdFault(obj, "did not start");
        }
      } else if (ready) {
        obj->md_unready = false;
      } else if (!obj->md_unready) {
        obj->md_unready = true;
        Timer_Reset(&obj->md_unready_timer);
      } else if (Timer_Read(&obj->md_unready_timer) >= VEHICLE_MD_FAULT_DEBOUNCE_S) {
        LatchMdFault(obj, "stopped");
      }
    }
  }
  obj->drive_power_prev = powered;
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

  // mode = 3 は予約なので、受信しても現在のモードを維持する
  if (command->mode != RAS_MODE_RESERVED) obj->mode = command->mode;

  bool arm_requested = (command->flags & RAS_CMD_FLAG_ARM) != 0;
  // フェイルセーフのあとは ARM の立ち上がりを要求する (vehicle.h の rearm_required)。受け付けない間は
  // ARM が外れているのと同じに扱うので、走っていれば下の UpdateDisarmBraking が停車まで制動する
  if (!arm_requested) {
    obj->rearm_required = false;
    obj->rearm_logged = false;
  } else if (obj->rearm_required) {
    arm_requested = false;
    if (!obj->rearm_logged) {
      RasLink_Log(obj->ras_link, RAS_LOG_WARN, "ARM ignored: disarm once to re-arm (after e-stop / COMMAND loss)");
      obj->rearm_logged = true;
    }
  }
  // 中心点が未較正だと舵角の絶対値が信用できないため走行させない。走らせない以上、
  // 駆動電源も入れない (MD に給電しなければ、どの経路の指令も届かない)
  bool center_valid = Steering_IsCenterValid(obj->steering);
  obj->failsafe_idle = false;
  UpdateDisarmBraking(obj, arm_requested);
  Power_SetDrivePower(obj->power, (arm_requested && center_valid) || obj->disarm_braking);
  UpdateLidarPower(obj, arm_requested);

  // 3台の MD が使える状態になるまでは走らせない。MD は給電から約1秒指令を受け付けず、
  // その間に車速 PI を回すと、積分が振り切れた状態で MD が動き出して最大トルクが一気に出る
  bool armed = arm_requested && center_valid && obj->md_started &&
               (obj->mode == RAS_MODE_MANUAL || obj->mode == RAS_MODE_AUTO);
  obj->armed = armed;
  // 上位の指令に関わらず最大制動で止める場合: ARM が外れたあとの停止待ちと、MD の異常
  bool forced_braking = obj->disarm_braking || obj->md_fault;
  bool cmd_braking = (command->flags & RAS_CMD_FLAG_BRAKE) != 0;
  bool torque_mode = (command->flags & RAS_CMD_FLAG_TORQUE_MODE) != 0;
  bool auto_stop_enabled = (command->flags & RAS_CMD_FLAG_AUTO_STOP) != 0;
  bool side_brake_requested = (command->flags2 & RAS_CMD_FLAG2_SIDE_BRAKE) != 0;
  bool launch_requested = (command->flags2 & RAS_CMD_FLAG2_LAUNCH) != 0;
  bool winker_left = (command->flags2 & RAS_CMD_FLAG2_WINKER_LEFT) != 0;
  bool winker_right = (command->flags2 & RAS_CMD_FLAG2_WINKER_RIGHT) != 0;
  if (winker_left && winker_right) {
    obj->winker_request = LIGHTING_WINKER_HAZARD;
  } else if (winker_left) {
    obj->winker_request = LIGHTING_WINKER_LEFT;
  } else if (winker_right) {
    obj->winker_request = LIGHTING_WINKER_RIGHT;
  } else {
    obj->winker_request = LIGHTING_WINKER_OFF;
  }

  // 静止時の前後判定フォールバック用。torque_mode なら target_torque、そうでなければ
  // target_speed の符号を「これから進もうとしている方向」として使う
  float desired_direction = torque_mode ? command->target_torque_nm : command->target_speed_m_s;
  // 一度作動したら、止まるまで解除しない。停止距離は車速とともに縮むので、判定だけで決めると
  // 減速した途端に「範囲外」になって解除→再加速→再作動を繰り返しながら障害物へ寄っていく。
  // 止まってからは毎周期判定し直すので、障害物が無くなれば (または上位が逆方向を指令すれば) 動ける
  bool auto_stop_moving =
      fabsf(Drive_GetVehicleSpeedForDirection(obj->drive)) > VEHICLE_AUTO_STOP_DIRECTION_DEADBAND_M_S;
  obj->auto_stop_active =
      armed && auto_stop_enabled &&
      ((obj->auto_stop_active && auto_stop_moving) ||
       IsAutoStopObstacleAhead(obj, config, desired_direction));
  bool braking = cmd_braking || obj->auto_stop_active || forced_braking;

  float target_speed_m_s = 0.0f;
  if (armed && !braking && !torque_mode && !side_brake_requested) {
    target_speed_m_s = command->target_speed_m_s;  // 最終クランプは Drive_SetTargetSpeed が行う
  }
  // 制動トルクの指定が無い (0) ときは最大で掛ける。0 をそのまま「制動トルク0」と解釈すると、
  // 上位がフィールドを埋め忘れただけでブレーキが効かなくなる。自動停止はできるだけ強く
  // 止めることが目的なので、上位が明示的にブレーキを踏んでいるとき以外は常に最大で掛ける
  float brake_torque_nm = (cmd_braking && !forced_braking && command->brake_torque_nm > 0.0f)
                               ? command->brake_torque_nm
                               : DRIVE_MAX_BRAKE_TORQUE_NM;

  float target_steer_rad = Constrain(command->target_steer_rad, -Steering_GetMaxRoadWheelAngleRad(),
                                     Steering_GetMaxRoadWheelAngleRad());

  float steer_rate_limit = command->steer_rate_limit_rad_s > 0.0f
                               ? command->steer_rate_limit_rad_s
                               : VEHICLE_DEFAULT_STEER_RATE_RAD_S;

  // armed でない間 (中心点未較正・DISARM・MD の起動待ち・自律/手動以外のモード) は
  // target_speed 側と同様に舵角も進めない。applied_steer_rad を凍結して最後に送った角度を
  // MD に保持させ続ける (未較正の間は Steering 側が何も送らない)。再アーム時は
  // 凍結していた applied_steer_rad から steer_rate_limit で滑らかに追従する
  if (armed) {
    float steer_step = steer_rate_limit * dt_s;
    obj->applied_steer_rad +=
        Constrain(target_steer_rad - obj->applied_steer_rad, -steer_step, steer_step);
  }
  // armed でなくても指令 (凍結した角度) は出し続ける。ステア MD は指令フレームを受け取って
  // 初めてトルク上限を適用するので、送らないと「MD が揃うまで armed にしない」が永久に
  // 解けない (起動してから一度もフェイルセーフを通らずに上位の COMMAND が届いた場合)
  Steering_SetRoadWheelAngleRad(obj->steering, obj->applied_steer_rad);

  // Drive_Enable は積分項とTCの上限をリセットするため、状態が変わったときだけ呼ぶ。
  // ARM が外れたあとの停止待ちの間は、制動を掛けるために有効のままにする
  bool drive_active = armed || obj->disarm_braking;
  if (drive_active && !Drive_IsEnabled(obj->drive)) Drive_Enable(obj->drive);
  if (!drive_active && Drive_IsEnabled(obj->drive)) Drive_Disable(obj->drive);

  // 加速度レート制限 (上位が送る accel_limit の傾き。上限なし) は Drive 側が持つため、
  // 生の目標車速をそのまま渡す。上限0は「制限なし」ではなく「動かない」になってしまうため、
  // 0 のときは Drive_SetTargetSpeed 側で DRIVE_DEFAULT_ACCEL_M_S2 に読み替える
  Drive_SetTargetSpeed(obj->drive, target_speed_m_s, command->accel_limit_m_s2);
  Drive_SetBrake(obj->drive, braking, brake_torque_nm);
  Drive_SetTorque(obj->drive, torque_mode, command->target_torque_nm);
  // 制動・torque_mode・サイドブレーキとの優先順位は Drive 側が持つ (車速指令の発進でだけ働く)
  Drive_SetLaunch(obj->drive, armed && launch_requested);
  // 後輪の位置保持に入るのは、上位のサイドブレーキ要求か、ブレーキホールド (制動して止まった)。
  // 未アーム中は無効化する (アームが外れた瞬間に位置保持へ入り込まないようにするため)。
  // 強制の制動中は上位のサイドブレーキ要求を聞かない (Drive は位置保持を制動より優先するので、
  // 走行中に入ると最大制動が効かなくなる)。ブレーキホールドは止まってからなので構わない
  bool brake_hold = UpdateBrakeHold(obj, armed && braking) && config->brake_hold_enabled;
  Drive_SetSideBrake(obj->drive,
                     armed && ((side_brake_requested && !forced_braking) || brake_hold));
  Drive_SetTractionControlEnabled(obj->drive, config->tc_enabled);
  Drive_SetTorqueVectoringEnabled(obj->drive, config->tv_enabled);
  Drive_SetWheelLiftGuardEnabled(obj->drive, config->wheel_lift_guard_enabled);
  Drive_SetAbsEnabled(obj->drive, config->abs_enabled);
  Drive_SetParams(obj->drive, &config->control);

  // 上位が生きている間の制動は常に全光量 (減光はフェイルセーフの停車保持だけ)
  Lighting_SetBrakeDimmed(obj->lighting, false);
  ApplyBrakeLight(obj, braking);
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
// 走り出さないため)。解除はハートビートが戻っている状態でボタン2を押す。解除しても、上位が ARM を
// 一度下ろすまでは走らせない (ApplyRasCommand の rearm_required)。
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
// 止め、止まったら後輪の位置保持へ移る。上位の操作で入りっぱなしになりうる出力
// (クラクション・パッシング) は解除する。
// 舵角は最後の指令値のまま保持する (直進へ戻すと車体が予期しない方向へ動くため)
static void ApplyFailsafe(Vehicle* obj) {
  // ApplyRasCommand() 側の command_rate_timer をここでもリセットしておく。フェイルセーフ中は
  // 呼ばれないため、これをしないと復帰した瞬間の dt が途絶していた時間分だけ膨らみ、
  // 加速度・舵角速度のレート制限が1周期だけ無効化されたのと同じ状態になる
  Timer_Reset(&obj->command_rate_timer);

  // 上位が DISARM を送っていた間 (MD の起動待ちを含む) は Drive が無効で、そのままでは
  // 下の制動が出ない。駆動電源が入ったまま途絶しても必ず制動が掛かるようにする
  if (!Drive_IsEnabled(obj->drive)) Drive_Enable(obj->drive);

  // 止まったら停車保持へ移る。動き出したら (押された・坂で保持し切れない) 最大制動からやり直す
  bool stopped = UpdateBrakeHold(obj, true);

  // 止まったまま長く保持し続けない (過熱・電池消耗の防止)。切ったあとは ARM し直すまで電源を入れない
  // (電源は ApplyRasCommand だけが入れる)。動き出した (押された・坂で滑る) らやり直す
  if (!stopped || !Power_IsDriveOn(obj->power)) {
    obj->failsafe_idle = false;
  } else if (!obj->failsafe_idle) {
    obj->failsafe_idle = true;
    obj->failsafe_idle_start_us = Micros();
  } else if ((uint32_t)(Micros() - obj->failsafe_idle_start_us) >=
             (uint32_t)(VEHICLE_FAILSAFE_POWER_OFF_S * 1000000.0f)) {
    Power_SetDrivePower(obj->power, 0);
    obj->failsafe_idle = false;
    RasLink_Log(obj->ras_link, RAS_LOG_WARN, "drive power off: stopped for a long time in failsafe");
  }

  Drive_SetTargetSpeed(obj->drive, 0.0f, 0.0f);
  Drive_SetBrake(obj->drive, true, DRIVE_MAX_BRAKE_TORQUE_NM);
  Drive_SetTorque(obj->drive, false, 0.0f);
  Drive_SetLaunch(obj->drive, false);
  // 上位の指令で入っていたサイドブレーキ (後輪の位置保持) は持ち越さない。Drive は位置保持を
  // 制動より優先するので、走行中に入ったままだと最大制動が効かなくなる。止まるまでは必ず
  // 制動トルクで減速し、止まってからだけ位置保持へ切り替える。MD の制動は回転と逆向きの
  // トルクで、車輪が止まる手前で抜ける (2rad/s 未満で 0)。それだけでは坂で動き出す
  Drive_SetSideBrake(obj->drive,
                     stopped && RasLink_GetConfig(obj->ras_link)->brake_hold_enabled);
  Steering_SetRoadWheelAngleRad(obj->steering, obj->applied_steer_rad);
  obj->armed = false;
  obj->rearm_required = true;
  // 減速・クリープしている間は全光量のまま (後続への表示を弱めない)。Pi 未接続・起動中は
  // ずっとここを通るので、止まったら減光する (省電力。ブレーキホールドが無効でも)
  Lighting_SetBrakeDimmed(obj->lighting, stopped);
  ApplyBrakeLight(obj, true);
  Lighting_SetPassing(obj->lighting, false);
  SetHorn(obj, false);
  obj->winker_request = LIGHTING_WINKER_OFF;
  obj->auto_stop_active = false;
}

void Vehicle_Init(Vehicle* obj, RasLink* ras_link, Drive* drive, Motors* motors, Steering* steering,
                  Lighting* lighting, Power* power, Heartbeat* heartbeat, Buzzer* buzzer,
                  DigitalIn* estop_reset_button, RangeSensor* range_sensor, Lidar* lidar) {
  obj->ras_link = ras_link;
  obj->drive = drive;
  obj->motors = motors;
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
  obj->winker_request = LIGHTING_WINKER_OFF;
  obj->auto_stop_active = false;
  obj->auto_stop_blind = false;
  obj->auto_stop_blind_pending = false;
  obj->auto_stop_blind_start_us = 0;
  // Steering の既定の換算 (gain 1, cubic 0) と同じ。RasConfig の既定も同じなので起動直後は何もしない
  obj->linkage_gain = 1.0f;
  obj->linkage_cubic = 0.0f;
  obj->failsafe_idle = false;
  obj->failsafe_idle_start_us = 0;

  Timer_Init(&obj->ess_decel_timer);
  obj->ess_prev_abs_speed_m_s = 0.0f;
  obj->ess_decel_m_s2 = 0.0f;
  obj->ess_active = false;
  Timer_Init(&obj->brake_light_hold_timer);
  obj->brake_light_decel_m_s2 = 0.0f;
  obj->brake_light_decel_on = false;

  // Setup() は LIDAR_POWER を意図的に投入しない (Pi 未接続/DISARM の間は LiDAR も
  // 無給電にする既定のため、実機で確認済み)。ここを true にすると UpdateLidarPower() の
  // if (!obj->lidar_on) が常に偽になり、初回 ARM で Power_SetLidarPower() が一度も
  // 呼ばれず給電されないまま放置される不具合になるため、実際の起動時給電状態 (false) に
  // 合わせておく
  Timer_Init(&obj->lidar_idle_timer);
  obj->lidar_on = false;
  // Lidar_Init() は回転 PWM を duty 40% で出し始めるが、給電は ARM まで入らないので止めておく
  // (UpdateLidarPower() が給電と一緒に戻す)
  Lidar_SetMotorDuty(obj->lidar, 0.0f);
  obj->lidar_pwm_on = false;

  obj->armed = false;
  obj->prev_armed = false;
  Timer_Init(&obj->brake_hold_timer);
  obj->brake_hold = false;

  obj->rearm_required = true;
  obj->rearm_logged = false;
  obj->arm_requested_prev = false;
  obj->disarm_braking = false;
  Timer_Init(&obj->disarm_brake_timer);

  obj->drive_power_prev = false;
  obj->md_started = false;
  obj->md_fault = false;
  Timer_Init(&obj->md_boot_timer);
  obj->md_unready = false;
  Timer_Init(&obj->md_unready_timer);
}

// armed が変化した周期に一度だけ通知音を鳴らす。緊急停止・通信途絶で落ちた場合も
// DISARM 音になる (異常停止も音で分かる)
static void NotifyArmChange(Vehicle* obj) {
  if (obj->armed == obj->prev_armed) return;
  obj->prev_armed = obj->armed;
  if (obj->armed) {
    // ハートビートを一度も見ていなければ緊急停止 (UpdateEstop) は働かない。ベンチでの単体確認の
    // ために走行は止めないが、配線の抜け・上位の出力忘れに気付けるよう上位へ知らせる
    if (!Heartbeat_HasEverBeenSeen(obj->heartbeat)) {
      RasLink_Log(obj->ras_link, RAS_LOG_WARN, "armed without heartbeat (RAS_SIG never seen): e-stop is inactive");
    }
    Buzzer_Sweep(obj->buzzer, VEHICLE_ARM_SWEEP_LOW_HZ, VEHICLE_ARM_SWEEP_HIGH_HZ,
                 VEHICLE_ARM_SWEEP_DURATION_MS);
  } else {
    Buzzer_Sweep(obj->buzzer, VEHICLE_ARM_SWEEP_HIGH_HZ, VEHICLE_ARM_SWEEP_LOW_HZ,
                 VEHICLE_ARM_SWEEP_DURATION_MS);
  }
}

// CONFIG_SET で受けたステアのリンクの換算を、停車して DISARM のときに適用する。走っている間に
// 換算が変わると、同じ指令でも舵角が途中で変わる。適用したら可動範囲が変わるので LIMITS を送り直す
static void UpdateSteeringLinkage(Vehicle* obj) {
  if (obj->armed || obj->disarm_braking) return;
  if (fabsf(Drive_GetVehicleSpeed(obj->drive)) >= DRIVE_STANDSTILL_SPEED_M_S) return;
  const RasConfig* config = RasLink_GetConfig(obj->ras_link);
  if (config->control.steer_link_gain == obj->linkage_gain &&
      config->control.steer_link_cubic == obj->linkage_cubic) {
    return;
  }
  Steering_SetLinkage(config->control.steer_link_gain, config->control.steer_link_cubic);
  obj->linkage_gain = config->control.steer_link_gain;
  obj->linkage_cubic = config->control.steer_link_cubic;
  RasLink_SendLimits(obj->ras_link);
}

static void UpdateInner(Vehicle* obj) {
  UpdateEstop(obj);
  UpdateSteeringLinkage(obj);
  UpdateMdHealth(obj);

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

// 自動停止が何も検知できない状態か (ARM 中に LiDAR も前後の超音波も使えない) を更新する。
// 超音波はトリガを止めている DISARM 中は使えないので、ARM 中だけ見る
static void UpdateAutoStopBlind(Vehicle* obj) {
  bool blind_now = obj->armed && !Lidar_IsOk(obj->lidar) &&
                   !RangeSensor_IsFrontAlive(obj->range_sensor) &&
                   !RangeSensor_IsRearAlive(obj->range_sensor);
  if (!blind_now) {
    obj->auto_stop_blind_pending = false;
    obj->auto_stop_blind = false;
  } else if (!obj->auto_stop_blind_pending) {
    obj->auto_stop_blind_pending = true;
    obj->auto_stop_blind_start_us = Micros();
  } else if ((uint32_t)(Micros() - obj->auto_stop_blind_start_us) >=
             (uint32_t)(VEHICLE_AUTO_STOP_BLIND_DELAY_S * 1000000.0f)) {
    obj->auto_stop_blind = true;
  }
}

void Vehicle_Update(Vehicle* obj) {
  UpdateInner(obj);
  UpdateAutoStopBlind(obj);
  NotifyArmChange(obj);
}

bool Vehicle_IsEstopLatched(const Vehicle* obj) { return obj->estop_latched; }

bool Vehicle_IsArmed(const Vehicle* obj) { return obj->armed; }

bool Vehicle_IsMdFault(const Vehicle* obj) { return obj->md_fault; }

uint8_t Vehicle_GetMode(const Vehicle* obj) { return obj->mode; }

float Vehicle_GetAppliedSteerRad(const Vehicle* obj) { return obj->applied_steer_rad; }

bool Vehicle_IsAutoStopActive(const Vehicle* obj) { return obj->auto_stop_active; }

bool Vehicle_IsHeartbeatMissing(const Vehicle* obj) {
  return !Heartbeat_HasEverBeenSeen(obj->heartbeat);
}

bool Vehicle_IsAutoStopBlind(const Vehicle* obj) { return obj->auto_stop_blind; }

LightingWinkerState Vehicle_GetWinkerRequest(const Vehicle* obj) { return obj->winker_request; }

bool Vehicle_IsWinkerLeftActive(const Vehicle* obj) {
  LightingWinkerState state = Lighting_GetWinkerState(obj->lighting);
  return state == LIGHTING_WINKER_LEFT || state == LIGHTING_WINKER_HAZARD;
}

bool Vehicle_IsWinkerRightActive(const Vehicle* obj) {
  LightingWinkerState state = Lighting_GetWinkerState(obj->lighting);
  return state == LIGHTING_WINKER_RIGHT || state == LIGHTING_WINKER_HAZARD;
}
