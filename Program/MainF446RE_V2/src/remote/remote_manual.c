#include "remote_manual.h"

#include "drive.h"
#include "lidar_utils.h"

#define AUTO_BRAKE_DISTANCE_MM 300.0f

void RemoteManual_Run(const RemoteCommand* cmd, const LD06* lidar) {
  if (cmd->do_brake) {
    Drive_Brake(0.5, cmd->steer);
    return;
  }

  if (cmd->enable_auto_brake) {
    float front_distance;
    Lidar_FindNearestSector(lidar, 0, 30, 5, 10, 3, &front_distance);  // 前方20度のセクタで最も近い点を探す
    bool front_blocked = front_distance < AUTO_BRAKE_DISTANCE_MM + Drive_GetSpeed() * 250.0f && cmd->move_speed > 0;
    float rear_distance;
    Lidar_FindNearestSector(lidar, 180, 30, 5, 10, 3, &rear_distance);  // 後方20度のセクタで最も近い点を探す
    bool rear_blocked = rear_distance < AUTO_BRAKE_DISTANCE_MM - Drive_GetSpeed() * 250.0f && cmd->move_speed < 0;
    if (front_blocked || rear_blocked) {
      Drive_Brake(0.5, cmd->steer);
      return;
    }
  }

  float steer = cmd->steer;
  if (Abs(Drive_GetSpeed()) > 1.0) {
    steer *= 1.0f - Constrain(Abs(Drive_GetSpeed()) / 5.0f, 0.0f, 0.8f);  // 高速域でステアリングを弱める
  }
  Drive_SetVelocity(cmd->move_speed, cmd->acceleration, steer);
}
