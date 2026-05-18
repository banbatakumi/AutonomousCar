#include "remote_manual.h"

#include "drive.h"
#include "lidar_utils.h"

#define AUTO_BRAKE_DISTANCE_MM 350.0f

void RemoteManual_Run(const RemoteCommand* cmd, const LD06* lidar) {
  if (cmd->do_brake) {
    Drive_Brake(0.5, cmd->steer);
    return;
  }

  if (cmd->enable_auto_brake) {
    bool front_blocked = Lidar_GetSector(lidar, 0, 20).avg < AUTO_BRAKE_DISTANCE_MM + Drive_GetSpeed() * 200.0f && cmd->move_speed > 0;
    bool rear_blocked = Lidar_GetSector(lidar, 180, 20).avg < AUTO_BRAKE_DISTANCE_MM - Drive_GetSpeed() * 200.0f && cmd->move_speed < 0;
    if (front_blocked || rear_blocked) {
      Drive_Brake(0.5, cmd->steer);
      return;
    }
  }

  Drive_SetVelocity(cmd->move_speed, cmd->acceleration, cmd->steer);
}
