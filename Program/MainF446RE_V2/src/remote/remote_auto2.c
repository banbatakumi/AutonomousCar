#include "remote_auto2.h"

#include "drive.h"

void RemoteAuto2_Run(const RemoteCommand* cmd, const LD06* lidar) {
  (void)cmd;
  (void)lidar;
  Drive_Free();
}
