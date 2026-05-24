#ifndef REMOTE_AUTO2_H_
#define REMOTE_AUTO2_H_

#include "buzzer.h"
#include "ld06.h"
#include "lidar_utils.h"
#include "remote.h"

/**
 * @brief 自動モード2 (mode == 3) の制御を実行する。
 */
void RemoteAuto2_Run(const RemoteCommand* cmd, const LD06* lidar);

#endif  // REMOTE_AUTO2_H_
