#ifndef REMOTE_AUTO1_H_
#define REMOTE_AUTO1_H_

#include "ld06.h"
#include "lidar_utils.h"
#include "remote.h"

/**
 * @brief 自動モード1 (mode == 2) の制御を実行する。
 */
void RemoteAuto1_Run(const RemoteCommand* cmd, const LD06* lidar);

#endif  // REMOTE_AUTO1_H_
