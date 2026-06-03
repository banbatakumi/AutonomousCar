#ifndef REMOTE_MANUAL_H_
#define REMOTE_MANUAL_H_

#include "ld06.h"
#include "lidar_utils.h"
#include "remote.h"

/**
 * @brief 手動操縦モード (mode == 1) の制御を実行する。
 *        自動ブレーキが有効な場合、前後方向の障害物検知で自動減速する。
 */
void RemoteManual_Run(const RemoteCommand* cmd, const LD06* lidar);

#endif  // REMOTE_MANUAL_H_
