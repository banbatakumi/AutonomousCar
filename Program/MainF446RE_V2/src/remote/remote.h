#ifndef REMOTE_H_
#define REMOTE_H_

#include <stdbool.h>
#include <stdint.h>

// シリアルで受信するリモコンコマンド
typedef struct {
  bool do_stop;
  bool do_brake;
  bool on_headlight;
  bool on_hazard;
  bool play_sound;
  bool enable_auto_brake;
  bool enable_traction_control;
  bool enable_stability_control;
  uint8_t mode;        // 0:停止 1:手動 2:自動1 3:自動2
  float move_speed;    // [m/s]
  float acceleration;  // [m/s²]
  float steer;         // -1.0(右) 〜 +1.0(左)
} RemoteCommand;

/**
 * @brief シリアル受信・コマンド解析・モード実行・テレメトリ送信を行う。
 *        MainApp() の MODE_STANDBY case から毎ループ呼ぶこと。
 */
void Remote_Update(void);

/**
 * @brief 最後に受信したコマンドを返す。
 */
const RemoteCommand* Remote_GetCommand(void);

#endif  // REMOTE_H_
