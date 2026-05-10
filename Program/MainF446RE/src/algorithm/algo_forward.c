#include "algo_forward.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drive.h"
#include "lidar_utils.h"
#include "mymath.h"
void Algorithm_ForwardOnly(LD06* lidar) {
  // Heading hold 単体確認用: 速度 0・ステア 0 を指令し、
  // heading hold の補正だけをサーボに反映させる。
  // 車体を手で右回転 → サーボが左へ動けば符号正常。
  // 逆なら drive.c の error の符号を反転すること。
  Drive_Set(0.0f, 0.0f, 0.0f);
}
