#include "algo_forward.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drive.h"
#include "lidar_utils.h"
#include "mymath.h"
void Algorithm_ForwardOnly(LD06* lidar) {
  uint16_t front_dis = Lidar_GetSector(lidar, 0, 10).avg;
  if (front_dis > 0 && front_dis < 350 + Drive_GetSpeed() * 400.0f) {
    Drive_Brake(0.3f, 0.0f);
  } else {
    Drive_SetVelocity(2.0f, 1.0f, 0.0f);
  }
}
