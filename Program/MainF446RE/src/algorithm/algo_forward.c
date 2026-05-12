#include "algo_forward.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drive.h"
#include "lidar_utils.h"
#include "mymath.h"
void Algorithm_ForwardOnly(LD06* lidar) {
  Drive_SetVelocity(0.05f, 1.0f, 0);
}
