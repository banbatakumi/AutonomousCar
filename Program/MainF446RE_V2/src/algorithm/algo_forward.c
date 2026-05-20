#include "algo_forward.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "drive.h"
#include "lidar_utils.h"
#include "mymath.h"
void Algorithm_ForwardOnly(const LD06* lidar) {
  Drive_SetVelocity(1.0f, 2.0f, 0.0f);
}
