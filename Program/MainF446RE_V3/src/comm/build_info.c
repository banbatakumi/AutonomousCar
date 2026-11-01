#include "build_info.h"

#ifndef BUILD_EPOCH
#define BUILD_EPOCH 0u
#endif
#ifndef BUILD_GIT
#define BUILD_GIT "unknown"
#endif

const uint32_t g_build_epoch = BUILD_EPOCH;
const char g_build_git[] = BUILD_GIT;
