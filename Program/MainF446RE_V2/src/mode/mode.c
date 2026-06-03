#include "mode.h"

static OperationMode current_mode = MODE_STANDBY;
static int prev_button1_state = 0;
static int prev_button2_state = 0;

void Mode_Init(int initial_button1_state, int initial_button2_state) {
  current_mode = MODE_STANDBY;
  prev_button1_state = initial_button1_state;
  prev_button2_state = initial_button2_state;
}

bool Mode_Update(int button1_state, int button2_state) {
  bool any_pressed = false;

  // button1が押された瞬間: STANDBY <-> RUN のトグル
  if (button1_state == 1 && prev_button1_state == 0) {
    current_mode = (current_mode == MODE_RUN) ? MODE_STANDBY : MODE_RUN;
    any_pressed = true;
  }
  prev_button1_state = button1_state;

  // button2が押された瞬間: STANDBY <-> FORWARD_ONLY のトグル
  if (button2_state == 1 && prev_button2_state == 0) {
    current_mode = (current_mode == MODE_FORWARD_ONLY) ? MODE_STANDBY : MODE_FORWARD_ONLY;
    any_pressed = true;
  }
  prev_button2_state = button2_state;

  return any_pressed;
}

OperationMode Mode_Get(void) {
  return current_mode;
}
