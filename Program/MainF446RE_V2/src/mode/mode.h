#ifndef MODE_H_
#define MODE_H_

typedef enum {
  MODE_STANDBY,
  MODE_RUN,
  MODE_FORWARD_ONLY,
  MODE_MAX
} OperationMode;

void Mode_Init(int initial_button1_state, int initial_button2_state);
void Mode_Update(int button1_state, int button2_state);
OperationMode Mode_Get(void);

#endif  // MODE_H_
