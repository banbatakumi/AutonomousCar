#include "ultrasonic.h"

void Ultrasonic_Init(Ultrasonic* obj, GPIO_TypeDef* trig_port, uint16_t trig_pin,
                     GPIO_TypeDef* echo_port, uint16_t echo_pin, float lpf_alpha) {
  DigitalOut_Init(&obj->trigger, trig_port, trig_pin);
  DigitalIn_Init(&obj->echo, echo_port, echo_pin);
  Timer_Init(&obj->timer);
  Timer_Reset(&obj->timer);
  LPF_Init(&obj->filter, lpf_alpha, 0);
  obj->distance = 0;
  obj->pulse_count = 0;
}

void Ultrasonic_Read(Ultrasonic* obj) {
  if (Timer_ReadMs(&obj->timer) >= 100) {
    Timer_Reset(&obj->timer);
    DigitalOut_Write(&obj->trigger, 0);

    obj->distance = LPF_Update(&obj->filter, obj->pulse_count);
    obj->pulse_count = 0;
  } else if (Timer_ReadMs(&obj->timer) >= 90) {
    DigitalOut_Write(&obj->trigger, 1);
  }

  if (DigitalIn_Read(&obj->echo) == 1) {
    obj->pulse_count++;
  }
}

uint16_t Ultrasonic_Get(Ultrasonic* obj) {
  return obj->distance;
}

void Ultrasonic_Reset(Ultrasonic* obj) {
  obj->distance = 0;
  obj->pulse_count = 0;
  Timer_Reset(&obj->timer);
}
