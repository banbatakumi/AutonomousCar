#ifndef ULTRASONIC_H_
#define ULTRASONIC_H_

#include <stdint.h>
#include <stdio.h>

#include "digitalinout.h"
#include "lpf.h"
#include "timer.h"

#define DISTANSE_ADJUSTMENT 20  // センサーの誤差を補正するための係数（実測値に合わせて調整）

/**
 * Ultrasonic sensor driver
 * Measures distance using a trigger pin and echo pin
 */
typedef struct {
  DigitalOut trigger;
  DigitalIn echo;
  Timer timer;
  Timer dt_timer;
  LPF filter;
  uint16_t distance;
  uint16_t pulse_count;
} Ultrasonic;

/**
 * Initialize an ultrasonic sensor
 * @param obj Pointer to Ultrasonic structure
 * @param trig_port GPIO port for trigger pin
 * @param trig_pin GPIO pin number for trigger
 * @param echo_port GPIO port for echo pin
 * @param echo_pin GPIO pin number for echo
 * @param lpf_alpha Low-pass filter alpha coefficient (0.0-1.0)
 */
void Ultrasonic_Init(Ultrasonic* obj, GPIO_TypeDef* trig_port, uint16_t trig_pin,
                     GPIO_TypeDef* echo_port, uint16_t echo_pin, float lpf_alpha);

/**
 * Read the ultrasonic sensor (call periodically)
 * @param obj Pointer to Ultrasonic structure
 */
void Ultrasonic_Update(Ultrasonic* obj);

/**
 * Get the last measured distance
 * @param obj Pointer to Ultrasonic structure
 * @return Distance in cm
 */
uint16_t Ultrasonic_Get(Ultrasonic* obj);

/**
 * Reset the sensor
 * @param obj Pointer to Ultrasonic structure
 */
void Ultrasonic_Reset(Ultrasonic* obj);

#endif  // ULTRASONIC_H_
