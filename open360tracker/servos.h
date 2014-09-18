#ifndef SERVOS_H
#define SERVOS_H
/**
 * Functions for handling tilt and pan servos
 *
 * The servos needs to be connected to the hardware PWMs
 * of the arduino, these are Digital 9 (PAN) and Digital 10 (Tilt)
 */

#include <avr/io.h>
#include "config.h"


// use this define to set the tilt servo pulse duration directly
#define TILT_SERVO OCR1B
#define PAN_SERVO OCR1A
#define SET_PAN_SERVO_SPEED(A) PAN_SERVO = A * 2
#define SET_TILT_SERVO_SPEED(A) TILT_SERVO = A * 2

// Initializes both hardware PWMs to be used for tilt and pan servos.
inline void initServos()
{
  // Set OC1A (PB1) and OC1B (PB2) to output.
  // these are our hardware PWM ports.
  DDRB |= _BV(PORTB1) | _BV(PORTB2);
  
  // Setup of Timer1, Prescaler 8, 16bit fast pwm
  TCCR1A = _BV(WGM11) | _BV(COM1A1) | _BV(COM1B1);
  TCCR1B = _BV(WGM12) | _BV(WGM13) | _BV(CS11);
  // we need to multiply our pwm value by 2
  OCR1A = PAN_0 * 2;
  OCR1B = TILT_0 * 2;
  // ((16mhz / 8 Prescaler) / 50hz) - 1 = 40000
  ICR1 = 39999;
}

#endif

