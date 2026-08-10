#ifndef ENCODER_H
#define ENCODER_H

// ============================================================
// encoder.h — Quadrature wheel-encoder counting (Motors A, B, C)
//
// Uses one interrupt pin (C1) per motor and reads the second channel
// (C2) inside the ISR to decide count direction. Counts are signed
// and cumulative; convert to millimetres with the per-motor
// MM_PER_COUNT constants from robot_config.h.
//
// NOTE: This module was scaffolded from the encoder pins/CPR already
// present in robot_config.h (nothing read them before). The +/- count
// direction assumes a given wiring of C1/C2; if a wheel counts the
// "wrong way" when driven forward, swap that motor's C1/C2 wires or
// flip the sign in its ISR.
// ============================================================

#include <Arduino.h>

// Attach the encoder interrupts and configure the pins. Call from setup().
void encoderInit();

// Signed cumulative count for a wheel. motor is 'A', 'B', or 'C'.
long encoderCount(char motor);

// Distance travelled by a wheel since the last reset, in millimetres.
float encoderDistanceMM(char motor);

// Zero the count(s).
void encoderReset(char motor);
void encoderResetAll();

#endif // ENCODER_H
