#ifndef DIAGNOSTICS_H
#define DIAGNOSTICS_H

// ============================================================
// diagnostics.h — Human-facing feedback: status LED + serial debug
//
// The status LED gives at-a-glance state with no laptop attached;
// the serial debug dump is for bench diagnosis.
//
// STATUS LED patterns (priority order):
//   slow blink (500ms)     -> waiting for a valid iBUS link
//   fast blink (150ms)     -> link OK, sticks not centered / not armed
//   solid ON               -> link OK, centered, disarmed & ready to arm
//   solid OFF              -> ARMED - motors live, be careful
//   very fast blink (60ms) -> failsafe / link lost while running
// ============================================================

#include <Arduino.h>

// Configure the status LED pin. Call from setup().
void diagInit();

// Low-level LED driver. onMs/offMs are the blink half-periods:
//   offMs == 0 -> solid ON,  onMs == 0 -> solid OFF.
// Non-blocking; call frequently.
void setStatusLed(unsigned long onMs, unsigned long offMs);

// Pick and apply the correct LED pattern for the current robot state.
void statusLedUpdate(bool failsafe, bool enabled, bool sticksCentered);

// Hand the latest per-wheel command to the diagnostics layer so the
// next debug dump can report it. powers are normalized (pre-direction),
// pwm values are the 0..MOTOR_PWM_MAX magnitudes actually sent.
void diagSetWheelDebug(float powerC, float powerA, float powerB,
                       int pwmC, int pwmA, int pwmB);

// Print a full state line (RC inputs, centers, per-wheel power/PWM).
// Throttle the call rate from the caller.
void debugPrint(bool enabled, bool failsafe);

#endif // DIAGNOSTICS_H
