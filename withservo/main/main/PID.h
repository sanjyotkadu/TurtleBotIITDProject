#ifndef PID_H
#define PID_H

// ============================================================
// PID.h — Per-wheel closed-loop velocity control (Motors A, B, C)
//
// Built on the standard Arduino PID library (PID_v1 by Brett
// Beauregard). Install it once via the IDE:
//   Sketch -> Include Library -> Manage Libraries... -> search "PID"
//   -> install "PID by Brett Beauregard".
// (Library home: github.com/br3ttb/Arduino-PID-Library)
//
// This module owns three PID objects from that library — one per wheel —
// reads the encoders to measure each wheel's ACTUAL speed, and turns the
// open-loop normalized wheel powers from kiwiMix() into CLOSED-LOOP
// powers so all three wheels really hit the commanded speed (instead of
// assuming "power == speed", which drifts under load / low battery).
//
// The Arduino PID library already gives us the hard parts for free:
//   - fixed sample-time scheduling (SetSampleTime)
//   - integral anti-windup (clamped to the output limits)
//   - derivative-on-measurement (no "derivative kick" on setpoint jumps)
// so we only add: velocity sensing from the encoders + unit plumbing.
//
// Units: velocities are millimetres/second (mm/s). PID output is a
// normalized power in -1.0 .. +1.0, exactly what driveMotor() expects.
//
// Sign convention: pidUpdate() works in the SAME frame as kiwiMix()'s
// WheelPowers — i.e. BEFORE the DIR_x_CW wiring correction. main.ino
// still applies DIR_x_CW when handing powers to the motors.
// ============================================================

#include <Arduino.h>
#include <PID_v1.h>       // Arduino PID library — install via Library Manager
#include "kinematics.h"   // for WheelPowers

// Construct the three wheel PID controllers with gains from
// robot_config.h and snapshot the encoders as the starting point for
// speed measurement. Call once from setup(), AFTER encoderInit().
void pidInit();

// Close the loop for all three wheels.
//   target : the open-loop normalized powers from kiwiMix() (pre-DIR).
// Returns the corrected normalized powers (still pre-DIR) to send to the
// motors. Call every loop(); the library's Compute() self-gates on the
// configured sample time, so calling it at full loop speed is correct.
WheelPowers pidUpdate(WheelPowers target);

// Clear all three controllers' integral state and re-baseline the
// encoder snapshot. Call whenever the motors go disabled -> enabled
// (e.g. while !rcEnabled()) so wound-up integral can't lurch the wheels
// the moment we re-arm.
void pidResetAll();

// Last measured wheel speed (mm/s), pre-DIR frame. Diagnostics only.
float pidMeasuredSpeed(char motor);   // motor = 'A', 'B', or 'C'

#endif // PID_H
