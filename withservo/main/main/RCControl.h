#ifndef RC_CONTROL_H
#define RC_CONTROL_H

// ============================================================
// RCControl.h — FlySky RC link + arming state machine
//
// Wraps IBusReader and turns the raw transmitter signal into the
// high-level pieces the rest of the robot cares about:
//   - normalized stick values (Vx, Vy, Omega), deadzone applied
//   - stick-center calibration + continuous drift tracking
//   - the arm/disarm state machine (kill switch + safety gating)
//   - failsafe (RC link lost) detection
//
// The IBusReader instance and all calibration/arm state live inside
// RCControl.cpp so nothing else has to know about the iBUS protocol.
// ============================================================

#include <Arduino.h>

// One-time setup: start the iBUS serial link. Call from setup().
void rcInit();

// Blocking one-second calibration of the resting stick centers.
// Call once, after a valid iBUS link has been established.
void rcCalibrateCenters();

// Per-loop tick. Consumes iBUS bytes, updates failsafe/arm state and
// (while disarmed) slowly tracks stick-center drift. Call every loop().
void rcUpdate();

// --- state queries (valid after rcUpdate() has run) ---
bool rcFailsafe();        // true = no fresh RC frame recently (link lost)
bool rcArmed();           // true = armed via kill switch + safety checks
bool rcEnabled();         // true = armed && switch on && !failsafe
bool rcSticksCentered();  // true = all three control sticks at rest

// --- normalized control inputs, each -1.0 .. +1.0 (deadzone applied) ---
float rcVx();     // strafe   (right stick L/R)
float rcVy();     // forward  (right stick U/D)
float rcOmega();  // rotation (left stick L/R)

// --- raw values, for diagnostics/telemetry ---
uint16_t rcRawChannel(uint8_t ch);  // raw iBUS channel (~1000..2000)
float rcCenterVx();
float rcCenterVy();
float rcCenterOmega();

#endif // RC_CONTROL_H
