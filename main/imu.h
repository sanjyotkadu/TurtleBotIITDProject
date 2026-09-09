#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// ============================================================
// imu.h — BNO085 orientation interface (yaw for navigation,
//         pitch/roll for chassis-tip reporting)
//
// Wraps the Adafruit BNO08x driver. Heading is CCW-positive and
// wrapped to -PI..PI, matching the kinematics' angle convention.
// Pitch/roll are reporting-only: this is a wheeled chassis, so
// there's no actuator to correct a physical tilt — imu_isTipped()
// exists to flag "robot is on its side", not to drive a response.
// Call imu_update() every loop() iteration.
// ============================================================

void  imu_init();          // bring up BNO085 on Wire2, enable Game Rotation Vector
bool  imu_update();        // poll sensor; true if a fresh heading sample arrived
float imu_headingRad();    // latest yaw, radians, wrapped -PI..PI, CCW positive
float imu_headingDeg();    // same, in degrees -180..180
bool  imu_healthy();       // true if a sample arrived within IMU_TIMEOUT_MS
uint32_t imu_resetCount(); // count of BNO085 resets detected since imu_init() (debug)

// Minimal one-line print — "heading: X.X deg" — same format as
// imuTest.ino's stream output, so you can watch this alongside the main
// debugPrint() line and confirm both agree on the current angle.
void imu_printHeading();
void  imu_zeroHeading();   // define the current orientation as heading 0

// --- pitch/roll (chassis tilt) — reporting only, wheels can't correct these ---

float imu_pitchDeg();      // nose-up/down tilt, degrees
float imu_rollDeg();       // side-to-side tilt, degrees
bool  imu_isTipped();      // true if pitch or roll exceeds IMU_TIP_THRESHOLD_DEG

// --- heading-hold trim (called from main.ino's drive loop) ---

// Resist yaw drift while translating with the rotation stick centered.
// Releases immediately on any pilot rotation input and re-latches once
// the stick returns to center. Fails open (passes Omega through
// unchanged) if the IMU hasn't produced a fresh sample recently.
float imu_applyHeadingHold(float omegaCmd);

// Drop the heading-hold lock so the next call to imu_applyHeadingHold()
// re-latches a fresh target heading. Call whenever the robot goes
// disabled -> enabled (mirrors pidResetAll()).
void imu_resetHeadingHold();

// --- heading-hold telemetry (debug only, updated each imu_applyHeadingHold() call) ---

bool  imu_headingHoldLocked();       // true if a target heading is currently latched
float imu_headingHoldTargetDeg();    // latched target heading, degrees -180..180
float imu_headingHoldErrorDeg();     // wrapped (target - current), degrees, signed
float imu_headingHoldCorrection();   // last Omega correction actually returned (post-clamp)

#endif // IMU_H