#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// ============================================================
// imu.h — BNO085 heading interface (yaw only, for navigation)
//
// Wraps the Adafruit BNO08x driver. Heading is CCW-positive and
// wrapped to -PI..PI, matching the kinematics' angle convention.
// Call imu_update() every loop() iteration.
// ============================================================

void  imu_init();          // bring up BNO085 on Wire2, enable Game Rotation Vector
bool  imu_update();        // poll sensor; true if a fresh heading sample arrived
float imu_headingRad();    // latest yaw, radians, wrapped -PI..PI, CCW positive
float imu_headingDeg();    // same, in degrees -180..180
bool  imu_healthy();       // true if a sample arrived within IMU_TIMEOUT_MS
void  imu_zeroHeading();   // define the current orientation as heading 0

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

#endif // IMU_H