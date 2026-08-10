#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// ============================================================
// imu.h — BNO085 heading interface (mounting-orientation-agnostic)
//
// Wraps the Adafruit BNO08x driver. Every heading/roll/pitch sample is
// computed as a quaternion DELTA from a captured reference pose (see
// imu_zeroHeading()) instead of being read straight off the raw sensor
// axes. That makes heading correct no matter how the BNO085 board is
// physically mounted on the chassis (flat, tilted, sideways — anything
// rigid) — the fixed sensor-to-chassis mounting offset cancels out
// exactly in the delta. The earlier version extracted yaw directly from
// the raw quaternion, which only worked if the board happened to be
// mounted dead flat; tilting the robot then leaked into "yaw" and
// heading-hold would spin the robot chasing a disturbance that never
// actually happened.
//
// IMPORTANT: the reference pose is auto-captured on the first sample
// after imu_init(), and re-capturable any time via imu_zeroHeading().
// Do that while the robot is STATIONARY and settled in the orientation
// you want to call "home" (ideally level) — everything reported here is
// relative to that pose. Heading is CCW-positive and wrapped to
// -PI..PI, matching the kinematics' angle convention. Call imu_update()
// every loop() iteration.
// ============================================================

void  imu_init();          // bring up BNO085 on Wire2, enable Game Rotation Vector
bool  imu_update();        // poll sensor; true if a fresh heading sample arrived
float imu_headingRad();    // yaw since the reference pose, radians, -PI..PI, CCW+
float imu_headingDeg();    // same, in degrees -180..180
float imu_rollDeg();       // roll since the reference pose, degrees — diagnostic only
float imu_pitchDeg();      // pitch since the reference pose, degrees — diagnostic only
bool  imu_healthy();       // true if a sample arrived within IMU_TIMEOUT_MS

// Re-anchor the reference pose at the CURRENT orientation, so heading
// (and roll/pitch) read ~0 right now. Call while the robot is stationary
// and settled in whatever orientation should count as "home" — e.g.
// right after placing it down level, before arming.
void  imu_zeroHeading();

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
