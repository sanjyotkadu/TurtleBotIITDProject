#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

// ============================================================
// imu.h — BNO085 heading interface (yaw only, for navigation)
//
// Wraps the Adafruit BNO08x driver. Heading is CCW-positive and
// wrapped to -PI..PI, matching the kinematics' angle convention.
// Call imuUpdate() every loop() iteration.
// ============================================================

void  imuInit();          // bring up BNO085 on Wire2, enable Game Rotation Vector
bool  imuUpdate();        // poll sensor; true if a fresh heading sample arrived
float imuHeadingRad();    // latest yaw, radians, wrapped -PI..PI, CCW positive
float imuHeadingDeg();    // same, in degrees -180..180
bool  imuHealthy();       // true if a sample arrived within IMU_TIMEOUT_MS
void  imuZeroHeading();   // define the current orientation as heading 0

#endif // IMU_H