#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

// ============================================================
// navigation.h — blocking autonomous demo: straight -> turn -> straight
//
// A minimal open-loop-distance / IMU-heading autonomous sequence:
//   1. drive straight for NAV_DRIVE_DISTANCE_MM (default: 2 feet)
//   2. turn in place by NAV_TURN_ANGLE_DEG (default: 90 deg, CCW+)
//   3. drive straight again for NAV_DRIVE_DISTANCE_MM
//
// Distance comes from the A/B wheel encoders (the front wheel C carries
// no load during pure forward/back motion in this kiwi layout, so it
// isn't used for distance). Heading comes from the IMU, using the same
// "P + minimum-torque floor + close-enough tolerance" recipe as the RC
// heading-hold trim in imu.cpp, so it doesn't stall/buzz at low error
// the way a pure-P controller does.
//
// This BLOCKS the caller until the sequence finishes or aborts — same
// style as startupSequence.cpp's calibration flow. It re-polls the RC
// link and E-stop every iteration of its internal loops so a real E-stop
// press, a lost RC link, or the pilot disarming still cuts the motors
// immediately; the hardware E-stop button is independent of all of this
// (its ISR kills the motors directly) and works even during this call.
//
// Call only while armed (rcEnabled()). main.ino triggers this from CH2
// (left stick U/D, pushed to the top) or 'n' over Serial as a bench-test
// fallback. All tuning constants are in robot_config.h under
// "AUTONOMOUS NAVIGATION DEMO".
// ============================================================

// Runs the full straight -> turn -> straight sequence once.
// Returns true if it completed; false if it aborted early (E-stop,
// failsafe, disarm, or IMU unhealthy) — motors are already stopped
// by the time this returns either way.
bool navRunDemoSequence();

#endif // NAVIGATION_H
