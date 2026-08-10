#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

// ============================================================
// navigation.h — Scripted step sequencer ("pre-coordinate" autopilot)
//
// Runs the robot through a fixed ARRAY of steps, each specifying:
//   - duration (seconds) to translate for
//   - a body-frame translation direction (forward/backward/left/right)
//   - an ABSOLUTE target heading (degrees) to reach and hold, using the
//     IMU — independent of the translation direction, because a kiwi/
//     holonomic drive can strafe without turning to face that way.
//
// Each step runs in two phases:
//   1. TURN  — rotate in place (no translation) until heading is within
//              NAV_HEADING_TOLERANCE_DEG of the step's target, or
//              NAV_TURN_TIMEOUT_MS elapses (whichever comes first).
//   2. DRIVE — translate in the step's direction for its duration,
//              while a P-controller keeps nudging Omega toward the
//              target heading the whole time (so drift during the
//              drive still gets corrected).
//
// This is TIME-based dead reckoning only — no position/coordinate
// tracking yet (no encoder distance, no X/Y). That's the deliberate
// scope for this stage; this is the natural place to add real
// coordinate-based navigation later (swap the DRIVE phase's completion
// test for an encoder-distance target, add X/Y tracking on top).
//
// Reuses the same modules as main/straightLineTest: kinematics, PID,
// motorControl, imu. Call navInit() once, after imu_init()/pidInit().
// The step array you pass to navLoadProgram() must stay alive (e.g.
// static/global) for as long as the program might run.
// ============================================================

enum NavDir { NAV_FORWARD, NAV_BACKWARD, NAV_LEFT, NAV_RIGHT };

struct NavStep {
  float  durationSec;  // how long the DRIVE phase runs for this step
  NavDir dir;           // body-frame translation direction
  float  headingDeg;    // absolute target heading (relative to imu_zeroHeading()'s reference)
  float  speed;          // normalized drive speed 0..1 for this step; 0 = use NAV_DEFAULT_SPEED
};

void navInit();
void navLoadProgram(const NavStep *steps, int count);
void navStart();              // begin executing the loaded program from step 0
void navStop();               // abort immediately, motors off
bool navIsRunning();
int  navCurrentStepIndex();   // -1 if not running

// Call every loop() while a program might be running (no-op if not).
// Drives the motors directly, same shape as main.ino's driveturBot() —
// do NOT also call the normal RC/manual drive path while this runs.
void navUpdate();

#endif // NAVIGATION_H
