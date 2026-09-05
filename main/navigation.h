#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

// ============================================================
// navigation.h — blocking autonomous demo: straight -> turn -> straight,
//                plus a go-to-goal waypoint follower built on the same
//                turn/drive primitives.
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
// Call only while armed (rcEnabled()). 'n' over Serial runs this as a
// bench-test fallback. All tuning constants are in robot_config.h under
// "AUTONOMOUS NAVIGATION DEMO".
//
// --- Waypoint follower (go-to-goal, chained) ---
//
// navRunWaypointSequence() drives through a list of (x, y) points, one
// at a time: for each point it turns to face it (navTurn) then drives
// straight to it (navDriveStraight) — exactly the "turn, then drive"
// go-to-goal recipe, just repeated over a list instead of one fixed
// angle/distance. There's no absolute position sensor on this robot, so
// the position used to aim at each waypoint is dead-reckoned from the
// distance driven and the heading turned to on each leg; like any
// odometry estimate it drifts over a long path, but is accurate enough
// for a handful of waypoints on the bench/floor.
//
// main.ino wires CH2 (left stick U/D, no spring return) to this: pushed
// to the top, it launches navRunWaypointSequence() over a hardcoded demo
// list; pulled back down mid-run, it aborts that run immediately and
// hands control straight back to the joystick (see abortIfTriggerReleased
// below) — no need to reach for a laptop/Serial once the RC link is the
// only thing connected. 'w' over Serial runs the same waypoint list as a
// bench-test fallback, and 'n' still runs the plain demo sequence. This
// CH2 arm/abort mechanism is untouched by obstacle avoidance below — it
// only ever gets *set* here, at the top of this function, exactly as
// before.
//
// Each leg's straight-line portion now runs through
// driveStraightWithObstacleCheck() (obstacleAvoid.h) instead of a plain
// blind navDriveStraight(): if it stops early because something is in
// the way, this function scans for an opening (obstacleFindOpening())
// and inserts a TEMPORARY waypoint toward it ahead of the real target,
// then continues the same loop - so the real destination is retried
// (from the new position) right after the temporary one is reached,
// rather than the detour being a dead end.
// ============================================================

// One 2D goal, in millimetres, relative to wherever the waypoint
// sequence started (dead-reckoned — see navRunWaypointSequence() above).
struct NavWaypoint {
  float x_mm;
  float y_mm;
};

// --- Primitives shared with obstacleAvoid.cpp ---
//
// Pure visibility change (these were `static` i.e. private to this file)
// so obstacleAvoid.cpp's driveStraightWithObstacleCheck() can reuse the
// exact same abort-polling/drive-straight code navRunWaypointSequence()
// runs on, instead of a second copy of it. No behavior changes, and
// nothing outside this file sets s_abortOnTriggerRelease (the CH2-release
// abort flag) - only navRunDemoSequence()/navRunWaypointSequence() do
// that, once each, at their own entry, same as always.

// Re-polls RC link / E-stop / arm state (and the CH2-release trigger, if
// a nav*Sequence() call is currently running with abortIfTriggerReleased
// = true). Returns true if the caller must stop right now.
bool navShouldAbort();

// Pause for `ms` while continuing to poll navShouldAbort() and imu_update().
bool navSettle(unsigned long ms);

// Drive straight (current heading held) until the A/B encoders show
// distanceMM covered, at the given -1..1 speed.
bool navDriveStraight(float distanceMM, float speed);

// Runs the full straight -> turn -> straight sequence once.
//   abortIfTriggerReleased : if true, releasing CH2 (the nav trigger
//     stick) back below NAV_TRIGGER_THRESHOLD aborts immediately, same
//     as an E-stop/failsafe/disarm. Only pass true when this call was
//     itself launched by CH2 going high — leave false (default) for
//     Serial-triggered runs, since CH2 has no spring return and may
//     already be resting below the threshold.
// Returns true if it completed; false if it aborted early (E-stop,
// failsafe, disarm, trigger release, or IMU unhealthy) — motors are
// already stopped by the time this returns either way.
bool navRunDemoSequence(bool abortIfTriggerReleased = false);

// Visits each waypoint in `waypoints` in order (turn to face it, drive
// straight to it), stopping once the last one is reached.
//   abortIfTriggerReleased : same meaning as navRunDemoSequence()'s.
// Returns true if every waypoint was reached; false if it aborted early
// (E-stop, failsafe, disarm, trigger release, IMU unhealthy, or an
// empty/null list) — motors are already stopped by the time this
// returns either way.
bool navRunWaypointSequence(const NavWaypoint* waypoints, int count,
                             bool abortIfTriggerReleased = false);

#endif // NAVIGATION_H
