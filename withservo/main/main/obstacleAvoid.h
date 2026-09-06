#ifndef OBSTACLE_AVOID_H
#define OBSTACLE_AVOID_H

#include <Arduino.h>

// ============================================================
// obstacleAvoid.h — SG90 servo + HC-SR04 sensing, used by navigation.cpp's
//                   waypoint follower to dodge obstacles by inserting a
//                   temporary waypoint toward whichever opening it finds
//                   (see robot_config.h's OBSTACLE AVOIDANCE section for
//                   the tuning knobs, and obstacleAvoidTest/ for the
//                   standalone scan logic this is built on).
//
// Bring up with obstacleAvoidInit() once from setup(), after imu_init()
// and motorsInit(). The CH2 arm/trigger mechanism in main.ino is
// untouched by this module - it only supplies the two primitives below,
// which navRunWaypointSequence() (navigation.cpp) calls into.
// ============================================================

// Attaches the servo, arms the ultrasonic + LED pins, and centers the
// servo to 90 deg (straight ahead). Call once from setup().
void obstacleAvoidInit();

// Turns the obstacle-present indicator LED (OBSTACLE_LED_PIN) off.
// driveStraightWithObstacleCheck() already does this itself whenever it
// completes a leg without being blocked (see below) or aborts - call
// this directly only from a spot that skips that function entirely, e.g.
// navigation.cpp's "already within tolerance" case, so the LED can't
// stay lit past a point where a waypoint sequence considers itself done
// with the leg that turned it on.
void obstacleClearAlert();

enum ObstacleDriveResult {
  OBSTACLE_DRIVE_REACHED,  // the full distanceMM was covered
  OBSTACLE_DRIVE_BLOCKED,  // stopped early - something within OBSTACLE_TRIGGER_CM, caller decides what's next
  OBSTACLE_DRIVE_ABORTED,  // navShouldAbort() tripped (disarm/E-stop/failsafe/CH2 released)
};

// Drives straight (current heading held, same as navDriveStraight()) up
// to distanceMM, checking the front-facing HC-SR04 (servo centered at 90
// deg) every OBSTACLE_CHECK_CHUNK_MM. Reacts in two tiers along the way,
// closest first (see robot_config.h for the exact cm values):
//   <= OBSTACLE_EMERGENCY_CM : too close to safely turn in place - stops,
//       reverses OBSTACLE_BACKUP_MM, then keeps trying for the same
//       target (this tier is handled internally and never counts as
//       "blocked" - it's a flinch, not a direction change).
//   <= OBSTACLE_TRIGGER_CM : stops and returns OBSTACLE_DRIVE_BLOCKED
//       immediately - finding a new direction is the CALLER's job (see
//       obstacleFindOpening() below), since only it knows how to turn
//       that into a new waypoint.
//   <= OBSTACLE_ALERT_CM : just a warning (print + quick LED blink),
//       keeps driving - not reported back to the caller.
// distanceDrivenMM is always set to how far it actually got (accurate
// even on an early BLOCKED/ABORTED return), so the caller can update its
// own dead-reckoned position before deciding what to do next.
ObstacleDriveResult driveStraightWithObstacleCheck(float distanceMM, float speed, float &distanceDrivenMM);

// Glides the servo LEFT then RIGHT from center looking for an opening
// (SCAN_CONFIRM_STEPS consecutive clear check-points before trusting
// one - see robot_config.h). On success, returns true and sets
// outRelativeDegCCW to the heading offset (CCW+, degrees, ready to hand
// straight to navTurn()) to turn the CHASSIS by to face the opening.
// Returns false if blocked on both sides - the caller decides whether
// to wait and retry or give up.
bool obstacleFindOpening(float &outRelativeDegCCW);

#endif // OBSTACLE_AVOID_H
