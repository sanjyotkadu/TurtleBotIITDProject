#include "navigation.h"
#include "robot_config.h"
#include "kinematics.h"
#include "motorControl.h"
#include "encoder.h"
#include "imu.h"
#include "PID.h"
#include "RCControl.h"
#include "emergencyStop.h"
#include "obstacleAvoid.h"
#include "statusIndicator.h"
#include <math.h>

static float wrapPi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

// Set true only while a sequence launched from CH2 (the nav trigger
// stick) is running; then releasing the stick back below
// NAV_TRIGGER_THRESHOLD aborts immediately and hands control back to the
// joystick. Left false for Serial-triggered runs ('n'/'w') since CH2 has
// no spring return and just sits wherever the pilot last left it -
// gating those on it would abort them the instant they started. Set at
// the top of navRunDemoSequence()/navRunWaypointSequence() on every
// call, so it can't carry a stale value from a previous run.
static bool s_abortOnTriggerRelease = false;

// During a pure straight leg (Vx=0), kiwiMix() gives the A/B wheels a
// command of only (sqrt(3)/2)*Vy each - i.e. each wheel physically rolls
// cos(30 deg) as far as the robot's chassis actually travels, since they
// sit 30 deg off the direction of travel (front wheel C carries none of
// it at all). navDriveStraight() below divides the wheel-rolled distance
// by this same factor to recover the robot's real ground distance -
// without it, the loop keeps going until the WHEELS log the full
// requested distance, by which point the chassis has gone ~1/0.866 =
// 15.5% farther than asked.
static const float NAV_WHEEL_TRAVEL_RATIO = 0.8660254f;   // cos(30 deg) = sqrt(3)/2, matches kiwiMix()'s Vy coefficient

// Shared with obstacleAvoid.cpp (see navigation.h) so both places convert
// encoder counts to ground distance identically instead of a second copy
// of the NAV_WHEEL_TRAVEL_RATIO projection.
float navEncoderDistanceMM(long startA, long startB) {
  float distA = fabs((encoderCount('A') - startA) * MOTOR_A_MM_PER_COUNT);
  float distB = fabs((encoderCount('B') - startB) * MOTOR_B_MM_PER_COUNT);
  float wheelTraveled = (distA + distB) * 0.5f;
  return wheelTraveled / NAV_WHEEL_TRAVEL_RATIO;
}

// Mirrors driveturBot()'s pipeline in main.ino (kinematics -> PID ->
// DIR-corrected motor output) so autonomous moves go through the exact
// same path RC-driven ones do. Duplicated rather than shared because
// main.ino's driveturBot() is intentionally private to the sketch (see
// its header comment). Not `static` - obstacleAvoid.cpp's continuous
// drive loop calls this directly too (see navigation.h).
void navDrive(float Vx, float Vy, float Omega) {
  WheelPowers wp = kiwiMix(Vx, Vy, Omega);
  wp = pidUpdate(wp);   // closed-loop wheel speeds - keeps A/B tracking evenly, which "drive straight" depends on
  driveMotor(EN_C, IN3_C, IN4_C, wp.front_C     * DIR_C_CW);
  driveMotor(EN_A, IN1,   IN2,   wp.backLeft_A  * DIR_A_CW);
  driveMotor(EN_B, IN3,   IN4,   wp.backRight_B * DIR_B_CW);
}

// Re-polls the safety-critical inputs every iteration of the blocking
// loops below. Returns true if the sequence must stop right now.
bool navShouldAbort() {
  rcUpdate();
  estopUpdate();
  if (estopActive())  return true;   // E-stop latched
  if (rcFailsafe())    return true;   // RC link lost
  if (!rcEnabled())     return true;   // pilot disarmed / switch off
  if (s_abortOnTriggerRelease &&
      rcRawChannel(IBUS_CH_NAV_TRIGGER) <= NAV_TRIGGER_THRESHOLD) {
    return true;   // pilot pulled CH2 back down - hand control back to the joystick now
  }
  return false;
}

// Drive straight (current heading held via a fresh IMU lock) until the
// A/B encoders show distanceMM covered. C isn't used for distance: in
// this kiwi layout, front_C = -Vx + Omega, so during pure forward/back
// motion (Vx=0, tiny Omega trim) it carries essentially no travel.
bool navDriveStraight(float distanceMM, float speed) {
  if (!imu_healthy()) {
    Serial.println("[NAV] IMU not healthy - aborting straight leg.");
    return false;
  }
  imu_resetHeadingHold();   // drop any stale lock so this leg latches to the CURRENT heading, not an old one

  long startA = encoderCount('A');
  long startB = encoderCount('B');
  unsigned long lastPrint = 0;

  while (true) {
    if (navShouldAbort()) {
      stopAllMotors();
      Serial.println("[NAV] straight leg aborted.");
      return false;
    }
    imu_update();
    statusIndicatorUpdate();

    // Project the wheel-rolled distance back to robot ground distance
    // (see NAV_WHEEL_TRAVEL_RATIO above) so this compares correctly
    // against distanceMM instead of overshooting by ~15.5%.
    float traveled = navEncoderDistanceMM(startA, startB);

    if (millis() - lastPrint >= 150) {
      lastPrint = millis();
      Serial.print("[NAV] straight: ");
      Serial.print(traveled, 0);
      Serial.print(" / ");
      Serial.print(distanceMM, 0);
      Serial.println(" mm");
    }

    if (traveled >= distanceMM) break;

    float omega = imu_applyHeadingHold(0.0f);   // trim to stay on the heading latched at the top of this leg
    navDrive(0.0f, speed, omega);
  }

  stopAllMotors();
  return true;
}

// Turn in place by angleDeg (CCW+, matching the IMU's heading convention)
// relative to whatever heading we're at right now. Same P + floor +
// tolerance shape as imu_applyHeadingHold(), but against a one-shot
// absolute target instead of a re-latching trim target.
static bool navTurn(float angleDeg) {
  if (!imu_healthy()) {
    Serial.println("[NAV] IMU not healthy - aborting turn.");
    return false;
  }

  float targetRad = wrapPi(imu_headingRad() + angleDeg * (float)M_PI / 180.0f);
  unsigned long settledSinceMs = 0;
  unsigned long lastPrint = 0;

  while (true) {
    if (navShouldAbort()) {
      stopAllMotors();
      Serial.println("[NAV] turn aborted.");
      return false;
    }
    imu_update();
    statusIndicatorUpdate();

    float error    = wrapPi(targetRad - imu_headingRad());
    float errorDeg = fabs(error) * 180.0f / (float)M_PI;

    if (millis() - lastPrint >= 150) {
      lastPrint = millis();
      Serial.print("[NAV] turn: heading ");
      Serial.print(imu_headingDeg(), 1);
      Serial.print(" target ");
      Serial.print(targetRad * 180.0f / (float)M_PI, 1);
      Serial.print(" error ");
      Serial.println(errorDeg, 1);
    }

    float correction;
    if (errorDeg < NAV_TURN_TOLERANCE_DEG) {
      // Within tolerance - require it to stay there for NAV_SETTLE_MS
      // before declaring the turn done, same "don't buzz at the edge"
      // reasoning as heading-hold's tolerance cutoff.
      if (settledSinceMs == 0) settledSinceMs = millis();
      if (millis() - settledSinceMs >= NAV_SETTLE_MS) break;
      correction = 0.0f;
    } else {
      settledSinceMs = 0;
      correction = NAV_TURN_KP * error;
      float mag = fabs(correction);
      if (mag < NAV_TURN_MIN_OMEGA) mag = NAV_TURN_MIN_OMEGA;
      if (mag > NAV_TURN_MAX_OMEGA) mag = NAV_TURN_MAX_OMEGA;
      correction = (error >= 0.0f) ? mag : -mag;
    }

    navDrive(0.0f, 0.0f, correction);
  }

  stopAllMotors();
  return true;
}

// Pause between legs WITHOUT a blind delay(): a plain delay() never
// calls imu_update(), so if the pause is longer than IMU_TIMEOUT_MS the
// IMU looks "unhealthy" the instant the next leg checks it and that leg
// aborts immediately - i.e. the robot drives leg 1, pauses, and just
// stops, never turning. This keeps sampling (and safety-polling) through
// the pause instead.
bool navSettle(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (navShouldAbort()) {
      stopAllMotors();
      Serial.println("[NAV] aborted during settle pause.");
      return false;
    }
    imu_update();
    statusIndicatorUpdate();
  }
  return true;
}

bool navRunDemoSequence(bool abortIfTriggerReleased) {
  s_abortOnTriggerRelease = abortIfTriggerReleased;

  if (!rcEnabled()) {
    Serial.println("[NAV] must be armed before running the demo sequence.");
    return false;
  }

  Serial.println("[NAV] starting demo: straight -> turn -> straight");
  pidResetAll();
  imu_resetHeadingHold();

  if (!navDriveStraight(NAV_DRIVE_DISTANCE_MM, NAV_DRIVE_SPEED)) return false;
  if (!navSettle(NAV_SETTLE_MS)) return false;

  if (!navTurn(NAV_TURN_ANGLE_DEG)) return false;
  if (!navSettle(NAV_SETTLE_MS)) return false;

  if (!navDriveStraight(NAV_DRIVE_DISTANCE_MM, NAV_DRIVE_SPEED)) return false;

  Serial.println("[NAV] demo sequence complete.");
  return true;
}

bool navRunWaypointSequence(const NavWaypoint* waypoints, int count, bool abortIfTriggerReleased) {
  s_abortOnTriggerRelease = abortIfTriggerReleased;

  if (!rcEnabled()) {
    Serial.println("[NAV] must be armed before running a waypoint sequence.");
    return false;
  }
  if (waypoints == nullptr || count <= 0) {
    Serial.println("[NAV] waypoint list is empty.");
    return false;
  }
  if (!imu_healthy()) {
    Serial.println("[NAV] IMU not healthy - aborting waypoint sequence.");
    return false;
  }

  Serial.print("[NAV] starting waypoint sequence: ");
  Serial.print(count);
  Serial.println(" waypoint(s)");
  obstacleClearAlert(); // start with the indicator off regardless of how a previous run ended
  pidResetAll();
  imu_resetHeadingHold();

  // Dead-reckoned pose: (0,0) is wherever the robot is right now, and the
  // frame's angle reference is whatever heading the IMU currently calls
  // zero. Each leg below only trusts the encoders' *distance* and the
  // heading it just turned to — there's no absolute position sensor, so
  // this drifts over a long run just like any odometry estimate, but is
  // plenty for a handful of chained waypoints.
  float x = 0.0f, y = 0.0f;

  // Small LIFO stack of TEMPORARY waypoints inserted ahead of the real
  // list when an obstacle forces a detour: the most recently pushed one
  // is targeted first, so multiple obstacles in a row just stack up
  // instead of losing track of the real destination underneath them.
  // Popped (not the real list) whenever one is reached, so the real
  // waypoint below it is naturally retried next, from wherever the
  // detour left off.
  const int MAX_DETOUR_DEPTH = 4;
  NavWaypoint detourStack[MAX_DETOUR_DEPTH];
  int detourCount = 0;

  int nextIndex = 0;
  while (nextIndex < count || detourCount > 0) {
    if (navShouldAbort()) {
      stopAllMotors();
      obstacleClearAlert(); // don't leave the indicator lit if this ends mid-detour
      Serial.println("[NAV] waypoint sequence aborted.");
      return false;
    }

    bool isDetour = (detourCount > 0);
    NavWaypoint target = isDetour ? detourStack[detourCount - 1] : waypoints[nextIndex];

    float dx = target.x_mm - x;
    float dy = target.y_mm - y;
    float distance = sqrtf(dx * dx + dy * dy);

    Serial.print(isDetour ? "[NAV] detour target: (" : "[NAV] waypoint target: (");
    Serial.print(target.x_mm, 0);
    Serial.print(", ");
    Serial.print(target.y_mm, 0);
    Serial.print(") from (");
    Serial.print(x, 0);
    Serial.print(", ");
    Serial.print(y, 0);
    Serial.print(") dist=");
    Serial.println(distance, 0);

    if (distance < NAV_WAYPOINT_TOLERANCE_MM) {
      Serial.println("[NAV] already within tolerance - skipping.");
      obstacleClearAlert(); // this leg counts as done - don't leave the LED lit past it
      if (isDetour) detourCount--; else nextIndex++;
      continue;
    }

    // Go-to-goal: face the goal (atan2 of the remaining vector), then
    // drive toward it - now with obstacle avoidance, instead of a plain
    // blind navDriveStraight() (see obstacleAvoid.h).
    float targetHeadingRad = atan2f(dy, dx);
    float turnDeg = wrapPi(targetHeadingRad - imu_headingRad()) * 180.0f / (float)M_PI;

    // Drop any PID/heading-hold state carried over from the previous leg
    // so a fresh waypoint doesn't inherit stale integral/derivative terms.
    pidResetAll();
    imu_resetHeadingHold();

    if (!navTurn(turnDeg))          return false;
    if (!navSettle(NAV_SETTLE_MS))  return false;

    float distanceDriven = 0.0f;
    ObstacleDriveResult driveResult =
        driveStraightWithObstacleCheck(distance, NAV_DRIVE_SPEED, distanceDriven);

    // Advance the dead-reckoned pose by what was ACTUALLY driven this
    // leg (not the full commanded `distance`- a block can stop it early).
    x += distanceDriven * cosf(targetHeadingRad);
    y += distanceDriven * sinf(targetHeadingRad);

    if (driveResult == OBSTACLE_DRIVE_ABORTED) return false;

    if (driveResult == OBSTACLE_DRIVE_BLOCKED) {
      // Something's in the way - scan for an opening and route around
      // it with a temporary waypoint instead of giving up on the real
      // target. Keeps rescanning if boxed in on both sides rather than
      // aborting the whole sequence; navSettle() during the wait still
      // polls navShouldAbort(), so CH2-down/E-stop/disarm/failsafe still
      // breaks out of this immediately.
      Serial.println("[NAV] obstacle ahead - scanning for an opening...");
      float relDeg;
      while (!obstacleFindOpening(relDeg)) {
        Serial.println("[NAV] blocked on all sides - waiting, then rescanning...");
        if (!navSettle(OBSTACLE_RETRY_WAIT_MS)) { obstacleClearAlert(); return false; }
      }

      float detourHeadingRad = imu_headingRad() + relDeg * (float)M_PI / 180.0f;
      NavWaypoint detour;
      detour.x_mm = x + OBSTACLE_AVOID_LEG_MM * cosf(detourHeadingRad);
      detour.y_mm = y + OBSTACLE_AVOID_LEG_MM * sinf(detourHeadingRad);

      if (detourCount >= MAX_DETOUR_DEPTH) {
        Serial.println("[NAV] too many nested detours - giving up.");
        stopAllMotors();
        obstacleClearAlert();
        return false;
      }
      detourStack[detourCount++] = detour;
      Serial.print("[NAV] inserting temporary waypoint toward the opening: (");
      Serial.print(detour.x_mm, 0);
      Serial.print(", ");
      Serial.print(detour.y_mm, 0);
      Serial.println(")");
      // Don't pop/advance - the target we were just aiming for (detour
      // or real) is retried right after this new one is reached.
      continue;
    }

    if (!navSettle(NAV_SETTLE_MS)) return false;

    // Reached this target (detour or real) - pop/advance to the next one.
    if (isDetour) detourCount--; else nextIndex++;
  }

  Serial.println("[NAV] waypoint sequence complete.");
  return true;
}
