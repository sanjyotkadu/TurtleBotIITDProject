#include "navigation.h"
#include "robot_config.h"
#include "kinematics.h"
#include "motorControl.h"
#include "encoder.h"
#include "imu.h"
#include "PID.h"
#include "RCControl.h"
#include "emergencyStop.h"
#include <math.h>

static float wrapPi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

// Mirrors driveturBot()'s pipeline in main.ino (kinematics -> PID ->
// DIR-corrected motor output) so autonomous moves go through the exact
// same path RC-driven ones do. Duplicated rather than shared because
// main.ino's driveturBot() is intentionally private to the sketch (see
// its header comment) and this is the only other caller.
static void navDrive(float Vx, float Vy, float Omega) {
  WheelPowers wp = kiwiMix(Vx, Vy, Omega);
  wp = pidUpdate(wp);   // closed-loop wheel speeds - keeps A/B tracking evenly, which "drive straight" depends on
  driveMotor(EN_C, IN3_C, IN4_C, wp.front_C     * DIR_C_CW);
  driveMotor(EN_A, IN1,   IN2,   wp.backLeft_A  * DIR_A_CW);
  driveMotor(EN_B, IN3,   IN4,   wp.backRight_B * DIR_B_CW);
}

// Re-polls the safety-critical inputs every iteration of the blocking
// loops below. Returns true if the sequence must stop right now.
static bool navShouldAbort() {
  rcUpdate();
  estopUpdate();
  if (estopActive())  return true;   // E-stop latched
  if (rcFailsafe())    return true;   // RC link lost
  if (!rcEnabled())     return true;   // pilot disarmed / switch off
  return false;
}

// Drive straight (current heading held via a fresh IMU lock) until the
// A/B encoders show distanceMM covered. C isn't used for distance: in
// this kiwi layout, front_C = -Vx + Omega, so during pure forward/back
// motion (Vx=0, tiny Omega trim) it carries essentially no travel.
static bool navDriveStraight(float distanceMM, float speed) {
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

    float distA = fabs((encoderCount('A') - startA) * MOTOR_A_MM_PER_COUNT);
    float distB = fabs((encoderCount('B') - startB) * MOTOR_B_MM_PER_COUNT);
    float traveled = (distA + distB) * 0.5f;

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
static bool navSettle(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (navShouldAbort()) {
      stopAllMotors();
      Serial.println("[NAV] aborted during settle pause.");
      return false;
    }
    imu_update();
  }
  return true;
}

bool navRunDemoSequence() {
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
