#include "obstacleAvoid.h"
#include "robot_config.h"
#include "navigation.h"
#include "motorControl.h"
#include "encoder.h"
#include "imu.h"
#include "statusIndicator.h"
#include <Servo.h>
#include <math.h>

// ---------------- servo ----------------

static Servo sg90;
static int   currentServoDeg = 90;

// Steps 1 deg at a time with a delay between steps instead of jumping
// straight to the target, so the glide is visibly slow/controlled
// instead of snapping at the SG90's own max speed - same approach
// validated in obstacleAvoidTest.ino.
//
// A full-range recenter (e.g. gliding back from a found opening at 140+
// deg to 90) takes 50+ steps * SERVO_SLEW_DELAY_MS - comfortably over
// IMU_TIMEOUT_MS (200ms, robot_config.h). Without polling imu_update()
// during that glide, imu_healthy() goes false partway through purely from
// not being fed, and whatever calls navTurn()/navDriveStraight() right
// after this returns aborts immediately with "IMU not healthy" even
// though the IMU itself is fine - this is what caused the waypoint
// sequence to abort right after successfully finding a detour opening.
// Same fix as glideAndCheck() already uses below.
static void servoMoveToSmooth(int targetDeg) {
  targetDeg = constrain(targetDeg, 0, 180);
  int step = (targetDeg >= currentServoDeg) ? SERVO_SLEW_STEP_DEG : -SERVO_SLEW_STEP_DEG;

  while (currentServoDeg != targetDeg) {
    currentServoDeg += step;
    if ((step > 0 && currentServoDeg > targetDeg) ||
        (step < 0 && currentServoDeg < targetDeg)) {
      currentServoDeg = targetDeg;
    }
    sg90.writeMicroseconds(map(currentServoDeg, 0, 180, SERVO_MIN_US, SERVO_MAX_US));
    delay(SERVO_SLEW_DELAY_MS);
    imu_update();
    statusIndicatorUpdate();
  }
}

// ---------------- ultrasonic ----------------

// Quick single ping for use WHILE the servo is gliding during a scan: a
// short timeout (SCAN_CHECK_TIMEOUT_US, ~100cm range) so a check barely
// pauses the glide. No-echo within that short window just means "nothing
// within useful range" - i.e. clear.
static float pingQuickCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long echoUs = pulseIn(ECHO_PIN, HIGH, SCAN_CHECK_TIMEOUT_US);
  if (echoUs == 0) return (float)SONAR_MAX_RANGE_CM;

  float cm = (echoUs * 0.0343f) / 2.0f;
  if (cm < SONAR_MIN_RANGE_CM) return 0.0f; // too close - treat as blocked
  return cm;
}

// A single quick ping can catch a noise glitch, so require two in a row
// above SCAN_CLEAR_CM before trusting "this direction is open". The gap
// between them matters: back-to-back HC-SR04 triggers with too little
// time between them can pick up the FIRST ping's own echo/reverb instead
// of a fresh reading - see OBSTACLE_PING_GAP_MS in robot_config.h.
static bool directionIsClear() {
  if (pingQuickCm() < SCAN_CLEAR_CM) return false;
  delay(OBSTACLE_PING_GAP_MS);
  return pingQuickCm() >= SCAN_CLEAR_CM;
}

// Takes OBSTACLE_CHECK_SAMPLES quick pings in a row and returns the
// CLOSEST (most conservative) one, in the same cm convention pingQuickCm()
// uses (near 0 = very close, SONAR_MAX_RANGE_CM = clear). A close reading
// only has to win once out of the samples to be trusted - deliberately
// biased toward "assume it's there" over "assume it's clear", because a
// single ping right at the edge of this sensor's usable range (see
// OBSTACLE_ALERT_CM/OBSTACLE_TRIGGER_CM in robot_config.h) can come back
// a spurious "no echo" even with something genuinely in front - missing
// the obstacle entirely on that one sample. Uses the same short timeout
// as the scan checks since these tiers only care about near-field range.
// OBSTACLE_PING_GAP_MS between samples avoids one ping catching the
// previous one's echo/reverb (see directionIsClear() above).
static float pingClosestCm(int samples) {
  float closest = (float)SONAR_MAX_RANGE_CM;
  for (int i = 0; i < samples; i++) {
    float cm = pingQuickCm();
    if (cm < closest) closest = cm;
    if (i < samples - 1) delay(OBSTACLE_PING_GAP_MS);
  }
  return closest;
}

// ---------------- obstacle LED alert ----------------

static void ledSet(bool on) {
  digitalWrite(OBSTACLE_LED_PIN, on ? HIGH : LOW);
}

// OBSTACLE_ALERT_BLINKS * 2 * OBSTACLE_ALERT_BLINK_MS worth of blocking
// delay (720ms at the defaults) - well past IMU_TIMEOUT_MS (200ms). This
// is called right before navDriveStraight() in the EMERGENCY tier
// (driveStraightWithObstacleCheck()), which opens with its own
// imu_healthy() check - same starvation bug as servoMoveToSmooth() above,
// so poll here too.
static void ledAlertBurst() {
  for (int i = 0; i < OBSTACLE_ALERT_BLINKS; i++) {
    ledSet(true);  delay(OBSTACLE_ALERT_BLINK_MS); imu_update();
    ledSet(false); delay(OBSTACLE_ALERT_BLINK_MS); imu_update();
  }
}

// ---------------- scan-for-opening (see obstacleAvoidTest.ino) ----------------

// Which physical servo angle is "LEFT" / "RIGHT" depends on SCAN_DIR_SIGN.
static int leftLimitDeg()  { return (SCAN_DIR_SIGN > 0) ? 180 : 0; }
static int rightLimitDeg() { return (SCAN_DIR_SIGN > 0) ? 0 : 180; }

// One continuous glide from the servo's CURRENT position to toDeg,
// checking for an opening every SCAN_STEP_DEG of travel. Requires
// SCAN_CONFIRM_STEPS consecutive clear check-points (any blocked
// reading resets the streak) before committing to "opening found" and
// stopping there - guards against a single narrow gap in the beam, or
// an obstacle angled enough to reflect the ping away, looking clear
// when the arc right after it isn't. If it reaches toDeg without ever
// reaching that streak, returns false and the servo is left at toDeg.
static bool glideAndCheck(int toDeg, int &stoppedAtDeg) {
  int fromDeg          = currentServoDeg;
  int dir              = (toDeg >= fromDeg) ? 1 : -1;
  int angle            = fromDeg;
  int lastCheckDeg     = fromDeg;
  int consecutiveClear = 0;

  while (angle != toDeg) {
    angle += dir * SERVO_SLEW_STEP_DEG;
    if ((dir > 0 && angle > toDeg) || (dir < 0 && angle < toDeg)) angle = toDeg;

    currentServoDeg = angle;
    sg90.writeMicroseconds(map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US));
    delay(SERVO_SLEW_DELAY_MS);
    imu_update(); // keep heading fresh while the glide is blocking
    statusIndicatorUpdate();

    if (abs(angle - lastCheckDeg) >= SCAN_STEP_DEG || angle == toDeg) {
      lastCheckDeg = angle;
      bool clear = directionIsClear();
      consecutiveClear = clear ? (consecutiveClear + 1) : 0;

      Serial.print("[AVOID]   check ");
      Serial.print(angle);
      Serial.print(" deg -> ");
      Serial.println(clear ? "clear" : "blocked");

      if (consecutiveClear >= SCAN_CONFIRM_STEPS) {
        stoppedAtDeg = angle;
        return true;
      }
    }
  }

  stoppedAtDeg = toDeg;
  return false;
}

// ---------------- public API ----------------

void obstacleAvoidInit() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(OBSTACLE_LED_PIN, OUTPUT);
  digitalWrite(OBSTACLE_LED_PIN, LOW);

  sg90.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  currentServoDeg = 90;
  sg90.writeMicroseconds(map(currentServoDeg, 0, 180, SERVO_MIN_US, SERVO_MAX_US));

  Serial.println("[AVOID] obstacle-avoidance servo/sonar ready, centered at 90 deg.");
}

void obstacleClearAlert() {
  ledSet(false);
  // Only ever called from within a running nav sequence (see the header
  // comment), so it's always safe to drop back to the "running normally"
  // pattern here - covers the "already within tolerance" skip case in
  // navigation.cpp, which doesn't go through driveStraightWithObstacleCheck()
  // at all and so wouldn't otherwise clear a STATUS_OBSTACLE left over from
  // the leg before it.
  statusIndicatorSetState(STATUS_NAV_RUNNING);
}

// Drives the whole leg as ONE continuous motion - a single heading-hold
// lock at the top, held the entire way, same as navDriveStraight(). This
// used to drive OBSTACLE_CHECK_CHUNK_MM at a time via repeated calls to
// navDriveStraight(), which stops the motors and re-latches a fresh
// heading-hold reference at the end of EVERY chunk. With
// OBSTACLE_CHECK_CHUNK_MM at 10mm that meant a full stop/restart roughly
// every centimetre of travel - each restart samples a slightly different,
// noisy heading right off a dead stop and corrects toward IT instead of
// the leg's real target heading, which is what showed up as the chassis
// zig-zagging instead of driving straight. It also meant every ultrasonic
// check happened right at a motor stop/start transition, where electrical
// noise and chassis vibration make the HC-SR04 reading least reliable -
// contributing to both missed obstacles and false "blocked" scans.
// Checking the sonar in-line on a continuous drive (below) fixes both.
ObstacleDriveResult driveStraightWithObstacleCheck(float distanceMM, float speed, float &distanceDrivenMM) {
  distanceDrivenMM = 0.0f;

  if (!imu_healthy()) {
    Serial.println("[AVOID] IMU not healthy - aborting straight leg.");
    return OBSTACLE_DRIVE_ABORTED;
  }

  servoMoveToSmooth(90);
  imu_resetHeadingHold();   // latch ONCE for the whole leg, not per chunk

  long  startA = encoderCount('A');
  long  startB = encoderCount('B');
  float drivenBeforeThisStretch = 0.0f;   // carries progress across an emergency-backup re-baseline
  float distanceAtLastCheck     = 0.0f;
  unsigned long lastRangePrint  = 0;

  while (true) {
    if (navShouldAbort()) {
      stopAllMotors();
      servoMoveToSmooth(90);
      ledSet(false);
      return OBSTACLE_DRIVE_ABORTED;
    }
    imu_update();
    statusIndicatorUpdate();

    float traveledThisStretch = navEncoderDistanceMM(startA, startB);
    distanceDrivenMM = drivenBeforeThisStretch + traveledThisStretch;
    if (distanceDrivenMM >= distanceMM) break;

    // Check the sonar every OBSTACLE_CHECK_CHUNK_MM of ACTUAL progress,
    // without stopping the motors to do it - the drive command below just
    // keeps running with whatever speed/omega was last computed while
    // pingClosestCm()'s pulseIn() calls block briefly for the echo.
    if (traveledThisStretch - distanceAtLastCheck >= OBSTACLE_CHECK_CHUNK_MM) {
      distanceAtLastCheck = traveledThisStretch;

      // Multiple quick samples, closest wins - see pingClosestCm() above
      // for why a single ping isn't trusted here.
      float cm = pingClosestCm(OBSTACLE_CHECK_SAMPLES);

      // Raw-range heartbeat, throttled to ~4/sec - lets you watch what the
      // sensor is actually reporting on Serial regardless of whether any
      // tier below fires. If this sits pinned at SONAR_MAX_RANGE_CM (400)
      // even with something well within range in front of it, that points
      // at wiring/the level shifter rather than these thresholds; if it
      // tracks a real object's distance but never dips under
      // OBSTACLE_ALERT_CM, the thresholds (or the sensor's aim/mounting
      // height) need adjusting instead.
      if (millis() - lastRangePrint >= 250) {
        lastRangePrint = millis();
        Serial.print("[AVOID] range: ");
        Serial.print(cm, 1);
        Serial.println(" cm");
      }

      if (cm <= OBSTACLE_EMERGENCY_CM) {
        // Too close to trust a turn-in-place (might clip the obstacle
        // swinging around) - back straight off instead, then keep trying
        // for the same target from the new position. This is a flinch,
        // not a direction change, so it never counts as "blocked" - but
        // it's still an obstacle present right now, so the indicator LED
        // goes on same as the TRIGGER case below.
        stopAllMotors();
        ledAlertBurst(); // ends with the LED off (last blink cycle) - leave it ON steady after
        ledSet(true);
        statusIndicatorSetState(STATUS_OBSTACLE);
        Serial.print("[AVOID] EMERGENCY - obstacle within ");
        Serial.print(OBSTACLE_EMERGENCY_CM, 0);
        Serial.println(" cm, too close to turn - backing up.");
        if (!navDriveStraight(OBSTACLE_BACKUP_MM, -OBSTACLE_BACKUP_SPEED)) { ledSet(false); return OBSTACLE_DRIVE_ABORTED; }
        if (!navSettle(NAV_SETTLE_MS))                                    { ledSet(false); return OBSTACLE_DRIVE_ABORTED; }

        // The backup above moved the A/B encoders (backward), so
        // re-baseline instead of letting the fabs()-based distance
        // calculation count that reverse roll as forward progress - and
        // re-latch heading-hold since navDriveStraight() reset it to
        // whatever heading the backup ended on.
        imu_resetHeadingHold();
        startA = encoderCount('A');
        startB = encoderCount('B');
        drivenBeforeThisStretch = distanceDrivenMM;
        distanceAtLastCheck = 0.0f;
        // Nothing in this function moves the servo off 90 deg, so this is
        // a no-op today - but make resuming after a restart explicitly
        // re-center it (cheap/idempotent) rather than relying on that
        // staying true as this function changes.
        servoMoveToSmooth(90);
        continue;
      }

      if (cm <= OBSTACLE_TRIGGER_CM) {
        // A new direction is needed - that decision belongs to the caller
        // (it knows how to turn this into a waypoint), so just stop and
        // report back. LED stays ON (obstacle-present indicator) until a
        // future call to this function completes a leg clear - see the
        // OBSTACLE_DRIVE_REACHED case below.
        stopAllMotors();
        ledAlertBurst(); // ends with the LED off (last blink cycle) - leave it ON steady after
        ledSet(true);
        statusIndicatorSetState(STATUS_OBSTACLE);
        return OBSTACLE_DRIVE_BLOCKED;
      }

      if (cm <= OBSTACLE_ALERT_CM) {
        // Early warning only - nothing to react to yet, just note it and
        // keep cruising forward. Separate quick blink, does not touch the
        // persistent obstacle-present LED state above.
        Serial.print("[AVOID] alert: obstacle ");
        Serial.print(cm, 1);
        Serial.println(" cm ahead");
        ledSet(true); delay(60); ledSet(false);
      }
    }

    float omega = imu_applyHeadingHold(0.0f);   // trim to stay on the heading latched at the top of this leg
    navDrive(0.0f, speed, omega);
  }

  stopAllMotors();
  // Covered the whole leg without ever being blocked - whatever was in
  // the way before is no longer in front of us, whether it moved off on
  // its own or we routed around it via a detour waypoint. Clear the
  // obstacle-present indicator.
  ledSet(false);
  statusIndicatorSetState(STATUS_NAV_RUNNING);
  return OBSTACLE_DRIVE_REACHED;
}

bool obstacleFindOpening(float &outRelativeDegCCW) {
  servoMoveToSmooth(90); // baseline - forward is already known blocked, that's why we're here
  int stoppedAtDeg;

  // Note: the obstacle-present LED is deliberately left alone here (not
  // turned off just because an opening was found) - it means "not
  // currently able to drive straight ahead", which is still true right
  // up until a leg actually completes clear (see
  // driveStraightWithObstacleCheck()'s OBSTACLE_DRIVE_REACHED case).
  Serial.println("[AVOID] scanning LEFT...");
  if (glideAndCheck(leftLimitDeg(), stoppedAtDeg)) {
    outRelativeDegCCW = (float)(stoppedAtDeg - 90) * (float)SCAN_DIR_SIGN;
    Serial.print("[AVOID] opening on the LEFT, ");
    Serial.print(outRelativeDegCCW, 0);
    Serial.println(" deg from straight ahead");
    // Angle is already captured above - safe to recenter now instead of
    // leaving the servo (and its forward-facing sensor) pointed off to
    // the side through the navTurn()+drive that follows.
    servoMoveToSmooth(90);
    return true;
  }

  Serial.println("[AVOID] LEFT blocked all the way - scanning RIGHT...");
  if (glideAndCheck(rightLimitDeg(), stoppedAtDeg)) {
    outRelativeDegCCW = (float)(stoppedAtDeg - 90) * (float)SCAN_DIR_SIGN;
    Serial.print("[AVOID] opening on the RIGHT, ");
    Serial.print(outRelativeDegCCW, 0);
    Serial.println(" deg from straight ahead");
    servoMoveToSmooth(90);
    return true;
  }

  Serial.println("[AVOID] blocked on both sides.");
  servoMoveToSmooth(90);
  return false;
}
