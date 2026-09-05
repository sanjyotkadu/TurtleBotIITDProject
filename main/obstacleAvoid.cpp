#include "obstacleAvoid.h"
#include "robot_config.h"
#include "navigation.h"
#include "motorControl.h"
#include "imu.h"
#include <Servo.h>
#include <math.h>

// ---------------- servo ----------------

static Servo sg90;
static int   currentServoDeg = 90;

// Steps 1 deg at a time with a delay between steps instead of jumping
// straight to the target, so the glide is visibly slow/controlled
// instead of snapping at the SG90's own max speed - same approach
// validated in obstacleAvoidTest.ino.
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
// above SCAN_CLEAR_CM before trusting "this direction is open".
static bool directionIsClear() {
  if (pingQuickCm() < SCAN_CLEAR_CM) return false;
  delay(5);
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
static float pingClosestCm(int samples) {
  float closest = (float)SONAR_MAX_RANGE_CM;
  for (int i = 0; i < samples; i++) {
    float cm = pingQuickCm();
    if (cm < closest) closest = cm;
    if (i < samples - 1) delay(3);
  }
  return closest;
}

// ---------------- obstacle LED alert ----------------

static void ledSet(bool on) {
  digitalWrite(OBSTACLE_LED_PIN, on ? HIGH : LOW);
}

static void ledAlertBurst() {
  for (int i = 0; i < OBSTACLE_ALERT_BLINKS; i++) {
    ledSet(true);  delay(OBSTACLE_ALERT_BLINK_MS);
    ledSet(false); delay(OBSTACLE_ALERT_BLINK_MS);
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
}

ObstacleDriveResult driveStraightWithObstacleCheck(float distanceMM, float speed, float &distanceDrivenMM) {
  servoMoveToSmooth(90);
  distanceDrivenMM = 0.0f;
  float remaining = distanceMM;

  while (remaining > 0.0f) {
    if (navShouldAbort()) {
      stopAllMotors();
      servoMoveToSmooth(90);
      ledSet(false);
      return OBSTACLE_DRIVE_ABORTED;
    }
    imu_update();

    // Multiple quick samples, closest wins - see pingClosestCm() above
    // for why a single ping isn't trusted here.
    float cm = pingClosestCm(OBSTACLE_CHECK_SAMPLES);

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
      Serial.print("[AVOID] EMERGENCY - obstacle within ");
      Serial.print(OBSTACLE_EMERGENCY_CM, 0);
      Serial.println(" cm, too close to turn - backing up.");
      if (!navDriveStraight(OBSTACLE_BACKUP_MM, -OBSTACLE_BACKUP_SPEED)) { ledSet(false); return OBSTACLE_DRIVE_ABORTED; }
      if (!navSettle(NAV_SETTLE_MS))                                    { ledSet(false); return OBSTACLE_DRIVE_ABORTED; }
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

    float chunk = fminf(remaining, OBSTACLE_CHECK_CHUNK_MM);
    if (!navDriveStraight(chunk, speed)) { ledSet(false); return OBSTACLE_DRIVE_ABORTED; }
    distanceDrivenMM += chunk;
    remaining -= chunk;
  }

  // Covered the whole leg without ever being blocked - whatever was in
  // the way before is no longer in front of us, whether it moved off on
  // its own or we routed around it via a detour waypoint. Clear the
  // obstacle-present indicator.
  ledSet(false);
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
    return true;
  }

  Serial.println("[AVOID] LEFT blocked all the way - scanning RIGHT...");
  if (glideAndCheck(rightLimitDeg(), stoppedAtDeg)) {
    outRelativeDegCCW = (float)(stoppedAtDeg - 90) * (float)SCAN_DIR_SIGN;
    Serial.print("[AVOID] opening on the RIGHT, ");
    Serial.print(outRelativeDegCCW, 0);
    Serial.println(" deg from straight ahead");
    return true;
  }

  Serial.println("[AVOID] blocked on both sides.");
  servoMoveToSmooth(90);
  return false;
}
