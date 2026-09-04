/*
 * obstacleAvoidTest.ino — servo + HC-SR04 obstacle avoidance, IMU-referenced result
 * ---------------------------------------------------------------------
 * Combines the servoTest and ultrasonicTest bring-ups into one behavior:
 *
 *   WATCH  - servo pings whatever direction it is CURRENTLY pointed at
 *            (90 deg / straight ahead at boot, or wherever the last scan
 *            parked it - see SCAN below). It just sits there: no
 *            periodic swinging back to check anywhere else. Every
 *            reading also updates the obstacle map (see below).
 *   ALERT  - triggered when that watched direction drops below
 *            OBSTACLE_TRIGGER_CM (or reads "too close"): prints
 *            "Obstacle is present in front!" and blinks the onboard LED
 *            (pin 13) a few times. This is the SAME trigger whether it's
 *            the very first obstacle (servo still at 90) or a NEW
 *            obstacle appearing at wherever it was previously parked -
 *            either way it means "find a new direction".
 *   SCAN   - one continuous, slow servo glide (no full-stop-per-step
 *            sweep - that's what made earlier versions look jerky):
 *              1. re-center to 90 deg, then glide toward LEFT, checking
 *                 for an opening every SCAN_STEP_DEG of travel with a
 *                 fast ping. A single clear check-point does NOT count -
 *                 needs SCAN_CONFIRM_STEPS in a row (any blocked
 *                 reading resets the streak) before it commits and
 *                 STOPS there.
 *              2. nothing found by the LEFT limit -> keep gliding, same
 *                 motion, across center to the RIGHT limit, same check.
 *                 Opening found -> STOP and stay there.
 *              3. nothing found on either side -> glide back to center
 *                 and set the "blocked on all sides" flag.
 *   REPORT - prints which side (if any) had an opening, in degrees
 *            left/right of straight ahead, plus the resulting absolute
 *            heading to turn to if the IMU is healthy. If blocked on
 *            all sides, prints that instead and raises the flag. Either
 *            way, drops back into WATCH at wherever it ended up parked -
 *            it stays there for good until THAT specific direction
 *            reads blocked again, which starts a new SCAN.
 *
 * OBSTACLE MAP — every ping, in WATCH or during a SCAN, records
 * clear/blocked at that angle (10 deg bins) so you can see which
 * headings have had obstacles and which are open. Press 'm' to print
 * it. A bin is simply overwritten with whatever the latest ping there
 * said, so a direction that was blocked and is now confirmed clear
 * updates (and vice versa) - there's no separate "clear the data" step,
 * the map just always reflects the most recent reading per angle.
 *
 * RESULT FLAG FOR THE MAIN NAVIGATION PROJECT — the goal is for main/'s
 * navigation to call this same logic and, when the front is blocked,
 * steer toward whichever side has no obstacle. This sketch exposes that
 * as three functions meant to be lifted into main/ later:
 *   obstacleIsBlockedAllSides()   - true only if neither side had an
 *                                   opening; navigation should not drive
 *                                   forward while this is true.
 *   obstacleHasSuggestedHeading() - true once an opening was found.
 *   obstacleSuggestedHeadingDeg() - the IMU heading to turn to.
 *
 * SWEEP SPEED — glides 1 deg at a time with a delay between steps
 * (SERVO_SLEW_STEP_DEG / SERVO_SLEW_DELAY_MS in robot_config.h). Raise
 * SERVO_SLEW_DELAY_MS to slow it down further. Each in-glide check uses a
 * short pulseIn() timeout (SCAN_CHECK_TIMEOUT_US) instead of the long one
 * used for idle ranging, specifically so a check barely pauses the glide.
 *
 * HARDWARE LIMIT — READ BEFORE WIRING/TESTING:
 *   A single SG90 physically travels 0-180 deg; it cannot look behind the
 *   robot, so this cannot do a true -180..180 (360 deg) scan with one
 *   servo. "LEFT"/"RIGHT" above are the servo's own 180 deg / 0 deg ends,
 *   reported as a relative angle in -90..+90 deg (left/right of
 *   straight ahead). Mount the servo so its 0-180 deg travel covers the
 *   robot's forward hemisphere.
 *
 *   The HC-SR04 is also unreliable below ~4-5cm (see SONAR_MIN_RANGE_CM
 *   in robot_config.h) - readings that close are reported as "too
 *   close" rather than trusted, same as ultrasonicTest.ino.
 *
 * WIRING: see the header comments in servoTest.ino, ultrasonicTest.ino,
 * and imuTest.ino (same pins, now all on one sketch) - or the pin map at
 * the bottom of robot_config.h.
 *
 * NEEDS: Servo library (bundled with Teensyduino) + "Adafruit BNO08x"
 * library. Serial Monitor @ 115200, "Newline".
 *
 * COMMANDS:
 *   p   -> force a scan right now (ignores the trigger distance)
 *   c   -> recenter servo to 90 deg and clear the blocked flag
 *   b   -> print obstacleIsBlockedAllSides()
 *   m   -> print the obstacle map (clear/blocked per angle bin)
 *   s   -> toggle watch-direction distance streaming
 *   z   -> zero the IMU heading here
 *   ?   -> help
 * ---------------------------------------------------------------------
 */

#include "robot_config.h"
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

// ---------------- servo ----------------

static Servo sg90;
static int   currentServoDeg = 90;

// Steps 1 deg at a time (SERVO_SLEW_STEP_DEG) with a delay between steps
// (SERVO_SLEW_DELAY_MS), instead of jumping straight to the target - that
// jump is what made the sweep look too fast, since the SG90 then slews
// there at its own max speed. This gives a visibly slower, controlled
// sweep whose speed is tunable from robot_config.h.
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

// Same sentinel scheme as ultrasonicTest.ino. Used for the idle forward
// watch, which wants the sensor's full range - hence the long
// SONAR_TIMEOUT_US.
// -1 = no echo (out of range / nothing there), -2 = below the sensor's
// reliable minimum range.
static float pingDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long echoUs = pulseIn(ECHO_PIN, HIGH, SONAR_TIMEOUT_US);
  if (echoUs == 0) return -1.0f;

  float cm = (echoUs * 0.0343f) / 2.0f;
  if (cm > SONAR_MAX_RANGE_CM) return -1.0f;
  if (cm < SONAR_MIN_RANGE_CM) return -2.0f;
  return cm;
}

// Quick single ping for use WHILE the servo is gliding: a short timeout
// (SCAN_CHECK_TIMEOUT_US, ~100cm range) instead of pingDistanceCm()'s full
// SONAR_TIMEOUT_US (~5m), so a check barely pauses the sweep. No-echo
// within that short window just means "nothing within useful range" -
// i.e. clear - so it's reported as SONAR_MAX_RANGE_CM same as a long ping.
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
// above SCAN_CLEAR_CM before trusting "this direction is open" enough to
// stop the sweep there.
static bool directionIsClear() {
  if (pingQuickCm() < SCAN_CLEAR_CM) return false;
  delay(5);
  return pingQuickCm() >= SCAN_CLEAR_CM;
}

// ---------------- IMU (same Game Rotation Vector approach as imuTest.ino) ----------------

static Adafruit_BNO08x   bno08x(BNO08X_RESET_PIN);
static sh2_SensorValue_t sensorValue;

static bool          imuOk          = false;
static float         headingOffset  = 0.0f;
static float         headingDeg     = 0.0f;
static unsigned long imuLastGoodMs  = 0;

static float wrapPi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

static float wrapDeg(float d) {
  while (d >  180.0f) d -= 360.0f;
  while (d < -180.0f) d += 360.0f;
  return d;
}

static void imuEnableReports() {
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US);
}

static void imuBegin() {
  Wire2.begin();
  Wire2.setClock(400000);

  for (int i = 0; i < 10 && !imuOk; i++) {
    imuOk = bno08x.begin_I2C(BNO08X_I2C_ADDR, &Wire2);
    if (!imuOk) delay(150);
  }

  if (!imuOk) {
    Serial.println("[IMU] BNO085 NOT FOUND (Wire2, pins 24/25) - continuing without heading.");
    Serial.println("[IMU] Scan will still report LEFT/RIGHT + degrees, just not an absolute heading.");
    return;
  }
  imuEnableReports();
  imuLastGoodMs = millis();
  Serial.println("[IMU] BNO085 online (Game Rotation Vector).");
}

static void imuPoll() {
  if (!imuOk) return;

  if (bno08x.wasReset()) {
    Serial.println("[IMU] BNO085 reset detected mid-run - re-enabling reports.");
    imuEnableReports();
  }

  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qr = sensorValue.un.gameRotationVector.real;
      float qi = sensorValue.un.gameRotationVector.i;
      float qj = sensorValue.un.gameRotationVector.j;
      float qk = sensorValue.un.gameRotationVector.k;

      float yaw = atan2f(2.0f * (qr * qk + qi * qj),
                         1.0f - 2.0f * (qj * qj + qk * qk));
      float h   = wrapPi(IMU_YAW_SIGN * yaw - headingOffset);
      headingDeg = h * 180.0f / (float)M_PI;
      imuLastGoodMs = millis();
    }
  }
}

static bool imuHealthy() {
  return imuOk && (millis() - imuLastGoodMs) < IMU_TIMEOUT_MS;
}

static void imuZeroHeading() {
  headingOffset = wrapPi(headingOffset + headingDeg * (float)M_PI / 180.0f);
  headingDeg = 0.0f;
}

// ---------------- obstacle LED alert ----------------

static void ledSet(bool on) {
  digitalWrite(OBSTACLE_LED_PIN, on ? HIGH : LOW);
}

// Quick blink burst the instant the idle watch trips, before the scan
// even starts.
static void ledAlertBurst() {
  for (int i = 0; i < OBSTACLE_ALERT_BLINKS; i++) {
    ledSet(true);  delay(OBSTACLE_ALERT_BLINK_MS);
    ledSet(false); delay(OBSTACLE_ALERT_BLINK_MS);
  }
}

// ---------------- obstacle map (per-angle memory) ----------------
//
// Coarse memory of what the last ping AT each angle bin (SCAN_STEP_DEG
// wide) found, fed by every ping this sketch takes - both the ongoing
// watch and each check-point during a scan. A bin is simply overwritten
// with the latest reading, so it self-clears the moment that angle is
// re-checked and found open again - there's nothing separate to reset.
#define OBSTACLE_MAP_BINS (180 / SCAN_STEP_DEG + 1)

enum BinState { BIN_UNKNOWN, BIN_CLEAR, BIN_BLOCKED };
static BinState obstacleMap[OBSTACLE_MAP_BINS];

static int binIndexForDeg(int deg) {
  return constrain(deg, 0, 180) / SCAN_STEP_DEG;
}

static void recordObstacle(int deg, bool blocked) {
  obstacleMap[binIndexForDeg(deg)] = blocked ? BIN_BLOCKED : BIN_CLEAR;
}

static void printObstacleMap() {
  Serial.println("[obstacle map] deg: status");
  for (int i = 0; i < OBSTACLE_MAP_BINS; i++) {
    Serial.print("  ");
    Serial.print(i * SCAN_STEP_DEG);
    Serial.print(" deg: ");
    switch (obstacleMap[i]) {
      case BIN_CLEAR:   Serial.println("clear");   break;
      case BIN_BLOCKED: Serial.println("blocked"); break;
      default:          Serial.println("unknown"); break;
    }
  }
}

// ---------------- result flag (for the main navigation project) ----------------
//
// This is the piece meant to be lifted into main/ later: after a scan,
// g_obstacleBlockedAllSides is true only if NEITHER left NOR right had an
// opening - i.e. the robot is boxed in and navigation should do something
// other than drive forward (back up, stop, replan). g_hasSuggestedHeading
// / g_suggestedHeadingDeg carry the IMU heading to turn to when an
// opening WAS found. Both are only meaningful right after a scan - see
// obstacleIsBlockedAllSides() / obstacleSuggestedHeadingDeg() below.
static bool  g_obstacleBlockedAllSides = false;
static bool  g_hasSuggestedHeading     = false;
static float g_suggestedHeadingDeg     = 0.0f;

bool  obstacleIsBlockedAllSides()  { return g_obstacleBlockedAllSides; }
bool  obstacleHasSuggestedHeading() { return g_hasSuggestedHeading; }
float obstacleSuggestedHeadingDeg() { return g_suggestedHeadingDeg; }

// ---------------- obstacle scan state machine ----------------

enum ScanState { STATE_IDLE, STATE_SCANNING, STATE_COOLDOWN };

static ScanState      state           = STATE_IDLE;
static bool           streamIdle      = true;
static unsigned long  lastIdlePingMs  = 0;
static unsigned long  cooldownUntilMs = 0;

static void printHelp() {
  Serial.println("\n==================================================");
  Serial.println(" Servo + HC-SR04 obstacle avoidance scan (IMU-referenced)");
  Serial.println("==================================================");
  Serial.println(" p = force a scan now");
  Serial.println(" c = recenter servo to 90 deg + clear blocked flag");
  Serial.println(" b = print obstacleIsBlockedAllSides()");
  Serial.println(" m = print the obstacle map (clear/blocked per angle)");
  Serial.println(" s = toggle watch-direction distance streaming");
  Serial.println(" z = zero IMU heading here");
  Serial.println(" ? = this help");
  Serial.println("==================================================\n");
}

static void printWatchReading(int deg, float raw) {
  Serial.print(" [watch ");
  Serial.print(deg);
  Serial.print(" deg] ");
  if (raw == -2.0f)      Serial.println("too close (below sensor's reliable minimum range)");
  else if (raw < 0)      Serial.println("clear (no echo / out of range)");
  else                   { Serial.print(raw, 1); Serial.println(" cm"); }
}

// Which physical servo angle is "LEFT" / "RIGHT" depends on SCAN_DIR_SIGN
// (see robot_config.h) - keeps the LEFT/RIGHT wording in the printouts
// correct even if that sign gets flipped for a different servo mount.
static int leftLimitDeg()  { return (SCAN_DIR_SIGN > 0) ? 180 : 0; }
static int rightLimitDeg() { return (SCAN_DIR_SIGN > 0) ? 0 : 180; }

// One continuous glide from the servo's CURRENT position (currentServoDeg
// - never an assumed starting angle, so a repeat scan that starts with
// the servo already parked somewhere from a previous result still glides
// smoothly instead of snapping) to toDeg, checking for an opening every
// SCAN_STEP_DEG of travel. A single check-point reading clear is NOT
// enough to stop on - a narrow gap in the ultrasonic beam, or an
// obstacle angled enough to reflect the ping away instead of back, can
// make one check-point look clear even though the arc right after it is
// still blocked. Requires SCAN_CONFIRM_STEPS consecutive clear
// check-points (any blocked reading resets the streak) before
// committing to "opening found" and stopping there. If it reaches toDeg
// without ever reaching that streak, returns false and the servo is left
// at toDeg.
static bool glideAndCheck(int toDeg, int &stoppedAtDeg) {
  int fromDeg         = currentServoDeg;
  int dir             = (toDeg >= fromDeg) ? 1 : -1;
  int angle           = fromDeg;
  int lastCheckDeg    = fromDeg;
  int consecutiveClear = 0;

  while (angle != toDeg) {
    angle += dir * SERVO_SLEW_STEP_DEG;
    if ((dir > 0 && angle > toDeg) || (dir < 0 && angle < toDeg)) angle = toDeg;

    currentServoDeg = angle;
    sg90.writeMicroseconds(map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US));
    delay(SERVO_SLEW_DELAY_MS);
    imuPoll(); // keep heading fresh while the glide is blocking

    if (abs(angle - lastCheckDeg) >= SCAN_STEP_DEG || angle == toDeg) {
      lastCheckDeg = angle;
      bool clear = directionIsClear();
      consecutiveClear = clear ? (consecutiveClear + 1) : 0;
      recordObstacle(angle, !clear);

      Serial.print("    check ");
      Serial.print(angle);
      Serial.print(" deg -> ");
      Serial.print(clear ? "clear" : "blocked");
      Serial.print(" (streak ");
      Serial.print(consecutiveClear);
      Serial.print("/");
      Serial.print(SCAN_CONFIRM_STEPS);
      Serial.println(")");

      if (consecutiveClear >= SCAN_CONFIRM_STEPS) {
        stoppedAtDeg = angle;
        return true;
      }
    }
  }

  stoppedAtDeg = toDeg;
  return false;
}

// Prints the LEFT/RIGHT relative angle plus, if the IMU is up, the
// absolute heading to turn to - the "which way to go" result.
static void reportOpening(const char *side, int stoppedAtDeg, float startHeadingDeg, bool haveHeading) {
  float relativeDeg = (float)(stoppedAtDeg - 90) * (float)SCAN_DIR_SIGN;

  Serial.println("[SCAN] result: opening found");
  Serial.print("  side: "); Serial.println(side);
  Serial.print("  servo stopped at "); Serial.print(stoppedAtDeg);
  Serial.print(" deg ("); Serial.print(relativeDeg >= 0 ? "LEFT " : "RIGHT ");
  Serial.print(fabs(relativeDeg), 0);
  Serial.println(" deg from straight ahead, CCW+)");

  g_hasSuggestedHeading = haveHeading;
  if (haveHeading) {
    g_suggestedHeadingDeg = wrapDeg(startHeadingDeg + relativeDeg);
    Serial.print("  current heading: ");   Serial.print(startHeadingDeg, 1);   Serial.println(" deg");
    Serial.print("  suggested heading: "); Serial.print(g_suggestedHeadingDeg, 1);
    Serial.println(" deg (turn to this heading to head into open space)");
  } else {
    Serial.println("  IMU heading unavailable - go by the relative LEFT/RIGHT figure above.");
  }
  Serial.println();
}

// Left-then-right avoidance check, per the requested behavior: from
// center, glide left looking for an opening and stop there if found;
// otherwise keep gliding across to the right looking for one there;
// otherwise glide back to center and flag "blocked on all sides" for
// the navigation layer to act on.
static void runScan() {
  Serial.println("\n[ALERT] Obstacle is present in front!");

  // Baseline at straight-ahead before probing - a quick plain move, no
  // checking needed en route since forward is already known blocked
  // (that's the trigger condition that got us here).
  servoMoveToSmooth(90);

  float startHeadingDeg = headingDeg;
  bool  haveHeading     = imuHealthy();
  g_obstacleBlockedAllSides = false;
  g_hasSuggestedHeading     = false;

  int stoppedAtDeg;

  Serial.println("[SCAN] checking LEFT...");
  if (glideAndCheck(leftLimitDeg(), stoppedAtDeg)) {
    ledSet(false);
    reportOpening("LEFT", stoppedAtDeg, startHeadingDeg, haveHeading);
  } else {
    Serial.println("[SCAN] LEFT blocked all the way - checking RIGHT...");
    if (glideAndCheck(rightLimitDeg(), stoppedAtDeg)) {
      ledSet(false);
      reportOpening("RIGHT", stoppedAtDeg, startHeadingDeg, haveHeading);
    } else {
      Serial.println("[SCAN] RIGHT blocked too - returning to center.");
      servoMoveToSmooth(90);
      g_obstacleBlockedAllSides = true;
      ledSet(true); // solid ON while blocked - OFF as soon as it's found clear again
      Serial.println("[SCAN] result: BLOCKED ON ALL SIDES.");
      Serial.println("  obstacleIsBlockedAllSides() = true - navigation should not drive forward.\n");
    }
  }

  // Whichever way it ended up parked, stay there for good - the watch
  // loop below now checks THAT direction, and only starts a new scan if
  // it reads blocked again (a new obstacle right where it's parked).
  cooldownUntilMs = millis() + SCAN_REPORT_COOLDOWN_MS;
  state = STATE_COOLDOWN;
}

static void handleSerial() {
  if (!Serial.available()) return;
  char ch = Serial.read();
  switch (ch) {
    case 'p': case 'P':
      state = STATE_SCANNING;
      break;
    case 's': case 'S':
      streamIdle = !streamIdle;
      Serial.println(streamIdle ? "[idle streaming ON]" : "[idle streaming OFF]");
      break;
    case 'z': case 'Z':
      imuZeroHeading();
      Serial.println("[heading zeroed here]");
      break;
    case 'c': case 'C':
      // A found opening leaves the servo parked pointing at it (see the
      // WATCH/SCAN description at the top of this file) - use this to
      // manually recenter and resume watching forward, and to clear a
      // stale blocked-all-sides flag.
      servoMoveToSmooth(90);
      g_obstacleBlockedAllSides = false;
      g_hasSuggestedHeading     = false;
      ledSet(false);
      state = STATE_IDLE;
      Serial.println("[re-centered - back to watching forward]");
      break;
    case 'b': case 'B':
      Serial.print("[obstacleIsBlockedAllSides() = ");
      Serial.print(g_obstacleBlockedAllSides ? "true" : "false");
      Serial.println("]");
      break;
    case 'm': case 'M':
      printObstacleMap();
      break;
    case '?':
      printHelp();
      break;
    default: break;
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial && millis() < 3000) { /* wait briefly for USB serial */ }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(OBSTACLE_LED_PIN, OUTPUT);
  digitalWrite(OBSTACLE_LED_PIN, LOW);

  sg90.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  // Direct jump only here at boot (position unknown beforehand, nothing
  // to slew smoothly from) - every move after this uses servoMoveToSmooth().
  currentServoDeg = 90;
  sg90.writeMicroseconds(map(currentServoDeg, 0, 180, SERVO_MIN_US, SERVO_MAX_US));

  imuBegin();
  printHelp();
  Serial.println(" Watching forward. Bring an object within OBSTACLE_TRIGGER_CM to trigger a scan.\n");
}

void loop() {
  imuPoll();
  handleSerial();

  switch (state) {
    // Watches whatever direction the servo is CURRENTLY pointed at - 90
    // deg / straight ahead at boot, or wherever the last scan parked it.
    // It does not move on its own here: it just sits and keeps checking
    // that one direction. If that direction was a previously-found
    // opening and stays clear, it stays parked there indefinitely. Only
    // a NEW obstacle showing up right there triggers a fresh scan.
    case STATE_IDLE: {
      if (millis() - lastIdlePingMs >= 150) {
        lastIdlePingMs = millis();
        float raw = pingDistanceCm();
        if (streamIdle) printWatchReading(currentServoDeg, raw);

        bool objectClose = (raw == -2.0f) || (raw > 0 && raw < OBSTACLE_TRIGGER_CM);
        recordObstacle(currentServoDeg, objectClose);

        if (objectClose) {
          ledAlertBurst(); // immediate blink, before the scan even starts
          state = STATE_SCANNING;
        } else if (g_obstacleBlockedAllSides) {
          // Blocked-all-sides is parked watching center (90 deg), same
          // as this loop always did before any scan ever ran - so
          // "still clear here" really does mean "no longer boxed in".
          // (g_hasSuggestedHeading is left alone: it means "still
          // parked at a found opening", which staying clear here
          // confirms rather than invalidates.)
          g_obstacleBlockedAllSides = false;
          ledSet(false);
        }
      }
      break;
    }

    case STATE_SCANNING:
      runScan();
      break;

    // Short pause after a scan resolves before the watch loop above
    // starts checking the angle it parked at, so a transient noisy
    // reading right after stopping doesn't immediately re-trigger.
    case STATE_COOLDOWN:
      if ((long)(millis() - cooldownUntilMs) >= 0) state = STATE_IDLE;
      break;
  }
}
