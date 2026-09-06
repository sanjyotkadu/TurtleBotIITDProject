#include "imu.h"
#include "robot_config.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

// Reset pin from config (-1 = none, we poll over the Qwiic cable).
static Adafruit_BNO08x   bno08x(BNO08X_RESET_PIN);
static sh2_SensorValue_t sensorValue;

static float         headingRad    = 0.0f;   // processed yaw, -PI..PI, CCW+
static float         headingOffset = 0.0f;   // subtracted to set the zero reference
static float         pitchDeg      = 0.0f;   // nose-up/down tilt, degrees
static float         rollDeg       = 0.0f;   // side-to-side tilt, degrees
static unsigned long lastGoodMs    = 0;

static bool  headingLocked    = false;   // heading-hold: is a target latched?
static float headingTargetRad = 0.0f;    // heading-hold: latched target
static float headingErrorRad  = 0.0f;    // heading-hold: last computed (target - current), debug only
static float headingCorrection = 0.0f;   // heading-hold: last Omega actually returned, debug only

// The BNO085 can silently reset mid-run (brownout from motor current draw,
// I2C glitch from motor EMI, etc). When it does, its internal orientation
// reference resets too, which would otherwise make headingOffset stale and
// either (a) jump headingRad to a bogus value, or (b) fight to "correct"
// back to a target latched against a reference that no longer exists.
// awaitingResetRealign defers re-anchoring the offset to the first sample
// after the reset, so the reported heading stays continuous through it.
static uint32_t resetCount            = 0;
static bool     awaitingResetRealign  = false;
static float    headingAtReset        = 0.0f;

static float wrapPi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

// Gyro+accel fusion, no magnetometer — immune to motor/steel interference.
static void enableReports() {
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US);
}

void imu_init() {
  Wire2.begin();
  Wire2.setClock(400000);   // BNO08x supports 400 kHz I2C

  // SHTP bring-up is occasionally slow right after power-up; retry a few times.
  bool ok = false;
  for (int i = 0; i < 5 && !ok; i++) {
    ok = bno08x.begin_I2C(BNO08X_I2C_ADDR, &Wire2);
    if (!ok) delay(100);
  }
  if (!ok) {
    Serial.println("[IMU] BNO085 not found on Wire2 (pins 24/25) — check wiring/addr.");
    return;
  }
  enableReports();
  lastGoodMs = millis();
  Serial.println("[IMU] BNO085 online (Game Rotation Vector).");
}

bool imu_update() {
  // If the sensor rebooted underneath us, its reports stop until
  // re-enabled, AND its internal orientation reference has reset — so
  // headingOffset (computed against the OLD reference) is now stale.
  // Flag it so the next sample re-anchors the offset instead of jumping,
  // drop any heading-hold lock (its target is against a dead reference),
  // and log it — this is otherwise completely silent.
  if (bno08x.wasReset()) {
    resetCount++;
    headingAtReset = headingRad;
    awaitingResetRealign = true;
    headingLocked = false;
    Serial.print("[IMU] BNO085 reset detected (#");
    Serial.print(resetCount);
    Serial.println(") mid-run - re-enabling reports and re-anchoring heading zero.");
    enableReports();
  }

  bool gotNew = false;
  // Drain everything queued this tick so heading is as fresh as possible.
  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      float qr = sensorValue.un.gameRotationVector.real;
      float qi = sensorValue.un.gameRotationVector.i;
      float qj = sensorValue.un.gameRotationVector.j;
      float qk = sensorValue.un.gameRotationVector.k;

      // Yaw about the vertical axis from the orientation quaternion.
      float yaw = atan2f(2.0f * (qr * qk + qi * qj),
                         1.0f - 2.0f * (qj * qj + qk * qk));

      if (awaitingResetRealign) {
        // Re-anchor so this first post-reset sample continues from the
        // last known-good heading (assumes the robot didn't physically
        // snap-rotate during the few-ms reset gap) instead of jumping to
        // whatever the chip's new internal zero happens to be.
        headingOffset = wrapPi(IMU_YAW_SIGN * yaw - headingAtReset);
        awaitingResetRealign = false;
      }

      // Apply mounting sign (so CCW -> +heading), remove the zero
      // reference, and wrap to -PI..PI.
      headingRad = wrapPi(IMU_YAW_SIGN * yaw - headingOffset);

      // Roll (x-axis) and pitch (y-axis) from the same quaternion,
      // same rotation convention as the yaw term above. Reporting
      // only — no zero-offset, no drive-loop consumer.
      float roll  = atan2f(2.0f * (qr * qi + qj * qk),
                           1.0f - 2.0f * (qi * qi + qj * qj));
      float sinp  = 2.0f * (qr * qj - qk * qi);
      if (sinp >  1.0f) sinp =  1.0f;
      if (sinp < -1.0f) sinp = -1.0f;
      float pitch = asinf(sinp);

      rollDeg  = roll  * 180.0f / (float)M_PI;
      pitchDeg = pitch * 180.0f / (float)M_PI;

      lastGoodMs = millis();
      gotNew = true;
    }
  }
  return gotNew;
}

float imu_headingRad() { return headingRad; }
float imu_headingDeg() { return headingRad * 180.0f / (float)M_PI; }

bool imu_healthy() { return (millis() - lastGoodMs) < IMU_TIMEOUT_MS; }

uint32_t imu_resetCount() { return resetCount; }

void imu_printHeading() {
  Serial.print("heading: ");
  Serial.print(imu_headingDeg(), 1);
  Serial.println(" deg");
}

float imu_pitchDeg() { return pitchDeg; }
float imu_rollDeg()  { return rollDeg; }

bool imu_isTipped() {
  if (!imu_healthy()) return false;   // no data = no claim
  return fabs(pitchDeg) > IMU_TIP_THRESHOLD_DEG ||
         fabs(rollDeg)  > IMU_TIP_THRESHOLD_DEG;
}

void imu_zeroHeading() {
  // Fold the current reading into the offset so heading reads 0 right now.
  headingOffset = wrapPi(headingOffset + headingRad);
  headingRad    = 0.0f;
}

void imu_resetHeadingHold() {
  headingLocked = false;
}

float imu_applyHeadingHold(float omegaCmd) {
  if (!imu_healthy() || fabs(omegaCmd) > HEADING_HOLD_OMEGA_DEADZONE) {
    headingLocked   = false;
    headingErrorRad = 0.0f;
    headingCorrection = omegaCmd;
    return omegaCmd;
  }

  if (!headingLocked) {
    headingTargetRad = headingRad;
    headingLocked = true;
  }

  float error    = wrapPi(headingTargetRad - headingRad);
  float errorDeg = fabs(error) * 180.0f / (float)M_PI;

  float correction;
  if (errorDeg < HEADING_HOLD_TOLERANCE_DEG) {
    // Close enough — command a hard zero instead of a tiny P output that
    // would just dither at the stiction boundary (the "buzzing").
    correction = 0.0f;
  } else {
    correction = HEADING_HOLD_KP * error;
    float mag = fabs(correction);
    // Floor: never ask for less than enough torque to actually move once
    // we're committed to correcting, so the command can't decay into the
    // stall zone before the error is gone.
    if (mag < HEADING_HOLD_MIN_OMEGA) mag = HEADING_HOLD_MIN_OMEGA;
    if (mag > HEADING_HOLD_MAX_OMEGA) mag = HEADING_HOLD_MAX_OMEGA;
    correction = (error >= 0.0f) ? mag : -mag;
  }

  headingErrorRad   = error;
  headingCorrection = correction;
  return correction;
}

bool  imu_headingHoldLocked()     { return headingLocked; }
float imu_headingHoldTargetDeg()  { return headingTargetRad * 180.0f / (float)M_PI; }
float imu_headingHoldErrorDeg()   { return headingErrorRad  * 180.0f / (float)M_PI; }
float imu_headingHoldCorrection() { return headingCorrection; }