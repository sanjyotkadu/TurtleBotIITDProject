#include "imu.h"
#include "robot_config.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

// Reset pin from config (-1 = none, we poll over the Qwiic cable).
static Adafruit_BNO08x   bno08x(BNO08X_RESET_PIN);
static sh2_SensorValue_t sensorValue;

static float         headingRad    = 0.0f;   // yaw since the reference pose, -PI..PI, CCW+
static float         rollDeg       = 0.0f;   // diagnostic only
static float         pitchDeg      = 0.0f;   // diagnostic only
static unsigned long lastGoodMs    = 0;

// Reference ("zero") orientation quaternion, captured live from the
// sensor. Every sample is expressed as a delta from this pose (see
// imu_update()/imu_zeroHeading()), which is what makes heading
// extraction immune to the physical mounting angle of the board.
static float refQr = 1.0f, refQi = 0.0f, refQj = 0.0f, refQk = 0.0f;
static bool  haveRef = false;

// Last raw sensor quaternion, kept so imu_zeroHeading() can (re)anchor
// the reference at any time, not just in step with a fresh sample.
static float lastQr = 1.0f, lastQi = 0.0f, lastQj = 0.0f, lastQk = 0.0f;

static bool  headingLocked    = false;   // heading-hold: is a target latched?
static float headingTargetRad = 0.0f;    // heading-hold: latched target

static float wrapPi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

// Hamilton product: out = a * b (quaternions as w,x,y,z).
static void quatMul(float aw, float ax, float ay, float az,
                    float bw, float bx, float by, float bz,
                    float &ow, float &ox, float &oy, float &oz) {
  ow = aw*bw - ax*bx - ay*by - az*bz;
  ox = aw*bx + ax*bw + ay*bz - az*by;
  oy = aw*by - ax*bz + ay*bw + az*bx;
  oz = aw*bz + ax*by - ay*bx + az*bw;
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
  haveRef = false;   // anchor the reference pose on the first sample we get
  Serial.println("[IMU] BNO085 online (Game Rotation Vector).");
}

bool imu_update() {
  // If the sensor rebooted underneath us, its reports stop until re-enabled.
  if (bno08x.wasReset()) enableReports();

  bool gotNew = false;
  // Drain everything queued this tick so heading is as fresh as possible.
  while (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_GAME_ROTATION_VECTOR) {
      lastQr = sensorValue.un.gameRotationVector.real;
      lastQi = sensorValue.un.gameRotationVector.i;
      lastQj = sensorValue.un.gameRotationVector.j;
      lastQk = sensorValue.un.gameRotationVector.k;

      if (!haveRef) {
        // First sample after boot/reset becomes the reference pose. If
        // the robot wasn't level/stationary right then, re-anchor with
        // imu_zeroHeading() once it's settled.
        refQr = lastQr; refQi = lastQi; refQj = lastQj; refQk = lastQk;
        haveRef = true;
      }

      // Delta quaternion from the reference pose to now: d = q(t) *
      // conjugate(q_ref). This cancels the fixed (but otherwise unknown)
      // sensor-to-chassis mounting rotation EXACTLY, so yaw/roll/pitch
      // extracted from d reflect how the CHASSIS actually moved since
      // the reference pose, regardless of which way the board is
      // mounted — flat, tilted, sideways, doesn't matter, as long as
      // it's rigidly fixed. (Extracting yaw straight from the raw
      // quaternion, like the old code did, only works if the board
      // happens to be mounted dead flat — a tilt then leaks into "yaw".)
      float dw, dx, dy, dz;
      quatMul(lastQr, lastQi, lastQj, lastQk,
              refQr, -refQi, -refQj, -refQk,
              dw, dx, dy, dz);

      float yaw = atan2f(2.0f * (dw * dz + dx * dy),
                         1.0f - 2.0f * (dy * dy + dz * dz));
      float sinPitch = 2.0f * (dw * dy - dz * dx);
      sinPitch = sinPitch >  1.0f ?  1.0f : (sinPitch < -1.0f ? -1.0f : sinPitch);
      float pitch = asinf(sinPitch);
      float roll  = atan2f(2.0f * (dw * dx + dy * dz),
                           1.0f - 2.0f * (dx * dx + dy * dy));

      // Apply mounting sign (so CCW -> +heading) and wrap to -PI..PI.
      headingRad = wrapPi(IMU_YAW_SIGN * yaw);
      rollDeg    = roll  * 180.0f / (float)M_PI;
      pitchDeg   = pitch * 180.0f / (float)M_PI;
      lastGoodMs = millis();
      gotNew = true;
    }
  }
  return gotNew;
}

float imu_headingRad() { return headingRad; }
float imu_headingDeg() { return headingRad * 180.0f / (float)M_PI; }
float imu_rollDeg()    { return rollDeg; }
float imu_pitchDeg()   { return pitchDeg; }

bool imu_healthy() { return (millis() - lastGoodMs) < IMU_TIMEOUT_MS; }

void imu_zeroHeading() {
  // Re-anchor the reference pose at the most recent raw sample, so
  // heading (and roll/pitch) read ~0 from here. Do this while the robot
  // is actually stationary and settled for a clean reference.
  refQr = lastQr; refQi = lastQi; refQj = lastQj; refQk = lastQk;
  haveRef = true;
}

void imu_resetHeadingHold() {
  headingLocked = false;
}

float imu_applyHeadingHold(float omegaCmd) {
  if (!imu_healthy() || fabs(omegaCmd) > HEADING_HOLD_OMEGA_DEADZONE) {
    headingLocked = false;
    return omegaCmd;
  }

  if (!headingLocked) {
    headingTargetRad = headingRad;
    headingLocked = true;
  }

  float error = wrapPi(headingTargetRad - headingRad);

  float correction = HEADING_HOLD_KP * error;
  if (correction >  HEADING_HOLD_MAX_OMEGA) correction =  HEADING_HOLD_MAX_OMEGA;
  if (correction < -HEADING_HOLD_MAX_OMEGA) correction = -HEADING_HOLD_MAX_OMEGA;
  return correction;
}
