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
static unsigned long lastGoodMs    = 0;

static float wrapPi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

// Gyro+accel fusion, no magnetometer — immune to motor/steel interference.
static void enableReports() {
  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US);
}

void imuInit() {
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

bool imuUpdate() {
  // If the sensor rebooted underneath us, its reports stop until re-enabled.
  if (bno08x.wasReset()) enableReports();

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

      // Apply mounting sign (so CCW -> +heading), remove the zero
      // reference, and wrap to -PI..PI.
      headingRad = wrapPi(IMU_YAW_SIGN * yaw - headingOffset);
      lastGoodMs = millis();
      gotNew = true;
    }
  }
  return gotNew;
}

float imuHeadingRad() { return headingRad; }
float imuHeadingDeg() { return headingRad * 180.0f / (float)M_PI; }

bool imuHealthy() { return (millis() - lastGoodMs) < IMU_TIMEOUT_MS; }

void imuZeroHeading() {
  // Fold the current reading into the offset so heading reads 0 right now.
  headingOffset = wrapPi(headingOffset + headingRad);
  headingRad    = 0.0f;
}