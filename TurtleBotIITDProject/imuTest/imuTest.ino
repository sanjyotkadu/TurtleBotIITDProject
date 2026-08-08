/*
 * imuTest.ino — BNO085 heading bring-up & verification (Stage-1 IMU test)
 * ---------------------------------------------------------------------
 * Brings up the Adafruit BNO085 on I2C bus Wire2 (SDA 25 / SCL 24),
 * enables the GAME ROTATION VECTOR (gyro+accel fusion, NO magnetometer),
 * and streams heading (yaw) to serial. Run this BEFORE touching the main
 * firmware.
 *
 * WHAT TO CONFIRM TONIGHT:
 *   1. Sensor is found and reporting.
 *   2. Rotate the robot by hand — yaw tracks smoothly, no jumps.
 *   3. SIGN: rotating CCW (top view) must make heading INCREASE. If it
 *      DECREASES, set IMU_YAW_SIGN (-1) in robot_config.h.
 *   4. DRIFT: leave it still ~1 min — heading should barely move.
 *
 * WIRING (STEMMA QT / Qwiic, 3.3V native):
 *   SDA -> pin 25   SCL -> pin 24   3V -> 3.3V   GND -> GND
 *
 * NEEDS: "Adafruit BNO08x" library. Serial Monitor @ 115200, "Newline".
 * Commands:  s = stream on/off   z = zero heading here
 *            q = print raw quaternion once   ? = help
 * ---------------------------------------------------------------------
 */

#include "robot_config.h"
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <math.h>

static Adafruit_BNO08x   bno08x(BNO08X_RESET_PIN);
static sh2_SensorValue_t val;

static float headingOffset = 0.0f;
static bool  streaming     = true;

// last-read state
static float qr = 1, qi = 0, qj = 0, qk = 0;
static float headingDeg = 0.0f;
static bool  haveSample = false;

static float wrapPi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

static void enableReports() {
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US)) {
    Serial.println("[IMU] enableReport FAILED");
  }
}

static void poll() {
  if (bno08x.wasReset()) {
    Serial.println("[IMU] sensor reset — re-enabling reports");
    enableReports();
  }
  while (bno08x.getSensorEvent(&val)) {
    if (val.sensorId == SH2_GAME_ROTATION_VECTOR) {
      qr = val.un.gameRotationVector.real;
      qi = val.un.gameRotationVector.i;
      qj = val.un.gameRotationVector.j;
      qk = val.un.gameRotationVector.k;

      float yaw = atan2f(2.0f * (qr * qk + qi * qj),
                         1.0f - 2.0f * (qj * qj + qk * qk));
      float h   = wrapPi(IMU_YAW_SIGN * yaw - headingOffset);
      headingDeg = h * 180.0f / (float)M_PI;
      haveSample = true;
    }
  }
}

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial && millis() < 3000) {}

  Wire2.begin();
  Wire2.setClock(400000);

  Serial.println("\n==================================================");
  Serial.println(" BNO085 heading test  (Wire2: SDA 25 / SCL 24)");
  Serial.println("==================================================");

  bool ok = false;
  for (int i = 0; i < 10 && !ok; i++) {
    ok = bno08x.begin_I2C(BNO08X_I2C_ADDR, &Wire2);
    if (!ok) { Serial.println(" ...searching for BNO085"); delay(150); }
  }
  if (!ok) {
    Serial.println(" BNO085 NOT FOUND. Check Qwiic wiring, 3.3V, and addr");
    Serial.println(" (0x4A default, 0x4B if ADR high). Halting.");
    while (1) delay(1000);
  }
  Serial.println(" BNO085 found. Enabling Game Rotation Vector...");
  enableReports();

  Serial.println("\n ROTATE THE ROBOT CCW (top view) -> heading must INCREASE.");
  Serial.println(" If it DECREASES, set IMU_YAW_SIGN (-1) in robot_config.h.");
  Serial.println(" Commands: s=stream toggle  z=zero here  q=raw quat  ?=help\n");
}

static void handleSerial() {
  if (!Serial.available()) return;
  char c = Serial.read();
  switch (c) {
    case 's': case 'S':
      streaming = !streaming;
      Serial.println(streaming ? "[stream ON]" : "[stream OFF]");
      break;
    case 'z': case 'Z':
      headingOffset = wrapPi(headingOffset + headingDeg * (float)M_PI / 180.0f);
      Serial.println("[heading zeroed here]");
      break;
    case 'q': case 'Q':
      Serial.print("quat  w="); Serial.print(qr, 4);
      Serial.print(" i=");      Serial.print(qi, 4);
      Serial.print(" j=");      Serial.print(qj, 4);
      Serial.print(" k=");      Serial.println(qk, 4);
      break;
    case '?':
      Serial.println("s=stream toggle  z=zero here  q=raw quat  ?=help");
      break;
    default: break;
  }
}

void loop() {
  poll();
  handleSerial();

  static unsigned long lastPrint = 0;
  if (streaming && haveSample && millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print("heading: ");
    Serial.print(headingDeg, 1);
    Serial.println(" deg");
  }
}