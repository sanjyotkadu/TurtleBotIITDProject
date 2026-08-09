#include "diagnostics.h"
#include "robot_config.h"
#include "RCControl.h"
#include "imu.h"


// Teensy 4.1 onboard LED. Local to diagnostics so robot_config.h stays
// as the pure pin/geometry map it already is.
#define STATUS_LED_PIN 13

// Latest per-wheel command, cached for the next debug dump.
static float dbgPowerC = 0, dbgPowerA = 0, dbgPowerB = 0; // normalized, pre-direction
static int   dbgPwmC = 0,   dbgPwmA = 0,   dbgPwmB = 0;   // actual PWM (0..MOTOR_PWM_MAX)

void diagInit() {
  pinMode(STATUS_LED_PIN, OUTPUT);
}

void setStatusLed(unsigned long onMs, unsigned long offMs) {
  static unsigned long lastToggle = 0;
  static bool ledOn = false;
  static unsigned long curOnMs = 0xFFFFFFFF, curOffMs = 0xFFFFFFFF;

  if (onMs != curOnMs || offMs != curOffMs) {
    curOnMs = onMs;
    curOffMs = offMs;
    lastToggle = millis();
    ledOn = true;
  }

  if (onMs == 0)  { digitalWrite(STATUS_LED_PIN, LOW);  return; } // solid OFF
  if (offMs == 0) { digitalWrite(STATUS_LED_PIN, HIGH); return; } // solid ON

  unsigned long interval = ledOn ? onMs : offMs;
  if (millis() - lastToggle >= interval) {
    ledOn = !ledOn;
    lastToggle = millis();
  }
  digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
}

void statusLedUpdate(bool failsafe, bool enabled, bool sticksCentered) {
  if (failsafe) {
    setStatusLed(60, 60);        // very fast blink - link lost
  } else if (enabled) {
    setStatusLed(0, 1);          // solid OFF - armed, motors live
  } else if (!sticksCentered) {
    setStatusLed(150, 150);      // fast blink - not centered / not armed
  } else {
    setStatusLed(1, 0);          // solid ON - centered, ready to arm
  }
}

void diagSetWheelDebug(float powerC, float powerA, float powerB,
                       int pwmC, int pwmA, int pwmB) {
  dbgPowerC = powerC; dbgPowerA = powerA; dbgPowerB = powerB;
  dbgPwmC   = pwmC;   dbgPwmA   = pwmA;   dbgPwmB   = pwmB;
}

void debugPrint(bool enabled, bool failsafe) {
  if (failsafe) {
    Serial.println("!! FAILSAFE - no valid iBUS frame recently - motors stopped !!");
    return;
  }

  float Vx    = rcVx();
  float Vy    = rcVy();
  float Omega = rcOmega();

  Serial.print("ENABLE: ");
  Serial.print(enabled ? "ON " : "OFF");
  Serial.print("  Vx: ");
  Serial.print(Vx, 2);
  Serial.print("  Vy: ");
  Serial.print(Vy, 2);
  Serial.print("  Omega: ");
  Serial.print(Omega, 2);
  Serial.print("  [raw VX:");
  Serial.print(rcRawChannel(IBUS_CH_VX));
  Serial.print(" VY:");
  Serial.print(rcRawChannel(IBUS_CH_VY));
  Serial.print(" OMEGA:");
  Serial.print(rcRawChannel(IBUS_CH_OMEGA));
  Serial.print(" | centers ");
  Serial.print(rcCenterVx(), 1);
  Serial.print("/");
  Serial.print(rcCenterVy(), 1);
  Serial.print("/");
  Serial.print(rcCenterOmega(), 1);
  Serial.println("]");

  Serial.print("  [power C:");
  Serial.print(dbgPowerC, 2);
  Serial.print(" A:");
  Serial.print(dbgPowerA, 2);
  Serial.print(" B:");
  Serial.print(dbgPowerB, 2);
  Serial.print(" | pwm C:");
  Serial.print(dbgPwmC);
  Serial.print(" A:");
  Serial.print(dbgPwmA);
  Serial.print(" B:");
  Serial.print(dbgPwmB);
  Serial.println("]");

  Serial.print("  [imu heading:");
  Serial.print(imu_headingDeg(), 1);
  Serial.print(" deg  healthy:");
  Serial.print(imu_healthy() ? "Y" : "N");
  Serial.println("]");
}
