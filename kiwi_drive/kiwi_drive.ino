/*
 * Kiwi Holonomic Drive — Duttallbot / TurtleBot IITD
 * ---------------------------------------------------------------------
 * Combines:
 *   - robot_config.h   (pins, channel map, tuning constants — untouched)
 *   - IBusReader.h      (iBUS protocol parser — untouched)
 *   - Joystick reading  (same logic as joystick_read.ino)
 *   - 3-wheel kiwi kinematics (from user-provided pseudocode, renamed
 *     to match this project's Vx/Vy/Omega convention)
 *
 * STICK MAPPING (per robot_config.h, confirmed with user):
 *   RIGHT stick L/R (CH1, IBUS_CH_VX)    -> Vx     (strafe)
 *   RIGHT stick U/D (CH2, IBUS_CH_VY)    -> Vy     (forward/back)
 *   LEFT  stick L/R (CH4, IBUS_CH_OMEGA) -> Omega  (rotation)
 *
 * KINEMATICS (3-wheel kiwi, 120 deg apart):
 *   wheel_speed = -Vx*sin(theta) + Vy*cos(theta) + Omega
 *   Matching wheel angles from robot_config.h (A=210, B=330, C=90):
 *     Motor C (90 deg,  front)      = -Vx + Omega
 *     Motor A (210 deg, back-left)  = 0.5*Vx - (sqrt(3)/2)*Vy + Omega
 *     Motor B (330 deg, back-right) = 0.5*Vx + (sqrt(3)/2)*Vy + Omega
 *
 * DIRECTION CORRECTION: robot_config.h's DIR_A_CW / DIR_B_CW / DIR_C_CW
 * are applied as a final sign flip so a positive kinematic command always
 * drives the wheel the direction the maths expects, regardless of wiring.
 *
 * NOTE: physical hardware arm-switch pin was removed on request - no
 * wiring changes to the Teensy required. Arming is RC-switch-only again,
 * gated by a confirmed OFF->ON edge with sticks centered.
 */

#include "robot_config.h"
#include "IBusReader.h"
#include <math.h>

IBusReader ibus(IBUS_SERIAL);

#define IBUS_CH_LEFT_UD  2   // CH3, left stick U/D — unused axis, read for reference only

const float SQRT3_OVER_2 = 0.8660254f;

// Calibrated resting centers, captured at boot and continuously tracked
// while disarmed.
float centerVX    = IBUS_PWM_MID;
float centerVY    = IBUS_PWM_MID;
float centerOmega = IBUS_PWM_MID;

// ------------------------------------------------------------
// STATUS LED - onboard LED, gives visual feedback with no PC/serial
// needed. Patterns (checked in this priority order):
//   slow blink (500ms)     -> waiting for a valid iBUS link from receiver
//   fast blink (150ms)     -> link OK, but sticks not centered / not armed yet
//   solid ON                -> link OK, sticks centered, disarmed & ready to arm
//   solid OFF                -> ARMED - motors live, be careful
//   very fast blink (60ms)  -> failsafe / link lost while running
// ------------------------------------------------------------
#define STATUS_LED_PIN   13   // Teensy 4.1 onboard LED

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

  if (onMs == 0) { digitalWrite(STATUS_LED_PIN, LOW); return; }   // solid OFF
  if (offMs == 0) { digitalWrite(STATUS_LED_PIN, HIGH); return; } // solid ON

  unsigned long interval = ledOn ? onMs : offMs;
  if (millis() - lastToggle >= interval) {
    ledOn = !ledOn;
    lastToggle = millis();
  }
  digitalWrite(STATUS_LED_PIN, ledOn ? HIGH : LOW);
}

bool sticksAreCentered() {
  float vx = normalizedStick(IBUS_CH_OMEGA, centerOmega);
  float vy = normalizedStick(IBUS_CH_VY,    centerVY);
  float om = normalizedStick(IBUS_CH_VX,    centerVX);
  return (vx == 0.0f && vy == 0.0f && om == 0.0f);
}

void calibrateCenters() {
  Serial.println("Calibrating stick centers - DO NOT TOUCH STICKS...");

  long sumVX = 0, sumVY = 0, sumOmega = 0;
  int samples = 0;
  unsigned long calStart = millis();

  while (millis() - calStart < 1000) {
    ibus.update();
    if (!ibus.isFailsafe(IBUS_FAILSAFE_MS)) {
      sumVX    += ibus.channel(IBUS_CH_VX);
      sumVY    += ibus.channel(IBUS_CH_VY);
      sumOmega += ibus.channel(IBUS_CH_OMEGA);
      samples++;
    }
  }

  if (samples > 0) {
    centerVX    = sumVX    / samples;
    centerVY    = sumVY    / samples;
    centerOmega = sumOmega / samples;
  }

  Serial.print("Calibrated centers -> VX: ");
  Serial.print(centerVX);
  Serial.print("  VY: ");
  Serial.print(centerVY);
  Serial.print("  Omega: ");
  Serial.println(centerOmega);
}

void setup() {
  Serial.begin(BAUD_RATE);
  ibus.begin(IBUS_BAUD);

  pinMode(EN_A, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(EN_B, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(EN_C, OUTPUT); pinMode(IN3_C, OUTPUT); pinMode(IN4_C, OUTPUT);

  pinMode(STATUS_LED_PIN, OUTPUT);

  stopAllMotors();

  while (!Serial && millis() < 3000) { ; }
  Serial.println("Kiwi drive starting...");

  Serial.println("Waiting for valid iBUS signal from receiver...");
  unsigned long linkWaitStart = millis();
  unsigned long lastWarnMs = linkWaitStart;
  while (ibus.isFailsafe(IBUS_FAILSAFE_MS)) {
    ibus.update();
    setStatusLed(500, 500);
    if (millis() - lastWarnMs > 3000) {
      lastWarnMs = millis();
      Serial.print("  still waiting for receiver... (");
      Serial.print((millis() - linkWaitStart) / 1000);
      Serial.println("s) - check receiver power / bind / wiring");
    }
  }
  Serial.println("Valid iBUS signal detected.");

  calibrateCenters();
}

bool switchSeenOff = false;
bool armed = false;
bool prevSwitchOn = false;

void loop() {
  ibus.update();

  bool failsafe = ibus.isFailsafe(IBUS_FAILSAFE_MS);
  uint16_t rawEnable = ibus.channel(IBUS_CH_ENABLE);
  bool switchOn = (rawEnable > IBUS_ENABLE_THRESHOLD);

  if (!switchOn) {
    switchSeenOff = true;
    armed = false;

    if (!failsafe) {
      int rawVX    = ibus.channel(IBUS_CH_VX);
      int rawVY    = ibus.channel(IBUS_CH_VY);
      int rawOmega = ibus.channel(IBUS_CH_OMEGA);
      centerVX    += (rawVX    - centerVX)    * 0.02f;
      centerVY    += (rawVY    - centerVY)    * 0.02f;
      centerOmega += (rawOmega - centerOmega) * 0.02f;
    }
  }

  bool risingEdge = switchOn && !prevSwitchOn && switchSeenOff;
  if (risingEdge) {
    if (sticksAreCentered()) {
      armed = true;
    } else {
      armed = false;
      Serial.println("ARM BLOCKED: center all sticks, then toggle the enable switch OFF then ON again.");
    }
  }
  prevSwitchOn = switchOn;

  bool enabled = armed && switchOn && !failsafe;

  if (failsafe) {
    setStatusLed(60, 60);
  } else if (enabled) {
    setStatusLed(0, 1);
  } else if (!sticksAreCentered()) {
    setStatusLed(150, 150);
  } else {
    setStatusLed(1, 0);
  }

  if (!enabled) {
    stopAllMotors();
  } else {
    float Vx    = normalizedStick(IBUS_CH_OMEGA, centerOmega);
    float Vy    = normalizedStick(IBUS_CH_VY, centerVY);
    float Omega = normalizedStick(IBUS_CH_VX, centerVX);
    driveKiwi(Vx, Vy, Omega);
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    debugPrint(enabled, failsafe);
  }
}

float normalizedStick(uint8_t channel, float center) {
  uint16_t raw = ibus.channel(channel);
  raw = constrain(raw, IBUS_PWM_MIN, IBUS_PWM_MAX);

  float offsetFromCenter = (float)raw - center;
  if (fabs(offsetFromCenter) <= IBUS_DEADZONE) {
    return 0.0f;
  }

  float span = (offsetFromCenter > 0) ? (IBUS_PWM_MAX - center) : (center - IBUS_PWM_MIN);
  float value = (float)offsetFromCenter / span;
  return constrain(value, -1.0f, 1.0f);
}

void driveKiwi(float Vx, float Vy, float Omega) {
  float motorOnePower   = -Vx + Omega;
  float motorTwoPower   = (Vx * 0.5f) - (Vy * SQRT3_OVER_2) + Omega;
  float motorThreePower = (Vx * 0.5f) + (Vy * SQRT3_OVER_2) + Omega;

  float maxPower = findAbsoluteMax(motorOnePower, motorTwoPower, motorThreePower);
  if (maxPower > 1.0f) {
    motorOnePower   /= maxPower;
    motorTwoPower   /= maxPower;
    motorThreePower /= maxPower;
  }

  driveMotor(EN_C, IN3_C, IN4_C, motorOnePower   * DIR_C_CW);
  driveMotor(EN_A, IN1,   IN2,   motorTwoPower   * DIR_A_CW);
  driveMotor(EN_B, IN3,   IN4,   motorThreePower * DIR_B_CW);
}

float findAbsoluteMax(float a, float b, float c) {
  float m = fabs(a);
  if (fabs(b) > m) m = fabs(b);
  if (fabs(c) > m) m = fabs(c);
  return (m < 1.0f) ? 1.0f : m;
}

void driveMotor(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin, float power) {
  power = constrain(power, -1.0f, 1.0f);

  if (power > 0.0f) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (power < 0.0f) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }

  int pwmValue = (int)(fabs(power) * MOTOR_PWM_MAX);
  analogWrite(enPin, pwmValue);
}

void stopAllMotors() {
  driveMotor(EN_A, IN1,   IN2,   0.0f);
  driveMotor(EN_B, IN3,   IN4,   0.0f);
  driveMotor(EN_C, IN3_C, IN4_C, 0.0f);
}

void debugPrint(bool enabled, bool failsafe) {
  if (failsafe) {
    Serial.println("!! FAILSAFE - no valid iBUS frame recently - motors stopped !!");
    return;
  }

  float Vx    = normalizedStick(IBUS_CH_OMEGA, centerOmega);
  float Vy    = normalizedStick(IBUS_CH_VY, centerVY);
  float Omega = normalizedStick(IBUS_CH_VX, centerVX);

  Serial.print("ENABLE: ");
  Serial.print(enabled ? "ON " : "OFF");
  Serial.print("  Vx: ");
  Serial.print(Vx, 2);
  Serial.print("  Vy: ");
  Serial.print(Vy, 2);
  Serial.print("  Omega: ");
  Serial.print(Omega, 2);
  Serial.print("  [raw VX:");
  Serial.print(ibus.channel(IBUS_CH_VX));
  Serial.print(" VY:");
  Serial.print(ibus.channel(IBUS_CH_VY));
  Serial.print(" OMEGA:");
  Serial.print(ibus.channel(IBUS_CH_OMEGA));
  Serial.print(" | centers ");
  Serial.print(centerVX, 1);
  Serial.print("/");
  Serial.print(centerVY, 1);
  Serial.print("/");
  Serial.print(centerOmega, 1);
  Serial.println("]");
}