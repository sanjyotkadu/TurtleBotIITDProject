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
 *   This is exactly the formula in the pasted pseudocode:
 *     motorOnePower   (-> Motor C) = -Vx            ... + Omega
 *     motorTwoPower   (-> Motor A) =  Vx/2 - Vy*sqrt(3)/2  ... + Omega
 *     motorThreePower (-> Motor B) =  Vx/2 + Vy*sqrt(3)/2  ... + Omega
 *
 * BUG FIX vs pasted pseudocode: original code only called SetPower()
 * inside the "if any |power|>1" block, so motors never moved unless
 * saturated. Fixed here: normalize only when needed, but always drive.
 *
 * DIRECTION CORRECTION: robot_config.h's DIR_A_CW / DIR_B_CW / DIR_C_CW
 * (currently all -1, from your physical CW rotation test) are applied
 * as a final sign flip so a positive kinematic command always drives
 * the wheel the direction the maths expects, regardless of wiring polarity.
 */

#include "robot_config.h"
#include "IBusReader.h"
#include <math.h>

IBusReader ibus(IBUS_SERIAL);

#define IBUS_CH_LEFT_UD  2   // CH3, left stick U/D — unused axis, read for reference only

const float SQRT3_OVER_2 = 0.8660254f;

// Calibrated resting centers, captured at boot. Spring-centered sticks
// (especially rudder-type, like your rotation axis) often don't return
// to exactly 1500us — calibrating removes that drift instead of just
// widening the deadzone, which would cost you low-speed precision.
int centerVX    = IBUS_PWM_MID;
int centerVY    = IBUS_PWM_MID;
int centerOmega = IBUS_PWM_MID;

void calibrateCenters() {
  Serial.println("Calibrating stick centers - DO NOT TOUCH STICKS...");

  long sumVX = 0, sumVY = 0, sumOmega = 0;
  int samples = 0;
  unsigned long calStart = millis();

  while (millis() - calStart < 1000) {  // sample for 1 second
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

  // Motor A pins
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  // Motor B pins
  pinMode(EN_B, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  // Motor C pins
  pinMode(EN_C, OUTPUT);
  pinMode(IN3_C, OUTPUT);
  pinMode(IN4_C, OUTPUT);

  stopAllMotors();

  while (!Serial && millis() < 3000) {
    ; // wait briefly for USB serial on boot
  }
  Serial.println("Kiwi drive starting...");

  calibrateCenters();
}

void loop() {
  ibus.update();

  bool failsafe = ibus.isFailsafe(IBUS_FAILSAFE_MS);
  uint16_t rawEnable = ibus.channel(IBUS_CH_ENABLE);
  bool enabled = (rawEnable > IBUS_ENABLE_THRESHOLD) && !failsafe;

  if (!enabled) {
    stopAllMotors();
  } else {
    // NOTE: On this transmitter, testing showed CH1 (IBUS_CH_VX) is
    // physically wired to the LEFT stick L/R, and CH4 (IBUS_CH_OMEGA)
    // is physically wired to the RIGHT stick L/R — opposite of the
    // comment in robot_config.h. We swap which macro feeds which
    // variable here (config file itself is untouched) so that:
    //   RIGHT stick L/R -> Vx (strafe)      [reads IBUS_CH_OMEGA's index]
    //   LEFT  stick L/R -> Omega (rotation) [reads IBUS_CH_VX's index]
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

// ---- Joystick reading -------------------------------------------------

// Maps raw 1000-2000us iBUS value to -1.0..1.0, using a calibrated
// center (captured at boot) rather than assuming exactly IBUS_PWM_MID,
// and applying IBUS_DEADZONE around that calibrated center.
float normalizedStick(uint8_t channel, int center) {
  uint16_t raw = ibus.channel(channel);
  raw = constrain(raw, IBUS_PWM_MIN, IBUS_PWM_MAX);

  int offsetFromCenter = (int)raw - center;
  if (abs(offsetFromCenter) <= IBUS_DEADZONE) {
    return 0.0f;
  }

  float span = (offsetFromCenter > 0) ? (IBUS_PWM_MAX - center) : (center - IBUS_PWM_MIN);
  float value = (float)offsetFromCenter / span;
  return constrain(value, -1.0f, 1.0f);
}

// ---- Kinematics --------------------------------------------------------

void driveKiwi(float Vx, float Vy, float Omega) {
  // Wheel formula: -Vx*sin(theta) + Vy*cos(theta) + Omega
  float motorOnePower   = -Vx + Omega;                              // Motor C, 90 deg
  float motorTwoPower   = (Vx * 0.5f) - (Vy * SQRT3_OVER_2) + Omega; // Motor A, 210 deg
  float motorThreePower = (Vx * 0.5f) + (Vy * SQRT3_OVER_2) + Omega; // Motor B, 330 deg

  // Normalize only if any wheel command exceeds full power
  float maxPower = findAbsoluteMax(motorOnePower, motorTwoPower, motorThreePower);
  if (maxPower > 1.0f) {
    motorOnePower   /= maxPower;
    motorTwoPower   /= maxPower;
    motorThreePower /= maxPower;
  }

  // Apply physical wiring direction correction from robot_config.h
  driveMotor(EN_C, IN3_C, IN4_C, motorOnePower   * DIR_C_CW);
  driveMotor(EN_A, IN1,   IN2,   motorTwoPower   * DIR_A_CW);
  driveMotor(EN_B, IN3,   IN4,   motorThreePower * DIR_B_CW);
}

float findAbsoluteMax(float a, float b, float c) {
  float m = fabs(a);
  if (fabs(b) > m) m = fabs(b);
  if (fabs(c) > m) m = fabs(c);
  return (m < 1.0f) ? 1.0f : m; // never divide by less than 1
}

// ---- Motor output --------------------------------------------------------

// power expected in range -1.0..1.0
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

// ---- Debug ---------------------------------------------------------------

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
  Serial.println(Omega, 2);
}
