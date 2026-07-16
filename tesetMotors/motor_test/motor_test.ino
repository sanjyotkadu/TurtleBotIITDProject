/*
 * Motor Test — Duttallbot / TurtleBot IITD
 * ---------------------------------------------------------------------
 * Standalone diagnostic: no iBUS, no joystick, no kinematics.
 * Runs each motor FORWARD, then BACKWARD, then STOPS, one at a time,
 * so you can visually confirm each motor + its wiring works before
 * bringing the RC/kinematics layer back into the picture.
 *
 * Uses only robot_config.h (untouched) for pin definitions.
 *
 * WHAT TO WATCH FOR:
 *   - Does the motor spin at all? (checks power wiring + EN pin)
 *   - Does it spin the "right" way for FORWARD vs BACKWARD?
 *     (checks IN1/IN2 or IN3/IN4 or IN3_C/IN4_C wiring)
 *   - Does it stutter, stall, or sound like it's fighting itself?
 *     (often a sign of a loose connection or a driver channel issue)
 *
 * TEST SEQUENCE (repeats forever):
 *   Motor A: forward 2s -> stop 1s -> backward 2s -> stop 1s
 *   Motor B: forward 2s -> stop 1s -> backward 2s -> stop 1s
 *   Motor C: forward 2s -> stop 1s -> backward 2s -> stop 1s
 *   (2s pause, then repeat)
 */

#include "robot_config.h"

#define TEST_PWM        150   // moderate speed (0-255) - safe for bench testing
#define RUN_TIME_MS      2000
#define PAUSE_TIME_MS    1000
#define CYCLE_PAUSE_MS   2000

void setup() {
  Serial.begin(BAUD_RATE);

  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(EN_B, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(EN_C, OUTPUT);
  pinMode(IN3_C, OUTPUT);
  pinMode(IN4_C, OUTPUT);

  stopAll();

  while (!Serial && millis() < 3000) {
    ; // wait briefly for USB serial on boot
  }
  Serial.println("Motor test starting...");
  Serial.println("Each motor: FORWARD 2s -> STOP -> BACKWARD 2s -> STOP");
}

void loop() {
  testMotor("A", EN_A, IN1, IN2);
  testMotor("B", EN_B, IN3, IN4);
  testMotor("C", EN_C, IN3_C, IN4_C);

  Serial.println("--- cycle complete, pausing ---");
  delay(CYCLE_PAUSE_MS);
}

void testMotor(const char* name, uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin) {
  Serial.print("Motor ");
  Serial.print(name);
  Serial.println(": FORWARD");
  digitalWrite(in1Pin, HIGH);
  digitalWrite(in2Pin, LOW);
  analogWrite(enPin, TEST_PWM);
  delay(RUN_TIME_MS);

  stopMotor(enPin, in1Pin, in2Pin);
  delay(PAUSE_TIME_MS);

  Serial.print("Motor ");
  Serial.print(name);
  Serial.println(": BACKWARD");
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  analogWrite(enPin, TEST_PWM);
  delay(RUN_TIME_MS);

  stopMotor(enPin, in1Pin, in2Pin);
  delay(PAUSE_TIME_MS);
}

void stopMotor(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin) {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);
  analogWrite(enPin, 0);
}

void stopAll() {
  stopMotor(EN_A, IN1, IN2);
  stopMotor(EN_B, IN3, IN4);
  stopMotor(EN_C, IN3_C, IN4_C);
}
