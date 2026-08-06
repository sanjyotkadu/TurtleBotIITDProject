#include "motorControl.h"
#include "robot_config.h"
#include <math.h>

void motorsInit() {
  pinMode(EN_A, OUTPUT); pinMode(IN1, OUTPUT);   pinMode(IN2, OUTPUT);
  pinMode(EN_B, OUTPUT); pinMode(IN3, OUTPUT);   pinMode(IN4, OUTPUT);
  pinMode(EN_C, OUTPUT); pinMode(IN3_C, OUTPUT); pinMode(IN4_C, OUTPUT);

  stopAllMotors();
}

void driveMotor(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin, float power) {
  power = constrain(power, -1.0f, 1.0f);

  if (power > 0.0f) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(enPin, (int)(power * MOTOR_PWM_MAX));
  } else if (power < 0.0f) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(enPin, (int)(-power * MOTOR_PWM_MAX));
  } else {
    // Stopped. On the L298N, EN high + both inputs equal shorts the motor
    // windings (fast brake); EN low disables the bridge (coast).
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
#if MOTOR_BRAKE_ON_STOP
    analogWrite(enPin, MOTOR_PWM_MAX);   // brake — fast electrical stop
#else
    analogWrite(enPin, 0);               // coast — motor free-wheels
#endif
  }
}

void stopAllMotors() {
  driveMotor(EN_A, IN1,   IN2,   0.0f);
  driveMotor(EN_B, IN3,   IN4,   0.0f);
  driveMotor(EN_C, IN3_C, IN4_C, 0.0f);
}
