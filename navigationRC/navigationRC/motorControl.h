#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

// ============================================================
// motorControl.h — Low-level H-bridge (L298N) motor driver API
//
// Knows nothing about kinematics or the RC link. It just turns a
// signed, normalized power command (-1.0 .. +1.0) into direction
// pins + a PWM duty cycle on an L298N channel.
// ============================================================

#include <Arduino.h>

// Configure all motor direction/enable pins as outputs and leave
// every motor stopped. Call once from setup().
void motorsInit();

// Drive a single L298N channel.
//   enPin        : PWM "enable" pin (speed)
//   in1Pin/in2Pin: direction pins
//   power        : -1.0 (full reverse) .. 0 (stop) .. +1.0 (full forward)
// Magnitude sets PWM (scaled to MOTOR_PWM_MAX), sign sets direction.
void driveMotor(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin, float power);

// Immediately stop all three wheels (A, B, C).
void stopAllMotors();

#endif // MOTOR_CONTROL_H
