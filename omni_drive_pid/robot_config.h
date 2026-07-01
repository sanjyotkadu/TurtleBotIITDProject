#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

// ============================================================
// robot_config.h — TurtleBot IITD Central Configuration
//
// All pin assignments and robot parameters are defined here.
// Include in any sketch with: #include "robot_config.h"
// Change any value here and it updates across the whole project.
// ============================================================

// ------------------------------------------------------------
// ENCODER — Counts Per Revolution (CPR)
// Measured by manually rotating each wheel one full revolution.
// All motors ~2115 counts/rev. Offset: ±10 counts.
// ------------------------------------------------------------
#define MOTOR_A_CPR      2115
#define MOTOR_B_CPR      2115
#define MOTOR_C_CPR      2115
#define ENCODER_OFFSET   10     // ±10 counts tolerance per revolution

// ------------------------------------------------------------
// WHEEL GEOMETRY (Omni Wheel)
// Screw-to-screw (hub diameter)      : 35.0mm
// Roller-to-roller (rolling diameter): 37.3mm  ← used for calculations
// ------------------------------------------------------------
#define WHEEL_DIAMETER_MM       37.3   // roller-to-roller
#define WHEEL_CIRCUMFERENCE_MM  (WHEEL_DIAMETER_MM * 3.14159265)

// Distance per encoder count = 117.1mm / 2115 = ~0.0554mm
#define MOTOR_A_MM_PER_COUNT  (WHEEL_CIRCUMFERENCE_MM / MOTOR_A_CPR)
#define MOTOR_B_MM_PER_COUNT  (WHEEL_CIRCUMFERENCE_MM / MOTOR_B_CPR)
#define MOTOR_C_MM_PER_COUNT  (WHEEL_CIRCUMFERENCE_MM / MOTOR_C_CPR)

// ------------------------------------------------------------
// ROBOT CHASSIS GEOMETRY
// Chassis plate diameter             : 162.0mm (16.2cm)
// Chassis plate radius               : 81.0mm
// Wheel protrusion beyond chassis    : 3.0mm
// Robot radius (center to wheel)     : 84.0mm
// ------------------------------------------------------------
#define CHASSIS_DIAMETER_MM    162.0
#define CHASSIS_RADIUS_MM       81.0
#define WHEEL_PROTRUSION_MM      3.0
#define ROBOT_RADIUS_MM         84.0

// ------------------------------------------------------------
// ENCODER COUNTS FOR EXACTLY 360° ROTATION
// Measured empirically on flat floor.
// ------------------------------------------------------------
#define COUNTS_360_A  7566
#define COUNTS_360_B  7813
#define COUNTS_360_C  7634

// ------------------------------------------------------------
// ROTATION DIRECTION FOR CLOCKWISE TURN
// Confirmed via direction test: all motors BWD = CW
// +1 = forward = CW,  -1 = backward = CW
// ------------------------------------------------------------
#define DIR_A_CW  -1
#define DIR_B_CW  -1
#define DIR_C_CW  -1

// ------------------------------------------------------------
// WHEEL LAYOUT (top view)
//         FRONT
//           C (90°)
//          / \
//         /   \
//        A     B
//    (210°)   (330°)
// ------------------------------------------------------------
#define MOTOR_FRONT       'C'
#define MOTOR_BACK_LEFT   'A'
#define MOTOR_BACK_RIGHT  'B'

// ------------------------------------------------------------
// MOTOR DRIVER 1 — Motors A & B (L298N)
// ------------------------------------------------------------
#define EN_A   9    // Motor A PWM speed
#define IN1    8    // Motor A direction pin 1
#define IN2    6    // Motor A direction pin 2
#define EN_B   2    // Motor B PWM speed
#define IN3    4    // Motor B direction pin 1
#define IN4    3    // Motor B direction pin 2

// ------------------------------------------------------------
// MOTOR DRIVER 2 — Motor C (L298N)
// ------------------------------------------------------------
#define EN_C   5    // Motor C PWM speed
#define IN3_C  10   // Motor C direction pin 1
#define IN4_C  11   // Motor C direction pin 2

// ------------------------------------------------------------
// ENCODER PINS (VCC = 3.3V — Teensy 4.1 NOT 5V tolerant)
// ------------------------------------------------------------
#define ENC_A1  15  // Motor A C1 (interrupt)
#define ENC_A2  16  // Motor A C2 (direction)
#define ENC_B1  12  // Motor B C1 (interrupt)
#define ENC_B2  14  // Motor B C2 (direction)
#define ENC_C1  17  // Motor C C1 (interrupt)
#define ENC_C2  18  // Motor C C2 (direction)

// ------------------------------------------------------------
// SERIAL
// ------------------------------------------------------------
#define BAUD_RATE  115200

// ------------------------------------------------------------
// QUICK REFERENCE — Pin Allocation Summary
//
//  Pin  2  → EN_B   (Motor B PWM enable)
//  Pin  3  → IN4    (Motor B direction)
//  Pin  4  → IN3    (Motor B direction)
//  Pin  5  → EN_C   (Motor C PWM enable)
//  Pin  6  → IN2    (Motor A direction)
//  Pin  8  → IN1    (Motor A direction)
//  Pin  9  → EN_A   (Motor A PWM enable)
//  Pin 10  → IN3_C  (Motor C direction)
//  Pin 11  → IN4_C  (Motor C direction)
//  Pin 12  → ENC_B1 (Motor B encoder C1 — interrupt)
//  Pin 14  → ENC_B2 (Motor B encoder C2 — direction)
//  Pin 15  → ENC_A1 (Motor A encoder C1 — interrupt)
//  Pin 16  → ENC_A2 (Motor A encoder C2 — direction)
//  Pin 17  → ENC_C1 (Motor C encoder C1 — interrupt)
//  Pin 18  → ENC_C2 (Motor C encoder C2 — direction)
//
//  3.3V → Encoder VCC (all 3 motors)
//  5V   → L298N logic supply (VSS)
//  12V  → L298N motor supply (VS) [external]
//  GND  → Common ground (Teensy + L298N + encoders)
// ------------------------------------------------------------

#endif // ROBOT_CONFIG_H