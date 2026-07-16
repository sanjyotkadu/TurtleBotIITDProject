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
#define EN_A   9    // Motor A PWM speed ..........MOT 2
#define IN1    8    // Motor A direction pin 1
#define IN2    6    // Motor A direction pin 2
#define EN_B   2    // Motor B PWM speed..........MOT 1
#define IN3    4    // Motor B direction pin 1
#define IN4    3    // Motor B direction pin 2

// ------------------------------------------------------------
// MOTOR DRIVER 2 — Motor C (L298N)
// ------------------------------------------------------------
#define EN_C   5    // Motor C PWM speed.........MOT 3
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
// FLYSKY FS-iA6B RECEIVER — iBUS PROTOCOL
// iBUS is a single-wire digital serial link (all channels
// multiplexed), so we use one free hardware serial port instead
// of burning a pin per channel like classic PWM receivers need.
//
// Wiring:
//   FS-iA6B "iBUS" pad  -> Teensy 4.1 pin 0 (RX1)
//   FS-iA6B GND          -> Teensy GND
//   FS-iA6B VCC          -> Teensy 3.3V or 5V per receiver spec
//   (Teensy pin 1 / TX1 is unused — iBUS servo output is RX only)
//
// Pins 0/1 were the only hardware-serial-capable pins not
// already committed to motors/encoders in this config.
// ------------------------------------------------------------
#define IBUS_SERIAL             Serial1   // uses pins 0(RX1)/1(TX1)
#define IBUS_BAUD               115200
#define IBUS_NUM_CHANNELS       14
#define IBUS_FAILSAFE_MS        500       // stop motors if no valid frame in this long

// Channel index (0-based) -> stick function
// Standard FlySky i6/i6X mapping: CH1=Roll/Aileron, CH2=Pitch/Elevator,
// CH3=Throttle, CH4=Yaw/Rudder, CH5/CH6=aux switches (SWA/SWB/SWC/SWD)
#define IBUS_CH_VX              0   // CH1 — right stick L/R  -> strafe
#define IBUS_CH_VY              1   // CH2 — right stick U/D  -> forward/back
#define IBUS_CH_OMEGA           3   // CH4 — left stick L/R   -> rotation
#define IBUS_CH_ENABLE          4   // CH5 — 2/3-pos switch   -> motor enable / kill switch

// Raw PWM range iBUS reports per channel (microseconds, FlySky standard)
#define IBUS_PWM_MIN            1000
#define IBUS_PWM_MID            1500
#define IBUS_PWM_MAX            2000
#define IBUS_DEADZONE           30      // +/- counts around center ignored (stick drift)
#define IBUS_ENABLE_THRESHOLD   1500    // switch HIGH position must exceed this to enable

// ------------------------------------------------------------
// RC DRIVE TUNING
// ------------------------------------------------------------
#define MAX_LINEAR_SPEED_MM_S    300.0   // full-stick translation speed
#define MAX_ANGULAR_SPEED_RAD_S  2.0     // full-stick rotation speed
#define MOTOR_PWM_MAX            255     // analogWrite ceiling

// Wheel mounting angles (deg), standard math convention:
// 0 deg = robot's right (+x), 90 deg = front (+y), CCW positive.
// Matches the physical layout diagram above (C front, A back-left, B back-right).
#define WHEEL_A_ANGLE_DEG   210.0
#define WHEEL_B_ANGLE_DEG   330.0
#define WHEEL_C_ANGLE_DEG    90.0

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