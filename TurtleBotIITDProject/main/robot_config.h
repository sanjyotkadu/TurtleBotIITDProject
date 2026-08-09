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
#define DIR_C_CW  +1

// ------------------------------------------------------------
// WHEEL LAYOUT (top view)
//         FRONT
//           C (90°)
//          / \.
//         /   \.
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

// ------------------------------------------------------------
// CHANNEL MAPPING — CONFIRMED BY BENCH TEST (not the FlySky "standard"
// guess previously assumed here). One stick/switch moved at a time,
// watching raw iBUS channel values change:
//
//   CH0 = RIGHT stick L/R   (min 1065, center 1474, max 1899) -> Vx (strafe)
//   CH1 = RIGHT stick U/D   (min 1000, center 1498, max 1998) -> Vy (fwd/back)
//   CH2 = LEFT  stick U/D   (min 1000, center 1412, max 1870) -> unused,
//         no spring return (stays wherever last left) - do not use for
//         anything requiring a resting center.
//   CH3 = LEFT  stick L/R   (min 1000, center 1488, max 1987) -> Omega (rotation)
//   CH4 = arm switch        (min 1000, max 2000)               -> enable
//   CH5 = aux switch        (up = 1000, down = 2000, inverted) -> unused
//
// NOTE: this means RIGHT stick currently drives strafe/fwd-back, and
// LEFT stick drives rotation. If you'd rather have LEFT stick do
// left/right strafe instead, swap the CH0/CH3 assignment below (and
// nothing else needs to change - normalizedStick() just reads whichever
// channel index you give it).
// ------------------------------------------------------------
#define IBUS_CH_VX              0   // CH0 — right stick L/R -> strafe
#define IBUS_CH_VY              1   // CH1 — right stick U/D -> forward/back
#define IBUS_CH_OMEGA           3   // CH3 — left stick L/R  -> rotation
#define IBUS_CH_ENABLE          4   // CH4 — 2-pos switch    -> motor enable / kill switch

// Raw PWM range iBUS reports per channel (microseconds, FlySky standard)
#define IBUS_PWM_MIN            1000
#define IBUS_PWM_MID            1500
#define IBUS_PWM_MAX            2000
#define IBUS_DEADZONE           30      // +/- counts around center ignored (stick drift)
#define IBUS_ENABLE_THRESHOLD   1500    // switch HIGH position must exceed this to enable

// ------------------------------------------------------------
// RC DRIVE TUNING
// ------------------------------------------------------------
#define MAX_LINEAR_SPEED_MM_S    130.0   // full-stick translation speed
#define MAX_ANGULAR_SPEED_RAD_S  2.0     // full-stick rotation speed
#define MOTOR_PWM_MAX            255     // analogWrite ceiling

// Active braking on stop. When a wheel is commanded to exactly 0
// (neutral / PID idle), 1 = short the motor windings for a fast
// electrical stop, 0 = let the motor coast (free-wheel) to a stop.
// On a stand a coasting omni wheel free-spins for a couple of seconds;
// braking stops it almost at once. On the ground the difference is much
// smaller. The emergency stop is unaffected — it always cuts drive.
#define MOTOR_BRAKE_ON_STOP      1

// Wheel mounting angles (deg), standard math convention:
// 0 deg = robot's right (+x), 90 deg = front (+y), CCW positive.
// Matches the physical layout diagram above (C front, A back-left, B back-right).
#define WHEEL_A_ANGLE_DEG   210.0
#define WHEEL_B_ANGLE_DEG   330.0
#define WHEEL_C_ANGLE_DEG    90.0

// ------------------------------------------------------------
// PID — per-wheel closed-loop velocity control (see PID.h / PID.cpp)
//
// Uses the Arduino PID library (PID_v1 by Brett Beauregard — install it
// from Library Manager). The open-loop path sends the kinematic power
// straight to PWM. With PID enabled, each wheel instead measures its
// ACTUAL speed from its encoder and adjusts PWM to hit the commanded
// speed — so all three wheels track together even under uneven load /
// battery sag.
//
// TUNING IS PER-ROBOT. The gains below are conservative starting
// points, NOT final values — you MUST tune them on your hardware
// (see the tuning recipe in PID.cpp). Same gains for all three wheels
// is fine to start; split them per wheel later only if one motor
// behaves differently. Ki/Kd are in standard per-second units; the
// library scales them by the sample time, so do NOT pre-scale them.
// ------------------------------------------------------------
#define PID_KP               0.002/// 0.0045// 0.0020   // proportional  (power per mm/s of error)
#define PID_KI                0.004// 0.0040   // integral      (per second)
#define PID_KD                0.0// 0.0000   // derivative    (start at 0, add last)

// Control-loop rate. PID runs on a FIXED interval (not every loop()) so
// dt is stable and each interval accumulates enough encoder counts to
// measure speed cleanly. 20ms = 50Hz is a good default for these motors.
#define PID_SAMPLE_TIME_MS     20

// Top wheel speed (mm/s) at full PWM. A normalized command of 1.0 maps to
// this as the velocity setpoint. MEASURE THIS: drive one wheel at PWM 255
// on the ground and clock its speed; set this to that value. Too high and
// the wheel can never reach setpoint (integral winds up); too low and it
// saturates before full stick.
#define PID_MAX_WHEEL_SPEED_MM_S   130.0

// Encoder direction alignment. The encoder's +/- count direction (set by
// C1/C2 wiring) must match the sign of the PRE-direction kinematic command
// for that wheel, or PID will fight itself (positive feedback -> runaway).
// If a wheel accelerates uncontrollably the instant PID engages, flip its
// sign here (+1 <-> -1). See the "MUST DO before enabling" note in PID.cpp.
#define PID_ENC_SIGN_A         -1
#define PID_ENC_SIGN_B         -1
#define PID_ENC_SIGN_C         -1

// Zero-command deadband. When a wheel's normalized command is smaller
// than this, that wheel is hard-stopped (PWM 0) and its integrator is
// cleared, instead of running PID toward "0 speed". Without this the
// integral term left over from the last move keeps a little PWM on the
// wheel at neutral (and holds the front wheel powered during pure
// straight-line moves). 0.01 = 1% of full stick.
#define PID_CMD_DEADBAND       0.01

// ------------------------------------------------------------
// EMERGENCY STOP (see emergencyStop.h / emergencyStop.cpp)
//
// Two INDEPENDENT trigger sources, OR'd together and LATCHING. Either
// one cuts all three motors the instant it asserts:
//
//   1. HARDWARE PIN  — a physical button on a spare digital pin, served
//      by a hardware interrupt. This is the truly independent path: it
//      fires and kills the motors even if loop() is stuck, because the
//      ISR writes the L298N enable pins LOW directly.
//   2. RC SWITCH     — a FlySky aux switch (CH5, otherwise unused). Handy
//      to reach from the transmitter, but it rides the RC link, so it is
//      NOT independent of the software / iBUS parsing. Keep the hardware
//      button as your real E-stop; treat the RC switch as a convenience.
//
// Recovery is deliberate: once tripped it stays stopped until the trigger
// is released AND the robot is disarmed (kill switch off), then re-armed.
// ------------------------------------------------------------
#define ESTOP_USE_HARDWARE_PIN  1     // set 0 if you have no physical button
#define ESTOP_PIN               7     // spare digital pin (free in this map)
#define ESTOP_PIN_ACTIVE_LOW    1     // 1 = pressed pulls pin LOW (button to GND,
                                      //     INPUT_PULLUP). For a fail-safe wire a
                                      //     NORMALLY-CLOSED button so a cut wire
                                      //     also trips: keep ACTIVE_LOW=1 and wire
                                      //     the NC contact between pin 7 and GND.

#define ESTOP_USE_RC_SWITCH     1     // set 0 to ignore the FlySky switch
#define ESTOP_RC_CHANNEL        5     // CH5 aux switch (up=1000, down=2000)
#define ESTOP_RC_THRESHOLD      1500  // asserted when the channel is on the
                                      // "stop" side of this value
#define ESTOP_RC_ASSERT_ABOVE   1     // 1 = stop when raw > threshold (switch
                                      //     DOWN). Set 0 if your switch is
                                      //     reversed (stop when raw < threshold).

#define ESTOP_RESET_DEBOUNCE_MS 50    // trigger must read "safe" this long to clear

// ------------------------------------------------------------
// IMU — Adafruit BNO085 (BNO08x) 9-DOF, on I2C bus Wire2
//
// STEMMA QT / Qwiic wiring (4 wires, Teensy is 3.3V native — no shift):
//   SDA -> pin 25 (SDA2)    SCL -> pin 24 (SCL2)
//   3V  -> 3.3V             GND -> GND
// The Qwiic cable carries no RST/INT line, so we run in polling mode
// with the reset pin disabled. Wire RST to a spare GPIO later if you
// want the driver to hard-reset the sensor on fault (optional).
//
// We use the GAME ROTATION VECTOR report: gyro+accel fusion with NO
// magnetometer, so the robot's motors / steel can't corrupt heading.
// Trade-off: yaw is relative to power-on orientation (not true north) —
// exactly what we want for local heading-hold / go-to-pose.
// ------------------------------------------------------------
#define BNO08X_I2C_ADDR          0x4A     // Adafruit default (0x4B if ADR pin high)
#define BNO08X_RESET_PIN         -1       // -1 = no reset pin wired (Qwiic cable)
#define IMU_REPORT_INTERVAL_US   10000    // 10 ms = 100 Hz orientation updates
#define IMU_TIMEOUT_MS           200      // no fresh sample within this = unhealthy

// Mounting sign: flip to -1 if rotating the robot CCW (top view) makes
// heading DECREASE. VERIFY on the bench with imuTest so +Omega and
// +heading agree with the kinematics' CCW-positive convention.
#define IMU_YAW_SIGN             (-1)

// ------------------------------------------------------------
// HEADING HOLD — uses the IMU to resist unwanted yaw drift while
// translating (Vx/Vy) with the rotation stick centered. Purely a trim:
// if the pilot commands rotation, heading-hold releases immediately and
// re-latches to the new heading once the stick returns to center.
// ------------------------------------------------------------
#define HEADING_HOLD_KP              1.0 //1.5f   // Omega correction per rad of heading error
#define HEADING_HOLD_MAX_OMEGA       0.35f  // clamp on the correction, -1.0..1.0 scale
#define HEADING_HOLD_OMEGA_DEADZONE  0.05f  // |Omega stick| below this = "not turning"

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