/*
 * Kiwi Holonomic Drive — Duttallbot / TurtleBot IITD
 * ---------------------------------------------------------------------
 * main.ino — top-level sketch. Owns only setup()/loop() and wires the
 * modules together; all real work lives in the modules it calls:
 *
 *   robot_config.h    — pins, channel map, tuning constants (untouched)
 *   IBusReader.h      — raw FlySky iBUS protocol parser (untouched)
 *   RCControl.*       — RC link, stick normalization, arm state machine
 *   kinematics.*      — 3-wheel kiwi inverse kinematics (pure math)
 *   motorControl.*    — L298N H-bridge driver (driveMotor/stopAllMotors)
 *   encoder.*         — quadrature wheel-encoder counting
 *   diagnostics.*     — status LED + serial debug output
 *   startupSequence.* — blocking boot flow (link wait + calibration)
 *
 * Control flow:
 *   setup() -> init every module, then run startupSequence().
 *   loop()  -> rcUpdate() -> if enabled, driveKiwi(Vx,Vy,Omega); else
 *              stopAllMotors(). LED + throttled debug each iteration.
 */

#include "robot_config.h"
#include "RCControl.h"
#include "kinematics.h"
#include "motorControl.h"
#include "encoder.h"
#include "diagnostics.h"
#include "startupSequence.h"
#include <math.h>

// Orchestrate one drive command: kinematics -> direction correction ->
// motors, then hand the resulting powers/PWM to diagnostics.
static void driveturBot(float Vx, float Vy, float Omega) {
  WheelPowers wp = kiwiMix(Vx, Vy, Omega);

  // Apply physical wiring direction correction from robot_config.h so a
  // positive kinematic command always spins the wheel the intended way.
  driveMotor(EN_C, IN3_C, IN4_C, wp.front_C     * DIR_C_CW);
  driveMotor(EN_A, IN1,   IN2,   wp.backLeft_A  * DIR_A_CW);
  driveMotor(EN_B, IN3,   IN4,   wp.backRight_B * DIR_B_CW);

  diagSetWheelDebug(wp.front_C, wp.backLeft_A, wp.backRight_B,
                    (int)(fabs(wp.front_C)     * MOTOR_PWM_MAX),
                    (int)(fabs(wp.backLeft_A)  * MOTOR_PWM_MAX),
                    (int)(fabs(wp.backRight_B) * MOTOR_PWM_MAX));
}

void setup() {
  Serial.begin(BAUD_RATE);

  rcInit();
  motorsInit();
  encoderInit();
  diagInit();

  startupSequence();
}

void loop() {
  rcUpdate();

  bool failsafe = rcFailsafe();
  bool enabled  = rcEnabled();

  statusLedUpdate(failsafe, enabled, rcSticksCentered());

  if (!enabled) {
    stopAllMotors();
  } else {
    driveturBot(rcVx(), rcVy(), rcOmega());
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    debugPrint(enabled, failsafe);
  }
}
