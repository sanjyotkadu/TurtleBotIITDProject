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
 *   imu.*             — BNO085 heading (yaw) + heading-hold trim
 *   diagnostics.*      — status LED + serial debug output
 *   startupSequence.* — blocking boot flow (link wait + calibration)
 *   navigation.*       — blocking autonomous demo (straight->turn->straight),
 *                        triggered by CH2 stick-up (or 'n' over Serial) while armed
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
#include "imu.h"
#include "diagnostics.h"
#include "startupSequence.h"
#include "PID.h"
#include "emergencyStop.h"
#include "navigation.h"
#include <math.h>

// Set to 0 to fall back to the original open-loop path (power -> PWM
// directly, no encoder feedback). Keep it easy to disable so you can
// A/B against open-loop and bail out fast if PID misbehaves on the bench.
#define USE_PID  1

// Set to 0 to disable the IMU heading-hold trim entirely (raw Omega
// stick passes straight through, exactly like before the IMU existed).
#define USE_IMU_HEADING_HOLD  1

// Orchestrate one drive command: kinematics -> direction correction ->
// motors, then hand the resulting powers/PWM to diagnostics.
static void driveturBot(float Vx, float Vy, float Omega) {
#if USE_IMU_HEADING_HOLD
  // Trim only — replaces a near-zero Omega with a small IMU-derived
  // correction; any real stick rotation still passes straight through.
  Omega = imu_applyHeadingHold(Omega);
#endif

  WheelPowers wp = kiwiMix(Vx, Vy, Omega);

#if USE_PID
  // Close the loop: replace the open-loop powers with encoder-corrected
  // ones. Still in the pre-direction frame, so the DIR_x_CW correction
  // below is applied exactly as before. pidUpdate() only recomputes on
  // its fixed interval; calling it every loop() is fine and intended.
  wp = pidUpdate(wp);
#endif

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
  estopInit();   // after motorsInit() — arms the independent kill interrupt
  encoderInit();
  diagInit();
  imu_init();

#if USE_PID
  pidInit();   // must come after encoderInit() — it snapshots the counts
#endif

  startupSequence();
}

void loop() {
  rcUpdate();
  imu_update();           // poll the IMU every tick; cheap if no new sample
  estopUpdate();          // poll E-stop triggers (the HW button also kills
                          // itself via interrupt, independent of this call)

  bool failsafe = rcFailsafe();
  bool estop    = estopActive();
  // E-stop overrides everything: no driving while latched.
  bool enabled  = rcEnabled() && !estop;

  // While disarmed, allow a released E-stop to clear so it can be re-armed.
  if (!rcEnabled()) estopClear();

  statusLedUpdate(failsafe, enabled, rcSticksCentered());

  // Push CH2 (left stick U/D - no spring return, otherwise unused) to the
  // top and hold to run the autonomous straight->turn->straight demo
  // (navigation.h). Rising-edge detected against navTriggerReady so
  // holding the stick up only fires once; it re-arms when the stick
  // comes back down. Only takes effect while armed.
  static bool navTriggerReady = true;
  bool navTriggerHigh = rcRawChannel(IBUS_CH_NAV_TRIGGER) > NAV_TRIGGER_THRESHOLD;
  if (navTriggerHigh && navTriggerReady && enabled) {
    navTriggerReady = false;
    navRunDemoSequence();
  } else if (!navTriggerHigh) {
    navTriggerReady = true;
  }

  // 'n' over Serial does the same thing, for bench testing with a laptop
  // attached. Same armed-only gating.
  if (Serial.available()) {
    char navCmd = Serial.read();
    if ((navCmd == 'n' || navCmd == 'N') && enabled) {
      navRunDemoSequence();
    }
  }

  if (!enabled) {
    stopAllMotors();
#if USE_PID
    // Motors are off — keep the PID clean so wound-up integral / stale
    // derivative can't lurch the wheels the moment we re-arm.
    pidResetAll();
#endif
#if USE_IMU_HEADING_HOLD
    // Same idea for heading-hold: drop the lock so re-arming re-latches
    // a fresh target heading instead of an old one.
    imu_resetHeadingHold();
#endif
  } else {
    driveturBot(rcVx(), rcVy(), rcOmega());
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    debugPrint(enabled, failsafe);
    imu_printHeading();   // "heading: X.X deg" - compare against the [imu heading:...] line above
  }
}