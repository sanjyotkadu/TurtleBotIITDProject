/*
 * navigationRC.ino — RC-triggered scripted step-sequencer autopilot
 * ---------------------------------------------------------------------
 * Field version: NO serial connection needed to operate. Arm and start
 * the pre-programmed route entirely from the transmitter:
 *
 *   CH4 (enable switch)  -> arm/disarm, same as main.ino (requires
 *                           sticks centered at the moment you flip it).
 *   CH2 (NAV_TRIGGER_CH)  -> flip past NAV_TRIGGER_THRESHOLD to START
 *                           the loaded program; flip back to ABORT it.
 *                           See the NAV TRIGGER note in robot_config.h —
 *                           VERIFY this is really the channel your spare
 *                           button/switch lands on before trusting it.
 *   CH5 switch / HW button -> emergency stop, exactly like main.ino —
 *                           always active, independent of everything
 *                           below.
 *
 * The JOYSTICKS (Vx/Vy/Omega) are NOT used to drive in this sketch —
 * translation/rotation come only from the loaded navigation program, on
 * a timer. RC link loss (failsafe) or E-stop both abort the program
 * immediately and stop the motors.
 *
 * Serial (115200) is still there for STATUS ONLY — handy on the bench,
 * not required in the field.
 * ---------------------------------------------------------------------
 */

#include "robot_config.h"
#include "RCControl.h"
#include "motorControl.h"
#include "encoder.h"
#include "imu.h"
#include "PID.h"
#include "emergencyStop.h"
#include "navigation.h"

#define USE_PID 1

// ---- EDIT THIS: the route the robot runs when triggered ----
static const NavStep program[] = {
  { 2.0f, NAV_FORWARD, 90.0f, 0.0f },   // 2 sec forward,  heading 90 deg
  { 5.0f, NAV_LEFT,    90.0f, 0.0f },   // 5 sec left,     heading 90 deg
  { 7.0f, NAV_RIGHT,   30.0f, 0.0f },   // 7 sec right,    heading 30 deg
};
static const int programLen = sizeof(program) / sizeof(program[0]);
// --------------------------------------------------------------

static bool prevTriggerOn = false;

static bool triggerAsserted() {
  return rcRawChannel(NAV_TRIGGER_CH) > NAV_TRIGGER_THRESHOLD;
}

void setup() {
  Serial.begin(BAUD_RATE);   // status only — not required for control

  rcInit();
  motorsInit();
  estopInit();          // after motorsInit() — arms the independent kill interrupt
  encoderInit();
  imu_init();
#if USE_PID
  pidInit();
#endif
  navInit();

  Serial.println(F("\n=================================================="));
  Serial.println(F(" navigationRC — RC-triggered autopilot, no joystick drive"));
  Serial.println(F("=================================================="));
  Serial.println(F("Arm with CH4 (sticks centered). Start/abort the route with CH2."));
  Serial.println(F("E-stop (button + CH5 switch) always active."));
}

void loop() {
  rcUpdate();
  imu_update();
  estopUpdate();      // poll E-stop triggers (HW button also kills itself via interrupt)

  bool failsafe = rcFailsafe();
  bool estop    = estopActive();
  // E-stop overrides everything: no driving while latched.
  bool enabled  = rcEnabled() && !estop;

  // While disarmed, allow a released E-stop to clear so it can be re-armed.
  if (!rcEnabled()) estopClear();

  bool triggerOn   = enabled && !failsafe && triggerAsserted();
  bool risingEdge  = triggerOn && !prevTriggerOn;
  bool fallingEdge = !triggerOn && prevTriggerOn;
  prevTriggerOn = triggerOn;

  if (!enabled || failsafe) {
    // Disarmed, switch off, E-stop tripped, or RC link lost — hard stop
    // and drop whatever the program was doing. Safety wins over
    // finishing the route.
    if (navIsRunning()) navStop();
    stopAllMotors();
#if USE_PID
    pidResetAll();
#endif
  } else if (risingEdge && !navIsRunning()) {
    navLoadProgram(program, programLen);
    navStart();
  } else if (fallingEdge && navIsRunning()) {
    // Trigger flipped back off — treat that as a deliberate abort.
    navStop();
    Serial.println("[NAV-RC] trigger released - route aborted.");
  } else if (navIsRunning()) {
    navUpdate();
    if (!navIsRunning()) {
      Serial.println("[NAV-RC] program complete — flip the trigger off then on to run it again.");
    }
  } else {
    stopAllMotors();
#if USE_PID
    pidResetAll();
#endif
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    Serial.print("[enabled:"); Serial.print(enabled ? "Y" : "N");
    Serial.print(" failsafe:"); Serial.print(failsafe ? "Y" : "N");
    Serial.print(" estop:"); Serial.print(estop ? "Y" : "N");
    Serial.print(" trigger:"); Serial.print(triggerOn ? "ON" : "off");
    Serial.print(" navRunning:"); Serial.print(navIsRunning() ? "Y" : "N");
    Serial.print(" step:"); Serial.print(navCurrentStepIndex());
    Serial.print("/"); Serial.print(programLen);
    Serial.print("  [raw trigger ch:"); Serial.print(rcRawChannel(NAV_TRIGGER_CH));
    Serial.print("]  [imu heading:"); Serial.print(imu_headingDeg(), 1);
    Serial.print(" healthy:"); Serial.print(imu_healthy() ? "Y" : "N");
    Serial.println("]");
  }
}
