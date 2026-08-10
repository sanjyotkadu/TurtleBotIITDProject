#include "navigation.h"
#include "robot_config.h"
#include "kinematics.h"
#include "motorControl.h"
#include "imu.h"
#include "PID.h"
#include <math.h>

#define NAV_USE_PID 1

enum NavPhase { NAV_PHASE_TURN, NAV_PHASE_DRIVE };

static const NavStep *program      = nullptr;
static int            programLen   = 0;
static int            stepIndex    = -1;
static NavPhase       phase        = NAV_PHASE_TURN;
static unsigned long  phaseStartMs = 0;
static bool           running      = false;

static float wrapPi(float a) {
  while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
  return a;
}

// Same shape as main.ino's driveturBot(): kinematics -> PID -> direction
// correction -> motors.
static void driveNavBot(float Vx, float Vy, float Omega) {
  WheelPowers wp = kiwiMix(Vx, Vy, Omega);
#if NAV_USE_PID
  wp = pidUpdate(wp);
#endif
  driveMotor(EN_C, IN3_C, IN4_C, wp.front_C     * DIR_C_CW);
  driveMotor(EN_A, IN1,   IN2,   wp.backLeft_A  * DIR_A_CW);
  driveMotor(EN_B, IN3,   IN4,   wp.backRight_B * DIR_B_CW);
}

static void dirToVxVy(NavDir dir, float speed, float &Vx, float &Vy) {
  Vx = 0.0f; Vy = 0.0f;
  switch (dir) {
    case NAV_FORWARD:  Vy =  speed; break;
    case NAV_BACKWARD: Vy = -speed; break;
    case NAV_LEFT:      Vx = -speed; break;
    case NAV_RIGHT:      Vx =  speed; break;
  }
}

// Signed shortest-path error to an ABSOLUTE target heading. Unlike
// imu_applyHeadingHold() (which trims toward whatever heading it
// happened to latch when the stick centered), this drives toward a
// heading the caller chooses.
static float headingErrorRad(float targetHeadingDeg) {
  float targetRad = targetHeadingDeg * (float)M_PI / 180.0f;
  return wrapPi(targetRad - imu_headingRad());
}

static float clampOmega(float omega) {
  if (omega >  NAV_TURN_MAX_OMEGA) return  NAV_TURN_MAX_OMEGA;
  if (omega < -NAV_TURN_MAX_OMEGA) return -NAV_TURN_MAX_OMEGA;
  return omega;
}

void navInit() {
  program    = nullptr;
  programLen = 0;
  stepIndex  = -1;
  running    = false;
}

void navLoadProgram(const NavStep *steps, int count) {
  program    = steps;
  programLen = count;
  stepIndex  = -1;
  running    = false;
}

void navStart() {
  if (program == nullptr || programLen <= 0) {
    Serial.println("[NAV] no program loaded - call navLoadProgram() first.");
    return;
  }
  stepIndex    = 0;
  phase        = NAV_PHASE_TURN;
  phaseStartMs = millis();
  running      = true;
  Serial.print("[NAV] starting, "); Serial.print(programLen); Serial.println(" step(s).");
}

void navStop() {
  running   = false;
  stepIndex = -1;
  stopAllMotors();
#if NAV_USE_PID
  pidResetAll();
#endif
}

bool navIsRunning()        { return running; }
int  navCurrentStepIndex() { return stepIndex; }

void navUpdate() {
  if (!running) return;

  if (!imu_healthy()) {
    Serial.println("[NAV] IMU unhealthy - aborting program.");
    navStop();
    return;
  }

  if (stepIndex >= programLen) {
    Serial.println("[NAV] program complete.");
    navStop();
    return;
  }

  const NavStep &step  = program[stepIndex];
  float          speed = (step.speed > 0.0f) ? step.speed : NAV_DEFAULT_SPEED;
  float          errRad = headingErrorRad(step.headingDeg);

  if (phase == NAV_PHASE_TURN) {
    bool withinTolerance = fabs(errRad) <= (NAV_HEADING_TOLERANCE_DEG * (float)M_PI / 180.0f);
    bool timedOut        = (millis() - phaseStartMs) > NAV_TURN_TIMEOUT_MS;

    if (!withinTolerance && !timedOut) {
      driveNavBot(0.0f, 0.0f, clampOmega(NAV_TURN_KP * errRad));
      return;
    }
    if (timedOut && !withinTolerance) {
      Serial.print("[NAV] step "); Serial.print(stepIndex);
      Serial.println(": turn timed out, driving anyway.");
    }
    phase        = NAV_PHASE_DRIVE;
    phaseStartMs = millis();
  }

  // NAV_PHASE_DRIVE
  if ((millis() - phaseStartMs) >= (unsigned long)(step.durationSec * 1000.0f)) {
    Serial.print("[NAV] step "); Serial.print(stepIndex); Serial.println(" done.");
    stepIndex++;
    phase        = NAV_PHASE_TURN;
    phaseStartMs = millis();
#if NAV_USE_PID
    pidResetAll();
#endif
    return;
  }

  float Vx, Vy;
  dirToVxVy(step.dir, speed, Vx, Vy);
  driveNavBot(Vx, Vy, clampOmega(NAV_TURN_KP * errRad));
}
