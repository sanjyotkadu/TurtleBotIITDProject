#include "RCControl.h"
#include "robot_config.h"
#include "IBusReader.h"
#include <math.h>

// ------------------------------------------------------------
// Module-private state — hidden from the rest of the firmware.
// ------------------------------------------------------------
static IBusReader ibus(IBUS_SERIAL);

// Calibrated resting centers, captured at boot and continuously
// tracked while disarmed.
static float centerVX    = IBUS_PWM_MID;
static float centerVY    = IBUS_PWM_MID;
static float centerOmega = IBUS_PWM_MID;

// Arm state machine + cached per-tick status.
static bool switchSeenOff = false;
static bool armedFlag     = false;
static bool prevSwitchOn  = false;
static bool failsafeFlag   = true;
static bool switchOnFlag   = false;

// Map a raw channel to a normalized -1..1 value around its center,
// with a deadzone to reject stick drift.
static float normalizedStick(uint8_t channel, float center) {
  uint16_t raw = ibus.channel(channel);
  raw = constrain(raw, (uint16_t)IBUS_PWM_MIN, (uint16_t)IBUS_PWM_MAX);

  float offsetFromCenter = (float)raw - center;
  if (fabs(offsetFromCenter) <= IBUS_DEADZONE) {
    return 0.0f;
  }

  float span = (offsetFromCenter > 0) ? (IBUS_PWM_MAX - center)
                                      : (center - IBUS_PWM_MIN);
  float value = offsetFromCenter / span;
  return constrain(value, -1.0f, 1.0f);
}

void rcInit() {
  ibus.begin(IBUS_BAUD);
}

void rcCalibrateCenters() {
  Serial.println("Calibrating stick centers - DO NOT TOUCH STICKS...");

  long sumVX = 0, sumVY = 0, sumOmega = 0;
  int samples = 0;
  unsigned long calStart = millis();

  while (millis() - calStart < 1000) {
    ibus.update();
    if (!ibus.isFailsafe(IBUS_FAILSAFE_MS)) {
      sumVX    += ibus.channel(IBUS_CH_VX);
      sumVY    += ibus.channel(IBUS_CH_VY);
      sumOmega += ibus.channel(IBUS_CH_OMEGA);
      samples++;
    }
  }

  if (samples > 0) {
    centerVX    = sumVX    / samples;
    centerVY    = sumVY    / samples;
    centerOmega = sumOmega / samples;
  }

  Serial.print("Calibrated centers -> VX: ");
  Serial.print(centerVX);
  Serial.print("  VY: ");
  Serial.print(centerVY);
  Serial.print("  Omega: ");
  Serial.println(centerOmega);
}

void rcUpdate() {
  ibus.update();

  failsafeFlag = ibus.isFailsafe(IBUS_FAILSAFE_MS);
  uint16_t rawEnable = ibus.channel(IBUS_CH_ENABLE);
  switchOnFlag = (rawEnable > IBUS_ENABLE_THRESHOLD);

  if (!switchOnFlag) {
    switchSeenOff = true;
    armedFlag = false;

    // While disarmed and linked, slowly follow any center drift so a
    // stick nudged at rest doesn't lock in a bad zero at arm time.
    if (!failsafeFlag) {
      int rawVX    = ibus.channel(IBUS_CH_VX);
      int rawVY    = ibus.channel(IBUS_CH_VY);
      int rawOmega = ibus.channel(IBUS_CH_OMEGA);
      centerVX    += (rawVX    - centerVX)    * 0.02f;
      centerVY    += (rawVY    - centerVY)    * 0.02f;
      centerOmega += (rawOmega - centerOmega) * 0.02f;
    }
  }

  // Require a fresh OFF->ON flip of the kill switch, with sticks
  // centered, before arming.
  bool risingEdge = switchOnFlag && !prevSwitchOn && switchSeenOff;
  if (risingEdge) {
    if (rcSticksCentered()) {
      armedFlag = true;
    } else {
      armedFlag = false;
      Serial.println("ARM BLOCKED: center all sticks, then toggle the enable switch OFF then ON again.");
    }
  }
  prevSwitchOn = switchOnFlag;
}

bool rcFailsafe()  { return failsafeFlag; }
bool rcArmed()     { return armedFlag; }
bool rcEnabled()   { return armedFlag && switchOnFlag && !failsafeFlag; }

bool rcSticksCentered() {
  return (rcVx() == 0.0f && rcVy() == 0.0f && rcOmega() == 0.0f);
}

float rcVx()    { return normalizedStick(IBUS_CH_VX,    centerVX); }
float rcVy()    { return normalizedStick(IBUS_CH_VY,    centerVY); }
float rcOmega() { return normalizedStick(IBUS_CH_OMEGA, centerOmega); }

uint16_t rcRawChannel(uint8_t ch) { return ibus.channel(ch); }

float rcCenterVx()    { return centerVX; }
float rcCenterVy()    { return centerVY; }
float rcCenterOmega() { return centerOmega; }
