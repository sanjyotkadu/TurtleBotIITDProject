#include "emergencyStop.h"
#include "robot_config.h"
#include "RCControl.h"    // rcRawChannel() for the RC-switch trigger

// ------------------------------------------------------------
// Latched state. volatile because the hardware ISR writes it while the
// main loop reads it.
// ------------------------------------------------------------
static volatile bool estopLatched = false;

// For edge-detecting the state so we print only on change.
static bool prevLatched = false;

// For clear-debounce: when did the trigger last read "not safe".
static unsigned long lastAssertedMs = 0;

// ------------------------------------------------------------
// The actual kill. Writes the L298N ENABLE and direction pins LOW with
// plain digitalWrite — no PWM, no library calls — so it is safe to run
// from inside an interrupt and stops the H-bridges dead (EN low = outputs
// disabled). Deliberately does NOT go through motorControl so the kill
// path has zero dependencies.
// ------------------------------------------------------------
static inline void killMotorsHard() {
  digitalWrite(EN_A, LOW); digitalWrite(IN1,   LOW); digitalWrite(IN2,   LOW);
  digitalWrite(EN_B, LOW); digitalWrite(IN3,   LOW); digitalWrite(IN4,   LOW);
  digitalWrite(EN_C, LOW); digitalWrite(IN3_C, LOW); digitalWrite(IN4_C, LOW);
}

// ------------------------------------------------------------
// Is each trigger currently asserting "STOP"?
// ------------------------------------------------------------
#if ESTOP_USE_HARDWARE_PIN
static inline bool hwPinAsserted() {
#if ESTOP_PIN_ACTIVE_LOW
  return digitalRead(ESTOP_PIN) == LOW;
#else
  return digitalRead(ESTOP_PIN) == HIGH;
#endif
}
#else
static inline bool hwPinAsserted() { return false; }
#endif

#if ESTOP_USE_RC_SWITCH
static inline bool rcSwitchAsserted() {
  uint16_t raw = rcRawChannel(ESTOP_RC_CHANNEL);
  if (raw == 0) return false;   // no valid frame yet -> not a trigger
#if ESTOP_RC_ASSERT_ABOVE
  return raw > ESTOP_RC_THRESHOLD;
#else
  return raw < ESTOP_RC_THRESHOLD;
#endif
}
#else
static inline bool rcSwitchAsserted() { return false; }
#endif

static inline bool anyTriggerAsserted() {
  return hwPinAsserted() || rcSwitchAsserted();
}

// ------------------------------------------------------------
// Hardware interrupt: the independent kill path. Fires on ANY edge of the
// E-stop pin; if the new level means "pressed", latch and kill instantly.
// Release edges do nothing (the latch is cleared elsewhere, deliberately).
// ------------------------------------------------------------
#if ESTOP_USE_HARDWARE_PIN
static void estopISR() {
  if (hwPinAsserted()) {
    estopLatched = true;
    killMotorsHard();
  }
}
#endif

void estopInit() {
#if ESTOP_USE_HARDWARE_PIN
  // Active-low -> pull-up (idle HIGH, pressed LOW). Active-high -> pull-down.
  #if ESTOP_PIN_ACTIVE_LOW
    pinMode(ESTOP_PIN, INPUT_PULLUP);
  #else
    pinMode(ESTOP_PIN, INPUT_PULLDOWN);
  #endif
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, CHANGE);

  // If the button is already held at boot, trip immediately.
  if (hwPinAsserted()) {
    estopLatched = true;
    killMotorsHard();
  }
#endif
  lastAssertedMs = millis();
}

void estopUpdate() {
  bool asserted = anyTriggerAsserted();

  if (asserted) {
    // Belt-and-suspenders: re-kill every loop while asserted. This is also
    // the ONLY kill path for the RC switch (it has no interrupt).
    estopLatched = true;
    killMotorsHard();
    lastAssertedMs = millis();
  }

  // Print only on a latched-state change so we don't spam the serial log.
  if (estopLatched != prevLatched) {
    prevLatched = estopLatched;
    if (estopLatched) Serial.println("!!! EMERGENCY STOP — MOTORS KILLED !!!");
    else              Serial.println("E-stop cleared — ready to re-arm.");
  }
}

bool estopActive() {
  return estopLatched;
}

void estopClear() {
  // Never clear while a trigger is still asserted, and require it to have
  // read "safe" for the debounce window (ignores button chatter / a switch
  // bouncing through the threshold).
  if (anyTriggerAsserted()) {
    lastAssertedMs = millis();
    return;
  }
  if (millis() - lastAssertedMs >= (unsigned long)ESTOP_RESET_DEBOUNCE_MS) {
    estopLatched = false;
  }
}
