#include "statusIndicator.h"
#include "robot_config.h"

static StatusIndicatorState s_state       = STATUS_IDLE;
static unsigned long        s_lastToggle  = 0;
static bool                 s_ledOn       = false;

static unsigned long blinkIntervalMs(StatusIndicatorState state) {
  switch (state) {
    case STATUS_NAV_RUNNING: return STATUS_BLINK_NAV_MS;
    case STATUS_OBSTACLE:    return STATUS_BLINK_OBSTACLE_MS;
    case STATUS_IDLE:
    default:                 return 0;   // unused - STATUS_IDLE returns early in statusIndicatorUpdate()
  }
}

void statusIndicatorInit() {
  pinMode(STATUS_INDICATOR_LED_PIN, OUTPUT);
  digitalWrite(STATUS_INDICATOR_LED_PIN, LOW);
  s_state      = STATUS_IDLE;
  s_ledOn      = false;
  s_lastToggle = millis();
}

void statusIndicatorSetState(StatusIndicatorState state) {
  if (state == s_state) return;
  s_state      = state;
  s_lastToggle = millis();
  // Snap on immediately on any real state change, so switching TO
  // STATUS_OBSTACLE (or back to STATUS_NAV_RUNNING) is visible right away
  // instead of waiting up to one blink interval for the first toggle.
  s_ledOn = (state != STATUS_IDLE);
  digitalWrite(STATUS_INDICATOR_LED_PIN, s_ledOn ? HIGH : LOW);
}

void statusIndicatorUpdate() {
  if (s_state == STATUS_IDLE) return;

  unsigned long interval = blinkIntervalMs(s_state);
  if (millis() - s_lastToggle >= interval) {
    s_lastToggle = millis();
    s_ledOn = !s_ledOn;
    digitalWrite(STATUS_INDICATOR_LED_PIN, s_ledOn ? HIGH : LOW);
  }
}
