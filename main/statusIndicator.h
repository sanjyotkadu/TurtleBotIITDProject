#ifndef STATUS_INDICATOR_H
#define STATUS_INDICATOR_H

#include <Arduino.h>

// ============================================================
// statusIndicator.h — non-blocking LED heartbeat on STATUS_INDICATOR_LED_PIN
// (robot_config.h) saying, at a glance, what an autonomous nav sequence
// (navigation.cpp) is currently doing. Separate from OBSTACLE_LED_PIN
// (obstacleAvoid.h), which means "something is in front of me right now" -
// this one means "a nav sequence is running at all":
//   STATUS_IDLE        : LED off - no navRunDemoSequence()/
//                         navRunWaypointSequence() call in progress.
//   STATUS_NAV_RUNNING  : slow steady blink - a sequence is running normally.
//   STATUS_OBSTACLE     : fast blink - driveStraightWithObstacleCheck()
//                         (obstacleAvoid.cpp) currently has something within
//                         OBSTACLE_TRIGGER_CM/OBSTACLE_EMERGENCY_CM, or is
//                         scanning/detouring around one.
//
// Wiring: main.ino sets STATUS_NAV_RUNNING right before calling a
// nav*Sequence() and back to STATUS_IDLE right after it returns (success
// or aborted) - that single pair of call sites covers every return path
// inside navigation.cpp. obstacleAvoid.cpp flips between STATUS_NAV_RUNNING
// and STATUS_OBSTACLE itself as it detects/clears an obstacle mid-leg.
//
// Non-blocking: statusIndicatorUpdate() times the blink off millis() and
// never calls delay() - it must be called often (every loop() pass, and
// from inside navigation.cpp/obstacleAvoid.cpp's blocking wait loops,
// exactly like those loops already call imu_update()) or the blink will
// visibly stall during a long blocking call.
// ============================================================

enum StatusIndicatorState {
  STATUS_IDLE,
  STATUS_NAV_RUNNING,
  STATUS_OBSTACLE,
};

// Sets STATUS_INDICATOR_LED_PIN as an output, LED off. Call once from setup().
void statusIndicatorInit();

// Switches the displayed pattern. Cheap and safe to call every time a
// caller thinks the state might have changed - it's a no-op if `state`
// already matches what's showing.
void statusIndicatorSetState(StatusIndicatorState state);

// Advances the current pattern's blink timing by however much time has
// passed since the last call. Non-blocking - safe to call every loop()
// iteration and from inside any blocking wait loop.
void statusIndicatorUpdate();

#endif // STATUS_INDICATOR_H
