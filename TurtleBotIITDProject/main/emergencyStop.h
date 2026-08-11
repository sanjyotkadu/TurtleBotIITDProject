#ifndef EMERGENCY_STOP_H
#define EMERGENCY_STOP_H

// ============================================================
// emergencyStop.h — Independent, latching emergency-stop
//
// Cuts all three motors the instant an E-stop trigger asserts, and keeps
// them cut until a deliberate recovery. There are two OR'd triggers
// (either one stops the robot), configured in robot_config.h:
//
//   1. A HARDWARE BUTTON on a digital pin (ESTOP_PIN). This is the truly
//      "independently running" path: it is wired to a hardware interrupt,
//      so the moment the button is pressed the ISR writes the L298N enable
//      pins LOW directly — the motors die even if loop() has hung.
//
//   2. A FlySky RC SWITCH (ESTOP_RC_CHANNEL, default CH5). Convenient from
//      the transmitter, but it rides the RC link so it is only as fast/
//      reliable as the iBUS parse in loop(). It is the backup, not the
//      primary — a physical button is the real safety device.
//
// The stop LATCHES: once tripped the robot stays stopped until the trigger
// is released AND you disarm (kill switch off), which is when main.ino
// calls estopClear(). Then you can re-arm normally.
// ============================================================

#include <Arduino.h>

// Configure the E-stop pin + interrupt (and nothing else). Call from
// setup(), AFTER motorsInit() so the motor pins are already OUTPUTs.
void estopInit();

// Poll the triggers, latch + hard-kill on a trip, and print state
// changes. Call every loop(), right after rcUpdate() (the RC-switch path
// needs a fresh iBUS frame). The hardware button does NOT depend on this
// being called — its interrupt kills the motors on its own.
void estopUpdate();

// true = E-stop is latched; the robot must not drive.
bool estopActive();

// Attempt to clear the latch. No-op if any trigger is still asserted, so
// you can never "reset into" a still-pressed button. Call this only when
// it is safe to allow re-arming — main.ino calls it while disarmed.
void estopClear();

#endif // EMERGENCY_STOP_H
