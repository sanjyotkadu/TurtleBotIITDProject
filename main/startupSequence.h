#ifndef STARTUP_SEQUENCE_H
#define STARTUP_SEQUENCE_H

// ============================================================
// startupSequence.h — Boot-time orchestration
//
// Runs the blocking startup flow that must complete before the main
// control loop is allowed to run:
//   1. wait (briefly) for the USB serial monitor
//   2. wait for a valid iBUS link from the receiver (slow-blink LED),
//      printing a periodic "still waiting" nudge
//   3. calibrate the resting stick centers
//
// Assumes rcInit(), motorsInit(), diagInit() (and encoderInit() if
// used) have already run, and that motors are stopped.
// ============================================================

void startupSequence();

#endif // STARTUP_SEQUENCE_H
