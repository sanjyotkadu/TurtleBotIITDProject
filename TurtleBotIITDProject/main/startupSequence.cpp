#include "startupSequence.h"
#include "robot_config.h"
#include "RCControl.h"
#include "diagnostics.h"

void startupSequence() {
  // Give the USB serial monitor a moment to attach, but don't hang
  // forever if the robot is running untethered.
  while (!Serial && millis() < 3000) { ; }
  Serial.println("Kiwi drive starting...");

  // Block until the receiver is actually sending valid frames, so the
  // robot can never calibrate or arm off garbage/no-link data.
  Serial.println("Waiting for valid iBUS signal from receiver...");
  unsigned long linkWaitStart = millis();
  unsigned long lastWarnMs = linkWaitStart;

  rcUpdate();
  while (rcFailsafe()) {
    rcUpdate();
    setStatusLed(500, 500);  // slow blink - waiting for link

    if (millis() - lastWarnMs > 3000) {
      lastWarnMs = millis();
      Serial.print("  still waiting for receiver... (");
      Serial.print((millis() - linkWaitStart) / 1000);
      Serial.println("s) - check receiver power / bind / wiring");
    }
  }
  Serial.println("Valid iBUS signal detected.");

  rcCalibrateCenters();
}
