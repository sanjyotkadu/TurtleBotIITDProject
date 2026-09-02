/*
 * servoTest.ino — SG90 micro servo bring-up & verification
 * ---------------------------------------------------------------------
 * Standalone diagnostic. Confirms the SG90 is wired correctly and moves
 * predictably before it's wired into any other mechanism.
 *
 * POWER — READ BEFORE WIRING
 *   The SG90 wants ~4.8-6V and can pull a few hundred mA moving under
 *   load (spec'd up to ~600mA stalled). Do NOT feed it from the Teensy's
 *   3.3V pin — too low a voltage and not enough current.
 *
 *   Running the servo's VCC off the Teensy's 5V pin while the Teensy is
 *   powered only from a laptop/USB port: for THIS bare bench test (servo
 *   horn free, no load, one SG90, nothing else drawing power) it will
 *   generally work — a lone SG90 idles around 10mA and only spikes
 *   under load. It is NOT safe once this servo shares the board with the
 *   drive motors, or once it's actually driving a load: a stall pulls
 *   enough current to brown out the Teensy through the same USB feed
 *   (symptom: servo twitches, Teensy resets/disconnects). For anything
 *   beyond this quick test, power the servo from an external 5-6V
 *   supply (e.g. a BEC / regulated rail) with its GND tied to the
 *   Teensy's GND — never leave the grounds unconnected.
 *
 * WIRING:
 *   SG90 signal (orange) -> Teensy pin SERVO_PIN (see robot_config.h)
 *   SG90 VCC (red)        -> 5V (see power note above)
 *   SG90 GND (brown)      -> Teensy GND
 *
 * NEEDS: the built-in "Servo" library (Teensyduino ships it — no
 * install needed). Serial Monitor @ 115200, "Newline".
 *
 * COMMANDS:
 *   0-9   -> jump to that decile of travel (0=0 deg, 5=100 deg, 9=180 deg)
 *   c     -> center (90 deg)
 *   w     -> sweep 0 -> 180 -> 0 once
 *   ?     -> help
 * ---------------------------------------------------------------------
 */

#include "robot_config.h"
#include <Servo.h>

static Servo sg90;

static void moveTo(int deg) {
  deg = constrain(deg, 0, 180);
  sg90.writeMicroseconds(map(deg, 0, 180, SERVO_MIN_US, SERVO_MAX_US));
  Serial.print(" -> ");
  Serial.print(deg);
  Serial.println(" deg");
}

static void sweep() {
  Serial.println(" sweeping 0 -> 180 -> 0 ...");
  for (int d = 0; d <= 180; d += 2)  { moveTo(d); delay(15); }
  for (int d = 180; d >= 0; d -= 2)  { moveTo(d); delay(15); }
  Serial.println(" sweep done.");
}

static void printHelp() {
  Serial.println("\n==================================================");
  Serial.println(" SG90 servo test");
  Serial.println("==================================================");
  Serial.println(" 0-9 = jump to that decile of travel (0=0 deg ... 9=180 deg)");
  Serial.println(" c   = center (90 deg)");
  Serial.println(" w   = sweep 0->180->0");
  Serial.println(" ?   = this help");
  Serial.println("==================================================\n");
}

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial && millis() < 3000) { /* wait briefly for USB serial */ }

  sg90.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  moveTo(90); // start centered — safest resting position for most linkages

  printHelp();
  Serial.println(" NOTE: if the servo buzzes/jitters instead of holding position,");
  Serial.println(" or the Teensy resets when it moves, stop and use an external");
  Serial.println(" 5-6V supply for the servo (see comment header in this file).\n");
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();

    if (ch >= '0' && ch <= '9') {
      Serial.print(" ["); Serial.print(ch); Serial.print("]");
      moveTo((ch - '0') * 20); // 0..9 -> 0,20,...,180 deg
      return;
    }

    switch (ch) {
      case 'c': case 'C': Serial.print(" [c] center"); moveTo(90); break;
      case 'w': case 'W': sweep(); break;
      case '?':           printHelp(); break;
      default: break;
    }
  }
}
