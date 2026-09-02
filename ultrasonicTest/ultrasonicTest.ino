/*
 * ultrasonicTest.ino — HC-SR04 ultrasonic range finder bring-up
 * ---------------------------------------------------------------------
 * Standalone diagnostic. Pings the HC-SR04 and streams distance
 * readings so you can confirm it's wired correctly before using it for
 * obstacle detection.
 *
 * POWER / LEVELS — READ BEFORE WIRING
 *   HC-SR04 runs its logic at 5V. TRIG is a Teensy OUTPUT, and the
 *   Teensy's 3.3V HIGH is well above the sensor's input threshold, so
 *   TRIG can connect directly.
 *
 *   ECHO is the opposite direction: the sensor drives ECHO_PIN at 5V,
 *   and Teensy 4.x pins are NOT 5V tolerant. Feeding ECHO straight into
 *   the Teensy can damage the pin. This build steps it down through a
 *   TXS0108E level-shifter module (chip marking "YF08E" — TI 8-channel
 *   bidirectional 3.3V<->5V translator). Only one channel is used:
 *
 *       HC-SR04 ECHO -> TXS0108E B1 (5V side)
 *       TXS0108E A1 (3.3V side) -> Teensy ECHO_PIN
 *       TXS0108E VCCA -> Teensy 3.3V   VCCB -> 5V   OE -> Teensy 3.3V
 *       TXS0108E GND  -> common GND (both LV and HV GND pins)
 *
 *   (A plain 1k/2k resistor divider works too if you don't have the
 *   level shifter — either way, do NOT skip stepping ECHO down.)
 *
 * WIRING:
 *   HC-SR04 VCC  -> 5V
 *   HC-SR04 GND  -> Teensy GND
 *   HC-SR04 TRIG -> Teensy pin TRIG_PIN (direct)
 *   HC-SR04 ECHO -> TXS0108E B1; TXS0108E A1 -> Teensy pin ECHO_PIN
 *
 * NEEDS: nothing extra — uses plain pulseIn(), no library. Serial
 * Monitor @ 115200, "Newline".
 *
 * COMMANDS:
 *   p   -> single ping, print distance once
 *   s   -> toggle continuous streaming (5/sec)
 *   ?   -> help
 * ---------------------------------------------------------------------
 */

#include "robot_config.h"

static bool streaming = false;

// Fires one ping and returns distance in cm, or a negative sentinel:
// -1 = no echo (timeout / out of range), -2 = below the sensor's
// reliable minimum range (see SONAR_MIN_RANGE_CM in robot_config.h).
static float pingDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long echoUs = pulseIn(ECHO_PIN, HIGH, SONAR_TIMEOUT_US);
  if (echoUs == 0) return -1.0f; // timed out, no echo

  // Speed of sound ~343 m/s -> 0.0343 cm/us; divide by 2 for round trip.
  float cm = (echoUs * 0.0343f) / 2.0f;
  if (cm > SONAR_MAX_RANGE_CM) return -1.0f;
  if (cm < SONAR_MIN_RANGE_CM) return -2.0f;
  return cm;
}

static void printReading() {
  float cm = pingDistanceCm();
  if (cm == -2.0f) {
    Serial.println("  too close (below sensor's reliable minimum range)");
  } else if (cm < 0) {
    Serial.println("  no echo (out of range or nothing in front)");
  } else {
    Serial.print("  distance: ");
    Serial.print(cm, 1);
    Serial.println(" cm");
  }
}

static void printHelp() {
  Serial.println("\n==================================================");
  Serial.println(" HC-SR04 ultrasonic test");
  Serial.println("==================================================");
  Serial.println(" p = single ping");
  Serial.println(" s = toggle continuous streaming (5/sec)");
  Serial.println(" ? = this help");
  Serial.println("==================================================\n");
}

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial && millis() < 3000) { /* wait briefly for USB serial */ }

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  printHelp();
}

void loop() {
  if (Serial.available()) {
    char ch = Serial.read();
    switch (ch) {
      case 'p': case 'P': Serial.println(" [p] ping:"); printReading(); break;
      case 's': case 'S':
        streaming = !streaming;
        Serial.println(streaming ? "\n[streaming ON]" : "\n[streaming OFF]");
        break;
      case '?': printHelp(); break;
      default: break;
    }
  }

  if (streaming) {
    static unsigned long lastPing = 0;
    if (millis() - lastPing >= 200) {
      lastPing = millis();
      printReading();
    }
  }
}
