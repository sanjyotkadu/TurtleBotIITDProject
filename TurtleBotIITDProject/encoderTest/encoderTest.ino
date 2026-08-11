/*
 * encoderTest.ino — Encoder wiring & PID sign-alignment checker
 * ---------------------------------------------------------------------
 * Standalone diagnostic sketch. Run this BEFORE tuning PID to prove:
 *
 *   1. Each encoder is wired and counting cleanly (turn wheels by hand).
 *   2. For each wheel, a POSITIVE pre-DIR motor command produces a
 *      POSITIVE measured speed — the exact condition PID.cpp needs. If a
 *      wheel fails this, the fix is printed: flip that wheel's
 *      PID_ENC_SIGN_x in robot_config.h.
 *
 * This sketch is self-contained on purpose (it re-implements the encoder
 * ISRs and motor drive inline) so it isolates the wiring from the rest of
 * the firmware — it mirrors encoder.cpp / motorControl.cpp exactly.
 *
 * ---------------------------------------------------------------------
 * HOW TO USE
 *   - Put this file in its own sketch folder named "encoderTest", next to
 *     a COPY of robot_config.h (Arduino needs one .ino per folder, so it
 *     can't share the folder with main.ino).
 *   - Open the Serial Monitor at 115200 baud, set line ending to
 *     "Newline" or "No line ending".
 *   - The loop prints live counts continuously. Type a command + Enter:
 *
 *        1 / 2 / 3  -> powered sign test on wheel A / B / C
 *        a          -> powered sign test on ALL wheels, one after another
 *        r          -> reset all counts to zero
 *        s          -> stop motors immediately
 *
 * >>> SAFETY: for the powered tests (1/2/3/a) put the robot ON A STAND
 * >>> with the wheels free to spin. Keep fingers clear. <<<
 * ---------------------------------------------------------------------
 */

#include "robot_config.h"

// ------------------------------------------------------------
// Test parameters
// ------------------------------------------------------------
#define TEST_POWER      0.35f   // gentle drive power (0..1) for the powered test
#define TEST_SPIN_MS    800     // how long to drive each wheel during the test
#define PRINT_PERIOD_MS 250     // live-count print interval

// ------------------------------------------------------------
// Encoder counting — identical logic to encoder.cpp
// ------------------------------------------------------------
volatile long countA = 0, countB = 0, countC = 0;

void isrA() { if (digitalReadFast(ENC_A2)) countA++; else countA--; }
void isrB() { if (digitalReadFast(ENC_B2)) countB++; else countB--; }
void isrC() { if (digitalReadFast(ENC_C2)) countC++; else countC--; }

long readCount(volatile long &c) {
  noInterrupts();
  long v = c;
  interrupts();
  return v;
}

void resetCounts() {
  noInterrupts();
  countA = countB = countC = 0;
  interrupts();
}

// ------------------------------------------------------------
// Motor drive — identical logic to motorControl.cpp driveMotor()
// ------------------------------------------------------------
void driveMotorRaw(uint8_t enPin, uint8_t in1Pin, uint8_t in2Pin, float power) {
  power = constrain(power, -1.0f, 1.0f);
  if (power > 0.0f)      { digitalWrite(in1Pin, HIGH); digitalWrite(in2Pin, LOW);  }
  else if (power < 0.0f) { digitalWrite(in1Pin, LOW);  digitalWrite(in2Pin, HIGH); }
  else                   { digitalWrite(in1Pin, LOW);  digitalWrite(in2Pin, LOW);  }
  analogWrite(enPin, (int)(fabs(power) * MOTOR_PWM_MAX));
}

void stopAll() {
  driveMotorRaw(EN_A, IN1,   IN2,   0.0f);
  driveMotorRaw(EN_B, IN3,   IN4,   0.0f);
  driveMotorRaw(EN_C, IN3_C, IN4_C, 0.0f);
}

// ------------------------------------------------------------
// Powered sign-alignment test for a single wheel.
//
// Drives the wheel with a POSITIVE pre-DIR command — i.e. we apply the
// DIR_x_CW wiring correction here exactly as main.ino does — then reads
// the encoder delta and applies that wheel's PID_ENC_SIGN_x, exactly as
// PID.cpp measures speed. If the sign-corrected delta is positive, the
// PID feedback sign is correct; otherwise it must be flipped.
// ------------------------------------------------------------
void testWheel(char wheel) {
  uint8_t en, in1, in2;
  int dirCW, encSign;
  volatile long *counter;
  const char *signName;

  switch (wheel) {
    case 'A': en = EN_A; in1 = IN1;   in2 = IN2;   dirCW = DIR_A_CW; encSign = PID_ENC_SIGN_A; counter = &countA; signName = "PID_ENC_SIGN_A"; break;
    case 'B': en = EN_B; in1 = IN3;   in2 = IN4;   dirCW = DIR_B_CW; encSign = PID_ENC_SIGN_B; counter = &countB; signName = "PID_ENC_SIGN_B"; break;
    case 'C': en = EN_C; in1 = IN3_C; in2 = IN4_C; dirCW = DIR_C_CW; encSign = PID_ENC_SIGN_C; counter = &countC; signName = "PID_ENC_SIGN_C"; break;
    default: return;
  }

  Serial.print("\n=== Powered test: Wheel ");
  Serial.print(wheel);
  Serial.println(" (commanding POSITIVE pre-DIR) ===");

  long before = readCount(*counter);

  // Positive pre-DIR command * wiring direction = what main.ino sends.
  driveMotorRaw(en, in1, in2, TEST_POWER * dirCW);
  delay(TEST_SPIN_MS);
  driveMotorRaw(en, in1, in2, 0.0f);
  delay(200);   // let it coast to a stop before reading

  long after = readCount(*counter);
  long rawDelta = after - before;
  long signedDelta = (long)encSign * rawDelta;

  Serial.print("  raw encoder delta      : "); Serial.println(rawDelta);
  Serial.print("  after "); Serial.print(signName);
  Serial.print(" (");  Serial.print(encSign); Serial.print(") : "); Serial.println(signedDelta);

  if (rawDelta == 0) {
    Serial.println("  RESULT: NO COUNTS — encoder not wired / not powered, or");
    Serial.println("          motor not spinning. Check wiring & 3.3V to encoder.");
  } else if (signedDelta > 0) {
    Serial.print("  RESULT: OK  -> keep "); Serial.print(signName);
    Serial.print(" = "); Serial.println(encSign);
  } else {
    Serial.print("  RESULT: FLIP -> set "); Serial.print(signName);
    Serial.print(" = "); Serial.println(-encSign);
    Serial.println("          (positive command gave negative measured speed:");
    Serial.println("           PID would run away with the current sign.)");
  }
}

// ------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial && millis() < 3000) { /* wait briefly for USB serial */ }

  pinMode(ENC_A1, INPUT_PULLUP); pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP); pinMode(ENC_B2, INPUT_PULLUP);
  pinMode(ENC_C1, INPUT_PULLUP); pinMode(ENC_C2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), isrB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_C1), isrC, RISING);

  pinMode(EN_A, OUTPUT); pinMode(IN1, OUTPUT);   pinMode(IN2, OUTPUT);
  pinMode(EN_B, OUTPUT); pinMode(IN3, OUTPUT);   pinMode(IN4, OUTPUT);
  pinMode(EN_C, OUTPUT); pinMode(IN3_C, OUTPUT); pinMode(IN4_C, OUTPUT);
  stopAll();

  Serial.println("\n==================================================");
  Serial.println(" Encoder wiring & PID sign-alignment checker");
  Serial.println("==================================================");
  Serial.println(" MANUAL test : rotate each wheel by hand and watch its");
  Serial.println("   count below move smoothly. One full turn ~= CPR counts");
  Serial.print  ("   (approx "); Serial.print(MOTOR_A_CPR); Serial.println(" per rev).");
  Serial.println(" POWERED test: put robot ON A STAND, then type a command:");
  Serial.println("   1 / 2 / 3 = test wheel A / B / C   a = test all");
  Serial.println("   r = reset counts   s = stop motors");
  Serial.println("==================================================\n");
}

void loop() {
  // Handle a serial command if one arrived.
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': testWheel('A'); break;
      case '2': testWheel('B'); break;
      case '3': testWheel('C'); break;
      case 'a': case 'A':
        testWheel('A'); testWheel('B'); testWheel('C');
        break;
      case 'r': case 'R':
        resetCounts();
        Serial.println("\n[counts reset to 0]");
        break;
      case 's': case 'S':
        stopAll();
        Serial.println("\n[motors stopped]");
        break;
      default: break;   // ignore newlines / other chars
    }
  }

  // Live count display for the manual test.
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= PRINT_PERIOD_MS) {
    lastPrint = millis();
    Serial.print("A: "); Serial.print(readCount(countA));
    Serial.print("\tB: "); Serial.print(readCount(countB));
    Serial.print("\tC: "); Serial.println(readCount(countC));
  }
}
