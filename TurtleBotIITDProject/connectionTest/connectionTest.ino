/*
 * connectionTest.ino — Encoder <-> Motor connection & mapping checker
 * ---------------------------------------------------------------------
 * Standalone diagnostic. Confirms every encoder connector is on the RIGHT
 * motor header and that BOTH channels of each encoder are wired, by
 * driving ONE motor at a time and watching which encoder responds.
 *
 * For each motor it checks three separate things:
 *
 *   1. MAPPING   — driving motor X must move encoder X (and ONLY X). If a
 *                  different encoder moves instead, that connector is on
 *                  the wrong header (cross-wired).
 *   2. C1 wired  — encoder X must actually produce counts. Zero counts on
 *                  a spinning motor = the C1 (interrupt) channel is not
 *                  connected, or the motor isn't turning.
 *   3. C2 wired  — driving the motor in reverse must reverse the count. If
 *                  the count won't go the other way, the C2 (direction)
 *                  channel is disconnected / miswired.
 *
 * This is a WIRING test only. It does NOT check the PID sign convention —
 * use encoderTest.ino for that after this passes.
 *
 * ---------------------------------------------------------------------
 * HOW TO USE
 *   - Put this file in its own sketch folder named "connectionTest", next
 *     to a COPY of robot_config.h.
 *   - Open Serial Monitor at 115200 baud, line ending "Newline".
 *   - >>> Put the robot ON A STAND, wheels free, fingers clear. <<<
 *   - Type a command + Enter:
 *        a          -> full automatic test of all three motors + summary
 *        1 / 2 / 3  -> test motor A / B / C only
 *        m          -> toggle live count stream (turn wheels by hand)
 *        r          -> reset counts
 *        s          -> stop motors
 * ---------------------------------------------------------------------
 */

#include "robot_config.h"

// ------------------------------------------------------------
// Test thresholds — tune if your motors are geared very differently.
// ------------------------------------------------------------
#define TEST_POWER        0.40f  // drive power (0..1) during the test
#define SPIN_MS           700    // how long to drive each direction
#define COAST_MS          250    // settle time after stopping
#define MIN_COUNTS        150    // the driven motor's own encoder must beat this
#define CROSSTALK_COUNTS  40     // any OTHER encoder must stay below this

// ------------------------------------------------------------
// Encoder counting — identical logic to encoder.cpp
// ------------------------------------------------------------
volatile long countA = 0, countB = 0, countC = 0;

void isrA() { if (digitalReadFast(ENC_A2)) countA++; else countA--; }
void isrB() { if (digitalReadFast(ENC_B2)) countB++; else countB--; }
void isrC() { if (digitalReadFast(ENC_C2)) countC++; else countC--; }

long readCount(volatile long &c) {
  noInterrupts(); long v = c; interrupts(); return v;
}
long countByMotor(char m) {
  switch (m) { case 'A': return readCount(countA);
               case 'B': return readCount(countB);
               case 'C': return readCount(countC); default: return 0; }
}
void resetCounts() { noInterrupts(); countA = countB = countC = 0; interrupts(); }

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

// Pins for a given motor.
static void motorPins(char m, uint8_t &en, uint8_t &in1, uint8_t &in2) {
  switch (m) {
    case 'A': en = EN_A; in1 = IN1;   in2 = IN2;   break;
    case 'B': en = EN_B; in1 = IN3;   in2 = IN4;   break;
    case 'C': en = EN_C; in1 = IN3_C; in2 = IN4_C; break;
  }
}

// Spin one motor at a signed power for SPIN_MS, then stop & settle.
static void spinMotor(char m, float power) {
  uint8_t en, in1, in2; motorPins(m, en, in1, in2);
  driveMotorRaw(en, in1, in2, power);
  delay(SPIN_MS);
  driveMotorRaw(en, in1, in2, 0.0f);
  delay(COAST_MS);
}

// ------------------------------------------------------------
// Test one motor. Returns true if MAPPING + C1 + C2 all pass.
// ------------------------------------------------------------
bool testMotor(char m) {
  Serial.print("\n--- Testing Motor "); Serial.print(m); Serial.println(" ---");

  // 1 & 2) FORWARD spin: which encoder moves, and by how much?
  resetCounts();
  spinMotor(m, TEST_POWER);
  long fA = readCount(countA), fB = readCount(countB), fC = readCount(countC);
  long own = countByMotor(m);

  // Find the encoder that actually moved the most.
  long aA = labs(fA), aB = labs(fB), aC = labs(fC);
  char moved = 'A'; long movedAbs = aA;
  if (aB > movedAbs) { moved = 'B'; movedAbs = aB; }
  if (aC > movedAbs) { moved = 'C'; movedAbs = aC; }

  Serial.print("  forward counts  A="); Serial.print(fA);
  Serial.print("  B="); Serial.print(fB);
  Serial.print("  C="); Serial.println(fC);

  bool mappingOk = false;

  if (labs(own) < MIN_COUNTS) {
    // This motor's own encoder barely moved. Did a DIFFERENT one move?
    if (movedAbs >= MIN_COUNTS && moved != m) {
      Serial.print("  [FAIL] MAPPING: driving Motor "); Serial.print(m);
      Serial.print(" moved Encoder "); Serial.print(moved);
      Serial.println(" instead — that encoder connector is on the wrong header.");
    } else {
      Serial.print("  [FAIL] NO COUNTS on Encoder "); Serial.print(m);
      Serial.println(" — C1 (interrupt) pin not connected, or motor not spinning.");
      Serial.println("         (If the motor clearly spun, suspect C1<->C2 swapped.)");
    }
  } else {
    // Own encoder moved. Make sure the others stayed quiet.
    bool crosstalk = false;
    if (m != 'A' && aA > CROSSTALK_COUNTS) crosstalk = true;
    if (m != 'B' && aB > CROSSTALK_COUNTS) crosstalk = true;
    if (m != 'C' && aC > CROSSTALK_COUNTS) crosstalk = true;
    if (crosstalk) {
      Serial.print("  [WARN] Encoder "); Serial.print(m);
      Serial.println(" moved, but another encoder also changed — check for a");
      Serial.println("         shared/crossed line or a loose connector.");
      mappingOk = true;   // primary mapping is right; flag for inspection
    } else {
      Serial.print("  [ OK ] MAPPING: Motor "); Serial.print(m);
      Serial.print(" drives Encoder "); Serial.print(m);
      Serial.print(" ("); Serial.print(own); Serial.println(" counts).");
      mappingOk = true;
    }
  }

  // 3) DIRECTION / C2 check: reverse must push the count the other way.
  bool directionOk = false;
  if (mappingOk) {
    resetCounts();
    spinMotor(m, -TEST_POWER);
    long rev = countByMotor(m);
    Serial.print("  reverse count   "); Serial.print(m); Serial.print("=");
    Serial.println(rev);

    // Forward gave sign(own); reverse should give the opposite sign.
    if (labs(rev) >= MIN_COUNTS && ((own > 0) != (rev > 0))) {
      Serial.println("  [ OK ] DIRECTION: count reverses — C2 (direction) wired.");
      directionOk = true;
    } else {
      Serial.println("  [FAIL] DIRECTION: count did not cleanly reverse —");
      Serial.println("         C2 (direction) pin likely disconnected/miswired.");
    }
  }

  bool pass = mappingOk && directionOk;
  Serial.print("  => Motor "); Serial.print(m);
  Serial.println(pass ? ": PASS" : ": CHECK WIRING");
  return pass;
}

void testAll() {
  Serial.println("\n================ FULL CONNECTION TEST ================");
  bool a = testMotor('A');
  bool b = testMotor('B');
  bool c = testMotor('C');
  Serial.println("\n===================== SUMMARY ========================");
  Serial.print("  Motor A / Encoder A : "); Serial.println(a ? "PASS" : "CHECK WIRING");
  Serial.print("  Motor B / Encoder B : "); Serial.println(b ? "PASS" : "CHECK WIRING");
  Serial.print("  Motor C / Encoder C : "); Serial.println(c ? "PASS" : "CHECK WIRING");
  Serial.println("=====================================================");
  if (a && b && c)
    Serial.println("  All connections good. Next: run encoderTest.ino for PID signs.");
  Serial.println();
}

// ------------------------------------------------------------
static bool manualMode = false;

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
  Serial.println(" Encoder <-> Motor connection & mapping checker");
  Serial.println("==================================================");
  Serial.println(" ROBOT ON A STAND. Commands:");
  Serial.println("   a = test all   1/2/3 = test motor A/B/C");
  Serial.println("   m = toggle live counts   r = reset   s = stop");
  Serial.println("==================================================\n");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'a': case 'A': manualMode = false; testAll();      break;
      case '1': manualMode = false; testMotor('A');           break;
      case '2': manualMode = false; testMotor('B');           break;
      case '3': manualMode = false; testMotor('C');           break;
      case 'm': case 'M':
        manualMode = !manualMode;
        Serial.println(manualMode ? "\n[live counts ON — turn wheels by hand]"
                                  : "\n[live counts OFF]");
        break;
      case 'r': case 'R': resetCounts(); Serial.println("\n[counts reset]"); break;
      case 's': case 'S': stopAll();     Serial.println("\n[motors stopped]"); break;
      default: break;
    }
  }

  if (manualMode) {
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 250) {
      lastPrint = millis();
      Serial.print("A: "); Serial.print(readCount(countA));
      Serial.print("\tB: "); Serial.print(readCount(countB));
      Serial.print("\tC: "); Serial.println(readCount(countC));
    }
  }
}
