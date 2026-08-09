/*
 * maxSpeedTest.ino — Measure each wheel's top speed for PID scaling
 * ---------------------------------------------------------------------
 * Drives each wheel OPEN-LOOP at full PWM (255) and measures its
 * steady-state speed in mm/s from the encoder. Use the result to set
 * PID_MAX_WHEEL_SPEED_MM_S in robot_config.h — that constant is the
 * exchange rate between "full stick" and "mm/s setpoint", so it must be a
 * speed the wheel can actually reach.
 *
 * HOW IT MEASURES: spins the wheel, waits RUNUP_MS for it to reach a
 * steady speed, then counts encoder ticks over MEASURE_MS and converts:
 *     speed(mm/s) = counts * mm_per_count / seconds
 *
 * ---------------------------------------------------------------------
 * HOW TO USE
 *   - Put this file in its own sketch folder named "maxSpeedTest", next
 *     to a COPY of robot_config.h.
 *   - Serial Monitor at 115200 baud, line ending "Newline".
 *   - Read the STAND vs GROUND note the sketch prints before you trust
 *     the number. >>> Robot on a stand, wheels free, fingers clear. <<<
 *   - Commands:
 *        a          -> measure all three wheels + print a suggested value
 *        1 / 2 / 3  -> measure wheel A / B / C only
 *        s          -> stop motors
 * ---------------------------------------------------------------------
 */

#include "robot_config.h"

// ------------------------------------------------------------
// Test parameters
// ------------------------------------------------------------
#define FULL_POWER   1.0f    // 1.0 = PWM 255 (the definition of "max")
#define RUNUP_MS     900     // let the wheel reach steady speed first
#define MEASURE_MS   1000    // measurement window
#define HEADROOM     0.85f   // suggest 85% of the slowest wheel, so the PID
                             // keeps PWM headroom to correct at full stick

// ------------------------------------------------------------
// Encoder counting — identical logic to encoder.cpp
// ------------------------------------------------------------
volatile long countA = 0, countB = 0, countC = 0;
void isrA() { if (digitalReadFast(ENC_A2)) countA++; else countA--; }
void isrB() { if (digitalReadFast(ENC_B2)) countB++; else countB--; }
void isrC() { if (digitalReadFast(ENC_C2)) countC++; else countC--; }
long readCount(volatile long &c) { noInterrupts(); long v = c; interrupts(); return v; }
long countByMotor(char m) {
  switch (m) { case 'A': return readCount(countA);
               case 'B': return readCount(countB);
               case 'C': return readCount(countC); default: return 0; }
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
static void motorPins(char m, uint8_t &en, uint8_t &in1, uint8_t &in2) {
  switch (m) {
    case 'A': en = EN_A; in1 = IN1;   in2 = IN2;   break;
    case 'B': en = EN_B; in1 = IN3;   in2 = IN4;   break;
    case 'C': en = EN_C; in1 = IN3_C; in2 = IN4_C; break;
  }
}
static float mmPerCount(char m) {
  switch (m) { case 'A': return MOTOR_A_MM_PER_COUNT;
               case 'B': return MOTOR_B_MM_PER_COUNT;
               case 'C': return MOTOR_C_MM_PER_COUNT; default: return 0.0f; }
}

// ------------------------------------------------------------
// Measure one wheel's steady-state top speed (mm/s), always positive.
// ------------------------------------------------------------
float measureMaxSpeed(char m) {
  uint8_t en, in1, in2; motorPins(m, en, in1, in2);

  driveMotorRaw(en, in1, in2, FULL_POWER);
  delay(RUNUP_MS);                          // reach steady speed

  long c0 = countByMotor(m);
  unsigned long t0 = millis();
  delay(MEASURE_MS);                        // measurement window
  long c1 = countByMotor(m);
  unsigned long t1 = millis();

  driveMotorRaw(en, in1, in2, 0.0f);        // stop

  float seconds = (t1 - t0) / 1000.0f;
  float speed = fabs((c1 - c0) * mmPerCount(m)) / seconds;

  Serial.print("  Wheel "); Serial.print(m);
  Serial.print(" : "); Serial.print(speed, 1); Serial.print(" mm/s  (");
  Serial.print(labs(c1 - c0)); Serial.print(" counts in ");
  Serial.print(seconds, 2); Serial.println(" s)");
  delay(400);                               // let it fully stop between wheels
  return speed;
}

void measureAll() {
  Serial.println("\n============ MAX WHEEL SPEED @ PWM 255 ============");
  float sa = measureMaxSpeed('A');
  float sb = measureMaxSpeed('B');
  float sc = measureMaxSpeed('C');

  float slowest = sa; if (sb < slowest) slowest = sb; if (sc < slowest) slowest = sc;
  float suggest = slowest * HEADROOM;

  Serial.println("---------------------------------------------------");
  Serial.print("  Slowest wheel: "); Serial.print(slowest, 1); Serial.println(" mm/s");
  Serial.print("  Suggested PID_MAX_WHEEL_SPEED_MM_S = ");
  Serial.print(suggest, 0);
  Serial.print("   (85% of slowest; round to a tidy number, e.g. ");
  Serial.print(((long)(suggest / 10.0f)) * 10L); Serial.println(")");
  Serial.println("  NOTE: measured on a STAND = free-spin. On the ground");
  Serial.println("  under load it will be lower — see the printed guidance.");
  Serial.println("===================================================\n");
}

// ------------------------------------------------------------
void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial && millis() < 3000) { }

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
  Serial.println(" Max wheel speed measurement (for PID scaling)");
  Serial.println("==================================================");
  Serial.println(" ROBOT ON A STAND, wheels free. Commands:");
  Serial.println("   a = measure all   1/2/3 = wheel A/B/C   s = stop");
  Serial.println(" Each wheel spins at FULL power for ~2 s.");
  Serial.println(" Battery should be reasonably charged for a fair number.");
  Serial.println("==================================================\n");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'a': case 'A': measureAll();                          break;
      case '1': Serial.println(); measureMaxSpeed('A');          break;
      case '2': Serial.println(); measureMaxSpeed('B');          break;
      case '3': Serial.println(); measureMaxSpeed('C');          break;
      case 's': case 'S': stopAll(); Serial.println("[stopped]"); break;
      default: break;
    }
  }
}
