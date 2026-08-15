#include "encoder.h"
#include "robot_config.h"

// Signed, cumulative counts. volatile: written from ISR context.
static volatile long countA = 0;
static volatile long countB = 0;
static volatile long countC = 0;

// One ISR per motor. Fires on a rising edge of channel C1; the level of
// channel C2 at that instant gives the rotation direction.
static void isrA() { if (digitalReadFast(ENC_A2)) countA++; else countA--; }
static void isrB() { if (digitalReadFast(ENC_B2)) countB++; else countB--; }
static void isrC() { if (digitalReadFast(ENC_C2)) countC++; else countC--; }

void encoderInit() {
  pinMode(ENC_A1, INPUT_PULLUP); pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP); pinMode(ENC_B2, INPUT_PULLUP);
  pinMode(ENC_C1, INPUT_PULLUP); pinMode(ENC_C2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), isrA, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), isrB, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_C1), isrC, RISING);
}

// Snapshot a volatile counter with interrupts briefly disabled so a
// 32-bit read can't tear against a concurrent ISR update.
static long readCount(volatile long &c) {
  noInterrupts();
  long v = c;
  interrupts();
  return v;
}

long encoderCount(char motor) {
  switch (motor) {
    case 'A': return readCount(countA);
    case 'B': return readCount(countB);
    case 'C': return readCount(countC);
    default:  return 0;
  }
}

float encoderDistanceMM(char motor) {
  switch (motor) {
    case 'A': return encoderCount('A') * MOTOR_A_MM_PER_COUNT;
    case 'B': return encoderCount('B') * MOTOR_B_MM_PER_COUNT;
    case 'C': return encoderCount('C') * MOTOR_C_MM_PER_COUNT;
    default:  return 0.0f;
  }
}

void encoderReset(char motor) {
  noInterrupts();
  switch (motor) {
    case 'A': countA = 0; break;
    case 'B': countB = 0; break;
    case 'C': countC = 0; break;
  }
  interrupts();
}

void encoderResetAll() {
  noInterrupts();
  countA = 0;
  countB = 0;
  countC = 0;
  interrupts();
}
