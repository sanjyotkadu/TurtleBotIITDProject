#include "PID.h"
#include "robot_config.h"
#include "encoder.h"
#include <math.h>

// ============================================================
//  Three PID controllers from the Arduino PID library, one per wheel.
//
//  The library talks to us through pointers to double variables:
//    Input    = measured wheel speed (mm/s)   [we write it]
//    Setpoint = commanded wheel speed (mm/s)  [we write it]
//    Output   = normalized power -1..+1        [library writes it]
//
//  ============ MUST DO BEFORE ENABLING PID ON HARDWARE ============
//  1. Install the "PID by Brett Beauregard" library (Library Manager).
//  2. Prop the robot up so the wheels spin free. Command a wheel
//     positive (pre-DIR) and confirm pidMeasuredSpeed() for that wheel
//     reads POSITIVE. If it reads negative, flip that wheel's
//     PID_ENC_SIGN_x in robot_config.h. A mismatched sign turns the loop
//     into POSITIVE feedback and the wheel runs away to full speed.
//  3. Measure PID_MAX_WHEEL_SPEED_MM_S (top speed at PWM 255) and set it.
//  4. Tune the gains (recipe below) with the robot ON A STAND first.
//
//  ============ TUNING RECIPE (per wheel, or all three together) ====
//    a. Kp only (Ki = Kd = 0). Raise Kp until the wheel responds crisply
//       to a stick step but only just starts to oscillate; back off ~30%.
//    b. Add Ki until the steady-state gap between commanded and measured
//       speed is removed within a fraction of a second. Too much Ki =
//       slow oscillation / overshoot.
//    c. Only if needed, add a little Kd to damp overshoot. Encoder speed
//       is noisy, so keep Kd small; too much makes it buzz/chatter.
//    (Ki/Kd here are in standard per-second units — the library scales
//     them by the sample time internally, so don't pre-scale them.)
// ============================================================

// PID_v1 works on doubles referenced by pointer. One trio per wheel.
static double inA = 0, outA = 0, setA = 0;
static double inB = 0, outB = 0, setB = 0;
static double inC = 0, outC = 0, setC = 0;

// DIRECT = a positive error drives a positive output. Once the encoder
// sign is aligned (step 2 above), this is correct for all three wheels.
static PID pidA(&inA, &outA, &setA, PID_KP, PID_KI, PID_KD, DIRECT);
static PID pidB(&inB, &outB, &setB, PID_KP, PID_KI, PID_KD, DIRECT);
static PID pidC(&inC, &outC, &setC, PID_KP, PID_KI, PID_KD, DIRECT);

// Encoder snapshot + timing for fixed-rate speed measurement.
static long lastCountA = 0, lastCountB = 0, lastCountC = 0;
static unsigned long lastMeasMs = 0;

// Held output (zero-order hold between measurement ticks).
static WheelPowers heldPowers = {0.0f, 0.0f, 0.0f};

// Last measured speeds (mm/s, pre-DIR frame) for telemetry.
static float measSpeedA = 0.0f, measSpeedB = 0.0f, measSpeedC = 0.0f;

static void configureController(PID &pid) {
  // Output is a normalized motor power, so the loop must be free to drive
  // both directions. (Library default is 0..255 — that would forbid
  // reverse, so setting symmetric limits here is essential.)
  pid.SetOutputLimits(-1.0, 1.0);
  pid.SetSampleTime(PID_SAMPLE_TIME_MS);
  pid.SetMode(AUTOMATIC);
}

void pidInit() {
  configureController(pidA);
  configureController(pidB);
  configureController(pidC);

  lastCountA = encoderCount('A');
  lastCountB = encoderCount('B');
  lastCountC = encoderCount('C');
  lastMeasMs = millis();

  heldPowers.front_C = heldPowers.backLeft_A = heldPowers.backRight_B = 0.0f;
  measSpeedA = measSpeedB = measSpeedC = 0.0f;
}

void pidResetAll() {
  // The library clears its integrator (ITerm) whenever it re-enters
  // AUTOMATIC: Initialize() sets ITerm = current Output. So zero the
  // outputs first, then bounce MANUAL -> AUTOMATIC to get a clean start.
  outA = outB = outC = 0.0;
  pidA.SetMode(MANUAL); pidA.SetMode(AUTOMATIC);
  pidB.SetMode(MANUAL); pidB.SetMode(AUTOMATIC);
  pidC.SetMode(MANUAL); pidC.SetMode(AUTOMATIC);

  // Re-baseline the encoders so the first speed sample after re-enabling
  // isn't a huge delta accumulated while the motors were off.
  lastCountA = encoderCount('A');
  lastCountB = encoderCount('B');
  lastCountC = encoderCount('C');
  lastMeasMs = millis();

  heldPowers.front_C = heldPowers.backLeft_A = heldPowers.backRight_B = 0.0f;
}

// Convert one wheel's encoder delta into speed (mm/s) in the pre-DIR
// command frame, applying the per-wheel encoder sign alignment.
static float measureWheelSpeed(long nowCount, long &lastCount,
                               float mmPerCount, int encSign, float dt) {
  long delta = nowCount - lastCount;
  lastCount = nowCount;
  return encSign * (delta * mmPerCount) / dt;   // (counts * mm/count) / s
}

WheelPowers pidUpdate(WheelPowers target) {
  unsigned long now = millis();
  unsigned long elapsed = now - lastMeasMs;

  // Refresh the measured speed on the same fixed interval the PID runs
  // at, so each sample accumulates enough encoder counts to be clean.
  // Between ticks the Input holds its last value.
  if (elapsed >= (unsigned long)PID_SAMPLE_TIME_MS) {
    float dt = elapsed / 1000.0f;   // seconds
    lastMeasMs = now;

    measSpeedA = measureWheelSpeed(encoderCount('A'), lastCountA,
                                   MOTOR_A_MM_PER_COUNT, PID_ENC_SIGN_A, dt);
    measSpeedB = measureWheelSpeed(encoderCount('B'), lastCountB,
                                   MOTOR_B_MM_PER_COUNT, PID_ENC_SIGN_B, dt);
    measSpeedC = measureWheelSpeed(encoderCount('C'), lastCountC,
                                   MOTOR_C_MM_PER_COUNT, PID_ENC_SIGN_C, dt);

    inA = measSpeedA;
    inB = measSpeedB;
    inC = measSpeedC;
  }

  // Setpoints: normalized command -> speed setpoint (mm/s). Cheap, so
  // update every call; the library only acts on the sample interval.
  setA = target.backLeft_A  * PID_MAX_WHEEL_SPEED_MM_S;
  setB = target.backRight_B * PID_MAX_WHEEL_SPEED_MM_S;
  setC = target.front_C     * PID_MAX_WHEEL_SPEED_MM_S;

  // Standard PID library usage: call Compute() every loop; it self-gates
  // on SetSampleTime() and only recomputes Output when a tick is due.
  pidA.Compute();
  pidB.Compute();
  pidC.Compute();

  heldPowers.backLeft_A  = (float)outA;
  heldPowers.backRight_B = (float)outB;
  heldPowers.front_C     = (float)outC;
  return heldPowers;
}

float pidMeasuredSpeed(char motor) {
  switch (motor) {
    case 'A': return measSpeedA;
    case 'B': return measSpeedB;
    case 'C': return measSpeedC;
    default:  return 0.0f;
  }
}
