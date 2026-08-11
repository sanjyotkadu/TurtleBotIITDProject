// ============================================================
// PID_Simple_Learning.ino
//
// GOAL: Build intuition for how PID correction works, using
//       a setpoint you define and feedback YOU type manually
//       over Serial (simulating a sensor reading).
//
// Flow:
//   setpoint = a + b                (your target value)
//   feedback = typed by you         (simulated sensor reading)
//   error    = setpoint - feedback
//   output   = P + I + D            (corrected power value)
//
// This version also PRINTS THE MATH for each term: it shows the
// formula for P, I and D with the actual numbers substituted in,
// so you can follow exactly how every value is produced.
//
// TIME MODEL (important for correct analysis):
//   A real controller runs at a FIXED sample period dt: every tick
//   it gets one fresh reading. If we measured dt from your typing
//   speed, dt would be random and the I and D terms would be
//   meaningless. So instead each feedback value you type counts as
//   ONE control cycle of a fixed step (simDt). The simulated clock
//   (simTime) advances by simDt with every value you enter.
//   Type 'd 0.1' to change the step to 0.1 s, etc.
//
// Open Serial Monitor at 115200 baud, line ending = Newline.
// ============================================================

float a = 30.0;
float b = 20.0;
float setpoint;

// ---- PID gains: change these and re-upload to see the effect ----
float Kp = 1;//2.0;   // reacts to current error
float Ki = 1;//0.5;   // reacts to accumulated error over time
float Kd = 1;//0.1;   // reacts to how fast error is changing

float error = 0;
float prevError = 0;
float integral = 0;
float output = 0;

// Anti-windup clamp for the integral term. Since YOU control how
// fast feedback arrives (typing speed), dt can be large and the
// integral can blow up otherwise. Tune this if needed.
float integralLimit = 50.0;

// ---- Simulated time step ----
// FIXED time step per control cycle. Each feedback value you type is
// treated as one cycle, so dt is ALWAYS this value regardless of how
// long you took to type. Change it with the 'd' command (e.g. 'd 0.1').
float simDt = 1.0;      // seconds per control cycle
float simTime = 0.0;    // total simulated time; += simDt each input

void setup() {
  Serial.begin(115200);
  while (!Serial) { }  // wait for USB serial on Teensy

  setpoint = a + b;

  Serial.println("=== Simple PID Learning Tool ===");
  Serial.print("Setpoint (a + b) = ");
  Serial.println(setpoint);
  Serial.println();
  Serial.print("Gains -> Kp = "); Serial.print(Kp);
  Serial.print(", Ki = "); Serial.print(Ki);
  Serial.print(", Kd = "); Serial.println(Kd);
  Serial.print("Fixed time step dt = "); Serial.print(simDt, 3);
  Serial.println(" s per input");
  Serial.println();
  Serial.println("Type a feedback value and press Enter (= one control cycle).");
  Serial.println("Try values below, equal to, and above the setpoint.");
  Serial.println("Type 's' then two numbers (e.g. 's 40 10') to set new a b.");
  Serial.println("Type 'd' then a number (e.g. 'd 0.1') to set the time step dt.");
  Serial.println();
}

void loop() {
  if (Serial.available() > 0) {
    char firstChar = Serial.peek();

    if (firstChar == 's' || firstChar == 'S') {
      Serial.read(); // consume 's'
      float newA = Serial.parseFloat();
      float newB = Serial.parseFloat();
      a = newA;
      b = newB;
      setpoint = a + b;
      integral = 0;      // reset history since target changed
      prevError = 0;
      Serial.print("New setpoint (a + b) = ");
      Serial.println(setpoint);
    } else if (firstChar == 'd' || firstChar == 'D') {
      Serial.read(); // consume 'd'
      float newDt = Serial.parseFloat();
      if (newDt > 0) {
        simDt = newDt;
        Serial.print("New time step dt = ");
        Serial.print(simDt, 3);
        Serial.println(" s per input");
      } else {
        Serial.println("dt must be > 0; keeping previous value.");
      }
    } else {
      float feedback = Serial.parseFloat();

      // Fixed time step: each input is exactly one control cycle of
      // simDt seconds. The simulated clock advances by that step, so
      // dt is constant and the I/D math is comparable across inputs.
      float dt = simDt;
      simTime += simDt;

      // ---- Capture the "before" values so we can show the math ----
      // The integral and prevError get overwritten below, but the
      // printout needs the values that went INTO the calculation.
      float integralBefore = integral;
      float prevErrorForPrint = prevError;

      error = setpoint - feedback;

      // Integral: running sum of error * dt (area under the error curve).
      integral += error * dt;
      // Remember the raw sum before clamping, so we can show whether the
      // anti-windup limit actually kicked in.
      float integralUnclamped = integral;
      if (integral > integralLimit) integral = integralLimit;
      if (integral < -integralLimit) integral = -integralLimit;

      // Derivative: how fast the error is changing since last time.
      float derivative = (error - prevError) / dt;

      float P = Kp * error;
      float I = Ki * integral;
      float D = Kd * derivative;

      output = P + I + D;

      prevError = error;

      // ================= RESULTS =================
      Serial.println("----------------------------");
      Serial.print("Setpoint   : "); Serial.println(setpoint);
      Serial.print("Feedback   : "); Serial.println(feedback);
      Serial.print("Error      : "); Serial.println(error);
      Serial.print("dt (s)     : "); Serial.println(dt, 3);
      Serial.print("Sim time(s): "); Serial.println(simTime, 3);

      // ============ HOW EACH TERM IS CALCULATED ============
      // Each block prints the formula, then the same formula with the
      // real numbers plugged in, then the result. Follow the "=" signs.
      Serial.println();
      Serial.println(">> How the calculations happen:");

      // ---- Error ----
      Serial.print("   error = setpoint - feedback = ");
      Serial.print(setpoint); Serial.print(" - "); Serial.print(feedback);
      Serial.print(" = "); Serial.println(error);

      // ---- P term ----
      // P answers: "how far off am I RIGHT NOW?"
      Serial.print("   P = Kp * error = ");
      Serial.print(Kp); Serial.print(" * "); Serial.print(error);
      Serial.print(" = "); Serial.println(P);

      // ---- I term ----
      // I answers: "how much error have I built up OVER TIME?"
      // First show how the running integral was updated this step...
      Serial.print("   integral += error * dt -> ");
      Serial.print(integralBefore); Serial.print(" + ");
      Serial.print(error); Serial.print(" * "); Serial.print(dt, 3);
      Serial.print(" = "); Serial.print(integralUnclamped);
      if (integralUnclamped != integral) {
        // Anti-windup clamp changed the value; make that visible.
        Serial.print("  (clamped to "); Serial.print(integral); Serial.print(")");
      }
      Serial.println();
      // ...then how that integral becomes the I term.
      Serial.print("   I = Ki * integral = ");
      Serial.print(Ki); Serial.print(" * "); Serial.print(integral);
      Serial.print(" = "); Serial.println(I);

      // ---- D term ----
      // D answers: "how FAST is the error changing?"
      // First show the derivative (slope of error vs time)...
      Serial.print("   derivative = (error - prevError) / dt = (");
      Serial.print(error); Serial.print(" - "); Serial.print(prevErrorForPrint);
      Serial.print(") / "); Serial.print(dt, 3);
      Serial.print(" = "); Serial.println(derivative);
      // ...then how that derivative becomes the D term.
      Serial.print("   D = Kd * derivative = ");
      Serial.print(Kd); Serial.print(" * "); Serial.print(derivative);
      Serial.print(" = "); Serial.println(D);

      // ---- Output ----
      Serial.print("   output = P + I + D = ");
      Serial.print(P); Serial.print(" + "); Serial.print(I);
      Serial.print(" + "); Serial.print(D);
      Serial.print(" = "); Serial.println(output);
      Serial.println();

      // ================= SUMMARY =================
      Serial.print("P term     : "); Serial.println(P);
      Serial.print("I term     : "); Serial.println(I);
      Serial.print("D term     : "); Serial.println(D);
      Serial.print("PID Output : "); Serial.println(output);
    }

    // flush leftover characters (e.g. newline)
    while (Serial.available() > 0) Serial.read();

    Serial.println();
    Serial.println("Enter next feedback value:");
  }
}
