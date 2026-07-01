#include "robot_config.h"

// ============================================================
// TurtleBot Omni Drive + PID Velocity Control — Teensy 4.1
// F=Forward  B=Backward  L=Strafe Left  R=Strafe Right
// <=Rotate CCW  >=Rotate CW  S=Stop  +=Faster  -=Slower  D=Debug
// ============================================================

// --- PID struct (must be declared before any function that uses it) ---
struct MotorPID {
  float target;
  float integral;
  float lastError;
  float velocity;
  int   pwm;
  long  lastCount;
};

// --- Global state ---
#define PID_INTERVAL_MS  20

float targetSpeed = 1200.0f;
float Kp = 0.12f, Ki = 0.01f, Kd = 0.005f;  // counts/sec at full command

volatile long encA = 0, encB = 0, encC = 0;
MotorPID pidA, pidB, pidC;
unsigned long lastPIDTime = 0;

// --- Encoder ISRs ---
void isrA() { encA += (digitalRead(ENC_A1) == digitalRead(ENC_A2)) ? 1 : -1; }
void isrB() { encB += (digitalRead(ENC_B1) == digitalRead(ENC_B2)) ? 1 : -1; }
void isrC() { encC += (digitalRead(ENC_C1) == digitalRead(ENC_C2)) ? -1 : 1; }

// --- Motor control ---
void setMotor(char m, int pwm) {
  pwm = constrain(pwm, -255, 255);
  switch (m) {
    case 'A': digitalWrite(IN1, pwm>0); digitalWrite(IN2, pwm<0); analogWrite(EN_A, abs(pwm)); break;
    case 'B': digitalWrite(IN3, pwm>0); digitalWrite(IN4, pwm<0); analogWrite(EN_B, abs(pwm)); break;
    case 'C': digitalWrite(IN3_C, pwm>0); digitalWrite(IN4_C, pwm<0); analogWrite(EN_C, abs(pwm)); break;
  }
}

void stopAll() {
  pidA.target = pidB.target = pidC.target = 0;
  pidA.integral = pidB.integral = pidC.integral = 0;
  pidA.pwm = pidB.pwm = pidC.pwm = 0;
  setMotor('A', 0); setMotor('B', 0); setMotor('C', 0);
}

// --- PID update ---
int updatePID(MotorPID &pid, long currentCount, float dt) {
  pid.velocity = (currentCount - pid.lastCount) / dt;
  pid.lastCount = currentCount;
  if (pid.target == 0) { pid.integral = 0; pid.lastError = 0; return 0; }

  float error = pid.target - pid.velocity;
  pid.integral = constrain(pid.integral + error * dt, -800.0f, 800.0f);
  float derivative = (error - pid.lastError) / dt;
  pid.lastError = error;

  // Direct output — not incremental
  int output = (int)(Kp * error + Ki * pid.integral + Kd * derivative);
  pid.pwm = constrain(output, -255, 255);
  if (abs(pid.pwm) < 25) pid.pwm = (pid.target > 0) ? 25 : -25;
  return pid.pwm;
}

// --- Inverse kinematics ---
void driveRobot(float vx, float vy, float omega) {
  // Reset integrals on new command
  pidA.integral = 0; pidB.integral = 0; pidC.integral = 0;
  pidA.pwm = 0; pidB.pwm = 0; pidC.pwm = 0;

  float wC = -vx                   + omega;
  float wA =  0.5f * vx - 0.866f * vy + omega;
  float wB =  0.5f * vx + 0.866f * vy + omega;
  float maxVal = max(abs(wC), max(abs(wA), abs(wB)));
  if (maxVal > 1.0f) { wC /= maxVal; wA /= maxVal; wB /= maxVal; }
  pidA.target = wA * targetSpeed;
  pidB.target = wB * targetSpeed;
  pidC.target = wC * targetSpeed;
}
void setup() {
  pinMode(EN_A,OUTPUT); pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT);
  pinMode(EN_B,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(EN_C,OUTPUT); pinMode(IN3_C,OUTPUT); pinMode(IN4_C,OUTPUT);
  stopAll();

  pinMode(ENC_A1,INPUT_PULLUP); pinMode(ENC_A2,INPUT_PULLUP);
  pinMode(ENC_B1,INPUT_PULLUP); pinMode(ENC_B2,INPUT_PULLUP);
  pinMode(ENC_C1,INPUT_PULLUP); pinMode(ENC_C2,INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), isrB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_C1), isrC, CHANGE);

  Serial.begin(BAUD_RATE);
  while (!Serial && millis() < 3000);

  Serial.println("========================================");
  Serial.println("  TurtleBot Omni Drive + PID");
  Serial.println("  Teensy 4.1");
  Serial.println("========================================");
  Serial.println("  F=Forward   B=Backward   S=Stop");
  Serial.println("  L=Strafe Left   R=Strafe Right");
  Serial.println("  <=Rotate CCW   >=Rotate CW");
  Serial.println("  +=Speed Up   -=Speed Down   D=Debug");
  Serial.println("========================================\n");

  lastPIDTime = millis();
}

void loop() {
  // PID loop at 50Hz
  unsigned long now = millis();
  if (now - lastPIDTime >= PID_INTERVAL_MS) {
    float dt = (now - lastPIDTime) / 1000.0f;
    lastPIDTime = now;
    noInterrupts();
    long cA = encA, cB = encB, cC = encC;
    interrupts();
    setMotor('A', updatePID(pidA, cA, dt));
    setMotor('B', updatePID(pidB, cB, dt));
    setMotor('C', updatePID(pidC, cC, dt));
  }

  // Serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    while (Serial.available()) Serial.read();
    switch (cmd) {
      case 'F': case 'f': driveRobot( 0,  1,  0); Serial.println(">> FORWARD");      break;
      case 'B': case 'b': driveRobot( 0, -1,  0); Serial.println(">> BACKWARD");     break;
      case 'L': case 'l': driveRobot(-1,  0,  0); Serial.println(">> STRAFE LEFT");  break;
      case 'R': case 'r': driveRobot( 1,  0,  0); Serial.println(">> STRAFE RIGHT"); break;
      case '<':           driveRobot( 0,  0,  1); Serial.println(">> ROTATE CCW");   break;
      case '>':           driveRobot( 0,  0, -1); Serial.println(">> ROTATE CW");    break;
      case 'S': case 's': stopAll();              Serial.println(">> STOP");          break;
      case '+': targetSpeed = min(targetSpeed + 200.0f, 6000.0f); Serial.print(">> Speed: "); Serial.println(targetSpeed); break;
      case '-': targetSpeed = max(targetSpeed - 200.0f,  200.0f); Serial.print(">> Speed: "); Serial.println(targetSpeed); break;
      case 'D': case 'd':
        Serial.print("A  vel:"); Serial.print(pidA.velocity,0); Serial.print("  tgt:"); Serial.print(pidA.target,0); Serial.print("  pwm:"); Serial.println(pidA.pwm);
        Serial.print("B  vel:"); Serial.print(pidB.velocity,0); Serial.print("  tgt:"); Serial.print(pidB.target,0); Serial.print("  pwm:"); Serial.println(pidB.pwm);
        Serial.print("C  vel:"); Serial.print(pidC.velocity,0); Serial.print("  tgt:"); Serial.print(pidC.target,0); Serial.print("  pwm:"); Serial.println(pidC.pwm);
        break;
    }
  }
}