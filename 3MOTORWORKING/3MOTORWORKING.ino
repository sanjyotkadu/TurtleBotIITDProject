// Teensy 4.1 - Three Motor Driver Test Sketch with Encoders
//
// Driver 1 (Motors A & B):
//   Pin 9  -> ENA  (Motor A enable / PWM speed)
//   Pin 8  -> IN1  (Motor A direction)
//   Pin 6  -> IN2  (Motor A direction)
//   Pin 4  -> IN3  (Motor B direction)
//   Pin 3  -> IN4  (Motor B direction)
//   Pin 2  -> ENB  (Motor B enable / PWM speed)
//
// Driver 2 (Motor C):
//   Pin 5  -> EN   (Motor C enable / PWM speed)
//   Pin 10 -> IN3  (Motor C direction)
//   Pin 11 -> IN4  (Motor C direction)
//
// Encoders:
//   Pin 12 -> Motor A Encoder C1
//   Pin 14 -> Motor A Encoder C2
//   Pin 15 -> Motor B Encoder C1
//   Pin 16 -> Motor B Encoder C2
//   Pin 17 -> Motor C Encoder C1
//   Pin 18 -> Motor C Encoder C2

// --- Motor pins ---
const int EN_A = 9;
const int IN1  = 8;
const int IN2  = 6;
const int IN3  = 4;
const int IN4  = 3;
const int EN_B = 2;

const int EN_C  = 5;
const int IN3_C = 10;
const int IN4_C = 11;

// --- Encoder pins ---
// NOTE: Motor A and Motor B encoder connectors were found to be swapped,
//       so Motor A reads pins 15/16 and Motor B reads pins 12/14.
const int ENC_A1 = 15;
const int ENC_A2 = 16;
const int ENC_B1 = 12;
const int ENC_B2 = 14;
const int ENC_C1 = 17;
const int ENC_C2 = 18;

// --- Encoder counters ---
volatile long encA = 0;
volatile long encB = 0;
volatile long encC = 0;

// --- Encoder ISRs ---
void isrA() { encA += (digitalRead(ENC_A1) == digitalRead(ENC_A2)) ? 1 : -1; }
void isrB() { encB += (digitalRead(ENC_B1) == digitalRead(ENC_B2)) ? 1 : -1; }
void isrC() { encC += (digitalRead(ENC_C1) == digitalRead(ENC_C2)) ? 1 : -1; }

// --- Motor control ---
void motorA(int speed) {  // -255 to 255
  digitalWrite(IN1, speed > 0 ? HIGH : LOW);
  digitalWrite(IN2, speed < 0 ? HIGH : LOW);
  analogWrite(EN_A, abs(speed));
}

void motorB(int speed) {
  digitalWrite(IN3, speed > 0 ? HIGH : LOW);
  digitalWrite(IN4, speed < 0 ? HIGH : LOW);
  analogWrite(EN_B, abs(speed));
}

void motorC(int speed) {
  digitalWrite(IN3_C, speed > 0 ? HIGH : LOW);
  digitalWrite(IN4_C, speed < 0 ? HIGH : LOW);
  analogWrite(EN_C, abs(speed));
}

void stopAll() {
  motorA(0);
  motorB(0);
  motorC(0);
}

void printEncoders() {
  Serial.print("Enc A: "); Serial.print(encA);
  Serial.print("  Enc B: "); Serial.print(encB);
  Serial.print("  Enc C: "); Serial.println(encC);
}

void setup() {
  pinMode(EN_A, OUTPUT);
  pinMode(IN1,  OUTPUT);
  pinMode(IN2,  OUTPUT);
  pinMode(IN3,  OUTPUT);
  pinMode(IN4,  OUTPUT);
  pinMode(EN_B, OUTPUT);
  pinMode(EN_C,  OUTPUT);
  pinMode(IN3_C, OUTPUT);
  pinMode(IN4_C, OUTPUT);

  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);
  pinMode(ENC_C1, INPUT_PULLUP);
  pinMode(ENC_C2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), isrB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_C1), isrC, CHANGE);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Three motor test starting...");
  stopAll();
}

void loop() {
  // --- Test 1: All motors forward ---
  Serial.println("All motors FORWARD (half speed)");
  encA = encB = encC = 0;
  motorA(150); motorB(150); motorC(150);
  delay(2000);
  stopAll();
  printEncoders();
  delay(1000);

  // --- Test 2: All motors reverse ---
  Serial.println("All motors REVERSE (half speed)");
  encA = encB = encC = 0;
  motorA(-150); motorB(-150); motorC(-150);
  delay(2000);
  stopAll();
  printEncoders();
  delay(1000);

  // --- Test 3: Motor A only, ramp up ---
  Serial.println("Motor A ramp 0 -> 255");
  encA = 0;
  for (int s = 0; s <= 255; s += 5) {
    motorA(s);
    delay(30);
  }
  motorA(0);
  Serial.print("Motor A encoder count: "); Serial.println(encA);
  delay(1000);

  // --- Test 4: Motor B only, ramp up ---
  Serial.println("Motor B ramp 0 -> 255");
  encB = 0;
  for (int s = 0; s <= 255; s += 5) {
    motorB(s);
    delay(30);
  }
  motorB(0);
  Serial.print("Motor B encoder count: "); Serial.println(encB);
  delay(1000);

  // --- Test 5: Motor C only, ramp up ---
  Serial.println("Motor C ramp 0 -> 255");
  encC = 0;
  for (int s = 0; s <= 255; s += 5) {
    motorC(s);
    delay(30);
  }
  motorC(0);
  Serial.print("Motor C encoder count: "); Serial.println(encC);
  delay(2000);
}
