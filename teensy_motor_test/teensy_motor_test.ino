// Teensy 4.1 - Dual Motor Driver (L298N-style) Test Sketch
// Wiring (as connected):
//   Pin 8  -> EN  (Motor A enable / PWM speed)
//   Pin 7  -> IN1 (Motor A direction)
//   Pin 6  -> IN2 (Motor A direction)
//   Pin 4  -> IN3 (Motor B direction)
//   Pin 3  -> IN4 (Motor B direction)
//   Pin 2  -> ENB (Motor B enable / PWM speed)
//   G      -> Driver GND tied to Teensy GND (common ground)
//   5V     -> Driver logic 5V supply

const int EN_A = 8;
const int IN1  = 7;
const int IN2  = 6;
const int IN3  = 4;
const int IN4  = 3;
const int EN_B = 2;

void motorA(int speed) {           // speed: -255 (full reverse) to 255 (full fwd), 0 = stop
  digitalWrite(IN1, speed > 0 ? HIGH : LOW);
  digitalWrite(IN2, speed < 0 ? HIGH : LOW);
  analogWrite(EN_A, abs(speed));
}

void motorB(int speed) {
  digitalWrite(IN3, speed > 0 ? HIGH : LOW);
  digitalWrite(IN4, speed < 0 ? HIGH : LOW);
  analogWrite(EN_B, abs(speed));
}

void stopAll() {
  motorA(0);
  motorB(0);
}

void setup() {
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN_B, OUTPUT);

  Serial.begin(115200);
  delay(1000);
  Serial.println("Motor test starting...");
  stopAll();
}

void loop() {
  Serial.println("Both motors FORWARD (half speed)");
  motorA(150);
  motorB(150);
  delay(2000);

  Serial.println("Stop");
  stopAll();
  delay(1000);

  Serial.println("Both motors REVERSE (half speed)");
  motorA(-150);
  motorB(-150);
  delay(2000);

  Serial.println("Stop");
  stopAll();
  delay(2000);

  Serial.println("Motor A only, ramp speed 0->255");
  for (int s = 0; s <= 255; s += 5) {
    motorA(s);
    delay(30);
  }
  stopAll();
  delay(2000);
}
