// Basic motor-control sketch
// Commands over Serial at 115200 baud:
//   ROTATE <motor> <degrees> <CW|CCW>
//   MOVE   <motor> <distance_mm> <FORWARD|BACKWARD>

#include <Arduino.h>

// --- Pin definitions ---
const int stepPin1   = 2;
const int dirPin1    = 3;
const int enablePin1 = 13;

const int stepPin2   = 4;
const int dirPin2    = 5;
const int enablePin2 = 12;

// --- Pulse timing (µs) ---
const unsigned int PULSE_DELAY_US = 500;

// --- Enable polarity (set to HIGH or LOW as needed) ---
const uint8_t ENABLE_ACTIVE = HIGH;

// --- Calibration constants (fill in your measured values) ---
// Motor 1:
const float STEPS_PER_DEGREE_M1 = 5.0;   // ← e.g. 200 steps/rev ÷ 360°
const float STEPS_PER_MM_M1     = 80.0;  // ← e.g. leadscrew pitch conversion

// Motor 2:
const float STEPS_PER_DEGREE_M2 = 5.0;
const float STEPS_PER_MM_M2     = 80.0;

// --- Helper to pulse a motor ---
void stepMotor(int stepPin, int dirPin, bool dirHigh, long count) {
  digitalWrite(dirPin, dirHigh ? HIGH : LOW);
  for (long i = 0; i < count; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(PULSE_DELAY_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(PULSE_DELAY_US);
  }
}

// --- High-level rotate by degrees ---
void rotateMotor(int motor, float deg, bool cw) {
  long steps = lround(deg * (motor == 1 
                             ? STEPS_PER_DEGREE_M1 
                             : STEPS_PER_DEGREE_M2));
  int sp = (motor == 1 ? stepPin1 : stepPin2);
  int dp = (motor == 1 ? dirPin1  : dirPin2);
  stepMotor(sp, dp, cw, steps);
}

// --- High-level move by millimeters ---
void moveMotor(int motor, float mm, bool forward) {
  long steps = lround(mm * (motor == 1 
                             ? STEPS_PER_MM_M1 
                             : STEPS_PER_MM_M2));
  int sp = (motor == 1 ? stepPin1 : stepPin2);
  int dp = (motor == 1 ? dirPin1  : dirPin2);
  stepMotor(sp, dp, forward, steps);
}

// --- Parse incoming Serial lines ---
void handleCommand(String line) {
  line.trim();
  if (line.startsWith("ROTATE")) {
    // ROTATE <motor> <degrees> <CW|CCW>
    int m = line.substring(7, 8).toInt();
    int p1 = line.indexOf(' ', 7);
    int p2 = line.indexOf(' ', p1 + 1);
    float deg = line.substring(p1 + 1, p2).toFloat();
    String dir = line.substring(p2 + 1);
    rotateMotor(m, deg, dir == "CW");
    Serial.println("OK");
  }
  else if (line.startsWith("MOVE")) {
    // MOVE <motor> <distance_mm> <FORWARD|BACKWARD>
    int m = line.substring(5, 6).toInt();
    int p1 = line.indexOf(' ', 5);
    int p2 = line.indexOf(' ', p1 + 1);
    float dist = line.substring(p1 + 1, p2).toFloat();
    String dir = line.substring(p2 + 1);
    moveMotor(m, dist, dir == "FORWARD");
    Serial.println("OK");
  }
  else {
    Serial.println("ERR: unknown command");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1,  OUTPUT);
  pinMode(enablePin1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2,  OUTPUT);
  pinMode(enablePin2, OUTPUT);

  // Enable drivers
  digitalWrite(enablePin1, ENABLE_ACTIVE);
  digitalWrite(enablePin2, ENABLE_ACTIVE);

  Serial.println("Ready for ROTATE/MOVE commands");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }
}
