#include <Arduino.h>

// ── Pin definitions ───────────────────────────────────────────────────────────
const int STEP_PIN_1     = 2;
const int DIR_PIN_1      = 3;
const int ENABLE_PIN_1   = 13;

const int STEP_PIN_2     = 4;
const int DIR_PIN_2      = 5;
const int ENABLE_PIN_2   = 12;
const int LIMIT_SWITCH_2 = 7;   // back/home limit for Motor 2

const int STEP_PIN_3     = 9;   // Motor 3: vertical tilt axis
const int DIR_PIN_3      = 10;
const int ENABLE_PIN_3   = 11;
const int LIMIT_SWITCH_3 = 8;   // future: bottom limit for Motor 3 (reserved)

// ── Timing & driver enable polarity ──────────────────────────────────────────
const unsigned int PULSE_DELAY_US   = 500;
// TB6600 ENABLE LOGIC (ENA+ to 5V, ENA- to pin): 
// HIGH = ENABLED (motor holds), LOW = DISABLED (quiet)
// See TB6600_ENABLE_LOGIC.md for full explanation
const bool      ENABLE_ACTIVE_HIGH  = true;

// Motor 3 hold settings (prevent overheating on counterbalanced tilt axis)
const unsigned long MOTOR3_SETTLE_TIME_MS = 1000;  // Hold time after movement before disabling

// ── Calibration constants ────────────────────────────────────────────────────
// All motors: 6400 pulses/revolution (32× microstepping)

const float STEPS_PER_DEGREE_M1 = 17.7778;  // 6400 pulses/rev ÷ 360° (32x microstepping)
const float STEPS_PER_DEGREE_M3 = 17.7778;  // 6400 pulses/rev ÷ 360° (32x microstepping)

// Motor 2: Lead screw with 32x microstepping = 6400 steps per screw revolution
// CALIBRATED: 100mm command = 96.36mm actual → 1234.57 * (100/96.36) = 1281.21 steps/mm
const float STEPS_PER_MM_M2 = 1281.21;  // Calibrated from actual measurement

// ── Direction inversion flags ────────────────────────────────────────────────
const bool DIR_REVERSE_M1 = false;
const bool DIR_REVERSE_M2 = false;  // now FORWARD/ BACKWARD map correctly
const bool DIR_REVERSE_M3 = false;  // adjust if tilt direction is inverted

// ── Position counters ────────────────────────────────────────────────────────
float positionM1_deg = 0.0;  // degrees
float positionM2_mm  = 0.0;  // millimetres
float positionM3_deg = 0.0;  // degrees (tilt)

// ── Driver control ────────────────────────────────────────────────────────────
void enableDriver(int m) {
  if (m == 1)
    digitalWrite(ENABLE_PIN_1, ENABLE_ACTIVE_HIGH);
  else if (m == 2)
    digitalWrite(ENABLE_PIN_2, ENABLE_ACTIVE_HIGH);
  else if (m == 3)
    digitalWrite(ENABLE_PIN_3, ENABLE_ACTIVE_HIGH);
}

void disableDriver(int m) {
  if (m == 1)
    digitalWrite(ENABLE_PIN_1, !ENABLE_ACTIVE_HIGH);
  else if (m == 2)
    digitalWrite(ENABLE_PIN_2, !ENABLE_ACTIVE_HIGH);
  else if (m == 3)
    digitalWrite(ENABLE_PIN_3, !ENABLE_ACTIVE_HIGH);
}

// ── Atomized stepping with limit‐switch check ───────────────────────────────
void stepMotor(int stepPin, int dirPin, bool dirHigh, long steps) {
  digitalWrite(dirPin, dirHigh ? HIGH : LOW);
  for (long i = 0; i < steps; ++i) {
    // if Motor 2 stepping backward toward switch, abort on trip
    if (stepPin==STEP_PIN_2 && !dirHigh 
        && digitalRead(LIMIT_SWITCH_2)==LOW) {
      break;
    }
    // pulse
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(PULSE_DELAY_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(PULSE_DELAY_US);
    // update positions
    if (stepPin==STEP_PIN_1) {
      positionM1_deg += dirHigh
        ? (1.0f / STEPS_PER_DEGREE_M1)
        : -(1.0f / STEPS_PER_DEGREE_M1);
    }
    else if (stepPin==STEP_PIN_2) {
      positionM2_mm  += dirHigh
        ? (1.0f / STEPS_PER_MM_M2)
        : -(1.0f / STEPS_PER_MM_M2);
    }
    else if (stepPin==STEP_PIN_3) {
      positionM3_deg += dirHigh
        ? (1.0f / STEPS_PER_DEGREE_M3)
        : -(1.0f / STEPS_PER_DEGREE_M3);
    }
  }
}

// ── High‐level rotate (Motor 1) ───────────────────────────────────────────────
void rotateMotor(int m, float deg, bool cw) {
  if (m != 1) return;
  long steps = lround(deg * STEPS_PER_DEGREE_M1);
  bool dirHigh = cw ^ DIR_REVERSE_M1;
  enableDriver(1);
  stepMotor(STEP_PIN_1, DIR_PIN_1, dirHigh, steps);
  disableDriver(1);
}

// ── High‐level move (Motor 2) ────────────────────────────────────────────────
void moveMotor(int m, float mm, bool fwd) {
  if (m != 2) return;
  long steps = lround(mm * STEPS_PER_MM_M2);
  bool dirHigh = fwd ^ DIR_REVERSE_M2;
  enableDriver(2);
  stepMotor(STEP_PIN_2, DIR_PIN_2, dirHigh, steps);
  disableDriver(2);
}

// ── High‐level tilt (Motor 3) ─────────────────────────────────────────────────
void tiltMotor(int m, float deg, bool up) {
  if (m != 3) return;
  long steps = lround(deg * STEPS_PER_DEGREE_M3);
  bool dirHigh = up ^ DIR_REVERSE_M3;
  enableDriver(3);
  stepMotor(STEP_PIN_3, DIR_PIN_3, dirHigh, steps);
  // Allow motor to settle into position before disabling (prevents rebound)
  delay(MOTOR3_SETTLE_TIME_MS);
  disableDriver(3);
}

// ── Serial parser & dispatcher ───────────────────────────────────────────────
void handleCommand(String line) {
  line.trim();
  if (line.length()==0) return;

  // split up to 4 tokens
  String tok[4];
  int tc=0, start=0;
  for (int i=0; i<=line.length() && tc<4; ++i) {
    if (i==line.length()||line.charAt(i)==' ') {
      if (i-start>0) tok[tc++]=line.substring(start,i);
      start=i+1;
    }
  }

  if (tok[0]=="ROTATE" && tc==4) {
    rotateMotor(tok[1].toInt(), tok[2].toFloat(), tok[3]=="CW");
    Serial.println("OK");
  }
  else if (tok[0]=="MOVE" && tc==4) {
    moveMotor(tok[1].toInt(), tok[2].toFloat(), tok[3]=="FORWARD");
    Serial.println("OK");
  }
  else if (tok[0]=="TILT" && tc==4) {
    tiltMotor(tok[1].toInt(), tok[2].toFloat(), tok[3]=="UP");
    Serial.println("OK");
  }
  else if (tok[0]=="ZERO" && tc>=2) {
    int m=tok[1].toInt();
    if (m==1) { positionM1_deg=0.0; Serial.println("OK"); }
    else if (m==2) { positionM2_mm=0.0;  Serial.println("OK"); }
    else if (m==3) { positionM3_deg=0.0;  Serial.println("OK"); }
    else Serial.println("ERR: ZERO unsupported for motor "+String(m));
  }
  else if (tok[0]=="GET_POS" && tc>=2) {
    int m=tok[1].toInt();
    if (m==1) Serial.println(positionM1_deg);
    else if (m==2) Serial.println(positionM2_mm);
    else if (m==3) Serial.println(positionM3_deg);
    else Serial.println("ERR: GET_POS unsupported for motor "+String(m));
  }
  else if (tok[0] == "DEBUG_PINS") {
    // Diagnostic command to check enable pin states
    Serial.print("EN1(13)=");
    Serial.print(digitalRead(ENABLE_PIN_1));
    Serial.print(" EN2(12)=");
    Serial.print(digitalRead(ENABLE_PIN_2));
    Serial.print(" EN3(11)=");
    Serial.println(digitalRead(ENABLE_PIN_3));
  }
  else {
    Serial.println("ERR: unknown or malformed command");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(STEP_PIN_1,   OUTPUT);
  pinMode(DIR_PIN_1,    OUTPUT);
  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2,   OUTPUT);
  pinMode(DIR_PIN_2,    OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);
  pinMode(STEP_PIN_3,   OUTPUT);
  pinMode(DIR_PIN_3,    OUTPUT);
  pinMode(ENABLE_PIN_3, OUTPUT);
  pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);
  // pinMode(LIMIT_SWITCH_3, INPUT_PULLUP);  // uncomment when limit switch is installed

  // start with all drivers disabled
  disableDriver(1);
  disableDriver(2);
  disableDriver(3);

  Serial.println("Ready for ROTATE, MOVE, TILT, ZERO, GET_POS, DEBUG_PINS");
}

void loop() {
  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }
}
