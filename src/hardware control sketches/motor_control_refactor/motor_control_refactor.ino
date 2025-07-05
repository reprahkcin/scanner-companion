#include <Arduino.h>

// ── Pin definitions ───────────────────────────────────────────────────────────
const int STEP_PIN_1     = 2;
const int DIR_PIN_1      = 3;
const int ENABLE_PIN_1   = 13;

const int STEP_PIN_2     = 4;
const int DIR_PIN_2      = 5;
const int ENABLE_PIN_2   = 12;
const int LIMIT_SWITCH_2 = 7;   // back/home limit for Motor 2

// ── Timing & driver enable polarity ──────────────────────────────────────────
const unsigned int PULSE_DELAY_US   = 500;
const bool      ENABLE_ACTIVE_HIGH  = HIGH;  // or LOW if your EN is active‐low

// ── Calibration constants ────────────────────────────────────────────────────
// Motor 1: we found that “ROTATE 1 90 CW” gives ≈2° on your table, so:
const float STEPS_PER_DEGREE_M1 = 75.0;

// Motor 2: keep at 13 steps/mm so “MOVE 2 50 FORWARD” → ≈1 mm
const float STEPS_PER_MM_M2     = 13.0;

// ── Direction inversion flags ────────────────────────────────────────────────
const bool DIR_REVERSE_M1 = false;
const bool DIR_REVERSE_M2 = false;  // now FORWARD/ BACKWARD map correctly

// ── Position counters ────────────────────────────────────────────────────────
float positionM1_deg = 0.0;  // degrees
float positionM2_mm  = 0.0;  // millimetres

// ── Driver control ────────────────────────────────────────────────────────────
void enableDriver(int m) {
  digitalWrite(m==1?ENABLE_PIN_1:ENABLE_PIN_2,
               ENABLE_ACTIVE_HIGH);
}
void disableDriver(int m) {
  digitalWrite(m==1?ENABLE_PIN_1:ENABLE_PIN_2,
               !ENABLE_ACTIVE_HIGH);
}

// ── Atomized stepping with limit‐switch check (Motor 2) ───────────────────────
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
  else if (tok[0]=="ZERO" && tc>=2) {
    int m=tok[1].toInt();
    if (m==1) { positionM1_deg=0.0; Serial.println("OK"); }
    else if (m==2) { positionM2_mm=0.0;  Serial.println("OK"); }
    else Serial.println("ERR: ZERO unsupported for motor "+String(m));
  }
  else if (tok[0]=="GET_POS" && tc>=2) {
    int m=tok[1].toInt();
    if (m==1) Serial.println(positionM1_deg);
    else if (m==2) Serial.println(positionM2_mm);
    else Serial.println("ERR: GET_POS unsupported for motor "+String(m));
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
  pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);

  // start with both drivers disabled
  disableDriver(1);
  disableDriver(2);

  Serial.println("Ready for ROTATE, MOVE, ZERO, GET_POS");
}

void loop() {
  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }

  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("SPEED")) {
      // format: SPEED <motor_id> <value>
      int motor = line.charAt(6) - '0';        // crude but works for single-digit
      int val   = line.substring(8).toInt();
      setSpeed(motor, val);
    }
    else if (line.startsWith("JOG")) {
      // format: JOG <motor_id> <DIR> <value>
      // e.g. "JOG 2 BACKWARD 50"
      char buf[line.length()+1];
      line.toCharArray(buf, sizeof(buf));
      int motor, speed;
      char dir[10];
      sscanf(buf, "JOG %d %s %d", &motor, dir, &speed);
      bool forward = (strcmp(dir, "FORWARD") == 0);
      jogMotor(motor, forward, speed);
    }
  }
  
}

// Example stubs — replace with your real functions:
void setSpeed(int motorId, int speed) {
  // e.g., driver[motorId].setMaxSpeed(speed);
}

void jogMotor(int motorId, bool forward, int speed) {
  // e.g., driver[motorId].setSpeed( forward ? speed : -speed );
  //       driver[motorId].runSpeedFor(steps);
}
