#include <Arduino.h>

// ── Pin definitions ───────────────────────────────────────────────────────────
// Motor 1 (turntable):
const int STEP_PIN_1   = 2;
const int DIR_PIN_1    = 3;
const int ENABLE_PIN_1 = 13;

// Motor 2 (linear actuator):
const int STEP_PIN_2     = 4;
const int DIR_PIN_2      = 5;
const int ENABLE_PIN_2   = 12;
const int LIMIT_SWITCH_2 = 7;   // “Home” switch for Motor 2

// ── Timing & enable‐pin polarity ──────────────────────────────────────────────
const unsigned int PULSE_DELAY_US = 500;  // µs between step‐high/step‐low

// If your driver EN pin is active‐high, leave this HIGH. If EN is active‐low, set to LOW.
const bool ENABLE_ACTIVE_HIGH = HIGH;

// ── Calibration constants for Motor 1 (turntable) ──────────────────────────────
//   1/16 microstepping → 3200 µsteps per motor rev.
//   Gear ratio motor:turntable = 15 cm / 2 cm = 7.5:1 → 24 000 µsteps per turntable rev.
//   Therefore: 24 000 / 360 ≈ 66.667 µsteps per degree.
//
// Option A (theoretical):
// const float STEPS_PER_DEGREE_M1 = 3200.0f * (15.0f / 2.0f) / 360.0f; // ≈ 66.667
//
// Option B (rounded, “old” value):
const float STEPS_PER_DEGREE_M1 = 75.32f;  // slightly conservative calibration

// ── Calibration constants for Motor 2 (linear actuator) ────────────────────────
// 1/16 microstepping → 3200 µsteps per motor rev.
// Let LEAD_MM = your leadscrew pitch in mm/rev (e.g., 2 mm/rev).
const float LEAD_MM = 5.0f;                       // ← set this to your actual mm/rev
const float STEPS_PER_MM_M2 = 3200.0f / LEAD_MM;  // e.g. 3200/2 = 1600 µsteps/mm

// ── Direction‐reversal flags (flip if motor spins “backwards”) ─────────────────
const bool DIR_REVERSE_M1 = false;  // flip if “CW” is reversed physically
const bool DIR_REVERSE_M2 = false;  // flip if “FORWARD” is reversed physically

// ── Internal “position” counters ────────────────────────────────────────────────
float positionM1_deg = 0.0f;  // Motor 1 tracks degrees
float positionM2_mm  = 0.0f;  // Motor 2 tracks millimetres

// ── Enable / Disable driver pins ───────────────────────────────────────────────
void enableDriver(int motor_id) {
  if (motor_id == 1)
    digitalWrite(ENABLE_PIN_1, ENABLE_ACTIVE_HIGH);
  else
    digitalWrite(ENABLE_PIN_2, ENABLE_ACTIVE_HIGH);
}

void disableDriver(int motor_id) {
  if (motor_id == 1)
    digitalWrite(ENABLE_PIN_1, !ENABLE_ACTIVE_HIGH);
  else
    digitalWrite(ENABLE_PIN_2, !ENABLE_ACTIVE_HIGH);
}

// ── Step‐pulse generator (stops if limit switch hit on Motor 2 backward) ───────
void stepMotor(int stepPin, int dirPin, bool dirHigh, long steps, bool isMotor2) {
  // Set direction:
  digitalWrite(dirPin, dirHigh ? HIGH : LOW);

  for (long i = 0; i < steps; ++i) {
    // If stepping Motor 2 “backward” (toward home) and limit switch pressed, break:
    if (isMotor2 && !dirHigh && digitalRead(LIMIT_SWITCH_2) == LOW) {
      break;
    }

    // One micro‐pulse:
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(PULSE_DELAY_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(PULSE_DELAY_US);

    // Update software position counters:
    if (stepPin == STEP_PIN_1) {
      // Motor 1: each microstep = 1 / STEPS_PER_DEGREE_M1 degrees
      positionM1_deg += dirHigh
        ? (1.0f / STEPS_PER_DEGREE_M1)
        : -(1.0f / STEPS_PER_DEGREE_M1);
    }
    else {
      // Motor 2: each microstep = 1 / STEPS_PER_MM_M2 mm
      positionM2_mm += dirHigh
        ? (1.0f / STEPS_PER_MM_M2)
        : -(1.0f / STEPS_PER_MM_M2);
    }
  }
}

// ── High‐level “rotate” (Motor 1) ───────────────────────────────────────────────
void rotateMotor(int motor, float degrees, bool cw) {
  if (motor != 1) return;
  long steps = lround(degrees * STEPS_PER_DEGREE_M1);
  bool dirHigh = cw ^ DIR_REVERSE_M1;

  enableDriver(1);
  stepMotor(STEP_PIN_1, DIR_PIN_1, dirHigh, steps, false);
  disableDriver(1);
}

// ── High‐level “move” (Motor 2) ────────────────────────────────────────────────
void moveMotor(int motor, float mm, bool forward) {
  if (motor != 2) return;
  long steps = lround(mm * STEPS_PER_MM_M2);
  bool dirHigh = forward ^ DIR_REVERSE_M2;

  enableDriver(2);
  stepMotor(STEP_PIN_2, DIR_PIN_2, dirHigh, steps, true);
  disableDriver(2);
}

// ── Serial command parser & dispatcher ─────────────────────────────────────────
void handleCommand(const String &line) {
  String s = line;
  s.trim();
  if (s.length() == 0) return;

  // Tokenize into up to 4 parts:
  String tok[4];
  int tc = 0, start = 0;
  for (int i = 0; i <= s.length() && tc < 4; ++i) {
    if (i == s.length() || s.charAt(i) == ' ') {
      if (i - start > 0) tok[tc++] = s.substring(start, i);
      start = i + 1;
    }
  }

  // ROTATE <motor_id> <degrees> <CW|CCW>
  if (tok[0] == "ROTATE" && tc == 4) {
    int m     = tok[1].toInt();
    float deg = tok[2].toFloat();
    bool cw   = (tok[3] == "CW");
    rotateMotor(m, deg, cw);
    Serial.println("OK");
  }
  // MOVE <motor_id> <mm> <FORWARD|BACKWARD>
  else if (tok[0] == "MOVE" && tc == 4) {
    int m     = tok[1].toInt();
    float mmv = tok[2].toFloat();
    bool fwd  = (tok[3] == "FORWARD");
    moveMotor(m, mmv, fwd);
    Serial.println("OK");
  }
  // ZERO <motor_id>
  else if (tok[0] == "ZERO" && tc >= 2) {
    int m = tok[1].toInt();
    if (m == 1) {
      positionM1_deg = 0.0f;
      Serial.println("OK");
    } else if (m == 2) {
      positionM2_mm = 0.0f;
      Serial.println("OK");
    } else {
      Serial.println("ERR: ZERO unsupported for motor " + tok[1]);
    }
  }
  // GET_POS <motor_id>
  else if (tok[0] == "GET_POS" && tc >= 2) {
    int m = tok[1].toInt();
    if (m == 1) {
      Serial.println(positionM1_deg, 3);  // print degrees
    } else if (m == 2) {
      Serial.println(positionM2_mm, 3);   // print mm
    } else {
      Serial.println("ERR: GET_POS unsupported for motor " + tok[1]);
    }
  }
  // Unknown
  else {
    Serial.println("ERR: unknown or malformed command");
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);  // give the host time to open the port

  pinMode(STEP_PIN_1,   OUTPUT);
  pinMode(DIR_PIN_1,    OUTPUT);
  pinMode(ENABLE_PIN_1, OUTPUT);

  pinMode(STEP_PIN_2,     OUTPUT);
  pinMode(DIR_PIN_2,      OUTPUT);
  pinMode(ENABLE_PIN_2,   OUTPUT);
  pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);

  // Start with both disabled
  disableDriver(1);
  disableDriver(2);

  Serial.println("Ready for ROTATE, MOVE, ZERO, GET_POS");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }
}
