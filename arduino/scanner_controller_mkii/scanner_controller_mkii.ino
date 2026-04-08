/*
 * Scanner Controller MKII - Adafruit Motor Shield v2.3
 * 
 * Hardware Configuration:
 *   - Motor Shield v2.3 on Arduino Uno/Mega via I2C
 *   - Stepper 1 (M1+M2): Rotary table - rotation axis
 *   - Stepper 2 (M3+M4): Camera rail - linear focus axis
 * 
 * Serial Protocol (115200 baud):
 *   ROTATE <motor> <degrees> <CW|CCW>     - Rotate motor 1
 *   MOVE <motor> <millimeters> <FORWARD|BACKWARD> - Move motor 2
 *   ZERO <motor>                           - Reset position counter
 *   GET_POS <motor>                        - Query current position
 *   SET_SPEED <motor> <rpm>                - Set motor speed in RPM
 *   GET_SPEED <motor>                      - Query current speed
 *   RELEASE <motor>                        - Release motor (de-energize coils)
 *   RELEASE_ALL                            - Release all motors
 * 
 * Compatible with scanner_control.py serial protocol (subset)
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// ── Motor Shield Setup ───────────────────────────────────────────────────────
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Stepper 1: Rotary table (M1+M2 terminals)
// 200 steps/rev is typical for NEMA17, adjust if your motor differs
Adafruit_StepperMotor *rotaryStepper = AFMS.getStepper(200, 1);

// Stepper 2: Camera rail (M3+M4 terminals)  
Adafruit_StepperMotor *railStepper = AFMS.getStepper(200, 2);

// ── Stepping Style ───────────────────────────────────────────────────────────
// SINGLE    - One coil at a time (lower torque, lower power)
// DOUBLE    - Both coils energized (highest torque, higher power)
// INTERLEAVE - Half-stepping (good torque + smoother, 400 steps/rev)
// MICROSTEP  - Smoothest motion but lowest torque (3200 steps/rev)

// Use different styles per motor based on load requirements
const uint8_t STEP_STYLE_M1 = INTERLEAVE;  // Rotary table needs torque
const uint8_t STEP_STYLE_M2 = MICROSTEP;   // Rail is light, prioritize smoothness

// ── Calibration Constants ────────────────────────────────────────────────────
// Motor 1: Rotary table
// 200-step motor with INTERLEAVE = 400 steps/rev (half-stepping)
// Adjust multiplier if you have gearing/belt reduction
const float STEPS_PER_DEGREE_M1 = 400.0 / 360.0;  // ~1.111 steps/degree (no reduction)

// Motor 2: Camera rail linear motion
// 200-step motor with MICROSTEP = 3200 steps/rev
// Example: 8mm lead screw = 3200 steps / 8mm = 400 steps/mm
// Adjust based on your actual lead screw pitch!
const float STEPS_PER_MM_M2 = 400.0;  // For 8mm lead screw - CALIBRATE THIS

// ── Speed Settings (RPM) ─────────────────────────────────────────────────────
// Adafruit Motor Shield speed is set in RPM
// Note: Microstepping requires slower speeds to maintain torque
float speedM1_rpm = 5.0;    // Rotary table - slow for heavy load/inertia
float speedM2_rpm = 30.0;   // Camera rail - can be faster (lighter load)

// ── Direction Inversion ──────────────────────────────────────────────────────
const bool DIR_REVERSE_M1 = false;  // Set true if CW/CCW are backwards
const bool DIR_REVERSE_M2 = false;  // Set true if FORWARD/BACKWARD are backwards

// ── Position Counters ────────────────────────────────────────────────────────
float positionM1_deg = 0.0;  // degrees (rotation)
float positionM2_mm  = 0.0;  // millimeters (linear rail)

// ── High-level Rotate (Motor 1 - Rotary Table) ───────────────────────────────
void rotateMotor(int m, float deg, bool cw) {
  if (m != 1) return;
  
  // Apply direction reversal if configured
  bool actualCW = cw ^ DIR_REVERSE_M1;
  uint8_t dir = actualCW ? FORWARD : BACKWARD;
  
  // Calculate steps
  long steps = lround(fabs(deg) * STEPS_PER_DEGREE_M1);
  
  Serial.print("DEBUG: M1 stepping ");
  Serial.print(steps);
  Serial.print(" steps, dir=");
  Serial.println(dir == FORWARD ? "FWD" : "BWD");
  
  // Set speed and execute
  rotaryStepper->setSpeed(speedM1_rpm);
  rotaryStepper->step(steps, dir, STEP_STYLE_M1);
  
  // IMPORTANT: Release motor immediately to prevent overheating
  rotaryStepper->release();
  
  // Update position
  if (cw) {
    positionM1_deg += deg;
  } else {
    positionM1_deg -= deg;
  }
}

// ── High-level Move (Motor 2 - Camera Rail) ──────────────────────────────────
void moveMotor(int m, float mm, bool fwd) {
  if (m != 2) return;
  
  // Apply direction reversal if configured
  bool actualFwd = fwd ^ DIR_REVERSE_M2;
  uint8_t dir = actualFwd ? FORWARD : BACKWARD;
  
  // Calculate steps
  long steps = lround(fabs(mm) * STEPS_PER_MM_M2);
  
  Serial.print("DEBUG: M2 stepping ");
  Serial.print(steps);
  Serial.print(" steps, dir=");
  Serial.println(dir == FORWARD ? "FWD" : "BWD");
  
  // Set speed and execute
  railStepper->setSpeed(speedM2_rpm);
  railStepper->step(steps, dir, STEP_STYLE_M2);
  
  // IMPORTANT: Release motor immediately to prevent overheating
  railStepper->release();
  
  // Update position
  if (fwd) {
    positionM2_mm += mm;
  } else {
    positionM2_mm -= mm;
  }
}

// ── Release Motors (de-energize coils) ───────────────────────────────────────
void releaseMotor(int m) {
  if (m == 1) {
    rotaryStepper->release();
  } else if (m == 2) {
    railStepper->release();
  }
}

void releaseAll() {
  rotaryStepper->release();
  railStepper->release();
}

// ── Serial Command Parser ────────────────────────────────────────────────────
void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Split into up to 4 tokens
  String tok[4];
  int tc = 0, start = 0;
  for (int i = 0; i <= (int)line.length() && tc < 4; ++i) {
    if (i == (int)line.length() || line.charAt(i) == ' ') {
      if (i - start > 0) tok[tc++] = line.substring(start, i);
      start = i + 1;
    }
  }

  // ── ROTATE <motor> <degrees> <CW|CCW> ──
  if (tok[0] == "ROTATE" && tc == 4) {
    int motor = tok[1].toInt();
    float degrees = tok[2].toFloat();
    bool cw = (tok[3] == "CW");
    rotateMotor(motor, degrees, cw);
    Serial.println("OK");
  }
  // ── MOVE <motor> <mm> <FORWARD|BACKWARD> ──
  else if (tok[0] == "MOVE" && tc == 4) {
    int motor = tok[1].toInt();
    float mm = tok[2].toFloat();
    bool fwd = (tok[3] == "FORWARD");
    moveMotor(motor, mm, fwd);
    Serial.println("OK");
  }
  // ── TILT - Not implemented on MKII (2-axis only) ──
  else if (tok[0] == "TILT") {
    Serial.println("ERR: TILT not available on MKII (2-axis system)");
  }
  // ── ZERO <motor> ──
  else if (tok[0] == "ZERO" && tc >= 2) {
    int m = tok[1].toInt();
    if (m == 1) {
      positionM1_deg = 0.0;
      Serial.println("OK");
    } else if (m == 2) {
      positionM2_mm = 0.0;
      Serial.println("OK");
    } else {
      Serial.println("ERR: ZERO unsupported for motor " + String(m));
    }
  }
  // ── GET_POS <motor> ──
  else if (tok[0] == "GET_POS" && tc >= 2) {
    int m = tok[1].toInt();
    if (m == 1) {
      Serial.println(positionM1_deg);
    } else if (m == 2) {
      Serial.println(positionM2_mm);
    } else {
      Serial.println("ERR: GET_POS unsupported for motor " + String(m));
    }
  }
  // ── SET_SPEED <motor> <rpm> ──
  else if (tok[0] == "SET_SPEED" && tc >= 3) {
    int m = tok[1].toInt();
    float rpm = tok[2].toFloat();
    if (m == 1) {
      speedM1_rpm = rpm;
      Serial.println("OK SPEED M1=" + String(rpm));
    } else if (m == 2) {
      speedM2_rpm = rpm;
      Serial.println("OK SPEED M2=" + String(rpm));
    } else {
      Serial.println("ERR: SET_SPEED unsupported for motor " + String(m));
    }
  }
  // ── GET_SPEED <motor> ──
  else if (tok[0] == "GET_SPEED" && tc >= 2) {
    int m = tok[1].toInt();
    if (m == 1) {
      Serial.println(speedM1_rpm);
    } else if (m == 2) {
      Serial.println(speedM2_rpm);
    } else {
      Serial.println("ERR: GET_SPEED unsupported for motor " + String(m));
    }
  }
  // ── RELEASE <motor> ──
  else if (tok[0] == "RELEASE" && tc >= 2) {
    int m = tok[1].toInt();
    releaseMotor(m);
    Serial.println("OK RELEASED M" + String(m));
  }
  // ── RELEASE_ALL ──
  else if (tok[0] == "RELEASE_ALL") {
    releaseAll();
    Serial.println("OK RELEASED ALL");
  }
  // ── POWER commands - acknowledge but no relay on shield ──
  else if (tok[0] == "POWER") {
    // Motor shield doesn't have external power relay
    // Just acknowledge for compatibility with MKI protocol
    if (tc >= 2) {
      Serial.println("OK POWER " + tok[1] + " (no relay on MKII)");
    } else {
      Serial.println("ERR: POWER ON or POWER OFF");
    }
  }
  else if (tok[0] == "GET_POWER") {
    Serial.println("ON");  // Always "on" - shield is powered when Arduino is
  }
  // ── DEBUG/STATUS ──
  else if (tok[0] == "DEBUG_PINS" || tok[0] == "STATUS") {
    Serial.println("MKII Motor Shield v2.3");
    Serial.print("M1(rotary): pos=");
    Serial.print(positionM1_deg);
    Serial.print("deg, speed=");
    Serial.print(speedM1_rpm);
    Serial.println("rpm");
    Serial.print("M2(rail): pos=");
    Serial.print(positionM2_mm);
    Serial.print("mm, speed=");
    Serial.print(speedM2_rpm);
    Serial.println("rpm");
    Serial.print("M1 step style: ");
    Serial.println(STEP_STYLE_M1 == SINGLE ? "SINGLE" : 
                   STEP_STYLE_M1 == DOUBLE ? "DOUBLE" :
                   STEP_STYLE_M1 == INTERLEAVE ? "INTERLEAVE" : "MICROSTEP");
    Serial.print("M2 step style: ");
    Serial.println(STEP_STYLE_M2 == SINGLE ? "SINGLE" : 
                   STEP_STYLE_M2 == DOUBLE ? "DOUBLE" :
                   STEP_STYLE_M2 == INTERLEAVE ? "INTERLEAVE" : "MICROSTEP");
  }
  // ── TEST - single step each motor to verify wiring ──
  else if (tok[0] == "TEST") {
    Serial.println("Testing M1 (rotary) - 50 steps INTERLEAVE...");
    rotaryStepper->setSpeed(10);
    rotaryStepper->step(50, FORWARD, STEP_STYLE_M1);
    rotaryStepper->release();
    delay(500);
    Serial.println("Testing M2 (rail) - 100 steps MICROSTEP...");
    railStepper->setSpeed(10);
    railStepper->step(100, FORWARD, STEP_STYLE_M2);
    railStepper->release();
    Serial.println("TEST complete - motors released");
  }
  // ── WIRETEST - diagnose motor wiring by single-stepping slowly ──
  else if (tok[0] == "WIRETEST") {
    int motor = 1;
    if (tc >= 2) motor = tok[1].toInt();
    
    Adafruit_StepperMotor *stepper = (motor == 1) ? rotaryStepper : railStepper;
    
    Serial.print("Wire test M");
    Serial.println(motor);
    Serial.println("Watch for smooth rotation. If chaotic, swap one coil pair.");
    Serial.println("Stepping 20 times with 200ms pause...");
    
    stepper->setSpeed(1);  // Very slow
    for (int i = 0; i < 20; i++) {
      stepper->onestep(FORWARD, DOUBLE);  // Single step, DOUBLE for max torque
      delay(200);
      Serial.print(".");
    }
    Serial.println();
    stepper->release();
    Serial.println("WIRETEST complete");
  }
  // ── COILTEST - energize coils individually to diagnose wiring ──
  else if (tok[0] == "COILTEST") {
    int motor = 1;
    if (tc >= 2) motor = tok[1].toInt();
    
    Adafruit_StepperMotor *stepper = (motor == 1) ? rotaryStepper : railStepper;
    
    Serial.print("Coil test M");
    Serial.println(motor);
    Serial.println("Will energize coils in sequence A+, A-, B+, B-");
    Serial.println("Motor should move 4 distinct positions (90 deg apart)");
    Serial.println();
    
    // Step through each coil state manually using SINGLE mode
    // This shows if coils are correctly paired
    Serial.println("Position 1 (Coil A)...");
    stepper->onestep(FORWARD, SINGLE);
    delay(1000);
    
    Serial.println("Position 2 (Coil B)...");
    stepper->onestep(FORWARD, SINGLE);
    delay(1000);
    
    Serial.println("Position 3 (Coil A reverse)...");
    stepper->onestep(FORWARD, SINGLE);
    delay(1000);
    
    Serial.println("Position 4 (Coil B reverse)...");
    stepper->onestep(FORWARD, SINGLE);
    delay(1000);
    
    stepper->release();
    Serial.println("COILTEST complete");
    Serial.println("If motor moved 4 positions in same direction: wiring OK");
    Serial.println("If motor oscillated or skipped: coils mis-paired");
  }
  // ── Unknown command ──
  else {
    Serial.println("ERR: unknown or malformed command");
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  
  // Initialize motor shield
  if (!AFMS.begin()) {
    Serial.println("ERR: Motor Shield not found. Check wiring!");
    while (1);  // Halt if shield not detected
  }
  
  // Set initial speeds
  rotaryStepper->setSpeed(speedM1_rpm);
  railStepper->setSpeed(speedM2_rpm);
  
  // Release motors on startup (don't hold position until commanded)
  releaseAll();
  
  Serial.println("MKII Ready - Adafruit Motor Shield v2.3");
  Serial.println("Commands: ROTATE, MOVE, ZERO, GET_POS, SET_SPEED, GET_SPEED, RELEASE, RELEASE_ALL, STATUS");
}

// ── Main Loop ────────────────────────────────────────────────────────────────
void loop() {
  if (Serial.available()) {
    handleCommand(Serial.readStringUntil('\n'));
  }
}
