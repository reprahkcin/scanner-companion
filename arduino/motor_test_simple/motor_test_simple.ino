/*
 * Simple TB6600 Motor Test Sketch
 * Tests each motor independently with small movements
 * All motors: 200 steps/rev (full step), no microstepping
 */

// Motor 1 - Z-rotation (turntable)
const int STEP_PIN_1 = 2;
const int DIR_PIN_1 = 3;
const int ENABLE_PIN_1 = 13;

// Motor 2 - Camera linear rail
const int STEP_PIN_2 = 4;
const int DIR_PIN_2 = 5;
const int ENABLE_PIN_2 = 12;

// Motor 3 - Tilt
const int STEP_PIN_3 = 9;
const int DIR_PIN_3 = 10;
const int ENABLE_PIN_3 = 11;

// TB6600 enable logic: INVERTED from typical drivers
// To ENABLE: set pin HIGH
// To DISABLE: set pin LOW

const int STEP_DELAY_US = 1000;  // 1ms between steps = slow and visible

void setup() {
  Serial.begin(115200);
  
  // Configure all pins as outputs
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_1, OUTPUT);
  
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);
  
  pinMode(STEP_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
  pinMode(ENABLE_PIN_3, OUTPUT);
  
  // CRITICAL: Set all enable pins LOW to DISABLE all motors
  digitalWrite(ENABLE_PIN_1, LOW);
  digitalWrite(ENABLE_PIN_2, LOW);
  digitalWrite(ENABLE_PIN_3, LOW);
  
  // Set all step pins LOW initially
  digitalWrite(STEP_PIN_1, LOW);
  digitalWrite(STEP_PIN_2, LOW);
  digitalWrite(STEP_PIN_3, LOW);
  
  Serial.println("=== TB6600 Motor Test ===");
  Serial.println("All motors should be DISABLED (not holding/hot)");
  Serial.println("Waiting 3 seconds...");
  delay(3000);
  
  Serial.println("\nStarting motor tests in 2 seconds...");
  delay(2000);
}

void loop() {
  // Test Motor 1 - Turntable
  Serial.println("\n--- Testing Motor 1 (Turntable) ---");
  testMotor(1, STEP_PIN_1, DIR_PIN_1, ENABLE_PIN_1, 10, "CW");
  delay(1000);
  testMotor(1, STEP_PIN_1, DIR_PIN_1, ENABLE_PIN_1, 10, "CCW");
  delay(2000);
  
  // Test Motor 2 - Linear Rail
  Serial.println("\n--- Testing Motor 2 (Linear Rail) ---");
  testMotor(2, STEP_PIN_2, DIR_PIN_2, ENABLE_PIN_2, 10, "Forward");
  delay(1000);
  testMotor(2, STEP_PIN_2, DIR_PIN_2, ENABLE_PIN_2, 10, "Backward");
  delay(2000);
  
  // Test Motor 3 - Tilt
  Serial.println("\n--- Testing Motor 3 (Tilt) ---");
  testMotor(3, STEP_PIN_3, DIR_PIN_3, ENABLE_PIN_3, 10, "Up");
  delay(1000);
  testMotor(3, STEP_PIN_3, DIR_PIN_3, ENABLE_PIN_3, 10, "Down");
  delay(2000);
  
  Serial.println("\n=== All tests complete. Repeating in 5 seconds ===\n");
  delay(5000);
}

void testMotor(int motorNum, int stepPin, int dirPin, int enablePin, int steps, const char* direction) {
  Serial.print("Motor ");
  Serial.print(motorNum);
  Serial.print(": Moving ");
  Serial.print(steps);
  Serial.print(" steps ");
  Serial.println(direction);
  
  // Set direction (LOW or HIGH - adjust if movement is wrong direction)
  bool dirHigh = (strcmp(direction, "CW") == 0 || strcmp(direction, "Forward") == 0 || strcmp(direction, "Up") == 0);
  digitalWrite(dirPin, dirHigh ? HIGH : LOW);
  
  // Enable motor (set pin HIGH for TB6600)
  digitalWrite(enablePin, HIGH);
  Serial.println("  Motor ENABLED");
  delay(50);  // Let motor energize
  
  // Step the motor
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(STEP_DELAY_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(STEP_DELAY_US);
  }
  
  Serial.println("  Steps complete");
  
  // Disable motor (set pin LOW for TB6600)
  digitalWrite(enablePin, LOW);
  Serial.println("  Motor DISABLED");
}
