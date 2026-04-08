/*
 * Minimal Motor Test - Adafruit Motor Shield v2.3
 * Just tests motor wiring - no frills
 */

#include <Wire.h>
#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_StepperMotor *motor1 = AFMS.getStepper(200, 1);  // M1+M2
Adafruit_StepperMotor *motor2 = AFMS.getStepper(200, 2);  // M3+M4

void setup() {
  Serial.begin(115200);
  
  if (!AFMS.begin()) {
    Serial.println(F("Shield not found"));
    while (1);
  }
  
  motor1->release();
  motor2->release();
  
  Serial.println(F("Ready. Commands: 1 2 C1 C2"));
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    // Test motor 1 (rotary)
    if (cmd == "1") {
      Serial.println(F("M1: 20 steps DOUBLE"));
      motor1->setSpeed(5);
      for (int i = 0; i < 20; i++) {
        motor1->onestep(FORWARD, DOUBLE);
        delay(100);
      }
      motor1->release();
      Serial.println(F("Done"));
    }
    // Test motor 2 (rail)
    else if (cmd == "2") {
      Serial.println(F("M2: 20 steps DOUBLE"));
      motor2->setSpeed(5);
      for (int i = 0; i < 20; i++) {
        motor2->onestep(FORWARD, DOUBLE);
        delay(100);
      }
      motor2->release();
      Serial.println(F("Done"));
    }
    // Coil test motor 1
    else if (cmd == "C1") {
      Serial.println(F("M1 coil test - 4 positions"));
      for (int i = 0; i < 4; i++) {
        motor1->onestep(FORWARD, SINGLE);
        delay(1000);
        Serial.println(i + 1);
      }
      motor1->release();
    }
    // Coil test motor 2
    else if (cmd == "C2") {
      Serial.println(F("M2 coil test - 4 positions"));
      for (int i = 0; i < 4; i++) {
        motor2->onestep(FORWARD, SINGLE);
        delay(1000);
        Serial.println(i + 1);
      }
      motor2->release();
    }
    else {
      Serial.println(F("? 1 2 C1 C2"));
    }
  }
}
