const int pulPin1 = 2; // Pulse for motor 1
const int dirPin1 = 3; // Direction for motor 1
const int enablePin1 = 13; // Enable for motor 1

const int pulPin2 = 4; // Pulse for motor 2
const int dirPin2 = 5; // Direction for motor 2
const int enablePin2 = 12; // Enable for motor 2

const int limitSwitchFront = 6; // Front limit switch (camera front)
const int limitSwitchRear = 7;  // Rear limit switch (camera back, used for homing)

long position = 0; // Position in steps for motor 2

const int normalDelay = 800; // microseconds
const int fastDelay = 300;   // microseconds, used during homing

void setup() {
  Serial.begin(9600);
  pinMode(pulPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enablePin1, OUTPUT);

  pinMode(pulPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enablePin2, OUTPUT);

  pinMode(limitSwitchFront, INPUT_PULLUP);
  pinMode(limitSwitchRear, INPUT_PULLUP);

  disableMotors();
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "HOME") {
      enableMotors();
      homeMotor2();
      disableMotors();
      Serial.println("HOMED");
    } else if (command == "GET_POS") {
      Serial.println(position);
    } else if (command.length() >= 3) {
      char motor = command.charAt(0);
      char direction = command.charAt(command.length() - 1);
      int steps = command.substring(1, command.length() - 1).toInt();

      if ((motor == '1' || motor == '2') && (direction == 'F' || direction == 'B')) {
        enableMotors();
        moveStepper(motor, steps, direction, normalDelay);
        disableMotors();
      }
    }
  }
}

void moveStepper(char motor, int steps, char direction, int delayMicros) {
  int dirPin = (motor == '1') ? dirPin1 : dirPin2;
  int pulPin = (motor == '1') ? pulPin1 : pulPin2;

  digitalWrite(dirPin, (direction == 'F') ? HIGH : LOW);

  for (int i = 0; i < steps; i++) {
    if (motor == '2') {
      if ((direction == 'F' && digitalRead(limitSwitchFront) == LOW) ||
          (direction == 'B' && digitalRead(limitSwitchRear) == LOW)) {
        Serial.println("Motion stopped due to limit switch");
        break;
      }
    }

    digitalWrite(pulPin, HIGH);
    delayMicroseconds(delayMicros);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(delayMicros);

    if (motor == '2') {
      position += (direction == 'F') ? 1 : -1;
    }
  }
}

void homeMotor2() {
  digitalWrite(dirPin2, LOW); // move backward
  while (digitalRead(limitSwitchRear) == HIGH) {
    digitalWrite(pulPin2, HIGH);
    delayMicroseconds(fastDelay);
    digitalWrite(pulPin2, LOW);
    delayMicroseconds(fastDelay);
  }
  position = 0;
}

void enableMotors() {
  digitalWrite(enablePin1, HIGH);
  digitalWrite(enablePin2, HIGH);
}

void disableMotors() {
  digitalWrite(enablePin1, LOW);
  digitalWrite(enablePin2, LOW);
}
