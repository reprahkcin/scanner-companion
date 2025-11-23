# TB6600 Stepper Driver Enable Logic - CRITICAL REFERENCE

## Hardware Configuration
- **ENA+ connected to:** Arduino 5V
- **ENA- connected to:** Arduino GPIO pin (11, 12, or 13)

## Enable/Disable Logic (VERIFIED WORKING)

```cpp
// Motor 1 - Pin 13
// Motor 2 - Pin 12  
// Motor 3 - Pin 11

// TO DISABLE MOTOR (quiet, no holding torque, no heat):
digitalWrite(ENABLE_PIN, LOW);

// TO ENABLE MOTOR (active, holding torque, can step):
digitalWrite(ENABLE_PIN, HIGH);
```

## Why This Works

With ENA+ tied to 5V and ENA- controlled by Arduino:
- When Arduino pin is **LOW (0V)**: ENA+ (5V) - ENA- (0V) = **5V differential** → But TB6600 reads this as **DISABLED**
- When Arduino pin is **HIGH (5V)**: ENA+ (5V) - ENA- (5V) = **0V differential** → But TB6600 reads this as **ENABLED**

**This is OPPOSITE of typical A4988/DRV8825 drivers!**

## Arduino Code Pattern

```cpp
const bool ENABLE_ACTIVE_HIGH = true;  // For TB6600 with ENA+ to 5V

void enableDriver(int motor) {
  digitalWrite(enablePin, HIGH);  // Enable = HIGH
}

void disableDriver(int motor) {
  digitalWrite(enablePin, LOW);   // Disable = LOW
}

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  
  // Start with motor DISABLED
  digitalWrite(ENABLE_PIN, LOW);  // Critical: LOW = disabled
}
```

## Symptoms of Wrong Configuration

❌ **If enable logic is inverted:**
- Motors squeal/whine constantly
- Motors get hot even when idle
- Random movement/jittering
- Motors respond to other motors' commands

✅ **Correct behavior:**
- Silence on startup
- Motors only hold when explicitly enabled
- Clean, predictable movement
- Motors cool when disabled

## Date Verified
November 22, 2025

## Hardware Tested
- Arduino Uno/Mega
- TB6600 stepper drivers (3x)
- All motors: 200 steps/rev, DIP switches set to microstep=1

**DO NOT CHANGE THIS CONFIGURATION WITHOUT TESTING!**
