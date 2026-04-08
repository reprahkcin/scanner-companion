// Auto-generated from profile: mkii_shield
// Generated: 2026-01-02 14:03:00
// DO NOT EDIT - regenerate from profile using generate_arduino_config.py

#pragma once

#include <Wire.h>
#include <Adafruit_MotorShield.h>

// ── Motor Shield Configuration ─────────────────────────────────────────────
// Motor 1: M1+M2 - Rotary table - INTERLEAVE for torque (400 half-steps/rev)
const int MOTOR_1_PORT = 1;
const int MOTOR_1_STEPS_PER_REV = 200;
const uint8_t STEP_STYLE_M1 = INTERLEAVE;

// Motor 2: M3+M4 - Camera rail - MICROSTEP for smoothness (3200 microsteps/rev)
const int MOTOR_2_PORT = 2;
const int MOTOR_2_STEPS_PER_REV = 200;
const uint8_t STEP_STYLE_M2 = MICROSTEP;

// ── Calibration Constants ───────────────────────────────────────────────────
const float STEPS_PER_DEGREE_M1 = 1.1110;
const float STEPS_PER_MM_M2 = 400.00;

// ── Speed Settings (RPM) ────────────────────────────────────────────────────
const float SPEED_M1_RPM = 5.0;
const float SPEED_M2_RPM = 30.0;

// ── Direction Inversion ─────────────────────────────────────────────────────
const bool DIR_REVERSE_M1 = false;
const bool DIR_REVERSE_M2 = false;