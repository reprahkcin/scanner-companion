// Auto-generated from profile: raspberry_pi_3axis
// Generated: 2026-01-02 11:14:55
// DO NOT EDIT - regenerate from profile using generate_arduino_config.py

#pragma once

// ── Pin Definitions ─────────────────────────────────────────────────────────
const int STEP_PIN_1     = 2;
const int DIR_PIN_1      = 3;
const int ENABLE_PIN_1   = 13;

const int STEP_PIN_2     = 4;
const int DIR_PIN_2      = 5;
const int ENABLE_PIN_2   = 12;
const int LIMIT_SWITCH_2 = 7;

const int STEP_PIN_3     = 9;
const int DIR_PIN_3      = 10;
const int ENABLE_PIN_3   = 11;
const int LIMIT_SWITCH_3 = 8;

const int POWER_RELAY_PIN = A0;

// ── Timing & Driver Settings ────────────────────────────────────────────────
const unsigned int PULSE_DELAY_M1_US = 800;
const unsigned int PULSE_DELAY_M2_US = 500;
const unsigned int PULSE_DELAY_M3_US = 1500;

const bool ENABLE_ACTIVE_HIGH = true;
const bool RELAY_ACTIVE_LOW = true;

// ── Calibration Constants ───────────────────────────────────────────────────
const float STEPS_PER_DEGREE_M1 = 17.7778;
const float STEPS_PER_MM_M2 = 1281.21;
const float STEPS_PER_DEGREE_M3 = 88.8889;

// ── Direction Inversion ─────────────────────────────────────────────────────
const bool DIR_REVERSE_M1 = false;
const bool DIR_REVERSE_M2 = false;
const bool DIR_REVERSE_M3 = false;