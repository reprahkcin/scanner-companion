#!/usr/bin/env python3
"""
Generate Arduino configuration header from hardware profile.

This script reads a hardware profile JSON and generates a C++ header file
with pin definitions, calibration constants, and driver settings.

Usage:
    python3 generate_arduino_config.py raspberry_pi_3axis
    python3 generate_arduino_config.py mkii_shield
"""

import json
import os
import sys
from datetime import datetime

def load_profile(profile_name: str) -> dict:
    """Load a hardware profile by name."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    profile_path = os.path.join(script_dir, "profiles", f"{profile_name}.json")
    
    if not os.path.exists(profile_path):
        raise FileNotFoundError(f"Profile not found: {profile_path}")
    
    with open(profile_path, 'r') as f:
        return json.load(f)


def generate_tb6600_header(profile: dict) -> str:
    """Generate header for TB6600-based setup (MKI style)."""
    hw = profile.get("arduino_hardware", {})
    motors = hw.get("motors", {})
    driver_settings = hw.get("driver_settings", {})
    relay = hw.get("power_relay", {})
    axes = {a["name"]: a for a in profile.get("axes", [])}
    
    lines = [
        "// Auto-generated from profile: " + profile.get("name", "unknown"),
        "// Generated: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "// DO NOT EDIT - regenerate from profile using generate_arduino_config.py",
        "",
        "#pragma once",
        "",
        "// ── Pin Definitions ─────────────────────────────────────────────────────────",
    ]
    
    for motor_name in sorted(motors.keys()):
        m = motors[motor_name]
        num = motor_name.replace("M", "")
        lines.append(f"const int STEP_PIN_{num}     = {m.get('step_pin', 0)};")
        lines.append(f"const int DIR_PIN_{num}      = {m.get('dir_pin', 0)};")
        lines.append(f"const int ENABLE_PIN_{num}   = {m.get('enable_pin', 0)};")
        if m.get("limit_switch_pin"):
            lines.append(f"const int LIMIT_SWITCH_{num} = {m.get('limit_switch_pin')};")
        lines.append("")
    
    # Power relay
    relay_pin = relay.get("pin", "A0")
    lines.append(f"const int POWER_RELAY_PIN = {relay_pin};")
    lines.append("")
    
    # Timing
    lines.append("// ── Timing & Driver Settings ────────────────────────────────────────────────")
    for motor_name in sorted(motors.keys()):
        m = motors[motor_name]
        num = motor_name.replace("M", "")
        lines.append(f"const unsigned int PULSE_DELAY_M{num}_US = {m.get('pulse_delay_us', 500)};")
    lines.append("")
    
    # Enable polarity
    enable_high = driver_settings.get("enable_active_high", True)
    lines.append(f"const bool ENABLE_ACTIVE_HIGH = {'true' if enable_high else 'false'};")
    
    # Relay polarity
    relay_low = relay.get("active_low", True)
    lines.append(f"const bool RELAY_ACTIVE_LOW = {'true' if relay_low else 'false'};")
    lines.append("")
    
    # Calibration constants
    lines.append("// ── Calibration Constants ───────────────────────────────────────────────────")
    for motor_name in sorted(motors.keys()):
        m = motors[motor_name]
        axis = axes.get(motor_name, {})
        num = motor_name.replace("M", "")
        
        steps_per_unit = axis.get("steps_per_unit", 1.0)
        units = axis.get("units", "")
        
        if units == "deg":
            lines.append(f"const float STEPS_PER_DEGREE_M{num} = {steps_per_unit:.4f};")
        elif units == "mm":
            lines.append(f"const float STEPS_PER_MM_M{num} = {steps_per_unit:.2f};")
    lines.append("")
    
    # Direction inversion
    lines.append("// ── Direction Inversion ─────────────────────────────────────────────────────")
    for motor_name in sorted(motors.keys()):
        m = motors[motor_name]
        num = motor_name.replace("M", "")
        inverted = m.get("dir_inverted", False)
        lines.append(f"const bool DIR_REVERSE_M{num} = {'true' if inverted else 'false'};")
    
    return "\n".join(lines)


def generate_motor_shield_header(profile: dict) -> str:
    """Generate header for Adafruit Motor Shield setup (MKII style)."""
    hw = profile.get("arduino_hardware", {})
    motors = hw.get("motors", {})
    axes = {a["name"]: a for a in profile.get("axes", [])}
    
    lines = [
        "// Auto-generated from profile: " + profile.get("name", "unknown"),
        "// Generated: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
        "// DO NOT EDIT - regenerate from profile using generate_arduino_config.py",
        "",
        "#pragma once",
        "",
        "#include <Wire.h>",
        "#include <Adafruit_MotorShield.h>",
        "",
        "// ── Motor Shield Configuration ─────────────────────────────────────────────",
    ]
    
    for motor_name in sorted(motors.keys()):
        m = motors[motor_name]
        num = motor_name.replace("M", "")
        port = m.get("shield_port", int(num))
        steps = m.get("steps_per_revolution", 200)
        mode = m.get("stepping_mode", "DOUBLE")
        
        lines.append(f"// Motor {num}: {m.get('terminals', '')} - {m.get('_comment', '')}")
        lines.append(f"const int MOTOR_{num}_PORT = {port};")
        lines.append(f"const int MOTOR_{num}_STEPS_PER_REV = {steps};")
        lines.append(f"const uint8_t STEP_STYLE_M{num} = {mode};")
        lines.append("")
    
    # Calibration constants
    lines.append("// ── Calibration Constants ───────────────────────────────────────────────────")
    for motor_name in sorted(motors.keys()):
        m = motors[motor_name]
        axis = axes.get(motor_name, {})
        num = motor_name.replace("M", "")
        
        steps_per_unit = axis.get("steps_per_unit", 1.0)
        units = axis.get("units", "")
        
        if units == "deg":
            lines.append(f"const float STEPS_PER_DEGREE_M{num} = {steps_per_unit:.4f};")
        elif units == "mm":
            lines.append(f"const float STEPS_PER_MM_M{num} = {steps_per_unit:.2f};")
    lines.append("")
    
    # Speed settings
    lines.append("// ── Speed Settings (RPM) ────────────────────────────────────────────────────")
    for motor_name in sorted(motors.keys()):
        m = motors[motor_name]
        num = motor_name.replace("M", "")
        rpm = m.get("speed_rpm", 10.0)
        lines.append(f"const float SPEED_M{num}_RPM = {rpm:.1f};")
    lines.append("")
    
    # Direction inversion
    lines.append("// ── Direction Inversion ─────────────────────────────────────────────────────")
    for motor_name in sorted(motors.keys()):
        m = motors[motor_name]
        num = motor_name.replace("M", "")
        inverted = m.get("dir_inverted", False)
        lines.append(f"const bool DIR_REVERSE_M{num} = {'true' if inverted else 'false'};")
    
    return "\n".join(lines)


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 generate_arduino_config.py <profile_name>")
        print("  Available profiles:")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        profiles_dir = os.path.join(script_dir, "profiles")
        for f in os.listdir(profiles_dir):
            if f.endswith(".json"):
                print(f"    - {f[:-5]}")
        sys.exit(1)
    
    profile_name = sys.argv[1]
    profile = load_profile(profile_name)
    
    # Determine which type of hardware
    hw = profile.get("arduino_hardware", {})
    
    if "shield" in hw:
        # Motor shield based
        header = generate_motor_shield_header(profile)
        output_name = f"config_{profile_name}.h"
    else:
        # TB6600/direct driver based
        header = generate_tb6600_header(profile)
        output_name = f"config_{profile_name}.h"
    
    # Write header file
    script_dir = os.path.dirname(os.path.abspath(__file__))
    output_path = os.path.join(script_dir, "arduino", output_name)
    
    with open(output_path, 'w') as f:
        f.write(header)
    
    print(f"Generated: {output_path}")
    print("\nTo use in Arduino sketch, add:")
    print(f'  #include "{output_name}"')


if __name__ == "__main__":
    main()
