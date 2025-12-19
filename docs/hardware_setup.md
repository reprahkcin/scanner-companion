# Hardware Setup Guide

This guide covers the physical setup and wiring for the 3D Scanner Control Panel.

## Hardware Components

### Required Components
- Raspberry Pi 4 (4GB+ recommended)
- Arduino Uno/Nano
- 3x Stepper Motors (NEMA 17 recommended)
- 3x Stepper Motor Drivers (TB6600 recommended for torque)
- Pi Camera Module (v2 or HQ Camera)
- Power Supply (24V for motors, 5V for Pi)
- 24V Relay Module (low-trigger, for motor power control)
- Breadboard/PCB for connections
- Jumper Wires

### Mechanical Components
- Rotating Platform (lazy susan bearing) - Motor 1
- Linear Rail System (20mm rail recommended) - Motor 2
- Lead Screw (T8 or similar) - Motor 2
- Vertical Tilt Mechanism with 5:1 planetary gearbox - Motor 3
- Motor Mounts
- Camera Mount
- Specimen Platform

## Wiring Diagram

### Arduino Connections

Motor 1 (Rotation) - Driver 1:
- Arduino Pin 2 -> STEP
- Arduino Pin 3 -> DIR
- Arduino Pin 13 -> ENABLE

Motor 2 (Linear Rail) - Driver 2:
- Arduino Pin 4 -> STEP
- Arduino Pin 5 -> DIR
- Arduino Pin 12 -> ENABLE
- Arduino Pin 7 -> Limit Switch (INPUT_PULLUP)

Motor 3 (Tilt) - Driver 3:
- Arduino Pin 9 -> STEP
- Arduino Pin 10 -> DIR
- Arduino Pin 11 -> ENABLE
- Arduino Pin 8 -> Limit Switch (reserved, INPUT_PULLUP)

Power Relay:
- Arduino Pin A0 -> Relay IN (low-trigger: LOW = ON)
- Relay COM -> 24V power supply positive
- Relay NO -> Motor drivers VMOT

Motor Driver Common:
- Driver ENA+ -> 5V (Arduino)
- Driver ENA- -> Arduino ENABLE pin
- Driver VDD -> 5V (Arduino)
- Driver VMOT -> 24V via relay
- Driver GND -> GND (Common)

### TB6600 Enable Logic
With ENA+ tied to 5V and ENA- connected to Arduino pin:
- Pin HIGH = Driver ENABLED (motor holds position)
- Pin LOW = Driver DISABLED (motor free, quiet)

See `arduino/TB6600_ENABLE_LOGIC.md` for detailed explanation.

## Safety Considerations

- Use proper gauge wires for motor current (16-18 AWG for 24V motors)
- Ensure all connections are secure
- Motor power defaults to OFF on Arduino boot
- Test emergency stop functionality (POWER OFF command)
- Provide adequate clearance for movement on all 3 axes

## Assembly Checklist

- [ ] All components received and verified
- [ ] Base frame assembled and level
- [ ] Motor 1 (rotation) mounted and aligned
- [ ] Motor 2 (linear rail) system installed
- [ ] Motor 3 (tilt) mechanism with gearbox installed
- [ ] Camera mount attached and secure
- [ ] Arduino programmed with firmware
- [ ] All 3 motor drivers installed and configured (32x microstepping)
- [ ] 24V relay wired for motor power control
- [ ] All wiring completed per diagram
- [ ] Power supplies connected and tested
- [ ] Serial communication verified
- [ ] Full system integration test
