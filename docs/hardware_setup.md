# Hardware Setup Guide

This guide covers the physical setup and wiring for the 3D Scanner Control Panel.

## Hardware Components

### Required Components
- Raspberry Pi 4 (4GB+ recommended)
- Arduino Uno/Nano
- 2x Stepper Motors (NEMA 17 recommended)
- 2x Stepper Motor Drivers (A4988 or DRV8825)
- Pi Camera Module (v2 or HQ Camera)
- Power Supply (12V for motors, 5V for Pi)
- Breadboard/PCB for connections
- Jumper Wires

### Mechanical Components
- Rotating Platform (lazy susan bearing)
- Linear Rail System (20mm rail recommended)
- Lead Screw (T8 or similar)
- Motor Mounts
- Camera Mount
- Specimen Platform

## Wiring Diagram

### Arduino Connections

Motor 1 (Rotation) - Driver 1:
- Arduino Pin 8 -> STEP
- Arduino Pin 9 -> DIR
- Arduino Pin 10 -> ENABLE

Motor 2 (Linear Rail) - Driver 2:
- Arduino Pin 5 -> STEP
- Arduino Pin 6 -> DIR
- Arduino Pin 7 -> ENABLE

Motor Driver Power:
- Driver VDD -> 5V (Arduino)
- Driver VMOT -> 12V (External PSU)
- Driver GND -> GND (Common)

## Safety Considerations

- Use proper gauge wires for motor current
- Ensure all connections are secure
- Test emergency stop functionality
- Provide adequate clearance for movement

## Assembly Checklist

- [ ] All components received and verified
- [ ] Base frame assembled and level
- [ ] Rotation motor mounted and aligned
- [ ] Linear rail system installed
- [ ] Camera mount attached and secure
- [ ] Arduino programmed with firmware
- [ ] Motor drivers installed and configured
- [ ] All wiring completed per diagram
- [ ] Power supplies connected and tested
- [ ] Serial communication verified
- [ ] Full system integration test
