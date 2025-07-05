# 3D Scanner Control Panel v4.0

A complete GUI application for controlling a 3D scanner with dual motor control and live camera preview.

## Features
- Live Camera Preview with Picamera2
- Dual Motor Control (rotation and linear)
- Arduino Integration via Serial
- Professional GUI with status feedback
- Real-time position tracking

## Files
- scanner_control.py - Main GUI application
- arduino/scanner_controller.ino - Arduino sketch
- README.md - This documentation

## Usage
1. Upload Arduino sketch to your board
2. Connect Arduino via USB
3. Run: python3 scanner_control.py

## Requirements
- Python 3.7+
- picamera2, opencv-python, Pillow, pyserial
- Arduino with motor drivers
- Raspberry Pi with camera module
