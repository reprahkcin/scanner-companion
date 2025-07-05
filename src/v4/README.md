# 3D Scanner Control Panel v4.0

A comprehensive control application for a dual-motor 3D photogrammetry scanner using Raspberry Pi and Arduino.

## Features

### Manual Control
- **Camera Preview**: Live camera feed with adjustable settings
- **Motor Control**: Precise control of rotation (Motor 1) and linear rail (Motor 2)
- **Position Tracking**: Real-time position display for both motors

### Calibration System
- **Cardinal Point Calibration**: Guided calibration at 0°, 90°, 180°, and 270°
- **Focus Range Mapping**: Define near and far focus positions for each cardinal point
- **Automatic Interpolation**: System interpolates focus positions for all angles
- **Save/Load Calibration**: Persistent calibration data in JSON format

### Automated Capture
- **Multi-Stack Workflow**: Capture multiple focus stacks with even distribution
- **360° Coverage**: Configurable shots per stack (e.g., 72 shots = 5° increments)
- **Intelligent Focus**: Automatic focus positioning based on calibration data
- **Flexible Output**: Support for JPG, PNG, and TIFF formats
- **Organized File Structure**: Hierarchical folder organization by specimen/session/stack

### Camera Settings
- **Manual Controls**: Shutter speed, brightness adjustment
- **Auto/Manual Modes**: Flexible exposure settings
- **Real-time Info**: Live camera property and metadata display

## Quick Start

1. **Hardware Setup**
   - Connect Arduino with `scanner_controller.ino` loaded
   - Attach camera and motors
   - Ensure proper wiring and power

2. **Software Installation**
   ```bash
   # Install dependencies
   pip install -r requirements.txt
   
   # Run the application
   ./run_scanner.sh
   # or
   python3 scanner_control.py
   ```

3. **Initial Calibration**
   - Go to **Calibration** tab
   - Click "Start Calibration"
   - Follow the guided workflow to set focus positions at each cardinal point
   - Save calibration for future use

4. **Capture Workflow**
   - Go to **Capture** tab
   - Set specimen name and output directory
   - Configure stacks and shots per stack
   - Click "Start Full Capture"

## Workflow Details

### Calibration Process
1. **Position at 0°**: Use manual controls to position specimen facing forward
2. **Set Near Focus**: Move linear rail to closest focus position, click "Capture Position"
3. **Set Far Focus**: Move to farthest focus position, click "Capture Position"
4. **Next Angle**: Click "Next Step" to move to 90°, 180°, then 270°
5. **Complete**: System validates and enables capture functionality

### Capture Process
- **Stack Distribution**: Focus positions evenly distributed between near and far
- **Angle Calculation**: Automatic 360° rotation with specified shot count
- **Focus Interpolation**: Linear interpolation between calibrated cardinal points
- **Settling Delays**: Configurable delays after movement for vibration dampening
- **Progress Tracking**: Real-time progress bar and detailed logging

## File Organization

```
output_directory/
└── specimen_name/
    └── session_YYYYMMDD_HHMMSS/
        ├── metadata.json
        ├── stack_00/
        │   ├── stack_00_shot_000_angle_000.00.jpg
        │   ├── stack_00_shot_001_angle_005.00.jpg
        │   └── ...
        ├── stack_01/
        └── ...
```

## Configuration

### Serial Communication
- **Port**: `/dev/ttyACM0` (configurable in script)
- **Baudrate**: 115200
- **Protocol**: Text commands matching Arduino sketch

### Motor Commands
- `ROTATE 1 <degrees> <CW|CCW>` - Rotate motor 1
- `MOVE 2 <mm> <FORWARD|BACKWARD>` - Move motor 2
- `ZERO <motor>` - Zero motor position
- `GET_POS <motor>` - Get current position

### Default Settings
- **Stacks**: 5
- **Shots per Stack**: 72 (5° increments)
- **Settle Delay**: 1.0 seconds
- **Image Format**: JPG

## Hardware Requirements

### Electronics
- Raspberry Pi 4+ (recommended)
- Arduino Uno/Nano with stepper drivers
- 2x Stepper motors (rotation + linear rail)
- Pi Camera Module

### Mechanical
- Rotating platform for specimen
- Linear rail system for focus adjustment
- Stable mounting for camera and specimen

## Advanced Features

### Focus Interpolation Algorithm
The system uses bilinear interpolation to calculate focus positions:
1. **Cardinal Points**: Uses calibrated positions at 0°, 90°, 180°, 270°
2. **Angle Interpolation**: Linear interpolation between adjacent cardinal points
3. **Stack Interpolation**: Linear distribution between near and far focus
4. **Wrap-around**: Handles 270° to 0° transition seamlessly

### Camera Integration
- **picamera2**: Native Pi Camera support
- **OpenCV**: Image processing and preview
- **PIL**: Multi-format image saving
- **Real-time Controls**: Exposure, brightness, and other parameters

### Error Handling
- **Serial Timeout**: Robust Arduino communication
- **Camera Errors**: Graceful camera failure handling
- **Movement Validation**: Position verification after commands
- **User Interruption**: Safe capture sequence stopping

## Troubleshooting

### Common Issues
1. **Serial Connection**: Check `/dev/ttyACM0` port and Arduino sketch
2. **Camera Preview**: Verify picamera2 installation and camera connection
3. **Movement Errors**: Check motor wiring and power supply
4. **Calibration**: Ensure all cardinal points have both near and far positions

### Dependencies
```bash
# Install system packages (Ubuntu/Debian)
sudo apt update
sudo apt install python3-opencv python3-pil python3-serial

# Install Pi-specific packages
sudo apt install python3-picamera2
```

## License

This project is open source. See individual component licenses for details.

## Development

### Project Structure
- `scanner_control.py` - Main application
- `arduino/scanner_controller.ino` - Arduino firmware
- `requirements.txt` - Python dependencies
- `sample_calibration.json` - Example calibration data

### Contributing
1. Test on hardware before submitting changes
2. Follow PEP 8 style guidelines
3. Document new features in README
4. Maintain backward compatibility where possible
