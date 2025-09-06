# 3D Scanner Control Panel

A comprehensive control application for a dual-motor 3D photogrammetry scanner using Raspberry Pi and Arduino.

![Scanner Control Panel](https://img.shields.io/badge/version-4.0-blue) ![Python](https://img.shields.io/badge/python-3.8%2B-green) ![License](https://img.shields.io/badge/license-MIT-yellow)

## 🚀 Quick Start

1. **Hardware Setup**
   - Connect Arduino with `arduino/scanner_controller.ino` loaded
   - Attach Pi Camera and stepper motors
   - Ensure proper wiring and power supply

2. **Software Installation**
   ```bash
   # Clone the repository
   git clone https://github.com/reprahkcin/scanner-companion.git
   cd scanner-companion
   
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
   - Follow the guided workflow to set focus positions
   - Save calibration for future use

## ✨ Features

### 📷 Manual Control
- **Live Camera Preview**: Real-time camera feed with adjustable settings
- **Precision Motor Control**: Control rotation and linear rail motors
- **Position Tracking**: Real-time position display for both motors

### 🎯 Automated Calibration
- **Cardinal Point Calibration**: Guided setup at 0°, 90°, 180°, and 270°
- **Focus Range Mapping**: Define near and far focus positions
- **Automatic Interpolation**: Smart interpolation for all angles
- **Persistent Storage**: Save/load calibration data

### 📸 Automated Capture
- **Multi-Stack Workflow**: Capture multiple focus stacks
- **360° Coverage**: Configurable shots per rotation
- **Intelligent Focus**: Automatic focus based on calibration
- **Flexible Formats**: Support for JPG, PNG, and TIFF
- **Organized Output**: Hierarchical folder structure

### ⚙️ Camera Settings
- **Manual Controls**: Shutter speed, brightness, exposure
- **Auto/Manual Modes**: Flexible exposure settings
- **Real-time Metadata**: Live camera property display

## 📁 Project Structure

```
scanner-companion/
├── README.md                    # This file
├── scanner_control.py          # Main application
├── requirements.txt            # Python dependencies
├── run_scanner.sh             # Launch script
├── arduino/
│   └── scanner_controller.ino # Arduino firmware
├── docs/
│   └── FEATURES.md            # Detailed feature documentation
├── examples/
│   └── sample_calibration.json # Example calibration
└── legacy/                    # Previous versions (v1-v4)
    ├── v1/                    # Original prototype
    ├── v2/                    # Early development
    ├── v3/                    # GUI development
    └── v4/                    # Final development version
```

## 🔧 Hardware Requirements

### Electronics
- **Raspberry Pi 4+** (recommended)
- **Arduino Uno/Nano** with stepper motor drivers
- **2x Stepper Motors** (rotation + linear rail)
- **Pi Camera Module** (v2 or HQ Camera)

### Mechanical
- **Rotating Platform** for specimen mounting
- **Linear Rail System** for focus adjustment
- **Stable Camera Mount** and specimen positioning

## 📊 Workflow

### Calibration Process
1. **Position at Cardinal Points**: 0°, 90°, 180°, 270°
2. **Set Focus Range**: Define near and far focus positions
3. **Validation**: System verifies calibration completeness
4. **Save Settings**: Store for future capture sessions

### Capture Process
- **Stack Distribution**: Even focus distribution between limits
- **360° Rotation**: Automatic positioning with specified increments
- **Focus Interpolation**: Smart interpolation between calibrated points
- **Progress Tracking**: Real-time progress and detailed logging

## 📋 File Organization

Captured images are organized in a hierarchical structure:

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

## 🤝 Contributing

We welcome contributions! Here's how to get started:

1. **Fork the repository**
2. **Create a feature branch**: `git checkout -b feature/your-feature`
3. **Test on actual hardware** before submitting
4. **Follow PEP 8** style guidelines
5. **Document new features** in the appropriate files
6. **Submit a pull request**

### Development Guidelines
- Test changes on actual hardware when possible
- Maintain backward compatibility for calibration files
- Update documentation for new features
- Include example configurations where applicable

## 🐛 Troubleshooting

### Common Issues
1. **Serial Connection**: Check `/dev/ttyACM0` and Arduino sketch
2. **Camera Preview**: Verify picamera2 installation
3. **Motor Movement**: Check wiring and power supply
4. **Calibration**: Ensure all cardinal points are set

### Dependencies
```bash
# System packages (Ubuntu/Debian)
sudo apt update
sudo apt install python3-opencv python3-pil python3-serial

# Pi-specific packages
sudo apt install python3-picamera2
```

## 📄 License

This project is open source under the MIT License. See individual component licenses for details.

## 🔗 Links

- **Documentation**: [docs/FEATURES.md](docs/FEATURES.md)
- **Arduino Firmware**: [arduino/scanner_controller.ino](arduino/scanner_controller.ino)
- **Examples**: [examples/](examples/)
- **Legacy Versions**: [legacy/](legacy/)

## 📞 Support

For questions, issues, or contributions, please:
1. Check existing issues on GitHub
2. Create a new issue with detailed description
3. Include hardware setup and error logs
4. Test with provided example configurations

---

**Built with ❤️ for the 3D scanning community**