# 3D Scanner v1.1 - Features Summary

## What's New in v1.1

### Major New Features
1. **Motor 3 (Tilt Axis)** - Vertical tilt control with 5:1 planetary gearbox for spherical scanning
2. **Motor Power Relay** - GUI-controlled 24V power relay for safe motor management
3. **Spherical Pose Math** - `spherical_pose()` function for future spherical capture sequences
4. **3-Axis Manual Control** - Full GUI controls for all three motor axes

### Technical Improvements
- Calibrated step constants for all motors (32x microstepping)
- Relay control with safe boot defaults (motors OFF)
- Extended serial protocol with TILT, POWER, GET_POWER commands
- Updated TB6600 driver enable logic documentation

## What's in v1.0

### Core Features
1. **Complete Calibration Wizard** - Step-by-step calibration at cardinal points
2. **Automated Capture Sequence** - Full 360° multi-stack capture workflow
3. **Focus Interpolation** - Intelligent focus positioning based on calibration
4. **Camera Settings Panel** - Manual exposure and camera controls
5. **Tabbed Interface** - Organized GUI with separate tabs for different functions

### Technical Improvements
- **Image Format Options**: JPG, PNG, TIFF support
- **Progress Tracking**: Real-time progress bars and detailed logging
- **File Organization**: Structured output with metadata
- **Error Handling**: Robust error checking and user feedback
- **Threading**: Non-blocking capture operations

## User Workflow

### Initial Setup (One-time)
1. **Hardware**: Connect Arduino, camera, motors
2. **Calibration**: Run calibration wizard to map focus positions
3. **Test**: Perform test captures to verify settings

### Daily Operation
1. **Specimen Setup**: Position specimen on platform
2. **Configure**: Set specimen name, output directory, capture parameters
3. **Capture**: Run automated sequence
4. **Review**: Check generated images and metadata

## Key Concepts

### Calibration System
- **Cardinal Points**: 0°, 90°, 180°, 270° reference positions
- **Focus Range**: Near and far focus limits for each cardinal point
- **Interpolation**: Automatic calculation of focus for any angle
- **Persistence**: Save/load calibration data for reuse

### Capture Parameters
- **Stacks**: Number of focus levels (default: 5)
- **Shots per Stack**: Images per 360° rotation (default: 72 = 5° increments)
- **Settle Delay**: Pause after movement for vibration dampening
- **Output Format**: JPG (fast), PNG (lossless), TIFF (highest quality)

## Usage Tips

### For Best Results
1. **Stable Platform**: Ensure no vibration during capture
2. **Consistent Lighting**: Use stable light sources
3. **Small Specimens**: Works best with objects 2-10cm
4. **Manual Focus**: Use calibration to map exact focus range
5. **Test First**: Always do test captures before full sequences

### Image Format Selection
- **JPG**: Fast capture, good for most photogrammetry
- **PNG**: Lossless compression, larger files
- **TIFF**: Highest quality, largest files, best for archival

### Troubleshooting
- **Serial Errors**: Check Arduino connection and sketch
- **Camera Issues**: Verify picamera2 installation
- **Slow Capture**: Reduce settle delay or image resolution
- **Focus Problems**: Recalibrate or check linear rail movement

## Advanced Features

### Custom Configurations
- Edit CONFIGURATION SECTION in scanner_control.py
- Adjust default values for stacks, shots, delays
- Modify output directory structure

### Calibration Data
- JSON format for easy editing
- Manual adjustment possible for fine-tuning
- Backup important calibration files

### Integration Ready
- Metadata includes all capture parameters
- Organized file structure for processing pipelines
- Future expansion points for additional camera controls

## Future Enhancements
- Spherical spiral capture sequences (Motor 3 integration with capture workflow)
- Auto-focus algorithms
- Exposure bracketing
- Lighting control integration
- Real-time photogrammetry preview
- Cloud upload and processing

This v1.1 release adds the hardware foundation for spherical scanning, with full capture sequence integration coming in a future release.
