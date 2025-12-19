# Copilot Instructions for Scanner Companion

## Project Overview

**3D photogrammetry scanner control system** for Raspberry Pi that generates XMP sidecar files for RealityScan/RealityCapture. The scanner captures images from multiple angles around a specimen using a rotating platform, linear rail for focus stacking, and vertical tilt axis for spherical coverage, with mathematically precise camera pose metadata.

**Architecture**: Python/Tkinter GUI (`scanner_control.py`) → Serial → Arduino firmware (`arduino/scanner_controller/scanner_controller.ino`) → TB6600 stepper motor drivers → 24V relay for motor power

**Current Development Status**: Hardware control (3 motors + power relay) and image capture are working. XMP pose generation produces valid XML but incorrect camera orientations (models render upside-down in RealityScan). Active work needed on rotation matrix convention and PosePrior settings.

## Critical XMP Format & Known Issues

### Current Status: XMP Generation Under Active Development

**IMPORTANT**: The XMP pose generation is producing geometrically valid but incorrectly oriented camera poses. Models render upside-down in RealityScan, indicating rotation matrix issues.

### Current XMP Constants (Partially Validated)

```python
# In scanner_control.py - XMP constants
VERIFIED_POSE_PRIOR = "locked"           # ISSUE: Should be "draft" or "initial"
VERIFIED_CALIB_PRIOR = "exact"           # Enables calibration dropdown
VERIFIED_DISTORTION_MODEL = "brown3"
VERIFIED_FOCAL_LENGTH_35MM = 50.0
VERIFIED_DISTORTION_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
```

**XMP Formatting Rules** (These are correct):
- Each xcr attribute on own line with proper indentation
- `xcr:Rotation` and `xcr:Position` as **child elements**, NOT attributes
- Rotation: 9 space-separated floats, 10 decimal places
- Position: 3 space-separated floats, 6 decimal places
- Spelling: `DistortionCoeficients` (NOT "Coefficients") per RealityCapture spec
- NO line breaks within attribute values

### Known Issues with Camera Pose Math

1. **Upside-Down Models**: `ring_pose()` rotation matrix produces inverted camera orientation
   - Position coordinates are correct (circular path verified)
   - Rotation matrix convention may be wrong (row vs column major, or axis orientation)
   - Issue affects all perspectives uniformly

2. **PosePrior Setting Confusion**:
   - Currently uses `PosePrior="locked"` which prevents RealityScan from adjusting poses
   - Should use `"draft"` or `"initial"` to allow refinement during alignment
   - Focus stacking introduces slight positional shifts that need correction
   - Goal: Provide good starting poses that can be refined, not rigid constraints

3. **Position Scale Complexity**:
   - `position_scale` parameter exists but adds user complexity
   - Should automatically scale positions to avoid precision issues (e.g., 0.001m → 1.0m)
   - Current approach: `scaled_lens_dist = lens_dist * position_scale / 1000.0`
   - Better approach: Detect when positions would be <0.01m and auto-scale

**Camera Pose Convention** (Z-up world, NEEDS VALIDATION):
```python
# ring_pose() in scanner_control.py - CURRENT implementation (produces upside-down models)
forward = normalize(target - camera_position)  # Inward toward origin
right = normalize(cross(world_up, forward))    # Right = world_up × forward
up = cross(forward, right)                     # Up = forward × right
R = [right, up, forward]  # Row-major 3x3 rotation matrix

# RealityCapture expects: R transforms world coords → camera coords
# Camera looks down -Z axis in camera space (OpenCV/RealityCapture convention)
```

**Angle Calculation** (This is correct):
```python
# In _run_capture_sequence()
angle_step = 360.0 / stacks_count
for perspective in range(stacks_count):
    angle = perspective * angle_step        # Turntable angle (0°, 5°, 10°, ...)
    camera_angle = -angle                   # Camera orbits opposite direction
    pose = ring_pose(lens_dist, rail_angle, camera_angle)
```

Starting angle: 0° = camera at +X axis looking toward origin, counter-clockwise rotation.

## Architecture & Data Flow

### 1. Hardware Control Chain
```
GUI Event → send_motor_command() → Serial "/dev/ttyACM0" @ 115200 baud
  → Arduino parses command → Stepper driver pulses → Motor moves
  → Position tracking in Arduino → Query via GET_POS
```

### 2. Serial Protocol (Arduino Commands)
```cpp
ROTATE <motor> <degrees> <CW|CCW>    // Motor 1: rotation platform
MOVE <motor> <millimeters> <FORWARD|BACKWARD>  // Motor 2: linear rail focus
TILT <motor> <degrees> <UP|DOWN>     // Motor 3: vertical tilt axis
ZERO <motor>                          // Reset position counter
GET_POS <motor>                       // Query current position
POWER <ON|OFF>                        // Control 24V motor power relay
GET_POWER                             // Query power state
```

**Calibration Constants** (in `arduino/scanner_controller/scanner_controller.ino`):
- Motor 1: `STEPS_PER_DEGREE_M1 = 17.7778` (rotation, 32x microstepping)
- Motor 2: `STEPS_PER_MM_M2 = 1281.21` (linear rail, calibrated)
- Motor 3: `STEPS_PER_DEGREE_M3 = 88.8889` (tilt, 5:1 gearbox)
- Limit switch on Motor 2 pin 7 for home detection
- Power relay on pin A0 (low-trigger)

### 3. Calibration System
**Cardinal point calibration** at 0°, 90°, 180°, 270° defines focus envelope:
```python
calibration_data = {
    0: {"near": 45.2, "far": 78.6},    # millimeters on linear rail
    90: {"near": 44.8, "far": 79.1},   # per cardinal angle
    # ...
}
```

**Focus interpolation** (`interpolate_focus_position()`):
- Uses bilinear interpolation between nearest cardinal angles
- Adjusts for specimen shape variations around rotation
- Example: angle 45° uses weighted average of 0° and 90° calibration data

### 4. Capture Workflow
```
Start → Move to angle → Interpolate focus range → Capture focus stack → Generate XMP
  → Rotate to next angle → Repeat → Session complete
```

**File Organization**:
```
output_dir/specimen_name/session_YYYYMMDD_HHMMSS/
  ├── metadata.json                    # Session parameters
  ├── stack_00/                        # Perspective 0 (angle 0°)
  │   ├── stack_00_shot_000_angle_000.00.jpg
  │   ├── stack_00_shot_000_angle_000.00.xmp  # Pose metadata
  │   ├── stack_00_shot_001_angle_000.00.jpg
  │   └── ...
  ├── stack_01/                        # Next perspective
  └── ...
```

**Naming Convention**: 
- 2-digit padding for ≤99 stacks: `stack_00` to `stack_63`
- 3-digit padding for ≥100 stacks: `stack_000` to `stack_119`

## Key Development Patterns

### Camera Control (Picamera2)
```python
# Always use camera_lock for thread safety
with self.camera_lock:
    self.camera.stop()
    config = self.camera.create_still_configuration(...)
    self.camera.configure(config)
    self.camera.start()
    array = self.camera.capture_array()
```

**Resolution switching**: Stop camera → Reconfigure → Restart before capture to avoid sensor mode conflicts.

### Threading Model
- Main GUI thread: Tkinter event loop
- Capture thread: `_run_capture_sequence()` in separate thread
- Preview updates: `after()` callbacks for non-blocking updates
- Use `self.capture_running` flag to coordinate stop requests

### Position Tracking
```python
# Python side maintains mirror of Arduino position counters
self.motor1_position_deg  # Degrees (rotation platform)
self.motor2_position_mm   # Millimeters (linear rail)
self.motor3_position_deg  # Degrees (vertical tilt axis)
self.motor_power_on       # 24V relay state (True/False)

# Sync with Arduino after movements:
actual_pos = self.get_motor_position(motor_num)
```

## Common Development Tasks

### Adding New Camera Settings
1. Add Tkinter variable in `__init__`: `self.new_setting = tk.DoubleVar(value=1.0)`
2. Create UI control in `_build_camera_settings_tab()`
3. Apply in `start_preview()` or `capture_image()`: `self.camera.set_controls({"NewSetting": self.new_setting.get()})`

### Modifying Capture Sequence
Edit `_run_capture_sequence()` method. Key sections:
- **Pre-capture setup**: Directory creation, metadata initialization
- **Main loop**: Perspective iteration with angle calculation
- **Focus stack loop**: Interpolated focus positioning
- **Post-capture**: XMP generation via `write_xmp_sidecar()`

### Testing XMP Changes
**IMPORTANT**: Create test scripts in `temp/` directory (untracked) to avoid repository pollution:
```python
# Good: temp/test_xmp_format.py
# Bad: test_xmp_format.py (tracked in git root)
```

**Validation workflow**:
1. Create test script in `temp/` to generate sample XMPs
2. Import into RealityScan to verify:
   - Dropdown options appear (indicates XMP parsed successfully)
   - Model orientation is correct (not upside-down)
   - Camera positions follow expected sequence
3. Only update `scanner_control.py` constants if validation succeeds
4. Document findings in `temp/` directory markdown files

**Debugging Rotation Matrix Issues**:
```python
# In temp/ test scripts, add validation:
import numpy as np

def validate_rotation_matrix(R):
    """Check if R is valid (orthonormal, det=1)"""
    R_mat = np.array(R).reshape(3, 3)
    # Check orthonormality
    should_be_identity = R_mat @ R_mat.T
    is_orthonormal = np.allclose(should_be_identity, np.eye(3), atol=1e-6)
    # Check determinant
    det = np.linalg.det(R_mat)
    is_right_handed = np.isclose(det, 1.0, atol=1e-6)
    return is_orthonormal and is_right_handed

# Test different rotation conventions:
# Convention 1: [right, up, forward] (current)
# Convention 2: [right, up, -forward] (invert look direction)
# Convention 3: [right, -up, forward] (flip vertical)
# etc.
```

**Position Scale Auto-Detection** (Future Improvement):
```python
# Proposed logic to eliminate user-facing position_scale parameter:
def auto_scale_positions(distance_mm):
    """Automatically scale positions to avoid precision issues"""
    distance_m = distance_mm / 1000.0
    if distance_m < 0.01:  # < 10mm
        scale = 1000.0  # Scale to 1-10m range
    elif distance_m < 0.1:  # 10-100mm
        scale = 100.0   # Scale to 1-10m range
    else:
        scale = 1.0     # No scaling needed
    return distance_m * scale, scale

# Usage in ring_pose():
scaled_distance, scale_factor = auto_scale_positions(distance_mm)
# ... compute pose with scaled_distance ...
# ... add scale_factor to metadata for RC to scale back ...
```

## RealityScan Integration

**XMP to UI Mapping**:
- `PosePrior="locked"` → Enables "Relative pose" dropdown (Unknown/Draft/Exact) but prevents refinement
- `PosePrior="draft"` or `"initial"` → Should allow RealityScan to adjust poses (NEEDS TESTING)
- `CalibrationPrior="exact"` → Enables "Prior" calibration dropdown

**Current Integration Issues**:
1. **Upside-down models**: Rotation matrix convention in `ring_pose()` is incorrect
2. **Rigid poses**: Using `"locked"` prevents correction of focus-stacking-induced shifts
3. **Import failures**: Some `PosePrior` values may break import (needs systematic testing)

**Desired Behavior**:
- XMP should provide **good initial poses** that RealityScan can refine
- Alignment algorithm should adjust for:
  - Focus stacking position shifts
  - Minor calibration errors
  - Camera orientation corrections
- Poses should guide alignment, not rigidly constrain it

**Testing Checklist for XMP Changes**:
1. Import XMP into RealityScan mobile app
2. Check "Prior pose" dropdown appears correctly
3. Verify model orientation (not upside-down)
4. Confirm alignment succeeds without manual intervention
5. Check that position sequence matches filename order

**Reference Documentation**:
- XMP spec: https://rshelp.capturingreality.com/en-US/tools/xmpalign.htm
- Camera math: https://dev.epicgames.com/community/learning/knowledge-base/vzwB/realityscan-realitycapture-xmp-camera-math
- XMP review notes: `temp/xmp_review.md` (filename-to-pose consistency analysis)

## Project Structure

```
scanner-companion/
├── scanner_control.py          # Main app (2800+ lines)
│   ├── ring_pose()            # Camera pose math
│   ├── spherical_pose()       # Spherical pose math
│   ├── write_xmp_sidecar()    # XMP generation
│   ├── ScannerGUI class       # Tkinter application
│   └── VERIFIED_* constants   # Locked XMP parameters
├── arduino/
│   └── scanner_controller/
│       └── scanner_controller.ino  # Motor control firmware
├── calibration_examples/       # Sample calibration JSON files
├── temp/                       # UNTRACKED - use for test scripts
│   └── small_fly_9/           # Working XMP reference files
├── docs/
│   ├── FEATURES.md            # Detailed feature documentation
│   ├── hardware_setup.md      # Wiring and assembly guide
│   ├── SPHERICAL_SCANNING_STATUS.md  # 4th-axis implementation status
│   └── QUICKSTART.md          # Installation steps
└── legacy/                    # Development history (v1-v4)
```

## Configuration & Settings

**Serial Connection** (`SERIAL_PORT = "/dev/ttyACM0"`):
- Raspberry Pi typically auto-assigns `/dev/ttyACM0` for Arduino Uno
- Check with `ls /dev/ttyACM*` or use `/dev/serial/by-id/...` for stability

**Camera Defaults**:
- Preview: 640x480 (performance)
- Capture: 4056x3040 (Pi HQ Camera max resolution)
- Format: JPG (fast), PNG (lossless), TIFF (archival)

**Capture Parameters**:
- Perspectives: 72 (5° steps for 360°) - configurable
- Focus slices: 5 per angle - configurable
- Settle delay: 1.0s after movement - allows vibration dampening

## Development Environment

**Platform**: Raspberry Pi 4+ with Pi Camera Module
**Python**: 3.8+ 
**Dependencies**: `pip install -r requirements.txt`
- picamera2 (Pi-specific, camera control)
- opencv-python (image processing)
- pyserial (Arduino communication)
- Pillow (image format conversion)

**Running**:
```bash
./run_scanner.sh  # Bash wrapper with environment setup
# OR
python3 scanner_control.py
```

**Hardware Setup Checklist**:
1. Arduino loaded with `arduino/scanner_controller/scanner_controller.ino`
2. Stepper drivers wired per `docs/hardware_setup.md`
3. Pi Camera connected and enabled (`sudo raspi-config`)
4. Serial permissions: `sudo usermod -a -G dialout $USER`

## Critical Warnings

### DO NOT:
- Modify XMP formatting (spacing, line breaks, attribute order) - these are correct
- Use values other than "locked"/"exact" for pose/calibration priors without systematic testing
- Create scripts in repository root (use `temp/` directory)
- Add line breaks within XML attribute values
- Assume current rotation matrix convention is correct (it produces upside-down models)

### ALWAYS:
- Create temporary/test scripts in `temp/` directory
- Test XMP changes in RealityScan before committing to codebase
- Validate rotation matrices are orthonormal (use numpy checks)
- Use `camera_lock` when accessing camera from multiple threads
- Stop preview before starting capture sequence
- Query Arduino position after movements for position verification

### NEEDS INVESTIGATION:
1. **Rotation matrix convention**: Current implementation produces upside-down models
   - Try different axis arrangements (e.g., swap up/down, invert forward)
   - Test with minimal XMP (single camera) before full sequence
   - Document which conventions were tested and results
2. **PosePrior values**: Test "draft", "initial", "approximate" to find best setting
   - Goal: RealityScan should refine poses, not lock them rigidly
   - Test if certain values break import entirely
3. **Position scaling**: Implement auto-detection to eliminate user parameter
   - Scale small positions (< 10mm) to avoid precision issues
   - Document scale factor in metadata for manual verification

## Testing & Validation

**Hardware Test Workflow**:
1. Manual Control tab: Test individual motor movements
2. Calibration tab: Run cardinal point calibration
3. Capture tab: Single test capture before full sequence
4. Verify serial communication: Check status bar for errors

**XMP Validation**:
1. Generate test XMP files in `temp/` directory
2. Import into RealityScan mobile app
3. Check "Prior pose" and "Calibration" dropdowns appear
4. Verify no import errors in RealityScan logs

## Troubleshooting

**Serial Connection Issues**:
- Check Arduino is connected: `ls /dev/ttyACM*`
- Verify firmware loaded: Arduino IDE Serial Monitor should show "Ready for ROTATE, MOVE, TILT, ZERO, GET_POS, POWER, GET_POWER, DEBUG_PINS"
- Permission error: Add user to dialout group

**Camera Failures**:
- `picamera2` import error: Raspberry Pi only, ensure system packages installed
- Black screen: Check camera cable connection, enable camera in raspi-config
- Resolution error: Stop preview before capture, ensure sensor supports requested resolution

**Motor Movement Problems**:
- No movement: Check enable pin wiring, driver power supply
- Wrong direction: Adjust `DIR_REVERSE_M1`/`DIR_REVERSE_M2`/`DIR_REVERSE_M3` in Arduino code
- Position drift: Use limit switches for homing, re-zero positions

**XMP Import Failures in RealityScan**:
- Compare against working reference XMP files in `temp/small_fly_9/`
- Check for line breaks within attribute values
- Verify spelling: "DistortionCoeficients" not "Coefficients"
- Ensure XMP filename matches image filename exactly

## Session Continuity Notes

When starting new development session, remember:
1. **XMP generation is under active debugging** - models render upside-down
2. Reference files in `temp/small_fly_9/` show current output format
3. Test scripts always go in `temp/` directory (untracked)
4. Camera pose convention needs validation (Z-up, counter-clockwise rotation)
5. Serial protocol: Commands are synchronous, wait for "OK" response
6. Position scale should be auto-detected, not user-configurable
7. PosePrior should allow refinement ("draft"), not lock poses ("locked")
6. Position scale should be auto-detected, not user-configurable
7. PosePrior should allow refinement ("draft"), not lock poses ("locked")
