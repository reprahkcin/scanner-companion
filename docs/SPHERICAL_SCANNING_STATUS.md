# Spherical Scanning Implementation Status

**Date**: December 18, 2025  
**Status**: ✅ Hardware integration complete - Motor 3 (tilt axis) and power relay functional

---

## Overview

Added vertical tilt axis (Motor 3) with 5:1 planetary gearbox to enable spherical photogrammetry scans. This allows capturing specimens from angles above and below the horizon, providing complete 360° coverage around a spherical volume instead of just a horizontal ring.

**Scan Pattern Goal**: Spiral sweep from +45° (looking down) to -45° (looking up), continuously rotating the turntable while descending through elevation angles.

---

## ✅ Completed Work

### 1. Arduino Firmware - Motor 3 Support
**File**: `arduino/scanner_controller/scanner_controller.ino`

**Changes**:
- Added pin assignments for TB6600 driver:
  - `STEP_PIN_3 = 9`
  - `DIR_PIN_3 = 10`
  - `ENABLE_PIN_3 = 11`
  - `LIMIT_SWITCH_3 = 8` (reserved for future use)
- Added `STEPS_PER_DEGREE_M3 = 88.8889` calibration constant (5:1 gearbox)
- Added `positionM3_deg` position tracker
- Implemented `tiltMotor()` function
- Added `TILT` command to serial protocol:
  - Format: `TILT 3 <degrees> <UP|DOWN>`
  - Example: `TILT 3 5.0 UP` → tilt specimen 5° upward
- Extended `ZERO` and `GET_POS` commands to support Motor 3

### 2. Arduino Firmware - Power Relay Control
**File**: `arduino/scanner_controller/scanner_controller.ino`

**Changes**:
- Added `POWER_RELAY_PIN = A0` for relay control
- Implemented `setMotorPower()` function (supports low-trigger relays)
- Added `POWER ON/OFF` and `GET_POWER` serial commands
- Safe default: motors powered OFF on boot
- Extended `DEBUG_PINS` output to show relay state

### 3. Python GUI - Motor 3 Controls
**File**: `scanner_control.py`

**Changes**:
- Added `self.motor3_position_deg` position tracker (initialized to 0.0°)
- Created Motor 3 UI frame in Manual Control tab:
  - Position display
  - Step size input (default 1.0°)
  - Three buttons: ▼ Down, ⌂ Home, ▲ Up
- Implemented control functions:
  - `motor3_up()` - Tilt specimen upward (positive angle)
  - `motor3_down()` - Tilt specimen downward (negative angle)
  - `motor3_home()` - Zero the tilt axis position

### 4. Python GUI - Motor Power Control
**File**: `scanner_control.py`

**Changes**:
- Added `self.motor_power_on` state tracker
- Added Motor Power status display and toggle button in Manual Control tab
- Implemented `toggle_motor_power()` function
- Updated serial command parsing to accept `OK <message>` responses

### 5. Spherical Pose Mathematics
**File**: `scanner_control.py`

**Function**: `spherical_pose(radius_mm, azimuth_deg, elevation_deg)`

**Features**:
- Calculates camera position on sphere surface
- Generates proper rotation matrix (camera always looks at origin)
- Handles gimbal lock at extreme elevations (±90°)
- Coordinate system: Z-up world, azimuth 0° = +X axis
- Returns `RigPose(pos_m, R_rowmajor)` compatible with XMP generation

**Validation** (completed):
- All rotation matrices are orthonormal with det=1
- Camera positions lie exactly on specified sphere radius
- Forward vectors correctly point toward origin
- Tested extreme angles: horizontal, ±45° elevation, oblique angles

---

## 🔧 Remaining Work

### 4. Spiral Sweep Capture Sequence
**Status**: Not started - hardware ready, software implementation needed

**Location**: Modify `_run_capture_sequence()` in `scanner_control.py`

**Implementation Plan**:

1. **Spiral Path Calculation**:
   ```python
   # User inputs:
   sphere_radius_mm = 100.0        # Distance from camera to specimen
   elevation_range = (-45, 45)     # Degrees (bottom to top)
   thread_count = 5                # Number of spiral threads
   shots_per_thread = 36           # Shots per 360° rotation
   focus_slices = 5                # Focus stack depth per pose
   
   # Calculate total shots
   total_shots = thread_count * shots_per_thread
   elevation_span = elevation_range[1] - elevation_range[0]
   
   # For each shot:
   for shot_idx in range(total_shots):
       progress = shot_idx / (total_shots - 1)
       elevation = elevation_range[1] - (progress * elevation_span)  # Top to bottom
       azimuth = (shot_idx * 360.0 / shots_per_thread) % 360.0
       
       # Move motors
       move_to_angle(azimuth)          # Motor 1: turntable
       move_to_tilt(elevation)         # Motor 3: tilt axis (NEW)
       
       # Focus stack at this (azimuth, elevation) pose
       for focus_slice in range(focus_slices):
           focus_position = interpolate_focus_position(azimuth, slice/slices)
           move_to_focus_position(focus_position)  # Motor 2: linear rail
           capture_image(filepath)
       
       # Generate XMP using spherical_pose()
       pose = spherical_pose(sphere_radius_mm, azimuth, elevation)
       write_xmp_sidecar(xmp_path, pose, ...)
   ```

2. **Motor Movement Function** (add to `scanner_control.py`):
   ```python
   def move_to_tilt(self, target_elevation_deg):
       """Move tilt axis (Motor 3) to specified elevation angle"""
       current = self.motor3_position_deg
       delta = target_elevation_deg - current
       
       if abs(delta) < 0.1:  # Already at target
           return True
       
       direction = "UP" if delta > 0 else "DOWN"
       command = f"TILT 3 {abs(delta):.2f} {direction}"
       
       if self.send_motor_command(command):
           self.motor3_position_deg = target_elevation_deg
           return True
       return False
   ```

3. **File Naming Convention**:
   - Current: `stack_00_shot_000_angle_000.00.jpg` (angle only)
   - Proposed: `shot_000_az_000.0_el_45.0_focus_0.jpg` (azimuth + elevation + focus)
   - XMP: `shot_000_az_000.0_el_45.0.xmp` (one per pose, references all focus slices)

4. **Metadata Updates**:
   ```json
   {
     "scan_type": "spherical_spiral",
     "sphere_radius_mm": 100.0,
     "elevation_range_deg": [-45, 45],
     "thread_count": 5,
     "total_poses": 180,
     "focus_slices_per_pose": 5,
     "total_images": 900
   }
   ```

---

### 5. UI Controls for Spherical Mode
**Status**: Not started

**Location**: Capture tab in `scanner_control.py`

**Required UI Elements**:

1. **Scan Mode Selector**:
   - Radio buttons: "Horizontal Ring" (existing) vs "Spherical Spiral" (new)
   - Show/hide relevant parameters based on selection

2. **Spherical Parameters** (show when spherical mode selected):
   ```
   ┌─ Spherical Scan Settings ─────────────────────┐
   │ Sphere Radius:      [100.0] mm               │
   │ Elevation Range:    [-45] to [+45] degrees   │
   │ Thread Count:       [5]                       │
   │ Shots per Thread:   [36] (10° steps)         │
   │ Focus Slices:       [5]                       │
   │                                               │
   │ Total Poses:        180                       │
   │ Total Images:       900                       │
   └───────────────────────────────────────────────┘
   ```

3. **Parameter Validation**:
   - Sphere radius > 0
   - Elevation range within ±90°
   - Thread count ≥ 1
   - Shots per thread ≥ 4

4. **Calibration Considerations**:
   - Existing calibration system is angle-based (horizontal ring)
   - Spherical mode may need different approach:
     - Option A: Disable calibration, use constant sphere radius
     - Option B: Extend calibration to include elevation angles
   - Recommend **Option A** for initial implementation

---

### 6. Testing & Validation
**Status**: Hardware verified, capture sequence testing pending

**Completed Tests**:
- ✅ Motor 3 wiring and basic movement
- ✅ Direction mapping (UP = positive angle)
- ✅ Steps/degree calibration (88.8889 with 5:1 gearbox)
- ✅ Power relay control

**Remaining Tests**:

1. **Motor 3 Hardware Test**:
   - Verify wiring and basic movement
   - Test direction mapping (UP = positive angle)
   - Calibrate steps/degree constant
   - Test limit switch (if installed)

2. **Combined Movement Test**:
   - Move to (azimuth=0°, elevation=0°) - horizontal, +X axis
   - Move to (azimuth=90°, elevation=0°) - horizontal, +Y axis
   - Move to (azimuth=0°, elevation=45°) - above, +X axis
   - Move to (azimuth=0°, elevation=-45°) - below, +X axis
   - Verify smooth transitions and accurate positioning

3. **Capture Test** (small sample):
   - 2 threads, 8 shots per thread = 16 poses
   - 2 focus slices per pose = 32 images
   - Verify file naming and organization
   - Check XMP files are generated correctly

4. **XMP Validation**:
   - Generate test set: `temp/test_spherical_xmp_set.py`
   - Import to RealityScan mobile app
   - Check pose dropdown appears
   - Verify camera positions on sphere
   - Run alignment and check model orientation
   - **Critical**: Check if models are upside-down (known issue with horizontal poses)

5. **Full Scan Test**:
   - 5 threads, 36 shots/thread = 180 poses
   - 5 focus slices = 900 images
   - Monitor for:
     - Motor overheating
     - Position drift
     - Camera stability
     - File system performance

---

## 📋 Hardware Setup Checklist

Motor 3 and power relay are now installed:

- [x] **Mechanical Installation**:
  - [x] Mount tilt mechanism securely
  - [x] Ensure smooth rotation without binding
  - [x] Check specimen remains centered during tilt
  - [x] Verify camera clears specimen at all angles

- [x] **Electrical Connection**:
  - [x] Connect TB6600 driver to Arduino pins 9, 10, 11
  - [x] Verify driver power supply (24V)
  - [x] Set microstepping switches on driver (32x microstepping)
  - [x] Connect enable pin (active HIGH per firmware)
  - [x] Connect 24V relay to Arduino pin A0

- [x] **Initial Testing**:
  - [x] Upload latest `scanner_controller.ino` firmware
  - [x] Test TILT command via Serial Monitor
  - [x] Verify direction (DIR_REVERSE_M3 = false works correctly)
  - [x] Calibrate STEPS_PER_DEGREE_M3 = 88.8889 (5:1 gearbox)
  - [x] Test position tracking accuracy
  - [x] Test POWER ON/OFF commands

- [x] **Software Configuration**:
  - [x] Launch Python GUI
  - [x] Test Motor 3 controls in Manual Control tab
  - [x] Verify position display updates correctly
  - [x] Test homing and position reset
  - [x] Test motor power toggle button

- [ ] **Limit Switch** (optional, future enhancement):
  - [ ] Install limit switch at bottom position (-45° or lower)
  - [ ] Connect to pin 8 (reserved)
  - [ ] Uncomment limit switch code in firmware
  - [ ] Test homing routine

---

## 🔍 Known Issues & Considerations

### 1. Upside-Down Model Issue
**Status**: Unresolved (affects horizontal ring poses too)

The current `ring_pose()` rotation matrix produces geometrically valid poses, but models render upside-down in RealityScan. Investigation showed the current rotation matrix convention `[right, up, forward]` is mathematically correct (orthonormal, det=1, origin at positive Z in camera space).

**Possible Causes**:
- World coordinate system mismatch (Z-up vs Y-up)
- RealityCapture expects different camera convention
- Row-major vs column-major interpretation

**Impact on Spherical Scanning**:
- `spherical_pose()` uses same convention as `ring_pose()`
- Will likely have same orientation issue
- May need to test different rotation matrix arrangements

**Testing Strategy**:
1. Generate minimal test set (4 poses: N, E, S, W at horizon)
2. Try different rotation matrix variants:
   - `[right, -up, forward]` (flip vertical)
   - `[right, up, -forward]` (flip look direction)
   - Column-major instead of row-major
3. Import each variant to RealityScan
4. Identify correct convention
5. Update both `ring_pose()` and `spherical_pose()`

### 2. Focus Stacking at Oblique Angles
When camera tilts up/down, the focus plane changes relative to the specimen. Current calibration system assumes horizontal ring.

**Options**:
- **Simple**: Use constant sphere radius, disable focus calibration
- **Advanced**: Extend calibration to (azimuth, elevation) grid
- **Compromise**: Use average focus distance for all angles

Recommend starting with constant radius approach.

### 3. PosePrior Settings
Current: `PosePrior="locked"` prevents RealityScan from adjusting poses.

**Issue**: Focus stacking introduces small position shifts that need correction.

**Recommendation**: Change to `PosePrior="draft"` or `"initial"` to allow refinement (needs testing).

---

## 📚 Reference Files

**Core Implementation**:
- `arduino/scanner_controller.ino` - Motor 3 firmware
- `scanner_control.py` - Main GUI and control logic
  - Lines 133-198: `spherical_pose()` function
  - Lines 1516-1560: Motor 3 control functions
  - Lines 2404+: `_run_capture_sequence()` (needs spherical mode)

**Testing & Validation**:
- `temp/test_rotation_conventions.py` - Rotation matrix validation
- `temp/test_spherical_poses.py` - Spherical pose generation test
- `temp/final_pose_sets/` - Working XMP reference files

**Documentation**:
- `.github/copilot-instructions.md` - Project context and conventions
- `docs/FEATURES.md` - Feature documentation
- `docs/hardware_setup.md` - Wiring and assembly guide

---

## 🚀 Next Steps (When Motor Arrives)

1. **Day 1 - Hardware Setup**:
   - Install Motor 3 mechanically
   - Wire TB6600 driver to Arduino
   - Upload firmware and test basic movement
   - Calibrate steps/degree constant

2. **Day 2 - Software Integration**:
   - Test Motor 3 controls in GUI
   - Implement `move_to_tilt()` function
   - Test combined movements (azimuth + elevation)

3. **Day 3 - Capture Sequence**:
   - Add spherical mode UI controls
   - Implement spiral sweep algorithm
   - Test with small sample scan (2 threads, 8 shots)

4. **Day 4 - XMP & Validation**:
   - Generate test XMP files
   - Import to RealityScan
   - Debug pose orientation if needed
   - Test full scan sequence

5. **Day 5 - Refinement**:
   - Optimize spiral pattern parameters
   - Tune focus stacking approach
   - Test with real specimen
   - Document final workflow

---

## 📞 Support & Troubleshooting

**Common Issues**:

| Problem | Solution |
|---------|----------|
| Motor 3 not responding | Check wiring, verify firmware upload, test with Serial Monitor |
| Wrong direction | Toggle `DIR_REVERSE_M3` in firmware |
| Position drift | Recalibrate `STEPS_PER_DEGREE_M3`, check mechanical binding |
| XMP import fails | Validate rotation matrix, check file naming consistency |
| Models upside-down | Test alternative rotation matrix conventions |

**Testing Commands** (Arduino Serial Monitor):
```
TILT 3 10 UP          # Tilt 10° upward
TILT 3 10 DOWN        # Tilt 10° downward
GET_POS 3             # Query current tilt position
ZERO 3                # Reset tilt position to 0°
```

---

## 📝 Notes for Future Development

- Consider adding limit switches for auto-homing
- Implement safety stops for extreme angles
- Add collision detection (camera hitting specimen)
- Support asymmetric elevation ranges (e.g., +60° to -30°)
- Variable thread density (more samples at difficult angles)
- Real-time preview during spiral sweep
- Pause/resume capability for long scans
- Integration with focus stacking software (Helicon, Zerene)

---

**Last Updated**: November 10, 2025  
**Status**: Ready for hardware integration once Motor 3 arrives
