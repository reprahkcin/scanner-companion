# XMP Feature Implementation - Usage Guide

## Overview
The XMP feature has been successfully implemented! This adds camera pose data to your 3D scanner captures for seamless import into RealityCapture/RealityScan.

## What Was Added

### 1. New Calibration Inputs
Added to the **Calibration tab**:
- **Lens to Object (mm)**: Distance from lens entrance pupil to object center
- **Rail to Horizon (deg)**: Camera pitch angle (positive = up, negative = down)

### 2. Enhanced Calibration Data
Calibration files now save both focus positions AND XMP settings:
```json
{
  "focus_positions": {
    "0": {"near": 0.0, "far": 10.0},
    "90": {"near": 0.0, "far": 10.0},
    "180": {"near": 0.0, "far": 10.0},
    "270": {"near": 0.0, "far": 10.0}
  },
  "xmp_settings": {
    "lens_to_object_mm": 250.0,
    "rail_to_horizon_deg": 0.0
  }
}
```

### 3. Automatic XMP Generation
During capture, the system automatically:
- Calculates camera poses on a circle around the object
- Generates one XMP sidecar per stack (per angle)
- Saves XMP files alongside images for RealityCapture import

### 4. Camera Pose Math
Uses the ring_pose() function to calculate:
- Camera positions on a tilted circle around the object
- Look-at-origin rotation matrices
- RealityCapture-compatible XMP format

## New Calibration Workflow

1. **Complete cardinal angle calibration** (as before)
   - Set focus positions at 0°, 90°, 180°, 270°

2. **Enter XMP pose settings** (new step)
   - Measure distance from lens to object center
   - Set rail tilt angle if camera is angled up/down

3. **Save calibration** 
   - Now includes both focus data and XMP settings

## Capture Output

Each capture session now creates a **self-contained, portable** directory:
```
specimen_name/
  session_20251101_143022/
    metadata.json          # Includes XMP settings
    README.md             # ← Workflow guide for this session
    rename_xmp_for_rc.py  # ← Helper script (portable!)
    xmp_files/            # ← Consolidated XMP directory
      stack_00.xmp
      stack_01.xmp
      stack_02.xmp
      ...
    stack_00/
      stack_00_shot_000_angle_000.00.jpg
      stack_00_shot_001_angle_000.00.jpg
      ...
      stack_00_angle_000.00.xmp     # Individual XMP (also copied to xmp_files/)
    stack_01/
      ...
    ...
```

**🎯 Key Benefit**: Copy the entire session folder to any computer - everything you need is included!

## RealityCapture Integration

### 🚀 **Super Simple Workflow** (Pi → Desktop → RealityCapture)

1. **On Raspberry Pi**: Run your capture session
2. **Copy session folder** to external drive (everything is self-contained!)  
3. **On Desktop**: Focus stack your images using Helicon Focus, etc.
4. **In session folder**: Rename XMP files using the included helper script:
   ```bash
   # Navigate to your copied session directory
   cd session_20251101_143022
   
   # Run the helper script (it's right there in the folder!)
   python rename_xmp_for_rc.py . "specimen_angle_{angle:03.0f}.xmp"
   
   # This creates xmp_renamed/ with files like:
   # specimen_angle_000.xmp, specimen_angle_090.xmp, etc.
   ```
5. **Copy both** your focus-stacked images and renamed XMP files to your RealityCapture project
6. **Import and align** with camera priors enabled

### Advanced Usage
```bash
# Custom naming patterns
python rename_xmp_for_rc.py . "my_specimen_{angle:03.0f}.xmp"
python rename_xmp_for_rc.py . "scan_{angle:06.2f}.xmp"

# The "." means "current directory" - perfect when you're in the session folder
```

### RealityCapture Settings
1. **Import** both images and XMP files (same directory, matching names)
2. **Enable camera priors** in Alignment settings  
3. **Increase prior hardness** (Position and Orientation) to respect calculated poses
4. **Run alignment** - cameras should form a perfect ring!

## Coordinate System

- **Object at origin** (0,0,0)
- **Camera moves on circle** around object
- **X**: Right, **Y**: Forward, **Z**: Up
- **Radius**: lens_to_object_mm distance
- **Height**: rail_to_horizon_deg tilt angle

The virtual coordinate system treats the camera as moving around a stationary object, even though physically your turntable moves the object around a stationary camera.

## Backward Compatibility

- Old calibration files still load correctly
- XMP settings default to reasonable values (250mm, 0°)
- All existing functionality unchanged