# Copilot Instructions for Scanner Companion Project

## Project Overview

This is a 3D photogrammetry scanner control system that generates XMP sidecar files for RealityScan/RealityCapture. The scanner captures images from a Raspberry Pi camera mounted on a circular rig that rotates around the specimen, generating camera pose metadata in XMP format for photogrammetry reconstruction.

## Critical XMP Configuration (DO NOT CHANGE)

### Verified Working XMP Format

The following XMP settings have been empirically validated and MUST be used exactly as specified:

```xml
<x:xmpmeta xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#" xmlns:x="adobe:ns:meta/" xmlns:xcr="http://www.capturingreality.com/ns/xcr/1.1#">
  <rdf:RDF>
    <rdf:Description
    xcr:Version="3"
    xcr:PosePrior="locked"
    xcr:CalibrationPrior="exact"
    xcr:Coordinates="absolute"
    xcr:DistortionModel="brown3"
    xcr:DistortionCoeficients="0 0 0 0 0 0"
    xcr:FocalLength35mm="50"
    xcr:Skew="0"
    xcr:AspectRatio="1"
    xcr:PrincipalPointU="0"
    xcr:PrincipalPointV="0"
    xcr:CalibrationGroup="-1"
    xcr:DistortionGroup="-1"
    xcr:InTexturing="1"
    xcr:InMeshing="1">
    <xcr:Rotation>[9 space-separated values]</xcr:Rotation>
    <xcr:Position>[3 space-separated values]</xcr:Position>
    </rdf:Description>
  </rdf:RDF>
</x:xmpmeta>
```

### Critical Constants in scanner_control.py

These constants are locked and validated - DO NOT modify without explicit user request:

```python
VERIFIED_POSE_PRIOR = "locked"           # Gives RealityScan dropdown options
VERIFIED_CALIB_PRIOR = "exact"           # Gives RealityScan dropdown options
VERIFIED_DISTORTION_MODEL = "brown3"
VERIFIED_DISTORTION_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
VERIFIED_FOCAL_LENGTH_35MM = 50.0
VERIFIED_SKEW = 0.0
VERIFIED_ASPECT_RATIO = 1.0
VERIFIED_PRINCIPAL_POINT_U = 0.0
VERIFIED_PRINCIPAL_POINT_V = 0.0
```

### XMP Formatting Requirements

- **Attribute placement**: Each xcr attribute on its own line with proper indentation
- **Child elements**: `xcr:Rotation` and `xcr:Position` must be child elements, NOT attributes
- **Rotation format**: 9 space-separated floats formatted to 10 decimal places
- **Position format**: 3 space-separated floats formatted to 6 decimal places
- **NO line breaks within attribute values** (e.g., no breaks in `xcr:CalibrationPrior="exact"`)
- **Spelling**: Use `DistortionCoeficients` (NOT "Coefficients") per RealityCapture spec

### Camera Pose Convention (Validated)

The working pose convention uses:

- **World coordinate system**: Z-up
- **Camera orientation**: Looking at origin from circular path
- **Rotation matrix construction**:
  ```python
  right = np.cross(world_up, forward)
  up = np.cross(forward, right)
  R = [right, up, forward]  # Row-major 3x3 matrix
  ```
- **Position**: Camera center in world coordinates (meters)
- **Rotation direction**: Counter-clockwise around Z-axis when viewed from above
- **Starting angle**: 0° = camera at +X axis looking toward origin

## RealityScan Integration

### UI to XMP Mapping

- **PosePrior="locked"** enables the "Relative pose" dropdown in RealityScan
  - Dropdown shows: Unknown, Draft, Exact
  - User can manually select "Draft" after import
- **CalibrationPrior="exact"** enables the "Prior" calibration dropdown
  - Shows: Approximate, Fixed

### Known Issues

- Setting `PosePrior="draft"` or `PosePrior="initial"` breaks RealityScan import
- Always use `PosePrior="locked"` and let user manually adjust in UI if needed
- Line breaks within XML attribute values cause import failures

## Project Structure

### Core Files

- **scanner_control.py**: Main application with GUI, motor control, and XMP generation
- **arduino/scanner_controller.ino**: Arduino firmware for stepper motor control
- **requirements.txt**: Python dependencies

### Test/Reference Directories

- **temp/**: Untracked directory for test scripts and temporary files
  - **final_pose_sets/**: Working XMP reference files (64-camera scans)
    - `start0_ccw/`: Verified working XMP format starting at 0°
  - **small fly major test/**: Current 120-camera test scan
    - `stacks/`: 120 XMP files with 3° angular steps

### Important Notes

- Working reference XMPs are in `temp/final_pose_sets/start0_ccw/`
- Use 2-digit padding for 64-camera scans: `stack_00.xmp` to `stack_63.xmp`
- Use 3-digit padding for 120-camera scans: `stack_000.xmp` to `stack_119.xmp`

## Development Guidelines

### Creating Temporary Scripts

**ALWAYS create test/utility scripts in the `temp/` directory** to keep them untracked:

```python
# Good: Untracked temporary script
temp_script_path = r"f:\GIT\scanner-companion\temp\my_test_script.py"

# Bad: Creates tracked file in repository root
bad_path = r"f:\GIT\scanner-companion\my_test_script.py"
```

### Terminal Command Issues

- Windows terminal has character truncation issues with long commands
- **Always use Python scripts instead of direct terminal commands** for reliability
- Create utility scripts in `temp/` directory when needed
- Avoid `ls`, `cat`, or other commands that output large amounts of text

### Testing XMP Changes

When testing XMP format changes:

1. Create test script in `temp/` directory
2. Make changes to `temp/small fly major test/stacks/` files
3. Import into RealityScan to validate
4. If successful, update `scanner_control.py` constants
5. Document the working format

### Code Style

- Use descriptive variable names
- Add comments for complex pose math
- Keep XMP generation logic together
- Maintain dataclass structures for pose data

## Common Tasks

### Updating XMP Format

1. **Reference check**: Compare against `temp/final_pose_sets/start0_ccw/stack_00.xmp`
2. **Test in temp directory**: Create script in `temp/` to modify test XMPs
3. **Validate in RealityScan**: Import and check dropdown options
4. **Update scanner_control.py**: Only if validation succeeds
5. **Document changes**: Note what was changed and why

### Debugging XMP Import Issues

1. Compare problematic XMP against working reference
2. Check for line breaks within attributes
3. Verify spelling of "DistortionCoeficients"
4. Ensure Rotation/Position are child elements, not attributes
5. Check decimal precision (10 places for rotation, 6 for position)

### Adding New Features

- Test on actual hardware when possible
- Maintain backward compatibility
- Update this document with any new critical constants
- Document XMP format dependencies

## Reference Links

- [RealityCapture XMP Spec](https://rshelp.capturingreality.com/en-US/tools/xmpalign.htm)
- [XMP Camera Math](https://dev.epicgames.com/community/learning/knowledge-base/vzwB/capturing-reality-realitycapture-xmp-camera-math)

## Critical Warnings

### DO NOT:

- Modify `VERIFIED_*` constants without explicit user request and empirical validation
- Change XMP formatting (spacing, line breaks, attribute order)
- Use values other than "locked"/"exact" for pose/calibration priors without testing
- Create scripts in repository root (use `temp/` directory)
- Attempt to "improve" the pose math without validation against working reference
- Add line breaks within XML attribute values

### ALWAYS:

- Reference `temp/final_pose_sets/start0_ccw/stack_00.xmp` as the gold standard
- Create temporary scripts in `temp/` directory
- Test XMP changes in RealityScan before committing
- Preserve exact formatting of working XMP files
- Use Python scripts instead of terminal commands for reliability

## Session Continuity

When starting a new chat session, remind the AI of:

1. The critical XMP constants that are locked
2. The location of working reference files (`temp/final_pose_sets/`)
3. The terminal command issues requiring Python scripts
4. The requirement to create temporary scripts in `temp/` directory
5. The validated camera pose convention (Z-up, CCW, cross products for basis)
