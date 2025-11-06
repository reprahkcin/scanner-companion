# XMP Camera Calibration Guide

## Overview

The scanner application now generates XMP sidecar files that are fully compatible with RealityCapture's expectations. These XMP files contain camera pose information (position and rotation) as well as camera calibration parameters (lens distortion, focal length, etc.).

## What Changed

### Previous Format Issues
- Used custom namespace (`sc:`) for scanner-specific metadata
- Missing standard RealityCapture camera calibration attributes
- Minimal XMP structure that wasn't well-recognized by RealityCapture

### New Format (RealityCapture Compatible)
- Uses only standard `xcr:` namespace from RealityCapture
- Includes comprehensive camera calibration parameters:
  - **Focal length** (35mm equivalent)
  - **Lens distortion model** (Brown distortion with 6 coefficients)
  - **Principal points** (optical center offset)
  - **Aspect ratio and skew**
  - **Calibration/distortion groups**
  - **Include flags** for texturing and meshing phases

## Camera Calibration Parameters

### Basic Pose Settings (Pose Tab)

#### Lens to Object Distance (mm)
- **What it is**: Physical distance from the lens entrance pupil to the center of the object
- **How to measure**: Use a ruler to measure from the camera lens to the object center
- **Typical value**: 150-500mm depending on your setup
- **Effect**: Affects the absolute scale of the 3D reconstruction

#### Rail to Horizon Angle (degrees)  
- **What it is**: The pitch angle of the camera rail from horizontal
- **Positive values**: Camera pointed upward from horizontal
- **Negative values**: Camera pointed downward from horizontal
- **Typical value**: -5° to +5° for most setups
- **Effect**: Corrects for any tilt in your linear rail mounting

#### Focal Length (35mm equivalent)
- **What it is**: The focal length of your camera lens expressed in 35mm film equivalent
- **For Raspberry Pi HQ Camera**: Typically 50-60mm equivalent
- **For Pi Camera v2/v3**: Typically 28-35mm equivalent
- **How to find**: Check camera specifications or use a lens calculator
- **Effect**: Critical for accurate 3D reconstruction scale and proportions

### Camera Intrinsics (Intrinsics Tab)

#### Principal Point U and V
- **What they are**: Normalized coordinates of the optical center of the lens
- **U coordinate**: Horizontal offset (-1.0 to 1.0, where 0 = center)
- **V coordinate**: Vertical offset (-1.0 to 1.0, where 0 = center)
- **Default**: (0.0, 0.0) assumes optical center is at sensor center
- **When to adjust**: If your lens is decentered or you have measured principal points
- **Effect**: Corrects for lens optical center not being perfectly centered

#### Aspect Ratio
- **What it is**: The ratio of pixel width to pixel height
- **Default**: 1.0 (square pixels)
- **When to adjust**: Rarely needed with modern sensors
- **Effect**: Corrects for non-square pixels

#### Skew
- **What it is**: The angle between pixel rows and columns  
- **Default**: 0.0 (orthogonal pixels)
- **When to adjust**: Almost never needed with modern sensors
- **Effect**: Corrects for non-orthogonal sensor array (manufacturing defect)

### Lens Distortion (Distortion Tab)

The Brown distortion model uses 6 coefficients to correct for lens distortion:

#### Radial Distortion (k1, k2, k3)
- **What they are**: Coefficients for barrel/pincushion distortion
- **k1**: Primary radial distortion (typically -0.5 to 0.5)
- **k2**: Secondary radial distortion (typically -0.5 to 0.5)  
- **k3**: Tertiary radial distortion (typically -0.2 to 0.2)
- **Negative values**: Barrel distortion (image bows outward)
- **Positive values**: Pincushion distortion (image bows inward)
- **Default**: All 0.0 (no distortion correction)

#### Tangential Distortion (p1, p2)
- **What they are**: Coefficients for asymmetric lens distortion
- **Typical values**: -0.05 to 0.05
- **Cause**: Lens elements not being perfectly parallel
- **Default**: 0.0 (no tangential distortion)

#### Additional Radial (k4)
- **What it is**: Fourth-order radial distortion coefficient
- **Typical values**: Very small, often 0
- **When needed**: High distortion lenses (fisheye, wide-angle)
- **Default**: 0.0

## How to Get Camera Calibration Values

### Option 1: Use Defaults (Good Starting Point)
If you don't have calibration data, the default values will work but may result in less accurate reconstructions:
- Focal length: 50mm (adjust based on your camera)
- Principal points: 0, 0 (sensor center)
- Distortion: All 0.0 (no correction)
- Aspect ratio: 1.0, Skew: 0.0

### Option 2: Perform Camera Calibration (Recommended)
Use OpenCV or other camera calibration tools:

1. **Print a calibration pattern** (checkerboard or ArUco markers)
2. **Capture 10-20 images** of the pattern at different angles and distances
3. **Run calibration software**:
   - OpenCV: `cv2.calibrateCamera()`
   - MATLAB Camera Calibrator app
   - RealityCapture's internal calibration tools

4. **Extract the following from calibration results**:
   - Focal length (convert to 35mm equivalent if needed)
   - Principal point (cx, cy) → normalize to (-1, 1) range
   - Distortion coefficients (k1-k3, p1-p2)

### Option 3: Import from RealityCapture (Advanced)
If you've already processed images in RealityCapture:

1. Export XMP files from a completed project
2. Open one of the exported XMP files
3. Copy the calibration values:
   - `xcr:FocalLength35mm`
   - `xcr:PrincipalPointU` and `xcr:PrincipalPointV`  
   - `xcr:DistortionCoeficients` (6 values)
4. Enter these values in the scanner app's Calibration tab

## Workflow Integration

### During Calibration
1. Go to the **Calibration** tab
2. Navigate to the **Camera Calibration (XMP)** section
3. Switch between tabs (Pose, Intrinsics, Distortion) to enter your values
4. Complete the focus calibration wizard as normal
5. Click **Save Calibration** - this saves both focus positions AND camera calibration

### During Capture
- Camera calibration values are automatically written to each XMP file
- Each image gets an XMP sidecar with its specific pose and shared camera calibration
- The XMP files are consolidated in the `xmp_files` directory of each capture session

### In RealityCapture
1. Import your captured images
2. RealityCapture will automatically detect and use the XMP sidecars
3. Camera calibration and poses are loaded from the XMP files
4. Alignment should be significantly better than without XMP data

## Troubleshooting

### RealityCapture Ignores XMP Files
- Check that XMP files have the same base name as images (e.g., `image.jpg` + `image.xmp`)
- Verify XMP files are in the same directory or in the `xmp_files` subfolder
- Ensure focal length is reasonable (20-200mm for most applications)

### Poor Alignment Despite XMP Data
- Verify lens to object distance is accurate
- Check that distortion coefficients match your lens
- Ensure focal length is correct (most common issue)
- Try recalibrating with a calibration pattern

### XMP Files Look Correct But Results Are Off
- Principal points should be small (-0.5 to 0.5) for centered lenses
- Check distortion coefficient magnitudes - very large values (>2.0) may indicate calibration error
- Verify pose calculation by checking that object appears centered in all views

## Example Values

### Raspberry Pi HQ Camera with C-Mount Lens (16mm)
```
Focal Length (35mm eq): 53mm
Principal Point U: 0.0
Principal Point V: 0.0
Distortion k1: -0.26
Distortion k2: 1.09
Distortion k3: -1.29
Distortion p1: 0.0
Distortion p2: 0.0
```

### Raspberry Pi Camera v2
```
Focal Length (35mm eq): 29mm
Principal Point U: 0.0
Principal Point V: 0.0
Distortion k1: 0.05
Distortion k2: -0.02
Distortion k3: 0.0
Distortion p1: 0.0
Distortion p2: 0.0
```

## References

- [RealityCapture XMP Documentation](https://support.capturingreality.com/)
- [Brown Distortion Model](https://en.wikipedia.org/wiki/Distortion_(optics))
- [OpenCV Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

## File Format Reference

The generated XMP files follow this structure:

```xml
<x:xmpmeta xmlns:x="adobe:ns:meta/">
  <rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">
    <rdf:Description xcr:Version="3" xcr:PosePrior="initial" xcr:Coordinates="absolute"
       xcr:DistortionModel="brown3" xcr:FocalLength35mm="50.0"
       xcr:Skew="0.0" xcr:AspectRatio="1.0" xcr:PrincipalPointU="0.0"
       xcr:PrincipalPointV="0.0" xcr:CalibrationPrior="initial"
       xcr:CalibrationGroup="-1" xcr:DistortionGroup="-1" xcr:InTexturing="1"
       xcr:InMeshing="1" xmlns:xcr="http://www.capturingreality.com/ns/xcr/1.1#">
      <xcr:Rotation>[9 rotation matrix values]</xcr:Rotation>
      <xcr:Position>[x y z position in meters]</xcr:Position>
      <xcr:DistortionCoeficients>[k1 k2 k3 p1 p2 k4]</xcr:DistortionCoeficients>
    </rdf:Description>
  </rdf:RDF>
</x:xmpmeta>
```

All attributes use the standard `xcr:` namespace recognized by RealityCapture.
