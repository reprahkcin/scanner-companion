"""XMP pose calculation and sidecar generation for RealityCapture.

Contains:
- `RigPose` dataclass for camera position + rotation
- `ring_pose()` and `spherical_pose()` for computing camera poses
- `write_xmp_sidecar()` for writing RealityCapture-compatible XMP files
- Verified XMP constants (do not change unless re-validated)

This module is deliberately free of GUI and hardware dependencies so it can
be used from the Tkinter GUI, the web server, and test scripts alike.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from xml.sax.saxutils import escape


# ──────────────────────────────────────────────────────────────────────────────
# VERIFIED CONSTANTS — do not change unless re-validated with RealityCapture
# ──────────────────────────────────────────────────────────────────────────────

VERIFIED_DISTORTION_MODEL = "brown3"
VERIFIED_DISTORTION_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
VERIFIED_FOCAL_LENGTH_35MM = 50.0
VERIFIED_SKEW = 0.0
VERIFIED_ASPECT_RATIO = 1.0
VERIFIED_PRINCIPAL_POINT_U = 0.0
VERIFIED_PRINCIPAL_POINT_V = 0.0
VERIFIED_POSE_PRIOR = "locked"
VERIFIED_CALIB_PRIOR = "exact"


# ──────────────────────────────────────────────────────────────────────────────
# DATA STRUCTURES
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class RigPose:
    """Camera pose for XMP export"""
    pos_m: tuple[float, float, float]     # (x,y,z) position in meters
    # 9 numbers, row-major 3x3 rotation matrix
    R_rowmajor: tuple[float, ...]


# ──────────────────────────────────────────────────────────────────────────────
# VECTOR HELPERS (pure math, no numpy dependency)
# ──────────────────────────────────────────────────────────────────────────────

def _vsub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
def _vdot(a, b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]


def _vcross(a, b): return (a[1]*b[2]-a[2]*b[1],
                           a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])


def _vnorm(a):
    m = math.sqrt(_vdot(a, a))
    return (a[0]/m, a[1]/m, a[2]/m) if m > 0 else (0, 0, 1)


# ──────────────────────────────────────────────────────────────────────────────
# POSE CALCULATION
# ──────────────────────────────────────────────────────────────────────────────

def ring_pose(distance_mm: float, rail_deg: float, theta_deg: float) -> RigPose:
    """Calculate camera pose on a tilted ring around object at origin.

    Args:
        distance_mm: Distance from lens entrance pupil to object center
        rail_deg: Rail tilt angle (positive = camera pitched up from horizontal)
        theta_deg: Angle around the object (0° = +X axis, 90° = +Y axis)

    Returns:
        RigPose with camera position and look-at-origin rotation matrix
    """
    # Object center at origin. X right, Y forward, Z up in rig space.
    r = distance_mm / 1000.0  # Convert to meters
    th = math.radians(theta_deg)
    a = math.radians(rail_deg)

    # Camera center in world coordinates on tilted ring
    x = r * math.cos(th) * math.cos(a)
    y = r * math.sin(th) * math.cos(a)
    z = r * math.sin(a)

    # Build a "look-at origin" rotation matrix
    C = (x, y, z)
    tgt = (0.0, 0.0, 0.0)  # Looking at object center
    upW = (0.0, 0.0, 1.0)  # World up vector (Z-up per RealityCapture standard)

    # Follow reference document exactly: f = normalize(L - C)
    # f points FROM camera TO look-at (inward toward origin)
    f = _vnorm(_vsub(tgt, C))  # Inward vector

    # Handle gimbal lock when camera is directly above/below
    if abs(f[0]) < 1e-6 and abs(f[1]) < 1e-6:
        # Camera pointing straight up or down
        rgt = (1.0, 0.0, 0.0)  # Arbitrary right vector
    else:
        # Use right = cross(worldUp, forward) for RC convention
        rgt = _vnorm(_vcross(upW, f))  # Right

    # Up = cross(forward, right)
    # Up (already unit length from cross of two unit vectors)
    up = _vcross(f, rgt)

    # Adopt empirically validated RealityCapture convention (variant 16):
    # rows = [right, up, forward]
    R9 = (rgt[0], rgt[1], rgt[2],
          up[0], up[1], up[2],
          f[0], f[1], f[2])

    return RigPose((x, y, z), R9)


def spherical_pose(radius_mm: float, azimuth_deg: float, elevation_deg: float) -> RigPose:
    """Calculate camera pose on a sphere around object at origin.

    This generates poses for spherical photogrammetry where the camera orbits
    the specimen at a constant distance on a virtual sphere.

    Args:
        radius_mm: Distance from camera to specimen center (sphere radius)
        azimuth_deg: Horizontal angle around specimen (0° = +X axis, 90° = +Y axis)
        elevation_deg: Vertical angle (-45° = below, 0° = horizontal, +45° = above)

    Returns:
        RigPose with camera position and look-at-origin rotation matrix

    Coordinate System:
        World: X-right, Y-forward, Z-up (RealityCapture standard)
        Camera: Looks down -Z axis, Y-up, X-right (OpenCV/RC standard)
    """
    # Convert angles to radians
    azimuth = math.radians(azimuth_deg)
    elevation = math.radians(elevation_deg)
    r = radius_mm / 1000.0  # Convert to meters

    # Calculate camera position on sphere (spherical → Cartesian)
    # Elevation: 0° = horizontal (XY plane), +90° = straight up (+Z), -90° = straight down (-Z)
    x = r * math.cos(elevation) * math.cos(azimuth)
    y = r * math.cos(elevation) * math.sin(azimuth)
    z = r * math.sin(elevation)

    # Build rotation matrix for camera looking at origin
    camera_pos = (x, y, z)
    target = (0.0, 0.0, 0.0)  # Specimen at origin
    world_up = (0.0, 0.0, 1.0)  # Z-up world

    # Forward vector: from camera toward target (inward)
    forward = _vnorm(_vsub(target, camera_pos))

    # Handle gimbal lock when camera is directly above/below specimen
    if abs(forward[0]) < 1e-6 and abs(forward[1]) < 1e-6:
        # Camera pointing straight up or down - use arbitrary right vector based on azimuth
        right = (math.cos(azimuth + math.pi/2),
                 math.sin(azimuth + math.pi/2), 0.0)
        right = _vnorm(right)
    else:
        # Standard case: right = world_up × forward
        right = _vnorm(_vcross(world_up, forward))

    # Up vector: forward × right (camera Y-axis points up in image)
    up = _vcross(forward, right)

    # Build rotation matrix (row-major: transforms world → camera coordinates)
    R9 = (right[0], right[1], right[2],
          up[0], up[1], up[2],
          forward[0], forward[1], forward[2])

    return RigPose(camera_pos, R9)


# ──────────────────────────────────────────────────────────────────────────────
# XMP SIDECAR WRITER
# ──────────────────────────────────────────────────────────────────────────────

def write_xmp_sidecar(img_path: str, pose: RigPose,
                      lens_to_object_mm: float, rail_to_horizon_deg: float,
                      theta_deg: float, stack_index: int,
                      focal_length_35mm: float = 50.0,
                      distortion_model: str = "brown3",
                      skew: float = 0.0,
                      aspect_ratio: float = 1.0,
                      principal_point_u: float = 0.0,
                      principal_point_v: float = 0.0,
                      distortion_coefficients: tuple = (
                          -0.1, 0.1, 0.0, 0.0, 0.0, 0.0),
                      calibration_group: int = -1,
                      distortion_group: int = -1,
                      in_texturing: int = 1,
                      in_meshing: int = 1,
                      position_scale: float = 1.0,
                      pose_prior: str = "locked",
                      calibration_prior: str = "exact") -> None:
    """Write XMP sidecar file with camera pose data in RealityCapture format.

    Generates XMP metadata files conforming to RealityCapture's XMP specification.
    See: https://rshelp.capturingreality.com/en-US/tools/xmpalign.htm

    Args:
        img_path: Path to the image file
        pose: Camera pose data (position and rotation matrix)
        lens_to_object_mm: Distance from lens to object center (for metadata)
        rail_to_horizon_deg: Rail tilt angle (for metadata)
        theta_deg: Rotation angle around object (for metadata)
        stack_index: Stack/perspective index (for metadata)
        focal_length_35mm: 35mm equivalent focal length
        distortion_model: Lens distortion model (e.g., "brown3", "division")
        skew: Camera skew parameter
        aspect_ratio: Pixel aspect ratio
        principal_point_u: Principal point U coordinate (normalized)
        principal_point_v: Principal point V coordinate (normalized)
        distortion_coefficients: Tuple of 6 distortion coefficients (k1-k6)
        calibration_group: Calibration group ID (-1 for independent)
        distortion_group: Distortion group ID (-1 for independent)
        in_texturing: Include in texturing phase (1=yes, 0=no)
        in_meshing: Include in meshing phase (1=yes, 0=no)
        position_scale: Scale factor for position values (e.g., 1000 for macro)

    Note:
        RealityCapture requires specific attribute spelling: "DistortionCoeficients"
        (not "Coefficients"). The rotation matrix must be 9 space-separated values
        in row-major order. Per RealityCapture XMP Camera Math, Rotation and Position
        are encoded as child elements, not attributes.
    """
    XCR_NS = 'http://www.capturingreality.com/ns/xcr/1.1#'

    # Per RealityCapture documentation: Position is the CAMERA location in world coordinates
    # The rotation matrix R transforms world coordinates to camera coordinates
    # See: https://dev.epicgames.com/community/learning/knowledge-base/vzwB/realityscan-realitycapture-xmp-camera-math

    # Camera position in world coordinates (meters)
    P = pose.pos_m

    # Apply position scale and format to match verified good set exactly (6 decimal places)
    position_str = f"{P[0] * position_scale:.6f} {P[1] * position_scale:.6f} {P[2] * position_scale:.6f}"

    R = pose.R_rowmajor

    # Format distortion coefficients to match verified good set (integers when zero)
    dist_str = " ".join("0" if abs(c) < 1e-9 else str(c)
                        for c in distortion_coefficients)

    # Format rotation matrix to match verified good set exactly (10 decimal places)
    rotation_str = " ".join(f"{r:.10f}" for r in R)

    # Build XMP content to match verified good set format exactly
    xmp_content = f'''<x:xmpmeta xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#" xmlns:x="adobe:ns:meta/" xmlns:xcr="{XCR_NS}">
  <rdf:RDF>
    <rdf:Description xcr:Version="3" xcr:PosePrior="{escape(pose_prior)}" xcr:CalibrationPrior="{escape(calibration_prior)}" xcr:Coordinates="absolute" xcr:DistortionModel="{escape(str(distortion_model))}" xcr:DistortionCoeficients="{escape(dist_str)}" xcr:FocalLength35mm="{focal_length_35mm:.0f}" xcr:Skew="{skew:.0f}" xcr:AspectRatio="{aspect_ratio:.0f}" xcr:PrincipalPointU="{principal_point_u:.0f}" xcr:PrincipalPointV="{principal_point_v:.0f}" xcr:CalibrationGroup="{calibration_group}" xcr:DistortionGroup="{distortion_group}" xcr:InTexturing="{in_texturing}" xcr:InMeshing="{in_meshing}">
      <xcr:Rotation>{rotation_str}</xcr:Rotation>
      <xcr:Position>{position_str}</xcr:Position>
    </rdf:Description>
  </rdf:RDF>
</x:xmpmeta>'''

    # Create XMP file path (replace extension with .xmp)
    xmp_path = img_path.rsplit('.', 1)[0] + '.xmp'

    with open(xmp_path, 'w', encoding='utf-8') as f:
        f.write(xmp_content)
