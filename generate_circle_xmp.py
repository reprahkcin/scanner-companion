#!/usr/bin/env python3
"""
Generate XMP files for cameras arranged in a circle looking at origin.
Implements the exact algorithm from realityscan_xmp_pose_reference.md Section 5.
"""

import math
import os


def create_xmp_content(position, rotation_matrix):
    """
    Create XMP file content per the reference manual.
    
    Args:
        position: tuple of (x, y, z) in meters
        rotation_matrix: tuple of 9 floats in row-major order
    """
    pos_str = f"{position[0]} {position[1]} {position[2]}"
    rot_str = " ".join(f"{x}" for x in rotation_matrix)
    
    return f"""<x:xmpmeta xmlns:x="adobe:ns:meta/">
  <rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">
    <rdf:Description
      xmlns:xcr="http://www.capturingreality.com/ns/xcr/1.1#"
      xcr:Version="3"
      xcr:PosePrior="locked"
      xcr:Coordinates="absolute"
      xcr:DistortionModel="perspective"
      xcr:FocalLength35mm="35"
      xcr:Skew="0"
      xcr:AspectRatio="1"
      xcr:PrincipalPointU="0"
      xcr:PrincipalPointV="0"
      xcr:CalibrationPrior="exact"
      xcr:CalibrationGroup="-1"
      xcr:DistortionGroup="-1"
      xcr:InTexturing="1"
      xcr:InMeshing="1">
      <xcr:Rotation>{rot_str}</xcr:Rotation>
      <xcr:Position>{pos_str}</xcr:Position>
    </rdf:Description>
  </rdf:RDF>
</x:xmpmeta>"""


def normalize(v):
    """Normalize a 3D vector."""
    mag = math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)
    if mag == 0:
        return (0, 0, 0)
    return (v[0]/mag, v[1]/mag, v[2]/mag)


def cross(a, b):
    """Cross product of two 3D vectors."""
    return (
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    )


def build_rotation_matrix(C, L, U):
    """
    Build rotation matrix per Section 5 of the reference manual.
    
    Args:
        C: Camera position (cx, cy, cz)
        L: Look-at point (lx, ly, lz)
        U: World up vector (ux, uy, uz)
    
    Returns:
        Tuple of 9 floats (row-major rotation matrix)
    """
    # Step 1: Forward (camera viewing direction)
    f = normalize((L[0] - C[0], L[1] - C[1], L[2] - C[2]))
    
    # Step 2: Right vector
    s = normalize(cross(f, U))
    
    # Step 3: True up
    u = cross(s, f)
    
    # Build rotation matrix per manual: R transforms world to camera
    # Row 1: s.x  s.y  s.z
    # Row 2: u.x  u.y  u.z
    # Row 3: -f.x -f.y -f.z
    R = (
        s[0], s[1], s[2],
        u[0], u[1], u[2],
        -f[0], -f[1], -f[2]
    )
    
    return R


def generate_circle_xmps(output_dir, num_cameras=128, radius_m=0.3):
    """
    Generate XMP files for cameras in a horizontal circle around origin.
    Per the reference manual Section 7 skeleton generator.
    
    Args:
        output_dir: Directory to write XMP files
        num_cameras: Number of cameras around the circle
        radius_m: Radius of circle in meters
    """
    os.makedirs(output_dir, exist_ok=True)
    
    L = (0.0, 0.0, 0.0)  # Look-at point (origin)
    U = (0.0, 0.0, 1.0)  # World up (Z-up per manual Section 3)
    
    for i in range(num_cameras):
        # Calculate angle for this camera (0° = +X axis, 90° = +Y axis)
        theta_deg = (i / num_cameras) * 360.0
        theta_rad = math.radians(theta_deg)
        
        # Camera position on horizontal circle (XY plane, Z=0)
        C = (
            radius_m * math.cos(theta_rad),
            radius_m * math.sin(theta_rad),
            0.0
        )
        
        # Build rotation matrix per manual Section 5
        R = build_rotation_matrix(C, L, U)
        
        # Create XMP content
        xmp_content = create_xmp_content(C, R)
        
        # Write file
        filename = f"stack_{i:02d}.xmp"
        filepath = os.path.join(output_dir, filename)
        with open(filepath, 'w') as f:
            f.write(xmp_content)
        
        if i % 10 == 0:
            print(f"Generated {filename}: camera at ({C[0]:.3f}, {C[1]:.3f}, {C[2]:.3f})")
    
    print(f"\nGenerated {num_cameras} XMP files in {output_dir}")
    print(f"Cameras on circle at radius {radius_m}m")
    print("All cameras looking at origin (0,0,0) with Z-up")


if __name__ == "__main__":
    generate_circle_xmps("temp/xmp_manual", num_cameras=4, radius_m=0.3)

