#!/usr/bin/env python3
"""
Generate XMP files for existing JPG images with matching basenames.
Ensures proper pairing between images and XMP files for RealityScan.
"""

import math
import os
import sys
from pathlib import Path


def create_xmp_content(position, rotation_matrix):
    """Create XMP file content."""
    # Round positions to 6 decimal places
    pos_str = f"{position[0]:.6f} {position[1]:.6f} {position[2]:.6f}"
    # Round rotation values to 10 decimal places
    rot_str = " ".join(f"{x:.10f}" for x in rotation_matrix)
    
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
    """Build rotation matrix for camera at C looking at L with up U."""
    # f points FROM origin TO camera (outward)
    f = normalize((C[0] - L[0], C[1] - L[1], C[2] - L[2]))
    
    # Right vector
    s = normalize(cross(f, U))
    
    # True up
    u = cross(s, f)
    
    # Row 1 = -s AND Row 2 = -u to flip model upright (180° roll)
    # Row 3 = -f (cameras face inward) ✓ WORKING IN TEST 8
    R = (
        -s[0], -s[1], -s[2],
        -u[0], -u[1], -u[2],
        -f[0], -f[1], -f[2]
    )
    
    return R


def generate_xmps_for_images(image_dir, radius_m=4.9):
    """
    Generate XMP files for existing JPG images.
    XMP basenames will exactly match image basenames.
    
    Args:
        image_dir: Directory containing JPG images
        radius_m: Radius of camera circle in meters
    """
    # Find all JPG files, sorted alphabetically
    image_files = sorted(Path(image_dir).glob('*.jpg'))
    if not image_files:
        image_files = sorted(Path(image_dir).glob('*.JPG'))
    
    if not image_files:
        print(f"No JPG files found in {image_dir}")
        return
    
    num_images = len(image_files)
    print(f"Found {num_images} JPG images")
    print(f"Generating XMP files with matching basenames...\n")
    
    L = (0.0, 0.0, 0.0)  # Look-at point (origin)
    U = (0.0, 0.0, 1.0)  # World up (Z-up)
    
    for i, img_file in enumerate(image_files):
        # Calculate angle for this camera
        # NEGATIVE angle because object rotated CCW (turntable), not camera rotating CW
        theta_deg = -(i / num_images) * 360.0
        theta_rad = math.radians(theta_deg)
        
        # Use base radius only - no adjustments
        radius_adjusted = radius_m
        
        # Camera position on horizontal circle at Z=0
        C = (
            radius_adjusted * math.cos(theta_rad),
            radius_adjusted * math.sin(theta_rad),
            0.0
        )
        
        # Build rotation matrix
        R = build_rotation_matrix(C, L, U)
        
        # Create XMP content
        xmp_content = create_xmp_content(C, R)
        
        # Write XMP with EXACT same basename as image
        xmp_path = img_file.with_suffix('.xmp')
        with open(xmp_path, 'w') as f:
            f.write(xmp_content)
        
        # Touch both files to sync timestamps
        os.utime(img_file, None)
        os.utime(xmp_path, None)
        
        if i < 5 or i % 32 == 0:
            print(f"[{i:3d}] {img_file.name:30s} → {xmp_path.name:30s}")
            print(f"      angle={theta_deg:6.1f}° pos=({C[0]:6.3f}, {C[1]:6.3f}, {C[2]:6.3f})")
    
    print(f"\n✓ Generated {num_images} XMP files")
    print(f"✓ Each XMP basename matches its JPG")
    print(f"✓ All file timestamps updated")
    print(f"\nReady for RealityScan import from: {image_dir}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        directory = sys.argv[1]
    else:
        print("Usage: python generate_xmp_for_images.py <image_directory>")
        print("\nExample: python generate_xmp_for_images.py /path/to/images")
        sys.exit(1)
    
    if not os.path.isdir(directory):
        print(f"Error: Directory not found: {directory}")
        sys.exit(1)
    
    generate_xmps_for_images(directory)
