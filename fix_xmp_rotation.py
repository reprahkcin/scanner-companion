#!/usr/bin/env python3
"""
Fix XMP camera rotations to point toward center (0,0,0) instead of away.

This script reads XMP files, extracts camera positions, recalculates rotation
matrices to make cameras look at the origin, and updates the files.
"""

import os
import re
import math
import sys
from pathlib import Path


def parse_xmp_file(filepath):
    """Extract position and rotation from XMP file."""
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Extract position
    pos_match = re.search(r'<xcr:Position>([\d\.\-\se]+)</xcr:Position>', content)
    if not pos_match:
        return None, None, content
    
    pos_str = pos_match.group(1).strip()
    position = tuple(float(x) for x in pos_str.split())
    
    # Extract rotation
    rot_match = re.search(r'xcr:Rotation="([\d\.\-\se]+)"', content)
    if not rot_match:
        return position, None, content
    
    rot_str = rot_match.group(1).strip()
    rotation = tuple(float(x) for x in rot_str.split())
    
    return position, rotation, content


def calculate_lookat_rotation(camera_pos, target=(0.0, 0.0, 0.0), up=(0.0, 0.0, 1.0)):
    """
    Calculate rotation matrix for camera at camera_pos looking at target.
    
    Returns rotation matrix as 9-tuple in row-major order for RealityCapture.
    Uses standard world-to-camera rotation.
    """
    x, y, z = camera_pos
    
    def vsub(a, b): 
        return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
    
    def vdot(a, b): 
        return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
    
    def vcross(a, b): 
        return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    
    def vnorm(a):
        m = math.sqrt(vdot(a, a))
        return (a[0]/m, a[1]/m, a[2]/m) if m > 0 else (0, 0, 1)
    
    # Forward: from camera toward target
    f = vnorm(vsub(target, camera_pos))
    
    # Handle gimbal lock when camera directly above/below target
    if abs(f[0]) < 1e-6 and abs(f[1]) < 1e-6:
        rgt = (1.0, 0.0, 0.0)
    else:
        rgt = vnorm(vcross(f, up))
    
    up_vec = vnorm(vcross(rgt, f))
    
    # World-to-camera rotation matrix
    # Rows are camera axes (right, down, forward) expressed in world coordinates
    R = (
        rgt[0], rgt[1], rgt[2],              # Camera X (right) in world
        -up_vec[0], -up_vec[1], -up_vec[2],  # Camera Y (down) in world
        f[0], f[1], f[2]                     # Camera Z (forward) in world
    )
    
    return R


def update_xmp_rotation(filepath, new_rotation, new_position=(0.0, 0.0, 0.0)):
    """Update rotation and position in XMP file."""
    with open(filepath, 'r') as f:
        content = f.read()
    
    rot_str = " ".join(f"{r}" for r in new_rotation)
    pos_str = " ".join(f"{p}" for p in new_position)
    
    # Replace rotation attribute
    new_content = re.sub(
        r'xcr:Rotation="[\d\.\-\se]+"',
        f'xcr:Rotation="{rot_str}"',
        content
    )
    
    # Replace position (set to origin - the look-at point)
    new_content = re.sub(
        r'<xcr:Position>[\d\.\-\se]+</xcr:Position>',
        f'<xcr:Position>{pos_str}</xcr:Position>',
        new_content
    )
    
    with open(filepath, 'w') as f:
        f.write(new_content)


def fix_xmp_directory(directory):
    """Process all XMP files in directory."""
    xmp_files = sorted(Path(directory).glob('*.xmp'))
    
    if not xmp_files:
        print(f"No XMP files found in {directory}")
        return
    
    print(f"Found {len(xmp_files)} XMP files")
    print("Processing...")
    
    fixed_count = 0
    for xmp_file in xmp_files:
        position, old_rotation, content = parse_xmp_file(xmp_file)
        
        if position is None:
            print(f"Warning: Could not parse position from {xmp_file.name}")
            continue
        
        # The position in the XMP is actually where the camera IS (on the circle)
        # Calculate rotation for this camera position looking at origin
        new_rotation = calculate_lookat_rotation(position, target=(0.0, 0.0, 0.0))
        
        # Update file: set Position to origin (look-at point) and update rotation
        update_xmp_rotation(xmp_file, new_rotation, new_position=(0.0, 0.0, 0.0))
        fixed_count += 1
        
        if fixed_count % 10 == 0:
            print(f"  Processed {fixed_count} files...")
    
    print(f"\nSuccessfully updated {fixed_count} XMP files")
    print("Position set to (0,0,0) - the look-at point")
    print("Cameras now positioned on circle via rotation matrix")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        directory = sys.argv[1]
    else:
        directory = "/home/pi/Desktop/scanner-companion/temp/xmp_files"
    
    if not os.path.isdir(directory):
        print(f"Error: Directory not found: {directory}")
        sys.exit(1)
    
    print(f"Fixing XMP rotations in: {directory}")
    fix_xmp_directory(directory)
