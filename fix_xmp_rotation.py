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


def calculate_lookat_rotation(camera_pos, target_pos, world_up):
    """
    Calculate rotation matrix for camera at camera_pos looking at target_pos.
    
    Per RealityCapture documentation:
    x = [R t] * X
    where R transforms world coordinates to camera coordinates.
    
    Camera coordinate system (standard computer vision):
    - +X right
    - +Y down
    - +Z forward (looking direction)
    
    Args:
        camera_pos: tuple (x, y, z) - camera position in world coordinates
        target_pos: tuple (x, y, z) - point camera is looking at
        world_up: tuple (x, y, z) - world up vector (usually (0, 0, 1) for Z-up)
    
    Returns:
        tuple of 9 floats - rotation matrix in row-major order
    """
    import math
    
    # Convert to arrays for easier math
    cam = camera_pos
    tgt = target_pos
    up = world_up
    
    # Calculate forward direction in world space (from camera toward target)
    fx = tgt[0] - cam[0]
    fy = tgt[1] - cam[1]
    fz = tgt[2] - cam[2]
    f_len = math.sqrt(fx*fx + fy*fy + fz*fz)
    fwd_world = (fx/f_len, fy/f_len, fz/f_len)
    
    # Calculate right direction in world space (cross product of forward and world up)
    rx = fwd_world[1]*up[2] - fwd_world[2]*up[1]
    ry = fwd_world[2]*up[0] - fwd_world[0]*up[2]
    rz = fwd_world[0]*up[1] - fwd_world[1]*up[0]
    r_len = math.sqrt(rx*rx + ry*ry + rz*rz)
    right_world = (rx/r_len, ry/r_len, rz/r_len)
    
    # Calculate camera up direction in world space (cross product of right and forward)
    ux = right_world[1]*fwd_world[2] - right_world[2]*fwd_world[1]
    uy = right_world[2]*fwd_world[0] - right_world[0]*fwd_world[2]
    uz = right_world[0]*fwd_world[1] - right_world[1]*fwd_world[0]
    up_world = (ux, uy, uz)
    
    # Build rotation matrix R that transforms world coords to camera coords
    # Each row of R represents where a camera axis points in world coordinates:
    # Row 0: camera X-axis (right) in world coords
    # Row 1: camera Y-axis (down, so negate up) in world coords  
    # Row 2: camera Z-axis (forward) in world coords
    R = (
        right_world[0], right_world[1], right_world[2],
        -up_world[0], -up_world[1], -up_world[2],
        fwd_world[0], fwd_world[1], fwd_world[2]
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
    import re
    
    # Natural sort key to handle numeric filenames correctly
    def natural_sort_key(path):
        return [int(text) if text.isdigit() else text.lower() 
                for text in re.split(r'(\d+)', str(path.name))]
    
    xmp_files = sorted(Path(directory).glob('*.xmp'), key=natural_sort_key)
    
    if not xmp_files:
        print(f"No XMP files found in {directory}")
        return
    
    print(f"Found {len(xmp_files)} XMP files")
    print("Processing...")
    
    # Configuration: assume 128 images in a 360° circle
    total_files = len(xmp_files)
    distance_mm = 4900.0  # Camera distance from origin
    
    fixed_count = 0
    for i, xmp_file in enumerate(xmp_files):
        # Calculate camera position based on index
        # Cameras arranged in horizontal circle (XY plane)
        theta_deg = (i / total_files) * 360.0
        theta_rad = math.radians(theta_deg)
        
        r = distance_mm  # Keep in millimeters for XMP
        x = r * math.cos(theta_rad)
        y = r * math.sin(theta_rad)
        z = 0.0  # Horizontal circle
        
        camera_position_mm = (x, y, z)
        camera_position_m = (x/1000.0, y/1000.0, z/1000.0)
        
        # Calculate rotation for this camera position looking at origin
        world_up = (0.0, 0.0, 1.0)  # Z-up world
        new_rotation = calculate_lookat_rotation(camera_position_m, (0.0, 0.0, 0.0), world_up)
        
        # Update file: Position is camera location, rotation makes it look at origin
        update_xmp_rotation(xmp_file, new_rotation, new_position=camera_position_mm)
        fixed_count += 1
        
        if fixed_count % 10 == 0:
            print(f"  Processed {fixed_count} files...")
    
    print(f"\nSuccessfully updated {fixed_count} XMP files")
    print("Position set to camera locations on horizontal circle (XY plane)")
    print("Rotation matrices orient cameras to look at origin with tops pointing +Z")


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
