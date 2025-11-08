#!/usr/bin/env python3
"""
Generate simple test XMP files - just 4 cameras at cardinal directions.
This should create a clear pattern to debug coordinate system issues.
"""

import math
import os


def create_xmp_content(position, rotation_matrix):
    """Create XMP file content."""
    pos_str = f"{position[0]:.6f} {position[1]:.6f} {position[2]:.6f}"
    rot_str = " ".join(f"{x:.6f}" for x in rotation_matrix)
    
    return f"""<x:xmpmeta xmlns:x="adobe:ns:meta/">
  <rdf:RDF xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#">
    <rdf:Description xcr:Version="3" xcr:PosePrior="initial" xcr:Coordinates="absolute"
       xcr:DistortionModel="brown3" xcr:FocalLength35mm="50"
       xcr:Skew="0" xcr:AspectRatio="1" xcr:PrincipalPointU="0"
       xcr:PrincipalPointV="0" xcr:CalibrationPrior="initial"
       xcr:CalibrationGroup="-1" xcr:DistortionGroup="-1" xcr:InTexturing="1"
       xcr:InMeshing="1" xmlns:xcr="http://www.capturingreality.com/ns/xcr/1.1#">
      <xcr:Rotation>{rot_str}</xcr:Rotation>
      <xcr:Position>{pos_str}</xcr:Position>
      <xcr:DistortionCoeficients>0 0 0 0 0 0</xcr:DistortionCoeficients>
    </rdf:Description>
  </rdf:RDF>
</x:xmpmeta>"""


def calculate_lookat_rotation(camera_pos, target_pos=(0, 0, 0), world_up=(0, 0, 1)):
    """
    Calculate rotation matrix for camera looking at target.
    
    Rotation matrix R transforms world coords to camera coords.
    Camera coordinate system:
    - +X right
    - +Y down
    - +Z forward (looking direction)
    
    Args:
        camera_pos: (x, y, z) camera position in world
        target_pos: (x, y, z) point camera looks at
        world_up: (x, y, z) world up vector
    
    Returns:
        tuple of 9 floats (rotation matrix in row-major order)
    """
    # Forward: from camera toward target
    fx = target_pos[0] - camera_pos[0]
    fy = target_pos[1] - camera_pos[1]
    fz = target_pos[2] - camera_pos[2]
    f_len = math.sqrt(fx**2 + fy**2 + fz**2)
    fwd = (fx/f_len, fy/f_len, fz/f_len)
    
    # Right: forward × world_up
    rx = fwd[1]*world_up[2] - fwd[2]*world_up[1]
    ry = fwd[2]*world_up[0] - fwd[0]*world_up[2]
    rz = fwd[0]*world_up[1] - fwd[1]*world_up[0]
    r_len = math.sqrt(rx**2 + ry**2 + rz**2)
    right = (rx/r_len, ry/r_len, rz/r_len)
    
    # Camera up: right × forward
    ux = right[1]*fwd[2] - right[2]*fwd[1]
    uy = right[2]*fwd[0] - right[0]*fwd[2]
    uz = right[0]*fwd[1] - right[1]*fwd[0]
    up = (ux, uy, uz)
    
    # Rotation matrix: rows are camera axes in world coords
    # Row 0: camera +X (right) in world
    # Row 1: camera +Y (down = -up) in world
    # Row 2: camera +Z (forward) in world
    R = (
        right[0], right[1], right[2],
        -up[0], -up[1], -up[2],
        fwd[0], fwd[1], fwd[2]
    )
    
    return R


def generate_test_xmps():
    """Generate 4 test XMP files at cardinal directions."""
    output_dir = "temp/test_4cameras"
    os.makedirs(output_dir, exist_ok=True)
    
    # Use meters for positions (RealityCapture standard)
    radius_m = 0.3  # 30cm radius - small test scene
    
    # 4 cameras at cardinal directions in XY plane (Z=0)
    cameras = [
        ("stack_00", (radius_m, 0, 0), "East (+X)"),
        ("stack_01", (0, radius_m, 0), "North (+Y)"),
        ("stack_02", (-radius_m, 0, 0), "West (-X)"),
        ("stack_03", (0, -radius_m, 0), "South (-Y)"),
    ]
    
    print(f"Generating 4 test cameras at radius {radius_m}m")
    print(f"Coordinate system: X=right, Y=forward, Z=up")
    print(f"All cameras on horizontal circle (XY plane, Z=0)\n")
    
    for filename, position, direction in cameras:
        # Calculate rotation to look at origin
        rotation = calculate_lookat_rotation(position)
        
        # Create XMP content
        xmp_content = create_xmp_content(position, rotation)
        
        # Write file
        filepath = os.path.join(output_dir, f"{filename}.xmp")
        with open(filepath, 'w') as f:
            f.write(xmp_content)
        
        print(f"{filename}.xmp: {direction}")
        print(f"  Position: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})")
        print(f"  Rotation: [{', '.join(f'{r:.3f}' for r in rotation)}]\n")
    
    print(f"Files written to: {output_dir}")
    print("\nExpected result: 4 cameras arranged in a square on XY plane,")
    print("all pointing toward origin at (0,0,0)")


if __name__ == "__main__":
    generate_test_xmps()
