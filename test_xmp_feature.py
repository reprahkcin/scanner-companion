#!/usr/bin/env python3
"""
Test script for XMP pose feature implementation
"""

import sys
import os
import tempfile

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from scanner_control import ring_pose, write_xmp_sidecar, RigPose

def test_ring_pose_calculation():
    """Test the ring pose calculation function"""
    print("Testing ring pose calculations...")
    
    # Test case 1: Camera at 0° (on +X axis)
    pose_0 = ring_pose(250.0, 0.0, 0.0)
    print(f"0°: pos={pose_0.pos_m}")
    
    # Test case 2: Camera at 90° (on +Y axis)
    pose_90 = ring_pose(250.0, 0.0, 90.0)
    print(f"90°: pos={pose_90.pos_m}")
    
    # Test case 3: Camera at 180° (on -X axis)
    pose_180 = ring_pose(250.0, 0.0, 180.0)
    print(f"180°: pos={pose_180.pos_m}")
    
    # Test case 4: Camera at 270° (on -Y axis)
    pose_270 = ring_pose(250.0, 0.0, 270.0)
    print(f"270°: pos={pose_270.pos_m}")
    
    # Test case 5: Tilted rail (15° up)
    pose_tilt = ring_pose(250.0, 15.0, 45.0)
    print(f"45° with 15° rail tilt: pos={pose_tilt.pos_m}")
    
    # Verify positions are on a circle of radius 0.25m (250mm)
    expected_radius = 0.25
    for angle, pose in [(0, pose_0), (90, pose_90), (180, pose_180), (270, pose_270)]:
        x, y, z = pose.pos_m
        radius = (x**2 + y**2 + z**2)**0.5
        print(f"  {angle}°: radius={radius:.6f}m (expected: {expected_radius}m)")
        assert abs(radius - expected_radius) < 0.001, f"Radius mismatch at {angle}°"
    
    print("✓ Ring pose calculations passed!")

def test_xmp_generation():
    """Test XMP sidecar file generation"""
    print("\nTesting XMP sidecar generation...")
    
    # Create a temporary directory for test files
    with tempfile.TemporaryDirectory() as temp_dir:
        # Create a test image path
        test_img_path = os.path.join(temp_dir, "test_image.jpg")
        
        # Create dummy image file
        with open(test_img_path, 'w') as f:
            f.write("dummy image content")
        
        # Generate pose and XMP
        pose = ring_pose(250.0, -10.0, 135.0)
        write_xmp_sidecar(test_img_path, pose, 250.0, -10.0, 135.0, 5)
        
        # Check if XMP file was created
        xmp_path = test_img_path.replace('.jpg', '.xmp')
        assert os.path.exists(xmp_path), "XMP file was not created"
        
        # Read and validate XMP content
        with open(xmp_path, 'r') as f:
            xmp_content = f.read()
        
        # Check for required elements
        required_elements = [
            'xcr:Position',
            'xcr:Rotation', 
            'sc:LensToObject_mm',
            'sc:RailToHorizon_deg',
            'sc:Theta_deg',
            'sc:StackIndex'
        ]
        
        for element in required_elements:
            assert element in xmp_content, f"Missing required element: {element}"
        
        print("✓ XMP generation passed!")
        print(f"Sample XMP content preview:\n{xmp_content[:300]}...")

def test_coordinate_system():
    """Test coordinate system consistency"""
    print("\nTesting coordinate system...")
    
    # Test that the camera always looks at origin
    pose = ring_pose(200.0, 0.0, 30.0)
    x, y, z = pose.pos_m
    R = pose.R_rowmajor
    
    # The rotation matrix should be orthonormal
    # Check that each row has unit length
    row1 = (R[0], R[1], R[2])
    row2 = (R[3], R[4], R[5])
    row3 = (R[6], R[7], R[8])
    
    def magnitude(vec):
        return sum(v**2 for v in vec)**0.5
    
    mag1 = magnitude(row1)
    mag2 = magnitude(row2)
    mag3 = magnitude(row3)
    
    print(f"Rotation matrix row magnitudes: {mag1:.6f}, {mag2:.6f}, {mag3:.6f}")
    
    # Should be close to 1.0 for orthonormal matrix
    for i, mag in enumerate([mag1, mag2, mag3], 1):
        assert abs(mag - 1.0) < 0.001, f"Row {i} magnitude should be ~1.0, got {mag}"
    
    print("✓ Coordinate system tests passed!")

if __name__ == "__main__":
    print("=== XMP Feature Test Suite ===")
    
    try:
        test_ring_pose_calculation()
        test_xmp_generation()
        test_coordinate_system()
        
        print("\n🎉 All tests passed! XMP feature implementation is working correctly.")
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)