#!/usr/bin/env python3
"""
Test the XMP consolidation feature
"""

import os
import sys
import tempfile
import shutil

# Add parent directory to path to import scanner_control
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from scanner_control import write_xmp_sidecar, ring_pose

def test_xmp_consolidation():
    """Test the XMP consolidation workflow"""
    print("Testing XMP consolidation workflow...")
    
    # Create a temporary session directory structure
    with tempfile.TemporaryDirectory() as temp_dir:
        session_dir = os.path.join(temp_dir, "test_session")
        os.makedirs(session_dir)
        
        # Simulate creating stack directories with XMP files
        perspectives = 4  # 4 angles: 0°, 90°, 180°, 270°
        angle_step = 360.0 / perspectives
        
        xmp_files_created = []
        
        for perspective in range(perspectives):
            angle = perspective * angle_step
            stack_dir = os.path.join(session_dir, f"stack_{perspective:02d}")
            os.makedirs(stack_dir)
            
            # Create some dummy image files
            for shot in range(3):  # 3 focus slices per angle
                img_filename = f"stack_{perspective:02d}_shot_{shot:03d}_angle_{angle:06.2f}.jpg"
                img_path = os.path.join(stack_dir, img_filename)
                with open(img_path, 'w') as f:
                    f.write("dummy image content")
            
            # Generate XMP for this stack
            pose = ring_pose(250.0, 0.0, angle)
            xmp_filename = f"stack_{perspective:02d}_angle_{angle:06.2f}.xmp"
            xmp_path = os.path.join(stack_dir, xmp_filename)
            
            write_xmp_sidecar(xmp_path.replace('.xmp', '.jpg'), pose, 
                            250.0, 0.0, angle, perspective)
            
            xmp_files_created.append(xmp_path)
            print(f"  Created: {xmp_filename}")
        
        # Now simulate the consolidation process
        xmp_dir = os.path.join(session_dir, "xmp_files")
        os.makedirs(xmp_dir, exist_ok=True)
        
        # Copy all XMP files to consolidated directory
        xmp_count = 0
        for perspective in range(perspectives):
            angle = perspective * angle_step
            stack_dir = os.path.join(session_dir, f"stack_{perspective:02d}")
            
            # Find XMP file in stack directory
            xmp_filename = f"stack_{perspective:02d}_angle_{angle:06.2f}.xmp"
            source_xmp = os.path.join(stack_dir, xmp_filename)
            
            if os.path.exists(source_xmp):
                # Create consolidated filename for RealityCapture import
                consolidated_filename = f"perspective_{perspective:02d}_angle_{angle:06.2f}.xmp"
                dest_xmp = os.path.join(xmp_dir, consolidated_filename)
                
                # Copy XMP file
                shutil.copy2(source_xmp, dest_xmp)
                xmp_count += 1
                print(f"  Consolidated: {consolidated_filename}")
        
        print(f"\n✓ Successfully consolidated {xmp_count} XMP files")
        
        # Verify consolidation
        consolidated_files = os.listdir(xmp_dir)
        expected_files = [f"perspective_{i:02d}_angle_{i * angle_step:06.2f}.xmp" 
                         for i in range(perspectives)]
        
        for expected in expected_files:
            assert expected in consolidated_files, f"Missing consolidated file: {expected}"
        
        print(f"✓ All expected files present in consolidated directory")
        
        # Test the helper script functionality
        print(f"\nTesting XMP renaming helper...")
        
        # Simulate renaming
        renamed_dir = os.path.join(session_dir, "xmp_renamed_test")
        os.makedirs(renamed_dir, exist_ok=True)
        
        naming_pattern = "specimen_angle_{angle:03.0f}.xmp"
        
        for consolidated_file in consolidated_files:
            # Extract angle from filename
            import re
            match = re.search(r'angle_(\d+\.\d+)\.xmp', consolidated_file)
            if match:
                angle = float(match.group(1))
                new_filename = naming_pattern.format(angle=angle)
                
                source_path = os.path.join(xmp_dir, consolidated_file)
                dest_path = os.path.join(renamed_dir, new_filename)
                shutil.copy2(source_path, dest_path)
                print(f"  Renamed: {consolidated_file} -> {new_filename}")
        
        renamed_files = os.listdir(renamed_dir)
        print(f"✓ Successfully renamed {len(renamed_files)} XMP files for RealityCapture")
        
        return True

if __name__ == "__main__":
    print("=== XMP Consolidation Test ===")
    
    try:
        test_xmp_consolidation()
        print("\n🎉 XMP consolidation test passed!")
        
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)