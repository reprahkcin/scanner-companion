#!/usr/bin/env python3
"""
XMP Renaming Helper Script

This script helps rename the consolidated XMP files to match your focus-stacked images
for RealityCapture import.

Usage:
    python rename_xmp_for_rc.py [session_directory] [naming_pattern]

Example:
    python rename_xmp_for_rc.py session_20251101_143022 "specimen_angle_{angle:03.0f}.xmp"
"""

import os
import sys
import glob
import re
import shutil

def rename_xmp_files(session_dir, naming_pattern):
    """Rename XMP files from consolidated directory to match flattened image names"""
    
    xmp_dir = os.path.join(session_dir, "xmp_files")
    if not os.path.exists(xmp_dir):
        print(f"Error: XMP directory not found: {xmp_dir}")
        return False
    
    # Find all perspective XMP files
    xmp_files = glob.glob(os.path.join(xmp_dir, "perspective_*_angle_*.xmp"))
    if not xmp_files:
        print(f"Error: No perspective XMP files found in {xmp_dir}")
        return False
    
    print(f"Found {len(xmp_files)} XMP files to rename...")
    
    # Create renamed directory
    renamed_dir = os.path.join(session_dir, "xmp_renamed")
    os.makedirs(renamed_dir, exist_ok=True)
    
    renamed_count = 0
    
    for xmp_file in sorted(xmp_files):
        # Extract angle from filename
        filename = os.path.basename(xmp_file)
        match = re.search(r'angle_(\d+\.\d+)\.xmp', filename)
        
        if match:
            angle = float(match.group(1))
            
            # Generate new filename using pattern
            try:
                new_filename = naming_pattern.format(angle=angle)
                new_path = os.path.join(renamed_dir, new_filename)
                
                # Copy file with new name
                shutil.copy2(xmp_file, new_path)
                print(f"  {filename} -> {new_filename}")
                renamed_count += 1
                
            except Exception as e:
                print(f"  Error renaming {filename}: {e}")
        else:
            print(f"  Warning: Could not extract angle from {filename}")
    
    print(f"\nRenamed {renamed_count} XMP files to: {renamed_dir}")
    print(f"Copy these XMP files to the same directory as your focus-stacked images.")
    
    return True

def main():
    if len(sys.argv) < 2:
        print("Usage: python rename_xmp_for_rc.py [session_directory] [naming_pattern]")
        print()
        print("Examples:")
        print('  python rename_xmp_for_rc.py . "specimen_angle_{angle:03.0f}.xmp"')
        print('  python rename_xmp_for_rc.py session_20251101_143022 "specimen_angle_{angle:03.0f}.xmp"')
        print('  python rename_xmp_for_rc.py . "my_object_{angle:06.2f}.xmp"')
        print()
        print("Available in naming pattern:")
        print("  {angle} - the rotation angle in degrees")
        print("  Use format specifiers like {angle:03.0f} for zero-padded integers")
        print("  or {angle:06.2f} for decimal angles")
        print()
        print("Tip: Use '.' as session_directory when running from inside a session folder")
        sys.exit(1)
    
    session_dir = sys.argv[1]
    naming_pattern = sys.argv[2] if len(sys.argv) > 2 else "stacked_angle_{angle:03.0f}.xmp"
    
    # Convert relative path to absolute for better error messages
    session_dir = os.path.abspath(session_dir)
    
    if not os.path.exists(session_dir):
        print(f"Error: Session directory not found: {session_dir}")
        sys.exit(1)
    
    print("=== XMP Renaming Helper ===")
    print(f"Session directory: {session_dir}")
    print(f"Naming pattern: {naming_pattern}")
    print()
    
    success = rename_xmp_files(session_dir, naming_pattern)
    
    if success:
        print("\n✓ XMP renaming complete!")
        print("Next steps:")
        print("1. Focus stack your images using Helicon or similar")
        print("2. Copy the renamed XMP files to the same directory as your stacked images")
        print("3. Import both images and XMP files into RealityCapture")
        print("4. Enable camera priors in Alignment settings")
    else:
        print("\n❌ XMP renaming failed!")
        sys.exit(1)

if __name__ == "__main__":
    main()