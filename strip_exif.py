#!/usr/bin/env python3
"""
EXIF Data Stripper Utility

This script removes all EXIF metadata from image files in the current directory
or a specified directory. Useful for cleaning images before uploading or when
metadata causes issues with processing software.

Usage:
    python strip_exif.py                    # Process current directory
    python strip_exif.py /path/to/images    # Process specific directory
    python strip_exif.py --recursive        # Process current directory recursively
    python strip_exif.py /path --recursive  # Process specific directory recursively

Supported formats: JPG, JPEG, PNG, TIFF, TIF

The script creates backup files with .original extension before stripping EXIF data.
"""

import os
import sys
from pathlib import Path
from PIL import Image

# Supported image extensions
IMAGE_EXTENSIONS = {'.jpg', '.jpeg', '.png', '.tiff', '.tif'}


def strip_exif_from_image(image_path: Path, create_backup: bool = True) -> bool:
    """
    Remove EXIF data from a single image file.
    
    Args:
        image_path: Path to the image file
        create_backup: Whether to create a backup with .original extension
        
    Returns:
        True if successful, False otherwise
    """
    try:
        # Open the image
        img = Image.open(image_path)
        
        # Create backup if requested
        if create_backup:
            backup_path = image_path.with_suffix(image_path.suffix + '.original')
            if not backup_path.exists():
                img.save(backup_path)
                print(f"  ✓ Backup created: {backup_path.name}")
        
        # Get image data without EXIF
        data = list(img.getdata())
        image_without_exif = Image.new(img.mode, img.size)
        image_without_exif.putdata(data)
        
        # Save the image without EXIF data
        # For JPEG, explicitly avoid saving EXIF
        if image_path.suffix.lower() in {'.jpg', '.jpeg'}:
            image_without_exif.save(image_path, "JPEG", quality=95, optimize=True)
        elif image_path.suffix.lower() == '.png':
            image_without_exif.save(image_path, "PNG", optimize=True)
        elif image_path.suffix.lower() in {'.tiff', '.tif'}:
            image_without_exif.save(image_path, "TIFF")
        else:
            image_without_exif.save(image_path)
        
        print(f"  ✓ EXIF stripped: {image_path.name}")
        return True
        
    except Exception as e:
        print(f"  ✗ Error processing {image_path.name}: {e}")
        return False


def process_directory(directory: Path, recursive: bool = False) -> tuple[int, int]:
    """
    Process all images in a directory.
    
    Args:
        directory: Directory path to process
        recursive: Whether to process subdirectories recursively
        
    Returns:
        Tuple of (successful_count, failed_count)
    """
    success_count = 0
    fail_count = 0
    
    # Get all image files
    if recursive:
        image_files = [
            f for f in directory.rglob('*')
            if f.is_file() and f.suffix.lower() in IMAGE_EXTENSIONS
        ]
    else:
        image_files = [
            f for f in directory.iterdir()
            if f.is_file() and f.suffix.lower() in IMAGE_EXTENSIONS
        ]
    
    if not image_files:
        print(f"No image files found in {directory}")
        return 0, 0
    
    print(f"\nFound {len(image_files)} image(s) to process\n")
    
    # Process each image
    for image_file in sorted(image_files):
        print(f"Processing: {image_file.relative_to(directory)}")
        if strip_exif_from_image(image_file):
            success_count += 1
        else:
            fail_count += 1
        print()
    
    return success_count, fail_count


def main():
    """Main entry point for the script."""
    # Parse command line arguments
    args = sys.argv[1:]
    directory = Path.cwd()
    recursive = False
    
    # Check for --recursive flag
    if '--recursive' in args or '-r' in args:
        recursive = True
        args = [arg for arg in args if arg not in ('--recursive', '-r')]
    
    # Get directory path if provided
    if args:
        directory = Path(args[0])
        if not directory.exists():
            print(f"Error: Directory '{directory}' does not exist")
            sys.exit(1)
        if not directory.is_dir():
            print(f"Error: '{directory}' is not a directory")
            sys.exit(1)
    
    # Display configuration
    print("=" * 60)
    print("EXIF Data Stripper Utility")
    print("=" * 60)
    print(f"Directory: {directory.absolute()}")
    print(f"Recursive: {'Yes' if recursive else 'No'}")
    print(f"Supported formats: {', '.join(sorted(IMAGE_EXTENSIONS))}")
    print("=" * 60)
    
    # Confirm action
    try:
        response = input("\nProceed with EXIF stripping? (y/n): ").strip().lower()
        if response not in ('y', 'yes'):
            print("Operation cancelled.")
            sys.exit(0)
    except KeyboardInterrupt:
        print("\nOperation cancelled.")
        sys.exit(0)
    
    # Process images
    success_count, fail_count = process_directory(directory, recursive)
    
    # Display results
    print("=" * 60)
    print("Results:")
    print(f"  Successfully processed: {success_count}")
    print(f"  Failed: {fail_count}")
    print(f"  Total: {success_count + fail_count}")
    print("=" * 60)
    
    if success_count > 0:
        print("\n✓ EXIF data has been stripped from images")
        print("  Original files backed up with .original extension")
    
    if fail_count > 0:
        sys.exit(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nOperation cancelled by user.")
        sys.exit(0)
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        sys.exit(1)
