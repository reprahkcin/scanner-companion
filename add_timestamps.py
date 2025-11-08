#!/usr/bin/env python3
"""
Add sequential EXIF timestamps to JPG files to enforce ordering.
This helps RealityScan recognize the correct sequence.
"""

import os
import sys
from pathlib import Path
from datetime import datetime, timedelta
from PIL import Image
import piexif

def add_sequential_timestamps(image_dir):
    """Add sequential EXIF timestamps to images."""
    # Find all JPG files, sorted
    image_files = sorted(Path(image_dir).glob('*.jpg'))
    if not image_files:
        image_files = sorted(Path(image_dir).glob('*.JPG'))
    
    if not image_files:
        print(f"No JPG files found in {image_dir}")
        return
    
    print(f"Adding sequential timestamps to {len(image_files)} images...")
    
    # Start time: Jan 1, 2024 00:00:00
    base_time = datetime(2024, 1, 1, 0, 0, 0)
    
    for i, img_file in enumerate(image_files):
        # Calculate timestamp (1 second apart)
        timestamp = base_time + timedelta(seconds=i)
        timestamp_str = timestamp.strftime("%Y:%m:%d %H:%M:%S")
        
        try:
            # Load image
            img = Image.open(img_file)
            
            # Get or create EXIF data
            try:
                exif_dict = piexif.load(img.info.get('exif', b''))
            except:
                exif_dict = {"0th": {}, "Exif": {}, "GPS": {}, "1st": {}}
            
            # Set datetime fields
            exif_dict['0th'][piexif.ImageIFD.DateTime] = timestamp_str
            exif_dict['Exif'][piexif.ExifIFD.DateTimeOriginal] = timestamp_str
            exif_dict['Exif'][piexif.ExifIFD.DateTimeDigitized] = timestamp_str
            
            # Convert to bytes
            exif_bytes = piexif.dump(exif_dict)
            
            # Save with new EXIF
            img.save(img_file, exif=exif_bytes, quality=95)
            
            if i < 5 or i % 32 == 0:
                print(f"[{i:3d}] {img_file.name:30s} → {timestamp_str}")
        
        except Exception as e:
            print(f"Error processing {img_file.name}: {e}")
    
    print(f"\n✓ Added sequential timestamps to {len(image_files)} images")
    print("Images now have 1-second intervals starting from 2024-01-01 00:00:00")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        directory = sys.argv[1]
    else:
        print("Usage: python add_timestamps.py <image_directory>")
        sys.exit(1)
    
    if not os.path.isdir(directory):
        print(f"Error: Directory not found: {directory}")
        sys.exit(1)
    
    add_sequential_timestamps(directory)
