#!/usr/bin/env python3
"""
Visualize camera positions from XMP files in a top-down view.
Shows camera positions and their indices on a 2D plot (XY plane).
"""

import os
import re
import sys
import matplotlib.pyplot as plt
from pathlib import Path


def parse_xmp_position(filepath):
    """Extract position from XMP file."""
    with open(filepath, 'r') as f:
        content = f.read()
    
    pos_match = re.search(r'<xcr:Position>([\d\.\-\se]+)</xcr:Position>', content)
    if not pos_match:
        return None
    
    pos_str = pos_match.group(1).strip()
    position = tuple(float(x) for x in pos_str.split())
    return position


def visualize_cameras(xmp_directory):
    """Create a top-down visualization of camera positions."""
    xmp_files = sorted(Path(xmp_directory).glob('*.xmp'))
    
    if not xmp_files:
        print(f"No XMP files found in {xmp_directory}")
        return
    
    print(f"Found {len(xmp_files)} XMP files")
    
    # Parse all camera positions
    cameras = []
    for xmp_file in xmp_files:
        pos = parse_xmp_position(xmp_file)
        if pos:
            # Extract index from filename (e.g., stack_032.xmp -> 32)
            match = re.search(r'(\d+)', xmp_file.stem)
            index = int(match.group(1)) if match else len(cameras)
            cameras.append({
                'index': index,
                'filename': xmp_file.name,
                'x': pos[0],
                'y': pos[1],
                'z': pos[2]
            })
    
    if not cameras:
        print("No valid camera positions found")
        return
    
    # Sort by index to check ordering
    cameras.sort(key=lambda c: c['index'])
    
    # Create plot
    fig, ax = plt.subplots(figsize=(12, 12))
    
    # Plot camera positions
    x_coords = [c['x'] for c in cameras]
    y_coords = [c['y'] for c in cameras]
    indices = [c['index'] for c in cameras]
    
    # Draw circle outline
    ax.scatter(x_coords, y_coords, c='blue', s=50, alpha=0.6, label='Camera positions')
    
    # Draw lines connecting cameras in order
    ax.plot(x_coords, y_coords, 'r-', alpha=0.3, linewidth=1, label='Connection order')
    
    # Label every 8th camera to avoid clutter
    for i, cam in enumerate(cameras):
        if i % 8 == 0 or i < 5:
            ax.annotate(f"{cam['index']}", 
                       (cam['x'], cam['y']), 
                       xytext=(5, 5), 
                       textcoords='offset points',
                       fontsize=8,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7))
    
    # Mark origin
    ax.scatter([0], [0], c='red', s=200, marker='x', linewidths=3, label='Origin (0,0,0)')
    
    # Mark first camera specially
    ax.scatter([cameras[0]['x']], [cameras[0]['y']], 
              c='green', s=200, marker='^', label=f"Camera 0 (start)")
    
    # Set equal aspect ratio and labels
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title(f'Top-Down View: Camera Positions from XMP Files\n{len(cameras)} cameras')
    ax.legend()
    
    # Print ordering check
    print("\nFirst 10 cameras:")
    for i in range(min(10, len(cameras))):
        cam = cameras[i]
        print(f"  {cam['index']:3d}: {cam['filename']:20s} at ({cam['x']:7.3f}, {cam['y']:7.3f}, {cam['z']:7.3f})")
    
    print("\nEvery 32nd camera:")
    for i in range(0, len(cameras), 32):
        cam = cameras[i]
        print(f"  {cam['index']:3d}: {cam['filename']:20s} at ({cam['x']:7.3f}, {cam['y']:7.3f}, {cam['z']:7.3f})")
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        directory = sys.argv[1]
    else:
        directory = "temp/xmp_files_new"
    
    if not os.path.isdir(directory):
        print(f"Error: Directory not found: {directory}")
        sys.exit(1)
    
    print(f"Visualizing cameras from: {directory}")
    visualize_cameras(directory)
