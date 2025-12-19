#!/usr/bin/env python3
"""3D Scanner Control Panel v1.0

A comprehensive control application for a dual-motor 3D photogrammetry scanner 
using Raspberry Pi and Arduino. Provides automated calibration, focus interpolation,
and multi-stack capture capabilities.

This application controls:
    - Rotation motor (Motor 1) for 360° specimen positioning
    - Linear rail motor (Motor 2) for focus adjustment
    - Pi Camera for image capture with manual settings
    - Arduino communication via serial commands

Features:
    - Manual motor control with real-time position tracking
    - Cardinal point calibration wizard (0°, 90°, 180°, 270°)
    - Automatic focus interpolation for any angle
    - Multi-stack capture with configurable parameters
    - Organized file output with metadata
    - Camera settings control (exposure, brightness)

Author: Generated for 3D Scanner Project
Date: September 2025
Version: 1.1
"""

import tkinter as tk
from tkinter import ttk
from tkinter import messagebox, filedialog
import threading
import time
import cv2
import json
import os
from datetime import datetime

# Enable camera imports, but don't use them yet
try:
    from picamera2 import Picamera2, Preview
    PICAMERA2_OK = True
except ImportError:
    PICAMERA2_OK = False

try:
    from PIL import Image, ImageTk
    PIL_OK = True
except ImportError:
    PIL_OK = False

PICAMERA2_AVAILABLE = PICAMERA2_OK and PIL_OK

import serial
import math
from dataclasses import dataclass
from xml.sax.saxutils import escape

# ──────────────────────────────────────────────────────────────────────────────
# XMP POSE CALCULATION
# ──────────────────────────────────────────────────────────────────────────────

# Hard-verified RealityCapture intrinsics and priors (do not change unless re-validated)
VERIFIED_DISTORTION_MODEL = "brown3"
VERIFIED_DISTORTION_COEFFS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
VERIFIED_FOCAL_LENGTH_35MM = 50.0
VERIFIED_SKEW = 0.0
VERIFIED_ASPECT_RATIO = 1.0
VERIFIED_PRINCIPAL_POINT_U = 0.0
VERIFIED_PRINCIPAL_POINT_V = 0.0
VERIFIED_POSE_PRIOR = "locked"
VERIFIED_CALIB_PRIOR = "exact"

@dataclass
class RigPose:
    """Camera pose for XMP export"""
    pos_m: tuple[float, float, float]     # (x,y,z) position in meters
    R_rowmajor: tuple[float, ...]         # 9 numbers, row-major 3x3 rotation matrix

def ring_pose(distance_mm: float, rail_deg: float, theta_deg: float) -> RigPose:
    """Calculate camera pose on a tilted ring around object at origin.
    
    Args:
        distance_mm: Distance from lens entrance pupil to object center
        rail_deg: Rail tilt angle (positive = camera pitched up from horizontal)
        theta_deg: Angle around the object (0° = +X axis, 90° = +Y axis)
        
    Returns:
        RigPose with camera position and look-at-origin rotation matrix
    """
    # Object center at origin. X right, Y forward, Z up in rig space.
    r = distance_mm / 1000.0  # Convert to meters
    th = math.radians(theta_deg)
    a = math.radians(rail_deg)
    
    # Camera center in world coordinates on tilted ring
    x = r * math.cos(th) * math.cos(a)
    y = r * math.sin(th) * math.cos(a)  
    z = r * math.sin(a)

    # Build a "look-at origin" rotation matrix
    C = (x, y, z)
    tgt = (0.0, 0.0, 0.0)  # Looking at object center
    upW = (0.0, 0.0, 1.0)  # World up vector (Z-up per RealityCapture standard)

    def vsub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
    def vdot(a, b): return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]
    def vcross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    def vnorm(a):
        m = math.sqrt(vdot(a, a))
        return (a[0]/m, a[1]/m, a[2]/m) if m > 0 else (0, 0, 1)

    # Follow reference document exactly: f = normalize(L - C)
    # f points FROM camera TO look-at (inward toward origin)
    f = vnorm(vsub(tgt, C))  # Inward vector
    
    # Handle gimbal lock when camera is directly above/below
    if abs(f[0]) < 1e-6 and abs(f[1]) < 1e-6:
        # Camera pointing straight up or down
        rgt = (1.0, 0.0, 0.0)  # Arbitrary right vector
    else:
        # Use right = cross(worldUp, forward) for RC convention
        rgt = vnorm(vcross(upW, f))  # Right

    # Up = cross(forward, right)
    up = vcross(f, rgt)  # Up (already unit length from cross of two unit vectors)

    # Adopt empirically validated RealityCapture convention (variant 16):
    # rows = [right, up, forward]
    R9 = (rgt[0], rgt[1], rgt[2],
        up[0], up[1], up[2],
        f[0], f[1], f[2])
    
    return RigPose((x, y, z), R9)

def spherical_pose(radius_mm: float, azimuth_deg: float, elevation_deg: float) -> RigPose:
    """Calculate camera pose on a sphere around object at origin.
    
    This generates poses for spherical photogrammetry where the camera orbits
    the specimen at a constant distance on a virtual sphere.
    
    Args:
        radius_mm: Distance from camera to specimen center (sphere radius)
        azimuth_deg: Horizontal angle around specimen (0° = +X axis, 90° = +Y axis)
        elevation_deg: Vertical angle (-45° = below, 0° = horizontal, +45° = above)
        
    Returns:
        RigPose with camera position and look-at-origin rotation matrix
        
    Coordinate System:
        World: X-right, Y-forward, Z-up (RealityCapture standard)
        Camera: Looks down -Z axis, Y-up, X-right (OpenCV/RC standard)
    """
    # Convert angles to radians
    azimuth = math.radians(azimuth_deg)
    elevation = math.radians(elevation_deg)
    r = radius_mm / 1000.0  # Convert to meters
    
    # Calculate camera position on sphere (spherical → Cartesian)
    # Elevation: 0° = horizontal (XY plane), +90° = straight up (+Z), -90° = straight down (-Z)
    x = r * math.cos(elevation) * math.cos(azimuth)
    y = r * math.cos(elevation) * math.sin(azimuth)
    z = r * math.sin(elevation)
    
    # Build rotation matrix for camera looking at origin
    camera_pos = (x, y, z)
    target = (0.0, 0.0, 0.0)  # Specimen at origin
    world_up = (0.0, 0.0, 1.0)  # Z-up world
    
    # Helper functions
    def vsub(a, b): return (a[0]-b[0], a[1]-b[1], a[2]-b[2])
    def vdot(a, b): return a[0]*b[0] + a[1]*b[1] + a[2]*b[2]
    def vcross(a, b): return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    def vnorm(a):
        m = math.sqrt(vdot(a, a))
        return (a[0]/m, a[1]/m, a[2]/m) if m > 0 else (0, 0, 1)
    
    # Forward vector: from camera toward target (inward)
    forward = vnorm(vsub(target, camera_pos))
    
    # Handle gimbal lock when camera is directly above/below specimen
    if abs(forward[0]) < 1e-6 and abs(forward[1]) < 1e-6:
        # Camera pointing straight up or down - use arbitrary right vector based on azimuth
        right = (math.cos(azimuth + math.pi/2), math.sin(azimuth + math.pi/2), 0.0)
        right = vnorm(right)
    else:
        # Standard case: right = world_up × forward
        right = vnorm(vcross(world_up, forward))
    
    # Up vector: forward × right (camera Y-axis points up in image)
    up = vcross(forward, right)
    
    # Build rotation matrix (row-major: transforms world → camera coordinates)
    # RealityCapture expects: R[world_point - camera_pos] = camera_coords
    # Camera looks down -Z, so forward vector should map to camera +Z
    # But we want specimen (ahead) to have positive Z in camera space
    # Current convention from ring_pose (validated as geometrically correct):
    R9 = (right[0], right[1], right[2],
          up[0], up[1], up[2],
          forward[0], forward[1], forward[2])
    
    return RigPose(camera_pos, R9)

def write_xmp_sidecar(img_path: str, pose: RigPose, 
                      lens_to_object_mm: float, rail_to_horizon_deg: float,
                      theta_deg: float, stack_index: int,
                      focal_length_35mm: float = 50.0,
                      distortion_model: str = "brown3",
                      skew: float = 0.0,
                      aspect_ratio: float = 1.0,
                      principal_point_u: float = 0.0,
                      principal_point_v: float = 0.0,
                      distortion_coefficients: tuple = (-0.1, 0.1, 0.0, 0.0, 0.0, 0.0),
                      calibration_group: int = -1,
                      distortion_group: int = -1,
                      in_texturing: int = 1,
                      in_meshing: int = 1,
                      position_scale: float = 1.0,
                      pose_prior: str = "locked",
                      calibration_prior: str = "exact") -> None:
    """Write XMP sidecar file with camera pose data in RealityCapture format.
    
    Generates XMP metadata files conforming to RealityCapture's XMP specification.
    See: https://rshelp.capturingreality.com/en-US/tools/xmpalign.htm
    
    Args:
        img_path: Path to the image file
        pose: Camera pose data (position and rotation matrix)
        lens_to_object_mm: Distance from lens to object center (for metadata)
        rail_to_horizon_deg: Rail tilt angle (for metadata)
        theta_deg: Rotation angle around object (for metadata)
        stack_index: Stack/perspective index (for metadata)
        focal_length_35mm: 35mm equivalent focal length
        distortion_model: Lens distortion model (e.g., "brown3", "division")
        skew: Camera skew parameter
        aspect_ratio: Pixel aspect ratio
        principal_point_u: Principal point U coordinate (normalized)
        principal_point_v: Principal point V coordinate (normalized)
        distortion_coefficients: Tuple of 6 distortion coefficients (k1-k6)
        calibration_group: Calibration group ID (-1 for independent)
        distortion_group: Distortion group ID (-1 for independent)
        in_texturing: Include in texturing phase (1=yes, 0=no)
        in_meshing: Include in meshing phase (1=yes, 0=no)
        position_scale: Scale factor for position values (e.g., 1000 for macro)
    
    Note:
        RealityCapture requires specific attribute spelling: "DistortionCoeficients"
        (not "Coefficients"). The rotation matrix must be 9 space-separated values
        in row-major order. Per RealityCapture XMP Camera Math, Rotation and Position
        are encoded as child elements, not attributes.
    """
    XCR_NS = 'http://www.capturingreality.com/ns/xcr/1.1#'
    
    # Per RealityCapture documentation: Position is the CAMERA location in world coordinates
    # The rotation matrix R transforms world coordinates to camera coordinates
    # See: https://dev.epicgames.com/community/learning/knowledge-base/vzwB/realityscan-realitycapture-xmp-camera-math
    
    # Camera position in world coordinates (meters)
    P = pose.pos_m
    
    # Apply position scale and format to match verified good set exactly (6 decimal places)
    position_str = f"{P[0] * position_scale:.6f} {P[1] * position_scale:.6f} {P[2] * position_scale:.6f}"

    R = pose.R_rowmajor

    # Format distortion coefficients to match verified good set (integers when zero)
    dist_str = " ".join("0" if abs(c) < 1e-9 else str(c) for c in distortion_coefficients)

    # Format rotation matrix to match verified good set exactly (10 decimal places)
    rotation_str = " ".join(f"{r:.10f}" for r in R)

    # Build XMP content to match verified good set format exactly
    xmp_content = f'''<x:xmpmeta xmlns:rdf="http://www.w3.org/1999/02/22-rdf-syntax-ns#" xmlns:x="adobe:ns:meta/" xmlns:xcr="{XCR_NS}">
  <rdf:RDF>
    <rdf:Description xcr:Version="3" xcr:PosePrior="{escape(pose_prior)}" xcr:CalibrationPrior="{escape(calibration_prior)}" xcr:Coordinates="absolute" xcr:DistortionModel="{escape(str(distortion_model))}" xcr:DistortionCoeficients="{escape(dist_str)}" xcr:FocalLength35mm="{focal_length_35mm:.0f}" xcr:Skew="{skew:.0f}" xcr:AspectRatio="{aspect_ratio:.0f}" xcr:PrincipalPointU="{principal_point_u:.0f}" xcr:PrincipalPointV="{principal_point_v:.0f}" xcr:CalibrationGroup="{calibration_group}" xcr:DistortionGroup="{distortion_group}" xcr:InTexturing="{in_texturing}" xcr:InMeshing="{in_meshing}">
      <xcr:Rotation>{rotation_str}</xcr:Rotation>
      <xcr:Position>{position_str}</xcr:Position>
    </rdf:Description>
  </rdf:RDF>
</x:xmpmeta>'''
    
    # Create XMP file path (replace extension with .xmp)
    xmp_path = img_path.rsplit('.', 1)[0] + '.xmp'
    
    with open(xmp_path, 'w', encoding='utf-8') as f:
        f.write(xmp_content)

# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION SECTION
# ──────────────────────────────────────────────────────────────────────────────

SERIAL_PORT       = "/dev/ttyACM0"  # or "/dev/serial/by-id/..." 
SERIAL_BAUDRATE   = 115200            # Match motor_control_refactor.ino baudrate
CAMERA_RESOLUTION = (320, 240)       # preview size (smaller for safety)
JOG_STEP_DELAY    = 0.05             # seconds between steps during jog
PREVIEW_COLOR_SWAP = True            # swap BGR->RGB for preview if colors look wrong

# Common resolution presets (WxH) for preview/capture
RESOLUTION_PRESETS = [
    "320x240", "640x480", "1280x720", "1920x1080", "2028x1520", "2592x1944",
    "3280x2464", "3840x2160", "4056x3040"
]

# Capture defaults
DEFAULT_OUTPUT_DIR = os.path.expanduser("~/Desktop/scanner_captures")
DEFAULT_STACKS = 5
DEFAULT_SHOTS_PER_STACK = 72  # 5-degree increments for 360°
DEFAULT_SETTLE_DELAY = 1.0    # seconds to wait after movement before capture

# Cardinal angles for calibration
CALIBRATION_ANGLES = [0, 90, 180, 270]

# ──────────────────────────────────────────────────────────────────────────────

class ScannerGUI(tk.Tk):
    """Main GUI application for 3D Scanner Control Panel.
    
    This class provides a comprehensive interface for controlling a dual-motor
    3D photogrammetry scanner. It includes manual controls, calibration wizard,
    automated capture sequences, and camera settings.
    
    The application uses a tabbed interface with four main sections:
        - Manual Control: Direct motor and camera control
        - Calibration: Step-by-step focus calibration wizard
        - Capture: Automated multi-stack scanning configuration
        - Camera Settings: Manual camera parameter adjustment
    
    Attributes:
        ser (serial.Serial): Arduino serial connection
        camera (Picamera2): Pi camera instance
        preview_on (bool): Camera preview state
        motor1_position_deg (float): Current rotation motor position in degrees
        motor2_position_mm (float): Current linear motor position in millimeters
        calibration_data (dict): Focus positions at cardinal angles
        is_calibrated (bool): Whether calibration is complete
        capture_running (bool): Whether capture sequence is active
        
    Example:
        app = ScannerGUI()
        app.mainloop()
    """
    
    def __init__(self):
        """Initialize the Scanner GUI application.
        
        Sets up the main window, establishes serial connection to Arduino,
        initializes camera interface, creates all GUI components, and
        prepares the application for user interaction.
        
        Raises:
            serial.SerialException: If Arduino connection fails (non-fatal)
        """
        super().__init__()
        self.title("3D Scanner Control Panel - v1.0")
        self.minsize(800, 600)
        self.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # Simple always-on-top - don't add aggressive focus handling
        self.wm_attributes('-topmost', True)
        
        # Disable focus maintenance - it causes more problems than it solves
        self._focus_maintenance_active = False
        # self.after(100, self._maintain_focus)

        # === Serial Setup ===
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0.1)
            self.status_var = tk.StringVar(value="Connected to Arduino")
        except serial.SerialException as e:
            self.status_var = tk.StringVar(value=f"Serial Error: {e}")
            self.ser = None

        # === Camera & Preview Attributes ===
        self.camera = None
        self.preview_on = False
        self.preview_image = None
        self.preview_update_active = False  # allow pausing preview updates during capture
        self.camera_lock = threading.Lock()  # guard camera access across threads

        # === Motor position trackers ===
        self.motor1_position_deg = 0.0  # Rotation platform
        self.motor2_position_mm = 0.0   # Linear rail
        self.motor3_position_deg = 0.0  # Vertical tilt axis
        
        # === Motor power relay state ===
        self.motor_power_on = False  # Tracks 24V relay state

        # === Calibration data ===
        self.calibration_data = {}  # {angle: {"near": mm_pos, "far": mm_pos}}
        self.is_calibrated = False
        self.current_calibration_angle = 0
        self.current_calibration_focus = "near"
        
        # === XMP pose settings ===
        self.lens_to_object_mm = tk.DoubleVar(value=250.0)  # Distance from lens to object center
        self.rail_to_horizon_deg = tk.DoubleVar(value=0.0)  # Camera pitch from horizontal
        
        # === Camera calibration parameters for XMP ===
        self.focal_length_35mm = tk.DoubleVar(value=50.0)  # 35mm equivalent focal length
        self.distortion_model = tk.StringVar(value="brown3")  # Lens distortion model
        self.skew = tk.DoubleVar(value=0.0)  # Camera skew parameter
        self.aspect_ratio = tk.DoubleVar(value=1.0)  # Pixel aspect ratio
        self.principal_point_u = tk.DoubleVar(value=0.0)  # Principal point U (normalized)
        self.principal_point_v = tk.DoubleVar(value=0.0)  # Principal point V (normalized)
        # Distortion coefficients (k1-k6 for Brown model)
        self.distortion_k1 = tk.DoubleVar(value=0.0)
        self.distortion_k2 = tk.DoubleVar(value=0.0)
        self.distortion_k3 = tk.DoubleVar(value=0.0)
        self.distortion_p1 = tk.DoubleVar(value=0.0)
        self.distortion_p2 = tk.DoubleVar(value=0.0)
        self.distortion_k4 = tk.DoubleVar(value=0.0)
        
        # === XMP Position Scale Factor ===
        # For macro/micro photography, scale up positions to avoid precision issues
        # e.g., 1000.0 converts 0.001m to 1.0m in XMP (scale model back in RC)
        self.xmp_position_scale = tk.DoubleVar(value=1.0)  # 1.0 = no scaling

        # === Capture settings ===
        self.specimen_name = tk.StringVar(value="specimen_001")
        self.output_dir = tk.StringVar(value=DEFAULT_OUTPUT_DIR)
        self.stacks_count = tk.IntVar(value=DEFAULT_STACKS)
        self.shots_per_stack = tk.IntVar(value=DEFAULT_SHOTS_PER_STACK)
        self.settle_delay = tk.DoubleVar(value=DEFAULT_SETTLE_DELAY)
        self.image_format = tk.StringVar(value="JPG")
        
        # === Scan mode settings ===
        self.scan_mode = tk.StringVar(value="ring")  # "ring" or "spherical"
        self.elevation_min = tk.DoubleVar(value=-60.0)  # Min tilt angle for spherical scan
        self.elevation_max = tk.DoubleVar(value=60.0)   # Max tilt angle for spherical scan
        self.elevation_steps = tk.IntVar(value=5)       # Number of elevation angles
        
        # === Camera settings ===
        self.shutter_speed = tk.IntVar(value=0)  # 0 = auto
        self.brightness = tk.DoubleVar(value=0.0)
        # Resolution settings
        self.preview_resolution = tk.StringVar(value="640x480")
        self.capture_resolution = tk.StringVar(value="4056x3040")
        # Exposure/Color settings
        self.ae_enable = tk.BooleanVar(value=True)
        self.iso_value = tk.StringVar(value="Auto")
        self.analogue_gain = tk.DoubleVar(value=1.0)
        self.ev_compensation = tk.DoubleVar(value=0.0)
        self.awb_enable = tk.BooleanVar(value=True)
        self.wb_red_gain = tk.DoubleVar(value=1.8)
        self.wb_blue_gain = tk.DoubleVar(value=1.5)
        self.contrast = tk.DoubleVar(value=1.0)
        self.saturation = tk.DoubleVar(value=1.0)
        self.sharpness = tk.DoubleVar(value=1.0)
        
        # === Progress tracking ===
        self.capture_progress = tk.DoubleVar(value=0.0)
        self.capture_running = False
        self.capture_thread = None

        # === Rotation mapping (app-level, no firmware changes) ===
        # rotation_scale: how many firmware degrees to command for 1° of desired movement
        # rotation_invert: swap CW/CCW if mechanical direction is flipped
        self.rotation_scale = tk.DoubleVar(value=1.0)
        self.rotation_invert = tk.BooleanVar(value=False)

        # Load persisted settings if available
        self._load_app_settings()

        # Build the GUI
        self._build_gui()
        self._layout_gui()
        
        # Remove all focus handling bindings - let tkinter handle it naturally
        # self.bind("<Button-1>", self._on_window_click)
        # Remove FocusOut binding - it was causing keyboard input issues
        # self.bind("<FocusOut>", self._on_focus_lost)

        # Create output directory if it doesn't exist
        os.makedirs(DEFAULT_OUTPUT_DIR, exist_ok=True)

        # Persist settings when changed
        try:
            # Trace changes to save settings automatically
            self.rotation_scale.trace_add('write', lambda *_: self._save_app_settings())
            self.rotation_invert.trace_add('write', lambda *_: self._save_app_settings())
        except AttributeError:
            # Older Tk fallback
            self.rotation_scale.trace('w', lambda *_: self._save_app_settings())
            self.rotation_invert.trace('w', lambda *_: self._save_app_settings())
            
        # Initialize status displays
        self.after(100, self._update_manual_status)

    def _build_gui(self):
        # Create main notebook for tabs
        # Create main container with notebook on left and help panel on right
        main_container = ttk.Frame(self)
        main_container.pack(fill="both", expand=True)
        
        # Left side: Tabs
        left_side = ttk.Frame(main_container)
        left_side.pack(side="left", fill="both", expand=True)
        
        self.notebook = ttk.Notebook(left_side)
        
        # === Manual Control Tab ===
        self.manual_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.manual_tab, text="Manual Control")
        self._build_manual_tab()
        
        # === Calibration Tab ===
        self.calibration_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.calibration_tab, text="Calibration")
        self._build_calibration_tab()
        
        # === Capture & Camera Tab (combined) ===
        self.capture_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.capture_tab, text="Capture & Camera")
        self._build_capture_and_camera_tab()
        
        self.notebook.pack(fill="both", expand=True)
        
        # Right side: Persistent help/notes panel
        right_side = ttk.Frame(main_container, width=400)
        right_side.pack(side="right", fill="both", padx=(5,0))
        right_side.pack_propagate(False)  # Maintain fixed width
        
        # Help/Instructions section
        self.help_frame = ttk.LabelFrame(right_side, text="User Manual", padding=10)
        self.help_frame.pack(fill="both", expand=True, pady=(0,5))
        
        # Create scrollable text widget for instructions
        help_container = ttk.Frame(self.help_frame)
        help_container.pack(fill="both", expand=True)
        
        self.help_text = tk.Text(help_container, wrap="word", font=("TkDefaultFont", 9),
                                padx=10, pady=10)
        self.help_text.pack(side="left", fill="both", expand=True)
        
        help_scrollbar = ttk.Scrollbar(help_container, orient="vertical", 
                                      command=self.help_text.yview)
        help_scrollbar.pack(side="right", fill="y")
        self.help_text.config(yscrollcommand=help_scrollbar.set)
        
        # Load manual content
        self._load_manual_content()
        self.help_text.config(state="disabled")  # Make read-only
        
        # Notes section
        self.notes_frame = ttk.LabelFrame(right_side, text="Session Notes", padding=10)
        self.notes_frame.pack(fill="both", pady=(5,0))
        
        notes_container = ttk.Frame(self.notes_frame)
        notes_container.pack(fill="both", expand=True)
        
        self.notes_text = tk.Text(notes_container, wrap="word", font=("TkDefaultFont", 9),
                                 height=8, padx=5, pady=5)
        self.notes_text.pack(side="left", fill="both", expand=True)
        
        notes_scrollbar = ttk.Scrollbar(notes_container, orient="vertical",
                                       command=self.notes_text.yview)
        notes_scrollbar.pack(side="right", fill="y")
        self.notes_text.config(yscrollcommand=notes_scrollbar.set)
        
        # Auto-save notes on any change
        self.notes_text.bind("<<Modified>>", self._auto_save_notes)
        
        # Load existing notes
        self._load_notes()

        # Status Bar (will be packed later in _layout_gui)
        self.status_bar = ttk.Label(self, textvariable=self.status_var, relief="sunken", anchor="w")

    def _build_manual_tab(self):
        """Build the manual control tab with controls on left and instructions on right"""
        # Create main layout with left and right sections
        main_frame = ttk.Frame(self.manual_tab)
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Left column for camera and motor controls
        left_column = ttk.Frame(main_frame)
        left_column.pack(side="left", fill="both", expand=True, padx=(0,5))
        
        # Right column for instructions
        right_column = ttk.Frame(main_frame)
        right_column.pack(side="right", fill="both", expand=True, padx=(5,0))
        
        # === LEFT COLUMN: Camera and Motor Controls ===
        
        # --- Camera Frame ---
        self.camera_frame = ttk.LabelFrame(left_column, text="Camera Preview", padding=10)
        self.camera_frame.pack(fill="both", expand=True, pady=(0,10))
        
        # Configure frame grid
        self.camera_frame.columnconfigure(0, weight=1)
        self.camera_frame.rowconfigure(0, weight=1)
        
        self.preview_label = tk.Label(self.camera_frame, bg="black", anchor="center")
        self.preview_label.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        self.btn_preview = ttk.Button(self.camera_frame, text="Start Preview", command=self.toggle_preview)
        self.btn_preview.grid(row=1, column=0, pady=(5,0))

        # --- Motor Controls Frame ---
        motors_frame = ttk.Frame(left_column)
        motors_frame.pack(fill="x")
        
        # Configure motors frame
        motors_frame.columnconfigure(0, weight=1)
        motors_frame.columnconfigure(1, weight=1)
        
        # --- Motor 1 Frame ---
        self.motor1_frame = ttk.LabelFrame(motors_frame, text="Motor 1 (Rotation)", padding=10)
        self.motor1_frame.grid(row=0, column=0, padx=(0,5), pady=5, sticky="nsew")
        
        self.motor1_pos_var = tk.StringVar(value="0.0°")
        ttk.Label(self.motor1_frame, text="Position:").grid(row=0, column=0, sticky="w")
        ttk.Label(self.motor1_frame, textvariable=self.motor1_pos_var, font=("TkDefaultFont", 10, "bold")).grid(row=0, column=1, sticky="w")
        
        ttk.Label(self.motor1_frame, text="Step Size:").grid(row=1, column=0, sticky="w")
        self.motor1_step_var = tk.StringVar(value="1.0")
        self.motor1_step_entry = ttk.Entry(self.motor1_frame, textvariable=self.motor1_step_var, width=8)
        self.motor1_step_entry.grid(row=1, column=1, sticky="w")
        ttk.Label(self.motor1_frame, text="degrees").grid(row=1, column=2, sticky="w")
        
        btn_frame1 = ttk.Frame(self.motor1_frame)
        btn_frame1.grid(row=2, column=0, columnspan=3, pady=(10,0))
        
        self.btn_m1_ccw = ttk.Button(btn_frame1, text="◀ CCW", command=self.motor1_ccw)
        self.btn_m1_ccw.grid(row=0, column=0, padx=(0,5))
        
        self.btn_m1_home = ttk.Button(btn_frame1, text="⌂ Home", command=self.motor1_home)
        self.btn_m1_home.grid(row=0, column=1, padx=5)
        
        self.btn_m1_cw = ttk.Button(btn_frame1, text="CW ▶", command=self.motor1_cw)
        self.btn_m1_cw.grid(row=0, column=2, padx=(5,0))

        # --- Motor 2 Frame ---
        self.motor2_frame = ttk.LabelFrame(motors_frame, text="Motor 2 (Linear)", padding=10)
        self.motor2_frame.grid(row=0, column=1, padx=(5,0), pady=5, sticky="nsew")
        
        self.motor2_pos_var = tk.StringVar(value="0.0mm")
        ttk.Label(self.motor2_frame, text="Position:").grid(row=0, column=0, sticky="w")
        ttk.Label(self.motor2_frame, textvariable=self.motor2_pos_var, font=("TkDefaultFont", 10, "bold")).grid(row=0, column=1, sticky="w")
        
        ttk.Label(self.motor2_frame, text="Step Size:").grid(row=1, column=0, sticky="w")
        self.motor2_step_var = tk.StringVar(value="1.0")
        self.motor2_step_entry = ttk.Entry(self.motor2_frame, textvariable=self.motor2_step_var, width=8)
        self.motor2_step_entry.grid(row=1, column=1, sticky="w")
        ttk.Label(self.motor2_frame, text="mm").grid(row=1, column=2, sticky="w")
        
        btn_frame2 = ttk.Frame(self.motor2_frame)
        btn_frame2.grid(row=2, column=0, columnspan=3, pady=(10,0))
        
        self.btn_m2_down = ttk.Button(btn_frame2, text="▼ Down", command=self.motor2_down)
        self.btn_m2_down.grid(row=0, column=0, padx=(0,5))
        
        self.btn_m2_home = ttk.Button(btn_frame2, text="⌂ Home", command=self.motor2_home)
        self.btn_m2_home.grid(row=0, column=1, padx=5)
        
        self.btn_m2_up = ttk.Button(btn_frame2, text="▲ Up", command=self.motor2_up)
        self.btn_m2_up.grid(row=0, column=2, padx=(5,0))

        # --- Motor 3 Frame (Vertical Tilt) ---
        self.motor3_frame = ttk.LabelFrame(motors_frame, text="Motor 3 (Tilt)", padding=10)
        self.motor3_frame.grid(row=1, column=0, columnspan=2, padx=0, pady=5, sticky="ew")
        
        self.motor3_pos_var = tk.StringVar(value="0.0°")
        ttk.Label(self.motor3_frame, text="Position:").grid(row=0, column=0, sticky="w")
        ttk.Label(self.motor3_frame, textvariable=self.motor3_pos_var, font=("TkDefaultFont", 10, "bold")).grid(row=0, column=1, sticky="w")
        
        ttk.Label(self.motor3_frame, text="Step Size:").grid(row=1, column=0, sticky="w")
        self.motor3_step_var = tk.StringVar(value="1.0")
        self.motor3_step_entry = ttk.Entry(self.motor3_frame, textvariable=self.motor3_step_var, width=8)
        self.motor3_step_entry.grid(row=1, column=1, sticky="w")
        ttk.Label(self.motor3_frame, text="degrees").grid(row=1, column=2, sticky="w")
        
        btn_frame3 = ttk.Frame(self.motor3_frame)
        btn_frame3.grid(row=2, column=0, columnspan=3, pady=(10,0))
        
        self.btn_m3_down = ttk.Button(btn_frame3, text="▼ Down", command=self.motor3_down)
        self.btn_m3_down.grid(row=0, column=0, padx=(0,5))
        
        self.btn_m3_home = ttk.Button(btn_frame3, text="⌂ Home", command=self.motor3_home)
        self.btn_m3_home.grid(row=0, column=1, padx=5)
        
        self.btn_m3_up = ttk.Button(btn_frame3, text="▲ Up", command=self.motor3_up)
        self.btn_m3_up.grid(row=0, column=2, padx=(5,0))

        # === RIGHT COLUMN: Setup Instructions ===
        
        # Hardware Setup Instructions
        self.setup_instructions_frame = ttk.LabelFrame(right_column, text="Hardware Setup Instructions", padding=10)
        self.setup_instructions_frame.pack(fill="both", expand=True, pady=(0,10))
        
        setup_instructions = """Initial Hardware Setup:

1. POWER AND CONNECTIONS
   • Ensure Arduino is connected via USB
   • Power on turntable and linear rail motors
   • Connect camera ribbon cable securely
   • Start preview to verify camera operation

2. OBJECT POSITIONING
   • Place specimen on turntable center
   • Use rotation controls to position object
   • Object should remain centered as turntable rotates
   • Adjust object or turntable as needed

3. FOCUS PLANE SETUP (Linear Rail)
   • Use linear rail to move camera closer/farther
   • Start with camera at nearest focus position
   • Object should fill most of preview frame
   • Ensure object is clearly in focus

4. ESTABLISH HOME POSITIONS
   • Position turntable at desired 0° reference
   • This will be your starting angle for captures
   • Position rail at your nearest focus distance
   • Press 'Home' buttons to set these as reference points

5. TEST MOVEMENTS
   • Test small movements with both motors
   • Verify directions match your expectations
   • Check that positions track correctly
   • Adjust step sizes for fine control

TIPS:
• Start with small step sizes (1° rotation, 0.5mm linear)
• Home positions are your reference points for calibration
• The preview helps verify proper positioning
• Use consistent lighting during setup"""
        
        self.setup_instructions_text = tk.Text(self.setup_instructions_frame, wrap="word", 
                                              width=50, height=25, font=("TkDefaultFont", 9))
        self.setup_instructions_text.pack(fill="both", expand=True, side="left")
        
        # Add scrollbar for instructions
        instructions_scrollbar = ttk.Scrollbar(self.setup_instructions_frame, orient="vertical", 
                                              command=self.setup_instructions_text.yview)
        instructions_scrollbar.pack(side="right", fill="y")
        self.setup_instructions_text.config(yscrollcommand=instructions_scrollbar.set)
        
        # Insert instructions and make read-only
        self.setup_instructions_text.insert("1.0", setup_instructions)
        self.setup_instructions_text.config(state="disabled")
        
        # Status and Tips
        self.manual_status_frame = ttk.LabelFrame(right_column, text="Current Status", padding=10)
        self.manual_status_frame.pack(fill="x")
        
        status_info = ttk.Frame(self.manual_status_frame)
        status_info.pack(fill="x")
        
        ttk.Label(status_info, text="Arduino:").grid(row=0, column=0, sticky="w")
        self.arduino_status_var = tk.StringVar(value="Checking...")
        ttk.Label(status_info, textvariable=self.arduino_status_var, 
                 font=("TkDefaultFont", 9)).grid(row=0, column=1, sticky="w", padx=(5,0))
        
        ttk.Label(status_info, text="Camera:").grid(row=1, column=0, sticky="w")
        self.camera_status_var = tk.StringVar(value="Not Started")
        ttk.Label(status_info, textvariable=self.camera_status_var, 
                 font=("TkDefaultFont", 9)).grid(row=1, column=1, sticky="w", padx=(5,0))
        
        # Motor Power Control
        ttk.Label(status_info, text="Motor Power:").grid(row=2, column=0, sticky="w")
        self.motor_power_status_var = tk.StringVar(value="OFF")
        ttk.Label(status_info, textvariable=self.motor_power_status_var,
                 font=("TkDefaultFont", 9, "bold")).grid(row=2, column=1, sticky="w", padx=(5,0))
        self.btn_motor_power = ttk.Button(status_info, text="Turn ON", command=self.toggle_motor_power, width=10)
        self.btn_motor_power.grid(row=2, column=2, sticky="w", padx=(10,0))
        
        # Update status display
        self._update_manual_status()
        
        # Rotation Mapping Settings
        self.rotation_map_frame = ttk.LabelFrame(right_column, text="Rotation Calibration", padding=10)
        self.rotation_map_frame.pack(fill="x", pady=(10,0))
        
        map_grid = ttk.Frame(self.rotation_map_frame)
        map_grid.pack(fill="x")
        
        ttk.Label(map_grid, text="Scale Factor:").grid(row=0, column=0, sticky="w", pady=2)
        scale_entry = ttk.Entry(map_grid, textvariable=self.rotation_scale, width=10)
        scale_entry.grid(row=0, column=1, sticky="w", padx=(5,0), pady=2)
        ttk.Label(map_grid, text="(firmware° per actual°)", foreground="#666", 
                 font=("TkDefaultFont", 8)).grid(row=0, column=2, sticky="w", padx=(5,0), pady=2)
        
        invert_check = ttk.Checkbutton(map_grid, text="Reverse Direction (CW ⟷ CCW)", 
                                       variable=self.rotation_invert)
        invert_check.grid(row=1, column=0, columnspan=3, sticky="w", pady=(5,2))
        
        # Help text
        help_text = ("Test rotation: Command 90° and adjust scale until\n"
                    "turntable physically rotates 90°. Check the invert\n"
                    "option if CW/CCW directions are reversed.")
        ttk.Label(map_grid, text=help_text, foreground="#666", justify="left", 
                 font=("TkDefaultFont", 8)).grid(row=2, column=0, columnspan=3, sticky="w", pady=(5,0))

    def _build_calibration_tab(self):
        # Create main layout with left and right sections
        main_frame = ttk.Frame(self.calibration_tab)
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Left column for camera and motor controls
        left_column = ttk.Frame(main_frame)
        left_column.pack(side="left", fill="both", expand=True, padx=(0,5))
        
        # Right column for calibration workflow
        right_column = ttk.Frame(main_frame)
        right_column.pack(side="right", fill="both", expand=True, padx=(5,0))
        
        # === LEFT COLUMN: Camera and Motor Controls ===
        
        # Camera preview (same as manual tab)
        self.cal_camera_frame = ttk.LabelFrame(left_column, text="Camera Preview", padding=10)
        self.cal_camera_frame.pack(fill="both", expand=True, pady=(0,10))
        
        # Configure frame grid
        self.cal_camera_frame.columnconfigure(0, weight=1)
        self.cal_camera_frame.rowconfigure(0, weight=1)
        
        self.cal_preview_label = tk.Label(self.cal_camera_frame, bg="black", anchor="center")
        self.cal_preview_label.grid(row=0, column=0, padx=5, pady=5, sticky="nsew")
        
        self.cal_btn_preview = ttk.Button(self.cal_camera_frame, text="Start Preview", 
                                         command=self.toggle_preview)
        self.cal_btn_preview.grid(row=1, column=0, pady=(5,0))
        
        # Motor 1 controls (rotation)
        self.cal_motor1_frame = ttk.LabelFrame(left_column, text="Rotation Motor", padding=10)
        self.cal_motor1_frame.pack(fill="x", pady=(0,10))
        
        # Position display
        pos_frame1 = ttk.Frame(self.cal_motor1_frame)
        pos_frame1.pack(fill="x", pady=(0,5))
        ttk.Label(pos_frame1, text="Position:").pack(side="left")
        ttk.Label(pos_frame1, textvariable=self.motor1_pos_var, 
                 font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=(5,0))
        
        # Step size
        step_frame1 = ttk.Frame(self.cal_motor1_frame)
        step_frame1.pack(fill="x", pady=(0,5))
        ttk.Label(step_frame1, text="Step:").pack(side="left")
        self.cal_motor1_step_var = tk.StringVar(value="1.0")
        step_entry1 = ttk.Entry(step_frame1, textvariable=self.cal_motor1_step_var, width=8)
        step_entry1.pack(side="left", padx=(5,0))
        ttk.Label(step_frame1, text="degrees").pack(side="left", padx=(5,0))
        
        # Control buttons
        btn_frame1 = ttk.Frame(self.cal_motor1_frame)
        btn_frame1.pack(fill="x")
        
        ttk.Button(btn_frame1, text="◀ CCW", command=self.cal_motor1_ccw).pack(side="left", padx=(0,5))
        ttk.Button(btn_frame1, text="⌂ Home", command=self.motor1_home).pack(side="left", padx=5)
        ttk.Button(btn_frame1, text="CW ▶", command=self.cal_motor1_cw).pack(side="left", padx=(5,0))
        
        # Motor 2 controls (linear)
        self.cal_motor2_frame = ttk.LabelFrame(left_column, text="Focus Motor (Linear)", padding=10)
        self.cal_motor2_frame.pack(fill="x")
        
        # Position display
        pos_frame2 = ttk.Frame(self.cal_motor2_frame)
        pos_frame2.pack(fill="x", pady=(0,5))
        ttk.Label(pos_frame2, text="Position:").pack(side="left")
        ttk.Label(pos_frame2, textvariable=self.motor2_pos_var, 
                 font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=(5,0))
        
        # Step size
        step_frame2 = ttk.Frame(self.cal_motor2_frame)
        step_frame2.pack(fill="x", pady=(0,5))
        ttk.Label(step_frame2, text="Step:").pack(side="left")
        self.cal_motor2_step_var = tk.StringVar(value="0.5")
        step_entry2 = ttk.Entry(step_frame2, textvariable=self.cal_motor2_step_var, width=8)
        step_entry2.pack(side="left", padx=(5,0))
        ttk.Label(step_frame2, text="mm").pack(side="left", padx=(5,0))
        
        # Control buttons
        btn_frame2 = ttk.Frame(self.cal_motor2_frame)
        btn_frame2.pack(fill="x")
        
        ttk.Button(btn_frame2, text="▼ Back", command=self.cal_motor2_down).pack(side="left", padx=(0,5))
        ttk.Button(btn_frame2, text="⌂ Home", command=self.motor2_home).pack(side="left", padx=5)
        ttk.Button(btn_frame2, text="▲ Forward", command=self.cal_motor2_up).pack(side="left", padx=(5,0))
        
        # Motor 3 controls (tilt)
        self.cal_motor3_frame = ttk.LabelFrame(left_column, text="Tilt Motor", padding=10)
        self.cal_motor3_frame.pack(fill="x", pady=(10,0))
        
        # Position display
        pos_frame3 = ttk.Frame(self.cal_motor3_frame)
        pos_frame3.pack(fill="x", pady=(0,5))
        ttk.Label(pos_frame3, text="Position:").pack(side="left")
        ttk.Label(pos_frame3, textvariable=self.motor3_pos_var, 
                 font=("TkDefaultFont", 10, "bold")).pack(side="left", padx=(5,0))
        
        # Step size
        step_frame3 = ttk.Frame(self.cal_motor3_frame)
        step_frame3.pack(fill="x", pady=(0,5))
        ttk.Label(step_frame3, text="Step:").pack(side="left")
        self.cal_motor3_step_var = tk.StringVar(value="1.0")
        step_entry3 = ttk.Entry(step_frame3, textvariable=self.cal_motor3_step_var, width=8)
        step_entry3.pack(side="left", padx=(5,0))
        ttk.Label(step_frame3, text="degrees").pack(side="left", padx=(5,0))
        
        # Control buttons
        btn_frame3 = ttk.Frame(self.cal_motor3_frame)
        btn_frame3.pack(fill="x")
        
        ttk.Button(btn_frame3, text="▼ Down", command=self.cal_motor3_down).pack(side="left", padx=(0,5))
        ttk.Button(btn_frame3, text="⌂ Zero", command=self.motor3_home).pack(side="left", padx=5)
        ttk.Button(btn_frame3, text="▲ Up", command=self.cal_motor3_up).pack(side="left", padx=(5,0))
        
        # === RIGHT COLUMN: Calibration Workflow ===
        
        # Calibration status
        self.calibration_status_frame = ttk.LabelFrame(right_column, text="Calibration Status", padding=10)
        self.calibration_status_frame.pack(fill="x", pady=(0,10))
        
        self.calibration_status_var = tk.StringVar(value="Not Calibrated")
        status_frame = ttk.Frame(self.calibration_status_frame)
        status_frame.pack(fill="x")
        ttk.Label(status_frame, text="Status:").pack(side="left")
        ttk.Label(status_frame, textvariable=self.calibration_status_var, 
                 font=("TkDefaultFont", 10, "bold"), foreground="red").pack(side="left", padx=(5,0))
        
        # Current calibration step
        self.calibration_step_frame = ttk.LabelFrame(right_column, text="Current Step", padding=10)
        self.calibration_step_frame.pack(fill="x", pady=(0,10))
        
        self.current_angle_var = tk.StringVar(value="0°")
        self.current_focus_var = tk.StringVar(value="Near")
        
        step_info_frame = ttk.Frame(self.calibration_step_frame)
        step_info_frame.pack(fill="x")
        
        angle_frame = ttk.Frame(step_info_frame)
        angle_frame.pack(fill="x", pady=(0,5))
        ttk.Label(angle_frame, text="Angle:").pack(side="left")
        ttk.Label(angle_frame, textvariable=self.current_angle_var, 
                 font=("TkDefaultFont", 12, "bold")).pack(side="left", padx=(5,0))
        
        focus_frame = ttk.Frame(step_info_frame)
        focus_frame.pack(fill="x")
        ttk.Label(focus_frame, text="Focus:").pack(side="left")
        ttk.Label(focus_frame, textvariable=self.current_focus_var, 
                 font=("TkDefaultFont", 12, "bold")).pack(side="left", padx=(5,0))
        
        # Calibration instructions
        instructions = """Calibration Workflow:
1. Use motor controls to position at current angle
2. Use focus motor to set near/far positions
3. Click "Capture Position" for each focus
4. Click "Next Step" when angle complete
5. Repeat for all cardinal angles"""
        
        self.instructions_frame = ttk.LabelFrame(right_column, text="Instructions", padding=10)
        self.instructions_frame.pack(fill="x", pady=(0,10))
        
        self.instructions_label = tk.Label(self.instructions_frame, text=instructions, 
                                          justify="left", wraplength=300)
        self.instructions_label.pack()
        
        # Calibration controls
        self.calibration_controls_frame = ttk.LabelFrame(right_column, text="Calibration Controls", padding=10)
        self.calibration_controls_frame.pack(fill="x", pady=(0,10))
        
        controls_grid = ttk.Frame(self.calibration_controls_frame)
        controls_grid.pack(fill="x")
        
        self.btn_start_calibration = ttk.Button(controls_grid, text="Start Calibration", 
                                               command=self.start_calibration)
        self.btn_start_calibration.grid(row=0, column=0, pady=2, sticky="ew")
        
        self.btn_capture_position = ttk.Button(controls_grid, text="Capture Position", 
                                              command=self.capture_calibration_position, state="disabled")
        self.btn_capture_position.grid(row=0, column=1, padx=(5,0), pady=2, sticky="ew")
        
        self.btn_next_step = ttk.Button(controls_grid, text="Next Step", 
                                       command=self.next_calibration_step, state="disabled")
        self.btn_next_step.grid(row=1, column=0, pady=2, sticky="ew")
        
        self.btn_save_calibration = ttk.Button(controls_grid, text="Save Calibration", 
                                              command=self.save_calibration, state="disabled")
        self.btn_save_calibration.grid(row=2, column=0, pady=2, sticky="ew")
        
        self.btn_load_calibration = ttk.Button(controls_grid, text="Load Calibration", 
                                              command=self.load_calibration)
        self.btn_load_calibration.grid(row=2, column=1, padx=(5,0), pady=2, sticky="ew")

        # Reset/Start Over button
        self.btn_reset_calibration = ttk.Button(controls_grid, text="Reset / Start Over",
                                               command=self.reset_calibration)
        self.btn_reset_calibration.grid(row=3, column=0, columnspan=2, pady=(8,2), sticky="ew")
        
        controls_grid.columnconfigure(0, weight=1)
        controls_grid.columnconfigure(1, weight=1)
        
        # XMP Pose Settings
        self.xmp_settings_frame = ttk.LabelFrame(right_column, text="Camera Calibration (XMP)", padding=10)
        self.xmp_settings_frame.pack(fill="x", pady=(0,10))
        
        # Create a notebook for organized settings
        xmp_notebook = ttk.Notebook(self.xmp_settings_frame)
        xmp_notebook.pack(fill="both", expand=True)
        
        # Tab 1: Basic Pose Settings
        pose_tab = ttk.Frame(xmp_notebook)
        xmp_notebook.add(pose_tab, text="Pose")
        
        pose_grid = ttk.Frame(pose_tab)
        pose_grid.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(pose_grid, text="Lens to Object (mm):").grid(row=0, column=0, sticky="w", pady=2)
        self.lens_distance_entry = ttk.Entry(pose_grid, textvariable=self.lens_to_object_mm, width=12)
        self.lens_distance_entry.grid(row=0, column=1, sticky="w", padx=(5,0), pady=2)
        
        ttk.Label(pose_grid, text="Focal Length (35mm eq):").grid(row=1, column=0, sticky="w", pady=2)
        self.focal_length_entry = ttk.Entry(pose_grid, textvariable=self.focal_length_35mm, width=12)
        self.focal_length_entry.grid(row=1, column=1, sticky="w", padx=(5,0), pady=2)
        
        ttk.Label(pose_grid, text="Position Scale Preset:").grid(row=2, column=0, sticky="w", pady=2)
        scale_preset_frame = ttk.Frame(pose_grid)
        scale_preset_frame.grid(row=2, column=1, sticky="w", padx=(5,0), pady=2)
        
        ttk.Button(scale_preset_frame, text="Tiny (1-5mm)", width=12,
                  command=lambda: self.xmp_position_scale.set(100000)).pack(side="left", padx=(0,2))
        ttk.Button(scale_preset_frame, text="Small (5-20mm)", width=12,
                  command=lambda: self.xmp_position_scale.set(20000)).pack(side="left", padx=2)
        ttk.Button(scale_preset_frame, text="Medium (20-50mm)", width=13,
                  command=lambda: self.xmp_position_scale.set(5000)).pack(side="left", padx=(2,0))
        
        ttk.Label(pose_grid, text="Current Scale:").grid(row=3, column=0, sticky="w", pady=2)
        scale_display = ttk.Label(pose_grid, textvariable=self.xmp_position_scale, 
                                 font=("TkDefaultFont", 9, "bold"))
        scale_display.grid(row=3, column=1, sticky="w", padx=(5,0), pady=2)
        
        # Help text with scale factor calculator
        help_text = """LENS TO OBJECT: Measure from lens front to specimen center (mm)
  How to measure: Use ruler/caliper from camera lens to turntable center
   
FOCAL LENGTH (35mm equivalent): Camera's field of view
  - Pi HQ Camera (6mm lens): ~50mm equivalent
  - Pi HQ Camera (16mm lens): ~130mm equivalent
  - Pi Camera V2: ~29mm equivalent
  How to find: Check camera spec sheet or calculate:
    35mm_equiv = actual_focal_mm x (36mm / sensor_width_mm)
   
POSITION SCALE PRESET: Keeps XMP precision manageable
  Select based on your specimen size:
  - Tiny (1-5mm): Insects, coins, small jewelry
  - Small (5-20mm): Larger insects, small fossils, electronic parts
  - Medium (20-50mm): Rocks, larger fossils, small sculptures
  
  Note: Rail angle is automatically calculated from tilt motor position
  Note: Scale final model back to real size in RealityCapture"""
        
        help_label = tk.Text(pose_grid, height=18, wrap="word", font=("TkDefaultFont", 8),
                            foreground="#444", relief="flat",
                            borderwidth=0, highlightthickness=0)
        help_label.insert("1.0", help_text)
        help_label.config(state="disabled")
        help_label.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(10,0))
        
        # Tab 2: Camera Intrinsics
        intrinsics_tab = ttk.Frame(xmp_notebook)
        xmp_notebook.add(intrinsics_tab, text="Intrinsics")
        
        intrinsics_grid = ttk.Frame(intrinsics_tab)
        intrinsics_grid.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(intrinsics_grid, text="Principal Point U:").grid(row=0, column=0, sticky="w", pady=2)
        self.principal_u_entry = ttk.Entry(intrinsics_grid, textvariable=self.principal_point_u, width=12)
        self.principal_u_entry.grid(row=0, column=1, sticky="w", padx=(5,0), pady=2)
        
        ttk.Label(intrinsics_grid, text="Principal Point V:").grid(row=1, column=0, sticky="w", pady=2)
        self.principal_v_entry = ttk.Entry(intrinsics_grid, textvariable=self.principal_point_v, width=12)
        self.principal_v_entry.grid(row=1, column=1, sticky="w", padx=(5,0), pady=2)
        
        ttk.Label(intrinsics_grid, text="Aspect Ratio:").grid(row=2, column=0, sticky="w", pady=2)
        self.aspect_ratio_entry = ttk.Entry(intrinsics_grid, textvariable=self.aspect_ratio, width=12)
        self.aspect_ratio_entry.grid(row=2, column=1, sticky="w", padx=(5,0), pady=2)
        
        ttk.Label(intrinsics_grid, text="Skew:").grid(row=3, column=0, sticky="w", pady=2)
        self.skew_entry = ttk.Entry(intrinsics_grid, textvariable=self.skew, width=12)
        self.skew_entry.grid(row=3, column=1, sticky="w", padx=(5,0), pady=2)
        
        # Help text
        help_text2 = """PRINCIPAL POINT: Optical center of sensor (normalized 0-1)
  - Default: (0.0, 0.0) = perfect sensor center
  - Leave at 0,0 unless you have calibration data
  - Positive U = optical center right of physical center
  - Positive V = optical center below physical center
   
ASPECT RATIO: Pixel width/height ratio
  - Default: 1.0 (square pixels)
  - Leave at 1.0 for modern cameras
   
SKEW: Sensor axis alignment
  - Default: 0.0 (axes perpendicular)
  - Leave at 0 unless using specialty camera"""
        
        help_label2 = tk.Text(intrinsics_grid, height=14, wrap="word", font=("TkDefaultFont", 8),
                             foreground="#444", relief="flat",
                             borderwidth=0, highlightthickness=0)
        help_label2.insert("1.0", help_text2)
        help_label2.config(state="disabled")
        help_label2.grid(row=4, column=0, columnspan=2, sticky="ew", pady=(10,0))
        
        # Tab 3: Distortion Coefficients
        distortion_tab = ttk.Frame(xmp_notebook)
        xmp_notebook.add(distortion_tab, text="Distortion")
        
        distortion_grid = ttk.Frame(distortion_tab)
        distortion_grid.pack(fill="x", padx=5, pady=5)
        
        ttk.Label(distortion_grid, text="Radial k1:").grid(row=0, column=0, sticky="w", pady=2)
        self.k1_entry = ttk.Entry(distortion_grid, textvariable=self.distortion_k1, width=12)
        self.k1_entry.grid(row=0, column=1, sticky="w", padx=(5,0), pady=2)
        
        ttk.Label(distortion_grid, text="Radial k2:").grid(row=1, column=0, sticky="w", pady=2)
        self.k2_entry = ttk.Entry(distortion_grid, textvariable=self.distortion_k2, width=12)
        self.k2_entry.grid(row=1, column=1, sticky="w", padx=(5,0), pady=2)
        
        ttk.Label(distortion_grid, text="Radial k3:").grid(row=2, column=0, sticky="w", pady=2)
        self.k3_entry = ttk.Entry(distortion_grid, textvariable=self.distortion_k3, width=12)
        self.k3_entry.grid(row=2, column=1, sticky="w", padx=(5,0), pady=2)
        
        ttk.Label(distortion_grid, text="Tangential p1:").grid(row=3, column=0, sticky="w", pady=2)
        self.p1_entry = ttk.Entry(distortion_grid, textvariable=self.distortion_p1, width=12)
        self.p1_entry.grid(row=3, column=1, sticky="w", padx=(5,0), pady=2)
        
        ttk.Label(distortion_grid, text="Tangential p2:").grid(row=4, column=0, sticky="w", pady=2)
        self.p2_entry = ttk.Entry(distortion_grid, textvariable=self.distortion_p2, width=12)
        self.p2_entry.grid(row=4, column=1, sticky="w", padx=(5,0), pady=2)
        
        ttk.Label(distortion_grid, text="Radial k4:").grid(row=5, column=0, sticky="w", pady=2)
        self.k4_entry = ttk.Entry(distortion_grid, textvariable=self.distortion_k4, width=12)
        self.k4_entry.grid(row=5, column=1, sticky="w", padx=(5,0), pady=2)
        
        # Help text
        help_text3 = """DISTORTION COEFFICIENTS: Lens optical corrections
Brown-Conrady distortion model (brown3):
   
RADIAL (k1, k2, k3, k4): Barrel/pincushion distortion
  - Straight lines appear curved
  - k1 is primary coefficient (strongest effect)
     
TANGENTIAL (p1, p2): Sensor/lens misalignment
  - Image appears tilted or decentered
  - Usually much smaller than radial
   
WARNING: Leave all at 0.0 unless you have calibration data
  - Use OpenCV camera calibration with checkerboard pattern
  - Or RealityCapture's "Undistort" feature (estimates automatically)
  - Wrong values are worse than no values!
   
How to calibrate (advanced):
  1. Print checkerboard pattern (8x6 or larger)
  2. Capture 20-30 images from different angles
  3. Use OpenCV calibrateCamera() function
  4. Enter k1-k4 and p1-p2 values here"""
        
        help_label3 = tk.Text(distortion_grid, height=21, wrap="word", font=("TkDefaultFont", 8),
                             foreground="#444", relief="flat",
                             borderwidth=0, highlightthickness=0)
        help_label3.insert("1.0", help_text3)
        help_label3.config(state="disabled")
        help_label3.grid(row=6, column=0, columnspan=2, sticky="ew", pady=(10,0))
        
        # Calibration progress
        self.calibration_progress_frame = ttk.LabelFrame(right_column, text="Progress Log", padding=10)
        self.calibration_progress_frame.pack(fill="both", expand=True)
        
        # Create frame for text and scrollbar
        log_frame = ttk.Frame(self.calibration_progress_frame)
        log_frame.pack(fill="both", expand=True)
        
        self.calibration_progress_text = tk.Text(log_frame, height=8, width=40)
        self.calibration_progress_text.pack(side="left", fill="both", expand=True)
        
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", 
                                 command=self.calibration_progress_text.yview)
        scrollbar.pack(side="right", fill="y")
        self.calibration_progress_text.config(yscrollcommand=scrollbar.set)

    def _build_capture_and_camera_tab(self):
        """Build combined capture and camera settings tab with two columns"""
        # Create main layout with left and right columns
        main_frame = ttk.Frame(self.capture_tab)
        main_frame.pack(fill="both", expand=True, padx=10, pady=5)
        
        # Left column for capture settings
        left_column = ttk.Frame(main_frame)
        left_column.pack(side="left", fill="both", expand=True, padx=(0,5))
        
        # Right column for camera settings
        right_column = ttk.Frame(main_frame)
        right_column.pack(side="right", fill="both", expand=True, padx=(5,0))
        
        # === LEFT COLUMN: Capture Settings ===
        
        # Specimen settings
        self.specimen_frame = ttk.LabelFrame(left_column, text="Specimen", padding=10)
        
        ttk.Label(self.specimen_frame, text="Name:").grid(row=0, column=0, sticky="w")
        self.specimen_entry = ttk.Entry(self.specimen_frame, textvariable=self.specimen_name, width=20)
        self.specimen_entry.grid(row=0, column=1, sticky="w", padx=(5,0))
        
        ttk.Label(self.specimen_frame, text="Output Dir:").grid(row=1, column=0, sticky="w")
        self.output_dir_entry = ttk.Entry(self.specimen_frame, textvariable=self.output_dir, width=40)
        self.output_dir_entry.grid(row=1, column=1, sticky="w", padx=(5,0))
        
        self.btn_browse_dir = ttk.Button(self.specimen_frame, text="Browse", command=self.browse_output_dir)
        self.btn_browse_dir.grid(row=1, column=2, padx=(5,0))
        
        # Capture settings
        self.capture_settings_frame = ttk.LabelFrame(left_column, text="Capture Settings", padding=10)
        
        # Scan mode selection
        ttk.Label(self.capture_settings_frame, text="Scan Mode:").grid(row=0, column=0, sticky="w")
        scan_mode_frame = ttk.Frame(self.capture_settings_frame)
        scan_mode_frame.grid(row=0, column=1, columnspan=2, sticky="w", padx=(5,0))
        ttk.Radiobutton(scan_mode_frame, text="Ring (Horizontal)", variable=self.scan_mode, 
                       value="ring", command=self._update_scan_mode_ui).pack(side="left", padx=(0,10))
        ttk.Radiobutton(scan_mode_frame, text="Spherical (Spiral)", variable=self.scan_mode, 
                       value="spherical", command=self._update_scan_mode_ui).pack(side="left")
        
        ttk.Label(self.capture_settings_frame, text="Perspectives (angles):").grid(row=1, column=0, sticky="w")
        self.stacks_spinbox = ttk.Spinbox(self.capture_settings_frame, from_=1, to=360, 
                                         textvariable=self.stacks_count, width=10)
        self.stacks_spinbox.grid(row=1, column=1, sticky="w", padx=(5,0))
        
        ttk.Label(self.capture_settings_frame, text="Focus slices per angle:").grid(row=2, column=0, sticky="w")
        self.shots_spinbox = ttk.Spinbox(self.capture_settings_frame, from_=1, to=360, 
                                        textvariable=self.shots_per_stack, width=10)
        self.shots_spinbox.grid(row=2, column=1, sticky="w", padx=(5,0))
        
        # Show computed angle step to clarify spacing
        self.angle_step_label_var = tk.StringVar(value="Angle step: —")
        ttk.Label(self.capture_settings_frame, textvariable=self.angle_step_label_var).grid(row=1, column=2, sticky="w", padx=(10,0))
        
        # Spherical scan settings (initially hidden)
        self.spherical_settings_label = ttk.Label(self.capture_settings_frame, text="Elevation angles:")
        self.spherical_min_label = ttk.Label(self.capture_settings_frame, text="Min:")
        self.spherical_min_spinbox = ttk.Spinbox(self.capture_settings_frame, from_=-60, to=60, 
                                                 textvariable=self.elevation_min, width=8)
        self.spherical_max_label = ttk.Label(self.capture_settings_frame, text="Max:")
        self.spherical_max_spinbox = ttk.Spinbox(self.capture_settings_frame, from_=-60, to=60, 
                                                 textvariable=self.elevation_max, width=8)
        self.spherical_steps_label = ttk.Label(self.capture_settings_frame, text="Steps:")
        self.spherical_steps_spinbox = ttk.Spinbox(self.capture_settings_frame, from_=2, to=20, 
                                                   textvariable=self.elevation_steps, width=8)
        
        ttk.Label(self.capture_settings_frame, text="Settle Delay (s):").grid(row=4, column=0, sticky="w")
        self.settle_spinbox = ttk.Spinbox(self.capture_settings_frame, from_=0.1, to=5.0, increment=0.1,
                                         textvariable=self.settle_delay, width=10)
        self.settle_spinbox.grid(row=4, column=1, sticky="w", padx=(5,0))
        
        ttk.Label(self.capture_settings_frame, text="Image Format:").grid(row=5, column=0, sticky="w")
        self.format_combo = ttk.Combobox(self.capture_settings_frame, textvariable=self.image_format, 
                                        values=["JPG", "PNG", "TIFF"], state="readonly", width=8)
        self.format_combo.grid(row=5, column=1, sticky="w", padx=(5,0))

        # Update angle step label whenever perspectives value changes
        def _update_angle_step_label(*_):
            try:
                perspectives = max(1, int(self.stacks_count.get()))
            except Exception:
                perspectives = 1
            step = 360.0 / perspectives
            self.angle_step_label_var.set(f"Angle step: {step:.2f}°")
        # Trace and initial update
        try:
            self.stacks_count.trace_add('write', lambda *args: _update_angle_step_label())
        except AttributeError:
            # Fallback for older Tk versions
            self.stacks_count.trace('w', lambda *args: _update_angle_step_label())
        _update_angle_step_label()
        
        # Capture controls
        self.capture_controls_frame = ttk.LabelFrame(left_column, text="Controls", padding=10)
        
        self.btn_test_capture = ttk.Button(self.capture_controls_frame, text="Test Capture", 
                                          command=self.test_capture)
        self.btn_test_capture.grid(row=0, column=0, pady=5)
        
        self.btn_start_capture = ttk.Button(self.capture_controls_frame, text="Start Full Capture", 
                                           command=self.start_full_capture)
        self.btn_start_capture.grid(row=0, column=1, padx=(10,0), pady=5)
        
        self.btn_stop_capture = ttk.Button(self.capture_controls_frame, text="Stop Capture", 
                                          command=self.stop_capture, state="disabled")
        self.btn_stop_capture.grid(row=0, column=2, padx=(10,0), pady=5)
        
        # Progress
        self.progress_frame = ttk.LabelFrame(left_column, text="Progress", padding=10)
        
        self.progress_bar = ttk.Progressbar(self.progress_frame, variable=self.capture_progress, 
                                           maximum=100, length=400)
        self.progress_bar.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0,10))
        
        self.progress_label_var = tk.StringVar(value="Ready")
        ttk.Label(self.progress_frame, textvariable=self.progress_label_var).grid(row=1, column=0, sticky="w")
        
        # Capture log
        self.capture_log_frame = ttk.LabelFrame(left_column, text="Capture Log", padding=10)
        
        self.capture_log_text = tk.Text(self.capture_log_frame, height=8, width=60)
        self.capture_log_text.grid(row=0, column=0, sticky="nsew")
        
        capture_scrollbar = ttk.Scrollbar(self.capture_log_frame, orient="vertical", 
                                         command=self.capture_log_text.yview)
        capture_scrollbar.grid(row=0, column=1, sticky="ns")
        self.capture_log_text.config(yscrollcommand=capture_scrollbar.set)
        
        # Layout left column (capture settings)
        self.specimen_frame.pack(fill="x", pady=(0,5))
        self.capture_settings_frame.pack(fill="x", pady=5)
        self.capture_controls_frame.pack(fill="x", pady=5)
        self.progress_frame.pack(fill="x", pady=5)
        self.capture_log_frame.pack(fill="both", expand=True, pady=(5,0))
        
        # === RIGHT COLUMN: Camera Settings ===
        
        # Camera preview
        self.camera_preview_frame = ttk.LabelFrame(right_column, text="Camera Preview", padding=10)
        self.camera_preview_label = tk.Label(self.camera_preview_frame, bg="black", anchor="center")
        self.camera_preview_label.pack(padx=5, pady=(5,0))
        
        # Preview control button
        self.camera_preview_btn = ttk.Button(self.camera_preview_frame, text="Start Preview", 
                                             command=self.toggle_preview)
        self.camera_preview_btn.pack(pady=(5,5))
        
        # Camera controls
        self.camera_controls_frame = ttk.LabelFrame(right_column, text="Camera Controls", padding=10)
        
        # Create three sub-columns for better organization
        left_col = ttk.Frame(self.camera_controls_frame)
        left_col.grid(row=0, column=0, sticky="n", padx=(0,10))
        
        middle_col = ttk.Frame(self.camera_controls_frame)
        middle_col.grid(row=0, column=1, sticky="n", padx=10)
        
        right_col = ttk.Frame(self.camera_controls_frame)
        right_col.grid(row=0, column=2, sticky="n", padx=(10,0))
        
        # === LEFT COLUMN: Exposure Settings ===
        ttk.Label(left_col, text="Shutter Speed (μs):").grid(row=0, column=0, sticky="w", pady=(0,5))
        self.shutter_spinbox = ttk.Spinbox(left_col, from_=0, to=1000000, 
                                          textvariable=self.shutter_speed, width=12)
        self.shutter_spinbox.grid(row=1, column=0, sticky="w", pady=(0,2))
        ttk.Label(left_col, text="(0 = auto)", foreground="#666", font=("TkDefaultFont", 8)).grid(row=2, column=0, sticky="w", pady=(0,10))
        
        ttk.Checkbutton(left_col, text="Auto Exposure", variable=self.ae_enable).grid(row=3, column=0, sticky="w", pady=(0,10))
        
        ttk.Label(left_col, text="ISO:").grid(row=4, column=0, sticky="w", pady=(0,5))
        self.iso_combo = ttk.Combobox(left_col, textvariable=self.iso_value, state="readonly",
                                      values=["Auto", "100", "200", "400", "800", "1600"], width=12)
        self.iso_combo.grid(row=5, column=0, sticky="w", pady=(0,10))
        
        ttk.Label(left_col, text="Analogue Gain:").grid(row=6, column=0, sticky="w", pady=(0,5))
        self.gain_scale = ttk.Scale(left_col, from_=1.0, to=16.0, variable=self.analogue_gain,
                                    orient="horizontal", length=150)
        self.gain_scale.grid(row=7, column=0, sticky="w", pady=(0,10))
        
        ttk.Label(left_col, text="EV Compensation:").grid(row=8, column=0, sticky="w", pady=(0,5))
        self.ev_scale = ttk.Scale(left_col, from_=-2.0, to=2.0, variable=self.ev_compensation,
                                 orient="horizontal", length=150)
        self.ev_scale.grid(row=9, column=0, sticky="w", pady=(0,10))
        
        ttk.Label(left_col, text="Brightness:").grid(row=10, column=0, sticky="w", pady=(0,5))
        self.brightness_scale = ttk.Scale(left_col, from_=-1.0, to=1.0, variable=self.brightness,
                                         orient="horizontal", length=150)
        self.brightness_scale.grid(row=11, column=0, sticky="w", pady=(0,2))
        self.brightness_value_var = tk.StringVar()
        ttk.Label(left_col, textvariable=self.brightness_value_var, foreground="#666").grid(row=12, column=0, sticky="w")
        
        # === MIDDLE COLUMN: White Balance ===
        ttk.Checkbutton(middle_col, text="Auto White Balance", variable=self.awb_enable).grid(row=0, column=0, sticky="w", pady=(0,10))
        
        ttk.Label(middle_col, text="WB Red Gain:").grid(row=1, column=0, sticky="w", pady=(0,5))
        self.wb_r_scale = ttk.Scale(middle_col, from_=0.5, to=3.5, variable=self.wb_red_gain,
                                    orient="horizontal", length=150)
        self.wb_r_scale.grid(row=2, column=0, sticky="w", pady=(0,10))
        
        ttk.Label(middle_col, text="WB Blue Gain:").grid(row=3, column=0, sticky="w", pady=(0,5))
        self.wb_b_scale = ttk.Scale(middle_col, from_=0.5, to=3.5, variable=self.wb_blue_gain,
                                    orient="horizontal", length=150)
        self.wb_b_scale.grid(row=4, column=0, sticky="w")
        
        # === RIGHT COLUMN: Image Adjustments ===
        ttk.Label(right_col, text="Contrast:").grid(row=0, column=0, sticky="w", pady=(0,5))
        self.contrast_scale = ttk.Scale(right_col, from_=0.0, to=2.0, variable=self.contrast,
                                       orient="horizontal", length=150)
        self.contrast_scale.grid(row=1, column=0, sticky="w", pady=(0,10))
        
        ttk.Label(right_col, text="Saturation:").grid(row=2, column=0, sticky="w", pady=(0,5))
        self.saturation_scale = ttk.Scale(right_col, from_=0.0, to=2.0, variable=self.saturation,
                                         orient="horizontal", length=150)
        self.saturation_scale.grid(row=3, column=0, sticky="w", pady=(0,10))
        
        ttk.Label(right_col, text="Sharpness:").grid(row=4, column=0, sticky="w", pady=(0,5))
        self.sharpness_scale = ttk.Scale(right_col, from_=0.0, to=2.0, variable=self.sharpness,
                                        orient="horizontal", length=150)
        self.sharpness_scale.grid(row=5, column=0, sticky="w")
        
        # Apply button spanning all columns
        self.btn_apply_settings = ttk.Button(self.camera_controls_frame, text="Apply Settings",
                                            command=self.apply_camera_settings)
        self.btn_apply_settings.grid(row=1, column=0, columnspan=3, pady=(15, 0))

        # Keep a live readout of brightness value
        def update_brightness_display(*args):
            self.brightness_value_var.set(f"{self.brightness.get():.2f}")

        self.brightness.trace("w", update_brightness_display)
        update_brightness_display()

        # Resolution controls
        self.resolution_frame = ttk.LabelFrame(right_column, text="Resolution", padding=10)
        ttk.Label(self.resolution_frame, text="Preview Resolution:").grid(row=0, column=0, sticky="w")
        self.preview_res_combo = ttk.Combobox(
            self.resolution_frame,
            textvariable=self.preview_resolution,
            values=RESOLUTION_PRESETS,
            state="readonly",
            width=12,
        )
        self.preview_res_combo.grid(row=0, column=1, sticky="w", padx=(5, 0))
        ttk.Label(self.resolution_frame, text="Capture Resolution:").grid(row=1, column=0, sticky="w")
        self.capture_res_combo = ttk.Combobox(
            self.resolution_frame,
            textvariable=self.capture_resolution,
            values=RESOLUTION_PRESETS,
            state="readonly",
            width=12,
        )
        self.capture_res_combo.grid(row=1, column=1, sticky="w", padx=(5, 0))

        # Hint
        ttk.Label(
            self.resolution_frame,
            text="Preview affects live view FPS; capture uses full still resolution.",
            foreground="#666",
        ).grid(row=2, column=0, columnspan=2, sticky="w", pady=(6, 0))

        # Camera info
        self.camera_info_frame = ttk.LabelFrame(right_column, text="Camera Information", padding=10)
        self.camera_info_text = tk.Text(self.camera_info_frame, height=10, width=60, state="disabled")
        self.camera_info_text.grid(row=0, column=0, sticky="nsew")
        info_scrollbar = ttk.Scrollbar(self.camera_info_frame, orient="vertical", command=self.camera_info_text.yview)
        info_scrollbar.grid(row=0, column=1, sticky="ns")
        self.camera_info_text.config(yscrollcommand=info_scrollbar.set)

        # Layout right column (camera settings)
        self.camera_preview_frame.pack(fill="x", pady=(0,5))
        self.camera_controls_frame.pack(fill="x", pady=5)
        self.resolution_frame.pack(fill="x", pady=5)
        self.camera_info_frame.pack(fill="both", expand=True, pady=(5,0))

    def _layout_gui(self):
        """Layout the main GUI components - converted to pack()"""
        # Status bar with window controls
        self.status_frame = ttk.Frame(self)
        self.status_frame.pack(side="bottom", fill="x", padx=10, pady=(5,10))
        
        self.status_bar.pack(side="left", fill="x", expand=True, in_=self.status_frame)
        
        # Add always-on-top toggle button (starts as pinned)
        self.topmost_button = ttk.Button(self.status_frame, text="📌", width=3, 
                                        command=self._toggle_topmost_display)
        self.topmost_button.pack(side="right", padx=(5,0), in_=self.status_frame)
        
        # Update initial status to show window is pinned
        self.after(100, self._update_initial_status)

    def toggle_preview(self):
        """Toggle camera preview for both manual and calibration tabs"""
        if not PICAMERA2_OK:
            self.status_var.set("picamera2 is not installed.")
            return
        if not PIL_OK:
            self.status_var.set("PIL (pillow) is not installed.")
            return

        if not self.preview_on:
            self.start_preview()
            self.btn_preview.config(text="Stop Preview")
            if hasattr(self, 'cal_btn_preview'):
                self.cal_btn_preview.config(text="Stop Preview")
            if hasattr(self, 'camera_preview_btn'):
                self.camera_preview_btn.config(text="Stop Preview")
        else:
            self.stop_preview()
            self.btn_preview.config(text="Start Preview")
            if hasattr(self, 'cal_btn_preview'):
                self.cal_btn_preview.config(text="Start Preview")
            if hasattr(self, 'camera_preview_btn'):
                self.camera_preview_btn.config(text="Start Preview")
        
        # Update manual tab status
        self._update_manual_status()

    def start_preview(self):
        try:
            if self.camera is None:
                self.camera = Picamera2()
            # Build preview configuration from selected resolution
            try:
                w, h = map(int, self.preview_resolution.get().lower().split("x"))
            except Exception:
                w, h = CAMERA_RESOLUTION
            self.preview_config = self.camera.create_preview_configuration(
                main={"size": (w, h), "format": "RGB888"}
            )
            self.camera.configure(self.preview_config)
            self.camera.start()
            self.preview_on = True
            self.preview_update_active = True
            self.after(200, self._update_preview_frame)
        except Exception as e:
            self.status_var.set(f"Camera error: {e}")

    def stop_preview(self):
        self.preview_update_active = False
        if self.camera is not None:
            self.camera.stop()
        self.preview_on = False

    def _update_preview_frame(self):
        """Update camera preview in both manual and calibration tabs"""
        if not self.preview_on or not self.preview_update_active:
            return
        try:
            with self.camera_lock:
                frame = self.camera.capture_array("main")
            # Picamera2 preview is configured as RGB888; avoid expensive cv2 conversion
            try:
                # Apply optional color swap (common if frame is actually BGR)
                if PREVIEW_COLOR_SWAP and hasattr(frame, 'shape') and len(frame.shape) == 3 and frame.shape[2] == 3:
                    arr = frame[:, :, ::-1]
                else:
                    arr = frame
                image = Image.fromarray(arr, 'RGB')
            except Exception:
                # Fallback if mode not matching
                image = Image.fromarray(frame)

            if image.size != CAMERA_RESOLUTION:
                image = image.resize(CAMERA_RESOLUTION, Image.LANCZOS)
            
            photo = ImageTk.PhotoImage(image)
            self.preview_image = photo
            
            # Update all preview labels
            self.preview_label.config(image=photo)
            self.cal_preview_label.config(image=photo)
            if hasattr(self, 'camera_preview_label'):
                self.camera_preview_label.config(image=photo)
            
            # Throttle updates a bit to reduce CPU load
            self.after(150, self._update_preview_frame)
        except Exception as e:
            self.status_var.set(f"Preview error: {e}")
            self.stop_preview()

    def send_motor_command(self, command, wait_for_done=False, timeout=10):
        """Send command to Arduino and block until completion.

        The Arduino sketch prints "OK" only after finishing a motion command
        (ROTATE/MOVE). For queries like GET_POS, it returns a numeric value.
        We wait until we see "OK"/"ERR"/number or the timeout elapses.

        Args:
            command (str): Command string to send (e.g., "ROTATE 1 90 CW")
            wait_for_done (bool): Ignored; retained for backward compatibility
            timeout (int): Max seconds to wait for a response (default 10)

        Returns:
            bool or str: True if successful (OK), False on ERR/timeout,
                         or a string for GET_POS numeric responses.
        """
        try:
            if not self.ser or not self.ser.is_open:
                self.status_var.set("Serial connection not available")
                return False

            # Clear any stale input to avoid consuming previous lines
            try:
                self.ser.reset_input_buffer()
            except Exception:
                pass

            # Send command
            full_command = command.strip() + "\n"
            print(f"DEBUG: Sending to Arduino: {repr(full_command)}")  # Debug output
            self.ser.write(full_command.encode())
            self.ser.flush()

            is_query = command.strip().upper().startswith("GET_POS")
            start = time.time()
            last_seen = ""

            while time.time() - start < timeout:
                line = self.ser.readline().decode(errors='ignore').strip()
                if not line:
                    time.sleep(0.01)
                    continue
                print(f"DEBUG: Arduino response: {repr(line)}")  # Debug output
                last_seen = line

                # Skip boot or banner lines
                if line.lower().startswith("ready"):
                    continue

                if is_query:
                    # Return raw numeric string; caller parses to float
                    return line

                if line == "OK" or line.startswith("OK "):
                    return True
                if line.startswith("ERR"):
                    self.status_var.set(f"Arduino error: {line}")
                    return False

            # Timeout
            self.status_var.set(f"Timeout waiting for response to '{command}'. Last: '{last_seen}'")
            return False
        except Exception as e:
            self.status_var.set(f"Serial error: {e}")
            return False
    
    def get_motor_position(self, motor):
        """Get current position from Arduino.
        
        Queries the Arduino for the current position of the specified motor.
        
        Args:
            motor (int): Motor number (1 for rotation, 2 for linear)
            
        Returns:
            float or None: Current position, or None if query failed
        """
        command = f"GET_POS {motor}"
        response = self.send_motor_command(command)
        if response and response != True and response != False:
            try:
                return float(response)
            except ValueError:
                pass
        return None
    
    def zero_motor_position(self, motor):
        """Zero the motor position on Arduino"""
        command = f"ZERO {motor}"
        return self.send_motor_command(command)

    def toggle_motor_power(self):
        """Toggle the 24V motor power relay on/off"""
        if self.motor_power_on:
            # Turn OFF
            if self.send_motor_command("POWER OFF"):
                self.motor_power_on = False
                self.motor_power_status_var.set("OFF")
                self.btn_motor_power.config(text="Turn ON")
                self.status_var.set("Motor power OFF")
            else:
                self.status_var.set("Failed to turn off motor power")
        else:
            # Turn ON
            if self.send_motor_command("POWER ON"):
                self.motor_power_on = True
                self.motor_power_status_var.set("ON")
                self.btn_motor_power.config(text="Turn OFF")
                self.status_var.set("Motor power ON")
            else:
                self.status_var.set("Failed to turn on motor power")

    def motor1_ccw(self):
        try:
            step = float(self.motor1_step_var.get())
            # Apply direction inversion if enabled
            direction = "CCW" if self.rotation_invert.get() else "CW"
            command = f"ROTATE 1 {step} {direction}"
            
            if self.send_motor_command(command):
                self.motor1_position_deg -= step
                self.motor1_pos_var.set(f"{self.motor1_position_deg:.1f}°")
                self.status_var.set(f"Motor 1: Rotated {step}° CCW")
            else:
                self.status_var.set("Failed to rotate motor 1")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 1")

    def motor1_cw(self):
        try:
            step = float(self.motor1_step_var.get())
            # Apply direction inversion if enabled
            direction = "CW" if self.rotation_invert.get() else "CCW"
            command = f"ROTATE 1 {step} {direction}"
            
            if self.send_motor_command(command):
                self.motor1_position_deg += step
                self.motor1_pos_var.set(f"{self.motor1_position_deg:.1f}°")
                self.status_var.set(f"Motor 1: Rotated {step}° CW")
            else:
                self.status_var.set("Failed to rotate motor 1")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 1")

    def motor1_home(self):
        if self.zero_motor_position(1):
            self.motor1_position_deg = 0.0
            self.motor1_pos_var.set("0.0°")
            self.status_var.set("Motor 1: Position zeroed")
        else:
            self.status_var.set("Failed to zero motor 1")

    def motor2_up(self):
        try:
            step = float(self.motor2_step_var.get())
            command = f"MOVE 2 {step} FORWARD"
            
            if self.send_motor_command(command):
                self.motor2_position_mm += step
                self.motor2_pos_var.set(f"{self.motor2_position_mm:.1f}mm")
                self.status_var.set(f"Motor 2: Moved {step}mm forward")
            else:
                self.status_var.set("Failed to move motor 2")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 2")

    def motor2_down(self):
        try:
            step = float(self.motor2_step_var.get())
            command = f"MOVE 2 {step} BACKWARD"
            
            if self.send_motor_command(command):
                self.motor2_position_mm -= step
                self.motor2_pos_var.set(f"{self.motor2_position_mm:.1f}mm")
                self.status_var.set(f"Motor 2: Moved {step}mm backward")
            else:
                self.status_var.set("Failed to move motor 2")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 2")

    def motor2_home(self):
        if self.zero_motor_position(2):
            self.motor2_position_mm = 0.0
            self.motor2_pos_var.set("0.0mm")
            self.status_var.set("Motor 2: Position zeroed")
        else:
            self.status_var.set("Failed to zero motor 2")

    def motor3_up(self):
        """Tilt specimen upward (positive angle)"""
        try:
            step = float(self.motor3_step_var.get())
            command = f"TILT 3 {step} UP"
            
            if self.send_motor_command(command):
                self.motor3_position_deg += step
                self.motor3_pos_var.set(f"{self.motor3_position_deg:.1f}°")
                self.status_var.set(f"Motor 3: Tilted {step}° up")
            else:
                self.status_var.set("Failed to tilt motor 3")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 3")

    def motor3_down(self):
        """Tilt specimen downward (negative angle)"""
        try:
            step = float(self.motor3_step_var.get())
            command = f"TILT 3 {step} DOWN"
            
            if self.send_motor_command(command):
                self.motor3_position_deg -= step
                self.motor3_pos_var.set(f"{self.motor3_position_deg:.1f}°")
                self.status_var.set(f"Motor 3: Tilted {step}° down")
            else:
                self.status_var.set("Failed to tilt motor 3")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 3")

    def motor3_home(self):
        """Zero the tilt axis position"""
        if self.zero_motor_position(3):
            self.motor3_position_deg = 0.0
            self.motor3_pos_var.set("0.0°")
            self.status_var.set("Motor 3: Position zeroed")
        else:
            self.status_var.set("Failed to zero motor 3")

    def _maintain_focus(self):
        """Periodically maintain window on top to prevent jumping to background."""
        if self._focus_maintenance_active:
            try:
                if self.winfo_exists():
                    # Keep window on top without forcing focus (allows keyboard input)
                    self.lift()
            except:
                pass
            # Schedule next maintenance check
            self.after(1000, self._maintain_focus)

    def _on_window_click(self, event):
        """Handle window clicks - ensure window stays in front."""
        try:
            # Only lift window, don't force focus (allows Entry widgets to work)
            self.lift()
        except:
            pass

    def _on_focus_lost(self, event):
        """Handle when window loses focus."""
        # Disabled - was interfering with keyboard input in Entry widgets
        pass

    def bring_to_front(self):
        """Programmatically bring window to front and focus it."""
        try:
            self.lift()
            self.focus_force()
        except:
            pass
    
    def toggle_always_on_top(self):
        """Toggle the always-on-top behavior."""
        try:
            current_topmost = self.wm_attributes('-topmost')
            self.wm_attributes('-topmost', not current_topmost)
            return not current_topmost
        except:
            return False

    def _update_initial_status(self):
        """Update status to show initial pinned state."""
        try:
            current_status = self.status_var.get()
            if "pinned on top" not in current_status:
                self.status_var.set(current_status + " | Window pinned on top")
        except:
            pass

    def _toggle_topmost_display(self):
        """Toggle topmost and update button display."""
        try:
            new_state = self.toggle_always_on_top()
            # Update button appearance
            if new_state:
                self.topmost_button.configure(text="📌")
                current_status = self.status_var.get().replace(" | Window pinned on top", "")
                self.status_var.set(current_status + " | Window pinned on top")
            else:
                self.topmost_button.configure(text="📍")  
                current_status = self.status_var.get().replace(" | Window pinned on top", "")
                self.status_var.set(current_status)
        except:
            pass
    
    def _update_manual_status(self):
        """Update status displays on the manual control tab."""
        try:
            # Update Arduino status
            if hasattr(self, 'ser') and self.ser is not None:
                self.arduino_status_var.set("Connected")
            else:
                self.arduino_status_var.set("Not Connected")
            
            # Update camera status
            if hasattr(self, 'preview_on') and self.preview_on:
                self.camera_status_var.set("Preview Active")
            elif hasattr(self, 'camera') and self.camera is not None:
                self.camera_status_var.set("Ready")
            else:
                self.camera_status_var.set("Not Initialized")
        except AttributeError:
            # Variables not yet created during initialization
            pass
        except Exception:
            pass

    def on_close(self):
        # Stop focus maintenance
        self._focus_maintenance_active = False
        
        # Save settings one last time
        try:
            self._save_app_settings()
        except Exception:
            pass
        # Stop any running capture
        if self.capture_running:
            self.capture_running = False
            if self.capture_thread and self.capture_thread.is_alive():
                self.capture_thread.join(timeout=2)
        
        if self.preview_on:
            self.stop_preview()
        
        if self.camera is not None:
            try:
                self.camera.close()
            except:
                pass
        
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except:
                pass
        
        # Release window grab before destroying
        try:
            self.grab_release()
        except:
            pass
        
        self.destroy()

    # ──────────────────────────────────────────────────────────────────
    # App-level settings persistence
    # ──────────────────────────────────────────────────────────────────
    def _settings_path(self):
        # Store next to output dir as a simple, discoverable location
        base = self.output_dir.get() or DEFAULT_OUTPUT_DIR
        try:
            os.makedirs(base, exist_ok=True)
        except Exception:
            base = DEFAULT_OUTPUT_DIR
            os.makedirs(base, exist_ok=True)
        return os.path.join(base, "app_settings.json")

    def _load_app_settings(self):
        try:
            path = os.path.join(DEFAULT_OUTPUT_DIR, "app_settings.json")
            if os.path.exists(path):
                with open(path, 'r') as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    if 'rotation_scale' in data:
                        self.rotation_scale.set(float(data['rotation_scale']))
                    if 'rotation_invert' in data:
                        self.rotation_invert.set(bool(data['rotation_invert']))
        except Exception:
            pass

    def _save_app_settings(self):
        try:
            data = {
                'rotation_scale': float(self.rotation_scale.get()),
                'rotation_invert': bool(self.rotation_invert.get()),
            }
            with open(self._settings_path(), 'w') as f:
                json.dump(data, f, indent=2)
        except Exception:
            pass

# =====================================================================
# CALIBRATION METHODS
# =====================================================================
    
    def start_calibration(self):
        """Start the calibration wizard.
        
        Initiates the step-by-step calibration process for mapping focus positions
        at cardinal angles (0°, 90°, 180°, 270°). Resets calibration data and
        guides user through the workflow.
        
        The calibration process:
            1. Move to each cardinal angle
            2. Set near and far focus positions manually
            3. Capture positions for interpolation
            4. Validate complete dataset
            
        Note:
            Disables start button and enables capture/next buttons during process.
        """
        self.calibration_data = {}
        self.current_calibration_angle = 0
        self.current_calibration_focus = "near"
        self.current_angle_var.set(f"{CALIBRATION_ANGLES[0]}°")
        self.current_focus_var.set("Near")
        
        # Enable/disable buttons
        self.btn_start_calibration.config(state="disabled")
        self.btn_capture_position.config(state="normal")
        self.btn_next_step.config(state="normal")
        
        # Clear and update progress
        self.calibration_progress_text.delete(1.0, tk.END)
        self.log_calibration("Calibration started. Position specimen at 0° and set near focus position.")

        # Move to 0 degrees without freezing UI
        self.move_to_angle_async(0)
        
    def capture_calibration_position(self):
        """Capture the current linear position for calibration.
        
        Records the current motor 2 position as either the near or far focus
        point for the current calibration angle. Advances the calibration
        workflow state.
        
        State Management:
            - Stores position in calibration_data[angle][focus_type]
            - Advances from "near" to "far" focus for current angle
            - Updates UI to reflect current calibration step
        """
        angle = CALIBRATION_ANGLES[self.current_calibration_angle]
        focus_type = self.current_calibration_focus
        position = self.motor2_position_mm
        
        if angle not in self.calibration_data:
            self.calibration_data[angle] = {}
        
        self.calibration_data[angle][focus_type] = position
        self.log_calibration(f"Captured {focus_type} position at {angle}°: {position:.2f}mm")
        
        # Check if we can proceed
        if focus_type == "near":
            self.current_calibration_focus = "far"
            self.current_focus_var.set("Far")
            self.log_calibration("Now set the far focus position and capture again.")
        else:
            # Both near and far captured for this angle
            self.log_calibration(f"Angle {angle}° calibration complete.")
            
    def next_calibration_step(self):
        """Move to next step in calibration.
        
        Advances the calibration wizard to the next cardinal angle after
        verifying that both near and far positions have been captured
        for the current angle.
        
        Flow Control:
            - Validates current angle completion
            - Moves to next cardinal angle (0° → 90° → 180° → 270°)
            - Calls complete_calibration() when all angles done
            - Shows warning if current angle incomplete
        """
        current_angle = CALIBRATION_ANGLES[self.current_calibration_angle]
        
        # Check if current angle is complete
        if (current_angle in self.calibration_data and 
            "near" in self.calibration_data[current_angle] and 
            "far" in self.calibration_data[current_angle]):
            
            # Move to next angle
            self.current_calibration_angle += 1
            
            if self.current_calibration_angle >= len(CALIBRATION_ANGLES):
                # Calibration complete
                self.complete_calibration()
                return
            
            # Set up next angle
            next_angle = CALIBRATION_ANGLES[self.current_calibration_angle]
            self.current_calibration_focus = "near"
            self.current_angle_var.set(f"{next_angle}°")
            self.current_focus_var.set("Near")
            
            self.log_calibration(f"Moving to {next_angle}°. Set near focus position.")
            self.move_to_angle_async(next_angle)
        else:
            messagebox.showwarning("Calibration", "Please capture both near and far positions for current angle.")
    
    def complete_calibration(self):
        """Complete the calibration process.
        
        Finalizes calibration by setting the calibrated flag and updating
        the UI state. Enables capture functionality and disables calibration
        workflow buttons.
        
        UI Updates:
            - Sets status to "Calibrated" with green color
            - Enables save/capture buttons
            - Disables calibration workflow buttons
        """
        # Validate XMP settings before completing
        try:
            lens_dist = float(self.lens_to_object_mm.get())
            rail_angle = float(self.rail_to_horizon_deg.get())
            
            if lens_dist <= 0:
                messagebox.showwarning("XMP Settings", "Please enter a valid lens to object distance (> 0)")
                return
                
        except ValueError:
            messagebox.showwarning("XMP Settings", "Please enter valid numeric values for XMP pose settings")
            return
        
        self.is_calibrated = True
        self.calibration_status_var.set("Calibrated")
        
        # Update button states
        self.btn_start_calibration.config(state="normal")
        self.btn_capture_position.config(state="disabled")
        self.btn_next_step.config(state="disabled")
        self.btn_save_calibration.config(state="normal")
        
        self.log_calibration("Calibration complete! XMP pose settings validated.")
        self.log_calibration("You can now run capture sequences with XMP sidecars.")
        
        # Update status display color
        for widget in self.calibration_status_frame.winfo_children():
            if isinstance(widget, ttk.Label) and str(widget.cget("textvariable")) == str(self.calibration_status_var):
                widget.config(foreground="green")
                break

    def reset_calibration(self):
        """Reset calibration workflow to initial state and move to 0°."""
        # Clear data and state
        self.calibration_data = {}
        self.is_calibrated = False
        self.current_calibration_angle = 0
        self.current_calibration_focus = "near"
        self.current_angle_var.set(f"{CALIBRATION_ANGLES[0]}°")
        self.current_focus_var.set("Near")
        self.calibration_status_var.set("Not Calibrated")
        
        # Reset XMP settings to defaults
        self.lens_to_object_mm.set(250.0)
        self.rail_to_horizon_deg.set(0.0)
        self.focal_length_35mm.set(50.0)
        self.xmp_position_scale.set(1.0)
        self.principal_point_u.set(0.0)
        self.principal_point_v.set(0.0)
        self.aspect_ratio.set(1.0)
        self.skew.set(0.0)
        self.distortion_k1.set(0.0)
        self.distortion_k2.set(0.0)
        self.distortion_k3.set(0.0)
        self.distortion_p1.set(0.0)
        self.distortion_p2.set(0.0)
        self.distortion_k4.set(0.0)

        # Update status label color back to red
        for widget in self.calibration_status_frame.winfo_children():
            if isinstance(widget, ttk.Label) and str(widget.cget("textvariable")) == str(self.calibration_status_var):
                widget.config(foreground="red")
                break

        # Enable/disable buttons appropriate for starting over
        self.btn_start_calibration.config(state="normal")
        self.btn_capture_position.config(state="disabled")
        self.btn_next_step.config(state="disabled")
        self.btn_save_calibration.config(state="disabled")

        # Clear progress log and prompt user
        self.calibration_progress_text.delete(1.0, tk.END)
        self.log_calibration("Calibration reset. Click 'Start Calibration' to begin again at 0°.")

        # Move to 0° asynchronously so the UI stays responsive
        self.move_to_angle_async(0)
    
    def save_calibration(self):
        """Save calibration data to file.
        
        Exports the current calibration data to a JSON file using a file
        dialog. The saved file can be loaded later to restore calibration
        without repeating the wizard.
        
        File Format:
            JSON structure: {
                "focus_positions": {angle: {"near": mm_pos, "far": mm_pos}, ...},
                "xmp_settings": {pose and camera calibration parameters}
            }
        """
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Save Calibration Data"
        )
        
        if filename:
            try:
                # Prepare calibration data with all XMP and camera settings
                cal_data = {
                    "focus_positions": self.calibration_data,
                    "xmp_settings": {
                        # Pose settings
                        "lens_to_object_mm": float(self.lens_to_object_mm.get()),
                        "rail_to_horizon_deg": float(self.rail_to_horizon_deg.get()),
                        "focal_length_35mm": float(self.focal_length_35mm.get()),
                        "xmp_position_scale": float(self.xmp_position_scale.get()),
                        # Intrinsics
                        "principal_point_u": float(self.principal_point_u.get()),
                        "principal_point_v": float(self.principal_point_v.get()),
                        "aspect_ratio": float(self.aspect_ratio.get()),
                        "skew": float(self.skew.get()),
                        # Distortion
                        "distortion_model": str(self.distortion_model.get()),
                        "distortion_k1": float(self.distortion_k1.get()),
                        "distortion_k2": float(self.distortion_k2.get()),
                        "distortion_k3": float(self.distortion_k3.get()),
                        "distortion_p1": float(self.distortion_p1.get()),
                        "distortion_p2": float(self.distortion_p2.get()),
                        "distortion_k4": float(self.distortion_k4.get())
                    }
                }
                
                with open(filename, 'w') as f:
                    json.dump(cal_data, f, indent=2)
                self.log_calibration(f"Calibration saved to {filename}")
                messagebox.showinfo("Save", "Calibration data saved successfully!")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save calibration: {e}")
    
    def load_calibration(self):
        """Load calibration data from file.
        
        Imports calibration data from a JSON file and validates the format.
        If valid, enables capture functionality and updates the UI state.
        
        Validation:
            - Checks all cardinal angles present
            - Verifies near/far positions for each angle
            - Shows error dialog if validation fails
        """
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Load Calibration Data"
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    loaded = json.load(f)

                # Handle both old format (direct angle mapping) and new format (with xmp_settings)
                if "focus_positions" in loaded:
                    # New format with XMP settings
                    self.calibration_data = self._normalize_calibration_data(loaded["focus_positions"])
                    
                    # Load XMP settings if present
                    if "xmp_settings" in loaded:
                        xmp_settings = loaded["xmp_settings"]
                        # Pose settings
                        if "lens_to_object_mm" in xmp_settings:
                            self.lens_to_object_mm.set(float(xmp_settings["lens_to_object_mm"]))
                        if "rail_to_horizon_deg" in xmp_settings:
                            self.rail_to_horizon_deg.set(float(xmp_settings["rail_to_horizon_deg"]))
                        if "focal_length_35mm" in xmp_settings:
                            self.focal_length_35mm.set(float(xmp_settings["focal_length_35mm"]))
                        if "xmp_position_scale" in xmp_settings:
                            self.xmp_position_scale.set(float(xmp_settings["xmp_position_scale"]))
                        # Intrinsics
                        if "principal_point_u" in xmp_settings:
                            self.principal_point_u.set(float(xmp_settings["principal_point_u"]))
                        if "principal_point_v" in xmp_settings:
                            self.principal_point_v.set(float(xmp_settings["principal_point_v"]))
                        if "aspect_ratio" in xmp_settings:
                            self.aspect_ratio.set(float(xmp_settings["aspect_ratio"]))
                        if "skew" in xmp_settings:
                            self.skew.set(float(xmp_settings["skew"]))
                        # Distortion
                        if "distortion_model" in xmp_settings:
                            self.distortion_model.set(str(xmp_settings["distortion_model"]))
                        if "distortion_k1" in xmp_settings:
                            self.distortion_k1.set(float(xmp_settings["distortion_k1"]))
                        if "distortion_k2" in xmp_settings:
                            self.distortion_k2.set(float(xmp_settings["distortion_k2"]))
                        if "distortion_k3" in xmp_settings:
                            self.distortion_k3.set(float(xmp_settings["distortion_k3"]))
                        if "distortion_p1" in xmp_settings:
                            self.distortion_p1.set(float(xmp_settings["distortion_p1"]))
                        if "distortion_p2" in xmp_settings:
                            self.distortion_p2.set(float(xmp_settings["distortion_p2"]))
                        if "distortion_k4" in xmp_settings:
                            self.distortion_k4.set(float(xmp_settings["distortion_k4"]))
                else:
                    # Old format (direct angle mapping) - maintain compatibility
                    self.calibration_data = self._normalize_calibration_data(loaded)

                # Validate calibration data
                if self.validate_calibration_data():
                    self.is_calibrated = True
                    self.calibration_status_var.set("Calibrated")
                    self.btn_save_calibration.config(state="normal")
                    self.log_calibration(f"Calibration loaded from {filename}")
                    messagebox.showinfo("Load", "Calibration data loaded successfully!")
                    
                    # Update status display color
                    for widget in self.calibration_status_frame.winfo_children():
                        if isinstance(widget, ttk.Label) and str(widget.cget("textvariable")) == str(self.calibration_status_var):
                            widget.config(foreground="green")
                            break
                else:
                    messagebox.showerror("Error", "Invalid calibration data format")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to load calibration: {e}")
    
    def validate_calibration_data(self):
        """Validate that calibration data is complete.
        
        Checks that calibration data contains all required cardinal angles
        (0°, 90°, 180°, 270°) and that each angle has both near and far
        focus positions defined.
        
        Returns:
            bool: True if calibration data is complete and valid
        """
        try:
            for angle in CALIBRATION_ANGLES:
                if angle not in self.calibration_data:
                    return False
                entry = self.calibration_data[angle]
                if not isinstance(entry, dict):
                    return False
                if "near" not in entry or "far" not in entry:
                    return False
                # Ensure values are numbers
                float(entry["near"])
                float(entry["far"])
            return True
        except Exception:
            return False

    def _normalize_calibration_data(self, data):
        """Normalize loaded calibration data to use integer angle keys and float values.

        Accepts dicts with string keys like "0", "90" and ensures the resulting
        structure is {0: {"near": float, "far": float}, ...}.
        """
        normalized = {}
        if not isinstance(data, dict):
            return normalized
        for k, v in data.items():
            try:
                angle = int(k)
            except Exception:
                # Skip keys that are not integer-like
                continue
            if not isinstance(v, dict):
                continue
            try:
                near = float(v.get("near"))
                far = float(v.get("far"))
            except Exception:
                continue
            normalized[angle] = {"near": near, "far": far}
        return normalized
    
    def log_calibration(self, message):
        """Add message to calibration log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.calibration_progress_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.calibration_progress_text.see(tk.END)
    
    def move_to_angle(self, target_angle):
        """Move motor 1 to specific angle.
        
        Commands the rotation motor to move to the specified angle position.
        Calculates the required rotation direction and amount automatically.
        
        Args:
            target_angle (float): Target angle in degrees (0-360)
            
        Returns:
            bool: True if movement successful, False otherwise
            
        Note:
            Updates internal position tracking and GUI display
        """
        current_angle = self.motor1_position_deg
        rotation_needed = target_angle - current_angle
        
        if abs(rotation_needed) > 0.01:
            direction = "CW" if rotation_needed > 0 else "CCW"
            amount = abs(rotation_needed)

            command = f"ROTATE 1 {amount} {direction}"
            if self.send_motor_command(command, wait_for_done=True):
                self.motor1_position_deg = target_angle
                self.after(0, self.motor1_pos_var.set, f"{target_angle:.1f}°")
                self.after(0, self.status_var.set, f"Moved to {target_angle}°")
                return True
            else:
                self.after(0, self.status_var.set, f"Failed to move to {target_angle}°")
                return False
        return True

    def move_to_angle_async(self, target_angle):
        """Non-blocking move for rotation motor (used in calibration UI)."""
        def _worker():
            self.move_to_angle(target_angle)
        threading.Thread(target=_worker, daemon=True).start()
    
    def move_to_elevation(self, target_elevation):
        """Move tilt motor to specific elevation angle (synchronous).
        
        Args:
            target_elevation: Target angle in degrees (positive = tilt up)
            
        Returns:
            bool: True if movement succeeded, False otherwise
        """
        current_elevation = self.motor3_position_deg
        tilt_needed = target_elevation - current_elevation
        
        if abs(tilt_needed) > 0.01:
            direction = "UP" if tilt_needed > 0 else "DOWN"
            amount = abs(tilt_needed)
            
            command = f"TILT 3 {amount} {direction}"
            if self.send_motor_command(command, wait_for_done=True):
                self.motor3_position_deg = target_elevation
                self.after(0, self.motor3_pos_var.set, f"{target_elevation:.1f}°")
                self.after(0, self.status_var.set, f"Tilted to {target_elevation}°")
                return True
            else:
                self.after(0, self.status_var.set, f"Failed to tilt to {target_elevation}°")
                return False
        return True
    
    def _update_scan_mode_ui(self):
        """Show/hide spherical scan settings based on scan mode selection"""
        if self.scan_mode.get() == "spherical":
            # Show spherical settings
            self.spherical_settings_label.grid(row=3, column=0, sticky="w")
            self.spherical_min_label.grid(row=3, column=1, sticky="w", padx=(5,2))
            self.spherical_min_spinbox.grid(row=3, column=1, sticky="w", padx=(30,0))
            self.spherical_max_label.grid(row=3, column=1, sticky="w", padx=(100,0))
            self.spherical_max_spinbox.grid(row=3, column=1, sticky="w", padx=(135,0))
            self.spherical_steps_label.grid(row=3, column=2, sticky="w", padx=(10,2))
            self.spherical_steps_spinbox.grid(row=3, column=2, sticky="w", padx=(55,0))
        else:
            # Hide spherical settings
            self.spherical_settings_label.grid_remove()
            self.spherical_min_label.grid_remove()
            self.spherical_min_spinbox.grid_remove()
            self.spherical_max_label.grid_remove()
            self.spherical_max_spinbox.grid_remove()
            self.spherical_steps_label.grid_remove()
            self.spherical_steps_spinbox.grid_remove()
    
    def interpolate_focus_position(self, angle, stack_position):
        """Interpolate focus position for given angle and stack position.
        
        Uses bilinear interpolation to calculate the optimal focus position
        based on calibrated cardinal points and the desired stack level.
        
        Algorithm:
            1. Find adjacent cardinal points for the given angle
            2. Interpolate near/far positions between cardinal points
            3. Interpolate between near/far for the stack position
            4. Handle 270° to 0° wrap-around case
        
        Args:
            angle (float): Target angle in degrees (0-360)
            stack_position (float): Stack level (0.0=near focus, 1.0=far focus)
            
        Returns:
            float: Interpolated linear rail position in millimeters
            
        Example:
            # Get focus position for 45° at middle stack
            pos = self.interpolate_focus_position(45.0, 0.5)
        """
        if not self.is_calibrated:
            return 0.0
        
        # Find the two calibration points to interpolate between
        angles = sorted(CALIBRATION_ANGLES)
        
        if angle in self.calibration_data:
            # Exact match
            near_pos = self.calibration_data[angle]["near"]
            far_pos = self.calibration_data[angle]["far"]
        else:
            # Interpolate between adjacent angles
            lower_angle = None
            upper_angle = None
            
            for i, cal_angle in enumerate(angles):
                if cal_angle <= angle:
                    lower_angle = cal_angle
                if cal_angle >= angle:
                    upper_angle = cal_angle
                    break
            
            if lower_angle is None:
                lower_angle = angles[-1]  # Wrap around
            if upper_angle is None:
                upper_angle = angles[0]   # Wrap around
            
            # Handle wrap-around case
            if lower_angle == upper_angle:
                near_pos = self.calibration_data[lower_angle]["near"]
                far_pos = self.calibration_data[lower_angle]["far"]
            else:
                # Linear interpolation between angles
                if upper_angle < lower_angle:  # Wrap-around case
                    upper_angle += 360
                
                angle_factor = (angle - lower_angle) / (upper_angle - lower_angle)
                
                near_lower = self.calibration_data[lower_angle]["near"]
                near_upper = self.calibration_data[upper_angle % 360]["near"]
                far_lower = self.calibration_data[lower_angle]["far"]
                far_upper = self.calibration_data[upper_angle % 360]["far"]
                
                near_pos = near_lower + (near_upper - near_lower) * angle_factor
                far_pos = far_lower + (far_upper - far_lower) * angle_factor
        
        # Interpolate between near and far for this stack position
        return near_pos + (far_pos - near_pos) * stack_position
    
    # =====================================================================
    # CAPTURE METHODS
    # =====================================================================
    
    def browse_output_dir(self):
        """Browse for output directory"""
        directory = filedialog.askdirectory(initialdir=self.output_dir.get())
        if directory:
            self.output_dir.set(directory)
    
    def test_capture(self):
        """Capture a single test image"""
        if not PICAMERA2_AVAILABLE:
            messagebox.showerror("Error", "Camera not available")
            return
        
        try:
            if self.camera is None or not self.camera.started:
                self.start_preview()
                time.sleep(1)  # Let camera stabilize
            
            # Apply camera settings
            self.apply_camera_settings()
            
            # Create test filename
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            test_dir = os.path.join(self.output_dir.get(), "test_captures")
            os.makedirs(test_dir, exist_ok=True)
            
            filename = f"test_{timestamp}.{self.image_format.get().lower()}"
            filepath = os.path.join(test_dir, filename)
            
            # Capture image
            self.capture_image(filepath)
            
            self.log_capture(f"Test capture saved: {filename}")
            messagebox.showinfo("Test Capture", f"Image saved to:\n{filepath}")
            
        except Exception as e:
            messagebox.showerror("Error", f"Test capture failed: {e}")
    
    def start_full_capture(self):
        """Start the full automated capture sequence.
        
        Launches a complete 360° capture session. Interprets values as:
        - Perspectives (angles): number of angles around the 360° sweep
        - Focus slices per angle: number of focus positions captured at each angle
        
        Workflow:
            1. Validate calibration and settings
            2. Create output directory structure
            3. Save capture metadata and calibration data
            4. For each perspective (angle):
                - Move turntable to angle
                - For each focus slice:
                    - Interpolate and move focus
                    - Wait for settling
                    - Capture image
            5. Update progress and log results
            
        Directory Structure:
            specimen_name/
                session_YYYYMMDD_HHMMSS/
                    metadata.json
                    stack_00/   # perspective 0 (angle 0 * angle_step)
                        stack_00_shot_000_angle_000.00.jpg   # focus slice 0
                        stack_00_shot_001_angle_000.00.jpg   # focus slice 1
                        ...
                    stack_01/   # perspective 1 (next angle)
                    ...
        
        Raises:
            ValueError: If calibration incomplete or invalid settings
        """
        if not self.is_calibrated:
            messagebox.showerror("Error", "Please complete calibration first")
            return
        
        if not PICAMERA2_AVAILABLE:
            messagebox.showerror("Error", "Camera not available")
            return
        
        # Validate settings
        if not self.specimen_name.get().strip():
            messagebox.showerror("Error", "Please enter a specimen name")
            return
        
        # Confirm start
        total_images = self.stacks_count.get() * self.shots_per_stack.get()
        message = f"Start capture sequence?\n\n"
        message += f"Specimen: {self.specimen_name.get()}\n"
        message += f"Perspectives (angles): {self.stacks_count.get()}\n"
        message += f"Focus slices per angle: {self.shots_per_stack.get()}\n"
        message += f"Angle step: {360.0 / max(1, self.stacks_count.get()):.2f}°\n"
        message += f"Total images: {total_images}\n"
        message += f"Estimated time: {total_images * (self.settle_delay.get() + 2):.0f} seconds"
        
        if not messagebox.askyesno("Confirm Capture", message):
            return
        
        # Start capture in separate thread
        self.capture_running = True
        self.btn_start_capture.config(state="disabled")
        self.btn_stop_capture.config(state="normal")
        
        self.capture_thread = threading.Thread(target=self._run_capture_sequence)
        self.capture_thread.daemon = True
        self.capture_thread.start()
    
    def stop_capture(self):
        """Stop the capture sequence"""
        self.capture_running = False
        self.btn_start_capture.config(state="normal")
        self.btn_stop_capture.config(state="disabled")
        self.log_capture("Capture sequence stopped by user")
    
    def _run_capture_sequence(self):
        """Run the full capture sequence in a separate thread
        
        This method executes the complete capture workflow in a background thread
        to prevent GUI freezing. Handles all motor movements, camera operations,
        and file management.
        
        Threading Safety:
            - Updates GUI using self.after() calls
            - Checks self.capture_running flag for user interruption
            - Sets daemon thread for clean shutdown
            
        Error Handling:
            - Graceful recovery from motor/camera errors
            - User feedback via progress updates and logging
            - Proper cleanup in finally block
        """
        try:
            # Create output directory structure
            specimen_dir = os.path.join(self.output_dir.get(), self.specimen_name.get())
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_dir = os.path.join(specimen_dir, f"session_{timestamp}")
            os.makedirs(session_dir, exist_ok=True)
            
            # Save capture metadata
            scan_mode = self.scan_mode.get()
            metadata = {
                "specimen_name": self.specimen_name.get(),
                "timestamp": timestamp,
                "scan_mode": scan_mode,
                # Historical keys (legacy naming)
                "stacks": self.stacks_count.get(),
                "shots_per_stack": self.shots_per_stack.get(),
                # Clarified semantics
                "perspectives": self.stacks_count.get(),
                "focus_slices_per_perspective": self.shots_per_stack.get(),
                "angle_step_degrees": 360.0 / max(1, self.stacks_count.get()),
                "settle_delay": self.settle_delay.get(),
                "image_format": self.image_format.get(),
                "calibration_data": self.calibration_data,
                "xmp_settings": {
                    "lens_to_object_mm": float(self.lens_to_object_mm.get()),
                    "rail_to_horizon_deg": float(self.rail_to_horizon_deg.get())
                },
                "xmp_consolidation": {
                    "enabled": True,
                    "directory": "xmp_files"
                }
            }
            
            # Add spherical-specific metadata
            if scan_mode == "spherical":
                metadata["spherical_settings"] = {
                    "elevation_min": float(self.elevation_min.get()),
                    "elevation_max": float(self.elevation_max.get()),
                    "elevation_steps": int(self.elevation_steps.get())
                }
            
            with open(os.path.join(session_dir, "metadata.json"), 'w') as f:
                json.dump(metadata, f, indent=2)
            
            # Start camera
            if self.camera is None or not self.camera.started:
                self.after(0, self.start_preview)
                time.sleep(2)  # Let camera stabilize
            
            # Apply camera settings
            self.after(0, self.apply_camera_settings)
            time.sleep(1)
            
            # Pause preview updates during capture to reduce contention
            self.preview_update_active = False
            
            # Branch based on scan mode
            if scan_mode == "ring":
                self._run_ring_capture(session_dir, scan_mode)
            elif scan_mode == "spherical":
                self._run_spherical_capture(session_dir, scan_mode)
            else:
                raise ValueError(f"Unknown scan mode: {scan_mode}")
        
        except Exception as e:
            self.after(0, lambda: self.log_capture(f"Capture error: {e}"))
            self.after(0, lambda: messagebox.showerror("Error", f"Capture failed: {e}"))
        
        finally:
            self.capture_running = False
            # Resume preview updates after capture
            if self.preview_on:
                self.preview_update_active = True
            self.after(0, lambda: self.btn_start_capture.config(state="normal"))
            self.after(0, lambda: self.btn_stop_capture.config(state="disabled"))
    
    def _run_ring_capture(self, session_dir, scan_mode):
        """Execute ring scan capture sequence (horizontal plane only)"""
        total_images = self.stacks_count.get() * self.shots_per_stack.get()
        image_count = 0
        
        self.after(0, lambda: self.log_capture(f"Starting ring scan: {total_images} images"))

        # Strict waterfall order
        # Interpret 'stacks_count' as number of perspectives (angles) in full 360°
        # and 'shots_per_stack' as number of focus slices per angle.
        angle_step = 360.0 / max(1, self.stacks_count.get())
        
        # Determine zero-padding width for stack folders to match consolidated XMP naming
        try:
            perspectives_total = int(self.stacks_count.get())
        except Exception:
            perspectives_total = 0
        stack_pad_width = max(2, len(str(max(0, perspectives_total - 1))))

        # Move to starting positions
        if not self.move_to_angle(0.0):
            raise RuntimeError("Failed to home turntable to 0°")

        # Perspective-major loop: for each angle, capture all focus slices
        for perspective in range(self.stacks_count.get()):
            if not self.capture_running:
                break

            angle = perspective * angle_step
            # Turntable to angle
            if not self.move_to_angle(angle):
                self.after(0, self.log_capture, f"Aborting: Failed to move to angle {angle}")
                break

            # For each focus slice at this angle
            for slice_idx in range(self.shots_per_stack.get()):
                if not self.capture_running:
                    break

                stack_dir = os.path.join(session_dir, f"stack_{perspective:0{stack_pad_width}d}")
                os.makedirs(stack_dir, exist_ok=True)

                # Map slice index to 0.0..1.0 focus position
                stack_position = (slice_idx / (self.shots_per_stack.get() - 1)) if self.shots_per_stack.get() > 1 else 0.0
                focus_position = self.interpolate_focus_position(angle, stack_position)

                # Move focus
                if not self.move_to_focus_position(focus_position):
                    self.after(0, self.log_capture, f"Aborting: Failed to move focus to {focus_position:.2f}mm")
                    break

                # Settle
                time.sleep(self.settle_delay.get())

                # Capture
                filename = f"stack_{perspective:0{stack_pad_width}d}_shot_{slice_idx:04d}_angle_{angle:06.2f}.{self.image_format.get().lower()}"
                filepath = os.path.join(stack_dir, filename)
                self.capture_image(filepath)

                image_count += 1
                progress = (image_count / total_images) * 100
                self.after(0, self.capture_progress.set, progress)
                self.after(0, self.progress_label_var.set, f"Captured {image_count}/{total_images} images")
                if image_count % 10 == 0:
                    self.after(0, self.log_capture, f"Captured {image_count} images")

            # Generate XMP sidecar for this stack (perspective)
            self._generate_ring_xmp(session_dir, perspective, angle, stack_pad_width)
        
        # Consolidate XMP files
        self._consolidate_xmp_files(session_dir, image_count, angle_step, stack_pad_width)
    
    def _generate_ring_xmp(self, session_dir, perspective, angle, stack_pad_width):
        """Generate XMP sidecar for ring scan perspective"""
        try:
            # Calculate camera pose for this angle
            lens_dist = float(self.lens_to_object_mm.get())
            rail_angle = float(self.rail_to_horizon_deg.get())
            position_scale = float(self.xmp_position_scale.get())
            
            # Apply position scaling to distance BEFORE calculating pose
            scaled_lens_dist = lens_dist * position_scale / 1000.0
            
            # Turntable rotates CCW, so camera equivalent is CW orbit (negative angle)
            camera_angle = -angle
            pose = ring_pose(scaled_lens_dist, rail_angle, camera_angle)
            
            # Create XMP file for the stack
            stack_dir = os.path.join(session_dir, f"stack_{perspective:0{stack_pad_width}d}")
            stack_xmp_filename = f"stack_{perspective:0{stack_pad_width}d}_angle_{angle:06.2f}.xmp"
            stack_xmp_path = os.path.join(stack_dir, stack_xmp_filename)
            
            # Gather distortion coefficients
            distortion_coeffs = (
                self.distortion_k1.get(),
                self.distortion_k2.get(),
                self.distortion_k3.get(),
                self.distortion_p1.get(),
                self.distortion_p2.get(),
                self.distortion_k4.get()
            )
            
            # Write XMP with pose data and camera calibration
            write_xmp_sidecar(
                stack_xmp_path.replace('.xmp', '.jpg'),
                pose,
                lens_dist,
                rail_angle,
                angle,
                perspective,
                focal_length_35mm=VERIFIED_FOCAL_LENGTH_35MM,
                distortion_model=VERIFIED_DISTORTION_MODEL,
                skew=VERIFIED_SKEW,
                aspect_ratio=VERIFIED_ASPECT_RATIO,
                principal_point_u=VERIFIED_PRINCIPAL_POINT_U,
                principal_point_v=VERIFIED_PRINCIPAL_POINT_V,
                distortion_coefficients=VERIFIED_DISTORTION_COEFFS,
                position_scale=1.0,
                pose_prior=VERIFIED_POSE_PRIOR,
                calibration_prior=VERIFIED_CALIB_PRIOR,
            )
            
            self.after(0, self.log_capture, f"Generated XMP for stack {perspective} at {angle:.2f}°")
            
        except Exception as e:
            self.after(0, self.log_capture, f"Warning: Failed to generate XMP for stack {perspective}: {e}")
    
    def _generate_spherical_xmp(self, session_dir, perspective_idx, azimuth, elevation, stack_pad_width):
        """Generate XMP sidecar for spherical scan perspective"""
        try:
            lens_dist = float(self.lens_to_object_mm.get())
            position_scale = float(self.xmp_position_scale.get())
            
            # Use spherical pose function
            scaled_radius = lens_dist * position_scale / 1000.0
            pose = spherical_pose(scaled_radius, azimuth, elevation)
            
            # Create XMP file for the stack
            stack_dir = os.path.join(session_dir, f"stack_{perspective_idx:0{stack_pad_width}d}")
            stack_xmp_filename = f"stack_{perspective_idx:0{stack_pad_width}d}_elev_{elevation:+06.2f}_azim_{azimuth:06.2f}.xmp"
            stack_xmp_path = os.path.join(stack_dir, stack_xmp_filename)
            
            # Gather distortion coefficients
            distortion_coeffs = (
                self.distortion_k1.get(),
                self.distortion_k2.get(),
                self.distortion_k3.get(),
                self.distortion_p1.get(),
                self.distortion_p2.get(),
                self.distortion_k4.get()
            )
            
            # Write XMP with pose data and camera calibration
            write_xmp_sidecar(
                stack_xmp_path.replace('.xmp', '.jpg'),
                pose,
                lens_dist,
                elevation,  # Use elevation as rail_to_horizon equivalent
                azimuth,
                perspective_idx,
                focal_length_35mm=VERIFIED_FOCAL_LENGTH_35MM,
                distortion_model=VERIFIED_DISTORTION_MODEL,
                skew=VERIFIED_SKEW,
                aspect_ratio=VERIFIED_ASPECT_RATIO,
                principal_point_u=VERIFIED_PRINCIPAL_POINT_U,
                principal_point_v=VERIFIED_PRINCIPAL_POINT_V,
                distortion_coefficients=VERIFIED_DISTORTION_COEFFS,
                position_scale=1.0,
                pose_prior=VERIFIED_POSE_PRIOR,
                calibration_prior=VERIFIED_CALIB_PRIOR,
            )
            
            self.after(0, self.log_capture, f"Generated XMP for stack {perspective_idx} (elev={elevation:.1f}°, azim={azimuth:.1f}°)")
            
        except Exception as e:
            self.after(0, self.log_capture, f"Warning: Failed to generate XMP for stack {perspective_idx}: {e}")
    
    def _consolidate_xmp_files(self, session_dir, image_count, angle_step, stack_pad_width):
        """Consolidate XMP files into central directory"""
        if not self.capture_running:
            return
        
        try:
            # Create consolidated XMP directory
            xmp_dir = os.path.join(session_dir, "xmp_files")
            os.makedirs(xmp_dir, exist_ok=True)
            
            # Determine zero-padding width for consolidated filenames
            perspectives_total = self.stacks_count.get()
            pad_width = max(2, len(str(max(0, perspectives_total - 1))))

            # Copy all XMP files to consolidated directory
            xmp_count = 0
            for perspective in range(perspectives_total):
                angle = perspective * angle_step
                stack_dir = os.path.join(session_dir, f"stack_{perspective:0{stack_pad_width}d}")
                
                # Find XMP file in stack directory (matches filename pattern)
                for file in os.listdir(stack_dir):
                    if file.endswith('.xmp'):
                        source_xmp = os.path.join(stack_dir, file)
                        # Create consolidated filename matching stack directory naming
                        consolidated_filename = f"stack_{perspective:0{pad_width}d}.xmp"
                        dest_xmp = os.path.join(xmp_dir, consolidated_filename)
                        
                        import shutil
                        shutil.copy2(source_xmp, dest_xmp)
                        xmp_count += 1
                        break
            
            # Copy helper script and README to session directory
            import shutil
            script_dir = os.path.dirname(os.path.abspath(__file__))
            
            # Copy helper script if available
            helper_script_source = os.path.join(script_dir, "templates", "rename_xmp_for_rc.py")
            if os.path.exists(helper_script_source):
                helper_script_dest = os.path.join(session_dir, "rename_xmp_for_rc.py")
                shutil.copy2(helper_script_source, helper_script_dest)
                self.after(0, lambda: self.log_capture("Helper script copied to session directory"))
            
            # Copy README guide
            readme_source = os.path.join(script_dir, "templates", "SESSION_README.md")
            if os.path.exists(readme_source):
                readme_dest = os.path.join(session_dir, "README.md")
                shutil.copy2(readme_source, readme_dest)
                self.after(0, lambda: self.log_capture("Workflow README copied to session directory"))
            
            self.after(0, lambda: self.log_capture(f"Consolidated {xmp_count} XMP files to: {xmp_dir}"))
            
        except Exception as e:
            self.after(0, lambda: self.log_capture(f"Warning: Failed to consolidate XMP files: {e}"))
        
        self.after(0, lambda: self.log_capture(f"Capture sequence complete! {image_count} images saved to {session_dir}"))
        self.after(0, lambda: messagebox.showinfo("Complete", f"Capture sequence complete!\n{image_count} images saved.\nXMP files consolidated in xmp_files directory."))
    
    def _run_spherical_capture(self, session_dir, scan_mode):
        """Execute spherical/spiral scan capture sequence"""
        # Calculate scan parameters
        elevation_min = float(self.elevation_min.get())
        elevation_max = float(self.elevation_max.get())
        elevation_steps_count = int(self.elevation_steps.get())
        azimuth_steps_count = int(self.stacks_count.get())
        focus_slices = int(self.shots_per_stack.get())
        
        # Generate elevation angles
        if elevation_steps_count == 1:
            elevations = [(elevation_min + elevation_max) / 2.0]
        else:
            elevation_step = (elevation_max - elevation_min) / (elevation_steps_count - 1)
            elevations = [elevation_min + i * elevation_step for i in range(elevation_steps_count)]
        
        azimuth_step = 360.0 / max(1, azimuth_steps_count)
        
        total_images = elevation_steps_count * azimuth_steps_count * focus_slices
        image_count = 0
        
        self.after(0, lambda: self.log_capture(f"Starting spherical scan: {total_images} images"))
        self.after(0, lambda: self.log_capture(f"Elevations: {elevations}"))
        
        # Determine padding width
        perspective_total = elevation_steps_count * azimuth_steps_count
        stack_pad_width = max(2, len(str(max(0, perspective_total - 1))))
        
        # Move to starting positions
        if not self.move_to_angle(0.0):
            raise RuntimeError("Failed to home turntable to 0°")
        if not self.move_to_elevation(elevations[0]):
            raise RuntimeError(f"Failed to move to starting elevation {elevations[0]}°")
        
        # Elevation-major loop (outer): for each elevation
        perspective_idx = 0
        for elev_idx, elevation in enumerate(elevations):
            if not self.capture_running:
                break
            
            # Move to elevation
            if not self.move_to_elevation(elevation):
                self.after(0, lambda e=elevation: self.log_capture(f"Aborting: Failed to tilt to {e}°"))
                break
            
            self.after(0, lambda e=elevation, i=elev_idx: self.log_capture(f"Elevation {i+1}/{len(elevations)}: {e:.1f}°"))
            
            # Azimuth loop (middle): for each angle around turntable
            for azimuth_idx in range(azimuth_steps_count):
                if not self.capture_running:
                    break
                
                azimuth = azimuth_idx * azimuth_step
                
                # Move turntable
                if not self.move_to_angle(azimuth):
                    self.after(0, lambda a=azimuth: self.log_capture(f"Aborting: Failed to move to azimuth {a}°"))
                    break
                
                # Focus loop (inner): for each focus slice
                for slice_idx in range(focus_slices):
                    if not self.capture_running:
                        break
                    
                    stack_dir = os.path.join(session_dir, f"stack_{perspective_idx:0{stack_pad_width}d}")
                    os.makedirs(stack_dir, exist_ok=True)
                    
                    # Map slice index to 0.0..1.0 focus position
                    stack_position = (slice_idx / (focus_slices - 1)) if focus_slices > 1 else 0.0
                    focus_position = self.interpolate_focus_position(azimuth, stack_position)
                    
                    # Move focus
                    if not self.move_to_focus_position(focus_position):
                        self.after(0, lambda fp=focus_position: self.log_capture(f"Aborting: Failed to move focus to {fp:.2f}mm"))
                        break
                    
                    # Settle
                    time.sleep(self.settle_delay.get())
                    
                    # Capture
                    filename = f"stack_{perspective_idx:0{stack_pad_width}d}_shot_{slice_idx:04d}_elev_{elevation:+06.2f}_azim_{azimuth:06.2f}.{self.image_format.get().lower()}"
                    filepath = os.path.join(stack_dir, filename)
                    self.capture_image(filepath)
                    
                    image_count += 1
                    progress = (image_count / total_images) * 100
                    self.after(0, self.capture_progress.set, progress)
                    self.after(0, self.progress_label_var.set, f"Captured {image_count}/{total_images} images")
                    if image_count % 10 == 0:
                        self.after(0, lambda ic=image_count: self.log_capture(f"Captured {ic} images"))
                
                # Generate XMP for this perspective
                self._generate_spherical_xmp(session_dir, perspective_idx, azimuth, elevation, stack_pad_width)
                perspective_idx += 1
        
        # Consolidate XMP files - use azimuth_step for consistency
        self._consolidate_xmp_files(session_dir, image_count, azimuth_step, stack_pad_width)
    
    def move_to_focus_position(self, target_position):
        """Move motor 2 to specific position"""
        current_position = self.motor2_position_mm
        move_distance = target_position - current_position
        
        if abs(move_distance) > 0.01:  # Only move if significant difference
            direction = "FORWARD" if move_distance > 0 else "BACKWARD"
            amount = abs(move_distance)
            
            command = f"MOVE 2 {amount} {direction}"
            if self.send_motor_command(command, wait_for_done=True):
                self.motor2_position_mm = target_position
                self.after(0, self.motor2_pos_var.set, f"{target_position:.2f}mm")
                return True
            else:
                self.after(0, self.status_var.set, "Failed to move focus position")
                return False
        return True
    
    def capture_image(self, filepath):
        """Capture an image to the specified filepath"""
        if self.camera is None or not self.camera.started:
            raise Exception("Camera not started")
        
        # Prepare still configuration with target capture resolution
        try:
            w, h = map(int, self.capture_resolution.get().lower().split("x"))
        except Exception:
            w, h = (2028, 1520)

        still_config = self.camera.create_still_configuration(main={"size": (w, h)})

        # Capture high-resolution image
        fmt = self.image_format.get().upper()
        with self.camera_lock:
            if fmt == "JPG":
                try:
                    self.camera.switch_mode_and_capture_file(still_config, filepath)
                finally:
                    # Return to preview mode if active
                    if self.preview_on:
                        try:
                            self.camera.switch_mode(self.preview_config)
                        except Exception:
                            pass
            else:
                try:
                    array = self.camera.switch_mode_and_capture_array(still_config, "main")
                finally:
                    if self.preview_on:
                        try:
                            self.camera.switch_mode(self.preview_config)
                        except Exception:
                            pass
                image = Image.fromarray(array)
                image.save(filepath)
    
    def log_capture(self, message):
        """Add message to capture log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.capture_log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.capture_log_text.see(tk.END)
    
    # =====================================================================
    # CAMERA SETTINGS METHODS
    # =====================================================================
    
    def apply_camera_settings(self):
        """Apply camera settings"""
        if self.camera is None:
            return
        
        try:
            controls = {}
            
            # Shutter speed
            if self.shutter_speed.get() > 0:
                controls["ExposureTime"] = self.shutter_speed.get()
            
            # AE and ISO / Analogue Gain
            if self.ae_enable.get():
                controls["AeEnable"] = True
                # EV compensation only applies when AE is enabled
                if abs(self.ev_compensation.get()) > 0.01:
                    controls["ExposureValue"] = float(self.ev_compensation.get())
            else:
                controls["AeEnable"] = False
                # Manual ISO -> approximate analogue gain (ISO ~ 100 * gain)
                iso = self.iso_value.get()
                if iso and iso != "Auto":
                    try:
                        controls["AnalogueGain"] = float(int(iso) / 100.0)
                    except Exception:
                        pass
                elif self.analogue_gain.get() >= 1.0:
                    controls["AnalogueGain"] = float(self.analogue_gain.get())
            
            # Brightness
            if abs(self.brightness.get()) > 0.01:
                controls["Brightness"] = self.brightness.get()

            # AWB or manual WB
            if self.awb_enable.get():
                controls["AwbEnable"] = True
            else:
                controls["AwbEnable"] = False
                controls["ColourGains"] = (float(self.wb_red_gain.get()), float(self.wb_blue_gain.get()))

            # Image processing
            if abs(self.contrast.get() - 1.0) > 0.01:
                controls["Contrast"] = float(self.contrast.get())
            if abs(self.saturation.get() - 1.0) > 0.01:
                controls["Saturation"] = float(self.saturation.get())
            if abs(self.sharpness.get() - 1.0) > 0.01:
                controls["Sharpness"] = float(self.sharpness.get())
            
            if controls:
                with self.camera_lock:
                    self.camera.set_controls(controls)
                self.status_var.set("Camera settings applied")

            # If preview is running and resolution changed, restart preview with new config
            if self.preview_on:
                try:
                    self.stop_preview()
                    time.sleep(0.1)
                    self.start_preview()
                except Exception:
                    pass
            
            # Update camera info display
            self.update_camera_info()
            
        except Exception as e:
            self.status_var.set(f"Error applying camera settings: {e}")
    
    def update_camera_info(self):
        """Update camera information display"""
        if self.camera is None:
            return
        
        try:
            self.camera_info_text.config(state="normal")
            self.camera_info_text.delete(1.0, tk.END)
            
            # Get camera properties
            info = []
            info.append("Camera Information:")
            info.append("-" * 30)
            
            if hasattr(self.camera, 'camera_properties'):
                props = self.camera.camera_properties
                for key, value in props.items():
                    info.append(f"{key}: {value}")
            
            info.append("\nCurrent Controls:")
            info.append("-" * 20)
            
            if hasattr(self.camera, 'capture_metadata'):
                try:
                    metadata = self.camera.capture_metadata()
                    for key, value in metadata.items():
                        info.append(f"{key}: {value}")
                except:
                    info.append("Metadata not available")
            
            self.camera_info_text.insert(1.0, "\n".join(info))
            self.camera_info_text.config(state="disabled")
            
        except Exception as e:
            self.camera_info_text.config(state="normal")
            self.camera_info_text.delete(1.0, tk.END)
            self.camera_info_text.insert(1.0, f"Error getting camera info: {e}")
            self.camera_info_text.config(state="disabled")

# =====================================================================
# CALIBRATION TAB MOTOR CONTROLS
# =====================================================================
    
    def cal_motor1_ccw(self):
        """Motor 1 CCW control for calibration tab"""
        try:
            step = float(self.cal_motor1_step_var.get())
            # Apply direction inversion if enabled
            direction = "CCW" if self.rotation_invert.get() else "CW"
            command = f"ROTATE 1 {step} {direction}"
            
            if self.send_motor_command(command):
                self.motor1_position_deg -= step
                self.motor1_pos_var.set(f"{self.motor1_position_deg:.1f}°")
                self.status_var.set(f"Motor 1: Rotated {step}° CCW")
            else:
                self.status_var.set("Failed to rotate motor 1")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 1")

    def cal_motor1_cw(self):
        """Motor 1 CW control for calibration tab"""
        try:
            step = float(self.cal_motor1_step_var.get())
            # Apply direction inversion if enabled
            direction = "CW" if self.rotation_invert.get() else "CCW"
            command = f"ROTATE 1 {step} {direction}"
            
            if self.send_motor_command(command):
                self.motor1_position_deg += step
                self.motor1_pos_var.set(f"{self.motor1_position_deg:.1f}°")
                self.status_var.set(f"Motor 1: Rotated {step}° CW")
            else:
                self.status_var.set("Failed to rotate motor 1")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 1")

    def cal_motor2_up(self):
        """Motor 2 forward control for calibration tab"""
        try:
            step = float(self.cal_motor2_step_var.get())
            command = f"MOVE 2 {step} FORWARD"
            
            if self.send_motor_command(command):
                self.motor2_position_mm += step
                self.motor2_pos_var.set(f"{self.motor2_position_mm:.1f}mm")
                self.status_var.set(f"Motor 2: Moved {step}mm forward")
            else:
                self.status_var.set("Failed to move motor 2")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 2")

    def cal_motor2_down(self):
        """Motor 2 backward control for calibration tab"""
        try:
            step = float(self.cal_motor2_step_var.get())
            command = f"MOVE 2 {step} BACKWARD"
            
            if self.send_motor_command(command):
                self.motor2_position_mm -= step
                self.motor2_pos_var.set(f"{self.motor2_position_mm:.1f}mm")
                self.status_var.set(f"Motor 2: Moved {step}mm backward")
            else:
                self.status_var.set("Failed to move motor 2")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 2")
    
    def cal_motor3_up(self):
        """Motor 3 tilt up control for calibration tab"""
        try:
            step = float(self.cal_motor3_step_var.get())
            command = f"TILT 3 {step} UP"
            
            if self.send_motor_command(command):
                self.motor3_position_deg += step
                self.motor3_pos_var.set(f"{self.motor3_position_deg:.1f}°")
                self.status_var.set(f"Motor 3: Tilted {step}° up")
            else:
                self.status_var.set("Failed to tilt motor 3")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 3")

    def cal_motor3_down(self):
        """Motor 3 tilt down control for calibration tab"""
        try:
            step = float(self.cal_motor3_step_var.get())
            command = f"TILT 3 {step} DOWN"
            
            if self.send_motor_command(command):
                self.motor3_position_deg -= step
                self.motor3_pos_var.set(f"{self.motor3_position_deg:.1f}°")
                self.status_var.set(f"Motor 3: Tilted {step}° down")
            else:
                self.status_var.set("Failed to tilt motor 3")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 3")
    
    def _load_manual_content(self):
        """Load user manual content into help panel"""
        manual_text = """3D SCANNER USER MANUAL

=== QUICK START ===

1. SETUP
   - Connect Arduino via USB
   - Connect Pi Camera
   - Verify motors move correctly in Manual Control tab

2. CALIBRATION (Focus Envelope)
   - Go to Calibration tab
   - Click "Start Calibration"
   - At each angle (0°, 90°, 180°, 270°):
     * Move focus motor to NEAR limit (closest to camera)
     * Click "Capture Position"
     * Move focus motor to FAR limit (farthest from camera)
     * Click "Capture Position"
     * Click "Next Step"
   - Click "Save Calibration" when complete

3. CAMERA SETTINGS
   - Set exposure, gain, white balance in Camera Settings tab
   - Configure XMP calibration parameters in Calibration tab
   - Choose position scale preset based on specimen size

4. CAPTURE
   - Enter specimen name
   - Set number of perspectives (angles around turntable)
   - Set focus slices per perspective
   - Click "Start Capture"
   - Images saved to output_dir/specimen_name/session_YYYYMMDD/

=== MOTOR CONTROLS ===

Motor 1 (Turntable):
  - CCW/CW buttons rotate specimen
  - Use "Reverse Direction" toggle if motors are backwards
  - Zero button resets position counter

Motor 2 (Focus/Linear):
  - Forward/Backward moves camera closer/farther
  - Home button returns to limit switch (back position)
  - Used for focus stacking

Motor 3 (Tilt):
  - Up/Down tilts camera angle
  - Zero button resets position counter
  - Currently direct-drive (consider geared motor for long holds)

=== XMP CALIBRATION ===

LENS TO OBJECT:
  Measure distance from camera lens front to specimen center
  in millimeters using ruler or calipers

FOCAL LENGTH (35mm equiv):
  - Pi HQ Camera (6mm lens): ~50mm
  - Pi HQ Camera (16mm lens): ~130mm
  - Pi Camera V2: ~29mm
  Formula: 35mm_equiv = actual_focal × (36 / sensor_width_mm)

POSITION SCALE PRESET:
  - Tiny (1-5mm): Insects, coins, small jewelry → scale 100,000
  - Small (5-20mm): Large insects, small fossils → scale 20,000
  - Medium (20-50mm): Rocks, larger fossils → scale 5,000
  
  Note: This keeps XMP coordinate precision manageable.
  Scale model back to real size in RealityCapture after reconstruction.

PRINCIPAL POINT / ASPECT RATIO / SKEW:
  Leave at defaults (0, 0, 1.0, 0) unless you have calibration data

DISTORTION COEFFICIENTS:
  Leave all at 0.0 unless you have OpenCV calibration data
  Wrong values are worse than no values!
  To calibrate: use checkerboard pattern + OpenCV calibrateCamera()

=== FOCUS STACKING ===

The scanner captures multiple images at different focus distances
at each turntable angle. This creates a "focus stack" that can be
combined in RealityCapture or other software.

Focus calibration stores NEAR and FAR limits at cardinal angles
(0°, 90°, 180°, 270°). The software interpolates between these
for other angles to account for specimen shape variations.

Tips:
  - More focus slices = better depth coverage but longer scan time
  - 3-5 slices typical for small specimens
  - Ensure entire specimen is in focus across all angles

=== FILE OUTPUT ===

Directory structure:
  output_dir/
    specimen_name/
      session_YYYYMMDD_HHMMSS/
        metadata.json
        stack_00/
          stack_00_shot_000_angle_000.00.jpg
          stack_00_shot_000_angle_000.00.xmp
          stack_00_shot_001_angle_000.00.jpg
          ...
        stack_01/
          ...

XMP files contain camera pose data for RealityScan/RealityCapture.
Import the entire session folder to automatically load poses.

=== TROUBLESHOOTING ===

Motors not moving:
  - Check serial connection (status bar shows "Connected")
  - Verify Arduino loaded with scanner_controller.ino
  - Check enable pin polarity (TB6600: ENABLE_ACTIVE_HIGH = true)
  - Increase motor current if stuttering

Camera errors:
  - Ensure camera enabled in raspi-config
  - Check cable connection
  - Stop preview before starting capture

Position drift:
  - Use "Home" button on linear rail to re-zero
  - Zero all motors before capture sequence
  - Consider limit switches for automatic homing

Tilt motor overheating:
  - Check counterbalance is properly adjusted
  - Consider upgrading to geared stepper (5:1 planetary)
  - Reduce settle time if using direct-drive

XMP import issues:
  - Verify all XMP files have matching image files
  - Check position scale isn't causing coordinate overflow
  - Ensure focal length is reasonable for camera type

=== KEYBOARD SHORTCUTS ===

(None currently implemented - use mouse/touch)

=== SUPPORT ===

For bugs, feature requests, or questions:
https://github.com/reprahkcin/scanner-companion"""
        
        self.help_text.insert("1.0", manual_text)
    
    def _load_notes(self):
        """Load notes from local file"""
        notes_file = os.path.join(os.path.dirname(__file__), "session_notes.txt")
        try:
            if os.path.exists(notes_file):
                with open(notes_file, 'r') as f:
                    content = f.read()
                    self.notes_text.insert("1.0", content)
        except Exception as e:
            print(f"Could not load notes: {e}")
    
    def _auto_save_notes(self, event=None):
        """Auto-save notes to local file"""
        if self.notes_text.edit_modified():
            notes_file = os.path.join(os.path.dirname(__file__), "session_notes.txt")
            try:
                content = self.notes_text.get("1.0", "end-1c")
                with open(notes_file, 'w') as f:
                    f.write(content)
                self.notes_text.edit_modified(False)
            except Exception as e:
                print(f"Could not save notes: {e}")
    
if __name__ == "__main__":
    print("Starting 3D Scanner Control Panel v1.0...")
    
    # Test basic imports first
    try:
        print("Testing imports...")
        import tkinter as tk
        from tkinter import ttk
        print("- tkinter: OK")
        
        import serial
        print("- serial: OK")
        
        import cv2
        print("- opencv: OK")
        
        try:
            from picamera2 import Picamera2
            print("- picamera2: OK")
        except ImportError:
            print("- picamera2: Not available (expected on non-Pi systems)")
            
        try:
            from PIL import Image
            print("- PIL: OK")
        except ImportError:
            print("- PIL: Missing!")
            
    except Exception as e:
        print(f"Import error: {e}")
        exit(1)
    
    try:
        print("Creating GUI application...")
        app = ScannerGUI()
        print("GUI created successfully. Starting main loop...")
        app.mainloop()
        print("Application closed.")
    except Exception as e:
        print(f"Error starting application: {e}")
        import traceback
        traceback.print_exc()
