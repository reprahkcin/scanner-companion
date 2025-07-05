#!/usr/bin/env python3
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

# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURATION SECTION
# ──────────────────────────────────────────────────────────────────────────────

SERIAL_PORT       = "/dev/ttyACM0"  # or "/dev/serial/by-id/..." 
SERIAL_BAUDRATE   = 115200            # Match motor_control_refactor.ino baudrate
CAMERA_RESOLUTION = (320, 240)       # preview size (smaller for safety)
JOG_STEP_DELAY    = 0.05             # seconds between steps during jog

# Capture defaults
DEFAULT_OUTPUT_DIR = os.path.expanduser("~/Desktop/scanner_captures")
DEFAULT_STACKS = 5
DEFAULT_SHOTS_PER_STACK = 72  # 5-degree increments for 360°
DEFAULT_SETTLE_DELAY = 1.0    # seconds to wait after movement before capture

# Cardinal angles for calibration
CALIBRATION_ANGLES = [0, 90, 180, 270]

# ──────────────────────────────────────────────────────────────────────────────

class ScannerGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("3D Scanner Control Panel - v4.0")
        self.minsize(800, 600)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

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

        # === Motor position trackers ===
        self.motor1_position_deg = 0.0
        self.motor2_position_mm = 0.0

        # === Calibration data ===
        self.calibration_data = {}  # {angle: {"near": mm_pos, "far": mm_pos}}
        self.is_calibrated = False
        self.current_calibration_angle = 0
        self.current_calibration_focus = "near"

        # === Capture settings ===
        self.specimen_name = tk.StringVar(value="specimen_001")
        self.output_dir = tk.StringVar(value=DEFAULT_OUTPUT_DIR)
        self.stacks_count = tk.IntVar(value=DEFAULT_STACKS)
        self.shots_per_stack = tk.IntVar(value=DEFAULT_SHOTS_PER_STACK)
        self.settle_delay = tk.DoubleVar(value=DEFAULT_SETTLE_DELAY)
        self.image_format = tk.StringVar(value="JPG")
        
        # === Camera settings ===
        self.shutter_speed = tk.IntVar(value=0)  # 0 = auto
        self.brightness = tk.DoubleVar(value=0.0)
        
        # === Progress tracking ===
        self.capture_progress = tk.DoubleVar(value=0.0)
        self.capture_running = False
        self.capture_thread = None

        # Build the GUI
        self._build_gui()
        self._layout_gui()

        # Create output directory if it doesn't exist
        os.makedirs(DEFAULT_OUTPUT_DIR, exist_ok=True)

    def _build_gui(self):
        # Create main notebook for tabs
        self.notebook = ttk.Notebook(self)
        
        # === Manual Control Tab ===
        self.manual_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.manual_tab, text="Manual Control")
        self._build_manual_tab()
        
        # === Calibration Tab ===
        self.calibration_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.calibration_tab, text="Calibration")
        self._build_calibration_tab()
        
        # === Capture Tab ===
        self.capture_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.capture_tab, text="Capture")
        self._build_capture_tab()
        
        # === Camera Settings Tab ===
        self.camera_settings_tab = ttk.Frame(self.notebook)
        self.notebook.add(self.camera_settings_tab, text="Camera Settings")
        self._build_camera_settings_tab()

        # Status Bar
        self.status_bar = ttk.Label(self, textvariable=self.status_var, relief="sunken", anchor="w")

    def _build_manual_tab(self):
        # --- Camera Frame ---
        self.camera_frame = ttk.LabelFrame(self.manual_tab, text="Camera", padding=10)
        self.preview_label = tk.Label(self.camera_frame, bg="black", anchor="center")
        self.preview_label.grid(row=0, column=0, columnspan=2, padx=5, pady=5)
        self.btn_preview = ttk.Button(self.camera_frame, text="Start Preview", command=self.toggle_preview)
        self.btn_preview.grid(row=1, column=0, columnspan=2, pady=(5,0))

        # --- Motor 1 Frame ---
        self.motor1_frame = ttk.LabelFrame(self.manual_tab, text="Motor 1 (Rotation)", padding=10)
        
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
        self.motor2_frame = ttk.LabelFrame(self.manual_tab, text="Motor 2 (Linear)", padding=10)
        
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

        # Layout manual tab
        self.manual_tab.columnconfigure(0, weight=1)
        self.manual_tab.columnconfigure(1, weight=1)
        self.manual_tab.rowconfigure(0, weight=1)
        self.manual_tab.rowconfigure(1, weight=1)
        
        self.camera_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=5, sticky="nsew")
        self.motor1_frame.grid(row=1, column=0, padx=(10,5), pady=5, sticky="nsew")
        self.motor2_frame.grid(row=1, column=1, padx=(5,10), pady=5, sticky="nsew")

    def _build_calibration_tab(self):
        # Calibration status
        self.calibration_status_frame = ttk.LabelFrame(self.calibration_tab, text="Calibration Status", padding=10)
        
        self.calibration_status_var = tk.StringVar(value="Not Calibrated")
        ttk.Label(self.calibration_status_frame, text="Status:").grid(row=0, column=0, sticky="w")
        ttk.Label(self.calibration_status_frame, textvariable=self.calibration_status_var, 
                 font=("TkDefaultFont", 10, "bold"), foreground="red").grid(row=0, column=1, sticky="w")
        
        # Calibration instructions
        instructions = """Calibration Workflow:
1. Position specimen at 0° (front)
2. Set near and far focus positions
3. Repeat for 90°, 180°, and 270°
4. System will interpolate focus for all angles"""
        
        self.instructions_label = tk.Label(self.calibration_tab, text=instructions, 
                                          justify="left", bg="#f0f0f0", padx=10, pady=10)
        
        # Current calibration step
        self.calibration_step_frame = ttk.LabelFrame(self.calibration_tab, text="Current Step", padding=10)
        
        self.current_angle_var = tk.StringVar(value="0°")
        self.current_focus_var = tk.StringVar(value="Near")
        
        ttk.Label(self.calibration_step_frame, text="Angle:").grid(row=0, column=0, sticky="w")
        ttk.Label(self.calibration_step_frame, textvariable=self.current_angle_var, 
                 font=("TkDefaultFont", 12, "bold")).grid(row=0, column=1, sticky="w", padx=(5,0))
        
        ttk.Label(self.calibration_step_frame, text="Focus:").grid(row=1, column=0, sticky="w")
        ttk.Label(self.calibration_step_frame, textvariable=self.current_focus_var, 
                 font=("TkDefaultFont", 12, "bold")).grid(row=1, column=1, sticky="w", padx=(5,0))
        
        # Calibration controls
        self.calibration_controls_frame = ttk.LabelFrame(self.calibration_tab, text="Controls", padding=10)
        
        self.btn_start_calibration = ttk.Button(self.calibration_controls_frame, text="Start Calibration", 
                                               command=self.start_calibration)
        self.btn_start_calibration.grid(row=0, column=0, pady=5)
        
        self.btn_capture_position = ttk.Button(self.calibration_controls_frame, text="Capture Position", 
                                              command=self.capture_calibration_position, state="disabled")
        self.btn_capture_position.grid(row=0, column=1, padx=(10,0), pady=5)
        
        self.btn_next_step = ttk.Button(self.calibration_controls_frame, text="Next Step", 
                                       command=self.next_calibration_step, state="disabled")
        self.btn_next_step.grid(row=0, column=2, padx=(10,0), pady=5)
        
        self.btn_save_calibration = ttk.Button(self.calibration_controls_frame, text="Save Calibration", 
                                              command=self.save_calibration, state="disabled")
        self.btn_save_calibration.grid(row=1, column=0, pady=5)
        
        self.btn_load_calibration = ttk.Button(self.calibration_controls_frame, text="Load Calibration", 
                                              command=self.load_calibration)
        self.btn_load_calibration.grid(row=1, column=1, padx=(10,0), pady=5)
        
        # Calibration progress
        self.calibration_progress_frame = ttk.LabelFrame(self.calibration_tab, text="Progress", padding=10)
        
        self.calibration_progress_text = tk.Text(self.calibration_progress_frame, height=6, width=50)
        self.calibration_progress_text.grid(row=0, column=0, sticky="nsew")
        
        scrollbar = ttk.Scrollbar(self.calibration_progress_frame, orient="vertical", 
                                 command=self.calibration_progress_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.calibration_progress_text.config(yscrollcommand=scrollbar.set)
        
        # Layout calibration tab
        self.calibration_tab.rowconfigure(4, weight=1)
        self.calibration_tab.columnconfigure(0, weight=1)
        
        self.calibration_status_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        self.instructions_label.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        self.calibration_step_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")
        self.calibration_controls_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")
        self.calibration_progress_frame.grid(row=4, column=0, padx=10, pady=5, sticky="nsew")

    def _build_capture_tab(self):
        # Specimen settings
        self.specimen_frame = ttk.LabelFrame(self.capture_tab, text="Specimen", padding=10)
        
        ttk.Label(self.specimen_frame, text="Name:").grid(row=0, column=0, sticky="w")
        self.specimen_entry = ttk.Entry(self.specimen_frame, textvariable=self.specimen_name, width=20)
        self.specimen_entry.grid(row=0, column=1, sticky="w", padx=(5,0))
        
        ttk.Label(self.specimen_frame, text="Output Dir:").grid(row=1, column=0, sticky="w")
        self.output_dir_entry = ttk.Entry(self.specimen_frame, textvariable=self.output_dir, width=40)
        self.output_dir_entry.grid(row=1, column=1, sticky="w", padx=(5,0))
        
        self.btn_browse_dir = ttk.Button(self.specimen_frame, text="Browse", command=self.browse_output_dir)
        self.btn_browse_dir.grid(row=1, column=2, padx=(5,0))
        
        # Capture settings
        self.capture_settings_frame = ttk.LabelFrame(self.capture_tab, text="Capture Settings", padding=10)
        
        ttk.Label(self.capture_settings_frame, text="Stacks:").grid(row=0, column=0, sticky="w")
        self.stacks_spinbox = ttk.Spinbox(self.capture_settings_frame, from_=1, to=20, 
                                         textvariable=self.stacks_count, width=10)
        self.stacks_spinbox.grid(row=0, column=1, sticky="w", padx=(5,0))
        
        ttk.Label(self.capture_settings_frame, text="Shots per Stack:").grid(row=1, column=0, sticky="w")
        self.shots_spinbox = ttk.Spinbox(self.capture_settings_frame, from_=12, to=360, 
                                        textvariable=self.shots_per_stack, width=10)
        self.shots_spinbox.grid(row=1, column=1, sticky="w", padx=(5,0))
        
        ttk.Label(self.capture_settings_frame, text="Settle Delay (s):").grid(row=2, column=0, sticky="w")
        self.settle_spinbox = ttk.Spinbox(self.capture_settings_frame, from_=0.1, to=5.0, increment=0.1,
                                         textvariable=self.settle_delay, width=10)
        self.settle_spinbox.grid(row=2, column=1, sticky="w", padx=(5,0))
        
        ttk.Label(self.capture_settings_frame, text="Image Format:").grid(row=3, column=0, sticky="w")
        self.format_combo = ttk.Combobox(self.capture_settings_frame, textvariable=self.image_format, 
                                        values=["JPG", "PNG", "TIFF"], state="readonly", width=8)
        self.format_combo.grid(row=3, column=1, sticky="w", padx=(5,0))
        
        # Capture controls
        self.capture_controls_frame = ttk.LabelFrame(self.capture_tab, text="Controls", padding=10)
        
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
        self.progress_frame = ttk.LabelFrame(self.capture_tab, text="Progress", padding=10)
        
        self.progress_bar = ttk.Progressbar(self.progress_frame, variable=self.capture_progress, 
                                           maximum=100, length=400)
        self.progress_bar.grid(row=0, column=0, columnspan=3, sticky="ew", pady=(0,10))
        
        self.progress_label_var = tk.StringVar(value="Ready")
        ttk.Label(self.progress_frame, textvariable=self.progress_label_var).grid(row=1, column=0, sticky="w")
        
        # Capture log
        self.capture_log_frame = ttk.LabelFrame(self.capture_tab, text="Capture Log", padding=10)
        
        self.capture_log_text = tk.Text(self.capture_log_frame, height=8, width=60)
        self.capture_log_text.grid(row=0, column=0, sticky="nsew")
        
        capture_scrollbar = ttk.Scrollbar(self.capture_log_frame, orient="vertical", 
                                         command=self.capture_log_text.yview)
        capture_scrollbar.grid(row=0, column=1, sticky="ns")
        self.capture_log_text.config(yscrollcommand=capture_scrollbar.set)
        
        # Layout capture tab
        self.capture_tab.rowconfigure(4, weight=1)
        self.capture_tab.columnconfigure(0, weight=1)
        
        self.specimen_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        self.capture_settings_frame.grid(row=1, column=0, padx=10, pady=5, sticky="ew")
        self.capture_controls_frame.grid(row=2, column=0, padx=10, pady=5, sticky="ew")
        self.progress_frame.grid(row=3, column=0, padx=10, pady=5, sticky="ew")
        self.capture_log_frame.grid(row=4, column=0, padx=10, pady=5, sticky="nsew")

    def _build_camera_settings_tab(self):
        # Camera controls
        self.camera_controls_frame = ttk.LabelFrame(self.camera_settings_tab, text="Camera Controls", padding=10)
        
        ttk.Label(self.camera_controls_frame, text="Shutter Speed (μs):").grid(row=0, column=0, sticky="w")
        self.shutter_spinbox = ttk.Spinbox(self.camera_controls_frame, from_=0, to=1000000, 
                                          textvariable=self.shutter_speed, width=15)
        self.shutter_spinbox.grid(row=0, column=1, sticky="w", padx=(5,0))
        ttk.Label(self.camera_controls_frame, text="(0 = auto)").grid(row=0, column=2, sticky="w", padx=(5,0))
        
        ttk.Label(self.camera_controls_frame, text="Brightness:").grid(row=1, column=0, sticky="w")
        self.brightness_scale = ttk.Scale(self.camera_controls_frame, from_=-1.0, to=1.0, 
                                         variable=self.brightness, orient="horizontal", length=200)
        self.brightness_scale.grid(row=1, column=1, sticky="w", padx=(5,0))
        
        self.brightness_value_var = tk.StringVar()
        ttk.Label(self.camera_controls_frame, textvariable=self.brightness_value_var).grid(row=1, column=2, sticky="w", padx=(5,0))
        
        # Update brightness display
        def update_brightness_display(*args):
            self.brightness_value_var.set(f"{self.brightness.get():.2f}")
        self.brightness.trace('w', update_brightness_display)
        update_brightness_display()
        
        self.btn_apply_settings = ttk.Button(self.camera_controls_frame, text="Apply Settings", 
                                            command=self.apply_camera_settings)
        self.btn_apply_settings.grid(row=2, column=0, columnspan=3, pady=(10,0))
        
        # Camera info
        self.camera_info_frame = ttk.LabelFrame(self.camera_settings_tab, text="Camera Information", padding=10)
        
        self.camera_info_text = tk.Text(self.camera_info_frame, height=10, width=60, state="disabled")
        self.camera_info_text.grid(row=0, column=0, sticky="nsew")
        
        info_scrollbar = ttk.Scrollbar(self.camera_info_frame, orient="vertical", 
                                      command=self.camera_info_text.yview)
        info_scrollbar.grid(row=0, column=1, sticky="ns")
        self.camera_info_text.config(yscrollcommand=info_scrollbar.set)
        
        # Layout camera settings tab
        self.camera_settings_tab.rowconfigure(1, weight=1)
        self.camera_settings_tab.columnconfigure(0, weight=1)
        
        self.camera_controls_frame.grid(row=0, column=0, padx=10, pady=5, sticky="ew")
        self.camera_info_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")

    def _layout_gui(self):
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        
        self.notebook.grid(row=0, column=0, sticky="nsew", padx=10, pady=5)
        self.status_bar.grid(row=1, column=0, sticky="ew", padx=10, pady=(5,10))

    def toggle_preview(self):
        if not PICAMERA2_OK:
            self.status_var.set("picamera2 is not installed.")
            return
        if not PIL_OK:
            self.status_var.set("PIL (pillow) is not installed.")
            return

        if not self.preview_on:
            self.start_preview()
            self.btn_preview.config(text="Stop Preview")
        else:
            self.stop_preview()
            self.btn_preview.config(text="Start Preview")

    def start_preview(self):
        try:
            if self.camera is None:
                self.camera = Picamera2()
                config = self.camera.create_preview_configuration(
                    main={"size": CAMERA_RESOLUTION, "format": "RGB888"}
                )
                self.camera.configure(config)
            self.camera.start()
            self.preview_on = True
            self.after(500, self._update_preview_frame)
        except Exception as e:
            self.status_var.set(f"Camera error: {e}")

    def stop_preview(self):
        if self.camera is not None:
            self.camera.stop()
        self.preview_on = False

    def _update_preview_frame(self):
        if not self.preview_on:
            return
        try:
            frame = self.camera.capture_array("main")
            
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(frame_rgb, 'RGB')
            else:
                image = Image.fromarray(frame)

            if image.size != CAMERA_RESOLUTION:
                image = image.resize(CAMERA_RESOLUTION, Image.LANCZOS)
            
            photo = ImageTk.PhotoImage(image)
            self.preview_image = photo
            self.preview_label.config(image=photo)
            self.after(50, self._update_preview_frame)
        except Exception as e:
            self.status_var.set(f"Preview error: {e}")
            self.stop_preview()

    def send_motor_command(self, command):
        """Send command to Arduino and wait for response"""
        try:
            if not self.ser or not self.ser.is_open:
                self.status_var.set("Serial connection not available")
                return False
            
            # Send command with newline
            full_command = command + "\n"
            self.ser.write(full_command.encode())
            self.ser.flush()
            
            # Wait for response
            time.sleep(0.2)
            
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                if response:
                    if response == "OK":
                        return True
                    elif response.startswith("ERR"):
                        self.status_var.set(f"Arduino error: {response}")
                        return False
                    else:
                        # Numeric response (position)
                        self.status_var.set(f"Arduino: {response}")
                        return response
            
            return True
        except Exception as e:
            self.status_var.set(f"Serial error: {e}")
            return False
    
    def get_motor_position(self, motor):
        """Get current position from Arduino"""
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

    def motor1_ccw(self):
        try:
            step = float(self.motor1_step_var.get())
            command = f"ROTATE 1 {step} CCW"
            
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
            command = f"ROTATE 1 {step} CW"
            
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

    def on_close(self):
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
        
        self.destroy()

# =====================================================================
# CALIBRATION METHODS
# =====================================================================
    
    def start_calibration(self):
        """Start the calibration wizard"""
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
        
        # Move to 0 degrees
        self.move_to_angle(0)
        
    def capture_calibration_position(self):
        """Capture the current linear position for calibration"""
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
        """Move to next step in calibration"""
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
            self.move_to_angle(next_angle)
        else:
            messagebox.showwarning("Calibration", "Please capture both near and far positions for current angle.")
    
    def complete_calibration(self):
        """Complete the calibration process"""
        self.is_calibrated = True
        self.calibration_status_var.set("Calibrated")
        
        # Update button states
        self.btn_start_calibration.config(state="normal")
        self.btn_capture_position.config(state="disabled")
        self.btn_next_step.config(state="disabled")
        self.btn_save_calibration.config(state="normal")
        
        self.log_calibration("Calibration complete! You can now run capture sequences.")
        
        # Update status display color
        for widget in self.calibration_status_frame.winfo_children():
            if isinstance(widget, ttk.Label) and str(widget.cget("textvariable")) == str(self.calibration_status_var):
                widget.config(foreground="green")
                break
    
    def save_calibration(self):
        """Save calibration data to file"""
        filename = filedialog.asksaveasfilename(
            defaultextension=".json",
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Save Calibration Data"
        )
        
        if filename:
            try:
                with open(filename, 'w') as f:
                    json.dump(self.calibration_data, f, indent=2)
                self.log_calibration(f"Calibration saved to {filename}")
                messagebox.showinfo("Save", "Calibration data saved successfully!")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to save calibration: {e}")
    
    def load_calibration(self):
        """Load calibration data from file"""
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json"), ("All files", "*.*")],
            title="Load Calibration Data"
        )
        
        if filename:
            try:
                with open(filename, 'r') as f:
                    self.calibration_data = json.load(f)
                
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
        """Validate that calibration data is complete"""
        for angle in CALIBRATION_ANGLES:
            if angle not in self.calibration_data:
                return False
            if "near" not in self.calibration_data[angle] or "far" not in self.calibration_data[angle]:
                return False
        return True
    
    def log_calibration(self, message):
        """Add message to calibration log"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.calibration_progress_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.calibration_progress_text.see(tk.END)
    
    def move_to_angle(self, target_angle):
        """Move motor 1 to specific angle"""
        current_angle = self.motor1_position_deg
        rotation_needed = target_angle - current_angle
        
        if rotation_needed != 0:
            direction = "CW" if rotation_needed > 0 else "CCW"
            amount = abs(rotation_needed)
            
            command = f"ROTATE 1 {amount} {direction}"
            if self.send_motor_command(command):
                self.motor1_position_deg = target_angle
                self.motor1_pos_var.set(f"{target_angle:.1f}°")
                self.status_var.set(f"Moved to {target_angle}°")
                return True
        return False
    
    def interpolate_focus_position(self, angle, stack_position):
        """
        Interpolate focus position for given angle and stack position.
        stack_position: 0.0 = near focus, 1.0 = far focus
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
        """Start the full automated capture sequence"""
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
        message += f"Stacks: {self.stacks_count.get()}\n"
        message += f"Shots per stack: {self.shots_per_stack.get()}\n"
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
        """Run the full capture sequence in a separate thread"""
        try:
            # Create output directory structure
            specimen_dir = os.path.join(self.output_dir.get(), self.specimen_name.get())
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            session_dir = os.path.join(specimen_dir, f"session_{timestamp}")
            os.makedirs(session_dir, exist_ok=True)
            
            # Save capture metadata
            metadata = {
                "specimen_name": self.specimen_name.get(),
                "timestamp": timestamp,
                "stacks": self.stacks_count.get(),
                "shots_per_stack": self.shots_per_stack.get(),
                "settle_delay": self.settle_delay.get(),
                "image_format": self.image_format.get(),
                "calibration_data": self.calibration_data
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
            
            total_images = self.stacks_count.get() * self.shots_per_stack.get()
            image_count = 0
            
            self.after(0, lambda: self.log_capture(f"Starting capture sequence: {total_images} images"))
            
            # Calculate angle step
            angle_step = 360.0 / self.shots_per_stack.get()
            
            # For each stack
            for stack in range(self.stacks_count.get()):
                if not self.capture_running:
                    break
                
                stack_dir = os.path.join(session_dir, f"stack_{stack:02d}")
                os.makedirs(stack_dir, exist_ok=True)
                
                # Calculate focus position for this stack
                stack_position = stack / (self.stacks_count.get() - 1) if self.stacks_count.get() > 1 else 0.0
                
                self.after(0, lambda s=stack: self.log_capture(f"Starting stack {s + 1}/{self.stacks_count.get()}"))
                
                # For each shot in the stack
                for shot in range(self.shots_per_stack.get()):
                    if not self.capture_running:
                        break
                    
                    # Calculate angle and focus position
                    angle = shot * angle_step
                    focus_position = self.interpolate_focus_position(angle, stack_position)
                    
                    # Move to position
                    self.after(0, lambda a=angle: self.move_to_angle(a))
                    time.sleep(0.5)  # Let rotation settle
                    
                    self.after(0, lambda pos=focus_position: self.move_to_focus_position(pos))
                    time.sleep(self.settle_delay.get())  # Settle delay
                    
                    # Capture image
                    filename = f"stack_{stack:02d}_shot_{shot:03d}_angle_{angle:06.2f}.{self.image_format.get().lower()}"
                    filepath = os.path.join(stack_dir, filename)
                    
                    self.capture_image(filepath)
                    
                    image_count += 1
                    progress = (image_count / total_images) * 100
                    
                    self.after(0, lambda p=progress: self.capture_progress.set(p))
                    self.after(0, lambda i=image_count, t=total_images: 
                              self.progress_label_var.set(f"Captured {i}/{t} images"))
                    
                    if image_count % 10 == 0:  # Log every 10 images
                        self.after(0, lambda i=image_count: self.log_capture(f"Captured {i} images"))
            
            # Complete
            if self.capture_running:
                self.after(0, lambda: self.log_capture(f"Capture sequence complete! {image_count} images saved to {session_dir}"))
                self.after(0, lambda: messagebox.showinfo("Complete", f"Capture sequence complete!\n{image_count} images saved."))
            
        except Exception as e:
            self.after(0, lambda: self.log_capture(f"Capture error: {e}"))
            self.after(0, lambda: messagebox.showerror("Error", f"Capture failed: {e}"))
        
        finally:
            self.capture_running = False
            self.after(0, lambda: self.btn_start_capture.config(state="normal"))
            self.after(0, lambda: self.btn_stop_capture.config(state="disabled"))
    
    def move_to_focus_position(self, target_position):
        """Move motor 2 to specific position"""
        current_position = self.motor2_position_mm
        move_distance = target_position - current_position
        
        if abs(move_distance) > 0.01:  # Only move if significant difference
            direction = "FORWARD" if move_distance > 0 else "BACKWARD"
            amount = abs(move_distance)
            
            command = f"MOVE 2 {amount} {direction}"
            if self.send_motor_command(command):
                self.motor2_position_mm = target_position
                self.motor2_pos_var.set(f"{target_position:.2f}mm")
                return True
        return False
    
    def capture_image(self, filepath):
        """Capture an image to the specified filepath"""
        if self.camera is None or not self.camera.started:
            raise Exception("Camera not started")
        
        # Capture high-resolution image
        if self.image_format.get().upper() == "JPG":
            self.camera.capture_file(filepath)
        else:
            # For PNG/TIFF, capture array and save with PIL
            array = self.camera.capture_array("main")
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
            else:
                controls["AeEnable"] = True  # Auto exposure
            
            # Brightness
            if abs(self.brightness.get()) > 0.01:
                controls["Brightness"] = self.brightness.get()
            
            if controls:
                self.camera.set_controls(controls)
                self.status_var.set("Camera settings applied")
            
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

if __name__ == "__main__":
    print("Starting 3D Scanner Control Panel v4.0...")
    
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
