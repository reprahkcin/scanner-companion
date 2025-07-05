#!/usr/bin/env python3
importSERIAL_PORT       = "/dev/ttyACM0"  # or "/dev/serial/by-id/..." 
SERIAL_BAUDRATE   = 9600              # Match Arduino sketch baudrate
CAMERA_RESOLUTION = (320, 240)       # preview size (smaller for safety)
JOG_STEP_DELAY    = 0.05             # seconds between steps during jognter as tk
from tkinter import ttk
from tkinter import messagebox
import threading
import time
import cv2

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

SERIAL_PORT       = "/dev/ttyACM0"  # or "/dev/serial/by-id/...” 
SERIAL_BAUDRATE   = 115200
CAMERA_RESOLUTION = (320, 240)       # preview size (smaller for safety)
JOG_STEP_DELAY    = 0.05             # seconds between steps during jog
# ──────────────────────────────────────────────────────────────────────────────

class ScannerGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("3D Scanner Control Panel - v3.0")
        self.minsize(500, 500)  # Set a minimum window size
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # === Serial Setup ===
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0.1)
        except serial.SerialException as e:
            messagebox.showerror("Serial Error", f"Could not open serial port:\n{e}")
            self.destroy()
            return

        # === Camera & Preview Attributes ===
        self.camera    = None
        self.preview_on = False
        self.preview_image = None

        # === Jog‐thread flags ===
        self.jog_forward_active  = False
        self.jog_backward_active = False

        # === Motor 1 & Motor 2 position trackers (optional) ===
        self.motor1_position_deg = 0.0
        self.motor2_position_mm  = 0.0

        # Build the GUI
        self._build_gui()
        self._layout_gui()

    def _build_gui(self):
        # --- Camera Frame ---
        self.camera_frame = ttk.LabelFrame(self, text="Camera", padding=10)
        # Remove width and height so the label auto-sizes to the image
        self.preview_label = tk.Label(self.camera_frame, bg="black", anchor="center")
        self.preview_label.grid(row=0, column=0, columnspan=2, padx=5, pady=5)
        # Connect preview button to toggle_preview
        self.btn_preview = ttk.Button(self.camera_frame, text="Start Preview", command=self.toggle_preview)
        self.btn_preview.grid(row=1, column=0, columnspan=2, pady=(5,0))

        # --- Motor 1 Frame ---
        self.motor1_frame = ttk.LabelFrame(self, text="Motor 1 (Rotation)", padding=10)
        
        # Position display
        self.motor1_pos_var = tk.StringVar(value="0.0°")
        ttk.Label(self.motor1_frame, text="Position:").grid(row=0, column=0, sticky="w")
        ttk.Label(self.motor1_frame, textvariable=self.motor1_pos_var, font=("TkDefaultFont", 10, "bold")).grid(row=0, column=1, sticky="w")
        
        # Step size control
        ttk.Label(self.motor1_frame, text="Step Size:").grid(row=1, column=0, sticky="w")
        self.motor1_step_var = tk.StringVar(value="1.0")
        self.motor1_step_entry = ttk.Entry(self.motor1_frame, textvariable=self.motor1_step_var, width=8)
        self.motor1_step_entry.grid(row=1, column=1, sticky="w")
        ttk.Label(self.motor1_frame, text="degrees").grid(row=1, column=2, sticky="w")
        
        # Control buttons
        btn_frame1 = ttk.Frame(self.motor1_frame)
        btn_frame1.grid(row=2, column=0, columnspan=3, pady=(10,0))
        
        self.btn_m1_ccw = ttk.Button(btn_frame1, text="◀ CCW", command=self.motor1_ccw)
        self.btn_m1_ccw.grid(row=0, column=0, padx=(0,5))
        
        self.btn_m1_home = ttk.Button(btn_frame1, text="⌂ Home", command=self.motor1_home)
        self.btn_m1_home.grid(row=0, column=1, padx=5)
        
        self.btn_m1_cw = ttk.Button(btn_frame1, text="CW ▶", command=self.motor1_cw)
        self.btn_m1_cw.grid(row=0, column=2, padx=(5,0))
        
        # Goto position
        ttk.Label(self.motor1_frame, text="Go to:").grid(row=3, column=0, sticky="w", pady=(10,0))
        self.motor1_goto_var = tk.StringVar(value="0.0")
        self.motor1_goto_entry = ttk.Entry(self.motor1_frame, textvariable=self.motor1_goto_var, width=8)
        self.motor1_goto_entry.grid(row=3, column=1, pady=(10,0))
        self.btn_m1_goto = ttk.Button(self.motor1_frame, text="Go", command=self.motor1_goto)
        self.btn_m1_goto.grid(row=3, column=2, pady=(10,0), padx=(5,0))

        # --- Motor 2 Frame ---
        self.motor2_frame = ttk.LabelFrame(self, text="Motor 2 (Linear)", padding=10)
        
        # Position display
        self.motor2_pos_var = tk.StringVar(value="0.0mm")
        ttk.Label(self.motor2_frame, text="Position:").grid(row=0, column=0, sticky="w")
        ttk.Label(self.motor2_frame, textvariable=self.motor2_pos_var, font=("TkDefaultFont", 10, "bold")).grid(row=0, column=1, sticky="w")
        
        # Step size control
        ttk.Label(self.motor2_frame, text="Step Size:").grid(row=1, column=0, sticky="w")
        self.motor2_step_var = tk.StringVar(value="1.0")
        self.motor2_step_entry = ttk.Entry(self.motor2_frame, textvariable=self.motor2_step_var, width=8)
        self.motor2_step_entry.grid(row=1, column=1, sticky="w")
        ttk.Label(self.motor2_frame, text="mm").grid(row=1, column=2, sticky="w")
        
        # Control buttons
        btn_frame2 = ttk.Frame(self.motor2_frame)
        btn_frame2.grid(row=2, column=0, columnspan=3, pady=(10,0))
        
        self.btn_m2_down = ttk.Button(btn_frame2, text="▼ Down", command=self.motor2_down)
        self.btn_m2_down.grid(row=0, column=0, padx=(0,5))
        
        self.btn_m2_home = ttk.Button(btn_frame2, text="⌂ Home", command=self.motor2_home)
        self.btn_m2_home.grid(row=0, column=1, padx=5)
        
        self.btn_m2_up = ttk.Button(btn_frame2, text="▲ Up", command=self.motor2_up)
        self.btn_m2_up.grid(row=0, column=2, padx=(5,0))
        
        # Goto position
        ttk.Label(self.motor2_frame, text="Go to:").grid(row=3, column=0, sticky="w", pady=(10,0))
        self.motor2_goto_var = tk.StringVar(value="0.0")
        self.motor2_goto_entry = ttk.Entry(self.motor2_frame, textvariable=self.motor2_goto_var, width=8)
        self.motor2_goto_entry.grid(row=3, column=1, pady=(10,0))
        self.btn_m2_goto = ttk.Button(self.motor2_frame, text="Go", command=self.motor2_goto)
        self.btn_m2_goto.grid(row=3, column=2, pady=(10,0), padx=(5,0))

        # --- Capture Routine Builder Frame ---
        self.capture_frame = ttk.LabelFrame(self, text="Capture Routine", padding=10)
        
        # Scan parameters
        ttk.Label(self.capture_frame, text="Steps per rotation:").grid(row=0, column=0, sticky="w")
        self.steps_var = tk.StringVar(value="36")  # 10 degree steps
        ttk.Entry(self.capture_frame, textvariable=self.steps_var, width=8).grid(row=0, column=1, sticky="w")
        
        ttk.Label(self.capture_frame, text="Delay between shots:").grid(row=1, column=0, sticky="w")
        self.delay_var = tk.StringVar(value="2.0")
        ttk.Entry(self.capture_frame, textvariable=self.delay_var, width=8).grid(row=1, column=1, sticky="w")
        ttk.Label(self.capture_frame, text="seconds").grid(row=1, column=2, sticky="w")
        
        # Control buttons
        capture_btn_frame = ttk.Frame(self.capture_frame)
        capture_btn_frame.grid(row=2, column=0, columnspan=3, pady=(10,0))
        
        self.btn_start_scan = ttk.Button(capture_btn_frame, text="Start Scan", command=self.start_capture_routine)
        self.btn_start_scan.grid(row=0, column=0, padx=(0,5))
        
        self.btn_stop_scan = ttk.Button(capture_btn_frame, text="Stop Scan", command=self.stop_capture_routine, state="disabled")
        self.btn_stop_scan.grid(row=0, column=1, padx=5)
        
        # Progress display
        ttk.Label(self.capture_frame, text="Progress:").grid(row=3, column=0, sticky="w", pady=(10,0))
        self.progress_var = tk.StringVar(value="Ready")
        ttk.Label(self.capture_frame, textvariable=self.progress_var).grid(row=3, column=1, columnspan=2, sticky="w", pady=(10,0))

        # Status Bar (optional)
        self.status_var = tk.StringVar(value="Idle")
        self.status_bar = ttk.Label(self, textvariable=self.status_var, relief="sunken", anchor="w")

    def _layout_gui(self):
        # Configure grid weights for responsive layout
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        self.rowconfigure(2, weight=1)
        
        # Camera on top, spanning both columns
        self.camera_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=5, sticky="nsew")
        
        # Motor frames side by side
        self.motor1_frame.grid(row=1, column=0, padx=(10,5), pady=5, sticky="nsew")
        self.motor2_frame.grid(row=1, column=1, padx=(5,10), pady=5, sticky="nsew")
        
        # Capture routine spanning both columns
        self.capture_frame.grid(row=2, column=0, columnspan=2, padx=10, pady=5, sticky="nsew")
        
        # Status bar at the bottom
        self.status_bar.grid(row=99, column=0, columnspan=2, sticky="ew", padx=10, pady=(5,10))

    def toggle_preview(self):
        """Start or stop the live camera preview."""
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
        """Initialize Picamera2 and begin updating frames."""
        try:
            # Check for available cameras before creating Picamera2 instance
            if hasattr(Picamera2, "available_cameras"):
                if not Picamera2.available_cameras:
                    self.status_var.set("No camera detected by Picamera2.")
                    return
            if self.camera is None:
                self.camera = Picamera2()
                # Use main stream in RGB format instead of lores
                config = self.camera.create_preview_configuration(
                    main={"size": CAMERA_RESOLUTION, "format": "RGB888"}
                )
                self.camera.configure(config)
            self.camera.start()
            self.preview_on = True
            # Wait for camera to warm up before first frame
            self.after(500, self._update_preview_frame)
        except Exception as e:
            self.status_var.set(f"Camera error: {e}")

    def stop_preview(self):
        """Stop Picamera2 preview."""
        if self.camera is not None:
            self.camera.stop()
        self.preview_on = False

    def _update_preview_frame(self):
        """Grab a frame from Picamera2 and display it in the Label."""
        if not self.preview_on:
            return
        try:
            # Capture from main stream (RGB888)
            frame = self.camera.capture_array("main")
            
            # Handle color channel ordering
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                # Frame should be RGB, but might be BGR - let's convert to ensure correct colors
                # Picamera2 sometimes outputs BGR despite RGB888 format specification
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                image = Image.fromarray(frame_rgb, 'RGB')
            else:
                # Fallback: convert whatever format we got
                self.status_var.set(f"Unexpected frame format: {frame.shape}")
                # Try to handle it gracefully
                if len(frame.shape) == 2:
                    # Grayscale
                    image = Image.fromarray(frame, 'L').convert('RGB')
                else:
                    # Unknown format, try as-is
                    image = Image.fromarray(frame)

            # Resize for display (frame should already be the right size)
            if image.size != CAMERA_RESOLUTION:
                image = image.resize(CAMERA_RESOLUTION, Image.LANCZOS)
            
            photo = ImageTk.PhotoImage(image)

            # Keep a reference to avoid Python GC deleting it
            self.preview_image = photo
            self.preview_label.config(image=photo)

            # Call again after a short delay (50ms = 20fps for stability)
            self.after(50, self._update_preview_frame)
        except Exception as e:
            self.status_var.set(f"Preview error: {e}")
            # Optionally, stop preview on error
            self.stop_preview()

    def on_close(self):
        # Minimal cleanup for now
        self.destroy()

    # ──────────────────────────────────────────────────────────────────────────────
    # MOTOR 1 CONTROL METHODS (Rotation Motor)
    # ──────────────────────────────────────────────────────────────────────────────
    
    def motor1_ccw(self):
        """Move Motor 1 counter-clockwise by step amount."""
        try:
            step = float(self.motor1_step_var.get())
            steps = self.calculate_steps_from_degrees(step)
            
            if self.send_motor_command('1', steps, 'B'):  # CCW = Backward
                self.motor1_position_deg -= step
                self.motor1_pos_var.set(f"{self.motor1_position_deg:.1f}°")
                self.status_var.set(f"Motor 1: CCW {step}° ({steps} steps)")
            else:
                self.status_var.set("Failed to send motor command")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 1")
    
    def motor1_cw(self):
        """Move Motor 1 clockwise by step amount."""
        try:
            step = float(self.motor1_step_var.get())
            steps = self.calculate_steps_from_degrees(step)
            
            if self.send_motor_command('1', steps, 'F'):  # CW = Forward
                self.motor1_position_deg += step
                self.motor1_pos_var.set(f"{self.motor1_position_deg:.1f}°")
                self.status_var.set(f"Motor 1: CW {step}° ({steps} steps)")
            else:
                self.status_var.set("Failed to send motor command")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 1")
    
    def motor1_home(self):
        """Return Motor 1 to home position (0 degrees)."""
        # Calculate steps needed to return to zero
        current_pos = self.motor1_position_deg
        if current_pos != 0:
            steps = self.calculate_steps_from_degrees(abs(current_pos))
            direction = 'B' if current_pos > 0 else 'F'
            
            if self.send_motor_command('1', steps, direction):
                self.motor1_position_deg = 0.0
                self.motor1_pos_var.set("0.0°")
                self.status_var.set("Motor 1: Homed to 0°")
            else:
                self.status_var.set("Failed to home Motor 1")
        else:
            self.status_var.set("Motor 1: Already at home position")
    
    def motor1_goto(self):
        """Move Motor 1 to specific position."""
        try:
            target = float(self.motor1_goto_var.get())
            current_pos = self.motor1_position_deg
            diff = target - current_pos
            
            if abs(diff) > 0.1:  # Only move if difference is significant
                steps = self.calculate_steps_from_degrees(abs(diff))
                direction = 'F' if diff > 0 else 'B'
                
                if self.send_motor_command('1', steps, direction):
                    self.motor1_position_deg = target
                    self.motor1_pos_var.set(f"{self.motor1_position_deg:.1f}°")
                    self.status_var.set(f"Motor 1: Moved to {target}°")
                else:
                    self.status_var.set("Failed to move Motor 1")
            else:
                self.status_var.set("Motor 1: Already at target position")
        except ValueError:
            self.status_var.set("Invalid goto position for Motor 1")
    
    # ──────────────────────────────────────────────────────────────────────────────
    # MOTOR 2 CONTROL METHODS (Linear Motor)
    # ──────────────────────────────────────────────────────────────────────────────
    
    def motor2_up(self):
        """Move Motor 2 up by step amount."""
        try:
            step = float(self.motor2_step_var.get())
            steps = self.calculate_steps_from_mm(step)
            
            if self.send_motor_command('2', steps, 'F'):  # Up = Forward
                self.motor2_position_mm += step
                self.motor2_pos_var.set(f"{self.motor2_position_mm:.1f}mm")
                self.status_var.set(f"Motor 2: Up {step}mm ({steps} steps)")
            else:
                self.status_var.set("Failed to send motor command")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 2")
    
    def motor2_down(self):
        """Move Motor 2 down by step amount."""
        try:
            step = float(self.motor2_step_var.get())
            steps = self.calculate_steps_from_mm(step)
            
            if self.send_motor_command('2', steps, 'B'):  # Down = Backward
                self.motor2_position_mm -= step
                self.motor2_pos_var.set(f"{self.motor2_position_mm:.1f}mm")
                self.status_var.set(f"Motor 2: Down {step}mm ({steps} steps)")
            else:
                self.status_var.set("Failed to send motor command")
        except ValueError:
            self.status_var.set("Invalid step size for Motor 2")
    
    def motor2_home(self):
        """Return Motor 2 to home position (0 mm)."""
        # Calculate steps needed to return to zero
        current_pos = self.motor2_position_mm
        if abs(current_pos) > 0.1:  # Only move if not already at home
            steps = self.calculate_steps_from_mm(abs(current_pos))
            direction = 'B' if current_pos > 0 else 'F'
            
            if self.send_motor_command('2', steps, direction):
                self.motor2_position_mm = 0.0
                self.motor2_pos_var.set("0.0mm")
                self.status_var.set("Motor 2: Homed to 0mm")
            else:
                self.status_var.set("Failed to home Motor 2")
        else:
            self.status_var.set("Motor 2: Already at home position")
    
    def motor2_goto(self):
        """Move Motor 2 to specific position."""
        try:
            target = float(self.motor2_goto_var.get())
            current_pos = self.motor2_position_mm
            diff = target - current_pos
            
            if abs(diff) > 0.1:  # Only move if difference is significant
                steps = self.calculate_steps_from_mm(abs(diff))
                direction = 'F' if diff > 0 else 'B'
                
                if self.send_motor_command('2', steps, direction):
                    self.motor2_position_mm = target
                    self.motor2_pos_var.set(f"{self.motor2_position_mm:.1f}mm")
                    self.status_var.set(f"Motor 2: Moved to {target}mm")
                else:
                    self.status_var.set("Failed to move Motor 2")
            else:
                self.status_var.set("Motor 2: Already at target position")
        except ValueError:
            self.status_var.set("Invalid goto position for Motor 2")
    
    # ──────────────────────────────────────────────────────────────────────────────
    # CAPTURE ROUTINE METHODS
    # ──────────────────────────────────────────────────────────────────────────────
    
    def start_capture_routine(self):
        """Start the automated capture routine."""
        try:
            steps = int(self.steps_var.get())
            delay = float(self.delay_var.get())
            self.btn_start_scan.config(state="disabled")
            self.btn_stop_scan.config(state="normal")
            self.progress_var.set(f"Starting scan: {steps} steps, {delay}s delay")
            self.status_var.set("Capture routine started")
            # TODO: Implement actual capture routine in separate thread
        except ValueError:
            self.status_var.set("Invalid scan parameters")
    
    def stop_capture_routine(self):
        """Stop the automated capture routine."""
        self.btn_start_scan.config(state="normal")
        self.btn_stop_scan.config(state="disabled")
        self.progress_var.set("Scan stopped")
        self.status_var.set("Capture routine stopped")
        # TODO: Stop capture thread

    # ──────────────────────────────────────────────────────────────────────────────
    # SERIAL COMMUNICATION METHODS
    # ──────────────────────────────────────────────────────────────────────────────
    
    def send_motor_command(self, motor, steps, direction):
        """Send command to Arduino in format: {motor}{steps}{direction}"""
        try:
            if not hasattr(self, 'ser') or not self.ser.is_open:
                self.status_var.set("Serial connection not available")
                return False
            
            # Format: motor (1 or 2), steps (integer), direction (F or B)
            command = f"{motor}{steps}{direction}"
            self.ser.write(command.encode())
            self.ser.flush()
            
            # Wait a moment for Arduino to process
            time.sleep(0.1)
            
            # Check for any response from Arduino
            if self.ser.in_waiting > 0:
                response = self.ser.readline().decode().strip()
                if response:
                    self.status_var.set(f"Arduino: {response}")
            
            return True
        except Exception as e:
            self.status_var.set(f"Serial error: {e}")
            return False
    
    def calculate_steps_from_degrees(self, degrees):
        """Convert degrees to motor steps (assuming 200 steps per revolution)"""
        steps_per_revolution = 200  # Typical stepper motor
        return int((degrees / 360.0) * steps_per_revolution)
    
    def calculate_steps_from_mm(self, mm):
        """Convert mm to motor steps (assuming 1mm = 10 steps - adjust as needed)"""
        steps_per_mm = 10  # Adjust this based on your linear actuator setup
        return int(mm * steps_per_mm)

    def on_close(self):
        """Clean up resources before closing."""
        # Stop camera preview
        if self.preview_on:
            self.stop_preview()
        
        # Close camera if it exists
        if self.camera is not None:
            try:
                self.camera.close()
            except:
                pass
        
        # Close serial connection
        if hasattr(self, 'ser') and self.ser.is_open:
            try:
                self.ser.close()
            except:
                pass
        
        # Destroy the window
        self.destroy()

# ──────────────────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    app = ScannerGUI()
    app.mainloop()
