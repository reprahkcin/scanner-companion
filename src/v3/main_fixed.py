#!/usr/bin/env python3
import tkinter as tk
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

SERIAL_PORT       = "/dev/ttyACM0"  # or "/dev/serial/by-id/..." 
SERIAL_BAUDRATE   = 115200            # Match motor_control_refactor.ino baudrate
CAMERA_RESOLUTION = (320, 240)       # preview size (smaller for safety)
JOG_STEP_DELAY    = 0.05             # seconds between steps during jog
# ──────────────────────────────────────────────────────────────────────────────

class ScannerGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("3D Scanner Control Panel - v3.0")
        self.minsize(500, 500)
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

        # Build the GUI
        self._build_gui()
        self._layout_gui()

    def _build_gui(self):
        # --- Camera Frame ---
        self.camera_frame = ttk.LabelFrame(self, text="Camera", padding=10)
        self.preview_label = tk.Label(self.camera_frame, bg="black", anchor="center")
        self.preview_label.grid(row=0, column=0, columnspan=2, padx=5, pady=5)
        self.btn_preview = ttk.Button(self.camera_frame, text="Start Preview", command=self.toggle_preview)
        self.btn_preview.grid(row=1, column=0, columnspan=2, pady=(5,0))

        # --- Motor 1 Frame ---
        self.motor1_frame = ttk.LabelFrame(self, text="Motor 1 (Rotation)", padding=10)
        
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
        self.motor2_frame = ttk.LabelFrame(self, text="Motor 2 (Linear)", padding=10)
        
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

        # Status Bar
        self.status_bar = ttk.Label(self, textvariable=self.status_var, relief="sunken", anchor="w")

    def _layout_gui(self):
        self.columnconfigure(0, weight=1)
        self.columnconfigure(1, weight=1)
        self.rowconfigure(0, weight=1)
        self.rowconfigure(1, weight=1)
        
        self.camera_frame.grid(row=0, column=0, columnspan=2, padx=10, pady=5, sticky="nsew")
        self.motor1_frame.grid(row=1, column=0, padx=(10,5), pady=5, sticky="nsew")
        self.motor2_frame.grid(row=1, column=1, padx=(5,10), pady=5, sticky="nsew")
        self.status_bar.grid(row=99, column=0, columnspan=2, sticky="ew", padx=10, pady=(5,10))

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

if __name__ == "__main__":
    app = ScannerGUI()
    app.mainloop()
