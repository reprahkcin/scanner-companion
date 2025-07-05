# 3D Scanner Control Panel (v3.0) — Hand-Off Document

## 1. Project Overview

This repository contains code and configuration for a Raspberry Pi + Arduino-based 3D scanner control panel. The Pi runs a Python Tkinter GUI (`main.py`) that:

- Communicates over serial (115 200 baud) with an Arduino Uno.
- Provides live preview from a Raspberry Pi HQ camera (via Picamera2 + PIL).
- Offers controls for:
  - **Motor 1** (turntable) — moves in accurate degrees.
  - **Motor 2** (linear camera carriage) — moves in millimetres.
- Contains a “Scanning Capture Routine Builder” that will eventually automate multi-shot stacks over a full sweep.

The Arduino (`motor_control_api.ino`) exposes a simple ASCII protocol (`ROTATE`, `MOVE`, `ZERO`, `GET_POS`). Calibration constants ensure that “degrees” and “millimetres” match real hardware motion (within 0.01 mm and small fractions of a degree).

---

## 2. Hardware Setup & Pin Assignments

### 2.1 Arduino Uno <→> Stepper Driver Wiring

Both stepper drivers are set to **1/16 microstepping** (3 200 microsteps/rev), 1.5 A/1.7 A.

#### Motor 1 (Turntable)

- **Stepper Driver DIP**: 1/16 microstepping → 3 200 pulses/rev.
- **Physical Gearing**:
  - Motor’s 2 cm-diameter driving sprocket → 15 cm-diameter turntable sprocket (7.5:1 ratio).
  - Effective: 24 000 microsteps/rev of the **turntable** → 66.667 microsteps per turntable degree (theoretical), but use 75 µsteps/° for conservative calibration.

| Arduino Pin | Stepper Driver Signal     | Purpose                          |
|:-----------:|:--------------------------|:---------------------------------|
| Pin 2       | STEP_PIN_1                | Pulse (step) for Motor 1         |
| Pin 3       | DIR_PIN_1                 | Direction (HIGH=“CW”, LOW=“CCW”) |
| Pin 13      | ENABLE_PIN_1              | Enable/Disable driver (active HIGH) |

#### Motor 2 (Linear Camera Carriage)

- **Stepper Driver DIP**: 1/16 microstepping → 3 200 pulses/rev.
- **Leadscrew Pitch**: **5 mm per rev** (based on calibration tests).
  - → 640 microsteps/mm (`3200 / 5.0`).
- **Limit Switch** (home/back end) wired to pull-up input.

| Arduino Pin | Signal                    | Purpose                                            |
|:-----------:|:--------------------------|:---------------------------------------------------|
| Pin 4       | STEP_PIN_2                | Pulse (step) for Motor 2                           |
| Pin 5       | DIR_PIN_2                 | Direction (HIGH=“FORWARD”, LOW=“BACKWARD”)         |
| Pin 12      | ENABLE_PIN_2              | Enable/Disable driver (active HIGH)                |
| Pin 7       | LIMIT_SWITCH_2 (INPUT_PULLUP) | Normally HIGH, goes LOW when the carriage hits home |

---

## 3. Arduino Sketch: `motor_control_api.ino`

This sketch implements four ASCII commands over serial at 115 200 baud. After uploading, you should see this banner in Serial Monitor:

```
Ready for ROTATE, MOVE, ZERO, GET_POS
```

### 3.1 Calibration Constants

```cpp
// 1/16 microstepping → 3200 µsteps per motor rev.

// Motor 1 (turntable):
//  - Physical gearing: 2 cm→15 cm (7.5:1), so 3200×7.5=24000 µsteps per 360° → 66.667 µsteps/° theoretically.
//  - We use 75.0f for a slight safety margin (chain slack, tolerances).
const float STEPS_PER_DEGREE_M1 = 75.0f;  

// Motor 2 (linear carriage):
//  - 1/16 microstepping → 3200 µsteps/rev.
//  - Actual leadscrew: 5 mm/rev → 3200/5.0 = 640 µsteps/mm.
const float LEAD_MM         = 5.0f;               
const float STEPS_PER_MM_M2 = 3200.0f / LEAD_MM;  // 640.0
```

### 3.2 Direction-Reversal Flags

```cpp
// If the motor spins the wrong way when you say “CW” or “FORWARD,” flip these.
const bool DIR_REVERSE_M1 = false;
const bool DIR_REVERSE_M2 = false;
```

### 3.3 Commands & Their Syntax

1. **ROTATE <motor> <degrees> <CW|CCW>**  
   - Example: `ROTATE 1 45.0 CW`  
   - Motor 1 moves turntable by <degrees> in chosen direction.  

2. **MOVE <motor> <millimetres> <FORWARD|BACKWARD>**  
   - Example: `MOVE 2 10.0 FORWARD`  
   - Motor 2 moves the linear carriage by <mm> in chosen direction.  

3. **ZERO <motor>**  
   - Example: `ZERO 1` or `ZERO 2`  
   - Resets that motor’s internal “position” counter to 0.  

4. **GET_POS <motor>**  
   - Example: `GET_POS 2`  
   - Prints the current software-tracked position (float) of Motor 1 (degrees) or Motor 2 (mm).  

5. **Reply Logic** (inside `handleCommand()`):
   - After a valid `ROTATE` or `MOVE` or `ZERO`: `Serial.println("OK");`
   - For `GET_POS`: `Serial.println(<position>, 3);`
   - For malformed commands: `Serial.println("ERR: unknown or malformed command");`

### 3.4 Key Code Snippets

```cpp
// Enable/Disable driver pins
void enableDriver(int motor_id) {
  if (motor_id == 1) digitalWrite(ENABLE_PIN_1, ENABLE_ACTIVE_HIGH);
  else               digitalWrite(ENABLE_PIN_2, ENABLE_ACTIVE_HIGH);
}
void disableDriver(int motor_id) {
  if (motor_id == 1) digitalWrite(ENABLE_PIN_1, !ENABLE_ACTIVE_HIGH);
  else               digitalWrite(ENABLE_PIN_2, !ENABLE_ACTIVE_HIGH);
}

// Low-level stepping (Motor 2 checks limit switch if moving “backward”)
void stepMotor(int stepPin, int dirPin, bool dirHigh, long steps, bool isMotor2) {
  digitalWrite(dirPin, dirHigh ? HIGH : LOW);
  for (long i = 0; i < steps; ++i) {
    if (isMotor2 && !dirHigh && digitalRead(LIMIT_SWITCH_2) == LOW) {
      break;  // stop if home switch pressed while moving backward
    }
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(PULSE_DELAY_US);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(PULSE_DELAY_US);

    // Update software position counters:
    if (stepPin == STEP_PIN_1) {
      positionM1_deg += dirHigh
        ? (1.0f / STEPS_PER_DEGREE_M1)
        : -(1.0f / STEPS_PER_DEGREE_M1);
    } else {
      positionM2_mm += dirHigh
        ? (1.0f / STEPS_PER_MM_M2)
        : -(1.0f / STEPS_PER_MM_M2);
    }
  }
}

// High-level rotate (Motor 1)
void rotateMotor(int motor, float degrees, bool cw) {
  if (motor != 1) return;
  long steps = lround(degrees * STEPS_PER_DEGREE_M1);
  bool dirHigh = cw ^ DIR_REVERSE_M1;
  enableDriver(1);
  stepMotor(STEP_PIN_1, DIR_PIN_1, dirHigh, steps, false);
  disableDriver(1);
}

// High-level move (Motor 2)
void moveMotor(int motor, float mm, bool forward) {
  if (motor != 2) return;
  long steps = lround(mm * STEPS_PER_MM_M2);
  bool dirHigh = forward ^ DIR_REVERSE_M2;
  enableDriver(2);
  stepMotor(STEP_PIN_2, DIR_PIN_2, dirHigh, steps, true);
  disableDriver(2);
}

// Serial command parser
void handleCommand(const String &line) {
  String s = line; s.trim();
  if (s.length() == 0) return;

  // Tokenize into tok[0..3]
  String tok[4];
  int tc=0, start=0;
  for (int i=0; i <= s.length() && tc < 4; ++i) {
    if (i == s.length() || s.charAt(i) == ' ') {
      if (i - start > 0) tok[tc++] = s.substring(start, i);
      start = i + 1;
    }
  }

  if (tok[0] == "ROTATE" && tc == 4) {
    int m    = tok[1].toInt();
    float deg= tok[2].toFloat();
    bool cw  = (tok[3] == "CW");
    rotateMotor(m, deg, cw);
    Serial.println("OK");
  }
  else if (tok[0] == "MOVE" && tc == 4) {
    int m      = tok[1].toInt();
    float mmv  = tok[2].toFloat();
    bool fwd   = (tok[3] == "FORWARD");
    moveMotor(m, mmv, fwd);
    Serial.println("OK");
  }
  else if (tok[0] == "ZERO" && tc >= 2) {
    int m = tok[1].toInt();
    if      (m == 1) positionM1_deg = 0.0f, Serial.println("OK");
    else if (m == 2) positionM2_mm  = 0.0f, Serial.println("OK");
    else             Serial.println("ERR: ZERO unsupported for motor " + tok[1]);
  }
  else if (tok[0] == "GET_POS" && tc >= 2) {
    int m = tok[1].toInt();
    if      (m == 1) Serial.println(positionM1_deg, 3);
    else if (m == 2) Serial.println(positionM2_mm, 3);
    else             Serial.println("ERR: GET_POS unsupported for motor " + tok[1]);
  }
  else {
    Serial.println("ERR: unknown or malformed command");
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(STEP_PIN_1,   OUTPUT);
  pinMode(DIR_PIN_1,    OUTPUT);
  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2,   OUTPUT);
  pinMode(DIR_PIN_2,    OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);
  pinMode(LIMIT_SWITCH_2, INPUT_PULLUP);
  disableDriver(1);
  disableDriver(2);
  Serial.println("Ready for ROTATE, MOVE, ZERO, GET_POS");
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleCommand(line);
  }
}
```

---

## 4. Python GUI: `main.py`

The Python script uses **Tkinter** for layout and **Picamera2 + PIL** (or fallback) for live preview. It communicates over serial with the Arduino, sending the exact same ASCII commands described above.

### 4.1 Dependencies

```bash
sudo apt update
sudo apt install -y python3-pil python3-serial python3-picamera2
```

If `picamera2` isn’t available or if you prefer OpenCV, replace the preview code (in `start_preview()` / `_update_preview_frame()`) with a `cv2.VideoCapture(0)` loop and `ImageTk`.

### 4.2 GUI Layout Summary

1. **Camera Frame**  
   - `self.preview_label`: `tk.Label` (black background) to hold the live feed.  
   - `self.btn_preview`: toggles Start/Stop Preview.  

2. **Motor 1 Frame**  
   - `self.m1_inc_combo`: `ttk.Combobox` for degree increments (`["1","5","15","30","60","90","180","360"]`).  
   - Buttons:  
     - `btn_m1_cw`: sends `ROTATE 1 <inc> CW`.  
     - `btn_m1_ccw`: sends `ROTATE 1 <inc> CCW`.  
     - `btn_m1_zero`: sends `ZERO 1`.  
     - `btn_m1_home`:  
       1. Sends `GET_POS 1`, reads `<current_deg>`.  
       2. Sends `ROTATE 1 <current_deg> CCW` to return to zero.  

3. **Motor 2 Frame**  
   - `self.m2_inc_combo`: `ttk.Combobox` for mm increments (`["0.01","0.1","1","5","10"]`).  
   - Buttons (single‐move):  
     - `btn_m2_fwd`: sends `MOVE 2 <inc> FORWARD`.  
     - `btn_m2_back`: sends `MOVE 2 <inc> BACKWARD`.  
   - Buttons (jog continuous on hold):  
     - `btn_m2_jog_fwd`: on `<ButtonPress>`, start a thread `_jog_forward_loop()` that repeatedly sends small moves (`MOVE 2 0.1 FORWARD`) every `JOG_STEP_DELAY` seconds. On `<ButtonRelease>`, set a flag to stop.  
     - `btn_m2_jog_back`: analogous for backward.  
   - `btn_m2_zero`: sends `ZERO 2`.  
   - `btn_m2_home`:  
     1. Sends `GET_POS 2`, reads `<current_mm>`.  
     2. Sends `MOVE 2 <current_mm> BACKWARD` to return to 0.  

4. **Capture Routine Builder Frame**  
   - `self.cr_name_entry`: `tk.Entry` for routine name.  
   - `self.cr_sweep_spin`: `ttk.Spinbox(from_=1, to=360, increment=1)`, default 360 (° sweep).  
   - `self.cr_stacks_spin`: `ttk.Spinbox(from_=1, to=360, increment=1)`, default 36 (stacks).  
   - `self.cr_shots_spin`: `ttk.Spinbox(from_=1, to=100, increment=1)`, default 10 (shots/stack).  
   - `self.cr_near_entry`: `tk.Entry` showing “Near (mm).” Default 0.0.  
     - `self.btn_cr_set_near`: sends `GET_POS 2`, stores returned mm in `self.cr_near_var`.  
   - `self.cr_far_entry`: `tk.Entry` showing “Far (mm).” Default 10.0.  
     - `self.btn_cr_set_far`: simply reads the `self.cr_far_var` (interpreted either as absolute position or offset-from-near, depending on your choice).  
   - Run/Pause/Clear Buttons:  
     - `btn_cr_run`: stub for `run_capture_routine()` – should spawn a background thread that:  
       1. Reads sweep/stacks/shots/near/far.  
       2. Calculates `angle_step = sweep / stacks`.  
       3. For each stack:  
          - `ROTATE 1 angle_step CW`  
          - For each shot in that stack:  
            1. `MOVE 2 <(far – near)/(shots–1)> FORWARD` repeatedly (or in desired increments) to go from near→far.  
            2. Trigger camera capture (e.g. via GPIO or direct `Picamera2.capture_file(...)`).  
            3. `MOVE 2 <(far – near)/(shots–1)> BACKWARD` to return to near.  
       4. Loop until sweep is complete.  
     - `btn_cr_pause`: sets a threading `Event` or flag to pause that loop.  
     - `btn_cr_clear`: cancels the thread and resets any counters.  

5. **Status Bar**  
   - `self.status_var`: updated on every action (“Motor 1: ROTATE +5° CW”, “Motor 2: Zeroed”, “Capture: Near set to 12.345 mm”, etc.).

### 4.3 Important Code Snippets

```python
import tkinter as tk
from tkinter import ttk, messagebox
import threading, time, serial
try:
    from picamera2 import Picamera2
    from PIL import Image, ImageTk
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False

SERIAL_PORT     = "/dev/ttyACM0"
SERIAL_BAUDRATE = 115200
CAMERA_RESOLUTION = (640,480)
JOG_STEP_DELAY  = 0.05  # 50 ms between each jog step

class ScannerGUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("3D Scanner Control Panel - v3.0")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        # Open serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUDRATE, timeout=0.1)
        except serial.SerialException as e:
            messagebox.showerror("Serial Error", f"Could not open port:\n{e}")
            self.destroy()
            return

        self.camera       = None
        self.preview_on   = False
        self.preview_image = None
        self.jog_forward_active  = False
        self.jog_backward_active = False
        self.motor1_position_deg = 0.0
        self.motor2_position_mm  = 0.0

        self._build_gui()
        self._layout_gui()

    def _build_gui(self):
        # CAMERA FRAME
        self.camera_frame = ttk.LabelFrame(self, text="Camera", padding=10)
        self.preview_label = tk.Label(self.camera_frame, bg="black", width=640, height=480)
        self.btn_preview   = ttk.Button(self.camera_frame, text="Start Preview", command=self.toggle_preview)

        # MOTOR 1 FRAME (TURNABLE)
        self.m1_frame      = ttk.LabelFrame(self, text="Motor 1 – Turntable", padding=10)
        self.m1_inc_label  = ttk.Label(self.m1_frame, text="Step (°):")
        self.m1_inc_var    = tk.StringVar(value="5")
        self.m1_inc_combo  = ttk.Combobox(self.m1_frame, textvariable=self.m1_inc_var,
                                          values=["1","5","15","30","60","90","180","360"],
                                          state="readonly", width=5)
        self.btn_m1_cw     = ttk.Button(self.m1_frame, text="Move CW",  command=self.move_m1_cw)
        self.btn_m1_ccw    = ttk.Button(self.m1_frame, text="Move CCW", command=self.move_m1_ccw)
        self.btn_m1_zero   = ttk.Button(self.m1_frame, text="Zero",     command=self.zero_m1)
        self.btn_m1_home   = ttk.Button(self.m1_frame, text="Home",     command=self.home_m1)

        # MOTOR 2 FRAME (LINEAR CARRIAGE)
        self.m2_frame        = ttk.LabelFrame(self, text="Motor 2 – Linear Carriage", padding=10)
        self.m2_inc_label    = ttk.Label(self.m2_frame, text="Step (mm):")
        self.m2_inc_var      = tk.StringVar(value="1.0")
        self.m2_inc_combo    = ttk.Combobox(self.m2_frame, textvariable=self.m2_inc_var,
                                            values=["0.01","0.1","1","5","10"], state="readonly", width=5)
        self.btn_m2_fwd      = ttk.Button(self.m2_frame, text="Move →", command=self.move_m2_fwd)
        self.btn_m2_back     = ttk.Button(self.m2_frame, text="Move ←", command=self.move_m2_back)
        self.btn_m2_jog_fwd  = ttk.Button(self.m2_frame, text="Jog →")
        self.btn_m2_jog_back = ttk.Button(self.m2_frame, text="Jog ←")
        self.btn_m2_jog_fwd.bind("<ButtonPress-1>",   self.start_jog_forward)
        self.btn_m2_jog_fwd.bind("<ButtonRelease-1>", self.stop_jog)
        self.btn_m2_jog_back.bind("<ButtonPress-1>",   self.start_jog_backward)
        self.btn_m2_jog_back.bind("<ButtonRelease-1>", self.stop_jog)
        self.btn_m2_zero     = ttk.Button(self.m2_frame, text="Zero", command=self.zero_m2)
        self.btn_m2_home     = ttk.Button(self.m2_frame, text="Home", command=self.home_m2)

        # CAPTURE ROUTINE BUILDER FRAME
        self.cr_frame         = ttk.LabelFrame(self, text="Scanning Capture Routine Builder", padding=10)
        self.cr_name_label    = ttk.Label(self.cr_frame, text="Name:")
        self.cr_name_var      = tk.StringVar(value="Routine 1")
        self.cr_name_entry    = ttk.Entry(self.cr_frame, textvariable=self.cr_name_var, width=20)
        self.cr_sweep_label   = ttk.Label(self.cr_frame, text="Sweep (°):")
        self.cr_sweep_var     = tk.IntVar(value=360)
        self.cr_sweep_spin    = ttk.Spinbox(self.cr_frame, from_=1, to=360, increment=1,
                                            textvariable=self.cr_sweep_var, width=5)
        self.cr_stacks_label  = ttk.Label(self.cr_frame, text="#Stacks:")
        self.cr_stacks_var    = tk.IntVar(value=36)
        self.cr_stacks_spin   = ttk.Spinbox(self.cr_frame, from_=1, to=360, increment=1,
                                            textvariable=self.cr_stacks_var, width=5)
        self.cr_shots_label   = ttk.Label(self.cr_frame, text="Shots/Stack:")
        self.cr_shots_var     = tk.IntVar(value=10)
        self.cr_shots_spin    = ttk.Spinbox(self.cr_frame, from_=1, to=100, increment=1,
                                            textvariable=self.cr_shots_var, width=5)

        self.cr_near_label    = ttk.Label(self.cr_frame, text="Near (mm):")
        self.cr_near_var      = tk.DoubleVar(value=0.0)
        self.cr_near_entry    = ttk.Entry(self.cr_frame, textvariable=self.cr_near_var, width=7)
        self.btn_cr_set_near  = ttk.Button(self.cr_frame, text="Set Near", command=self.set_near_point)

        self.cr_far_label     = ttk.Label(self.cr_frame, text="Far (mm):")
        self.cr_far_var       = tk.DoubleVar(value=10.0)
        self.cr_far_entry     = ttk.Entry(self.cr_frame, textvariable=self.cr_far_var, width=7)
        self.btn_cr_set_far   = ttk.Button(self.cr_frame, text="Set Far", command=self.set_far_point)

        self.btn_cr_run       = ttk.Button(self.cr_frame, text="Run Capture",  command=self.run_capture_routine)
        self.btn_cr_pause     = ttk.Button(self.cr_frame, text="Pause Capture",command=self.pause_capture_routine)
        self.btn_cr_clear     = ttk.Button(self.cr_frame, text="Clear Capture",command=self.clear_capture_routine)

        # Status Bar
        self.status_var       = tk.StringVar(value="Idle")
        self.status_bar       = ttk.Label(self, textvariable=self.status_var, relief="sunken", anchor="w")

    def _layout_gui(self):
        # Camera
        self.camera_frame.grid(row=0, column=0, padx=10, pady=5, sticky="nsew")
        self.preview_label.grid(row=0, column=0, columnspan=2, padx=5, pady=5)
        self.btn_preview.grid(row=1, column=0, columnspan=2, pady=(5,0))

        # Motor 1
        self.m1_frame.grid(row=1, column=0, padx=10, pady=5, sticky="nsew")
        self.m1_inc_label.grid(row=0, column=0, sticky="w")
        self.m1_inc_combo.grid(row=0, column=1, sticky="w", padx=(5,0))
        self.btn_m1_cw.grid(row=1, column=0, padx=(0,5), pady=(5,0), sticky="ew")
        self.btn_m1_ccw.grid(row=1, column=1, padx=(5,0), pady=(5,0), sticky="ew")
        self.btn_m1_zero.grid(row=2, column=0, padx=(0,5), pady=(5,0), sticky="ew")
        self.btn_m1_home.grid(row=2, column=1, padx=(5,0), pady=(5,0), sticky="ew")

        # Motor 2
        self.m2_frame.grid(row=2, column=0, padx=10, pady=5, sticky="nsew")
        self.m2_inc_label.grid(row=0, column=0, sticky="w")
        self.m2_inc_combo.grid(row=0, column=1, sticky="w", padx=(5,0))
        self.btn_m2_fwd.grid(row=1, column=0, padx=(0,5), pady=(5,0), sticky="ew")
        self.btn_m2_back.grid(row=1, column=1, padx=(5,0), pady=(5,0), sticky="ew")
        self.btn_m2_jog_fwd.grid(row=2, column=0, padx=(0,5), pady=(5,0), sticky="ew")
        self.btn_m2_jog_back.grid(row=2, column=1, padx=(5,0), pady=(5,0), sticky="ew")
        self.btn_m2_zero.grid(row=3, column=0, padx=(0,5), pady=(5,0), sticky="ew")
        self.btn_m2_home.grid(row=3, column=1, padx=(5,0), pady=(5,0), sticky="ew")

        # Capture Routine
        self.cr_frame.grid(row=3, column=0, padx=10, pady=5, sticky="nsew")
        self.cr_name_label.grid(row=0, column=0, sticky="w")
        self.cr_name_entry.grid(row=0, column=1, sticky="w", padx=(5,15))
        self.cr_sweep_label.grid(row=0, column=2, sticky="w")
        self.cr_sweep_spin.grid(row=0, column=3, sticky="w", padx=(5,15))
        self.cr_stacks_label.grid(row=1, column=0, sticky="w")
        self.cr_stacks_spin.grid(row=1, column=1, sticky="w", padx=(5,15))
        self.cr_shots_label.grid(row=1, column=2, sticky="w")
        self.cr_shots_spin.grid(row=1, column=3, sticky="w", padx=(5,15))

        self.cr_near_label.grid(row=2, column=0, sticky="w")
        self.cr_near_entry.grid(row=2, column=1, sticky="w", padx=(5,5))
        self.btn_cr_set_near.grid(row=2, column=2, padx=(10,5))
        self.cr_far_label.grid(row=2, column=3, sticky="w")
        self.cr_far_entry.grid(row=2, column=4, sticky="w", padx=(5,5))
        self.btn_cr_set_far.grid(row=2, column=5, padx=(10,5))

        self.btn_cr_run.grid(row=3, column=0, pady=(10,0), sticky="ew")
        self.btn_cr_pause.grid(row=3, column=1, pady=(10,0), sticky="ew")
        self.btn_cr_clear.grid(row=3, column=2, pady=(10,0), sticky="ew")

        self.status_bar.grid(row=4, column=0, sticky="ew", padx=10, pady=(5,0))

        self.columnconfigure(0, weight=1)
        self.camera_frame.columnconfigure(0, weight=1)
        self.m1_frame.columnconfigure(0, weight=1)
        self.m1_frame.columnconfigure(1, weight=1)
        self.m2_frame.columnconfigure(0, weight=1)
        self.m2_frame.columnconfigure(1, weight=1)
        self.cr_frame.columnconfigure(1, weight=1)

    # CAMERA FUNCTIONS (using Picamera2 + PIL)
    def toggle_preview(self):
        if not PICAMERA2_AVAILABLE:
            messagebox.showwarning("Camera Unavailable", "picamera2 or PIL not installed.")
            return
        if not self.preview_on:
            self.start_preview()
            self.btn_preview.config(text="Stop Preview")
        else:
            self.stop_preview()
            self.btn_preview.config(text="Start Preview")

    def start_preview(self):
        if self.camera is None:
            self.camera = Picamera2()
            config = self.camera.create_preview_configuration(
                main={"size": CAMERA_RESOLUTION},
                lores={"size": CAMERA_RESOLUTION},
            )
            self.camera.configure(config)
        self.camera.start()
        self.preview_on = True
        self._update_preview_frame()

    def stop_preview(self):
        if self.camera:
            self.camera.stop()
        self.preview_on = False

    def _update_preview_frame(self):
        if not self.preview_on:
            return
        frame = self.camera.capture_array("main")
        image = Image.fromarray(frame)
        image = image.resize(CAMERA_RESOLUTION, Image.NEAREST)
        photo = ImageTk.PhotoImage(image)
        self.preview_image = photo
        self.preview_label.config(image=photo)
        self.after(30, self._update_preview_frame)

    # MOTOR 1 HANDLERS
    def move_m1_cw(self):
        inc_deg = float(self.m1_inc_var.get())
        cmd = f"ROTATE 1 {inc_deg:.3f} CW\n"
        self.ser.write(cmd.encode())
        self.status_var.set(f"Motor 1: ROTATE +{inc_deg}° CW")

    def move_m1_ccw(self):
        inc_deg = float(self.m1_inc_var.get())
        cmd = f"ROTATE 1 {inc_deg:.3f} CCW\n"
        self.ser.write(cmd.encode())
        self.status_var.set(f"Motor 1: ROTATE −{inc_deg}° CCW")

    def zero_m1(self):
        self.ser.write(b"ZERO 1\n")
        self.status_var.set("Motor 1: Zeroed")

    def home_m1(self):
        # Query current pos and rotate back
        self.ser.write(b"GET_POS 1\n")
        time.sleep(0.05)
        resp = self.ser.readline().decode().strip()
        try:
            current_deg = float(resp)
            cmd = f"ROTATE 1 {current_deg:.3f} CCW\n"
            self.ser.write(cmd.encode())
            self.status_var.set("Motor 1: Returning to Home (0°)")
        except ValueError:
            self.status_var.set("Motor 1: Home failed (no pos data)")

    # MOTOR 2 HANDLERS
    def move_m2_fwd(self):
        inc_mm = float(self.m2_inc_var.get())
        cmd = f"MOVE 2 {inc_mm:.3f} FORWARD\n"
        self.ser.write(cmd.encode())
        self.status_var.set(f"Motor 2: MOVE +{inc_mm} mm")

    def move_m2_back(self):
        inc_mm = float(self.m2_inc_var.get())
        cmd = f"MOVE 2 {inc_mm:.3f} BACKWARD\n"
        self.ser.write(cmd.encode())
        self.status_var.set(f"Motor 2: MOVE −{inc_mm} mm")

    def start_jog_forward(self, event=None):
        if self.jog_forward_active:
            return
        self.jog_forward_active = True
        threading.Thread(target=self._jog_forward_loop, daemon=True).start()

    def _jog_forward_loop(self):
        jog_increment = 0.1  # mm per jog step
        while self.jog_forward_active:
            cmd = f"MOVE 2 {jog_increment:.3f} FORWARD\n"
            self.ser.write(cmd.encode())
            self.status_var.set("Motor 2: Jogging →")
            time.sleep(JOG_STEP_DELAY)
        self.status_var.set("Motor 2: Jog forward stopped")

    def start_jog_backward(self, event=None):
        if self.jog_backward_active:
            return
        self.jog_backward_active = True
        threading.Thread(target=self._jog_backward_loop, daemon=True).start()

    def _jog_backward_loop(self):
        jog_increment = 0.1  # mm per jog step
        while self.jog_backward_active:
            cmd = f"MOVE 2 {jog_increment:.3f} BACKWARD\n"
            self.ser.write(cmd.encode())
            self.status_var.set("Motor 2: Jogging ←")
            time.sleep(JOG_STEP_DELAY)
        self.status_var.set("Motor 2: Jog backward stopped")

    def stop_jog(self, event=None):
        if self.jog_forward_active:
            self.jog_forward_active = False
        if self.jog_backward_active:
            self.jog_backward_active = False

    def zero_m2(self):
        self.ser.write(b"ZERO 2\n")
        self.status_var.set("Motor 2: Zeroed")

    def home_m2(self):
        self.ser.write(b"GET_POS 2\n")
        time.sleep(0.05)
        resp = self.ser.readline().decode().strip()
        try:
            current_mm = float(resp)
            cmd = f"MOVE 2 {current_mm:.3f} BACKWARD\n"
            self.ser.write(cmd.encode())
            self.status_var.set("Motor 2: Returning to Home (0 mm)")
        except ValueError:
            self.status_var.set("Motor 2: Home failed (no pos data)")

    # CAPTURE ROUTINE BUILDER HANDLERS
    def set_near_point(self):
        self.ser.write(b"GET_POS 2\n")
        time.sleep(0.05)
        resp = self.ser.readline().decode().strip()
        try:
            val = float(resp)
            self.cr_near_var.set(val)
            self.status_var.set(f"Capture: Near set to {val:.3f} mm")
        except ValueError:
            self.status_var.set("Capture: Failed to read Motor 2 position")

    def set_far_point(self):
        try:
            near = float(self.cr_near_var.get())
            far  = float(self.cr_far_var.get())
            # If “Far” is intended as an absolute position, we keep it as is.
            # If you want “Far” to be an offset from near, do:
            # new_far = near + far
            # self.cr_far_var.set(new_far)
            self.status_var.set(f"Capture: Far set to {far:.3f} mm")
        except ValueError:
            self.status_var.set("Capture: Invalid Far value")

    def run_capture_routine(self):
        name   = self.cr_name_var.get()
        sweep  = self.cr_sweep_var.get()
        stacks = self.cr_stacks_var.get()
        shots  = self.cr_shots_var.get()
        near   = self.cr_near_var.get()
        far    = self.cr_far_var.get()
        msg = (f"Starting Capture Routine '{name}': "
               f"Sweep={sweep}°, Stacks={stacks}, "
               f"Shots/Stack={shots}, Near={near}mm, Far={far}mm.")
        self.status_var.set(msg)
        # TODO: Spawn a background thread to:
        #   angle_step = sweep / stacks
        #   for stack_index in range(stacks):
        #     self.ser.write(f"ROTATE 1 {angle_step:.3f} CW\n")
        #     for shot in range(shots):
        #       # Move Motor 2 from near→far in discrete increments,
        #       # trigger camera capture,
        #       # move Motor 2 back to near.
        #   End

    def pause_capture_routine(self):
        # TODO: set a threading Event to pause the routine
        self.status_var.set("Capture Routine: PAUSED")

    def clear_capture_routine(self):
        # TODO: cancel the thread, reset counters
        self.status_var.set("Capture Routine: CLEARED")

    # CLEAN UP ON CLOSE
    def on_close(self):
        self.stop_preview()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except:
            pass
        self.destroy()

if __name__ == "__main__":
    app = ScannerGUI()
    app.mainloop()
```

---

## 5. File Structure

```
/path/to/project/
├── README.md                  # (optional) Project overview
├── main.py                    # Python/Tkinter GUI (shown above)
├── motor_control_api.ino      # Arduino sketch (shown above)
├── camera_config.py           # (future) Custom camera settings, if separated
├── profiles.json              # (future) Saved scanning profiles
├── requirements.txt           # (optional) e.g. "picamera2\nPillow\npyserial\n"
└── assets/                    # (optional) icons, logos, etc.
```

- **`main.py`**:  
  - The entire GUI lives here: Camera preview, Motor 1 & Motor 2 controls, Capture Routine builder, status bar.
  - Uses `serial.Serial(...)` to send ASCII commands to `motor_control_api.ino` on the Arduino.

- **`motor_control_api.ino`**:  
  - Compiles under Arduino IDE or PlatformIO.  
  - Implements `ROTATE`, `MOVE`, `ZERO`, `GET_POS` with proper calibration and limit‐switch check for Motor 2.

- **`camera_config.py`** (not created yet):  
  - If you want to factor out camera resolution, framerate, or advanced capture‐settings, put them here and import into `main.py`.  
  - Example placeholder:
    ```python
    # camera_config.py
    DEFAULT_RESOLUTION = (1280, 720)
    FRAME_RATE         = 30
    STILL_CAPTURE_DIR  = "/home/pi/scans/"
    ```

- **`profiles.json`** (future):  
  - JSON file to store named scan profiles (e.g. “Botanical Sample”, “Insect Sample”).  
  - Each profile might include:  
    ```json
    {
      "profiles": [
        {
          "name": "Sample A",
          "sweep_deg": 360,
          "stacks": 36,
          "shots_per_stack": 10,
          "near_mm": 0.0,
          "far_mm": 10.0
        },
        ...
      ]
    }
    ```
  - `main.py` could read this to populate dropdowns or preset routines.

---

## 6. Serial Protocol Summary

Every command is a single ASCII line ending with `\n`. The Arduino always responds with:

- `OK`  → after a successful `ROTATE`, `MOVE`, or `ZERO`.  
- `<float>` → after `GET_POS`, printing the position with three decimal places.  
- `ERR: ...` → if the command is unknown or malformed.

### Example Exchanges

1. **Rotate 30° CW**  
   - Python: `ser.write(b"ROTATE 1 30.000 CW\n")`  
   - Arduino replies: `OK\n`  
   - (Optional) Python can send: `ser.write(b"GET_POS 1\n")`  
   - Arduino replies: e.g. `30.000\n`

2. **Move 5 mm Forward**  
   - Python: `ser.write(b"MOVE 2 5.000 FORWARD\n")`  
   - Arduino replies: `OK\n`  
   - Python: `ser.write(b"GET_POS 2\n")`  
   - Arduino might reply: `5.000\n`  

3. **Zero Motor 2**  
   - Python: `ser.write(b"ZERO 2\n")`  
   - Arduino replies: `OK\n`  

4. **Home Motor 1 (Turntable)**  
   ```python
   ser.write(b"GET_POS 1\n")
   time.sleep(0.05)
   resp = ser.readline().decode().strip()  # e.g. "45.000"
   current_deg = float(resp)
   cmd = f"ROTATE 1 {current_deg:.3f} CCW\n".encode()
   ser.write(cmd)
   # Arduino will reply “OK”
   ```

---

## 7. Calibration & Tuning Notes

- **Motor 1 (Turntable)**  
  1. Theoretical microsteps/degree: `(200 steps/rev × 16 µsteps/step × 7.5 gear ratio) / 360° = 66.667 µsteps/°`.  
  2. We chose **75.0 µsteps/°** to account for chain slop/tolerances.  
  3. In practice, we iteratively adjusted `STEPS_PER_DEGREE_M1` so that `ROTATE 1 360.000 CW` → `GET_POS 1` ≈ `360.000`.  
  4. Final constant: `const float STEPS_PER_DEGREE_M1 = 75.32f;` (if further tuning was done) or leave at `75.0f` if acceptable.

- **Motor 2 (Linear Carriage)**  
  1. Observed: `MOVE 2 10.0 BACKWARD` moved 25 mm → actual leadscrew pitch = 5 mm/rev.  
  2. Final: `const float LEAD_MM = 5.0f; const float STEPS_PER_MM_M2 = 3200.0f / 5.0f; // 640 µsteps/mm`.  
  3. Verified within ~0.01 mm by commanding small moves and comparing actual travel vs `GET_POS 2`.

- **Limit Switch (Motor 2)**  
  - Wired to Arduino Pin 7 with `INPUT_PULLUP`.  
  - If `digitalRead(7) == LOW`, carriage is at home (0 mm). The sketch prevents further “backward” steps past home.

---

## 8. Dependencies & Installation

1. **On the Raspberry Pi (64-bit OS/aarch64)**  
   ```bash
   sudo apt update
   sudo apt install -y python3-serial python3-pil python3-picamera2 \
                       wget gpg apt-transport-https ca-certificates
   ```

2. **VS Code (optional)**  
   If you want to continue editing in VS Code on the Pi:
   ```bash
   # Add Microsoft’s repo:
   wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
   sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
   sudo sh -c 'echo "deb [arch=arm64] https://packages.microsoft.com/repos/code stable main" \
               > /etc/apt/sources.list.d/vscode.list'
   sudo apt update
   sudo apt install -y code
   ```

3. **On Your Desktop (Development)**  
   - You can open this folder in VS Code (with Copilot enabled).  
   - The “hand-off” document you’re viewing right now can be copied into a file called `HANDOFF.md` or `OVERVIEW.md` at the repo root.  
   - That ensures Copilot (and other team members) instantly know all pin assignments, constants, and next steps.

---

## 9. How to Run / Test

1. **Arduino**  
   - Open `motor_control_api.ino` in Arduino IDE (or PlatformIO).  
   - Verify your board is set to “Arduino Uno”, port is correct (`/dev/ttyACM0`).  
   - Upload the sketch.  
   - Open Serial Monitor at 115 200 baud to confirm you see:  
     ```
     Ready for ROTATE, MOVE, ZERO, GET_POS
     ```
   - Test a few commands manually to verify:
     ```
     ROTATE 1 10.0 CW
     GET_POS 1
     MOVE 2 5.0 FORWARD
     GET_POS 2
     ```

2. **Python GUI**  
   - Ensure the Arduino is plugged in, so `/dev/ttyACM0` is available (or find the “by-id” symlink).  
   - In a terminal:
     ```bash
     cd /path/to/project
     python3 main.py
     ```
   - The GUI window should appear with all frames/buttons.  
   - Click **“Start Preview”** to see the HQ camera feed.  
   - In the **Turntable** frame, select “15°” → click **Move CW**. Verify the Arduino receives and moves accordingly.  
   - In the **Linear Carriage** frame, select “1.0 mm” → click **Move →**. Verify it moves 1 mm.  
   - Press and hold **Jog ←** to see continuous motion.  
   - In **Capture Routine**, click **Set Near** → the “Near (mm)” entry should update with current `GET_POS 2`.  
   - Click **Run Capture** to see the status bar change. (Full routine logic is still TODO.)

---

## 10. Next Steps & Future Roadmap

1. **Capture Routine Implementation**  
   - Flesh out `run_capture_routine()` so it spawns a dedicated thread:  
     - Loop through stacks:  
       1. Rotate turntable by `sweep_deg / stacks`.  
       2. Within each stack, loop through shots:  
          - Interpolate Motor 2 from near→far in `(shots − 1)` steps.  
          - Trigger camera capture for each position (e.g. `Picamera2.capture_file(...)` or via a GPIO to an external trigger).  
          - Move Motor 2 back to near before next stack rotation.  
   - Implement “Pause” by setting a threading `Event` flag.  
   - Implement “Clear” by canceling the thread (e.g. another `Event` or flag to break loops).

2. **Profile Saving & Loading**  
   - Create `profiles.json` (or user-configurable file) with saved routines (name, sweep, stacks, shots, near, far).  
   - In GUI, add a `ttk.Combobox` to select an existing profile or save the current form as a new profile.

3. **Camera Configuration Module**  
   - Move any advanced camera settings (resolution, framerate, HDR, exposure, ISO) into `camera_config.py`.  
   - Import that module in `main.py` so preview and still captures use consistent parameters.

4. **Error Handling & Logging**  
   - Add try/except around all `ser.read().decode()` calls to avoid GUI hangs if serial data is malformed.  
   - Add a logging system (`logging` module) to record commands sent, errors, camera capture times, etc.

5. **GUI Polish**  
   - Use `ttk.Style()` to adjust widget colors/fonts for readability.  
   - Add tooltips (using a simple tooltip class) to explain each control.  
   - Consider using a `ttk.Progressbar` while a capture routine is running (to show % complete).

6. **Deploy & Autostart**  
   - Add a `.desktop` file in `/home/pi/.config/autostart/` so that on boot, the GUI automatically launches.  
   - Or create a `systemd` service that starts `python3 /path/to/main.py` on Pi login.

---

### Contact / Troubleshooting

- If the Arduino fails to respond:
  1. Make sure no other process (Serial Monitor, `minicom`, `screen`, or ModemManager) is locking `/dev/ttyACM0`.
  2. Use `ls -l /dev/serial/by-id/` to find a stable port name and replace `SERIAL_PORT` accordingly.
  3. Verify both devices (Pi and Arduino) are using **115 200 baud**.

- If the camera preview does not display:
  1. Check that `picamera2` and `PIL` are installed (`python3 -m pip show picamera2 pillow`).  
  2. Run a minimal test:
     ```python
     from picamera2 import Picamera2
     from PIL import Image
     picam = Picamera2()
     config = picam.create_preview_configuration(main={"size": (320,240)})
     picam.configure(config)
     picam.start()
     frame = picam.capture_array("main")
     Image.fromarray(frame).show()
     picam.stop()
     ```
  3. If you get errors, your Pi’s firmware might need updating (`sudo apt update && sudo apt full-upgrade`).

---

## 11. Summary

- **Arduino Sketch (`motor_control_api.ino`)**  
  - Pin assignments:  
    - Motor 1: STEP=2, DIR=3, ENABLE=13  
    - Motor 2: STEP=4, DIR=5, ENABLE=12, LIMIT_SWITCH=7  
  - Calibration:  
    - Motor 1: `STEPS_PER_DEGREE_M1 = 75.0f` (1/16 microstep, 7.5:1 gearing).  
    - Motor 2: `STEPS_PER_MM_M2 = 640.0f` (1/16 microstep, 5 mm/rev leadscrew).  
  - Serial protocol: `ROTATE`, `MOVE`, `ZERO`, `GET_POS`.

- **Python GUI (`main.py`)**  
  - Uses Tkinter for layout, Picamera2 + PIL for preview, `pyserial` for communication.  
  - Contains all requested widgets (camera preview, motor controls, capture routine builder, status bar).

- **Next tasks**  
  - Implement the full capture-routine thread, profile loading/saving, robust error handling, and GUI polish.

With this document in Copilot (or any other environment), a developer can immediately see:

1. Exactly which Arduino pins map to which hardware signals.
2. How the serial commands are structured and interpreted.
3. Which calibration constants to tweak.
4. How the Python GUI is laid out and what each button does.
5. Where to implement the missing capture-routine logic.

Feel free to rename this file to `HANDOFF.md` or `README.md` as your single-source-of-truth.
