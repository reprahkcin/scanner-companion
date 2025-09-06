import serial
import tkinter as tk
from tkinter import ttk

# === open serial once, at module scope ===
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)

def set_motor_speed(motor_id, speed):
    cmd = f"SPEED {motor_id} {int(speed)}\n"
    ser.write(cmd.encode())
    # optional: print(cmd.strip())

def jog_motor(motor_id, direction, speed):
    cmd = f"JOG {motor_id} {direction.upper()} {int(speed)}\n"
    ser.write(cmd.encode())
    # optional: print(cmd.strip())


class MotorControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("3D Scanner Motor Control Panel")
        self.configure(padx=10, pady=10)

        # Motor 1 controls
        m1_frame = ttk.LabelFrame(self, text="Motor 1", padding=10)
        m1_frame.grid(row=0, column=0, sticky="nsew", padx=5, pady=5)
        self.m1_speed = tk.DoubleVar(value=50)
        ttk.Label(m1_frame, text="Speed:").grid(row=0, column=0, sticky="w")
        ttk.Scale(m1_frame, from_=1, to=100, variable=self.m1_speed,
                  orient="horizontal", command=self._on_m1_speed).grid(row=0, column=1, sticky="ew")
        ttk.Button(m1_frame, text="Jog ←", 
                   command=lambda: jog_motor(1, "backward", self.m1_speed.get())).grid(row=1, column=0, pady=5, sticky="ew")
        ttk.Button(m1_frame, text="Jog →", 
                   command=lambda: jog_motor(1, "forward", self.m1_speed.get())).grid(row=1, column=1, pady=5, sticky="ew")

        # Motor 2 controls
        m2_frame = ttk.LabelFrame(self, text="Motor 2", padding=10)
        m2_frame.grid(row=1, column=0, sticky="nsew", padx=5, pady=5)
        self.m2_speed = tk.DoubleVar(value=50)
        ttk.Label(m2_frame, text="Speed:").grid(row=0, column=0, sticky="w")
        ttk.Scale(m2_frame, from_=1, to=100, variable=self.m2_speed,
                  orient="horizontal", command=self._on_m2_speed).grid(row=0, column=1, sticky="ew")
        ttk.Button(m2_frame, text="Jog ←", 
                   command=lambda: jog_motor(2, "backward", self.m2_speed.get())).grid(row=1, column=0, pady=5, sticky="ew")
        ttk.Button(m2_frame, text="Jog →", 
                   command=lambda: jog_motor(2, "forward", self.m2_speed.get())).grid(row=1, column=1, pady=5, sticky="ew")

        # Layout configuration
        self.columnconfigure(0, weight=1)
        m1_frame.columnconfigure(1, weight=1)
        m2_frame.columnconfigure(1, weight=1)

    def _on_m1_speed(self, val):
        set_motor_speed(1, float(val))

    def _on_m2_speed(self, val):
        set_motor_speed(2, float(val))

if __name__ == "__main__":
    app = MotorControlPanel()
    app.mainloop()
