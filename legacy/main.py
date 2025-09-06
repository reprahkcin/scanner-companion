
import tkinter as tk
from tkinter import ttk, messagebox
import json
import os
import time
from motor_controller import MotorController
from camera_controller import CameraController

PROFILE_FILE = "profiles.json"

class ScannerControlApp:
    def __init__(self, master):
        self.master = master
        self.master.title("3D Scanner Companion")

        self.mc = MotorController('/dev/ttyACM0', 9600)
        self.camera = CameraController()
        self.steps_per_revolution = 27118
        self.steps_per_cm = 6625
        self.z_home = 0
        self.profiles = self.load_profiles()
        self.profile = {}

        self.create_widgets()

    def load_profiles(self):
        if not os.path.exists(PROFILE_FILE):
            return {}
        with open(PROFILE_FILE, "r") as f:
            return json.load(f)

    def save_profiles(self):
        with open(PROFILE_FILE, "w") as f:
            json.dump(self.profiles, f, indent=4)

    def create_widgets(self):
        row = 0
        tk.Label(self.master, text="Select Profile").grid(row=row, column=0)
        self.selected_profile = tk.StringVar()
        self.profile_dropdown = ttk.Combobox(self.master, textvariable=self.selected_profile, state="readonly")
        self.profile_dropdown['values'] = list(self.profiles.keys())
        self.profile_dropdown.grid(row=row, column=1)
        tk.Button(self.master, text="Load", command=self.apply_selected_profile).grid(row=row, column=2)
        row += 1

        tk.Button(self.master, text="Set Z-Home", command=self.set_z_home).grid(row=row, column=0)
        self.z_home_label = tk.Label(self.master, text="Z-Origin: Not Set")
        self.z_home_label.grid(row=row, column=1, columnspan=2, sticky="w")
        row += 1

        tk.Label(self.master, text="Settling Time (1–5 sec)").grid(row=row, column=0)
        self.settle_slider = tk.Scale(self.master, from_=1, to=5, resolution=1, orient=tk.HORIZONTAL)
        self.settle_slider.set(2)
        self.settle_slider.grid(row=row, column=1, columnspan=2, sticky="ew")
        row += 1

        # Camera Settings Sliders
        self.cam_settings = {}
        camera_fields = [
            ("Brightness", "brightness", 0, 100, 50),
            ("Contrast", "contrast", -100, 100, 0),
            ("ISO", "iso", 100, 800, 100),
            ("Shutter Speed (µs)", "shutter_speed", 100, 100000, 10000)
        ]
        for label, key, min_val, max_val, default in camera_fields:
            tk.Label(self.master, text=label).grid(row=row, column=0)
            scale = tk.Scale(self.master, from_=min_val, to=max_val, orient=tk.HORIZONTAL)
            scale.set(default)
            scale.grid(row=row, column=1, columnspan=2, sticky="ew")
            self.cam_settings[key] = scale
            row += 1

        # Geometry & stack input
        self.entries = {}
        for label, key in [("Geometry Type (round/oval)", "geometry_type"),
                           ("Diameter (cm)", "diameter_cm"),
                           ("Start Z (cm)", "start_z"),
                           ("End Z (cm)", "end_z"),
                           ("Preset Name", "preset_name"),
                           ("Item Name", "item_name")]:
            tk.Label(self.master, text=label).grid(row=row, column=0)
            entry = tk.Entry(self.master)
            entry.grid(row=row, column=1, columnspan=2, sticky="ew")
            self.entries[key] = entry
            row += 1

        tk.Label(self.master, text="Stacks (1–72)").grid(row=row, column=0)
        self.stacks = tk.Scale(self.master, from_=1, to=72, orient=tk.HORIZONTAL)
        self.stacks.set(36)
        self.stacks.grid(row=row, column=1, columnspan=2, sticky="ew")
        row += 1

        tk.Label(self.master, text="Shots per Stack (1–100)").grid(row=row, column=0)
        self.shots_per_stack = tk.Scale(self.master, from_=1, to=100, orient=tk.HORIZONTAL)
        self.shots_per_stack.set(25)
        self.shots_per_stack.grid(row=row, column=1, columnspan=2, sticky="ew")
        row += 1

        tk.Button(self.master, text="Save New Profile", command=self.save_new_profile).grid(row=row, column=0, columnspan=3)
        row += 1

        tk.Button(self.master, text="Run Stack", command=self.run_stack_capture).grid(row=row, column=0, columnspan=3)
        row += 1

        tk.Button(self.master, text="Start Preview", command=self.camera.start_preview).grid(row=row, column=0)
        tk.Button(self.master, text="Stop Preview", command=self.camera.stop_preview).grid(row=row, column=1)
        row += 1

        tk.Button(self.master, text="Home Camera", command=self.home_camera).grid(row=13, column=0, columnspan=3)

        row += 1
        tk.Label(self.master, text="Manual Jog (0.1 cm)").grid(row=row, column=0, columnspan=3)
        row += 1
        tk.Button(self.master, text="← Backward", command=lambda: self.jog_camera('B')).grid(row=row, column=0)
        tk.Button(self.master, text="Forward →", command=lambda: self.jog_camera('F')).grid(row=row, column=1)
        tk.Button(self.master, text="Update Position", command=self.update_position).grid(row=row, column=2)
        row += 1


        self.profile_label = tk.Label(self.master, text="No profile loaded")
        self.profile_label.grid(row=row, column=0, columnspan=3)
        
    def jog_camera(self, direction):
        distance_cm = 0.1
        steps = int(distance_cm * self.steps_per_cm)
        print(f"Jogging {direction} {distance_cm} cm ({steps} steps)")
        self.mc.move_motor('2', steps, direction)
        self.update_position()

    def update_position(self):
        try:
            pos_steps = self.mc.get_position()
            pos_cm = pos_steps / self.steps_per_cm
            print(f"Camera Position: {pos_cm:.2f} cm")
        except Exception as e:
            print("Error reading position:", e)


    def set_z_home(self):
        self.z_home = 0
        self.z_home_label.config(text="Z-Origin Set")

    def home_camera(self):
        self.mc.home()
        self.update_position()


    def apply_selected_profile(self):
        name = self.selected_profile.get()
        if name in self.profiles:
            self.profile = self.profiles[name]
            self.profile_label.config(text=f"{name}: {self.profile.get('stacks', '?')} stacks, {self.profile.get('shots_per_stack', '?')} shots/stack")
            print(json.dumps(self.profile, indent=2))

    def save_new_profile(self):
        try:
            name = self.entries["preset_name"].get()
            profile = {
                "preset_name": name,
                "item_name": self.entries["item_name"].get(),
                "stacks": self.stacks.get(),
                "shots_per_stack": self.shots_per_stack.get(),
                "camera_settings": {
                    "brightness": self.cam_settings["brightness"].get(),
                    "contrast": self.cam_settings["contrast"].get(),
                    "iso": self.cam_settings["iso"].get(),
                    "shutter_speed": self.cam_settings["shutter_speed"].get()
                },
                "geometry": {
                    "type": self.entries["geometry_type"].get(),
                    "diameter_cm": float(self.entries["diameter_cm"].get()),
                    "start_z": float(self.entries["start_z"].get()),
                    "end_z": float(self.entries["end_z"].get())
                }
            }
            self.profiles[name] = profile
            self.save_profiles()
            self.profile_dropdown['values'] = list(self.profiles.keys())
            messagebox.showinfo("Success", f"Profile '{name}' saved.")
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save profile: {e}")

    def run_stack_capture(self):
        if not self.profile or self.z_home is None:
            messagebox.showwarning("Missing Data", "Set Z-Home and load a profile first.")
            return

        shots = self.profile["shots_per_stack"]
        z_start = self.profile["geometry"]["start_z"]
        z_end = self.profile["geometry"]["end_z"]
        step_distance = (z_end - z_start) / (shots - 1)
        settle_time = self.settle_slider.get()

        for i in range(shots):
            z_pos = z_start + i * step_distance
            steps = int(z_pos * self.steps_per_cm)
            print(f"[Stack {i+1}/{shots}] Moving to Z: {z_pos:.2f} cm ({steps} steps)")
            self.mc.move_motor('2', steps, 'F')
            print(f"Settling for {settle_time} sec...")
            time.sleep(settle_time)
            print("Capturing frame (stub)")

    def close(self):
        self.camera.stop_preview()
        self.mc.close()

def main():
    root = tk.Tk()
    app = ScannerControlApp(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close(), root.destroy()))
    root.mainloop()

if __name__ == "__main__":
    main()
