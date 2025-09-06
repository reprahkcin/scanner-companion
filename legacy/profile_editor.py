
import tkinter as tk
from tkinter import filedialog, messagebox
import json

class ProfileEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("Scan Profile Editor")
        self.profile = {}

        self.create_widgets()

    def create_widgets(self):
        row = 0

        # Preset name
        tk.Label(self.root, text="Preset Name").grid(row=row, column=0)
        self.preset_name = tk.Entry(self.root, width=30)
        self.preset_name.grid(row=row, column=1); row += 1

        # Item name
        tk.Label(self.root, text="Item Name").grid(row=row, column=0)
        self.item_name = tk.Entry(self.root, width=30)
        self.item_name.grid(row=row, column=1); row += 1

        # Number of stacks
        tk.Label(self.root, text="Stacks (in 360°)").grid(row=row, column=0)
        self.stacks = tk.Entry(self.root, width=10)
        self.stacks.insert(0, "72")
        self.stacks.grid(row=row, column=1); row += 1

        # Shots per stack
        tk.Label(self.root, text="Shots per Stack").grid(row=row, column=0)
        self.shots_per_stack = tk.Entry(self.root, width=10)
        self.shots_per_stack.insert(0, "25")
        self.shots_per_stack.grid(row=row, column=1); row += 1

        # Camera settings
        tk.Label(self.root, text="Brightness").grid(row=row, column=0)
        self.brightness = tk.Entry(self.root, width=10)
        self.brightness.insert(0, "50")
        self.brightness.grid(row=row, column=1); row += 1

        tk.Label(self.root, text="Contrast").grid(row=row, column=0)
        self.contrast = tk.Entry(self.root, width=10)
        self.contrast.insert(0, "0")
        self.contrast.grid(row=row, column=1); row += 1

        tk.Label(self.root, text="ISO").grid(row=row, column=0)
        self.iso = tk.Entry(self.root, width=10)
        self.iso.insert(0, "100")
        self.iso.grid(row=row, column=1); row += 1

        tk.Label(self.root, text="Shutter Speed (μs)").grid(row=row, column=0)
        self.shutter_speed = tk.Entry(self.root, width=10)
        self.shutter_speed.insert(0, "10000")
        self.shutter_speed.grid(row=row, column=1); row += 1

        # Geometry
        tk.Label(self.root, text="Object Type").grid(row=row, column=0)
        self.geometry_type = tk.StringVar(value="round")
        tk.OptionMenu(self.root, self.geometry_type, "round", "oval").grid(row=row, column=1); row += 1

        tk.Label(self.root, text="Diameter (cm)").grid(row=row, column=0)
        self.diameter_cm = tk.Entry(self.root, width=10)
        self.diameter_cm.insert(0, "10")
        self.diameter_cm.grid(row=row, column=1); row += 1

        tk.Label(self.root, text="Height (cm)").grid(row=row, column=0)
        self.height_cm = tk.Entry(self.root, width=10)
        self.height_cm.insert(0, "18")
        self.height_cm.grid(row=row, column=1); row += 1

        tk.Label(self.root, text="Start Z (cm)").grid(row=row, column=0)
        self.start_z = tk.Entry(self.root, width=10)
        self.start_z.insert(0, "0.5")
        self.start_z.grid(row=row, column=1); row += 1

        tk.Label(self.root, text="End Z (cm)").grid(row=row, column=0)
        self.end_z = tk.Entry(self.root, width=10)
        self.end_z.insert(0, "3.4")
        self.end_z.grid(row=row, column=1); row += 1

        # Save/Load Buttons
        tk.Button(self.root, text="Save Profile", command=self.save_profile).grid(row=row, column=0)
        tk.Button(self.root, text="Load Profile", command=self.load_profile).grid(row=row, column=1)

    def save_profile(self):
        data = {
            "preset_name": self.preset_name.get(),
            "item_name": self.item_name.get(),
            "stacks": int(self.stacks.get()),
            "shots_per_stack": int(self.shots_per_stack.get()),
            "camera_settings": {
                "brightness": int(self.brightness.get()),
                "contrast": int(self.contrast.get()),
                "iso": int(self.iso.get()),
                "shutter_speed": int(self.shutter_speed.get())
            },
            "geometry": {
                "type": self.geometry_type.get(),
                "diameter_cm": float(self.diameter_cm.get()),
                "height_cm": float(self.height_cm.get()),
                "start_z": float(self.start_z.get()),
                "end_z": float(self.end_z.get())
            }
        }
        file_path = filedialog.asksaveasfilename(defaultextension=".json", filetypes=[("JSON", "*.json")])
        if file_path:
            with open(file_path, 'w') as f:
                json.dump(data, f, indent=4)
            messagebox.showinfo("Saved", "Profile saved successfully.")

    def load_profile(self):
        file_path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if file_path:
            with open(file_path, 'r') as f:
                data = json.load(f)
            self.populate_fields(data)
            messagebox.showinfo("Loaded", "Profile loaded successfully.")

    def populate_fields(self, data):
        self.preset_name.delete(0, tk.END)
        self.preset_name.insert(0, data["preset_name"])
        self.item_name.delete(0, tk.END)
        self.item_name.insert(0, data["item_name"])
        self.stacks.delete(0, tk.END)
        self.stacks.insert(0, data["stacks"])
        self.shots_per_stack.delete(0, tk.END)
        self.shots_per_stack.insert(0, data["shots_per_stack"])

        cam = data["camera_settings"]
        self.brightness.delete(0, tk.END)
        self.brightness.insert(0, cam["brightness"])
        self.contrast.delete(0, tk.END)
        self.contrast.insert(0, cam["contrast"])
        self.iso.delete(0, tk.END)
        self.iso.insert(0, cam["iso"])
        self.shutter_speed.delete(0, tk.END)
        self.shutter_speed.insert(0, cam["shutter_speed"])

        geo = data["geometry"]
        self.geometry_type.set(geo["type"])
        self.diameter_cm.delete(0, tk.END)
        self.diameter_cm.insert(0, geo["diameter_cm"])
        self.height_cm.delete(0, tk.END)
        self.height_cm.insert(0, geo["height_cm"])
        self.start_z.delete(0, tk.END)
        self.start_z.insert(0, geo["start_z"])
        self.end_z.delete(0, tk.END)
        self.end_z.insert(0, geo["end_z"])

if __name__ == "__main__":
    root = tk.Tk()
    app = ProfileEditor(root)
    root.mainloop()
