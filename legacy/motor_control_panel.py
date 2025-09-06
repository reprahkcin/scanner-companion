import tkinter as tk
from motor_controller import MotorController

class MotorControlPanel:
    def __init__(self, root):
        self.root = root
        self.root.title("Motor Control Panel")
        self.mc = MotorController('/dev/ttyACM0')  # Adjust port as needed

        self.steps_var = tk.IntVar(value=100)

        tk.Label(root, text="Step Count:").grid(row=0, column=0)
        tk.Entry(root, textvariable=self.steps_var, width=10).grid(row=0, column=1)

        row = 1
        tk.Label(root, text="Motor 1 (Turntable)").grid(row=row, column=0, columnspan=2); row += 1
        tk.Button(root, text="CW", width=10, command=lambda: self.move_motor('1', 'F')).grid(row=row, column=0)
        tk.Button(root, text="CCW", width=10, command=lambda: self.move_motor('1', 'B')).grid(row=row, column=1); row += 1

        tk.Label(root, text="Motor 2 (Lead Screw)").grid(row=row, column=0, columnspan=2); row += 1
        tk.Button(root, text="Forward", width=10, command=lambda: self.move_motor('2', 'F')).grid(row=row, column=0)
        tk.Button(root, text="Reverse", width=10, command=lambda: self.move_motor('2', 'B')).grid(row=row, column=1); row += 1

        tk.Button(root, text="Home Motor 2", command=self.home_motor).grid(row=row, column=0, columnspan=2); row += 1

        tk.Button(root, text="Zero Position", command=self.zero_motor).grid(row=row, column=0, columnspan=2); row += 1

        tk.Button(root, text="Update Position", command=self.update_position).grid(row=row, column=0, columnspan=2); row += 1

        self.position_label = tk.Label(root, text="Position: ?")
        self.position_label.grid(row=row, column=0, columnspan=2)

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    def move_motor(self, motor, direction):
        steps = self.steps_var.get()
        print(f"Moving motor {motor} {direction} {steps} steps")
        self.mc.move_motor(motor, steps, direction)
        self.update_position()

    def home_motor(self):
        self.mc.home()
        self.update_position()

    def zero_motor(self):
        self.mc.zero_position()
        self.update_position()

    def update_position(self):
        pos = self.mc.get_position()
        if pos is not None:
            self.position_label.config(text=f"Position: {pos}")

    def on_close(self):
        self.mc.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControlPanel(root)
    root.mainloop()
