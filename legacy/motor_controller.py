import serial
import time

class MotorController:
    def __init__(self, port='/dev/ttyUSB0', baudrate=9600):
        self.ser = serial.Serial(port, baudrate)
        time.sleep(2)  # Allow Arduino time to initialize
        self.flush_initial()

    def flush_initial(self):
        """Flush any initial boot messages."""
        time.sleep(0.5)
        while self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            print(f"[INIT] {line}")

    def move_motor(self, motor, steps, direction):
        command = f"{motor}{steps}{direction}\n"
        self.ser.write(command.encode())

    def home(self):
        self.ser.write(b"HOME\n")
        ack = self.ser.readline().decode().strip()
        if ack == "HOMED":
            print("Successfully homed.")
        else:
            print("Homing failed or incomplete:", ack)

    def get_position(self):
        self.ser.write(b"GET_POS\n")
        response = self.ser.readline().decode().strip()
        try:
            return int(response)
        except ValueError:
            print("Invalid response for position:", response)
            return None

    def zero_position(self):
        self.ser.write(b"ZERO\n")
        ack = self.ser.readline().decode().strip()
        if ack == "ZEROED":
            print("Position zeroed.")
        else:
            print("Failed to zero position:", ack)

    def close(self):
        self.ser.close()

