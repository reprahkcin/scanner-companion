import sys
print("Python executable:", sys.executable)
print("Python version:", sys.version)

try:
    from picamera2 import Picamera2
    print("picamera2 is installed and importable!")
except ImportError as e:
    print("picamera2 is NOT installed or not importable.")
    print("ImportError:", e)

try:
    from PIL import Image
    print("PIL (Pillow) is installed and importable!")
except ImportError as e:
    print("PIL (Pillow) is NOT installed or not importable.")
    print("ImportError:", e)
