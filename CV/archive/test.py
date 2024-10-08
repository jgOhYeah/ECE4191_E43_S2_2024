from picamera2 import Picamera2
import time

print("Initializing camera...")
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}))
picam2.start()
time.sleep(2)

try:
    for i in range(100):  # Capture 100 frames
        print(f"Capturing frame {i+1}...")
        frame = picam2.capture_array()
        print(f"Captured frame {i+1} of shape: {frame.shape}")
        time.sleep(0.1)  # Slight delay between frames
except Exception as e:
    print(f"Error capturing frames: {e}")
finally:
    picam2.close()
    print("Camera closed.")
