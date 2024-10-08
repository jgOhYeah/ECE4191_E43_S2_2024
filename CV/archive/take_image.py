import os
import time
from picamera2 import Picamera2
from libcamera import controls

# Set up the camera
picam2 = Picamera2()
config = picam2.create_still_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.set_controls({"FrameRate": 30})

# Set rotation for upside-down mount
picam2.set_controls({"RotateImageBy": 180})

# Start the camera
picam2.start()

# Allow the camera to warm up
time.sleep(2)

# Set up the directory for saving images
save_directory = "training_images"
if not os.path.exists(save_directory):
    os.makedirs(save_directory)

# Function to get the next available file name
def get_next_filename():
    index = 1
    while True:
        filename = f"image_{index:04d}.jpg"
        if not os.path.exists(os.path.join(save_directory, filename)):
            return filename
        index += 1

# Main capture loop
try:
    print("Starting image capture. Press Ctrl+C to stop.")
    while True:
        # Get the next available filename
        filename = get_next_filename()
        
        # Capture the image
        picam2.capture_file(os.path.join(save_directory, filename))
        
        # Print status
        print(f"Captured {filename}")
        
        # Wait for 5 seconds
        time.sleep(5)

except KeyboardInterrupt:
    print("\nImage capture stopped by user.")

finally:
    # Clean up
    picam2.stop()
    print("Camera closed.")