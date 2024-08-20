import cv2
import numpy as np
from fbdev import fb_display

# Open the camera
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Cannot open camera")
    exit()

# Get the framebuffer device
device = fb_display.DisplayDevice()

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    
    if not ret:
        print("Can't receive frame. Exiting ...")
        break
    
    # Resize frame to match the framebuffer size
    frame = cv2.resize(frame, (device.xres, device.yres))
    
    # Convert the image from BGR to RGB
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Display the frame on the framebuffer
    device.display(frame)

    # Press 'q' to exit
    if cv2.waitKey(1) == ord('q'):
        break

# When everything done, release the capture
cap.release()