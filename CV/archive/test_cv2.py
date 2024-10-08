from picamera2 import Picamera2
import cv2
import time

print("Initializing camera...")
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}))
picam2.start()
time.sleep(2)

try:
    for i in range(10):
        print(f"Processing frame {i+1}...")
        frame = picam2.capture_array()
        # Perform some OpenCV operations
        cv2.putText(frame, 'Test', (50, 50), cv2.FONT_HERSHEY_SIMPLEX,
                    1, (255, 0, 0), 2, cv2.LINE_AA)
        cv2.rectangle(frame, (100, 100), (200, 200), (0, 255, 0), 3)
        # Optionally, display the frame (if you have GUI support)
        # cv2.imshow('Frame', frame)
        # cv2.waitKey(1)
        time.sleep(0.1)
except Exception as e:
    print(f"Error during OpenCV processing: {e}")
finally:
    picam2.close()
    cv2.destroyAllWindows()
    print("Camera and OpenCV closed.")
