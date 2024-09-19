import cv2
import time

def test_camera():
    print("Attempting to access camera...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Failed to open camera")
        return

    print("Camera opened successfully")
    
    for i in range(10):  # Try to capture 10 frames
        ret, frame = cap.read()
        if ret:
            print(f"Successfully captured frame {i+1}")
            cv2.imshow('Frame', frame)
            cv2.waitKey(1)
        else:
            print(f"Failed to capture frame {i+1}")
        time.sleep(1)

    cap.release()
    cv2.destroyAllWindows()
    print("Camera test complete")

if __name__ == "__main__":
    test_camera()