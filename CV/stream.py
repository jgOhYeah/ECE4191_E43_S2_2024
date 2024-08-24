from flask import Flask, Response
import cv2
import imutils
import threading

# Your object detection model import and setup here

app = Flask(__name__)

# Global variables
outputFrame = None
lock = threading.Lock()

def detect_objects():
    global outputFrame, lock
    
    # Initialize your camera
    vs = cv2.VideoCapture(0)
    
    while True:
        # Read frame from camera
        ret, frame = vs.read()
        if not ret:
            continue
        
        # Resize frame for faster processing
        frame = imutils.resize(frame, width=400)
        
        # Apply your object detection model here
        # Draw bounding boxes and center points
        
        # Acquire the lock, set the output frame, and release the lock
        with lock:
            outputFrame = frame.copy()

def generate():
    global outputFrame, lock
    
    while True:
        with lock:
            if outputFrame is None:
                continue
            
            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)
            if not flag:
                continue
        
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
            bytearray(encodedImage) + b'\r\n')

@app.route("/")
def video_feed():
    return Response(generate(),
        mimetype = "multipart/x-mixed-replace; boundary=frame")

if __name__ == '__main__':
    t = threading.Thread(target=detect_objects, args=())
    t.daemon = True
    t.start()
    
    app.run(host="0.0.0.0", port=8000, debug=True,
		threaded=True, use_reloader=False)