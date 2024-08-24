import cv2
import numpy as np
import tflite_runtime.interpreter as tflite
from flask import Flask, Response
import threading
import time

# Initialize Flask app
app = Flask(__name__)

# Global variables
outputFrame = None
lock = threading.Lock()

# Load the TFLite model
interpreter = tflite.Interpreter(model_path="saved_models/custom_model_lite/detect.tflite")
interpreter.allocate_tensors()

# Get input and output details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
input_shape = input_details[0]['shape']

# Load labels
with open("/home/tennis/ECE4191_G43_S2_2024/CV/saved_models/custom_model_lite/labelmap.txt", 'r') as f:
    labels = [line.strip() for line in f.readlines()]
    print(labels)

# VideoStream class
class VideoStream:
    def __init__(self, resolution=(640, 480), framerate=30):
        self.stream = cv2.VideoCapture(0)
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.stream.set(3, resolution[0])
        self.stream.set(4, resolution[1])
        self.stopped = False
    
    def start(self):
        threading.Thread(target=self.update, args=()).start()
        return self
    
    def update(self):
        while True:
            if self.stopped:
                self.stream.release()
                return
            (self.grabbed, self.frame) = self.stream.read()
    
    def read(self):
        return self.frame
    
    def stop(self):
        self.stopped = True

def detect_objects():
    global outputFrame, lock
    
    # Initialize video stream
    vs = VideoStream(resolution=(640, 480), framerate=30).start()
    time.sleep(1.0)
    
    # Variables for FPS calculation
    fps = 0
    frame_count = 0
    start_time = time.time()
    
    while True:
        frame = vs.read()
        
        # Prepare the frame for the model
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (input_shape[1], input_shape[2]))
        input_data = np.expand_dims(frame_resized, axis=0)
        
        # Normalize pixel values if using a floating model
        if input_details[0]['dtype'] == np.float32:
            input_data = (np.float32(input_data) - 127.5) / 127.5
        
        # Perform detection
        interpreter.set_tensor(input_details[0]['index'], input_data)
        interpreter.invoke()
        
        # Retrieve detection results
        boxes = interpreter.get_tensor(output_details[0]['index'])[0]
        classes = interpreter.get_tensor(output_details[1]['index'])[0]
        scores = interpreter.get_tensor(output_details[2]['index'])[0]
        
        print("Boxes:", boxes)
        print("Classes:", classes)
        print("Scores:", scores)
        
        # Loop over all detections
        for i in range(len(boxes)):
            score = boxes[i]
            class_info = classes[i]
            
            if score > 0.5:  # Adjust this threshold as needed
                # Determine class
                class_id = 0 if class_info[0] < 0.5 else 1  # Assuming binary classification
                
                # Get bounding box coordinates
                ymin, xmin, ymax, xmax = class_info
                
                # Convert normalized coordinates to pixel coordinates
                imH, imW = frame.shape[:2]
                xmin = int(max(1, xmin * imW))
                xmax = int(min(imW, xmax * imW))
                ymin = int(max(1, ymin * imH))
                ymax = int(min(imH, ymax * imH))
                
                # Draw bounding box
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 2)
                
                # Add label
                label = f'{labels[class_id]}: {score:.2f}'
                cv2.putText(frame, label, (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (10, 255, 0), 2)
                
                print(f"Detection: {label}, Class Info: {class_info}")
        
        # Calculate and display FPS
        frame_count += 1
        if frame_count >= 10:
            end_time = time.time()
            fps = frame_count / (end_time - start_time)
            frame_count = 0
            start_time = time.time()
        
        cv2.putText(frame, f'FPS: {fps:.2f}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # Update the output frame
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
    # Start a thread that will perform object detection
    t = threading.Thread(target=detect_objects)
    t.daemon = True
    t.start()
    
    # Start the flask app
    app.run(host="0.0.0.0", port=8000, debug=True,
        threaded=True, use_reloader=False)