import cv2
from ultralytics import YOLO
from picamera2 import Picamera2
import time
from flask import Flask, render_template, Response
import threading
import numpy as np
import paho.mqtt.client as mqtt
import json
import sys
import psutil
import os
from werkzeug.serving import make_server
import socket  # Add this import

# Set the number of cores to use (leave one core free)
num_cores = psutil.cpu_count() - 2
os.environ["OMP_NUM_THREADS"] = str(num_cores)
os.environ["OPENBLAS_NUM_THREADS"] = str(num_cores)
os.environ["MKL_NUM_THREADS"] = str(num_cores)
os.environ["VECLIB_MAXIMUM_THREADS"] = str(num_cores)
os.environ["NUMEXPR_NUM_THREADS"] = str(num_cores)

# Limit OpenCV threads
cv2.setNumThreads(num_cores)

# Initialize Flask app
app = Flask(__name__)

# Load the YOLOv8 model
model = YOLO('/home/tennis/ECE4191_G43_S2_2024/CV/saved_models/yolov8n/best_saved_model/best.pt')

# Initialize the Raspberry Pi Camera
picam2 = Picamera2()
# Configure the camera to capture at 1920x1080 resolution
picam2.configure(picam2.create_preview_configuration(main={"size": (1920, 1080), "format": "RGB888"}))
picam2.start()
time.sleep(2)  # Give the camera time to warm up

# Global variables
output_frame = None
lock = threading.Lock()
running = True  # Flag to control the threads
detection_thread = None
cleanup_done = threading.Event()  # Add this line

# MQTT setup
mqtt_broker = "localhost"  # Replace with your MQTT broker IP or hostname
mqtt_port = 1883
mqtt_topic = "comp_vis/balls"

client = mqtt.Client(protocol=mqtt.MQTTv311)
client.connect(mqtt_broker, mqtt_port, 60)
client.loop_start()

def cleanup():
    global running, detection_thread, server, picam2, client, app

    if cleanup_done.is_set():
        return  # Cleanup has already been performed

    print("Cleaning up resources...")
    running = False

    if detection_thread and detection_thread.is_alive():
        print("Waiting for detection thread to finish")
        detection_thread.join(timeout=5)
    
    if server:
        print("Shutting down Flask server")
        server.shutdown()

    if picam2:
        print("Closing Camera")
        picam2.close()

    if client:
        print("Stopping MQTT Client")
        client.loop_stop()
        client.disconnect()

    cv2.destroyAllWindows()
    print("Cleanup completed.")
    cleanup_done.set()

class ServerThread(threading.Thread):

    def __init__(self, app):
        threading.Thread.__init__(self)
        self.server = make_server('0.0.0.0', 5000, app)
        self.ctx = app.app_context()
        self.ctx.push()

    def run(self):
        print('Starting Flask server')
        self.server.serve_forever()

    def shutdown(self):
        print('Shutting down Flask server')
        self.server.shutdown()

def letterbox_image(image, size):
    '''Resize image with unchanged aspect ratio using padding (letterbox)'''
    ih, iw = image.shape[:2]
    h, w = size
    scale = min(w / iw, h / ih)
    nw = int(iw * scale)
    nh = int(ih * scale)

    # Resize the image
    image_resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)

    # Calculate padding
    top = (h - nh) // 2
    bottom = h - nh - top
    left = (w - nw) // 2
    right = w - nw - left

    # Add padding
    color = [0, 0, 0]  # Black color for padding
    new_image = cv2.copyMakeBorder(image_resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return new_image, scale, left, top

def detect_objects():
    global output_frame, lock, running
    try:
        while running:
            # Capture frame-by-frame (frame is in RGB format)
            frame = picam2.capture_array()

            if frame is None:
                print("Error: Failed to capture image")
                break

            # Resize frame to 640x640 with letterboxing
            frame_resized, scale, left, top = letterbox_image(frame, (640, 640))

            # Perform object detection on the resized frame
            results = model(frame_resized)

            detections = []  # List to hold all detections in the frame

            # Convert original frame to BGR for OpenCV drawing
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Draw bounding boxes on the frame and collect detections
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = box.xyxy[0]  # Bounding box coordinates on resized image

                    # Adjust coordinates back to original image scale
                    x1 = (x1 - left) / scale
                    y1 = (y1 - top) / scale
                    x2 = (x2 - left) / scale
                    y2 = (y2 - top) / scale

                    x1 = int(max(0, min(x1, frame_bgr.shape[1])))
                    y1 = int(max(0, min(y1, frame_bgr.shape[0])))
                    x2 = int(max(0, min(x2, frame_bgr.shape[1])))
                    y2 = int(max(0, min(y2, frame_bgr.shape[0])))

                    cls = int(box.cls[0])  # Class ID
                    conf = float(box.conf[0])  # Confidence score
                    if conf > 0.5:
                        # Calculate center coordinates
                        x_center = x1 + (x2 - x1) / 2
                        y_center = y1 + (y2 - y1) / 2
                        # Map class ID to category name
                        category = "box" if cls == 0 else "ball"

                        detection = {
                            "x1,y1,x2,y2": [x1,y1,x2,y2],
                            "coords": [x_center, y_center],
                            "confidence": conf,
                            "category": category
                        }
                        detections.append(detection)

                        label = f"{category}, Conf: {conf:.2f}"

                        # Draw the bounding box on the BGR image
                        cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)

                        # Improved text rendering
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 0.7
                        font_thickness = 2
                        text_size = cv2.getTextSize(label, font, font_scale, font_thickness)[0]

                        # Calculate background rectangle position
                        rect_x1 = x1
                        rect_y1 = y1 - text_size[1] - 10
                        rect_x2 = x1 + text_size[0] + 10
                        rect_y2 = y1

                        # Draw background rectangle
                        cv2.rectangle(frame_bgr, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 0, 0), -1)

                        # Draw text
                        cv2.putText(frame_bgr, label, (x1 + 5, y1 - 5), font, font_scale,
                                    (255, 255, 255), font_thickness, cv2.LINE_AA)

            # Publish detections via MQTT
            if running:
                client.publish(mqtt_topic, json.dumps(detections))

            # Acquire lock to set the global output frame (still in BGR format)
            with lock:
                output_frame = frame_bgr.copy()

            # Add a small sleep to prevent high CPU usage
            time.sleep(0.1)

    except Exception as e:
        print(f"Error in detect_objects: {e}")
    finally:
        print("Exiting detect_objects thread")

def generate_frames():
    global output_frame, lock, running
    while running:
        with lock:
            if output_frame is None:
                continue
            # Convert BGR back to RGB before encoding
            frame_rgb = cv2.cvtColor(output_frame, cv2.COLOR_BGR2RGB)
            # Encode the frame in JPEG format
            (flag, encodedImage) = cv2.imencode(".jpg", frame_rgb)
            if not flag:
                continue
        # Yield the output frame in byte format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + encodedImage.tobytes() + b'\r\n')

@app.route('/')
def index():
    # Return the rendered template
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    # Return the response generated along with the specific media type
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Doesn't have to be reachable
        s.connect(('8.8.8.8', 80))
        local_ip = s.getsockname()[0]
    except Exception:
        local_ip = '127.0.0.1'
    finally:
        s.close()
    return local_ip

if __name__ == '__main__':
    try:
        # Start threads and main loop as before
        detection_thread = threading.Thread(target=detect_objects, name="DetectionThread")
        detection_thread.start()
        print("Detection thread started")

        server = ServerThread(app)
        server.start()
        print("Flask server thread started")

        # Get the local IP address and print the server URL
        local_ip = get_ip_address()
        print(f"Server running on http://{local_ip}:5000/")

        while running:
            time.sleep(1)
    except KeyboardInterrupt:
        print("KeyboardInterrupt received")
        cleanup()
    except Exception as e:
        print(f"Unexpected error: {e}")
        cleanup()
