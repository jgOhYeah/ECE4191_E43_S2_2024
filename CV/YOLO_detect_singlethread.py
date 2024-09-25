import cv2
from ultralytics import YOLO
from picamera2 import Picamera2
import time
from flask import Flask, render_template, Response
import threading
import numpy as np
import paho.mqtt.client as mqtt
import json
import signal
import sys
import psutil
import os
import atexit


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
# Configure the camera
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 640), "format": "RGB888"}))
picam2.start()
time.sleep(2)  # Give the camera time to warm up

# Global variables
output_frame = None
lock = threading.Lock()
running = True  # Flag to control the threads
detection_thread = None
flask_thread = None

# MQTT setup
mqtt_broker = "localhost"  # Replace with your MQTT broker IP or hostname
mqtt_port = 1883
mqtt_topic = "comp_vis/balls"

client = mqtt.Client(protocol=mqtt.MQTTv311)
client.connect(mqtt_broker, mqtt_port, 60)
client.loop_start()

def cleanup():
    global running, detection_thread, flask_thread, picam2, client, app

    if cleanup_done.is_set():
        return  # Cleanup has already been performed

    print("Cleaning up resources...")
    running = False

    if detection_thread and detection_thread.is_alive():
        print("Waiting for detection thread to finish")
        detection_thread.join(timeout=5)
    
    if flask_thread and flask_thread.is_alive():
        print("Waiting for Flask Thread to finish")
        flask_thread.join(timeout=5)

    if picam2:
        print("Closing Camera")
        picam2.close()

    if client:
        print("Stopping MQTT Client")
        client.loop_stop()
        client.disconnect()

    # Force stop any remaining threads
    for thread in threading.enumerate():
        if thread != threading.current_thread() and thread.is_alive():
            try:
                thread._stop()
            except:
                pass

    cv2.destroyAllWindows()
    print("Cleanup completed.")
    cleanup_done.set()


def signal_handler(sig, frame):
    print("Interrupt received, stopping...")
    cleanup()
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)


def detect_objects():
    global output_frame, lock, running
    try:
        while running:
            # Capture frame-by-frame
            frame = picam2.capture_array()

            if frame is None:
                print("Error: Failed to capture image")
                break

            # Convert image from RGB to BGR (for OpenCV)
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

            # Perform object detection
            results = model(frame)

            detections = []  # List to hold all detections in the frame

            # Draw bounding boxes on the frame and collect detections
            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                    cls = int(box.cls[0])  # Class ID
                    conf = float(box.conf[0])  # Confidence score
                    if conf > 0.4:
                        # Calculate center coordinates
                        x_center = x1 + (x2 - x1) / 2
                        y_center = y1 + (y2 - y1) / 2
                        # Map class ID to category name
                        category = "box" if cls == 0 else "ball"

                        detection = {
                            "coords": [x_center, y_center],
                            "confidence": conf,
                            "category": category
                        }
                        detections.append(detection)

                        label = f"{category}, Conf: {conf:.2f}"
                        
                        # Draw the bounding box
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
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
                        cv2.rectangle(frame, (rect_x1, rect_y1), (rect_x2, rect_y2), (0, 0, 0), -1)
                        
                        # Draw text
                        cv2.putText(frame, label, (x1 + 5, y1 - 5), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)

            # Publish detections via MQTT
            if running:
                client.publish(mqtt_topic, json.dumps(detections))

            # Acquire lock to set the global output frame
            with lock:
                output_frame = frame.copy()

            # Add a small sleep to prevent high CPU usage
            time.sleep(0.1)

    except Exception as e:
        print(f"Error in detect_objects: {e}")
    finally:
        print("Exiting detect_objects thread")

def flask_run():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True, use_reloader=False)

def generate_frames():
    global output_frame, lock, running
    while running:
        with lock:
            if output_frame is None:
                continue
            # Encode the frame in JPEG format
            (flag, encodedImage) = cv2.imencode(".jpg", output_frame)
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

if __name__ == '__main__':
    try:
        # Start threads and main loop as before
        detection_thread = threading.Thread(target=detect_objects, name="DetectionThread")
        detection_thread.start()
        print("Detection thread started")

        flask_thread = threading.Thread(target=flask_run, name="FlaskThread")
        flask_thread.start()
        print("Flask thread started")

        while running:
            time.sleep(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        cleanup()
        sys.exit(0)  # Ensure the script exits