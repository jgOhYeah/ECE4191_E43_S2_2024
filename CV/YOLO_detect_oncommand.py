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
cleanup_done = threading.Event()  # Add this line

take_photo_event = threading.Event()


# MQTT setup
def on_message(client,userdata,msg):
    payload = msg.payload.decode()
    if payload == 'take_photo':
        if not take_photo_event.is_set():
            take_photo_event.set()


mqtt_broker = "localhost"  # Replace with your MQTT broker IP or hostname
mqtt_port = 1883
mqtt_topic = "comp_vis/balls"
command_topic = "comp_vis/commands"
status_topic = "comp_vis/status"

client = mqtt.Client(protocol=mqtt.MQTTv311)
client.on_message = on_message
client.connect(mqtt_broker, mqtt_port, 60)
client.subscribe(command_topic)     #subscribe to command topic)
client.loop_start()

def publish_status(status_dict):
    client.publish(status_topic, json.dumps(status_dict))

def cleanup():
    global running, detection_thread, flask_thread, picam2, client, app, take_photo_event

    if cleanup_done.is_set():
        return  # Cleanup has already been performed

    print("Cleaning up resources...")
    running = False
    # Set the event to unblock any waiting threads
    take_photo_event.set()

    if detection_thread and detection_thread.is_alive():
        print("Waiting for detection thread to finish")
        detection_thread.join(timeout=5)
    
    if flask_thread and flask_thread.is_alive():
        print("Flask thread is still running")
        # Flask server will be terminated when the process exits

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
            #waiting for the control system to send message
            take_photo_event.wait()
            if not running:
                break

            take_photo_event.clear()

            # publishing "taking photo"
            client.publish(status_topic,"Taking photo")
            publish_status({"taking_photo": True, "photo_taken": False, "processing": False})

            # Capture frame-by-frame (frame is in RGB format)
            frame = picam2.capture_array()

            if frame is None:
                print("Error: Failed to capture image")
                #resetting the flags
                publish_status({"taking_photo": False, "photo_taken": False, "processing": False})

                break

            #publish photo taken, publishing
            client.publish(status_topic,"Photo taken, processing")
            publish_status({"taking_photo": False, "photo_taken": True, "processing": True})


            # Perform object detection (model expects RGB format)
            results = model(frame)

            detections = []  # List to hold all detections in the frame

            # Convert frame to BGR for OpenCV drawing
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

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
                        cv2.putText(frame_bgr, label, (x1 + 5, y1 - 5), font, font_scale, (255, 255, 255), font_thickness, cv2.LINE_AA)

            # Publish detections via MQTT
            if running:
                client.publish(mqtt_topic, json.dumps(detections))
            
            publish_status({"taking_photo": False, "photo_taken": False, "processing": False})

            # Acquire lock to set the global output frame (still in BGR format)
            with lock:
                output_frame = frame_bgr.copy()

            # Add a small sleep to prevent high CPU usage
            time.sleep(0.1)

    except Exception as e:
        print(f"Error in detect_objects: {e}")
        publish_status({"taking_photo": False, "photo_taken": False, "processing": False})
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
    except KeyboardInterrupt:
        print("KeyboardInterrupt received")
        cleanup()
    except Exception as e:
        print(f"Unexpected error: {e}")
        cleanup()
    finally:
        sys.exit(0)  # Ensure the script exits
