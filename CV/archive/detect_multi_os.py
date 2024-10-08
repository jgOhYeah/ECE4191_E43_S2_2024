import os
import argparse
import cv2
import numpy as np
import sys
import time
import threading
import importlib.util
from flask import Flask, Response
import paho.mqtt.client as mqtt
import json
from scipy.optimize import curve_fit 
from collections import deque  
import platform
from ultralytics import YOLO
import subprocess
import logging



## Acknowledgements
## This code is based on the following repos:
## https://github.com/EdjeElectronics/TensorFlow-Lite-Object-Detection-on-Android-and-Raspberry-Pi
## the code was influenced by TF_lite_detection_webcam.py which is located in the archives folder. 
## modifications were made to the code to add NMS and estimate of the distance of the object.
## the code was also modified to stream the video from a webcam for Headless Raspberry Pi configurations. 
## Ability to communicate with MQTT was added to send the detections to a MQTT broker (Control Systems)
## 
## NOTE: When viewing the video stream, user may see poor performance and frames may take several seconds to refresh.
## If the local host link is not accessed, then the frames will be much faster.
## 
## NOTE: For certain models the detections may not be accurate when handling distant objects, this can be improved 
## lowering the threshold for the detection algorithm.



# Set up logging
logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                    filename='debug.log',
                    filemode='w')

logger = logging.getLogger(__name__)

# Initialize Flask app
app = Flask(__name__)

# MQTT setup (optional)
mqtt_client = None
try:
    import paho.mqtt.client as mqtt
    MQTT_BROKER = "localhost"
    MQTT_PORT = 1883
    MQTT_TOPIC = "/vision/balls"
    mqtt_client = mqtt.Client()
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()
    logger.info("MQTT client connected successfully")
except Exception as e:
    logger.warning(f"Failed to connect to MQTT broker: {e}. MQTT functionality will be disabled.")

# Global variables
outputFrame = None
lock = threading.Lock()

# loading the model
model = YOLO("./saved_models/yolov8n/best.pt")
logger.info("YOLO model loaded successfully")


class VideoStream:
    def __init__(self, resolution=(640, 480), framerate=30):
        self.resolution = resolution
        self.framerate = framerate
        self.is_pi = platform.system() == "Linux" and platform.machine().startswith("arm")
        self.frame = None 
        self.stopped = False
        self.stream = None
        logger.info(f"Initializing VideoStream. Is Raspberry Pi: {self.is_pi}")

    def start(self):
        logger.info("Starting VideoStream")
        threading.Thread(target=self.update, args=()).start()
        return self

    def update(self):
        logger.info("Entering update method")
        if self.is_pi:
            self.update_pi()
        else:
            self.update_pc()

    def update_pc(self):
        logger.info("Entering update_pc method")
        if self.stream is None:
            logger.info("Initializing VideoCapture")
            self.stream = cv2.VideoCapture(0)
            self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.stream.set(3, self.resolution[0])
            self.stream.set(4, self.resolution[1])
        
        while not self.stopped:
            if not self.stream.isOpened():
                logger.error("Error: VideoCapture not opened")
                time.sleep(1)
                continue

            ret, self.frame = self.stream.read()
            if not ret:
                logger.error("Error: Can't receive frame (stream end?). Exiting ...")
                self.stopped = True
                break
            logger.debug("Frame captured successfully")
            time.sleep(1/self.framerate)

    def update_pi(self):
        logger.info("Entering update_pi method")
        while not self.stopped:
            try:
                cmd = [
                    'libcamera-still',
                    '-n', '--width', str(self.resolution[0]),
                    '--height', str(self.resolution[1]),
                    '--framerate', str(self.framerate),
                    '-o', '-', '-t', '1', '--encoding', 'rgb'
                ]
                logger.debug(f"Running command: {' '.join(cmd)}")
                result = subprocess.run(cmd, capture_output=True, check=True)
                image = np.frombuffer(result.stdout, dtype=np.uint8).reshape(self.resolution[1], self.resolution[0], 3)
                self.frame = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
                logger.debug("Frame captured successfully on Raspberry Pi")
            except Exception as e:
                logger.error(f"Error capturing frame on Raspberry Pi: {e}")
                time.sleep(1)
            time.sleep(1/self.framerate)

    def read(self):
        return self.frame
    
    def stop(self):
        logger.info("Stopping VideoStream")
        self.stopped = True
        if not self.is_pi and self.stream is not None and self.stream.isOpened():
            self.stream.release()

class FrameRateCalc:
    def __init__(self):
        self.freq = cv2.getTickFrequency()
        self.prev_time= 0
        self.curr_fps = 0

    def update(self):
        curr_time = cv2.getTickCount()
        time_diff = (curr_time - self.prev_time) / self.freq
        self.curr_fps = 1 / time_diff
        self.prev_time = curr_time
        return self.curr_fps

class estimate_distance:
    ## Calclating the Approximate distance of the object by using the power law function. 
    ## The detection algorithm can be improved by providing more accurate correlations between the 
    ## known distances and the modified pixel area of the object. 
    # Modified pixel area = pixel area * aspect ratio. 
    def __init__(self, smoothing_factor=0.2, max_history=10):
        # Updated calibration points
        self.known_distances = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.2, 1.5, 2.0]
        self.known_pixel_areas = [282089, 72090, 26404, 14040, 8460, 5698, 4000, 3100, 2401, 2150, 1225, 875, 360]
        
        self.popt = self._fit_function()
        self.smoothing_factor = smoothing_factor
        self.distance_history = deque(maxlen=max_history)

    def _estimation_function(self, x, a, b, c):
        return a * np.power(x, b) + c

    def _fit_function(self):
        popt, _ = curve_fit(self._estimation_function, self.known_pixel_areas, self.known_distances, p0=[1, -0.5, 0])
        return popt

    def estimate_distance(self, pixel_area):
        # Estimate the distance using the fitted function
        estimated_distance = self._estimation_function(pixel_area, *self.popt)
        estimated_distance = max(0, min(estimated_distance, 2.5))

        if self.distance_history:
            smoothed_distance = (self.smoothing_factor * estimated_distance + (1 - self.smoothing_factor) * self.distance_history[-1])
        else:
            smoothed_distance = estimated_distance

        self.distance_history.append(smoothed_distance)
        return smoothed_distance

    def print_calibration(self):
        print("Calibration Points:")
        for dist, area in zip(self.known_distances, self.known_pixel_areas):
            estimated = self._estimation_function(area, *self.popt)
            print(f"Actual: {dist:.2f}m, Area: {area}, Estimated: {estimated:.2f}m")

def NMS(boxes,scores,threshold):
    # Non-Maximum Suppression (NMS) algorithm to remove overlapping bounding boxes
    # inputs: 
    #   boxes: array of bounding boxes (shape: num_boxes x 4)
    #   scores: array of confidence scores (shape: num_boxes x 1)
    #   threshold: threshold for non-maximum suppression (int)
    boxes = boxes.astype(float)
    x1 = boxes[:,1]
    y1 = boxes[:,0]
    x2 = boxes[:,3]
    y2 = boxes[:,2]
    areas = (x2 - x1 + 1) * (y2-y1 +1)
    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
        # comparison of the confidence scores of the bounding boxes and removing the overlapping ones 
        # with the lowest confidence score
        i = order[0] 
        keep.append(i)

        xx1 = np.maximum(x1[i],x1[order[1:]])
        yy1 = np.maximum(y1[i],y1[order[1:]])
        xx2 = np.maximum(x2[i],x2[order[1:]])
        yy2 = np.maximum(y2[i],y2[order[1:]])
        w = np.maximum(0.0,xx2-xx1 + 1)
        h = np.maximum(0.0,yy2-yy1 + 1)
        overlap = (w * h) / (areas[i] + areas[order[1:]] - (w * h))
        inds = np.where(overlap <= threshold)[0]
        order = order[inds + 1]
    return keep

def normalize_coordinates(x, y, image_width, image_height):
    x_normalized = 2 * x / (image_width - 1)  - 1
    y_normalized = 2 * y / (image_height - 1) - 1 
    # print(f"x: {x}, y: {y}, image_width: {image_width}, image_height: {image_height}")
    return x_normalized, y_normalized



def detect_object():
    global outputFrame, lock, videostream
    videostream = VideoStream(resolution=(640,640), framerate=30).start()
    logger.info("VideoStream started in detect_object")
    time.sleep(2)  # Give more time for camera initialization

    frame_rate_calc = FrameRateCalc()
    distance_estimator = estimate_distance()

    while True:
        frame = videostream.read()
        if frame is None:
            logger.warning("Frame is None, retrying...")
            time.sleep(0.1)
            continue
        
        logger.debug(f"Frame shape: {frame.shape}")
        fps = frame_rate_calc.update()

        try:
            results = model(frame)

            for r in results:
                boxes = r.boxes.xyxy.cpu().numpy()
                scores = r.boxes.conf.cpu().numpy()
                classes = r.boxes.cls.cpu().numpy()

                keep = NMS(boxes, scores, 0.5)

                for i in keep:
                    x1, y1, x2, y2 = map(int, boxes[i])
                    confidence = float(scores[i])
                    class_id = int(classes[i])

                    if class_id in [0, 1]:
                        class_name = "Ball" if class_id == 0 else "Box"
                        logger.info(f"Detected {class_name} at ({x1},{y1},{x2},{y2}) with confidence {confidence:.2f}")
                        
                        color = (0, 255, 0) if class_id == 0 else (255, 0, 0)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                        cv2.putText(frame, f'{class_name}: {confidence:.2f}', 
                                    (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

            if mqtt_client:
                mqtt_client.publish(MQTT_TOPIC, json.dumps([{"class": class_name, "confidence": confidence, "bbox": [x1,y1,x2,y2]}]))
        
        except Exception as e:
            logger.error(f"Error during detection: {e}")

        cv2.putText(frame, f'FPS: {fps:.2f}', (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 0), 2)
        
        with lock:
            outputFrame = frame.copy()
        logger.debug("Frame processed and outputFrame updated")

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
                    mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == '__main__':
    t = threading.Thread(target=detect_object)
    t.daemon = True
    t.start()
    
    print("Starting Flask server...")
    print("To view the video feed, open a web browser and go to:")
    print("http://localhost:8000")
    
    app.run(host="0.0.0.0", port=8000, debug=False, use_reloader=False)

# Clean up
cv2.destroyAllWindows()
videostream.stop()
if mqtt_client:
    mqtt_client.loop_stop()
    mqtt_client.disconnect()