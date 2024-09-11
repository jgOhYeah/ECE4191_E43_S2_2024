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




# Initialize Flask app
app = Flask(__name__)

# MQTT setup
MQTT_BROKER = "localhost"  # or the IP of your Raspberry Pi if running on a different device
MQTT_PORT = 1883
MQTT_TOPIC = "/vision/balls"
try:
    mqtt_client = mqtt.Client()
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_start()
except Exception as e:
    print(f"Failed to connect to MQTT broker: {e}")
    mqtt_client = None


# Global variables
outputFrame = None
lock = threading.Lock()



class VideoStream:
    def __init__(self, resolution=(640, 480), framerate=30):
        self.stream = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Use DirectShow on Windows
        if not self.stream.isOpened():
            raise ValueError("Could not open video stream")
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.stream.set(3, resolution[0])
        self.stream.set(4, resolution[1])
        self.stream.set(cv2.CAP_PROP_FPS, framerate)
        self.stopped = False
        self.frame = None
        self.is_pi = platform.system() == "Linux" and platform.machine().startswith("arm")

    def start(self):
        if self.is_pi:
            threading.Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while not self.stopped:
            ret, frame = self.stream.read()
            if ret:
                self.frame = frame
            else:
                print("Failed to capture frame")

    def read(self):
        if self.is_pi:
            return self.frame
        else:
            ret, frame = self.stream.read()
            if not ret:
                print("Failed to capture frame")
                return None
            return frame

    def stop(self):
        self.stopped = True
        if self.stream.isOpened():
            self.stream.release()

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

        # Implement some bounds to handle extreme values
        if estimated_distance < 0:
            estimated_distance = 0
        elif estimated_distance > 2.5:  # Assuming we don't expect distances beyond 2.5m
            estimated_distance = 2.5

        # Apply smoothing
        if self.distance_history:
            smoothed_distance = (self.smoothing_factor * estimated_distance + 
                                 (1 - self.smoothing_factor) * self.distance_history[-1])
        else:
            smoothed_distance = estimated_distance

        self.distance_history.append(smoothed_distance)
        
        return smoothed_distance

    def print_calibration(self):
        print("Calibration Points:")
        for dist, area in zip(self.known_distances, self.known_pixel_areas):
            estimated = self._estimation_function(area, *self.popt)
            print(f"Actual: {dist:.2f}m, Area: {area}, Estimated: {estimated:.2f}m")

#Defining and parsing input arguments you can add to further customise the mode
parser = argparse.ArgumentParser()
parser.add_argument('--modeldir', help='Folder the .tflite file is located in',
                    required=True)
parser.add_argument('--graph', help='Name of the .tflite file, if different than detect.tflite',
                    default='detect.tflite')
parser.add_argument('--labels', help='Name of the labelmap file, if different than labelmap.txt',
                    default='labelmap.txt')
parser.add_argument('--threshold', help='Minimum confidence threshold for displaying detected objects',
                    default=0.5,required=True)
parser.add_argument('--resolution', help='Desired webcam resolution in WxH. If the webcam does not support the resolution entered, errors may occur.',
                    default='640x480')
parser.add_argument('--edgetpu', help='Use Coral Edge TPU Accelerator to speed up detection',
                    action='store_true')

args = parser.parse_args()

MODEL_NAME = args.modeldir
# some stuff from the github repo in Acknowledgements 
GRAPH_NAME = args.graph
LABELMAP_NAME = args.labels
min_conf_threshold = float(args.threshold)
resW, resH = args.resolution.split('x')
imW, imH = int(resW), int(resH)
use_TPU = args.edgetpu

pkg = importlib.util.find_spec('tflite_runtime')
if pkg:
    from tflite_runtime.interpreter import Interpreter
    if use_TPU:
        from tflite_runtime.interpreter import load_delegate
else:
    from tensorflow.lite.python.interpreter import Interpreter
    if use_TPU:
        from tensorflow.lite.python.interpreter import load_delegate

CWD_PATH = os.getcwd()
PATH_TO_CKPT = os.path.join(CWD_PATH,MODEL_NAME,GRAPH_NAME)

PATH_TO_LABELS = os.path.join(CWD_PATH,MODEL_NAME,LABELMAP_NAME)

with open(PATH_TO_LABELS,'r') as f:
    labels = [line.strip() for line in f.readlines()]

if labels[0] == '???':
    del(labels[0])


interpreter = Interpreter(model_path=PATH_TO_CKPT)

interpreter.allocate_tensors()

# Get model details
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()
height = input_details[0]['shape'][1]
width = input_details[0]['shape'][2]

floating_model = (input_details[0]['dtype'] == np.float32)

input_mean = 127.5
input_std = 127.5

# Check output layer name to determine if this model was created with TF2 or TF1,
# because outputs are ordered differently for TF2 and TF1 models
outname = output_details[0]['name']

if ('StatefulPartitionedCall' in outname): # This is a TF2 model
    boxes_idx, classes_idx, scores_idx = 1, 3, 0
else: # This is a TF1 model
    boxes_idx, classes_idx, scores_idx = 0, 1, 2

videostream = None  # Declare videostream as a global variable

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
    x_normalized = 2 * x / (image_width -1) - 1
    y_normalized = 2 * y / (image_height - 1) -1 
    print(f"x: {x}, y: {y}, image_width: {image_width}, image_height: {image_height}")
    return x_normalized, y_normalized



def detect_object():
    # Function to detect the object in the video stream
    global outputFrame,lock, videostream
    videostream = VideoStream(resolution=(imW,imH),framerate=30).start()
    time.sleep(1)

    frame_rate_calc = 1
    freq = cv2.getTickFrequency()

    distance_estimator = estimate_distance()

    while True:
        # Setting Min and Max Box areas to get rid of false positives 
        # these can be changed to suit the needs of the project by observing the results of false positives
        MIN_BOX_AREA = 50
        MAX_BOX_AREA = 400000
        
        #Starting timer for frame rate
        t1 = cv2.getTickCount()
        #Grabbing frame from video stream
        frame1 = videostream.read()
        if frame1 is None:
            print("Failed to capture frame. Retrying...")
            time.sleep(0.1)  # Wait a bit before trying again
            continue

        frame = frame1.copy()
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height))
        input_data = np.expand_dims(frame_resized, axis=0)

        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()

        boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
        classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
        scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects
        keep = NMS(boxes,scores,0.9)
    #   for i in range(len(scores)):
        frame_ball_detections = []
        for i in keep:
            #if ((scores[i] > min_conf_threshold) and (scores[i] <=1.0)):
            if scores[i] > min_conf_threshold:
                ymin = int(max(1,(boxes[i][0] * frame.shape[0])))
                xmin = int(max(1,(boxes[i][1] * frame.shape[1])))
                ymax = int(min(imH,(boxes[i][2] * frame.shape[0])))
                xmax = int(min(imW,(boxes[i][3] * frame.shape[1])))

                # Calculating the area 
                box_area = (xmax - xmin) * (ymax-ymin)
                confidence = int(scores[i]*100)
                # Check to see if box size is within acceptable range. 
                if (MIN_BOX_AREA <= box_area <= MAX_BOX_AREA):
                    aspect_ratio = (xmax - xmin) / (ymax - ymin)
                    modified_area = box_area * aspect_ratio
                    if 0.7 <= aspect_ratio <= 1/0.7 or confidence>=75: 
                        cv2.rectangle(frame,(xmin,ymin),(xmax,ymax),(10,255,0),2)
                        
                        #DEBGGING 
                        center_x = int((xmin + xmax) / 2)
                        center_y = int((ymin + ymax) / 2)
                        # print(f"Original box: {boxes[i]}")
                        # print(f"Frame shape: {frame.shape}")
                        # print(f"Calculated box: xmin={xmin}, ymin={ymin}, xmax={xmax}, ymax={ymax}")
                        estimated_distance = distance_estimator.estimate_distance(modified_area)

                        

                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                        # Draw label
                        object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
                        label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                        label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                        cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                        cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
                        xmin_norm, ymin_norm = normalize_coordinates(xmin,ymin,frame.shape[1],frame.shape[0])
                        xmax_norm, ymax_norm = normalize_coordinates(xmax,ymax,frame.shape[1],frame.shape[0])
                        # sending the info through MQTT
                        frame_ball_detections.append({
                            "xmin,xmax norm": [xmin_norm,xmax_norm],
                            "ymin,ymax": [ymin_norm,ymax_norm],
                            "center point": [center_x,center_y],
                            "confidence": confidence,
                            "area": modified_area,
                            "distance": estimated_distance
                        })

                        print(frame.shape[0])
                        print(f"Boundary Box at: x: [{xmin_norm:.2f},{xmax_norm:.2f}] [{xmin:.2f},{xmax:.2f}], y: [{ymin_norm:.2f},{ymax_norm:.2f}] [{ymin:.2f},{ymax:.2f}], area: {box_area:.4f}, conf.: {int(scores[i]*100)}%, aspect_ratio: {aspect_ratio}, mod. area: {modified_area}, est. dist.: {estimated_distance}")

                        
                        
        if mqtt_client:
            mqtt_client.publish(MQTT_TOPIC, json.dumps(frame_ball_detections))                        
        cv2.putText(frame,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)

        # Calculate framerate
        t2 = cv2.getTickCount()
        time1 = (t2-t1)/freq
        frame_rate_calc= 1/time1

        # Update the output frame
        with lock:
            outputFrame = frame.copy()



def generate():
    global outputFrame, lock
    while True:
        with lock:
            if outputFrame is None:
                continue
            (flag,encodedImg) = cv2.imencode(".jpg",outputFrame)
            if not flag:
                continue
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImg) + b'\r\n')
@app.route("/")
def video_feed():
    return Response(generate(),mimetype = "multipart/x-mixed-replace; boundary=frame")

if __name__ == '__main__':
    # Start a thread that will perform object detection
    t = threading.Thread(target=detect_object)
    t.daemon = True
    t.start()

    # Start the flask app
    app.run(host="0.0.0.0", port=8000, debug=True,
        threaded=True, use_reloader=False)

# Clean up
cv2.destroyAllWindows()
videostream.stop()
mqtt_client.loop_stop()
mqtt_client.disconnect()