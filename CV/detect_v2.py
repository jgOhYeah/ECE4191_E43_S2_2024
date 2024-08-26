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


# Initialize Flask app
app = Flask(__name__)

# MQTT setup
MQTT_BROKER = "localhost"  # or the IP of your Raspberry Pi if running on a different device
MQTT_PORT = 1883
MQTT_TOPIC = "/vision/balls"
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.loop_start()

ball_detections = []
last_publish_time = time.time()

# Global variables
outputFrame = None
lock = threading.Lock()

class VideoStream:
    def __init__(self, resolution=(640, 480), framerate=30):
        self.stream = cv2.VideoCapture(0)
        self.stream.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.stream.set(3, resolution[0])
        self.stream.set(4, resolution[1])
        self.stopped = False

        #calculating fps 
        set
    
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
    boxes = boxes.astype(float)
    x1 = boxes[:,1]
    y1 = boxes[:,0]
    x2 = boxes[:,3]
    y2 = boxes[:,2]
    areas = (x2 - x1 + 1) * (y2-y1 +1)
    order = scores.argsort()[::-1]
    keep = []

    while order.size > 0:
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




def detect_object():
    global outputFrame,lock, videostream, ball_detections, last_publish_time
    videostream = VideoStream(resolution=(imW,imH),framerate=30).start()
    time.sleep(1)

    frame_rate_calc = 1
    freq = cv2.getTickFrequency()

    while True:
        # Setting Min and Max Box areas to get rid of false positives 
        MIN_BOX_AREA = 50
        MAX_BOX_AREA = 400000
        
        #Starting timer for frame rate
        t1 = cv2.getTickCount()
        #Grabbing frame from video stream
        frame1 = videostream.read()

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




                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                        # Draw label
                        object_name = labels[int(classes[i])] # Look up object name from "labels" array using class index
                        label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                        labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                        label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                        cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                        cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text

                        # sending the info through MQTT
                        ball_detections.append({
                            "coords": [center_x,center_y],
                            "confidence": confidence,
                            "area": modified_area
                        })



                        current_time = time.time()
                        #if current_time - last_publish_time > 1:
                        if ball_detections:
                            mqtt_client.publish(MQTT_TOPIC, json.dumps(ball_detections))
                            ball_detections = []
                        last_publish_time = current_time



                        print(f"Boundary Box at: x: [{xmin},{xmax}], y: [{ymin},{ymax}], area: {box_area}, confidence: {int(scores[i]*100)}%, aspect_ratio: {aspect_ratio}, mod. area: {modified_area}")

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