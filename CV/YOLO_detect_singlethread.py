import os
import argparse
import cv2
import numpy as np
import sys
import time
from flask import Flask, Response
# import paho.mqtt.client as mqtt  # Commented out since MQTT is not used now
import json
from scipy.optimize import curve_fit
from collections import deque
import ncnn
from picamera2 import Picamera2
import atexit

# Initialize Flask app
app = Flask(__name__)

# Global variables
model = None
distance_estimator = None
args = None
picam2 = None

class DistanceEstimator:
    # Distance estimation using a power law function
    def __init__(self, smoothing_factor=0.2, max_history=10):
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
        estimated_distance = self._estimation_function(pixel_area, *self.popt)
        estimated_distance = max(0, min(estimated_distance, 2.5))

        if self.distance_history:
            smoothed_distance = (self.smoothing_factor * estimated_distance +
                                 (1 - self.smoothing_factor) * self.distance_history[-1])
        else:
            smoothed_distance = estimated_distance

        self.distance_history.append(smoothed_distance)
        return smoothed_distance

class YOLOv8_ncnn:
    def __init__(self, param_path, bin_path, num_threads=2, resize_mode='crop', num_classes=2):
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False  # Set to True if Vulkan is available
        self.net.opt.num_threads = num_threads

        # Load param and bin files
        ret = self.net.load_param(param_path)
        if ret != 0:
            print(f"Failed to load param file: {param_path}")
            exit(1)
        
        ret = self.net.load_model(bin_path)
        if ret != 0:
            print(f"Failed to load bin file: {bin_path}")
            exit(1)

        self.resize_mode = resize_mode
        self.num_classes = num_classes

    def detect(self, img):
        # Preprocess the image based on the resize_mode
        if self.resize_mode == 'crop':
            # Center crop the image to 640x640
            height, width = img.shape[:2]
            start_x = max((width - 640) // 2, 0)
            start_y = max((height - 640) // 2, 0)
            end_x = start_x + 640
            end_y = start_y + 640
            img_cropped = img[start_y:end_y, start_x:end_x]
            img_resized = img_cropped  # Already 640x640
        elif self.resize_mode == 'resize':
            # Resize the entire image to 640x640, changing aspect ratio
            img_resized = cv2.resize(img, (640, 640))
            start_x = 0
            start_y = 0
        else:
            raise ValueError("Invalid resize_mode. Choose 'crop' or 'resize'.")

        # Continue with preprocessing and inference
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        mat_in = ncnn.Mat.from_pixels(img_rgb, ncnn.Mat.PixelType.PIXEL_RGB, 640, 640)
        mean_vals = [0, 0, 0]
        norm_vals = [1/255.0, 1/255.0, 1/255.0]
        mat_in.substract_mean_normalize(mean_vals, norm_vals)

        ex = self.net.create_extractor()
        ex.input("in0", mat_in)
        ret, mat_out = ex.extract("out0")

        if ret != 0:
            return []

        # Process the outputs, adjusting for any cropping
        return self._process_output(mat_out, img_resized.shape[1], img_resized.shape[0],
                                    img.shape[1], img.shape[0], start_x, start_y)

    def _process_output(self, mat_out, img_width_resized, img_height_resized,
                        img_width_original, img_height_original, start_x=0, start_y=0):
        detections = []
        for i in range(mat_out.h):
            values = np.array(mat_out.row(i))
            if len(values) < 6:
                continue
            class_id = int(values[0])
            score = values[1]
            x1 = values[2] * img_width_resized + start_x
            y1 = values[3] * img_height_resized + start_y
            x2 = values[4] * img_width_resized + start_x
            y2 = values[5] * img_height_resized + start_y
            detections.append((class_id, x1, y1, x2, y2, score))
        return detections

def process_results(results, img_width, img_height):
    detections = []
    for detection in results:
        class_idx, x1, y1, x2, y2, confidence = detection
        if confidence > args.confidence_threshold:
            # Ensure coordinates are within image bounds
            x1 = max(0, min(int(x1), img_width - 1))
            y1 = max(0, min(int(y1), img_height - 1))
            x2 = max(0, min(int(x2), img_width - 1))
            y2 = max(0, min(int(y2), img_height - 1))

            area = (x2 - x1) * (y2 - y1)
            if area <= 0:
                continue

            estimated_distance = distance_estimator.estimate_distance(area)

            # Normalizing coordinates
            x1_norm, x2_norm = 2 * x1 / (img_width - 1) - 1, 2 * x2 / (img_width - 1) - 1
            y1_norm, y2_norm = 2 * y1 / (img_height - 1) - 1, 2 * y2 / (img_height - 1) - 1
            center_x_norm = (x1_norm + x2_norm) / 2
            center_y_norm = (y1_norm + y2_norm) / 2

            detection_dict = {
                "xmin_norm": x1_norm,
                "xmax_norm": x2_norm,
                "ymin_norm": y1_norm,
                "ymax_norm": y2_norm,
                "center_point": [center_x_norm, center_y_norm],
                "confidence": float(confidence),
                "class": int(class_idx),
                "area": float(area),
                "distance": float(estimated_distance)
            }
            detections.append(detection_dict)
    return detections

def draw_detections(frame, detections):
    for detection in detections:
        x1 = int((detection["xmin_norm"] + 1) * (frame.shape[1] -1 ) / 2)
        x2 = int((detection["xmax_norm"] + 1) * (frame.shape[1] -1 ) / 2)
        y1 = int((detection["ymin_norm"] + 1) * (frame.shape[0] -1 ) / 2)
        y2 = int((detection["ymax_norm"] + 1) * (frame.shape[0] -1 ) / 2)
        label = f"{detection['class']}: {detection['confidence']:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), (10, 255, 0), 2)
        cv2.putText(frame, label, (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (10, 255, 0), 2)

        # Optionally, draw center point
        center_x = int((x1 + x2) / 2)
        center_y = int((y1 + y2) / 2)
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)

    return frame

@app.route('/')
def video_feed():
    # Capture image, process, and return as response
    global model, distance_estimator, args, picam2
    try:
        # Capture image
        frame = picam2.capture_array()

        # Run detection
        results = model.detect(frame)
        detections = process_results(results, frame.shape[1], frame.shape[0])
        annotated_frame = draw_detections(frame, detections)

        # Encode image to JPEG
        ret, jpeg = cv2.imencode('.jpg', annotated_frame)
        if not ret:
            return "Failed to encode image", 500

        # Return image as response
        return Response(jpeg.tobytes(), mimetype='image/jpeg')
    except Exception as e:
        return f"Error: {e}", 500

def cleanup():
    global picam2
    if picam2 is not None:
        picam2.close()
        print("Camera closed.")

if __name__ == '__main__':
    # Parse command-line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('--param_path', type=str, default='best-opt.param', help='Path to param file')
    parser.add_argument('--bin_path', type=str, default='best-opt.bin', help='Path to bin file')
    parser.add_argument('--confidence_threshold', type=float, default=0.5, help='Minimum confidence threshold for displaying detected objects')
    parser.add_argument('--resolution', help='Desired webcam resolution in WxH.', default='640x480')
    # Removed MQTT-related arguments since MQTT is not used now
    parser.add_argument('--resize_mode', type=str, default='crop', choices=['crop', 'resize'], help='Image resize mode')
    parser.add_argument('--num_threads', type=int, default=2, help='Number of threads for model inference')
    parser.add_argument('--num_classes', type=int, default=2, help='Number of classes in the model')
    args = parser.parse_args()

    # Process resolution
    resW, resH = args.resolution.split('x')
    imW, imH = int(resW), int(resH)

    # Initialize the model
    model = YOLOv8_ncnn(
        param_path=args.param_path,
        bin_path=args.bin_path,
        num_threads=args.num_threads,
        resize_mode=args.resize_mode,
        num_classes=args.num_classes
    )

    # Initialize distance estimator
    distance_estimator = DistanceEstimator()

    # Initialize the camera
    picam2 = Picamera2()
    picam2.configure(picam2.create_preview_configuration(
        main={"size": (imW, imH), "format": "RGB888"}))
    picam2.start()
    time.sleep(0.1)  # Allow camera to warm up

    # Register cleanup function
    atexit.register(cleanup)

    # Start the Flask app in single-threaded mode
    app.run(host="0.0.0.0", port=8000, debug=False, threaded=False)

    # Clean up
    cleanup()
