import cv2
import numpy as np
import paho.mqtt.client as mqtt
import json
from picamera2 import Picamera2
import ncnn
import threading
import queue
import time
from scipy.optimize import curve_fit
from collections import deque
from flask import Flask, Response
import sys
import argparse

# Video Streaming Setup
app = Flask(__name__)

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', '1', 'y'):
        return True
    elif v.lower() in ('no', 'false', 'f', '0', 'n'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

class VideoStream:
    def __init__(self, resolution=(640, 480), framerate=30):
        self.camera = Picamera2()
        self.camera.configure(self.camera.create_preview_configuration(main={"size": resolution, "format": "RGB888"}))
        self.camera.start()
        self.frame = None
        self.stopped = False

    def start(self):
        threading.Thread(target=self.update, args=(), daemon=True).start()
        return self

    def update(self):
        while not self.stopped:
            self.frame = self.camera.capture_array()
            time.sleep(1 / 30)  # Adjust for desired framerate

    def read(self):
        return self.frame

    def stop(self):
        self.stopped = True

class DistanceEstimator:
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
        self.net.load_param(param_path)
        self.net.load_model(bin_path)
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
        ex.set_num_threads(self.net.opt.num_threads)
        ex.input("images", mat_in)
        ret, mat_out = ex.extract("output0")

        if ret != 0:
            return []

        # Process the outputs, adjusting for any cropping
        return self._process_output(mat_out, img_resized.shape[1], img_resized.shape[0],
                                    img.shape[1], img.shape[0], start_x, start_y)

    def _process_output(self, mat_out, img_width_resized, img_height_resized,
                        img_width_original, img_height_original, start_x=0, start_y=0):
        detections = []
        # Assuming mat_out is in the format required
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

class YOLODetectionSystem:
    def __init__(self, param_path, bin_path, mqtt_broker="localhost", mqtt_port=1883,
                 resize_mode='crop', confidence_threshold=0.4, num_threads=2, num_classes=2):
        self.model = YOLOv8_ncnn(
            param_path=param_path,
            bin_path=bin_path,
            num_threads=num_threads,
            resize_mode=resize_mode,
            num_classes=num_classes
        )
        self.confidence_threshold = confidence_threshold
        self.distance_estimator = DistanceEstimator()
        self.video_stream = VideoStream().start()
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect(mqtt_broker, mqtt_port, 60)
        self.mqtt_client.loop_start()
        self.output_frame = None
        self.lock = threading.Lock()
        self.fps = 0

    def detect_objects(self):
        prev_time = time.time()
        while True:
            frame = self.video_stream.read()
            if frame is None:
                continue

            results = self.model.detect(frame)
            detections = self.process_results(results, frame.shape[1], frame.shape[0])
            annotated_frame = self.draw_detections(frame, detections)

            # Calculating FPS
            curr_time = time.time()
            self.fps = 1 / (curr_time - prev_time)
            prev_time = curr_time

            # Publishing MQTT message
            mqtt_message = json.dumps(detections)
            self.mqtt_client.publish("vision/balls", mqtt_message)

            # Printing MQTT message to the command line
            print("MQTT Message Published:", mqtt_message)

            # Updating the output frame
            with self.lock:
                self.output_frame = annotated_frame.copy()

    def process_results(self, results, img_width, img_height):
        detections = []
        for detection in results:
            class_idx, x1, y1, x2, y2, confidence = detection
            if confidence > self.confidence_threshold:
                # Ensure coordinates are within image bounds
                x1 = max(0, min(int(x1), img_width - 1))
                y1 = max(0, min(int(y1), img_height - 1))
                x2 = max(0, min(int(x2), img_width - 1))
                y2 = max(0, min(int(y2), img_height - 1))

                area = (x2 - x1) * (y2 - y1)
                distance = self.distance_estimator.estimate_distance(area)

                # Normalizing coordinates
                x1_norm, x2_norm = 2 * x1 / img_width - 1, 2 * x2 / img_width - 1
                y1_norm, y2_norm = 2 * y1 / img_height - 1, 2 * y2 / img_height - 1
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
                    "distance": float(distance)
                }
                detections.append(detection_dict)
        return detections

    def draw_detections(self, frame, detections):
        for detection in detections:
            x1 = int((detection["xmin_norm"] + 1) * frame.shape[1] / 2)
            x2 = int((detection["xmax_norm"] + 1) * frame.shape[1] / 2)
            y1 = int((detection["ymin_norm"] + 1) * frame.shape[0] / 2)
            y2 = int((detection["ymax_norm"] + 1) * frame.shape[0] / 2)
            label = f"{detection['class']}: {detection['confidence']:.2f}"
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Optionally display FPS on the frame
        cv2.putText(frame, f"FPS: {self.fps:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        return frame

# Flask app for video streaming
@app.route('/')
def index():
    return Response(gen(yolo_system),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

def gen(yolo_system):
    while True:
        with yolo_system.lock:
            if yolo_system.output_frame is None:
                continue
            # Encode the frame in JPEG format
            ret, jpeg = cv2.imencode('.jpg', yolo_system.output_frame)
            if not ret:
                continue
            frame = jpeg.tobytes()

        # Yield the output frame in byte format
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

# Main execution
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='YOLOv8 NCNN Detection System')
    parser.add_argument('--param_path', type=str, default='best-opt.param', help='Path to param file')
    parser.add_argument('--bin_path', type=str, default='best-opt.bin', help='Path to bin file')
    parser.add_argument('--mqtt_broker', type=str, default='localhost', help='MQTT broker address')
    parser.add_argument('--mqtt_port', type=int, default=1883, help='MQTT broker port')
    parser.add_argument('--show_video', type=str2bool, nargs='?', const=True, default=False, help='Show video feed in a window')
    parser.add_argument('--resize_mode', type=str, default='crop', choices=['crop', 'resize'], help='Image resize mode')
    parser.add_argument('--confidence_threshold', type=float, default=0.4, help='Minimum confidence threshold for detections')
    parser.add_argument('--num_threads', type=int, default=2, help='Number of threads for model inference')
    parser.add_argument('--num_classes', type=int, default=2, help='Number of classes in the model')
    args = parser.parse_args()

    # Initialize the detection system with the parsed arguments
    yolo_system = YOLODetectionSystem(
        param_path=args.param_path,
        bin_path=args.bin_path,
        mqtt_broker=args.mqtt_broker,
        mqtt_port=args.mqtt_port,
        resize_mode=args.resize_mode,
        confidence_threshold=args.confidence_threshold,
        num_threads=args.num_threads,
        num_classes=args.num_classes
    )

    detection_thread = threading.Thread(target=yolo_system.detect_objects, daemon=True)
    detection_thread.start()

    if args.show_video:
        # Show video feed in a window
        while True:
            with yolo_system.lock:
                if yolo_system.output_frame is None:
                    continue
                cv2.imshow("Detection", yolo_system.output_frame)
            if cv2.waitKey(1) == ord('q'):
                break
        cv2.destroyAllWindows()
        yolo_system.video_stream.stop()
    else:
        # Start Flask app for video streaming
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
