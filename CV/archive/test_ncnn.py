import ncnn
from picamera2 import Picamera2
import cv2
import numpy as np
import time

print("Initializing camera...")
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(
    main={"size": (640, 480), "format": "RGB888"}))
picam2.start()
time.sleep(2)

print("Loading model...")
net = ncnn.Net()
net.opt.use_vulkan_compute = False  # Adjust if necessary
ret = net.load_param("saved_models/yolov8n/best_saved_model/best_ncnn_model/model.ncnn.param")
if ret != 0:
    print("Failed to load param file")
    exit(1)
ret = net.load_model("saved_models/yolov8n/best_saved_model/best_ncnn_model/model.ncnn.bin")
if ret != 0:
    print("Failed to load bin file")
    exit(1)

try:
    for i in range(10):  # Process 10 frames
        print(f"Processing frame {i+1}...")
        frame = picam2.capture_array()
        img_resized = cv2.resize(frame, (640, 640))
        img_rgb = cv2.cvtColor(img_resized, cv2.COLOR_BGR2RGB)
        mat_in = ncnn.Mat.from_pixels(img_rgb, ncnn.Mat.PixelType.PIXEL_RGB, 640, 640)
        norm_vals = [1/255.0, 1/255.0, 1/255.0]
        mat_in.substract_mean_normalize([], norm_vals)
        ex = net.create_extractor()
        ex.input("in0", mat_in)
        ret, mat_out = ex.extract("out0")
        if ret != 0:
            print(f"Inference failed on frame {i+1}")
        else:
            print(f"Inference succeeded on frame {i+1}")
        time.sleep(0.1)  # Slight delay between frames
except Exception as e:
    print(f"Error during processing: {e}")
finally:
    picam2.close()
    print("Camera closed.")
