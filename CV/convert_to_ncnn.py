from ultralytics import YOLO

# Load your trained model
model = YOLO('./saved_models/yolov8n/best_saved_model/best.pt')

# Export the model to ONNX format
model.export(format='ncnn')
