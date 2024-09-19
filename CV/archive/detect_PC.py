import cv2
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO('C:/Users/parth/Desktop/Uni Work/2024/Sem 2/ECE4191/github/PC/ECE4191_G43_S2_2024/CV/saved_models/yolov8n/best_saved_model/best.pt')

# Open the webcam
cap = cv2.VideoCapture(0)

while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    if not ret:
        break
    
    # Resize the frame to 640x640
    frame = cv2.resize(frame, (640, 640))
    
    # Run YOLOv8 inference on the frame
    results = model(frame)
    
    # Visualize the results on the frame
    annotated_frame = results[0].plot()
    
    # Display the annotated frame
    cv2.imshow("YOLOv8 Inference", annotated_frame)
    
    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release the webcam and close the window
cap.release()
cv2.destroyAllWindows()