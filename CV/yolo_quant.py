import torch
from ultralytics import YOLO


# Load the original model
model = YOLO(r'G:\Other computers\Surface\Uni Work\2024\Sem 2\ECE4191\github\ECE4191_G43_S2_2024\CV\saved_models\yolov8n\best_saved_model\best.pt')

# Get the underlying PyTorch model
pytorch_model = model.model

# Apply dynamic quantization
quantized_model = torch.quantization.quantize_dynamic(
    pytorch_model,
    {torch.nn.Linear, torch.nn.Conv2d},
    dtype=torch.qint8
)

# Replace the model's underlying PyTorch model with the quantized version
model.model = quantized_model

torch.save(model.model.state_dict(), 'best_quantized.pt')

# Save the entire model
torch.save(model.model, 'best_quantized_full.pt')