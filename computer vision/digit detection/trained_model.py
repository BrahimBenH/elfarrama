from transformers import AutoModelForImageClassification, AutoImageProcessor
import torch
import cv2
from PIL import Image

# Load model and processor
model_name = "farleyknight/mnist-digit-classification-2022-09-04"
model = AutoModelForImageClassification.from_pretrained(model_name)
processor = AutoImageProcessor.from_pretrained(model_name)

# Open webcam
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Optional: define a center ROI for detection
    h, w, _ = frame.shape
    cx, cy, size = w//2, h//2, 200  # center x, y and ROI size
    x1, y1 = cx-size//2, cy-size//2
    x2, y2 = cx+size//2, cy+size//2
    roi = frame[y1:y2, x1:x2]

    # Draw yellow rectangle around ROI
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 255), 2)

    # Prepare image for model
    pil_img = Image.fromarray(cv2.cvtColor(roi, cv2.COLOR_BGR2RGB)).resize((28,28))
    inputs = processor(images=pil_img, return_tensors="pt")

    # Predict
    with torch.no_grad():
        logits = model(**inputs).logits
    predicted_digit = logits.argmax(-1).item()

    # Display prediction on frame
    cv2.putText(frame, f"Digit: {predicted_digit}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0,0,255), 3)

    # Show frame
    cv2.imshow("Digit Recognition", frame)

    # Exit on pressing 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
