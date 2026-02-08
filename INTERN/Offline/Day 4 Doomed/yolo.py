import cv2
import torch
from global_variable import *

model = torch.hub.load(
    'ultralytics/yolov5',
    'custom',
    path='model.pt',
    force_reload=False
)

cap = cv2.VideoCapture(0)

def detect_object():
    ret, frame = cap.read()
    if not ret:
        return False, None, None, frame

    results = model(frame)
    detections = results.xyxy[0]

    if len(detections) == 0:
        return False, None, None, frame

    # ambil objek dengan confidence tertinggi
    best = detections[detections[:, 4].argmax()]
    x1, y1, x2, y2 = best[:4]

    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)

    return True, cx, cy, frame
