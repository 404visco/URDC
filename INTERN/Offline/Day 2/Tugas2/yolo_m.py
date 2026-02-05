from ultralytics import YOLO
import cv2

# Load model
model = YOLO("yolov8n.pt")

# Buka kamera (0 = webcam)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Deteksi objek
    results = model(frame)

    # Gambar bounding box
    annotated_frame = results[0].plot()

    # Tampilkan hasil
    cv2.imshow("Object Detection", annotated_frame)

    # Tekan Q untuk keluar
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
