import cv2
from ultralytics import YOLO

def yolo8_detect(
    model_path,
    cam_id=0,
    conf=0.25,
    imgsz=640
):
    """
    Fungsi deteksi objek real-time menggunakan YOLOv8

    Parameters:
    - model_path (str): path ke model.pt
    - cam_id (int): id kamera (default 0)
    - conf (float): confidence threshold
    - imgsz (int): ukuran input image
    """

    # Load model
    model = YOLO(model_path)

    # Buka kamera
    cap = cv2.VideoCapture(cam_id)
    if not cap.isOpened():
        print("Kamera tidak bisa dibuka")
        return

    while True:
        ret, frame = cap.read()
        frame = cv2.flip(frame, 1)
        if not ret:
            break

        # Inference
        results = model(frame, conf=conf, imgsz=imgsz, verbose=False)

        # Parsing hasil
        for r in results:
            if r.boxes is None:
                continue

            for box in r.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                score = float(box.conf[0])
                cls_id = int(box.cls[0])
                label = model.names[cls_id]

                # Bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

                # Label
                text = f"{label} {score:.2f}"
                cv2.putText(
                    frame,
                    text,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

        cv2.imshow("YOLOv8 Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

yolo8_detect(
    model_path="C:/Users/visco/JENTAYU/INTERN/Offline/Day 2/best.pt",
    cam_id=0,
    conf=0.2
)
