import cv2
import threading
import time
import global_variable as gv

from ultralytics import YOLO


class YoloDetector:
    def __init__(self, model_path="model.pt", cam_index=0, conf=0.5):
        self.model = YOLO(model_path)
        self.cap = cv2.VideoCapture(cam_index)
        self.conf = conf
        self.running = False

        if not self.cap.isOpened():
            raise RuntimeError("Camera tidak bisa dibuka")

    def start(self):
        self.running = True
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def stop(self):
        self.running = False
        self.cap.release()

    def _loop(self):
        while self.running:
            if gv.vision_mode != gv.VISION_OBJECT:
                gv.object_detected = False
                time.sleep(0.05)
                continue
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            h, w, _ = frame.shape
            cx_frame = w // 2
            cy_frame = h // 2

            results = self.model(frame, conf=self.conf, verbose=False)

            detected = False

            for r in results:
                if r.boxes is None:
                    continue

                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx_obj = (x1 + x2) // 2
                    cy_obj = (y1 + y2) // 2

                    # error dinormalisasi (-1 sampai 1)
                    gv.object_x_error = (cx_obj - cx_frame) / cx_frame
                    gv.object_y_error = (cy_obj - cy_frame) / cy_frame

                    gv.object_detected = True
                    detected = True
                    break

                if detected:
                    break

            if not detected:
                gv.object_detected = False
                gv.object_x_error = 0.0
                gv.object_y_error = 0.0

            time.sleep(0.05)  # ~20 Hz

class ObjectDetector:
    def __init__(self, model_path="model.pt", cam_index=1, conf=0.5):
        self.model = YOLO(model_path)
        self.cap = cv2.VideoCapture(cam_index)
        self.conf = conf
        self.running = False

        if not self.cap.isOpened():
            raise RuntimeError("Camera tidak bisa dibuka")

    def start(self):
        self.running = True
        t = threading.Thread(target=self._loop, daemon=True)
        t.start()

    def stop(self):
        self.running = False
        self.cap.release()

    def _loop(self):
        while self.running:
            if gv.vision_mode != gv.VISION_BASKET:
                gv.basket_detected = False
                time.sleep(0.05)
                continue                      

            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            h, w, _ = frame.shape
            cx_frame = w // 2
            cy_frame = h // 2

            results = self.model(frame, conf=self.conf, verbose=False)

            detected = False

            for r in results:
                if r.boxes is None:
                    continue

                for box in r.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx_obj = (x1 + x2) // 2
                    cy_obj = (y1 + y2) // 2

                    # error dinormalisasi (-1 sampai 1)
                    gv.basket_x_error = (cx_obj - cx_frame) / cx_frame
                    gv.basket_y_error = (cy_obj - cy_frame) / cy_frame

                    gv.basket_detected = True
                    detected = True
                    break

                if detected:
                    break

            if not detected:
                gv.basket_detected = False

            time.sleep(0.05)  # ~20 Hz