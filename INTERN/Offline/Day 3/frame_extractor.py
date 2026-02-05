import cv2
import os
import numpy as np

def extract_n_frames(video_path, num_frames):
    output_folder = "dataset"
    os.makedirs(output_folder, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    if total_frames == 0:
        print("Video tidak bisa dibaca!")
        return

    # Indeks frame yang diambil merata
    frame_indices = np.linspace(
        0, total_frames - 1, num_frames, dtype=int
    )

    for i, idx in enumerate(frame_indices):
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()
        if ret:
            filename = os.path.join(
                output_folder, f"frame_{i:03d}.jpg"
            )
            cv2.imwrite(filename, frame)

    cap.release()
    print(f"Selesai! {num_frames} frame disimpan di folder 'dataset'")

# =====================
# CONTOH PAKAI
# =====================
video_path = "C:/Users/visco/Downloads/WhatsApp Video 2026-02-05 at 19.21.18.mp4"
jumlah_frame = 150

extract_n_frames(video_path, jumlah_frame)
