import cv2
import mediapipe as mp

mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0)

def get_finger_count():
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)

    if not ret:
        return 

    # Ubah BGR ke RGB
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    result = hands.process(rgb)

    finger_count = 0

    if result.multi_hand_landmarks:
        for hand in result.multi_hand_landmarks:

            lm = hand.landmark

            # Jempol
            if lm[4].x < lm[3].x:
                finger_count += 1

            # Telunjuk
            if lm[8].y < lm[6].y:
                finger_count += 1

            # Tengah
            if lm[12].y < lm[10].y:
                finger_count += 1

            # Manis
            if lm[16].y < lm[14].y:
                finger_count += 1

            # Kelingking
            if lm[20].y < lm[18].y:
                finger_count += 1

            mp_draw.draw_landmarks(frame, hand, mp_hands.HAND_CONNECTIONS)

    # Tampilkan jumlah jari
    cv2.putText(frame, f"Jari: {finger_count}",
                (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.5, (0, 255, 0), 3)

    cv2.imshow("Deteksi Jari", frame)
    cv2.waitKey(1)

    return finger_count
