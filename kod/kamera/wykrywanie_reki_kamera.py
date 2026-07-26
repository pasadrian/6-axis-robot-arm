import cv2
import mediapipe as mp
import numpy as np

# Konfiguracja MediaPipe
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False, max_num_hands=1, min_detection_confidence=0.7)
mp_draw = mp.solutions.drawing_utils

# Twoje macierze z kalibracji (użyj ich tutaj, aby prostować obraz przed detekcją!)
camera_matrix = np.array([
    [622.85932525,   0.0        , 334.20613867],
    [0.0        , 623.7371788 , 254.85975734],
    [0.0        ,   0.0        ,   1.0        ]
], dtype=np.float32)

dist_coeffs = np.array([-0.2800055 , -0.1592228 ,  0.00144821, -0.00373484,  0.28514275], dtype=np.float32)

cap = cv2.VideoCapture(1)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break

    # 1. Prostowanie obrazu (opcjonalnie, ale zalecane dla precyzji)
    frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

    # 2. Detekcja dłoni
    img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)

    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Punkt nr 9 to środek dłoni (nasada palca środkowego)
            lm = hand_landmarks.landmark[9]
            h, w, _ = frame.shape
            
            # Współrzędne w pikselach
            pixel_x = int(lm.x * w)
            pixel_y = int(lm.y * h)

            # TUTAJ WYKONUJESZ MATEMATYKĘ:
            # robot_x = map_x(pixel_x)
            # robot_y = map_y(pixel_y)
            # send_to_stm(robot_x, robot_y)

            cv2.circle(frame, (pixel_x, pixel_y), 10, (0, 255, 0), cv2.FILLED)
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

    cv2.imshow("Sledzenie dloni", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()