import cv2
import cv2.aruco as aruco
import numpy as np

# --- KONFIGURACJA ARUCO ---
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
ARUCO_PARAMS = aruco.DetectorParameters()
MARKER_SIZE_MM = 200  # Twoje 20x20 cm
TARGET_ID = 11

camera_matrix = np.array([
    [622.85932525,   0.0        , 334.20613867],
    [0.0        , 623.7371788 , 254.85975734],
    [0.0        ,   0.0        ,   1.0        ]
], dtype=np.float32)

dist_coeffs = np.array([-0.2800055 , -0.1592228 ,  0.00144821, -0.00373484,  0.28514275], dtype=np.float32)

cap = cv2.VideoCapture(1)

while True:
    ret, frame = cap.read()
    if not ret: break

    # 1. Tu powinieneś użyć swojego kodu do prostowania obrazu (undistort)
    frame = cv2.undistort(frame, camera_matrix, dist_coeffs)

    # 2. Wykrywanie ArUco
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, ARUCO_DICT, parameters=ARUCO_PARAMS)

    if ids is not None and TARGET_ID in ids:
        # Znajdź indeks naszego znacznika nr 11
        idx = np.where(ids == TARGET_ID)[0][0]
        marker_corners = corners[idx][0]

        # Oblicz środek znacznika w pikselach
        center_x = int(np.mean(marker_corners[:, 0]))
        center_y = int(np.mean(marker_corners[:, 1]))

        # Oblicz skalę: szerokość znacznika w pikselach
        # (Dystans między lewym a prawym górnym rogiem)
        width_px = np.linalg.norm(marker_corners[0] - marker_corners[1])
        pixel_per_mm = width_px / MARKER_SIZE_MM

        # Rysowanie
        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
        aruco.drawDetectedMarkers(frame, corners, ids)
        
        cv2.putText(frame, f"Srodek ArUco: {center_x}, {center_y}", (10, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"Skala: {pixel_per_mm:.2f} px/mm", (10, 80), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Kalibracja ArUco", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()