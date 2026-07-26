from fileinput import filename

import cv2
import cv2.aruco as aruco
import mediapipe as mp
import numpy as np

class VisionProcessor:
    def __init__(self):
        # --- PARAMETRY TWOJEJ KAMERY (Wpisz tu swoje dane z kalibracji!) ---
        self.camera_matrix = np.array([
                [622.85932525,   0.0        , 334.20613867],
                [0.0        , 623.7371788 , 254.85975734],
                [0.0        ,   0.0        ,   1.0        ]
                ], dtype=np.float32)

        self.dist_coeffs = np.array([-0.2800055 , -0.1592228 ,  0.00144821, -0.00373484,  0.28514275], 
                       dtype=np.float32)
        
        # Inicjalizacja MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(min_detection_confidence=0.7, max_num_hands=1)
        self.mp_draw = mp.solutions.drawing_utils
        
        # ArUco Config
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.aruco_params = aruco.DetectorParameters()
        
        # Dane do kalibracji ręcznej
        self.calib_points_px = []    # Piksele z kamery
        self.calib_points_robot = [] # Współrzędne z STM (X, Y)
        self.homography_matrix = None
        
        # Zmienne pomocnicze
        self.last_corners = None # Zapamiętuje ostatnio widziane rogi ArUco

        self.locked_corners = None
        self.calibration_done = False

    def lock_aruco_corners(self):
        """Zapisuje aktualnie widziane rogi ArUco na stałe do pamięci"""
        if self.last_corners is not None:
            self.locked_corners = self.last_corners.copy()
            print("✅ Rogi ArUco zostały zablokowane w pamięci!")
            return True
        return False

    def add_manual_point(self, robot_x, robot_y, corner_idx):
        """Używa ZABLOKOWANYCH rogów do kalibracji, nawet jeśli ich nie widać"""
        if self.locked_corners is not None:
            px_point = self.locked_corners[corner_idx]
            self.calib_points_px.append(px_point)
            self.calib_points_robot.append([robot_x, robot_y])
            print(f"Dodano punkt z pamięci: Róg {corner_idx} -> Robot ({robot_x}, {robot_y})")
            
            if len(self.calib_points_px) >= 4:
                self.calculate_homography()
            return True
        return False

    def calculate_homography(self):
        src = np.array(self.calib_points_px, dtype=np.float32)
        dst = np.array(self.calib_points_robot, dtype=np.float32)
        self.homography_matrix, _ = cv2.findHomography(src, dst)
        self.calibration_done = True
        print("!!! KALIBRACJA ZAKOŃCZONA !!!")

    def process_frame(self, frame):
        frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)
        h, w, _ = frame.shape
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Wykrywamy ArUco tylko jeśli NIE mamy jeszcze zablokowanych punktów
        if self.locked_corners is None:
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            if ids is not None and 11 in ids:
                idx = np.where(ids == 11)[0][0]
                self.last_corners = corners[idx][0]
                aruco.drawDetectedMarkers(frame, corners, ids)
        
        # Rysujemy "Duchy" tylko w trakcie kalibracji (po lock, przed końcem 4 punktów)
        if self.locked_corners is not None and not self.calibration_done:
            for i, corner in enumerate(self.locked_corners):
                pt = (int(corner[0]), int(corner[1]))
                cv2.circle(frame, pt, 8, (255, 0, 255), 2) # Różowe kółka
                cv2.putText(frame, f"CEL {i}", (pt[0]+10, pt[1]), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        # 3. Wykrywanie dłoni i przeliczanie
        img_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(img_rgb)
        
        hand_coords = None
        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                lm9 = hand_lms.landmark[9]
                cx, cy = int(lm9.x * w), int(lm9.y * h)
                
                if self.homography_matrix is not None:
                    # PRECYZYJNE PRZELICZENIE PRZEZ HOMOGRAFIĘ
                    point = np.array([[[cx, cy]]], dtype=np.float32)
                    target = cv2.perspectiveTransform(point, self.homography_matrix)
                    hand_coords = (round(target[0][0][0], 2), round(target[0][0][1], 2))
                
                if hand_coords:
                    cv2.putText(frame, f"Robot: X{hand_coords[0]} Y{hand_coords[1]}", (cx, cy-20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.circle(frame, (cx, cy), 10, (0, 255, 0), cv2.FILLED)
        
        return frame, hand_coords
    
    def save_calibration(self, filename="calibration.npy"):
        if self.homography_matrix is not None:
            np.save(filename, self.homography_matrix)
            print("Kalibracja zapisana do pliku.")

    def load_calibration(self, filename="calibration.npy"):
        try:
            self.homography_matrix = np.load(filename)
            print("Kalibracja wczytana pomyślnie.")
            return True
        except:
            print("Nie znaleziono pliku kalibracji.")
            return False