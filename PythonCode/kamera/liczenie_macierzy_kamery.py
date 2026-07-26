import numpy as np
import cv2
import glob

# --- KONFIGURACJA ---
# Liczba WEWNĘTRZNYCH rogów szachownicy (szerokość, wysokość)
# Dla szachownicy 9x7 kwadratów, liczba rogów to (8, 6)
BOARD_SIZE = (9, 7)
# Ścieżka do zdjęć, które zrobiłeś wcześniej
images_path = 'zdjecia_kalibracja/*.jpg'
# ---------------------

# Przygotowanie punktów 3D (0,0,0), (1,0,0), (2,0,0) ... (7,5,0)
objp = np.zeros((BOARD_SIZE[0] * BOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:BOARD_SIZE[0], 0:BOARD_SIZE[1]].T.reshape(-1, 2)

objpoints = [] # Punkty 3D w świecie rzeczywistym
imgpoints = [] # Punkty 2D na obrazie

images = glob.glob(images_path)

print(f"Znaleziono {len(images)} zdjęć. Rozpoczynam analizę...")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Znajdź rogi szachownicy
    ret, corners = cv2.findChessboardCorners(gray, BOARD_SIZE, None)

    if ret:
        objpoints.append(objp)
        # Subpixelowa dokładność (poprawia precyzję)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)
        print(f"V - Rogi wykryte na: {fname}")
    else:
        print(f"X - Nie udało się znaleźć rogów na: {fname}")

if len(objpoints) > 0:
    # GŁÓWNA KALIBRACJA
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("\n--- SUKCES! TWOJE PARAMETRY ---")
    print("\nMacierz kamery (mtx):")
    print(np.array2string(mtx, separator=', '))
    print("\nWspółczynniki dystorsji (dist):")
    print(np.array2string(dist, separator=', '))
    print("\n-------------------------------")
    print("Skopiuj te wartości do swojego głównego skryptu.")
else:
    print("Błąd: Nie wykryto szachownicy na żadnym ze zdjęć. Sprawdź parametry BOARD_SIZE.")