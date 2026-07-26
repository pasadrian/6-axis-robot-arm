import cv2
import os

# --- KONFIGURACJA ---
folder_name = "zdjecia_kalibracja"
device_index = 1  # Zmień na 1, jeśli otwiera się zła kamera
# ---------------------

# Utwórz folder, jeśli nie istnieje
if not os.path.exists(folder_name):
    os.makedirs(folder_name)
    print(f"Utworzono folder: {folder_name}")

cap = cv2.VideoCapture(device_index)
count = 0

print("INSTRUKCJA:")
print("- SPACJA: Zrób zdjęcie")
print("- Q: Zakończ i zamknij")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Wyświetlamy kopię z licznikiem, żeby nie psuć zapisanego pliku tekstem
    display_frame = frame.copy()
    cv2.putText(display_frame, f"Zdjecia: {count}", (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.imshow('Przygotowanie do kalibracji', display_frame)

    key = cv2.waitKey(1) & 0xFF

    # Zapisywanie na spację
    if key == ord(' '):
        file_path = os.path.join(folder_name, f"calib_{count:02d}.jpg")
        cv2.imwrite(file_path, frame)
        print(f"Zapisano: {file_path}")
        count += 1

    # Wyjście na 'q'
    elif key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"\nGotowe! Zebrałeś {count} zdjęć w folderze '{folder_name}'.")