import tkinter as tk
from tkinter import filedialog, ttk
import serial
import time
import threading
import math
from vision_processor import VisionProcessor
import cv2
import numpy as np

# Nowe globalne
vision = VisionProcessor()
vision.load_calibration()
 
tracking_active = False
camera_thread_running = False
busy_tracking = False

current_robot_x = 300.0 
current_robot_y = 0.0
current_robot_z = 120.0
current_pitch = -90.0

latest_hand_target = None
STEP_SIZE = 3.0

# Nowe domyślne punkty dla trajektorii liniowych/okrężnych
path_start_x, path_start_y, path_start_z = 300.0, 0.0, 120.0
path_end_x, path_end_y, path_end_z = 500.0, 0.0, 120.0
# ================== SERIAL ==================

try:
    # Upewnij się, że COM5 jest właściwym portem
    ser = serial.Serial("COM5", 115200, timeout=0)
    time.sleep(2)
except:
    print("BŁĄD: Nie można otworzyć portu COM5")

def send(cmd):
    if 'ser' in globals() and ser.is_open:
        ser.write((cmd + "\n").encode())
        print("WYSŁANO:", cmd)

# ================== MATEMATYKA ROTACJI ==================

def get_rotation_matrix(roll_deg, pitch_deg, yaw_deg):
    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    y = math.radians(yaw_deg)

    Rx = [
        [1, 0, 0],
        [0, math.cos(r), -math.sin(r)],
        [0, math.sin(r), math.cos(r)]
    ]
    Ry = [
        [math.cos(p), 0, math.sin(p)],
        [0, 1, 0],
        [-math.sin(p), 0, math.cos(p)]
    ]
    Rz = [
        [math.cos(y), -math.sin(y), 0],
        [math.sin(y), math.cos(y), 0],
        [0, 0, 1]
    ]

    def mat_mul(A, B):
        C = [[0,0,0],[0,0,0],[0,0,0]]
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    C[i][j] += A[i][k] * B[k][j]
        return C

    R = mat_mul(Rz, mat_mul(Ry, Rx))
    return R

# ================== GLOBALNE ==================

motion_active = False
demo_active = False 

# ================== RUCH BLOKUJĄCY ==================

def start_blocking_motion(cmd, start_text, done_msg):
    global motion_active
    if motion_active:
        return

    motion_active = True
    send(cmd)

    set_controls_state(False)
    status_label.config(text=start_text, bg="red")

    threading.Thread(
        target=serial_wait_for,
        args=(done_msg,),
        daemon=True
    ).start()

def serial_wait_for(done_msg):
    buffer = ""
    while True:
        try:
            if ser.in_waiting > 0:
                new_data = ser.read(ser.in_waiting).decode(errors="ignore")
                buffer += new_data
                
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    
                    if line:
                        print("ODPOWIEDŹ STM32:", line) 
                    
                    if line == done_msg:
                        root.after(0, finish_motion)
                        return
        except Exception as e:
            print(f"Błąd odczytu serial: {e}")
        time.sleep(0.01)

def finish_motion():
    global motion_active, busy_tracking
    motion_active = False
    busy_tracking = False
    set_controls_state(True)
    status_label.config(text="🟢 GOTOWE", bg="green")

def hand_tracking_thread():
    global tracking_active, latest_hand_target, camera_thread_running
    
    cap = cv2.VideoCapture(1)
    
    # --- WEWNĘTRZNA FUNKCJA KLIKNIĘCIA ---
    def on_mouse_click(event, x, y, flags, param):
        global latest_hand_target
        if event == cv2.EVENT_LBUTTONDOWN:
            if vision.homography_matrix is not None:
                # Przelicz piksel na współrzędne robota
                point = np.array([[[x, y]]], dtype=np.float32)
                target = cv2.perspectiveTransform(point, vision.homography_matrix)
                rx, ry = target[0][0][0], target[0][0][1]
                
                # Ustawiamy to jako nowy cel (nawet jeśli śledzenie jest wyłączone!)
                latest_hand_target = [rx, ry]
                print(f"KLIKNIĘCIE: Nowy cel liniowy -> X:{rx:.1f}, Y:{ry:.1f}")

    # Uruchom proces planisty jako osobny wątek (raz na całe życie programu)
    threading.Thread(target=move_to_target_linear, daemon=True).start()

    # Tworzymy okno i przypisujemy mu obsługę myszy
    cv2.namedWindow("Podglad Kamery - Magisterka")
    cv2.setMouseCallback("Podglad Kamery - Magisterka", on_mouse_click)

    while camera_thread_running:
        ret, frame = cap.read()
        if not ret: break

        frame, coords = vision.process_frame(frame)
        
        # Tylko jeśli śledzenie dłoni jest włączone, aktualizuj cel automatycznie
        if tracking_active and coords:
            rx, ry = coords
            latest_hand_target = [rx, ry]

        cv2.imshow("Podglad Kamery - Magisterka", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'): 
            camera_thread_running = False
            break

    cap.release()
    cv2.destroyAllWindows()

def move_to_target_linear():
    global current_robot_x, current_robot_y, latest_hand_target, busy_tracking
    
    while camera_thread_running:
        if latest_hand_target is None:
            time.sleep(0.01)
            continue
            
        # Pobieramy cel "zamrożony" na tę iterację
        start_x = float(current_robot_x)
        start_y = float(current_robot_y)
        tx, ty = latest_hand_target

        dx = tx - start_x
        dy = ty - start_y
        distance = math.sqrt(dx**2 + dy**2)

        # DEBUG: Wydrukuj co widzi Python, jeśli odrzuca cel
        if distance > 10000.0 or math.isnan(distance):
            print(f"DEBUG DISTANCE: {distance:.1f} | Robot at: ({current_robot_x:.1f}, {current_robot_y:.1f}) | Target at: ({tx:.1f}, {ty:.1f})")
            current_robot_x = tx
            current_robot_y = ty
            latest_hand_target = None
            continue

        # Jeśli jesteśmy blisko celu, czekaj na nowy
        if distance < 5.0:
            latest_hand_target = None
            continue

        num_steps = int(distance / STEP_SIZE)
        if num_steps <= 0: num_steps = 1

        busy_tracking = True
        send("O:0") # Rampa rozpędzania wyłączona
        for i in range(1, num_steps + 1):
            # SPRAWDZENIE: Czy dłoń nie zmieniła pozycji drastycznie?
            if tracking_active and latest_hand_target is not None:
                new_tx, new_ty = latest_hand_target
                # Jeśli nowy cel dłoni jest dalej niż 20mm od tego, do którego aktualnie idziemy
                dist_change = math.sqrt((new_tx - tx)**2 + (new_ty - ty)**2)
                if dist_change > 15.0: 
                    break # Przerwij i przelicz trasę od nowa

            # Wylicz punkt pośredni
            next_x = start_x + (dx * i / num_steps)
            next_y = start_y + (dy * i / num_steps)
            
            cmd = f"G:{next_x:.2f},{next_y:.2f},{current_robot_z:.2f},{current_pitch:.2f},0.00"
            
            # Wyślij i czekaj na IK_DONE
            start_blocking_motion(cmd, "🔵 ŚLEDZENIE...", "IK_DONE")

            while motion_active:
                time.sleep(0.005)
            
            current_robot_x = next_x
            current_robot_y = next_y

        busy_tracking = False
        send("O:1") # Rampa rozpędzania włączona z powrotem 
        if not tracking_active:
            latest_hand_target = None

def sync_and_ready():
    global current_robot_x, current_robot_y, current_robot_z, current_pitch, latest_hand_target
    
    if not vision.load_calibration():
        status_label.config(text="❌ BŁĄD: Brak pliku kalibracji!", bg="red")
        return

    target_x, target_y, target_z = 300.0, 0.0, 120.0
    
    # Aktualizacja pól w GUI, żeby użytkownik widział co się dzieje
    entry_x.delete(0, tk.END); entry_x.insert(0, str(target_x))
    entry_y.delete(0, tk.END); entry_y.insert(0, str(target_y))
    entry_z.delete(0, tk.END); entry_z.insert(0, str(target_z))

    current_robot_x = target_x
    current_robot_y = target_y
    current_robot_z = target_z
    current_pitch = -90.0
    latest_hand_target = None 

    cmd = f"G:{target_x:.2f},{target_y:.2f},{target_z:.2f},-90.00,0.00"
    start_blocking_motion(cmd, "🔄 SYNCHRONIZACJA STARTOWA...", "IK_DONE")
    status_label.config(text="🚀 ZSYNCHRONIZOWANO. Gotowy!", bg="green")

# ================== CALLBACKS ==================
def start_trajectory_path(mode):
    """Pobiera punkty z GUI i uruchamia ruch w osobnym wątku, aby nie zawiesić Tkintera"""
    global motion_active
    if motion_active:
        status_label.config(text="⚠️ BŁĄD: Robot jest zajęty ruchem!", bg="orange")
        return
        
    try:
        # Pobieranie punktów start/end wpisanych przez użytkownika w GUI
        sx = float(entry_p_start_x.get())
        sy = float(entry_p_start_y.get())
        sz = float(entry_p_start_z.get())
        
        ex = float(entry_p_end_x.get())
        ey = float(entry_p_end_y.get())
        ez = float(entry_p_end_z.get())
        
        p_pitch = float(entry_pitch.get())  # Korzystamy z kątów z głównego panelu IK
        p_roll = float(entry_roll.get())
        
        threading.Thread(
            target=execute_path_movement_thread, 
            args=(mode, sx, sy, sz, ex, ey, ez, p_pitch, p_roll), 
            daemon=True
        ).start()
        
    except ValueError:
        status_label.config(text="⚠️ BŁĄD: Wpisz poprawne współrzędne trajektorii!", bg="red")

def execute_path_movement_thread(mode, sx, sy, sz, ex, ey, ez, pitch_val, roll_val):
    """Wątek roboczy dzielący trasę na kroki i wysyłający je po serialu"""
    global current_robot_x, current_robot_y, current_robot_z, current_pitch, motion_active
    
    # 1. Wyznaczenie odległości i liczby kroków podziału
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    linear_distance = math.sqrt(dx**2 + dy**2 + dz**2)
    
    if mode == 'LINE':
        total_distance = linear_distance
    elif mode == 'CIRCLE':
        # Zakładamy półokrąg unoszący się nad linią łączącą punkty o promień R = 1/2 odległości liniowej
        total_distance = math.pi * (linear_distance / 2.0)
        
    # Wyznaczenie liczby kroków na podstawie Twojego STEP_SIZE (3.0 mm)
    num_steps = max(1, int(total_distance / STEP_SIZE))
    
    # Wyłączenie rampy przyspieszenia w STM32 dla płynnego ruchu na łączeniach (tak jak w śledzeniu dłoni)
    send("O:0")
    time.sleep(0.05)
    
    for i in range(num_steps + 1):
        t = i / num_steps
        
        if mode == 'LINE':
            # Prosta interpolacja liniowa w 3D
            next_x = sx + dx * t
            next_y = sy + dy * t
            next_z = sz + dz * t
        elif mode == 'CIRCLE':
            # Ruch po półokręgu (łuk wznoszący się w pionie w osi Z)
            # Interpolacja bazy liniowej (X, Y)
            base_x = sx + dx * t
            base_y = sy + dy * t
            base_z = sz + dz * t
            
            # Dodanie łuku (funkcja sinus) unoszącego się nad bazową linią Z
            R = linear_distance / 2.0
            arc_offset = R * math.sin(math.radians(180.0 * t))
            
            next_x = base_x
            next_y = base_y
            next_z = base_z + arc_offset
            
        cmd = f"G:{next_x:.2f},{next_y:.2f},{next_z:.2f},{pitch_val:.2f},{roll_val:.2f}"
        
        # Wyślij krok i zablokuj wątek czekając na IK_DONE
        start_blocking_motion(cmd, f"🔵 RUCH {mode} ({i}/{num_steps}): X:{next_x:.1f} Y:{next_y:.1f} Z:{next_z:.1f}", "IK_DONE")
        
        while motion_active:
            time.sleep(0.005)
            
        # Aktualizacja pozycji w pamięci podręcznej po każdym kroku
        current_robot_x = next_x
        current_robot_y = next_y
        current_robot_z = next_z
        current_pitch = pitch_val
        
    # Przywrócenie rampy po zakończeniu całej trajektorii
    send("O:1")
    root.after(0, lambda: status_label.config(text="🟢 TRAJEKTORIA UKOŃCZONA", bg="green"))


def toggle_demo():
    """Nowa uproszczona funkcja Demo - wysyła tylko komendę typu DEMO:X lub STOP"""
    global demo_active
    
    if not demo_active:
        selected_axis = combo_demo_type.get()[0] # Bierze P, X, Y lub Z
        demo_active = True
        btn_demo.config(text="STOP DEMO", bg="red")
        status_label.config(text=f"🟣 TRYB DEMO {selected_axis} AKTYWNY", bg="purple")
        send(f"DEMO:{selected_axis}")
        set_controls_state(False)
    else:
        demo_active = False
        btn_demo.config(text="URUCHOM DEMO", bg="magenta")
        status_label.config(text="🟢 GOTOWE", bg="green")
        send("STOP")
        set_controls_state(True)

def execute_ik_move():
    global current_robot_x, current_robot_y, current_robot_z, current_pitch
    try:
        x = float(entry_x.get())
        y = float(entry_y.get())
        z = float(entry_z.get())
        roll = float(entry_roll.get())
        pitch = float(entry_pitch.get())

        cmd = f"G:{x:.2f},{y:.2f},{z:.2f},{pitch:.2f},{roll:.2f}"
        start_blocking_motion(cmd, f"🔵 RUCH IK (X:{x} Y:{y} Z:{z})...", "IK_DONE")

        current_robot_x = x
        current_robot_y = y
        current_robot_z = z
        current_pitch = pitch

    except ValueError:
        status_label.config(text="⚠️ BŁĄD: Wpisz poprawne liczby!", bg="orange")

def make_speed_entry(motor, entry_widget):
    def handler(event=None):
        v = entry_widget.get()
        if v.isdigit():
            send(f"M{motor}V{v}")
    return handler

def make_press_cmd(motor, direction):
    def handler(event):
        if not motion_active and not demo_active:
            send(f"M{motor}{direction}")
    return handler

def make_release_cmd(motor):
    def handler(event):
        if not motion_active and not demo_active:
            send(f"M{motor}S")
    return handler

def execute_relative_move():
    try:
        motor = entry_rel_motor.get()
        steps = entry_rel_steps.get()
        if motor.isdigit() and (steps.startswith('-') or steps.isdigit()):
            cmd = f"R{motor}:{steps}"
            start_blocking_motion(cmd, f"🟡 RUCH RELATYWNY (M{motor} o {steps} kroków)", "REL_DONE")
    except ValueError:
        status_label.config(text="⚠️ BŁĄD: Podaj poprawne dane!", bg="orange")

def toggle_tracking():
    global tracking_active
    if not tracking_active:
        tracking_active = True
        btn_track.config(text="STOP ŚLEDZENIA", bg="red")
        status_label.config(text="🔵 ŚLEDZENIE DŁONI AKTYWNE", bg="blue")
    else:
        tracking_active = False
        btn_track.config(text="URUCHOM ŚLEDZENIE", bg="cyan")
        status_label.config(text="🟢 GOTOWE", bg="green")

def start_camera():
    global camera_thread_running
    if not camera_thread_running:
        camera_thread_running = True
        threading.Thread(target=hand_tracking_thread, daemon=True).start()

def calib_corner(idx):
    global current_robot_x, current_robot_y, current_robot_z, current_pitch
    try:
        # Pobieramy X i Y z Twoich istniejących pól Entry
        rx = float(entry_x.get())
        ry = float(entry_y.get())
        
        # Przekazujemy dane do obiektu vision
        success = vision.add_manual_point(rx, ry, idx)
        
        if success:
            status_label.config(text=f"✅ Zapisano róg {idx} (Robot X:{rx} Y:{ry})", bg="orange")

            # --- AUTOMATYCZNE PODNIESIENIE PO OSTATNIM ROGU ---
            if idx == 3:
                print("Ostatni róg zapisany. Podnoszę na Z=120...")
                # Aktualizujemy obecną pozycję (tę z pól Entry, gdzie robot jeszcze stoi)
                current_robot_x = rx
                current_robot_y = ry
                current_robot_z = 25.0 # Zakładamy, że kalibrujesz na tej wysokości
                
                # Komenda podniesienia
                cmd = f"G:{rx:.2f},{ry:.2f},120.00,-90.00,0.00"
                start_blocking_motion(cmd, "⬆ KALIBRACJA OK - PODNOSZENIE...", "IK_DONE")
                
                # Aktualizujemy wysokość w zmiennych globalnych
                current_robot_z = 120.0

                vision.save_calibration()
        else:
            status_label.config(text="❌ BŁĄD: Kamera nie widzi ArUco!", bg="red")
    except ValueError:
        status_label.config(text="⚠️ BŁĄD: Wpisz liczby w pola X i Y!", bg="red")

def lock_aruco():
    if vision.lock_aruco_corners():
        status_label.config(text="🔒 Pozycja ArUco zapisana! Możesz zasłonić znacznik.", bg="blue")
    else:
        status_label.config(text="❌ BŁĄD: Kamera nie widzi ArUco - nie mogę zablokować!", bg="red")

# ================== GUI ==================

root = tk.Tk()
root.title("Sterowanie robotem 6-osiowym – STM32H7")

main_frame = tk.Frame(root)
main_frame.pack(padx=10, pady=10)

control_widgets = []

def set_controls_state(state):
    for w in control_widgets:
        if w != btn_demo:
            try:
                w.config(state=tk.NORMAL if state else tk.DISABLED)
            except:
                pass

# ================== WIZJA I ŚLEDZENIE (POD AKCESORIAMI) ==================
vision_frame = tk.LabelFrame(main_frame, text="System Wizyjny i Kalibracja", padx=10, pady=10, fg="darkgreen")
# Ustawiamy row=3, dzięki czemu wskoczy idealnie pod Akcesoria
vision_frame.grid(row=3, column=2, rowspan=6, padx=20, sticky="new") 

# Przyciski sterowania wizją
btn_cam = tk.Button(vision_frame, text="ODPAL KAMERĘ", bg="lightgray", command=start_camera)
btn_cam.grid(row=0, column=0, columnspan=2, sticky="ew", pady=2)

btn_lock = tk.Button(vision_frame, text="ZABLOKUJ POZYCJĘ ARUCO", bg="yellow", command=lock_aruco)
btn_lock.grid(row=1, column=0, columnspan=2, sticky="ew", pady=2)

btn_track = tk.Button(vision_frame, text="ŚLEDŹ DŁOŃ", bg="cyan", font=("Arial", 10, "bold"), command=toggle_tracking)
btn_track.grid(row=2, column=0, columnspan=2, sticky="ew", pady=5)
control_widgets.append(btn_track)

# Sekcja kalibracji
tk.Label(vision_frame, text="--- KALIBRACJA ARUCO ---", font=("Arial", 8, "bold")).grid(row=3, column=0, columnspan=2, pady=(10,0))

for i in range(4):
    b = tk.Button(vision_frame, text=f"Róg {i}", width=10, command=lambda idx=i: calib_corner(idx))
    b.grid(row=4 + (i // 2), column=i % 2, padx=2, pady=2)
    control_widgets.append(b)

btn_sync = tk.Button(vision_frame, text="SYNC & START (Z:120)", bg="green", fg="white", 
                     font=("Arial", 9, "bold"), command=sync_and_ready)
btn_sync.grid(row=6, column=0, columnspan=2, sticky="ew", pady=10)

# ================== STATUS ==================

status_label = tk.Label(
    main_frame, text="🟢 GOTOWE",
    bg="green", fg="white",
    font=("Arial", 12),
    width=60, height=2
)
status_label.grid(row=0, column=0, columnspan=4, pady=10)

# ================== KINEMATYKA ODWROTNA (IK) ==================

ik_frame = tk.LabelFrame(main_frame, text="Kinematyka Odwrotna (Cel XYZ + Euler)", padx=10, pady=10, fg="blue")
ik_frame.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=5)

ik_inputs = [
    ("X:", "200"), ("Y:", "0"), ("Z:", "100"),
    ("Roll (A):", "0"), ("Pitch (B):", "-90"), ("Yaw (C):", "0")
]

entries = []
for i, (label, default) in enumerate(ik_inputs):
    tk.Label(ik_frame, text=label).grid(row=i//3, column=(i%3)*2, sticky="e", padx=2)
    ent = tk.Entry(ik_frame, width=8)
    ent.insert(0, default)
    ent.grid(row=i//3, column=(i%3)*2+1, padx=5, pady=2)
    entries.append(ent)

(entry_x, entry_y, entry_z, entry_roll, entry_pitch, entry_yaw) = entries

btn_ik_go = tk.Button(ik_frame, text="WYKONAJ RUCH IK", bg="blue", fg="white", font=("Arial", 10, "bold"),
                      command=execute_ik_move)
btn_ik_go.grid(row=2, column=0, columnspan=6, sticky="ew", pady=10)
control_widgets.append(btn_ik_go)

# ================== NOWA SEKCJA DEMO ==================

demo_frame = tk.LabelFrame(main_frame, text="Konfiguracja Trybu DEMO", padx=10, pady=10, fg="magenta")
demo_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=5)

tk.Label(demo_frame, text="Wybierz oś:").grid(row=0, column=0)
combo_demo_type = ttk.Combobox(demo_frame, values=["Pitch", "X", "Y", "Z"], width=10, state="readonly")
combo_demo_type.set("Pitch")
combo_demo_type.grid(row=0, column=1, padx=10)

btn_demo = tk.Button(demo_frame, text="URUCHOM DEMO", bg="magenta", fg="white", font=("Arial", 10, "bold"),
                      command=toggle_demo)
btn_demo.grid(row=0, column=2, padx=10, ipadx=10)

# --- PRZYCISK HOME ---
btn_home = tk.Button(main_frame, text="BAZOWANIE (HOME)", bg="orange", height=2, font=("Arial", 10, "bold"),
                      command=lambda: start_blocking_motion("HOME", "🔴 TRWA BAZOWANIE...", "HOMING_DONE"))
btn_home.grid(row=3, column=0, columnspan=2, sticky="ew", pady=10)
control_widgets.append(btn_home)

# ================== SEKCJA TRAJEKTORII (LINIA / PÓŁOKRĄG) ==================

path_frame = tk.LabelFrame(main_frame, text="Trajektoria Ruchu (Linia / Półokrąg)", padx=10, pady=10, fg="darkblue")
# Umieszczamy ramkę w kolumnie 0, rząd 13 (zaraz pod "Debugowanie Kroków")
path_frame.grid(row=13, column=0, columnspan=2, sticky="ew", pady=5)

# Nagłówki
tk.Label(path_frame, text="Współrzędne:", font=("Arial", 8, "bold")).grid(row=0, column=0, sticky="w")
tk.Label(path_frame, text="X").grid(row=0, column=1)
tk.Label(path_frame, text="Y").grid(row=0, column=2)
tk.Label(path_frame, text="Z").grid(row=0, column=3)

# Punkt początkowy (Start Point)
tk.Label(path_frame, text="START:").grid(row=1, column=0, sticky="e")
entry_p_start_x = tk.Entry(path_frame, width=8)
entry_p_start_x.insert(0, "300.0")
entry_p_start_x.grid(row=1, column=1, padx=2, pady=2)

entry_p_start_y = tk.Entry(path_frame, width=8)
entry_p_start_y.insert(0, "0.0")
entry_p_start_y.grid(row=1, column=2, padx=2, pady=2)

entry_p_start_z = tk.Entry(path_frame, width=8)
entry_p_start_z.insert(0, "120.0")
entry_p_start_z.grid(row=1, column=3, padx=2, pady=2)

# Punkt końcowy (End Point)
tk.Label(path_frame, text="END:").grid(row=2, column=0, sticky="e")
entry_p_end_x = tk.Entry(path_frame, width=8)
entry_p_end_x.insert(0, "500.0")
entry_p_end_x.grid(row=2, column=1, padx=2, pady=2)

entry_p_end_y = tk.Entry(path_frame, width=8)
entry_p_end_y.insert(0, "0.0")
entry_p_end_y.grid(row=2, column=2, padx=2, pady=2)

entry_p_end_z = tk.Entry(path_frame, width=8)
entry_p_end_z.insert(0, "120.0")
entry_p_end_z.grid(row=2, column=3, padx=2, pady=2)

# Przyciski uruchamiające ruch
btn_run_line = tk.Button(path_frame, text="RUCH PO LINII", bg="cyan", font=("Arial", 9, "bold"),
                         command=lambda: start_trajectory_path('LINE'))
btn_run_line.grid(row=3, column=0, columnspan=2, sticky="ew", pady=5, padx=2)
control_widgets.append(btn_run_line)

btn_run_circle = tk.Button(path_frame, text="RUCH PO PÓŁOKRĘGU", bg="magenta", fg="white", font=("Arial", 9, "bold"),
                           command=lambda: start_trajectory_path('CIRCLE'))
btn_run_circle.grid(row=3, column=2, columnspan=2, sticky="ew", pady=5, padx=2)
control_widgets.append(btn_run_circle)

# ================== OSIE KROKOWE (MANUAL) ==================

for i in range(1, 6):
    frame = tk.LabelFrame(main_frame, text=f"Oś {i} (Ręcznie)", padx=10, pady=5)
    frame.grid(row=i+3, column=0, columnspan=2, sticky="ew", pady=2)

    bl = tk.Button(frame, text="⬅ LEWO", width=12)
    br = tk.Button(frame, text="PRAWO ➡", width=12)

    bl.grid(row=0, column=0, padx=5)
    br.grid(row=0, column=1, padx=5)

    bl.bind("<ButtonPress>", make_press_cmd(i, "L"))
    bl.bind("<ButtonRelease>", make_release_cmd(i))
    br.bind("<ButtonPress>", make_press_cmd(i, "R"))
    br.bind("<ButtonRelease>", make_release_cmd(i))

    control_widgets.extend([bl, br])

    tk.Label(frame, text="Delay (µs):").grid(row=0, column=2, padx=5)
    e = tk.Entry(frame, width=8)
    if i == 4:
        e.insert(0, "1000")
    else:
        e.insert(0, "300")
    e.grid(row=0, column=3)
    e.bind("<Return>", make_speed_entry(i, e))
    control_widgets.append(e)

# ================== DEBUGOWANIE KROKÓW (RELATYWNE) ==================

rel_frame = tk.LabelFrame(main_frame, text="Debugowanie Kroków (Relatywne)", padx=10, pady=10, fg="orange")
rel_frame.grid(row=12, column=0, columnspan=2, sticky="ew", pady=5)

tk.Label(rel_frame, text="Silnik (1-5):").grid(row=0, column=0)
entry_rel_motor = tk.Entry(rel_frame, width=5)
entry_rel_motor.insert(0, "1")
entry_rel_motor.grid(row=0, column=1, padx=5)

tk.Label(rel_frame, text="Kroki (+/-):").grid(row=0, column=2)
entry_rel_steps = tk.Entry(rel_frame, width=10)
entry_rel_steps.insert(0, "1000")
entry_rel_steps.grid(row=0, column=3, padx=5)

btn_rel_go = tk.Button(rel_frame, text="WYKONAJ KROK", bg="orange", command=execute_relative_move)
btn_rel_go.grid(row=0, column=4, padx=10)
control_widgets.append(btn_rel_go)

# ================== SERWA ==================

servo_frame = tk.LabelFrame(main_frame, text="Akcesoria", padx=10, pady=10)
servo_frame.grid(row=1, column=2, rowspan=2, padx=20, sticky="n")

servo_labels = ["Nadgarstek (Oś 6)", "Chwytak"]
last_servo_value = [None, None]

def make_servo_cmd(id):
    def handler(value):
        v = int(float(value))
        if last_servo_value[id] != v:
            last_servo_value[id] = v
            send(f"S{id+1}:{v}")
    return handler

for i in range(2):
    tk.Label(servo_frame, text=servo_labels[i]).pack(anchor="w")
    s = tk.Scale(
        servo_frame, from_=1000, to=2000,
        orient=tk.HORIZONTAL, length=200, resolution=10,
        command=make_servo_cmd(i)
    )
    if i == 0:
        s.set(2000)
    else:
        s.set(1500)
    s.pack(pady=5)
    control_widgets.append(s)

def keep_alive():
    root.after(100, keep_alive)

keep_alive()
root.mainloop()