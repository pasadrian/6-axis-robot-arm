import numpy as np
import matplotlib.pyplot as plt
from math import atan2, sqrt, acos, degrees, radians, sin, cos

# =====================================================================
# CHOOSE MODE: 'LINE' lub 'CIRCLE'
# =====================================================================
MODE = 'LINE'  # Zmień na 'CIRCLE' aby uruchomić wersję z półokręgiem
# =====================================================================

class RobotMaster:
    def __init__(self):
        self.d1, self.a2, self.a3, self.d6 = 187.3, 335.6, 228.6, 211.5
        self.limits = {
            'q1': (-90.0, 90.0), 'q2': (0.0, 135.0), 'q3': (-165.0, 0.0),
            'q4': (-90.0, 90.0), 'q5': (-90.0, 90.0)
        }
        self.steps_per_deg = np.array([35.5556 * 2, 28.4444 * 2, 35.5556 * 2, 4.4444 * 2, 17.7778 * 2])
        self.q_start = np.array([-90.0, 135.0, -165.0, 0.0, 90.0])

    def solve_ik(self, x, y, z, roll_d, pitch_d):
        try:
            q1_r = atan2(y, x)
            q1_d = degrees(q1_r)
            pr = radians(pitch_d)
            
            d6_xy, d6_z = self.d6 * cos(pr), self.d6 * sin(pr)
            wx, wy, wz = x - d6_xy * cos(q1_r), y - d6_xy * sin(q1_r), z - d6_z
            
            r, h = sqrt(wx**2 + wy**2), wz - self.d1
            s = sqrt(r**2 + h**2)
            
            cos_q3 = (s**2 - self.a2**2 - self.a3**2) / (2 * self.a2 * self.a3)
            q3_r = -acos(np.clip(cos_q3, -1, 1))
            alpha, beta = atan2(h, r), atan2(self.a3 * sin(abs(q3_r)), self.a2 + self.a3 * cos(q3_r))
            
            q2_d, q3_d = degrees(alpha + beta), degrees(q3_r)
            q5_d = pitch_d - q2_d - q3_d
            return np.array([q1_d, q2_d, q3_d, roll_d, q5_d])
        except: return None

    def get_joint_positions(self, q_deg):
        q = np.radians(q_deg)
        p0, p1 = np.array([0, 0, 0]), np.array([0, 0, self.d1])
        p2 = p1 + np.array([self.a2*cos(q[1])*cos(q[0]), self.a2*cos(q[1])*sin(q[0]), self.a2*sin(q[1])])
        p3 = p2 + np.array([self.a3*cos(q[1]+q[2])*cos(q[0]), self.a3*cos(q[1]+q[2])*sin(q[0]), self.a3*sin(q[1]+q[2])])
        
        v_forward = (p3 - p2) / self.a3
        v_up = np.array([0, 0, 1])
        v_side = np.cross(v_up, v_forward)
        if np.linalg.norm(v_side) < 1e-6: v_side = np.array([0, 1, 0])
        v_side /= np.linalg.norm(v_side)
        v_true_up = np.cross(v_forward, v_side)
        
        q4_r, q5_r = q[3], q[4]
        dir_d6 = v_forward * cos(q5_r) + (v_true_up * cos(q4_r) + v_side * sin(q4_r)) * sin(q5_r)
        p4 = p3 + dir_d6 * self.d6
        
        return [p0, p1, p2, p3, p4]

# --- Generowanie Trajektorii ---
robot = RobotMaster()
N_steps = 5  # Liczba wyświetlanych kroków robota
t_vals = np.linspace(0, 1, N_steps)

# Punkty startowe i końcowe
X_start, X_end = 400.0, 600.0
Y_val, Z_val = 0.0, 0.0
pitch, roll = -60.0, 0.0

path_points = []

if MODE == 'LINE':
    # Trajektoria liniowa: X rośnie od 300 do 500, Y i Z są stałe (0)
    for t in t_vals:
        x = X_start + (X_end - X_start) * t
        path_points.append((x, Y_val, Z_val))
    title_suffix = "LINEAR PATH"

elif MODE == 'CIRCLE':
    # Trajektoria półokręgu w płaszczyźnie XY (lub XZ jeśli wolisz, domyślnie XY pionowy lub poziomy). 
    # Tutaj robimy półokrąg w płaszczyźnie XZ (ruch "łukiem w górę" nad osią X), aby ruch był ładnie widoczny.
    # Środek okręgu znajduje się w punkcie (X_start + X_end)/2 = 400. Promień R = 100.
    R = (X_end - X_start) / 2.0
    cx = (X_start + X_end) / 2.0
    cy = Y_val
    cz = Z_val
    
    for t in t_vals:
        # Kąt od 180 stopni (dla X=300) do 0 stopni (dla X=500)
        angle = radians(180 - 180 * t) 
        x = cx + R * cos(angle)
        y = cy
        z = cz + R * sin(angle)  # Łuk unosi się w osi Z
        path_points.append((x, y, z))
    title_suffix = "SEMICIRCULAR PATH (IN X-Z PLANE)"

# Gęsta ścieżka do narysowania linii trajektorii na wykresie
dense_path = []
for t in np.linspace(0, 1, 100):
    if MODE == 'LINE':
        dense_path.append((X_start + (X_end - X_start) * t, Y_val, Z_val))
    elif MODE == 'CIRCLE':
        R = (X_end - X_start) / 2.0
        cx = (X_start + X_end) / 2.0
        angle = radians(180 - 180 * t)
        dense_path.append((cx + R * cos(angle), Y_val, Z_val + R * sin(angle)))
dense_path = np.array(dense_path)

# --- Przygotowanie Wykresów ---
fig = plt.figure(figsize=(10, 7))
fig.suptitle(f"Robot Trajectory Simulation - {title_suffix}", fontsize=16, fontweight='bold')

ax3d = fig.add_subplot(221, projection='3d')
ax_xy = fig.add_subplot(222)
ax_xz = fig.add_subplot(223)
ax_yz = fig.add_subplot(224)

axes_2d = [ax_xy, ax_xz, ax_yz]

# Rysowanie ścieżki (czerwona przerywana linia)
ax3d.plot(dense_path[:, 0], dense_path[:, 1], dense_path[:, 2], 'r--', linewidth=2, label='TCP Trajectory')
ax_xy.plot(dense_path[:, 0], dense_path[:, 1], 'r--', linewidth=2)
ax_xz.plot(dense_path[:, 0], dense_path[:, 2], 'r--', linewidth=2)
ax_yz.plot(dense_path[:, 1], dense_path[:, 2], 'r--', linewidth=2)

# Pętla rysująca 7 kroków robota
for idx, (x, y, z) in enumerate(path_points):
    q_angles = robot.solve_ik(x, y, z, roll, pitch)
    
    if q_angles is not None:
        pts = robot.get_joint_positions(q_angles)
        
        # Stopniowanie przezroczystości (alpha) od 0.2 (start) do 1.0 (koniec)
        alpha = 0.2 + 0.8 * (idx / (N_steps - 1))
        
        # Ostatni krok zaznaczamy mocniejszymi kolorami, pośrednie są szare/wyblakłe
        if idx == 0:
            colors = ['#2ca02c', '#2ca02c', '#2ca02c', '#2ca02c'] # Zielony dla pozycji startowej
            lw = 3
        elif idx == N_steps - 1:
            colors = ['#1f77b4', '#d62728', '#9467bd', '#bc5090'] # Żywe kolory dla pozycji końcowej
            lw = 4
        else:
            colors = ['#7f7f7f', '#7f7f7f', '#7f7f7f', '#7f7f7f'] # Szary dla stanów pośrednich
            lw = 2
            
        # Rysowanie członów robota we wszystkich rzutach
        for i in range(4):
            # Widok 3D
            ax3d.plot([pts[i][0], pts[i+1][0]], [pts[i][1], pts[i+1][1]], [pts[i][2], pts[i+1][2]], 
                      linewidth=lw, marker='o', color=colors[i], alpha=alpha)
            # Rzut X-Y
            ax_xy.plot([pts[i][0], pts[i+1][0]], [pts[i][1], pts[i+1][1]], 
                       linewidth=lw, marker='o', color=colors[i], alpha=alpha)
            # Rzut X-Z
            ax_xz.plot([pts[i][0], pts[i+1][0]], [pts[i][2], pts[i+1][2]], 
                       linewidth=lw, marker='o', color=colors[i], alpha=alpha)
            # Rzut Y-Z
            ax_yz.plot([pts[i][1], pts[i+1][1]], [pts[i][2], pts[i+1][2]], 
                       linewidth=lw, marker='o', color=colors[i], alpha=alpha)
            
        # Zaznaczenie punktu TCP (końcówki)
        ax3d.scatter(pts[4][0], pts[4][1], pts[4][2], color='red', s=40, alpha=alpha)
        ax_xy.scatter(pts[4][0], pts[4][1], color='red', s=40, alpha=alpha)
        ax_xz.scatter(pts[4][0], pts[4][2], color='red', s=40, alpha=alpha)
        ax_yz.scatter(pts[4][1], pts[4][2], color='red', s=40, alpha=alpha)

# --- Chart Settings and Limits ---
ax3d.set_title("3D VIEW")
ax_xy.set_title("TOP VIEW (X-Y)"); ax_xy.set_xlabel("X [mm]"); ax_xy.set_ylabel("Y [mm]")
ax_xz.set_title("SIDE VIEW (X-Z)"); ax_xz.set_xlabel("X [mm]"); ax_xz.set_ylabel("Z [mm]")
ax_yz.set_title("FRONT VIEW (Y-Z)"); ax_yz.set_xlabel("Y [mm]"); ax_yz.set_ylabel("Z [mm]")

# Dostosowanie zakresu osi, aby całe ramię i ruch były dobrze widoczne
for ax in [ax3d, ax_xy, ax_xz]:
    ax.set_xlim([-100, 700])
ax3d.set_ylim([-400, 400]); ax3d.set_zlim([0, 850])
ax_xy.set_ylim([-400, 400]); ax_xy.grid(True); ax_xy.set_aspect('equal', 'box')
ax_xz.set_ylim([0, 850]); ax_xz.grid(True); ax_xz.set_aspect('equal', 'box')
ax_yz.set_xlim([-400, 400]); ax_yz.set_ylim([0, 850]); ax_yz.grid(True); ax_yz.set_aspect('equal', 'box')

ax3d.legend(loc='upper left')
plt.tight_layout()
plt.show()