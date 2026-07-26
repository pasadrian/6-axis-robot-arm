import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import TextBox
from math import atan2, sqrt, acos, degrees, radians, sin, cos

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

# --- Setup GUI ---
robot = RobotMaster()
fig = plt.figure(figsize=(10, 7))

# Automatycznie układa wykresy, zapobiegając nakładaniu się etykiet osi i tytułów
fig.tight_layout(rect=[0, 0.15, 1, 0.95]) 

# Subplots_adjust precyzuje marginesy (zostawiamy dół na poziomie 0.22 na widgety i info panel)
plt.subplots_adjust(bottom=0.22, left=0.1, right=0.95, top=0.9, hspace=0.35, wspace=0.25)

# Tworzenie siatki widoków
ax3d = fig.add_subplot(221, projection='3d') # Widok 3D
ax_xy = fig.add_subplot(222)                 # Rzut z góry (X-Y)
ax_xz = fig.add_subplot(223)                 # Rzut z boku (X-Z)
ax_yz = fig.add_subplot(224)                 # Rzut z przodu (Y-Z)

axes_2d = [ax_xy, ax_xz, ax_yz]
info_panel = fig.text(0.02, 0.02, "", family='monospace', fontsize=9, bbox=dict(facecolor='white', alpha=0.8))

inputs = {'X': '400', 'Y': '200', 'Z': '300', 'Pitch': '-60', 'Roll': '0'}
text_boxes = {}

def draw_robot(target_params):
    try:
        x, y, z = float(target_params['X']), float(target_params['Y']), float(target_params['Z'])
        p, r = float(target_params['Pitch']), float(target_params['Roll'])
        q_angles = robot.solve_ik(x, y, z, r, p)
        
        # Czyszczenie wszystkich osi
        ax3d.clear()
        for ax in axes_2d: ax.clear()
        
        if q_angles is not None:
            low_q = np.array([robot.limits[k][0] for k in ['q1','q2','q3','q4','q5']])
            high_q = np.array([robot.limits[k][1] for k in ['q1','q2','q3','q4','q5']])
            is_safe = np.all((q_angles >= low_q) & (q_angles <= high_q))
            
            pts = robot.get_joint_positions(q_angles)
            steps = (q_angles - robot.q_start) * robot.steps_per_deg
            colors = ['black', 'red', 'green', 'blue'] if is_safe else ['grey']*4
            
            # Rysowanie w 3D i 2D
            for i in range(4):
                # 3D
                ax3d.plot([pts[i][0], pts[i+1][0]], [pts[i][1], pts[i+1][1]], [pts[i][2], pts[i+1][2]], 
                         linewidth=4, marker='o', color=colors[i])
                # XY
                ax_xy.plot([pts[i][0], pts[i+1][0]], [pts[i][1], pts[i+1][1]], linewidth=3, marker='o', color=colors[i])
                # XZ
                ax_xz.plot([pts[i][0], pts[i+1][0]], [pts[i][2], pts[i+1][2]], linewidth=3, marker='o', color=colors[i])
                # YZ
                ax_yz.plot([pts[i][1], pts[i+1][1]], [pts[i][2], pts[i+1][2]], linewidth=3, marker='o', color=colors[i])

            # --- Chart Settings and Limits ---
            ax3d.set_title("3D VIEW")
            ax_xy.set_title("TOP VIEW (X-Y)"); ax_xy.set_xlabel("X [mm]"); ax_xy.set_ylabel("Y [mm]")
            ax_xz.set_title("SIDE VIEW (X-Z)"); ax_xz.set_xlabel("X [mm]"); ax_xz.set_ylabel("Z [mm]")
            ax_yz.set_title("FRONT VIEW (Y-Z)"); ax_yz.set_xlabel("Y [mm]"); ax_yz.set_ylabel("Z [mm]")

            info_str = (f"STEPS: M1:{int(steps[0])} M2:{int(steps[1])} M3:{int(steps[2])} M4:{int(steps[3])} M5:{int(steps[4])}\n"
                        f"ANGLES: Q1:{q_angles[0]:.1f} Q2:{q_angles[1]:.1f} Q3:{q_angles[2]:.1f} Q4:{q_angles[3]:.1f} Q5:{q_angles[4]:.1f}")
            info_panel.set_text(info_str)
            info_panel.get_bbox_patch().set_facecolor('honeydew' if is_safe else 'mistyrose')

        # Stałe limity osi dla czytelności
        for ax in [ax3d, ax_xy, ax_xz]:
            ax.set_xlim([-600, 600])
        ax3d.set_ylim([-600, 600]); ax3d.set_zlim([0, 800])
        ax_xy.set_ylim([-600, 600]); ax_xy.grid(True)
        ax_xz.set_ylim([0, 800]); ax_xz.grid(True)
        ax_yz.set_xlim([-600, 600]); ax_yz.set_ylim([0, 800]); ax_yz.grid(True)
        
        fig.canvas.draw_idle()
    except Exception as e: print(f"Błąd: {e}")

def submit(text): draw_robot({label: box.text for label, box in text_boxes.items()})

# Pozycjonowanie pól tekstowych na dole
for i, (label, init_val) in enumerate(inputs.items()):
    axbox = plt.axes([0.15 + i*0.16, 0.08, 0.08, 0.04])
    text_box = TextBox(axbox, label + " ", initial=init_val)
    text_box.on_submit(submit); text_boxes[label] = text_box

draw_robot(inputs)
plt.show()