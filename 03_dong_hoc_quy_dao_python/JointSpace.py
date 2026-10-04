"""
Quỹ đạo trong không gian khớp của chu trình hàn.

Quy hoạch quỹ đạo LSPB trong không gian Cartesian (Home -> B -> dừng -> A -> dừng
-> Home), sau đó chuyển sang không gian khớp:
    - q     : động học ngược (IK)
    - q_dot : nghịch đảo Jacobian (pseudo-inverse)
    - q_ddot: đạo hàm số của q_dot

Kết quả: đồ thị vị trí, vận tốc, gia tốc của 3 khớp q1, q2, q3.
"""

import numpy as np
import matplotlib.pyplot as plt

# =================================================================
# 1. THÔNG SỐ CƠ KHÍ & HÀM ĐỘNG HỌC (IK / JACOBIAN)
# =================================================================
D1 = 0.2175
A2 = 0.25
D3 = 0.105
D4 = 0.262
DT = 0.01


def inverse_kinematics(x, y, z):
    """
    Động học ngược cho robot RPRR (tương đương cơ cấu SCARA).

    Returns:
        ``(q1, q2, q3)`` ứng với cấu hình elbow-down.
    """
    # Khớp 2: tịnh tiến theo trục Z
    q2 = z - D1 - D4

    # Khớp 3: quay trong mặt phẳng XY (định lý hàm cosin)
    cos_q3 = (x ** 2 + y ** 2 - A2 ** 2 - D3 ** 2) / (2 * A2 * D3)
    cos_q3 = np.clip(cos_q3, -1.0, 1.0)  # tránh lỗi miền xác định khi vượt tầm với
    sin_q3 = np.sqrt(1 - cos_q3 ** 2)    # chọn nghiệm dương
    q3 = np.arctan2(sin_q3, cos_q3)

    # Khớp 1: quay đế
    k1 = A2 + D4 * cos_q3
    k2 = D4 * sin_q3
    q1 = np.arctan2(y, x) - np.arctan2(k2, k1)

    return q1, q2, q3


def calculate_jacobian(q1, q2, q3):
    """
    Jacobian giải tích 3x3, ánh xạ vận tốc khớp q_dot sang vận tốc mũi hàn (vx, vy, vz).

        x = A2·cos(q1) + D4·cos(q1 + q3)
        y = A2·sin(q1) + D4·sin(q1 + q3)
        z = q2 + D1 + D4
    """
    J = np.zeros((3, 3))

    J[0, 0] = -A2 * np.sin(q1) - D4 * np.sin(q1 + q3)  # dx/dq1
    J[0, 1] = 0.0                                      # dx/dq2
    J[0, 2] = -D4 * np.sin(q1 + q3)                    # dx/dq3

    J[1, 0] = A2 * np.cos(q1) + D4 * np.cos(q1 + q3)   # dy/dq1
    J[1, 1] = 0.0                                      # dy/dq2
    J[1, 2] = D4 * np.cos(q1 + q3)                     # dy/dq3

    J[2, 0] = 0.0                                      # dz/dq1
    J[2, 1] = 1.0                                      # dz/dq2 (Z tịnh tiến 1:1 theo q2)
    J[2, 2] = 0.0                                      # dz/dq3

    return J


# =================================================================
# 2. QUY HOẠCH QUỸ ĐẠO HÌNH THANG (LSPB) TRONG KHÔNG GIAN CARTESIAN
# =================================================================
def generate_lspb(P_start, P_end, V_max, A_max, dt):
    """Sinh quỹ đạo vị trí (P), vận tốc (V), gia tốc (A) dạng LSPB trên đoạn thẳng 3D."""
    dist = np.linalg.norm(P_end - P_start)
    if dist < 1e-5:
        return np.array([]), np.empty((0, 3)), np.empty((0, 3)), np.empty((0, 3))

    if V_max ** 2 / A_max > dist:
        V_max = np.sqrt(dist * A_max)

    t_b = V_max / A_max
    t_c = (dist - V_max * t_b) / V_max
    t_f = 2 * t_b + t_c

    t = np.arange(0, t_f, dt)
    s = np.zeros_like(t)
    v_s = np.zeros_like(t)
    a_s = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ti < t_b:
            s[i] = 0.5 * A_max * ti ** 2
            v_s[i] = A_max * ti
            a_s[i] = A_max
        elif ti < t_b + t_c:
            s[i] = 0.5 * A_max * t_b ** 2 + V_max * (ti - t_b)
            v_s[i] = V_max
            a_s[i] = 0.0
        else:
            tau = ti - t_b - t_c
            s[i] = (0.5 * A_max * t_b ** 2 + V_max * t_c) + (V_max * tau - 0.5 * A_max * tau ** 2)
            v_s[i] = V_max - A_max * tau
            a_s[i] = -A_max

    # Ánh xạ quãng đường s(t) sang không gian 3D
    direction = (P_end - P_start) / dist
    P_3D = P_start + s[:, np.newaxis] * direction
    V_3D = v_s[:, np.newaxis] * direction
    A_3D = a_s[:, np.newaxis] * direction

    return t, P_3D, V_3D, A_3D


def generate_dwell(P_stay, duration, dt):
    """Sinh đoạn dừng (dwell) tại một điểm trong khoảng thời gian ``duration``."""
    t = np.arange(0, duration, dt)
    P_3D = np.tile(P_stay, (len(t), 1))
    V_3D = np.zeros_like(P_3D)
    A_3D = np.zeros_like(P_3D)
    return t, P_3D, V_3D, A_3D


# =================================================================
# 3. KỊCH BẢN HÀN: HOME -> B (DỪNG) -> A (DỪNG) -> HOME
# =================================================================
P_Home = np.array([0.287, 0.0, 0.6895])
P_B = np.array([0.2, 0.15, 0.7])
P_A = np.array([0.2, 0.15, 0.5])

# Chặng 1: Home -> B, dừng tại B
t1, p1, v1, a1 = generate_lspb(P_Home, P_B, V_max=0.05, A_max=0.1, dt=DT)
td_b, pd_b, vd_b, ad_b = generate_dwell(P_B, duration=0.5, dt=DT)

# Chặng 2: B -> A, dừng tại A
t2, p2, v2, a2 = generate_lspb(P_B, P_A, V_max=0.01, A_max=0.05, dt=DT)
td_a, pd_a, vd_a, ad_a = generate_dwell(P_A, duration=1.0, dt=DT)

# Chặng 3: A -> Home
t3, p3, v3, a3 = generate_lspb(P_A, P_Home, V_max=0.05, A_max=0.1, dt=DT)

# Ghép chuỗi thời gian nối tiếp
time_blocks = [t1, td_b, t2, td_a, t3]
T_total = []
current_time = 0.0
for tb in time_blocks:
    if len(tb) > 0:
        T_total.extend(tb + current_time)
        current_time += tb[-1] + DT
T_total = np.array(T_total)

P_total = np.vstack([p1, pd_b, p2, pd_a, p3])
V_total = np.vstack([v1, vd_b, v2, vd_a, v3])

# =================================================================
# 4. CHUYỂN SANG KHÔNG GIAN KHỚP
# =================================================================
N_points = len(T_total)
Q = np.zeros((N_points, 3))
Q_dot = np.zeros((N_points, 3))
Q_ddot = np.zeros((N_points, 3))

for i in range(N_points):
    x, y, z = P_total[i]
    vx, vy, vz = V_total[i]

    # Vị trí khớp q từ động học ngược
    q1, q2, q3 = inverse_kinematics(x, y, z)
    Q[i] = [q1, q2, q3]

    # Vận tốc khớp q_dot từ nghịch đảo Jacobian (pseudo-inverse để tránh kỳ dị)
    J = calculate_jacobian(q1, q2, q3)
    try:
        J_inv = np.linalg.pinv(J)
        q_dot = J_inv @ np.array([vx, vy, vz])
        Q_dot[i] = q_dot
    except np.linalg.LinAlgError:
        Q_dot[i] = np.zeros(3)

# Gia tốc khớp q_ddot: đạo hàm số của q_dot
for j in range(3):
    Q_ddot[:, j] = np.gradient(Q_dot[:, j], DT)

# =================================================================
# 5. ĐỒ THỊ VỊ TRÍ - VẬN TỐC - GIA TỐC KHỚP
# =================================================================
fig, axs = plt.subplots(3, 3, figsize=(16, 10))
fig.suptitle(
    "ĐỘNG HỌC KHỚP ROBOT RPRR (ĐÃ TÍCH HỢP DWELL TIME & KHỬ BƯỚC NHẢY BẰNG JACOBIAN)",
    fontsize=14,
    fontweight='bold',
)

titles = ['Khớp 1 (Đế Xoay - Rad)', 'Khớp 2 (Trục Z - M)', 'Khớp 3 (Tay Đòn - Rad)']
colors = ['blue', 'red', 'green']


def draw_dwell_zones(ax):
    """Tô màu các khoảng thời gian dừng tại B và A."""
    start_dwell_b = t1[-1]
    end_dwell_b = start_dwell_b + 0.5
    ax.axvspan(start_dwell_b, end_dwell_b, color='yellow', alpha=0.3)

    start_dwell_a = end_dwell_b + t2[-1]
    end_dwell_a = start_dwell_a + 1.0
    ax.axvspan(start_dwell_a, end_dwell_a, color='orange', alpha=0.3)


for i in range(3):
    # Vị trí q(t)
    axs[0, i].plot(T_total, Q[:, i], color=colors[i], linewidth=2)
    axs[0, i].set_title(titles[i], fontweight='bold')
    axs[0, i].grid(True, linestyle='--')
    draw_dwell_zones(axs[0, i])

    # Vận tốc q_dot(t)
    axs[1, i].plot(T_total, Q_dot[:, i], color=colors[i], linewidth=2)
    axs[1, i].set_ylabel("Vận tốc")
    axs[1, i].grid(True, linestyle='--')
    draw_dwell_zones(axs[1, i])

    # Gia tốc q_ddot(t)
    axs[2, i].plot(T_total, Q_ddot[:, i], color=colors[i], linewidth=2)
    axs[2, i].set_xlabel("Thời gian (s)")
    axs[2, i].set_ylabel("Gia tốc")
    axs[2, i].grid(True, linestyle='--')
    draw_dwell_zones(axs[2, i])

plt.tight_layout()
plt.show()
