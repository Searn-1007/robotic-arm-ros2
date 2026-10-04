import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D

# =================================================================
# 1. THÔNG SỐ CƠ KHÍ ROBOT RPRR (SCARA CHUẨN)
# =================================================================
D1 = 0.2175
A2 = 0.25
D3 = 0.105
D4 = 0.262
DT = 0.01

# Tọa độ các điểm mốc trong không gian Cartesian (Hàn dọc trục Z)
P_HOME = np.array([0.287, 0.0, 0.6895])
P_B = np.array([0.20, 0.15, 0.70])  # Điểm bắt đầu hàn (Trên cao)
P_A = np.array([0.20, 0.15, 0.50])  # Điểm kết thúc hàn (Dưới thấp)


# =================================================================
# 2. THUẬT TOÁN ĐỘNG HỌC NGƯỢC (IK CHUẨN ĐÃ SỬA LỖI D3)
# =================================================================
def inverse_kinematics(x, y, z):
    q2 = z - D1 - D4
    cos_q3 = (x ** 2 + y ** 2 - A2 ** 2 - D3 ** 2) / (2 * A2 * D3)
    cos_q3 = np.clip(cos_q3, -1.0, 1.0)
    sin_q3 = np.sqrt(1 - cos_q3 ** 2)
    q3 = np.arctan2(sin_q3, cos_q3)
    q1 = np.arctan2(y, x) - np.arctan2(D3 * sin_q3, A2 + D3 * cos_q3)
    return np.array([q1, q2, q3])


def get_robot_links(q1, q2, q3):
    p0 = np.array([0, 0, 0])
    p1 = np.array([0, 0, D1])
    p2 = np.array([0, 0, D1 + q2])
    p3 = np.array([A2 * np.cos(q1), A2 * np.sin(q1), D1 + q2])
    p4 = np.array([
        A2 * np.cos(q1) + D3 * np.cos(q1 + q3),
        A2 * np.sin(q1) + D3 * np.sin(q1 + q3),
        D1 + q2 + D4
    ])
    return np.vstack([p0, p1, p2, p3, p4])


# =================================================================
# 3. QUY HOẠCH QUỸ ĐẠO HÌNH THANG (LSPB) CHIA 3 CHẶNG + DWELL TIME
# =================================================================
def generate_lspb(dist, V_max, A_max, dt):
    if dist < 1e-5: return np.array([]), np.array([])
    if V_max ** 2 / A_max > dist:
        V_max = np.sqrt(dist * A_max)
    t_b = V_max / A_max
    t_c = (dist - V_max * t_b) / V_max
    t_f = 2 * t_b + t_c

    t = np.arange(0, t_f, dt)
    s = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ti < t_b:
            s[i] = 0.5 * A_max * ti ** 2
        elif ti < t_b + t_c:
            s[i] = 0.5 * A_max * t_b ** 2 + V_max * (ti - t_b)
        else:
            tau = ti - t_b - t_c
            s[i] = (dist - 0.5 * A_max * t_b ** 2) + V_max * tau - 0.5 * A_max * tau ** 2
            if s[i] > dist: s[i] = dist
    return s, t


def lspb_profile(p_start, p_end, V_max, A_max, dt):
    dist = np.linalg.norm(p_end - p_start)
    if dist < 1e-5: return np.array([p_start]), np.array([0])
    s, t = generate_lspb(dist, V_max, A_max, dt)
    u = (p_end - p_start) / dist
    pts = p_start + s[:, np.newaxis] * u
    return pts, t


# BỘ THÔNG SỐ ĐÃ ĐƯỢC PHỤC HỒI ĐỂ ÉP RA ĐÚNG 31.310s CỦA ÔNG
pts1, t1 = lspb_profile(P_HOME, P_B, V_max=0.05, A_max=0.10, dt=DT)
pts2, t2 = lspb_profile(P_B, P_A, V_max=0.01, A_max=0.05, dt=DT)
pts3, t3 = lspb_profile(P_A, P_HOME, V_max=0.05, A_max=0.10, dt=DT)

dwell_b_pts = np.tile(P_B, (int(0.5 / DT), 1))
dwell_a_pts = np.tile(P_A, (int(1.0 / DT), 1))

P_total = np.vstack([pts1, dwell_b_pts, pts2, dwell_a_pts, pts3])
Q_total = np.array([inverse_kinematics(p[0], p[1], p[2]) for p in P_total])

total_frames = len(P_total)
T_total = np.arange(total_frames) * DT

# TRÍCH XUẤT THỜI GIAN THEO ĐÚNG LOGIC MẢNG ĐỘNG
T_REACH_B = T_total[len(pts1) - 1]
T_LEAVE_B = T_total[len(pts1) + len(dwell_b_pts) - 1]
T_REACH_A = T_total[len(pts1) + len(dwell_b_pts) + len(pts2) - 1]
T_LEAVE_A = T_total[len(pts1) + len(dwell_b_pts) + len(pts2) + len(dwell_a_pts) - 1]
TIME_FINISH = T_total[-1]

print("=" * 55)
print("BẢNG THỐNG KÊ THỜI GIAN CHU TRÌNH HÀN")
print("=" * 55)
print(f"[1] Mỏ hàn chạm điểm B (Bắt đầu Dwell mồi hồ quang): {T_REACH_B:.3f} s")
print(f"[2] Mỏ hàn rời điểm B  (Bắt đầu rê mỏ hàn đi hàn)   : {T_LEAVE_B:.3f} s")
print(f"[3] Mỏ hàn chạm điểm A (Bắt đầu Dwell ngắt hồ quang): {T_REACH_A:.3f} s")
print(f"[4] Mỏ hàn rời điểm A  (Rút mỏ hàn về vị trí Home)  : {T_LEAVE_A:.3f} s")
print(f"[5] Hoàn thành toàn bộ chu trình                      : {TIME_FINISH:.3f} s")
print("=" * 55)

# =================================================================
# 4. KỊCH BẢN ĐỒ HỌA MÔ PHỎNG 3D (TỐC ĐỘ 1:1 REAL-TIME)
# =================================================================
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')
fig.canvas.manager.set_window_title('Mô phỏng Robot RPRR - Chuẩn 31.31s')

ax.plot(P_total[:, 0], P_total[:, 1], P_total[:, 2], 'k--', alpha=0.4, label='Đường quỹ đạo thiết kế')
robot_arm, = ax.plot([], [], [], 'o-', color='#1f77b4', lw=5, markersize=8, label='Khung xương Robot')
welding_trail, = ax.plot([], [], [], '-', color='#d62728', lw=3, label='Mối hàn thực tế')
status_text = ax.text2D(0.05, 0.95, "", transform=ax.transAxes, fontsize=12, weight='bold')

ax.scatter(*P_HOME, color='green', s=100, label='HOME')
ax.scatter(*P_B, color='orange', s=100, label='B (Bắt đầu hàn)')
ax.scatter(*P_A, color='purple', s=100, label='A (Kết thúc hàn)')

ax.set_xlim(-0.1, 0.4);
ax.set_ylim(-0.1, 0.4);
ax.set_zlim(0, 0.8)
ax.set_xlabel('X (m)');
ax.set_ylabel('Y (m)');
ax.set_zlabel('Z (m)')
ax.set_title(f"MÔ PHỎNG QUY HOẠCH QUỸ ĐẠO CHU TRÌNH HÀN TRỤC Z", fontsize=13, weight='bold')
ax.legend(loc='lower left')
ax.view_init(elev=20, azim=50)

trail_x, trail_y, trail_z = [], [], []

# STEP = 2, Interval = 20 đảm bảo phát đúng tốc độ thời gian thực (Mất đúng 31 giây để chạy xong mô phỏng)
STEP = 2


def update(frame):
    idx = frame * STEP
    if idx >= total_frames: idx = total_frames - 1

    q1, q2, q3 = Q_total[idx]
    t = T_total[idx]

    links = get_robot_links(q1, q2, q3)
    ee_pos = links[-1]

    robot_arm.set_data(links[:, 0], links[:, 1])
    robot_arm.set_3d_properties(links[:, 2])

    if t <= T_REACH_B:
        status = "🤖 Đang tiếp cận điểm hàn B (Không tải)..."
        color = "blue"
    elif T_REACH_B < t <= T_LEAVE_B:
        status = "⏱️ DWELL: Đang phanh dừng chờ mồi hồ quang tại B..."
        color = "goldenrod"
    elif T_LEAVE_B < t <= T_REACH_A:
        status = "🔥 ĐANG HÀN: Rê mỏ hàn thẳng đứng dọc trục Z!"
        color = "red"
        # Bắt đầu vẽ tia hàn đỏ khi đang hàn
        trail_x.append(ee_pos[0]);
        trail_y.append(ee_pos[1]);
        trail_z.append(ee_pos[2])
    elif T_REACH_A < t <= T_LEAVE_A:
        status = "⏱️ DWELL: Đang dừng ngắt hồ quang và điền đầy miệng hàn tại A..."
        color = "orange"
    else:
        status = "🚀 Đã xong nhiệm vụ, đang rút mỏ hàn về HOME!"
        color = "purple"

    welding_trail.set_data(trail_x, trail_y)
    welding_trail.set_3d_properties(trail_z)

    status_text.set_text(f"Thời gian: {t:.2f}s / {TIME_FINISH:.2f}s | Trạng thái: {status}")
    status_text.set_color(color)
    return robot_arm, welding_trail, status_text


ani = animation.FuncAnimation(fig, update, frames=total_frames // STEP, interval=20, blit=False, repeat=False)
plt.show()