import numpy as np
import matplotlib.pyplot as plt

# ==========================================
# 1. TÍNH TOÁN GIẢI TÍCH s(t), v(t), a(t)
# ==========================================
def generate_lspb(dist, V_max, A_max, dt):
    """Tính toán trực tiếp vị trí (s), vận tốc (v), gia tốc (a) mà KHÔNG cần đạo hàm số"""
    if dist < 1e-5:
        return np.array([]), np.array([]), np.array([]), np.array([])

    if V_max ** 2 / A_max > dist:
        V_max = np.sqrt(dist * A_max)

    t_b = V_max / A_max
    t_c = (dist - V_max * t_b) / V_max
    t_f = 2 * t_b + t_c

    t = np.arange(0, t_f, dt)
    s = np.zeros_like(t)
    v = np.zeros_like(t)
    a = np.zeros_like(t)

    for i, ti in enumerate(t):
        if ti < t_b:
            s[i] = 0.5 * A_max * ti ** 2
            v[i] = A_max * ti
            a[i] = A_max
        elif ti < t_b + t_c:
            s[i] = 0.5 * A_max * t_b ** 2 + V_max * (ti - t_b)
            v[i] = V_max
            a[i] = 0.0
        else:
            tau = ti - t_b - t_c
            s[i] = (0.5 * A_max * t_b ** 2 + V_max * t_c) + (V_max * tau - 0.5 * A_max * tau ** 2)
            v[i] = V_max - A_max * tau
            a[i] = -A_max

    return t, s, v, a

# ==========================================
# 2. GHÉP CHẶNG CARTESIAN STOP-AND-GO
# ==========================================
DT = 0.01

P_Home = np.array([0.287, 0.0, 0.6895])
P_B = np.array([0.2, 0.15, 0.7])
P_A = np.array([0.2, 0.15, 0.5])

# Chặng 1: Home -> B
dist_1 = np.linalg.norm(P_B - P_Home)
t1, s1, v1, a1 = generate_lspb(dist_1, V_max=0.05, A_max=0.1, dt=DT)

# Chặng 2: B -> A
dist_2 = np.linalg.norm(P_A - P_B)
t2, s2, v2, a2 = generate_lspb(dist_2, V_max=0.01, A_max=0.05, dt=DT)

# Chặng 3: A -> Home
dist_3 = np.linalg.norm(P_Home - P_A)
t3, s3, v3, a3 = generate_lspb(dist_3, V_max=0.05, A_max=0.1, dt=DT)

# ==========================================
# 2.5. TẠO DỮ LIỆU DWELL TIME CỐ ĐỊNH TẠI B VÀ A
# ==========================================
T_DWELL_B = 0.5  # 0.5s chờ mồi hồ quang tại B
T_DWELL_A = 1.0  # 1.0s chờ điền đầy miệng hàn tại A

# Mảng thời gian và v, a (đều bằng 0)
t_dwell_b = np.arange(0, T_DWELL_B, DT)
v_dwell_b = np.zeros_like(t_dwell_b)
a_dwell_b = np.zeros_like(t_dwell_b)

t_dwell_a = np.arange(0, T_DWELL_A, DT)
v_dwell_a = np.zeros_like(t_dwell_a)
a_dwell_a = np.zeros_like(t_dwell_a)

# ==========================================
# 3. GỘP DỮ LIỆU ĐỂ VẼ ĐỒ THỊ TỔNG THỂ
# ==========================================
# Trượt mốc thời gian nối tiếp nhau
t_dwell_b_offset = t_dwell_b + t1[-1] + DT
t2_offset = t2 + t_dwell_b_offset[-1] + DT
t_dwell_a_offset = t_dwell_a + t2_offset[-1] + DT
t3_offset = t3 + t_dwell_a_offset[-1] + DT

T_total = np.concatenate([t1, t_dwell_b_offset, t2_offset, t_dwell_a_offset, t3_offset])

# Quãng đường s(t): Giữ nguyên vị trí trong suốt thời gian Dwell
s_dwell_b_offset = np.full_like(t_dwell_b, s1[-1])
s2_offset = s2 + s_dwell_b_offset[-1]

s_dwell_a_offset = np.full_like(t_dwell_a, s2_offset[-1])
s3_offset = s3 + s_dwell_a_offset[-1]

S_total = np.concatenate([s1, s_dwell_b_offset, s2_offset, s_dwell_a_offset, s3_offset])

# Vận tốc và Gia tốc
V_total = np.concatenate([v1, v_dwell_b, v2, v_dwell_a, v3])
A_total = np.concatenate([a1, a_dwell_b, a2, a_dwell_a, a3])

# ==========================================
# 3.5. NỘI SUY VỊ TRÍ 3D (X, Y, Z) VÀ XUẤT FILE TXT
# ==========================================
# Nội suy vị trí thực tế của robot dựa trên quãng đường s(t)
pos1 = P_Home + (s1[:, None] / dist_1) * (P_B - P_Home)
pos_dwell_b = np.tile(P_B, (len(t_dwell_b), 1))
pos2 = P_B + (s2[:, None] / dist_2) * (P_A - P_B)
pos_dwell_a = np.tile(P_A, (len(t_dwell_a), 1))
pos3 = P_A + (s3[:, None] / dist_3) * (P_Home - P_A)

# Gộp tất cả tọa độ 3D thành một mảng duy nhất
Pos_total = np.vstack([pos1, pos_dwell_b, pos2, pos_dwell_a, pos3])

# Ghi ra file .txt
file_name = "Cartesian_Waypoints.txt"
with open(file_name, "w", encoding="utf-8") as f:
    f.write("Time(s)\tX(m)\tY(m)\tZ(m)\n")
    for t_i, p_i in zip(T_total, Pos_total):
        f.write(f"{t_i:.3f}\t{p_i[0]:.4f}\t{p_i[1]:.4f}\t{p_i[2]:.4f}\n")


# ==========================================
# THỐNG KÊ THỜI GIAN CHU TRÌNH HÀN LÊN CONSOLE
# ==========================================
time_reach_B = t1[-1]
time_leave_B = t_dwell_b_offset[-1]
time_reach_A = t2_offset[-1]
time_leave_A = t_dwell_a_offset[-1]
time_finish  = T_total[-1]

print("\n" + "="*50)
print("BẢNG THỐNG KÊ THỜI GIAN CHU TRÌNH HÀN")
print("="*50)
print(f"[1] Mỏ hàn chạm điểm B (Bắt đầu Dwell mồi hồ quang): {time_reach_B:.3f} s")
print(f"[2] Mỏ hàn rời điểm B  (Bắt đầu rê mỏ hàn đi hàn)   : {time_leave_B:.3f} s")
print(f"[3] Mỏ hàn chạm điểm A (Bắt đầu Dwell ngắt hồ quang): {time_reach_A:.3f} s")
print(f"[4] Mỏ hàn rời điểm A  (Rút mỏ hàn về vị trí Home)  : {time_leave_A:.3f} s")
print(f"[5] Hoàn thành toàn bộ chu trình                      : {time_finish:.3f} s")
print("="*50)
print(f"[*] Đã xuất dữ liệu tọa độ 3D thành công ra file: {file_name}")
print("="*50 + "\n")

# ==========================================
# 4. XUẤT ĐỒ THỊ BÁO CÁO
# ==========================================
plt.figure(figsize=(12, 10))

# Hàm phụ trợ để highlight vùng Dwell Time
def highlight_dwells():
    plt.axvspan(t1[-1], t_dwell_b_offset[-1], color='yellow', alpha=0.3, label='Dwell B (Mồi)')
    plt.axvspan(t2_offset[-1], t_dwell_a_offset[-1], color='orange', alpha=0.3, label='Dwell A (Ngắt)')
    plt.legend()

# 1. Đồ thị Quãng đường
plt.subplot(3, 1, 1)
plt.plot(T_total, S_total, 'b-', linewidth=2)
highlight_dwells()
plt.title("QUÃNG ĐƯỜNG CARTESIAN TÍCH LŨY S(t)", fontweight='bold')
plt.ylabel("Quãng đường (m)")
plt.grid(True, linestyle='--')

# 2. Đồ thị Vận tốc
plt.subplot(3, 1, 2)
plt.plot(T_total, V_total, 'g-', linewidth=2)
highlight_dwells()
plt.title("VẬN TỐC CARTESIAN V(t)", fontweight='bold')
plt.ylabel("Vận tốc (m/s)")
plt.grid(True, linestyle='--')

# 3. Đồ thị Gia tốc
plt.subplot(3, 1, 3)
plt.plot(T_total, A_total, 'm-', linewidth=2)
highlight_dwells()
plt.title("GIA TỐC CARTESIAN A(t)", fontweight='bold')
plt.xlabel("Thời gian (s)")
plt.ylabel("Gia tốc (m/s²)")
plt.grid(True, linestyle='--')

plt.tight_layout()
plt.show()