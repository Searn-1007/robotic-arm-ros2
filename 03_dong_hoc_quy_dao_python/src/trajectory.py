import numpy as np

def plan_hust_trapezoid(p_home, p_B, p_A, v_const, a_max, dt=0.01):
    """
    Quy hoạch quỹ đạo theo đúng bài toán giải tích trang 118 Slide thầy Khôi.
    Hệ thống chuyển động là một hình thang thống nhất trên tổng quãng đường S.
    """
    p_home = np.array(p_home, dtype=float)
    p_B = np.array(p_B, dtype=float)
    p_A = np.array(p_A, dtype=float)

    # 1. Tính toán chiều dài hình học
    L1 = np.linalg.norm(p_B - p_home)   # Đoạn Home -> B
    L2 = np.linalg.norm(p_A - p_B)      # Đoạn B -> A (Đoạn hàn)
    L3 = np.linalg.norm(p_home - p_A)   # Đoạn A -> Home
    S_max = L1 + L2 + L3                # Tổng chiều dài hình thang

    # 2. Tính toán phân bổ thời gian hình thang tổng
    t_a = v_const / a_max               # Thời gian tăng tốc
    s_1 = 0.5 * a_max * (t_a**2)        # Quãng đường pha tăng tốc (s_1 = s_3)
    s_3 = s_1

    # Quãng đường còn lại dành cho pha đi đều (Lv_const)
    L_vconst = S_max - s_1 - s_3
    t_const = L_vconst / v_const        # Thời gian đi đều
    t_f = 2 * t_a + t_const             # Tổng thời gian chu kỳ (T_sum)
    t_b = t_f - t_a                     # Bắt đầu giảm tốc

    # Tạo mảng thời gian trích mẫu
    time_profile = np.arange(0, t_f + dt, dt)
    if time_profile[-1] < t_f:
        time_profile = np.append(time_profile, t_f)

    s_profile = np.zeros_like(time_profile)
    v_profile = np.zeros_like(time_profile)
    a_profile = np.zeros_like(time_profile)
    path_xyz = np.zeros((len(time_profile), 3))

    # 3. Tính toán s(t), v(t), a(t) và nội suy tọa độ 3D theo quãng đường tích lũy s
    for i, t in enumerate(time_profile):
        # Thiết lập luật thời gian hình thang cho s(t)
        if t <= t_a:
            s_profile[i] = 0.5 * a_max * (t**2)
            v_profile[i] = a_max * t
            a_profile[i] = a_max
        elif t <= t_b:
            s_profile[i] = s_1 + v_const * (t - t_a)
            v_profile[i] = v_const
            a_profile[i] = 0.0
        else:
            s_profile[i] = S_max - 0.5 * a_max * ((t_f - t)**2)
            v_profile[i] = a_max * (t_f - t)
            a_profile[i] = -a_max

        # Nội suy vị trí Cartesian dựa trên giá trị s thực tế
        s_val = s_profile[i]
        if s_val <= L1:
            ratio = s_val / L1 if L1 > 0 else 1.0
            path_xyz[i] = p_home + ratio * (p_B - p_home)
        elif s_val <= (L1 + L2):
            ratio = (s_val - L1) / L2 if L2 > 0 else 1.0
            path_xyz[i] = p_B + ratio * (p_A - p_B)
        else:
            ratio = (s_val - L1 - L2) / L3 if L3 > 0 else 1.0
            path_xyz[i] = p_A + ratio * (p_home - p_A)

    return path_xyz, time_profile, s_profile, v_profile, a_profile, (t_a, t_b, t_f, L1, L2)