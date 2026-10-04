"""
Quy hoạch quỹ đạo hình thang thống nhất trên tổng quãng đường Home -> B -> A -> Home.
"""

import numpy as np


def plan_hust_trapezoid(p_home, p_B, p_A, v_const, a_max, dt=0.01):
    """
    Quy hoạch quỹ đạo theo bài toán giải tích (slide bài giảng, trang 118).

    Toàn bộ chu trình được coi là một biên dạng vận tốc hình thang duy nhất
    trên tổng quãng đường S; vị trí Cartesian được nội suy theo quãng đường
    tích lũy s(t) trên từng đoạn thẳng.

    Returns:
        ``(path_xyz, time_profile, s_profile, v_profile, a_profile, (t_a, t_b, t_f, L1, L2))``
    """
    p_home = np.array(p_home, dtype=float)
    p_B = np.array(p_B, dtype=float)
    p_A = np.array(p_A, dtype=float)

    # 1. Chiều dài hình học các đoạn
    L1 = np.linalg.norm(p_B - p_home)   # Home -> B
    L2 = np.linalg.norm(p_A - p_B)      # B -> A (đoạn hàn)
    L3 = np.linalg.norm(p_home - p_A)   # A -> Home
    S_max = L1 + L2 + L3                # tổng quãng đường

    # 2. Phân bổ thời gian hình thang tổng
    t_a = v_const / a_max               # thời gian tăng tốc
    s_1 = 0.5 * a_max * (t_a**2)        # quãng đường pha tăng tốc (s_1 = s_3)
    s_3 = s_1

    L_vconst = S_max - s_1 - s_3        # quãng đường pha đi đều
    t_const = L_vconst / v_const        # thời gian đi đều
    t_f = 2 * t_a + t_const             # tổng thời gian chu kỳ
    t_b = t_f - t_a                     # thời điểm bắt đầu giảm tốc

    # Mảng thời gian lấy mẫu
    time_profile = np.arange(0, t_f + dt, dt)
    if time_profile[-1] < t_f:
        time_profile = np.append(time_profile, t_f)

    s_profile = np.zeros_like(time_profile)
    v_profile = np.zeros_like(time_profile)
    a_profile = np.zeros_like(time_profile)
    path_xyz = np.zeros((len(time_profile), 3))

    # 3. Tính s(t), v(t), a(t) và nội suy tọa độ 3D theo quãng đường tích lũy s
    for i, t in enumerate(time_profile):
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
