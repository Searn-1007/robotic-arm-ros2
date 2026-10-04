"""
================================================================
 kinematics.py - Dong hoc robot han 4DOF
----------------------------------------------------------------
 - Doc cau hinh tu config/dh_params.yaml
 - dh_matrix()         : ma tran bien doi thuan nhat DH
 - forward_kinematics(): FK -> vi tri dau mo han
 Ho tro input dang mang numpy (de chay Monte Carlo nhanh).
================================================================
"""

import numpy as np
import yaml


class Robot:
    def __init__(self, config_path):
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        self.base_offset_d = cfg["base_offset_d"]
        self.joints = cfg["joints"]
        self.torch_offset = np.array(cfg["torch_offset_mm"]) / 1000.0  # mm -> m
        self.torch_offset = np.append(self.torch_offset, 1.0)          # them 1 -> [x,y,z,1]
        self.n_samples = cfg["n_samples"]

    # ----- Gioi han 4 khop (doi sang rad neu la khop quay) -----
    def get_limits(self):
        """Tra ve list [(min, max), ...] theo dung don vi tinh toan (rad / m)."""
        limits = []
        for j in self.joints:
            lo, hi = j["limit"]
            if j["type"] == "revolute":
                limits.append((np.deg2rad(lo), np.deg2rad(hi)))
            else:  # prismatic
                limits.append((lo, hi))
        return limits

    # ----- Ma tran DH -----
    @staticmethod
    def dh_matrix(theta, d, a, alpha):
        """Ma tran DH 4x4. Cac tham so co the la scalar hoac mang numpy."""
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)
        ones = np.ones_like(ct)
        shape = ones.shape
        T = np.zeros(shape + (4, 4))
        T[..., 0, 0] = ct; T[..., 0, 1] = -st * ca; T[..., 0, 2] =  st * sa; T[..., 0, 3] = a * ct
        T[..., 1, 0] = st; T[..., 1, 1] =  ct * ca; T[..., 1, 2] = -ct * sa; T[..., 1, 3] = a * st
        T[..., 2, 1] = sa; T[..., 2, 2] = ca;       T[..., 2, 3] = d * ones
        T[..., 3, 3] = 1.0
        return T

    # ----- Dong hoc thuan -----
    def forward_kinematics(self, q):
        """
        q : list/array 4 phan tu [q1, q2, q3, q4]
            (moi phan tu co the la scalar hoac mang numpy cung kich thuoc)
        Tra ve: x, y, z  cua dau mo han trong he goc.
        """
        q = list(q)
        q1_shape = np.asarray(q[0]) * 1.0  # ep ve float/mang
        ones = np.ones_like(q1_shape)

        # Khau 0 - base co dinh
        T = self.dh_matrix(0.0 * ones, self.base_offset_d, 0.0, 0.0)

        # Lan luot nhan 4 khau dong
        for i, j in enumerate(self.joints):
            qi = np.asarray(q[i]) * 1.0
            if j["type"] == "revolute":
                theta = qi + np.deg2rad(j["theta_offset"])
                d = j["d"]
            else:  # prismatic
                theta = np.deg2rad(j["theta"]) * ones
                d = qi + j["d_offset"]
            a = j["a"]
            alpha = np.deg2rad(j["alpha"])
            T = T @ self.dh_matrix(theta, d, a, alpha)

        # Ghep offset dau mo han (he O4 -> he goc)
        p = T @ self.torch_offset
        return p[..., 0], p[..., 1], p[..., 2]