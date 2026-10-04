"""
================================================================
 workspace_calculator.py - Tinh vung lam viec (workspace)
----------------------------------------------------------------
 - compute_workspace(): random 4 khop trong gioi han -> goi FK
                        -> tra ve x, y, z cua dau mo han
 - print_summary()    : in bien (r, z, x, y)
 - save_points()      : luu diem ra data/*.npy (tuy chon)
================================================================
"""

import os
import numpy as np


def compute_workspace(robot, n_samples=None, seed=None):
    """
    robot     : doi tuong Robot (tu kinematics.py)
    n_samples : so diem random (mac dinh lay tu config)
    seed      : co dinh seed de ket qua lap lai duoc (tuy chon)

    Tra ve dict {x, y, z, r}.
    """
    if n_samples is None:
        n_samples = robot.n_samples
    if seed is not None:
        np.random.seed(seed)

    limits = robot.get_limits()      # [(min,max) x4] da dung don vi rad/m

    # Random tung khop trong gioi han (tuong duong 4 vong for)
    q = [np.random.uniform(lo, hi, n_samples) for (lo, hi) in limits]

    # Goi dong hoc thuan
    x, y, z = robot.forward_kinematics(q)
    r = np.sqrt(x ** 2 + y ** 2)

    # Tra ve them 'q' (list 4 mang goc khop) de con tinh manipulability
    return {"x": x, "y": y, "z": z, "r": r, "q": q}


def print_summary(ws):
    """In thong so bien cua workspace."""
    x, y, z, r = ws["x"], ws["y"], ws["z"], ws["r"]
    print("============ THONG SO WORKSPACE ============")
    print(f"So diem          : {x.size}")
    print(f"Ban kinh ngang r : {r.min():.3f}  ->  {r.max():.3f}  m")
    print(f"Chieu cao      z : {z.min():.3f}  ->  {z.max():.3f}  m")
    print(f"X                : {x.min():.3f}  ->  {x.max():.3f}  m")
    print(f"Y                : {y.min():.3f}  ->  {y.max():.3f}  m")
    print("============================================")


def save_points(ws, out_dir="data", filename="workspace_points.npy"):
    """Luu diem (x,y,z) ra file .npy de dung lai sau."""
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, filename)
    pts = np.column_stack([ws["x"], ws["y"], ws["z"]])
    np.save(path, pts)
    print(f"Da luu {pts.shape[0]} diem vao: {path}")
    return path