"""
================================================================
 utils.py - Tien ich ve workspace
----------------------------------------------------------------
 plot_workspace(): ve 3 hinh tren cung 1 cua so
   1) Workspace 3D (xoay duoc trong PyCharm)
   2) Nhin tu tren (XY)   -> dang vanh khan
   3) Mat cat doc (R - Z) -> hinh that cua workspace
================================================================
"""

import os
import numpy as np
import matplotlib.pyplot as plt


def plot_workspace(ws, max_points=25000, save_path=None):
    """
    ws         : dict {x, y, z, r} tu compute_workspace()
    max_points : so diem ve len (ve het 400k se lag) -> lay mau bot
    save_path  : neu co, luu hinh ra file (vd 'data/workspace.png')
    """
    x, y, z, r = ws["x"], ws["y"], ws["z"], ws["r"]

    # Lay mau bot cho nhe khi ve
    n = x.size
    if n > max_points:
        idx = np.random.choice(n, max_points, replace=False)
        x, y, z, r = x[idx], y[idx], z[idx], r[idx]

    fig = plt.figure(figsize=(15, 5))

    # --- 1) 3D ---
    ax1 = fig.add_subplot(131, projection="3d")
    ax1.scatter(x, y, z, s=1, c=z, cmap="viridis", alpha=0.3)
    ax1.set_title("Workspace 3D (dau mo han)")
    ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.set_zlabel("Z (m)")

    # --- 2) Nhin tu tren XY ---
    ax2 = fig.add_subplot(132)
    ax2.scatter(x, y, s=1, alpha=0.2, c="tab:blue")
    ax2.set_title("Nhin tu tren (XY) - vanh khan")
    ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)")
    ax2.axis("equal"); ax2.grid(True, alpha=0.3)

    # --- 3) Mat cat doc R-Z ---
    ax3 = fig.add_subplot(133)
    ax3.scatter(r, z, s=1, alpha=0.2, c="tab:red")
    ax3.set_title("Mat cat doc (R - Z)")
    ax3.set_xlabel("R = sqrt(x^2 + y^2) (m)"); ax3.set_ylabel("Z (m)")
    ax3.grid(True, alpha=0.3)

    plt.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=110)
        print(f"Da luu hinh vao: {save_path}")

    plt.show()   # PyCharm: mo cua so do hoa (3D xoay duoc bang chuot)


def plot_manipulability(ws, w, max_points=25000, save_path=None):
    """
    Ve workspace TO MAU theo manipulability w.
      ws : dict {x, y, z, r}
      w  : mang manipulability (cung kich thuoc voi so diem)
    Mau sang (vang) = w lon = kheo leo.
    Mau toi  (tim)  = w nho = gan diem ky di (singularity).
    """
    x, y, z, r = ws["x"], ws["y"], ws["z"], ws["r"]

    # Lay mau bot cho nhe
    n = x.size
    if n > max_points:
        idx = np.random.choice(n, max_points, replace=False)
        x, y, z, r, w = x[idx], y[idx], z[idx], r[idx], w[idx]

    fig = plt.figure(figsize=(11, 5))

    # --- Mat cat R-Z to mau theo w (hinh quan trong nhat) ---
    ax1 = fig.add_subplot(121)
    sc1 = ax1.scatter(r, z, s=2, c=w, cmap="viridis", alpha=0.5)
    ax1.set_title("Manipulability (mat cat R-Z)")
    ax1.set_xlabel("R (m)"); ax1.set_ylabel("Z (m)")
    ax1.grid(True, alpha=0.3)
    fig.colorbar(sc1, ax=ax1, label="w (cao = kheo, thap = ky di)")

    # --- Nhin tu tren XY to mau theo w ---
    ax2 = fig.add_subplot(122)
    sc2 = ax2.scatter(x, y, s=2, c=w, cmap="viridis", alpha=0.5)
    ax2.set_title("Manipulability (nhin tu tren XY)")
    ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)")
    ax2.axis("equal"); ax2.grid(True, alpha=0.3)
    fig.colorbar(sc2, ax=ax2, label="w")

    plt.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=110)
        print(f"Da luu hinh vao: {save_path}")

    plt.show()


def plot_all(ws, w, max_points=20000, save_path=None):
    """
    Gop TAT CA vao 1 cua so duy nhat (khoi phai dong cai nay moi hien cai kia).
      Hang tren : workspace (3D, XY, R-Z)
      Hang duoi : manipulability to mau theo w (R-Z, XY)
    """
    x, y, z, r = ws["x"], ws["y"], ws["z"], ws["r"]

    n = x.size
    if n > max_points:
        idx = np.random.choice(n, max_points, replace=False)
        x, y, z, r, w = x[idx], y[idx], z[idx], r[idx], w[idx]

    fig = plt.figure(figsize=(15, 9))

    # ===== HANG TREN: WORKSPACE =====
    ax1 = fig.add_subplot(231, projection="3d")
    ax1.scatter(x, y, z, s=1, c=z, cmap="viridis", alpha=0.3)
    ax1.set_title("Workspace 3D")
    ax1.set_xlabel("X (m)"); ax1.set_ylabel("Y (m)"); ax1.set_zlabel("Z (m)")

    ax2 = fig.add_subplot(232)
    ax2.scatter(x, y, s=1, alpha=0.2, c="tab:blue")
    ax2.set_title("Workspace - nhin tu tren (XY)")
    ax2.set_xlabel("X (m)"); ax2.set_ylabel("Y (m)")
    ax2.axis("equal"); ax2.grid(True, alpha=0.3)

    ax3 = fig.add_subplot(233)
    ax3.scatter(r, z, s=1, alpha=0.2, c="tab:red")
    ax3.set_title("Workspace - mat cat (R-Z)")
    ax3.set_xlabel("R (m)"); ax3.set_ylabel("Z (m)")
    ax3.grid(True, alpha=0.3)

    # ===== HANG DUOI: MANIPULABILITY =====
    ax4 = fig.add_subplot(234)
    sc4 = ax4.scatter(r, z, s=2, c=w, cmap="viridis", alpha=0.5)
    ax4.set_title("Manipulability (R-Z)")
    ax4.set_xlabel("R (m)"); ax4.set_ylabel("Z (m)")
    ax4.grid(True, alpha=0.3)
    fig.colorbar(sc4, ax=ax4, label="w")

    ax5 = fig.add_subplot(235)
    sc5 = ax5.scatter(x, y, s=2, c=w, cmap="viridis", alpha=0.5)
    ax5.set_title("Manipulability (XY)")
    ax5.set_xlabel("X (m)"); ax5.set_ylabel("Y (m)")
    ax5.axis("equal"); ax5.grid(True, alpha=0.3)
    fig.colorbar(sc5, ax=ax5, label="w")

    plt.tight_layout()

    if save_path:
        os.makedirs(os.path.dirname(save_path), exist_ok=True)
        plt.savefig(save_path, dpi=110)
        print(f"Da luu hinh vao: {save_path}")

    plt.show()