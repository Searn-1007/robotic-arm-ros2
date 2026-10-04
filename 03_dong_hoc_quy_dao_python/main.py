
import os

from src.kinematics import Robot
from src.workspace_calculator import compute_workspace, print_summary, save_points
from src.utils import plot_all
from src.jacobian import manipulability

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(BASE_DIR, "config", "dh_params.yaml")
DATA_DIR = os.path.join(BASE_DIR, "data")


def main():
    # 1) Doc cau hinh robot
    robot = Robot(CONFIG_PATH)

    # 2) Tinh workspace (random + FK). seed=42 de ket qua lap lai duoc.
    ws = compute_workspace(robot, seed=42)

    # 3) In thong so bien
    print_summary(ws)

    # 4) Luu diem (tuy chon)
    save_points(ws, out_dir=DATA_DIR)

    # 5) Tinh manipulability w (tim vung ky di)
    w = manipulability(robot, ws["q"])
    print(f"Manipulability w: {w.min():.5f}  ->  {w.max():.5f}")

    # 6) Ve TAT CA trong 1 cua so + luu hinh
    plot_all(ws, w, save_path=os.path.join(DATA_DIR, "ket_qua.png"))


if __name__ == "__main__":
    main()