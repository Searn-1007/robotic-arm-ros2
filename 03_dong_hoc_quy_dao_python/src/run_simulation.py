"""
Kiểm tra nhanh Jacobian vị trí tại một cấu hình khớp mẫu.

Chạy từ thư mục ``03_dong_hoc_quy_dao_python``:
    python -m src.run_simulation
"""

import numpy as np
from src.kinematics import Robot
from src.jacobian import position_jacobian

CONFIG_PATH = "config/dh_params.yaml"


def fixed_jacobian_wrapper(robot, q, h=1e-6):
    """Chuẩn hóa shape và kiểu dữ liệu của biến khớp trước khi gọi ``position_jacobian()``."""
    q_safe = [np.atleast_1d(np.array(qi, dtype=float)) for qi in q]
    return position_jacobian(robot, q_safe, h)


def main():
    robot = Robot(CONFIG_PATH)

    # Cấu hình khớp mẫu [q1, q2, q3, q4]
    q_test = [np.array([0.1]), np.array([0.1]), np.array([0.1]), np.array([0.1])]

    try:
        J = fixed_jacobian_wrapper(robot, q_test)
        print("Tính toán Jacobian thành công!")
        print("Shape của Jacobian:", J.shape)
        print("Giá trị J:\n", J)
    except Exception as e:
        print(f"Lỗi khi chạy: {e}")
        print(
            "Gợi ý: Kiểm tra lại hàm forward_kinematics trong kinematics.py "
            "xem nó có trả về đúng 3 giá trị x, y, z hay không."
        )


if __name__ == "__main__":
    main()
