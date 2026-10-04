import numpy as np
from src.kinematics import Robot
from src.jacobian import position_jacobian

# Đường dẫn config
CONFIG_PATH = "config/dh_params.yaml"


def fixed_jacobian_wrapper(robot, q, h=1e-6):
    """
    Wrapper này xử lý dữ liệu trước khi đưa vào hàm gốc trong jacobian.py
    để đảm bảo tính đồng nhất về shape và kiểu dữ liệu.
    """
    # Ép kiểu an toàn cho mọi biến khớp đầu vào
    q_safe = [np.atleast_1d(np.array(qi, dtype=float)) for qi in q]

    # Gọi hàm gốc từ jacobian.py (chúng ta không sửa file gốc)
    return position_jacobian(robot, q_safe, h)


def main():
    # Khởi tạo robot
    robot = Robot(CONFIG_PATH)

    # Giả lập 1 vị trí khớp (đảm bảo là numpy array)
    # q1, q2, q3, q4
    q_test = [np.array([0.1]), np.array([0.1]), np.array([0.1]), np.array([0.1])]

    try:
        # Sử dụng wrapper thay vì gọi trực tiếp để tránh lỗi shape
        J = fixed_jacobian_wrapper(robot, q_test)
        print("Tính toán Jacobian thành công!")
        print("Shape của Jacobian:", J.shape)
        print("Giá trị J:\n", J)
    except Exception as e:
        print(f"Lỗi khi chạy: {e}")
        print(
            "Gợi ý: Kiểm tra lại hàm forward_kinematics trong kinematics.py xem nó có trả về đúng 3 giá trị x, y, z hay không.")


if __name__ == "__main__":
    main()