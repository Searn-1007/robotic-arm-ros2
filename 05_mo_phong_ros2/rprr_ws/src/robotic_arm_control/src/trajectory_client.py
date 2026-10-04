#!/usr/bin/env python3
"""
Node ROS 2 phát quỹ đạo hàn cho robot RPRR trong Gazebo.

Quỹ đạo Cartesian LSPB Home -> B -> dừng -> A -> dừng -> Home được tính trước,
chuyển sang không gian khớp bằng động học ngược, rồi phát lần lượt từng điểm
(chu kỳ 10 ms) xuống các topic ``/q1_cmd_pos`` ... ``/q4_cmd_pos``.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import numpy as np


class TrajectoryDirectPublisher(Node):
    """Phát trực tiếp vị trí đặt của từng khớp theo quỹ đạo đã tính sẵn."""

    def __init__(self):
        super().__init__('trajectory_direct_publisher')

        # Publisher vị trí đặt cho từng khớp trong mô phỏng
        self._pub_q1 = self.create_publisher(Float64, '/q1_cmd_pos', 10)
        self._pub_q2 = self.create_publisher(Float64, '/q2_cmd_pos', 10)
        self._pub_q3 = self.create_publisher(Float64, '/q3_cmd_pos', 10)
        self._pub_q4 = self.create_publisher(Float64, '/q4_cmd_pos', 10)

        self.DT = 0.01

        self.Q_profile = self._calculate_joint_space_trajectory()
        self.current_index = 0

        self._timer = self.create_timer(self.DT, self.timer_callback)
        self.get_logger().info(f'Hệ thống sẵn sàng! Tổng chu trình gồm {len(self.Q_profile)} điểm.')

    def _calculate_joint_space_trajectory(self):
        """Tính trước toàn bộ quỹ đạo khớp ``(N, 3)`` cho q1, q2, q3."""
        D1, A2, D3, D4 = 0.2175, 0.25, 0.105, 0.262
        P_Home = np.array([0.287, 0.0, 0.6895])
        P_B = np.array([0.2, 0.15, 0.7])
        P_A = np.array([0.2, 0.15, 0.5])

        def generate_lspb(P_start, P_end, V_max, A_max, dt):
            dist = np.linalg.norm(P_end - P_start)
            if dist < 1e-5:
                return np.empty((0, 3))
            if V_max ** 2 / A_max > dist:
                V_max = np.sqrt(dist * A_max)
            t_b = V_max / A_max
            t_c = (dist - V_max * t_b) / V_max
            t = np.arange(0, 2 * t_b + t_c, dt)
            s = np.zeros_like(t)
            for i, ti in enumerate(t):
                if ti < t_b:
                    s[i] = 0.5 * A_max * ti ** 2
                elif ti < t_b + t_c:
                    s[i] = 0.5 * A_max * t_b ** 2 + V_max * (ti - t_b)
                else:
                    tau = ti - t_b - t_c
                    s[i] = (0.5 * A_max * t_b ** 2 + V_max * t_c) + (V_max * tau - 0.5 * A_max * tau ** 2)
            direction = (P_end - P_start) / dist
            return P_start + s[:, np.newaxis] * direction

        def generate_dwell(P_stay, duration, dt):
            t = np.arange(0, duration, dt)
            return np.tile(P_stay, (len(t), 1))

        def inverse_kinematics(x, y, z):
            q2 = z - D1 - D4
            cos_q3 = np.clip((x ** 2 + y ** 2 - A2 ** 2 - D3 ** 2) / (2 * A2 * D3), -1.0, 1.0)
            q3 = np.arctan2(np.sqrt(1 - cos_q3 ** 2), cos_q3)
            q1 = np.arctan2(y, x) - np.arctan2(D4 * np.sin(q3), A2 + D4 * np.cos(q3))
            return q1, q2, q3

        p1 = generate_lspb(P_Home, P_B, V_max=0.05, A_max=0.1, dt=self.DT)
        pd_b = generate_dwell(P_B, duration=0.5, dt=self.DT)
        p2 = generate_lspb(P_B, P_A, V_max=0.01, A_max=0.05, dt=self.DT)
        pd_a = generate_dwell(P_A, duration=1.0, dt=self.DT)
        p3 = generate_lspb(P_A, P_Home, V_max=0.05, A_max=0.1, dt=self.DT)

        P_total = np.vstack([p1, pd_b, p2, pd_a, p3])

        N_points = len(P_total)
        Q_profile = np.zeros((N_points, 3))

        for i in range(N_points):
            x, y, z = P_total[i]
            q1, q2, q3 = inverse_kinematics(x, y, z)
            Q_profile[i] = [q1, q2, q3]

        return Q_profile

    def timer_callback(self):
        """Phát điểm quỹ đạo kế tiếp; dừng timer khi hết quỹ đạo."""
        if self.current_index >= len(self.Q_profile):
            self.get_logger().info('Hoàn thành! Robot đã đi từ Home -> B -> Dwell -> A -> Dwell -> Rút về Home thành công!')
            self._timer.cancel()
            return

        q1, q2, q3 = self.Q_profile[self.current_index]
        q4 = 0.0

        msg_q1 = Float64(data=float(q1))
        msg_q2 = Float64(data=float(q2))
        msg_q3 = Float64(data=float(q3))
        msg_q4 = Float64(data=float(q4))

        self._pub_q1.publish(msg_q1)
        self._pub_q2.publish(msg_q2)
        self._pub_q3.publish(msg_q3)
        self._pub_q4.publish(msg_q4)

        self.current_index += 1


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryDirectPublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
