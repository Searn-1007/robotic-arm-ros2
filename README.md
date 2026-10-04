# Mô phỏng cánh tay robot RPRR trên ROS 2 (Gazebo + RViz)

Bài tập lớn môn Robotics: mô phỏng cánh tay robot 4 khớp (RPRR) gắn đầu hàn, gồm mô hình URDF/Xacro từ SolidWorks, điều khiển bằng `ros2_control` và sinh quỹ đạo LSPB trong không gian làm việc với động học ngược.

## Cấu trúc

```
src/
├── robotic_arm_description/   # URDF/Xacro, mesh STL, cấu hình RViz
├── robotic_arm_bringup/       # Launch file, cấu hình ros2_control và Gazebo bridge
└── robotic_arm_control/       # Node sinh quỹ đạo (trajectory_client.py)
```

## Yêu cầu

- ROS 2 (Jazzy / Humble) + Gazebo (gz sim)
- `ros_gz_sim`, `ros_gz_bridge`, `ros2_control`, `ros2_controllers`, `xacro`, `joint_state_publisher_gui`, `rviz2`
- Python: `numpy`

## Build

```bash
cd robotic-arm-ros2
colcon build
source install/setup.bash
```

## Chạy

Xem mô hình trên RViz (kéo thanh trượt để chỉnh từng khớp):

```bash
ros2 launch robotic_arm_bringup display.launch.py
```

Mô phỏng trên Gazebo + RViz:

```bash
ros2 launch robotic_arm_bringup gazebo.launch.py
```

Chạy quỹ đạo mẫu (Home → B → dừng → A → dừng → về Home) ở terminal khác:

```bash
ros2 run robotic_arm_control trajectory_client.py
```
