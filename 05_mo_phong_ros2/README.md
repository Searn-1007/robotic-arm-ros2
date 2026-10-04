# Mô phỏng ROS 2 (Gazebo + RViz)

Workspace `rprr_ws` gồm 3 package:

```
rprr_ws/src/
├── robotic_arm_description/   # URDF/Xacro, mesh STL (xuất từ SolidWorks), cấu hình RViz
├── robotic_arm_bringup/       # Launch file, cấu hình ros2_control và Gazebo bridge
└── robotic_arm_control/       # trajectory_client.py: sinh quỹ đạo LSPB + động học ngược
```

## Yêu cầu

- Ubuntu 22.04 + ROS 2 Humble, Gazebo (gz sim)
- `ros_gz_sim`, `ros_gz_bridge`, `ros2_control`, `ros2_controllers`, `xacro`, `joint_state_publisher_gui`, `rviz2`
- Python: `numpy`

## Build

```bash
cp -r rprr_ws ~/rprr_ws
cd ~/rprr_ws
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

Ở terminal thứ hai, chạy quỹ đạo hàn mẫu (Home → B → dừng → A → dừng → về Home):

```bash
cd ~/rprr_ws/src/robotic_arm_control/src
python3 trajectory_client.py
```

Quan sát Gazebo hoặc RViz, robot sẽ di chuyển theo đúng quỹ đạo đã thiết kế.
