# Robot hàn 4 bậc tự do RPRR — Bài tập lớn Robotics

Thiết kế, tính toán và mô phỏng một cánh tay robot hàn 4 bậc tự do kiểu **R–P–R–R** (quay – tịnh tiến – quay – quay), làm trọn vẹn từ thiết kế cơ khí, động học, động lực học, thiết kế quỹ đạo, bộ điều khiển cho tới mô phỏng trên ROS 2.

## Nội dung

| Thư mục | Nội dung | Công cụ |
|---|---|---|
| [`01_thiet_ke_solidworks`](01_thiet_ke_solidworks) | Mô hình 3D các khâu, cụm lắp ráp, đầu hàn, bản vẽ kỹ thuật; G-code in 3D | SolidWorks, Cura |
| [`02_dong_luc_hoc_maple`](02_dong_luc_hoc_maple) | Thiết lập phương trình vi phân chuyển động của robot | Maple |
| [`03_dong_hoc_quy_dao_python`](03_dong_hoc_quy_dao_python) | Động học thuận/ngược (DH), Jacobian, không gian làm việc, chỉ số manipulability, quỹ đạo LSPB trong không gian khớp và Cartesian, animation | Python |
| [`04_bo_dieu_khien_IDPD_simulink`](04_bo_dieu_khien_IDPD_simulink) | Bộ điều khiển IDPD (Inverse Dynamics PD) bám quỹ đạo | MATLAB / Simulink |
| [`05_mo_phong_ros2`](05_mo_phong_ros2) | Mô hình URDF, mô phỏng Gazebo + RViz, điều khiển bằng `ros2_control` | ROS 2 Humble |
| [`bao_cao`](bao_cao) | Báo cáo đầy đủ | PDF |

## Thông số robot (DH)

| Khớp | Loại | d (m) | a (m) | α (°) | Giới hạn |
|---|---|---|---|---|---|
| q1 | Quay | 0.2175 | 0 | 0 | 0° → 360° |
| q2 | Tịnh tiến | q2 | 0.25 | 90 | 0 → 0.225 m |
| q3 | Quay (θ = q3 + 90°) | 0.105 | 0 | 90 | −90° → 135° |
| q4 | Quay (mỏ hàn) | 0.262 | 0 | 0 | 0° → 360° |

Đế cố định d0 = 0.084 m. Đầu mỏ hàn lệch (−68, 0, 126) mm so với hệ O4.

## Hướng dẫn chạy nhanh

**Động học & quỹ đạo (Python)**

```bash
cd 03_dong_hoc_quy_dao_python
pip install -r requirements.txt
python main.py            # không gian làm việc + manipulability -> data/ket_qua.png
python JointSpace.py      # quỹ đạo trong không gian khớp
python CartesianSpace.py  # quỹ đạo trong không gian Cartesian
python animation.py       # animation 3D robot chạy theo quỹ đạo
```

**Bộ điều khiển IDPD (MATLAB)**: chạy `initialize.m` trước để nạp tham số, sau đó mở `controller_simulation.slx` và bấm Run. Xem [README](04_bo_dieu_khien_IDPD_simulink/README.md).

**Mô phỏng ROS 2**: xem [05_mo_phong_ros2/README.md](05_mo_phong_ros2/README.md).

## Kết quả

![Không gian làm việc và manipulability](03_dong_hoc_quy_dao_python/data/ket_qua.png)
