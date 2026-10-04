# CLAUDE.md — bối cảnh dự án (bàn giao từ phiên làm việc trên Windows, 2026-10-05)

> File tạm để chuyển bối cảnh sang máy Ubuntu. Có thể xóa sau khi không cần nữa.

## Người dùng

- Giao tiếp bằng **tiếng Việt**.
- Đây là bài tập lớn môn Robotics của người dùng (làm nhóm 2 người): robot hàn 4 bậc tự do **R-P-R-R**.
- Ưu tiên của người dùng: **mô phỏng ROS 2 (Gazebo) phải chạy ngon**. Khi được yêu cầu "dọn code" thì tuyệt đối
  không đổi tính năng; mọi thay đổi hành vi phải hỏi trước.
- Git: `Searn-1007 <hasonhd123@gmail.com>`, remote `https://github.com/Searn-1007/robotic-arm-ros2.git`, nhánh `main`.

## Cấu trúc repo

| Thư mục | Nội dung |
|---|---|
| `01_thiet_ke_solidworks/` | File SolidWorks (.SLDPRT/.SLDASM/.SLDDRW), `gcode_in_3d/` (2 file G-code in 3D, không liên quan code) |
| `02_dong_luc_hoc_maple/` | `ptvp_chuyen_dong_robotics.mw` — phương trình Lagrange M(q), C(q,q̇), G(q) |
| `03_dong_hoc_quy_dao_python/` | Động học, workspace, manipulability, quỹ đạo LSPB, animation |
| `04_bo_dieu_khien_IDPD_simulink/` | `initialize.m` (chạy trước), `controller_simulation.slx`, `Vitridat/Vantocdat/Giatocdat.mat` |
| `05_mo_phong_ros2/rprr_ws/src/` | `robotic_arm_description` (URDF/Xacro, STL), `robotic_arm_bringup` (launch, config), `robotic_arm_control` (`trajectory_client.py`) |

Các script Python trong `03_...`:
- `main.py` + `src/` (kinematics, jacobian, workspace_calculator, utils): workspace Monte Carlo 400k điểm (seed 42) theo
  `config/dh_params.yaml` (tính tới **đầu mỏ hàn**, có offset (−68, 0, 126) mm trong hệ O4).
- `JointSpace.py`: quỹ đạo Cartesian LSPB → IK → q, q̇ (pinv Jacobian), q̈ (np.gradient). **Đầu ra của file này chính là
  3 file `.mat` cho Simulink** (mỗi file 5×N double: hàng 0 = t, hàng 1–3 = q1..q3, hàng 4 = q4 = 0; tên biến trùng tên file).
- `CartesianSpace.py`: đồ thị s(t), v(t), a(t). `test.py`: như CartesianSpace + ghi `Cartesian_Waypoints.txt`.
- `animation.py`: animation 3D. `src/run_simulation.py`: chạy bằng `python -m src.run_simulation`. `src/trajectory.py`: không được dùng.

## Mô hình động học (thống nhất toàn repo)

Bảng DH: d0 = 0.084 (đế); q1: d=0.2175; q2 (tịnh tiến): a=0.25, α=90°; q3: θ=q3+90°, d=0.105, α=90°; q4: d=0.262.
Giới hạn: q2 ∈ [0, 0.225] m, q3 ∈ [−90°, 135°]. Quỹ đạo tính cho **gốc hệ O4** (không offset mỏ hàn), q4 = 0.

```
FK:  x = 0.25 c1 + 0.105 s1 + 0.262 c1 c3
     y = 0.25 s1 − 0.105 c1 + 0.262 s1 c3
     z = q2 + 0.262 s3 + 0.3015
IK:  q1 = atan2(0.105, √(x²+y²−0.105²)) + atan2(y, x)
     q3 = atan2(√(1−D²), D),  D = (x c1 + y s1 − 0.25)/0.262
     q2 = z − 0.262 s3 − 0.3015
```

Đã kiểm chứng: FK này trùng với URDF (trục q3 nằm ngang, (0,−1,0) trong hệ world).

## Chu trình hàn hiện tại

HOME (0.1518, −0.2730, 0.7698) → B (0.1589, −0.1543, 0.7782) → dừng 0.5 s → hàn xuống A (0.1589, −0.1543, 0.5782)
với 0.01 m/s → dừng 1.0 s → về HOME. LSPB: chặng di chuyển Vmax 0.05, Amax 0.1; chặng hàn Vmax 0.01, Amax 0.05; DT 0.01.
Tổng **29.60 s, 2961 điểm**. Biến khớp: HOME (−0.7205, 0.2101, 1.4013), B (−0.2768, 0.2205, 1.7822), A (−0.2768, 0.0205, 1.7822).

## Lịch sử công việc (các commit)

1. `d5d4d65` — đẩy workspace ROS 2 lên (đã bỏ build/install/log và file cache VS Code 1.6 GB).
2. `4e1401d` — thêm toàn bộ bài tập lớn, sắp xếp thành các thư mục đánh số; bỏ `.venv`, `__pycache__`, `.idea`, `slprj`, `.slxc`.
3. `195f216` — **dọn code thuần túy, không đổi hành vi**. Kiểm chứng: AST Python (bỏ docstring) giống hệt bản gốc,
   31 file đầu ra (stdout, ảnh, txt, npy) giống từng byte, URDF/YAML parse giống hệt, dòng lệnh MATLAB giống hệt.
   Khác biệt duy nhất: bỏ `import Axes3D` không dùng trong `animation.py`.
4. `3ff6132` — **sửa động học ngược**, gỡ báo cáo PDF:
   - Trước đó code dùng IK kiểu SCARA phẳng (`q2 = z − D1 − D4`, định lý cos trong mặt phẳng XY), còn tự mâu thuẫn
     (định lý cos dùng D3 nhưng q1 dùng D4; `animation.py` thì dùng D3 cả hai). IK này không khớp mô hình DH/URDF:
     trong Gazebo mỏ hàn lệch khoảng 32 cm so với điểm thiết kế cũ (HOME 0.287/0/0.6895, B 0.2/0.15/0.7, A 0.2/0.15/0.5).
     Đoạn hàn trong Gazebo vẫn thẳng đứng 20 cm vì khi hàn chỉ có q2 chạy.
   - Báo cáo (Chương 2) giải IK **đúng**, nhưng với điểm A cũ thì q2 = −0.0625 → A cũ nằm ngoài tầm với.
   - Người dùng chọn "sửa theo mô phỏng": thay IK đúng vào `JointSpace.py`, `animation.py`, `trajectory_client.py`;
     điểm mốc mới = vị trí robot thực sự đạt được trong Gazebo cũ, nên tư thế đầu và đoạn hàn trong Gazebo giữ nguyên,
     chỉ đoạn di chuyển Home↔B đổi (giờ là đường thẳng thật). Đã sinh lại `Cartesian_Waypoints.txt` và 3 file `.mat`.
   - Kiểm chứng: FK(IK(P)) sai số ~1e-16 trên cả quỹ đạo; khớp trong giới hạn; vận tốc khớp max (0.23, 0.05, 0.17)
     < giới hạn URDF (0.5, 0.4, 0.5); không bước nhảy; mọi script chạy không lỗi (node ROS 2 chạy với rclpy giả lập).

## Việc còn dở / cần làm trên Ubuntu

- [ ] **Chạy thử Gazebo** với IK mới (chưa từng chạy thật): `colcon build` trong `05_mo_phong_ros2/rprr_ws`,
      `ros2 launch robotic_arm_bringup gazebo.launch.py`, rồi `ros2 run robotic_arm_control trajectory_client.py`.
      Kỳ vọng: giống mô phỏng cũ, mỏ hàn hàn đoạn thẳng đứng 20 cm.
- [ ] Chạy lại Simulink nếu có MATLAB (StopTime trong .slx = 35 s, đủ cho 29.60 s).
- [ ] Báo cáo PDF (có mã số sinh viên) **vẫn còn trong lịch sử git** (commit `4e1401d`, `195f216`). Người dùng chưa
      quyết định có xóa hẳn hay không (cần viết lại lịch sử + force push → phải hỏi trước).

## Vấn đề đã phát hiện nhưng CHƯA sửa (chờ người dùng quyết)

- `robotic_arm_gazebo.xacro` tắt trọng lực cho link `gripper` không tồn tại (link cuối tên `end_effector`).
- `package.xml`: maintainer `thinkpad@todo.todo`, license `TODO`.
- `test.py` thực chất là script xuất waypoint; có thể đổi tên (vd. `export_waypoints.py`).
- Workspace (`main.py`) tính cho đầu mỏ hàn có offset, còn quỹ đạo tính cho gốc O4 — hai "điểm cuối" khác nhau.
- `JointSpace.py`: `draw_dwell_zones` tính mốc tô vùng dừng từ `t1[-1]`, `t2[-1]` nên lệch 1–2 mẫu (0.01–0.02 s) so với
  thời gian ghép thật (có cộng thêm DT mỗi đoạn); chỉ ảnh hưởng hiển thị, có từ code gốc.

## Ghi chú kỹ thuật

- Trên Windows, git dùng autocrlf; repo lưu LF.
- Cách kiểm chứng "không đổi hành vi" đã dùng: so `ast.dump` sau khi bỏ docstring; chạy script với backend Agg,
  patch `plt.show` để lưu hình, so byte đầu ra.
