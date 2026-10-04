# Hướng dẫn chạy mô phỏng Simulink

Dự án này bao gồm mô hình Simulink và các tệp dữ liệu cần thiết. Để đảm bảo mô phỏng chạy chính xác, bạn vui lòng thực hiện đúng theo các bước sau:

## Các bước thực hiện

1. **Khởi tạo thông số (Quan trọng):**
   - Mở phần mềm MATLAB.
   - Điều hướng (Navigate) đến thư mục chứa các tệp dự án này.
   - Chạy tệp **`initialize.m`** bằng cách gõ lệnh sau vào Command Window hoặc click chuột phải vào file rồi chọn *Run*:
     ```matlab
     initialize
     ```
   - Bước này sẽ nạp các biến và tham số cần thiết vào *MATLAB Workspace*.

2. **Chạy mô hình Simulink:**
   - Sau khi đã chạy xong `initialize.m`, hãy mở tệp **`controller_simulation.slx`**.
   - Nhấn nút **Run** trong giao diện Simulink để bắt đầu mô phỏng.

---
**Lưu ý:** Nếu bạn không chạy `initialize.m` trước, mô hình Simulink sẽ báo lỗi do thiếu dữ liệu đầu vào hoặc tham số hệ thống chưa được định nghĩa.