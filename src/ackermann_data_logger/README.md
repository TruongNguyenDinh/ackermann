# Ghi dữ liệu xe Ackermann

Package ROS 2 Humble này ghi **phiên chạy của xe thật** để tạo dữ liệu huấn luyện điều hướng về sau. Launch gọi `ros2 bag record` với danh sách topic trong [`config/topics.yaml`](config/topics.yaml), tạo manifest cho phiên chạy và có thể sao chép map 2D đã lưu. Package không khởi chạy Nav2, không điều khiển xe, không huấn luyện AI và không cần Gazebo.

## Cách ghi một phiên

Chạy trong container ROS 2 khi cảm biến, định vị và bộ điều khiển xe đã hoạt động:

```bash
source /opt/ros/humble/setup.bash
cd /home/rosdev/ros2_ws
colcon build --packages-select ackermann_data_logger
source install/setup.bash
ros2 launch ackermann_data_logger record.launch.py \
  map_yaml:=/home/rosdev/ros2_ws/maps/my_map.yaml
```

Đường dẫn map trong ví dụ phải được thay bằng file map thực tế; bỏ `map_yaml` nếu chưa có. Nhấn Ctrl+C để đóng bag đúng cách. Mặc định dữ liệu nằm trong `/home/rosdev/ros2_ws/src/ackermann_data_logger/recordings`, thuộc thư mục đang mount từ máy chủ Docker và đã được loại khỏi Git. Trên xe thật có thể dùng `output_dir:=/data/robot_runs`. `session_name:=run_001` đặt tên phiên; package từ chối ghi đè phiên cũ. `topics_file:=/đường/dẫn/topics.yaml` chọn danh sách topic khác.

Mỗi phiên gồm `session.yaml`, thư mục `bag/` chứa dữ liệu SQLite và metadata rosbag2, cùng `map/map.yaml` và ảnh map nếu truyền `map_yaml`. Bản sao map giữ độ phân giải, gốc tọa độ và ngưỡng occupancy. Topic `/map` cũng được yêu cầu ghi để giữ các bản cập nhật map được phát trong phiên. Xem dữ liệu sau khi dừng:

```bash
ros2 bag info /home/rosdev/ros2_ws/src/ackermann_data_logger/recordings/<session>/bag
```

Rosbag2 tiếp tục dò topic sau khi khởi chạy. Topic được yêu cầu nhưng chưa có publisher sẽ không có bản ghi; kiểm tra `ros2 bag info` trước khi dùng phiên đó để huấn luyện.

## Tín hiệu được ghi và hiện trạng

| Dữ liệu | Nguồn | Lưu ý |
| --- | --- | --- |
| LiDAR 2D | `/scan` | Driver cần phát `sensor_msgs/LaserScan` trong `laser_frame`. |
| IMU | `/imu/data` | Đầu ra dự kiến của MCU bridge; hiện chưa có node bridge trong dự án. |
| Encoder sau | `/joint_states` | Góc và tốc độ bánh là giá trị suy ra. Count tích lũy thô cần topic MCU riêng sau này. |
| Chuyển động, TF | `/odom`, `/tf`, `/tf_static` | Khôi phục `map → base_footprint` từ TF; riêng `/odom` chỉ ở hệ `odom`. |
| Map tĩnh | `/map`, bản sao map tùy chọn | Bản sao xác định chính xác map dùng trong phiên. |
| Goal, global path | `/goal_pose`, `/plan` | `/goal_pose` chỉ được ghi nếu có publisher. Điểm cuối `/plan` giúp suy ra goal đã lập được đường. Action status/feedback không chứa đầy đủ mọi goal được yêu cầu. |
| Lệnh Nav2 và lệnh đã làm mượt | `/cmd_vel_nav`, `/cmd_vel` | Đơn vị m/s và rad/s; chưa phải lệnh chuẩn hóa gửi MCU. |
| Lệnh sau động học | `/steering_controller/commands`, `/wheel_velocity_controller/commands` | Là góc/tốc độ *yêu cầu*, không phải phản hồi góc servo thực tế. |

[`ackermann_mcu_bridge`](../ackermann_mcu_bridge/README.md) hiện chỉ có khung package và tài liệu. Vì thế IMU và count thô chỉ có dữ liệu sau khi firmware và bridge được triển khai. Dự án hiện cũng chưa có topic được xác nhận cho goal nhận qua Nav2 action, nguồn điều khiển `nav2`/`teleop`, cặp lệnh chuẩn hóa `[-1, 1]`, hay lệnh MCU thực nhận. Khi các giao diện đó được định nghĩa, thêm **tên topic thực tế** vào `config/topics.yaml`. Không gán nhãn `/cmd_vel` là góc lái đo được hoặc lệnh MCU thực thi.

## Kế hoạch tạo dữ liệu và học

Bag là bản ghi gốc. Bộ xuất dữ liệu trong tương lai cần đồng bộ timestamp, loại mẫu cảm biến/TF lỗi, rồi tạo các mẫu gồm chuỗi LiDAR, IMU, encoder ngắn; mã map hoặc vùng map quanh xe; `(x, y, yaw)` trong `map`; goal; path tùy chọn; lệnh trước đó; lệnh đang chọn; nguồn `nav2`/`teleop`; và trạng thái xe ở thời điểm tiếp theo. Tách tập huấn luyện và đánh giá theo **phiên chạy**, tránh chia các frame liên tiếp của cùng phiên vào hai tập.

Bước đầu có thể học bắt chước lệnh lái bằng một bộ mã hóa LiDAR gọn và mạng nhỏ nhận map, pose, goal, lịch sử chuyển động để dự đoán tốc độ/góc lái chuẩn hóa. Sau đó có thể nghiên cứu mô hình chuỗi thời gian và các chuyển tiếp `(state_t, action_t, state_t+1)` để học phản ứng thực tế của xe. Việc ghi `/plan` cho phép so sánh cách học trực tiếp từ map + goal với cách dùng path tham khảo. Đánh giá bằng tỷ lệ tới goal, va chạm/can thiệp, độ lệch đường và độ mượt lệnh. Định vị và điều khiển an toàn khi chạy độc lập ROS 2 là các phần triển khai tương lai; package này chỉ thu dữ liệu.
