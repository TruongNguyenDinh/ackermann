# ackermann_mcu_bridge

Package C++ (`ament_cmake`) dự kiến nhận dữ liệu từ ESP32-S3 Nano trên robot thật. **Hiện mới có khung package và tài liệu; chưa có node, giao thức truyền hay executable.**

## Phần cứng và phân công

| Thiết bị | Vai trò trong luồng dữ liệu |
| --- | --- |
| ESP32-S3 Nano | Đọc encoder hai động cơ bánh sau và ICM-20948, đóng dấu thời gian, gửi mẫu đo tới Raspberry Pi. |
| ICM-20948 | Đo gyro 3 trục, gia tốc 3 trục và từ trường 3 trục. Yaw/quaternion chỉ có khi firmware hoặc Pi **thực hiện và hiệu chuẩn** thuật toán ước lượng hướng. |
| Hai động cơ Wheeltec MD36P27 | Mỗi encoder hai kênh được đếm X2. Theo thông số **bạn cung cấp**, 27.000 count tương ứng một vòng **trục bánh xe**. |
| Servo Wheeltec S20F | Nhận lệnh lái; **không phản hồi góc thực tế**. |
| RPLIDAR A1M8-R6 | Nên nối qua adapter phù hợp tới Pi và dùng driver LiDAR ROS 2 riêng để phát `/scan`; không đưa dữ liệu quét vào gói MCU nếu không có yêu cầu khác. |
| Raspberry Pi 4 Model B 8 GB | Chạy ROS 2 Humble, nhận dữ liệu MCU, tính odometry, chạy driver LiDAR và Nav2. |

## MCU gửi gì cho Pi?

Một mẫu encoder và IMU nên cùng thời điểm. Nếu tốc độ lấy mẫu khác nhau, gửi timestamp **riêng cho từng mẫu**. Các giá trị nên là dữ liệu đo, không phải tốc độ hoặc góc lái đã ra lệnh.

| Trường đề xuất | Kiểu và đơn vị | Mức độ | Mục đích |
| --- | --- | --- | --- |
| `sample_time_us` | `uint64`, µs đơn điệu từ lúc ESP32 khởi động | Bắt buộc | Ghép mẫu, tính `Δt`; không phải ROS/Unix time. |
| `rear_left_ticks`, `rear_right_ticks` | số nguyên có dấu, count tích lũy tại bánh | Bắt buộc | Tính quãng đường hai bánh sau; chiều tăng phải được hiệu chuẩn. Nên mở rộng bộ đếm lên 64 bit hoặc ghi rõ cách xử lý tràn. |
| `gyro_x/y/z` | `float32`, rad/s | Bắt buộc cho IMU | Đo vận tốc quay; đặc biệt `gyro_z` để theo dõi yaw sau khi đổi sang hệ trục robot. |
| `accel_x/y/z` | `float32`, m/s² | Bắt buộc cho IMU | Cung cấp gia tốc cho bộ lọc IMU; không lấy `accel_z` làm yaw. |
| `mag_x/y/z` | `float32`, µT | Nên có nếu đọc magnetometer | Hiệu chỉnh hướng dài hạn **chỉ khi** đã hiệu chuẩn và kiểm tra nhiễu từ từ motor, dây nguồn, khung xe. |
| `imu_valid`, `mag_valid` | bool hoặc bit trạng thái | Nên có | Phân biệt mẫu hợp lệ với cảm biến mất kết nối/chưa hiệu chuẩn. |
| `boot_id`, `seq` | số nguyên tăng/đổi theo lần khởi động | Nên có | Phát hiện reset MCU, gói mất hoặc đến sai thứ tự để không tích phân nhầm count. |
| `estop_active`, `fault_bits` | bool, bitmask | Nên có nếu mạch hỗ trợ | Báo trạng thái an toàn và lỗi driver/nguồn; không giả lập trạng thái mà phần cứng không đo được. |

**Không cần gửi** tốc độ bánh, tốc độ thân xe hoặc vị trí `x,y`: Pi tính chúng từ dữ liệu đo. ESP32 **không thể gửi góc servo thực tế** khi servo không có phản hồi góc. Nếu firmware đã có quaternion/yaw do thuật toán hợp nhất IMU tạo ra, có thể gửi **thêm** quaternion và trạng thái hiệu chuẩn; không gán quaternion từ gyro thô. Pi đã biết lệnh servo nó gửi, nhưng đó là **góc yêu cầu**, không phải góc đo.

Định dạng nhị phân/JSON, UART/USB/CAN, framing, checksum và tần số gửi **chưa chốt**. ESP32 cần timestamp tại lúc lấy mẫu, không phải lúc hoàn tất truyền gói.

## Bảng ví dụ hai mẫu liên tiếp

Các giá trị đo dưới đây **chỉ minh họa cấu trúc gói**, không phải số đo trên xe. Hai hàng có cùng `boot_id`; mỗi hàng là một gói.

| Trường | Gói A | Gói B | Đơn vị |
| --- | ---: | ---: | --- |
| `boot_id` / `seq` | `7` / `100` | `7` / `101` | — |
| `sample_time_us` | `12_500_000` | `12_520_000` | µs |
| `rear_left_ticks` | `10_000` | `10_270` | count |
| `rear_right_ticks` | `10_000` | `10_280` | count |
| `gyro_x/y/z` | `0, 0, 0.0235` | `0, 0, 0.0235` | rad/s |
| `accel_x/y/z` | `0, 0, 9.81` | `0, 0, 9.81` | m/s², ví dụ chiều trục |
| `mag_x/y/z` / `mag_valid` | `20, -5, 40` / `true` | `20, -5, 40` / `true` | µT / bool, nếu dùng |

## Pi suy ra vị trí và so sánh góc thế nào?

Đường kính bánh bạn cung cấp: **126 mm = 0,126 m**. Với **27.000 count/vòng bánh**, một count tương ứng `π × 0,126 / 27000 ≈ 0,00001466 m` (0,01466 mm). Cần xác nhận con số 27.000 đã tính đúng tỉ số truyền và chế độ X2 trên **cả hai bánh**.

```text
Δs_L = Δcount_L × π × 0,126 / 27000          [m]
Δs_R = Δcount_R × π × 0,126 / 27000          [m]
Δs   = (Δs_L + Δs_R) / 2                    [m]
Δyaw_encoder ≈ (Δs_R - Δs_L) / rear_track   [rad]
Δyaw_imu ≈ ∫ gyro_z dt                     [rad, sau hiệu chuẩn và đổi trục]
x_mới = x_cũ + Δs × cos(yaw_cũ + Δyaw/2)
y_mới = y_cũ + Δs × sin(yaw_cũ + Δyaw/2)
```

Với hai gói ví dụ: `Δt = 0,020 s`, bánh trái đi khoảng **3,958 mm**, bánh phải **4,105 mm**; `Δs ≈ 4,032 mm`. Dùng `rear_track = 0,3118 m` **đang cấu hình trong dự án** (cần đối chiếu kích thước xe thật), `Δyaw_encoder ≈ 0,000470 rad`; tích phân gyro z ví dụ cho `Δyaw_imu ≈ 0,000470 rad`. So sánh **độ đổi hướng** hai nguồn sau khi đồng bộ thời gian, quy đổi đúng trục và hiệu chuẩn bias; lệch nhau có thể do trượt bánh, nhiễu IMU hoặc sai thông số. Đây là phép kiểm tra chéo, **không chứng minh góc servo thực tế**.

Odometry dùng `base_footprint` tại tâm trục sau. Yaw từ gyro tích phân sẽ trôi theo thời gian; magnetometer có thể bị nhiễu gần động cơ. Muốn hướng ổn định dài hạn cần bộ lọc và/hoặc hiệu chỉnh định vị bằng LiDAR. Không lấy góc lái từ IMU khi xe đứng yên: IMU chỉ đo chuyển động thân xe. Khi xe chạy đủ quãng đường, có thể ước lượng **góc lái tương đương** `atan(wheelbase × Δyaw/Δs)`; đó không phải phản hồi góc của servo. So sánh góc lái yêu cầu với yaw đo được cần mô hình động học, vận tốc và thông số servo đã hiệu chuẩn.

## Ánh xạ ROS 2 và tình trạng dự án

| Nguồn | Topic/kiểu ROS dự kiến | Ghi chú |
| --- | --- | --- |
| Hai encoder | `/joint_states` — `sensor_msgs/msg/JointState` | Tên đúng `rear_left_wheel_joint`, `rear_right_wheel_joint`; `position = 2π × (count - count_mốc)/27000` rad, `velocity = 2π × Δcount/(27000 × Δt)` rad/s. Cả hai bánh phải cùng một mẫu. |
| ICM-20948 | `/imu/data` — `sensor_msgs/msg/Imu` | `frame_id = imu_link`; gyro rad/s, accel m/s². Không công bố orientation như dữ liệu hợp lệ nếu chưa có thuật toán ước lượng hướng. Magnetometer có thể dùng topic riêng khi cần. |
| Bộ tích hợp encoder + IMU | `/odom` và `odom → base_footprint` | Chỉ **một** node được sở hữu TF; nếu dùng bộ lọc mới, cần tắt TF từ nguồn odometry cũ. |
| Driver RPLIDAR trên Pi | `/scan` — `sensor_msgs/msg/LaserScan` | `frame_id = laser_frame`; độc lập với MCU. |

MCU **không cần gửi tốc độ riêng**. Pi vẫn tính vận tốc từ `Δcount/Δt` và yaw rate cho `/joint_states`, `/odom` và phản hồi Nav2; có thể hiển thị các giá trị này trên web sau này.

**Hiện trạng:** node [`ackermann_odometry_cpp`](../ackermann_odometry_cpp/README.md) chỉ đọc vận tốc hai bánh sau từ `/joint_states`, tự suy ra yaw từ encoder và **chưa hợp nhất IMU**. README này mô tả giao diện MCU tương lai; không tuyên bố hệ thống hiện tại đã thực hiện tích hợp trên. MCU uptime cũng không được chép thẳng vào ROS `header.stamp`: bridge phải ánh xạ sang ROS time, phát hiện reset/tràn count, bỏ gói cũ và `Δt` không hợp lệ.

## Cần xác nhận trước khi viết node/firmware

- Cổng kết nối ESP32 ↔ Pi và cấu trúc gói; tần số lấy mẫu encoder/IMU, cơ chế đồng bộ timestamp.
- Dấu hai encoder khi xe chạy tiến, số count X2 thực tế/vòng bánh, giới hạn/tràn bộ đếm.
- Hướng lắp ICM-20948 so với `imu_link`, dải đo, hiệu chuẩn gyro/accelerometer/magnetometer; IMU sẽ gửi raw hay quaternion đã hợp nhất.
- Vị trí lắp IMU và LiDAR thực tế so với `base_footprint` để cập nhật mô tả robot.
- Định nghĩa lỗi/E-stop thực sự có từ phần cứng; đặc tính và vùng chết của servo không phản hồi.
- Node duy nhất phát `odom → base_footprint` khi hợp nhất IMU và encoder.

## Build khung package

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select ackermann_mcu_bridge
```

Chưa có lệnh `ros2 run` vì package chưa có executable.

## Tài liệu phần cứng và driver

- [TDK ICM-20948](https://www.invensense.tdk.com/en-us/products/9-axis/icm-20948): gyro, accelerometer, compass và DMP.
- [Slamtec ROS 2 driver](https://github.com/Slamtec/sllidar_ros2): hỗ trợ RPLIDAR A1 trên ROS 2.
