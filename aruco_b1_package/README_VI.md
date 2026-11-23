# ArUco B1 (RGB-only) → map → AMCL

## Cấu hình mặc định

- ArUco ID `0`, từ điển `DICT_4X4_50`, **cạnh vuông đen 0.15 m**.
- Tâm marker trong `slot_B1`: `[0.0, 0.0, 0.115]` mét (11,5 cm trên sàn). Giá trị này áp dụng khi marker đúng tâm ô và `slot_B1` có Z=0 ở sàn như TF hiện tại. Nếu dán lệch gốc thì sửa X/Y/Z theo đo thật.
- Marker dựng đứng: X sang phải, Y lên, Z hướng về robot; slot_B1 X đi vào kệ, Y trái, Z lên.
- Dùng RGB `/color/image_raw`, `/color/camera_info` **có K và distortion D**. Không dùng Depth.
- SEARCH 1.5 Hz → TRACK 8 Hz khi thấy ID0; TRACK thêm 2 giây khi mất marker.
- Publishes `/aruco_b1/pose` (`PoseStamped`, planar pose trên map), `/aruco_b1/comparison` (JSON), `/aruco_b1/status` (JSON), and `/aruco_b1/debug_image` (BGR8, capped at 2 Hz).
- Broadcasts fixed `slot_B1 → aruco_B1` and dynamic `map → base_footprint_aruco`; never writes `map → odom`.
- `real_bringup.launch.py` bật `auto_update_amcl` mặc định; launch ArUco độc lập giữ chế độ quan sát. Chỉ gửi `/initialpose` khi có ít nhất 6 ảnh RGB mang timestamp khác nhau trong 6 giây, pose còn mới và ổn định, odometry xác nhận robot đứng yên ít nhất 2,5 giây, và AMCL lệch từ 5 cm hoặc 3°. Sai lệch trên 30 cm hoặc 15° yêu cầu ít nhất 10 ảnh trong 12 giây, reprojection ≤0,8 px, spread ≤4 cm/2° và Nav2 xác nhận không còn goal điều khiển. Các ngưỡng 30 cm/15° không chặn relocalization lớn.
- Sau một lần gửi, node đợi `/amcl_pose` phản ánh pose mới; nếu không xác nhận trong 5 giây thì chờ 30 giây trước khi xét một chuỗi ảnh mới. Một frame/timestamp đã dùng không được gửi lại. Khi mất marker, không gửi cập nhật mới; AMCL tiếp tục dùng LiDAR và odometry. Không gửi Emergency Stop.
- Pose tuyệt đối dùng trực tiếp cạnh `/tf_static` `map → slot_B1`; nếu cạnh này thiếu hoặc có hai giá trị xung đột thì không hiệu chỉnh. TF camera vẫn dùng cấu hình hiện tại, tra tại timestamp ảnh. Node không phát `map → odom`.
- `real_bringup.launch.py` chỉ chạy phần cứng và ArUco; để hiệu chỉnh hoạt động cần AMCL/Nav2 đang active cùng TF `map → slot_B1` từ `rack_tf_publisher`. Tắt nhanh bằng `aruco_auto_update_amcl:=false`.

## Cấu trúc package

- `aruco_b1_package/`: mã Python node.
- `launch/`: launch file ROS 2.
- `test/`: kiểm tra hình học offline.
- `config/`: chưa cần; vị trí marker B1 được truyền bằng ROS parameters.

## Build trong project_ws

Package này tự cài node và launch file qua CMake; không cần chép file vào `amr_perception`.

Build:

```bash
cd ~/project_ws
colcon build --packages-select aruco_b1_package
source /opt/ros/humble/setup.bash
source ~/project_ws/install/setup.bash
```

## Kiểm tra bắt buộc trước khi chạy tự cập nhật AMCL

```bash
ros2 run tf2_ros tf2_echo map slot_B1
ros2 run tf2_ros tf2_echo base_footprint camera_color_optical_frame
ros2 topic echo /color/camera_info --once
```

Nếu `map → slot_B1` không tồn tại thì phải bật node `rack_tf_publisher_node` tương ứng; **không lấy TF marker dựa trên một tọa độ chưa được gắn chính xác trên map**. Đảm bảo TF này không phụ thuộc `base_link` hay vòng lặp với localization.

Chạy ở chế độ QUAN SÁT:

```bash
ros2 launch aruco_b1_package aruco_b1.launch.py marker_x:=0.0 marker_y:=0.0 marker_z:=0.115
```

Xem kết quả:

```bash
ros2 topic echo /aruco_b1/status
ros2 topic echo /aruco_b1/pose --once
ros2 topic echo /aruco_b1/comparison
ros2 run rqt_image_view rqt_image_view /aruco_b1/debug_image
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo map base_footprint_aruco
ros2 run tf2_ros tf2_echo slot_B1 aruco_B1
```

Đặt robot đứng yên tại vài vị trí và so sánh X/Y/yaw được ước lượng với AMCL và với số đo thực tế. Đặc biệt xác nhận `marker_xyz_slot_m`, cạnh static trực tiếp `map → slot_B1`, TF `base_footprint → camera_color_optical_frame`, mã 4x4 ID 0 và kích thước cạnh đen thực 15 cm. `/aruco_b1/status` có `marker_detected`, `confirmed`, `sample_age_s`, `quality`, `delta_xy_m`, `delta_yaw_deg`, `relocalization_attempted`, `relocalization_accepted`, `rejection_reason` và `last_correction_time` (giây theo ROS clock).

Để xem đồng thời các frame trong RViz2, mở cấu hình `navigation_real.rviz`; các axes AMCL, ArUco và marker B1 đã được thêm làm display riêng.

## Giới hạn

- Unit tests và build không xác nhận chất lượng camera, vị trí marker thực tế hay hành vi AMCL/Nav2 trên robot. Trước khi bật tự hiệu chỉnh lớn, đo lại vị trí marker và đối chiếu pose tại vài vị trí đứng yên.
- Có `cv2.aruco` (OpenCV contrib) là điều kiện tiên quyết; dùng cùng loại dictionary in trên marker.
- `/initialpose` là tái khởi tạo bộ lọc AMCL, **không phải sensor fusion liên tục**. Nếu cần fusion liên tục cần thiết kế bộ lọc sensor fusion khác.
- Một mã ArUco phẳng có thể gây nghiệm PnP mơ hồ. Node lọc reprojection + giới hạn tư thế xe + nhiều khung hình nhưng không loại bỏ mọi sai số hệ thống.
