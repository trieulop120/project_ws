# ArUco B1 (RGB-only) → map → AMCL

## Cấu hình mặc định

- ArUco ID `0`, từ điển `DICT_4X4_50`, **cạnh vuông đen 0.10 m**.
- Tâm marker trong `slot_B1`: `[0.0, 0.0, 0.122]` mét; chỉ đúng nếu marker đúng tâm ô và `slot_B1` có Z=0 ở sàn. Nếu dán lệch gốc thì sửa X/Y/Z theo đo thật, không lấy khoảng cách tới sàn làm Z nếu `slot_B1` không ở cao độ 0.
- Marker dựng đứng: X sang phải, Y lên, Z hướng về robot; slot_B1 X đi vào kệ, Y trái, Z lên.
- Dùng RGB `/color/image_raw`, `/color/camera_info` **có K và distortion D**. Không dùng Depth.
- SEARCH 1.5 Hz → TRACK 8 Hz khi thấy ID0; TRACK thêm 2 giây khi mất marker.
- Publishes `/aruco_b1/pose` (`PoseStamped`, planar pose trên map), `/aruco_b1/comparison` (JSON), `/aruco_b1/status` (JSON), and `/aruco_b1/debug_image` (BGR8, capped at 2 Hz).
- Broadcasts fixed `slot_B1 → aruco_B1` and dynamic `map → base_footprint_aruco`; never writes `/initialpose` or `map → odom`.
- `auto_update_amcl=False` by default. The node is observe-only and does not publish `/initialpose`.

## Cấu trúc package

- `aruco_b1_package/`: mã Python node.
- `launch/`: launch file ROS 2.
- `test/`: kiểm tra hình học offline.
- `config/`: chưa cần; vị trí marker B1 được truyền bằng ROS parameters.

## Build trong project_ws

```bash
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
ros2 launch aruco_b1_package aruco_b1.launch.py marker_x:=0.0 marker_y:=0.0 marker_z:=0.122
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

Đặt robot đứng yên tại vài vị trí và so sánh X/Y/yaw được ước lượng với AMCL và với số đo thực tế. Đặc biệt xác nhận `marker_xyz_slot_m`, TF `map → slot_B1`, TF `base_footprint → camera_color_optical_frame`, mã 4x4 ID 0 và kích thước thực 10 cm.

Để xem đồng thời các frame trong RViz2, mở cấu hình `navigation_real.rviz`; các axes AMCL, ArUco và marker B1 đã được thêm làm display riêng.

## Giới hạn

- Chỉ kiểm tra cú pháp Python và test phép đảo transform với dữ liệu mô phỏng, chưa có phần cứng ROS 2 trong môi trường phát triển ở đây.
- Có `cv2.aruco` (OpenCV contrib) là điều kiện tiên quyết; dùng cùng loại dictionary in trên marker.
- `/initialpose` là tái khởi tạo bộ lọc AMCL, **không phải sensor fusion liên tục**. Nếu cần fusion liên tục cần thiết kế bộ lọc sensor fusion khác.
- Một mã ArUco phẳng có thể gây nghiệm PnP mơ hồ. Node lọc reprojection + giới hạn tư thế xe + nhiều khung hình nhưng không loại bỏ mọi sai số hệ thống.
