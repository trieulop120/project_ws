# ArUco B1 (RGB-only) → map → AMCL

## Cấu hình mặc định

- ArUco ID `0`, từ điển `DICT_4X4_50`, **cạnh vuông đen 0.15 m**.
- Tâm marker trong `slot_B1`: `[0.0, 0.0, 0.115]` mét (11,5 cm trên sàn). Giá trị này áp dụng khi marker đúng tâm ô và `slot_B1` có Z=0 ở sàn như TF hiện tại. Nếu dán lệch gốc thì sửa X/Y/Z theo đo thật.
- Marker dựng đứng: X sang phải, Y lên, Z hướng về robot; slot_B1 X đi vào kệ, Y trái, Z lên.
- Dùng RGB `/color/image_raw`, `/color/camera_info` **có K và distortion D**. Không dùng Depth.
- Launch robot thật dùng SEARCH 2 Hz và TRACK 8 Hz. Launch quan sát độc lập dùng SEARCH 1,5 Hz. TRACK giữ thêm 1,5 giây sau khi mất marker.
- Publishes `/aruco_b1/pose` (`PoseStamped`, planar pose trên map), `/aruco_b1/comparison` (JSON), `/aruco_b1/status` (JSON), and `/aruco_b1/debug_image` (BGR8, capped at 2 Hz).
- Broadcasts fixed `slot_B1 → aruco_B1` and dynamic `map → base_footprint_aruco`; never writes `map → odom`.
- Launch robot thật bật `auto_update_amcl` và nhận grant có chủ sở hữu từ `/aruco_b1/route`. Supervisor phát cùng quyết định trên `/aruco_b1/enable` cho client cũ. Grant placement gắn với session và operation PICK đang giữ pallet. Retrieval dùng grant riêng. Dữ liệu grant hết hạn sau 3 s.
- Cần ID 0, hình học hợp lệ và TF camera/odom tại timestamp ảnh. Cửa sổ thường cần 4 ảnh khác timestamp trong 0,9 s, ảnh cuối mới ≤0,8 s, spread ≤10 cm/6° và reprojection ≤1,5 px. Sai lệch trên 30 cm hoặc 15° yêu cầu 6 ảnh trong 1,2 s, spread ≤7,5 cm/4° và reprojection ≤1,2 px. Node so sánh các ước lượng `map → odom` để bù chuyển động giữa các ảnh.
- Khi vận tốc vượt 0,35 m/s hoặc 0,60 rad/s, node hoãn gửi. Nav2 active cũng kích hoạt kiểm tra TF odom mới ≤0,12 s. Trần `nav_active_max_correction_*` (15 cm/5°) áp dụng lên bước gửi. Với cấu hình thật, trần nhỏ hơn `moving_anchor_max_step_*` chi phối, tối đa **5 cm/2° mỗi lần**. Hai tham số `moving_large_max_*` hiện chỉ được khai báo.
- Riêng placement sau PICK, sai lệch từ **2 cm hoặc 1°** được xét lại (`placement_anchor_min_xy_m`, `placement_anchor_min_yaw_deg`, trước đây 5 cm/3°). Khoảng cách tối thiểu giữa hai lần gửi và thời gian nghỉ sau ACK cùng giảm **5 → 1,5 s** (`placement_anchor_min_interval_s`, `placement_anchor_ack_cooldown_s`). Ở TRACK 8 Hz khoảng này đủ cho một cửa sổ 4–6 ảnh mới. Retrieval giữ 5 cm/3° và 5 s. Timeout ACK vẫn 5 s, sau timeout nghỉ 30 s.
- Trong `/aruco_b1/status`, `marker_detected` cho biết thấy ID, `confirmed` nói về độ ổn định của các ảnh. `placement_anchor.state=pending` nghĩa là đã gửi bước. Chỉ `acknowledged` cho biết pose AMCL sau yêu cầu tiến sát bước đã gửi, với odom tra tại timestamp bản tin AMCL. Tắt gate dừng xử lý ảnh ngay, cho phép kiểm tra nốt ACK của placement đến hạn. Bản ghi xác nhận chứa timestamp ảnh, timestamp ACK và phép biến đổi đo từ AMCL/odom ở thời điểm ACK. D_1 dùng bản ghi này khi còn mới và cùng lượt.
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
