#!/usr/bin/env python3
import math
import os
import serial
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, BatteryState, Range
from std_msgs.msg import String, Int8MultiArray
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
import tf2_ros


class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # --- PARAMETERS ---
        self.declare_parameter('port', '/dev/esp32')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('wheel_radius', 0.0509)    
        self.declare_parameter('wheelbase', 0.4813)         
        self.declare_parameter('counts_per_rev', 3960.0)  # 3960 xung/vòng
        self.declare_parameter('max_rpm', 110.0)           # Tốc độ tối đa 110 RPM
        self.declare_parameter('gyro_scale_z', 0.977)
        # He so hieu chinh quang duong cho ODOMETRY (thuc/do, trung binh 11 lan chay = 1.054).
        # Chi ap len dc (tinh tien); KHONG dung cho lenh cmd_vel->RPM va KHONG dung cho yaw
        # (chua co bai test xoay de hieu chinh wheelbase).
        self.declare_parameter('odom_scale', 1.054)
        self.declare_parameter('publish_tf', False)
        # HC-SR04 (firmware gui cm, -1.0 khi khong co echo; timeout 6000us ~ 1.0 m)
        self.declare_parameter('ultrasonic_frame', 'ultrasonic_link')
        self.declare_parameter('ultrasonic_min_range', 0.02)
        self.declare_parameter('ultrasonic_max_range', 1.0)
        self.declare_parameter('ultrasonic_fov', 0.26)
        self.declare_parameter('shelf_topic', '/shelf/occupancy')
        self.declare_parameter('shelf_timeout', 1.0)     # s, khong co PKT -> bao mat lien lac
        self.declare_parameter('shelf_heartbeat', 1.0)
        self.declare_parameter('joint_names', ['front_encoder_wheel_left_joint', 'front_encoder_wheel_right_joint', 'encoder_lift_joint'])

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.cpr = self.get_parameter('counts_per_rev').value
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.gyro_scale_z = float(self.get_parameter('gyro_scale_z').value)
        self.odom_scale = float(self.get_parameter('odom_scale').value)
        self.publish_tf = self.get_parameter('publish_tf').value
        self.joint_names = self.get_parameter('joint_names').value
        self.us_frame = self.get_parameter('ultrasonic_frame').value
        self.us_min = float(self.get_parameter('ultrasonic_min_range').value)
        self.us_max = float(self.get_parameter('ultrasonic_max_range').value)
        self.us_fov = float(self.get_parameter('ultrasonic_fov').value)
        self.shelf_topic = self.get_parameter('shelf_topic').value
        self.shelf_timeout = float(self.get_parameter('shelf_timeout').value)
        self.shelf_heartbeat = float(self.get_parameter('shelf_heartbeat').value)
        self._warned_pkt_len = False

        # Biến lưu trữ dữ liệu phần cứng từ ESP32
        self.curr_el = 0
        self.curr_er = 0
        self.curr_em3 = 0
        self.curr_gz = 0.0
        self.curr_battery_percent = 0.0
        self.curr_dist_cm = -1.0   # -1.0 = khong co du lieu / khong co echo
        self.curr_shelf = None          # [1,0,0,0] hoac None neu NAN
        self.last_pkt_time = 0.0        # time.monotonic() cua PKT hop le gan nhat
        self.last_shelf_pub = 0.0
        self.last_shelf_sent = None
        self.data_lock = threading.Lock()

        self.ser = None
        if not self.connect_serial():
            self.get_logger().error('[FATAL] ESP32 Bridge startup FAILED. Serial not connected.')
            # Tiếp tục chạy để có thể debug, nhưng serial commands sẽ không hoạt động

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom_raw', 10)
        self.pub_battery = self.create_publisher(BatteryState, '/battery_state', 10)
        self.pub_ultrasonic = self.create_publisher(Range, '/ultrasonic/range', 10)
        shelf_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub_shelf = self.create_publisher(Int8MultiArray, self.shelf_topic, shelf_qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self) if self.publish_tf else None

        # Subscribers (ĐÃ SỬA: Lắng nghe /diff_drive/cmd_vel từ splitter)
        self.sub_drive = self.create_subscription(Twist, '/diff_drive/cmd_vel', self.drive_cb, 10)
        self.sub_lift = self.create_subscription(Twist, '/lift/cmd_vel', self.lift_cb, 10)
        self.sub_raw = self.create_subscription(String, '/esp32_cmd', self.raw_cb, 10)

        self.running = True
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_el = 0
        self.last_er = 0
        self.first_run = True
        self.last_time = self.get_clock().now()

        # (5) Lưu last commanded RPM để gửi liên tục mỗi 30Hz
        self.last_rpm_left = 0.0
        self.last_rpm_right = 0.0

        # Luồng đọc Serial riêng biệt
        threading.Thread(target=self.read_loop, daemon=True).start()

        # Timer cố định 30Hz
        self.timer = self.create_timer(1.0 / 100.0, self.publish_loop_100hz)
        self.get_logger().info(f'ESP32 Bridge Started (Port: {self.port}, Publish TF: {self.publish_tf}) - 30Hz Active')

    def check_symlink_conflict(self):
        rplidar_port = '/dev/rplidar'
        try:
            esp32_real = os.path.realpath(self.port)
            rplidar_real = os.path.realpath(rplidar_port) if os.path.exists(rplidar_port) else "NOT_EXIST"

            self.get_logger().info(f'[USB Check] {self.port} -> {esp32_real}')
            self.get_logger().info(f'[USB Check] {rplidar_port} -> {rplidar_real}')

            if esp32_real != "N/A" and esp32_real != "NOT_EXIST" and esp32_real == rplidar_real:
                self.get_logger().error('[CRITICAL] Symlink /dev/esp32 và /dev/rplidar bị trùng cổng vật lý!')
                return False
        except Exception:
            pass
        return True

    def connect_serial(self):
        """Kết nối serial với ESP32, có retry và chờ ESP32 sẵn sàng."""
        max_retries = 2
        connected = False

        for attempt in range(max_retries + 1):
            try:
                # Kiểm tra symlink conflict
                if not self.check_symlink_conflict():
                    self.get_logger().error('[FATAL] Xung đột Symlink USB!')
                    return False

                if not os.path.exists(self.port):
                    self.get_logger().error(f'[ERROR] Không tìm thấy cổng {self.port}!')
                    return False

                self.get_logger().info(f'[Attempt {attempt + 1}] Connecting to serial: {self.port}')

                # (1) Giữ timeout=0.1, không dùng 0
                # (2) Tắt DTR/RTS TRƯỚC KHI open() để ESP32 không bị reset
                self.ser = serial.Serial()
                self.ser.port = self.port
                self.ser.baudrate = self.baudrate
                self.ser.timeout = 0.1
                self.ser.dtr = False
                self.ser.rts = False
                self.ser.open()

                self.get_logger().info('Serial port opened (DTR/RTS disabled)')

                # (3) Chờ ESP32 báo sẵn sàng bằng cách đợi PKT telemetry
                if self.wait_for_esp32_ready(timeout=10.0):
                    # Sync buffer sau khi ESP32 sẵn sàng
                    self.ser.write(b'L0\nR0\n')
                    time.sleep(0.2)
                    if self.ser.in_waiting > 0:
                        self.ser.read(self.ser.in_waiting)

                    self.get_logger().info('Serial connection established successfully.')
                    connected = True
                    break
                else:
                    # Hết timeout
                    if attempt < max_retries:
                        self.get_logger().warn(f'ESP32 timeout, retrying ({attempt + 1}/{max_retries})...')
                        self.ser.close()
                        time.sleep(1.0)
                    else:
                        self.get_logger().error(
                            '[FATAL] ESP32 không phản hồi sau 2 retries (20s total). '
                            'Kiểm tra: (1) ESP32 có đang chạy firmware đúng không, '
                            '(2) baudrate có khớp không (921600), '
                            '(3) TX/RX wiring có đúng không.'
                        )
                        self.ser.close()
                        self.ser = None
                        return False

            except serial.SerialException as e:
                if attempt < max_retries:
                    self.get_logger().warn(f'Serial error: {e}, retrying ({attempt + 1}/{max_retries})...')
                    time.sleep(1.0)
                else:
                    self.get_logger().error(f'[FATAL] Serial connection failed after {max_retries + 1} attempts: {e}')
                    self.ser = None
                    return False

        return connected

    def wait_for_esp32_ready(self, timeout=10.0):
        """
        (3) Chờ ESP32 gửi telemetry PKT,... làm tín hiệu sẵn sàng.
        Đảm bảo read_loop chưa chạy (hoặc đã có dữ liệu) trước khi proceed.
        """
        start = time.time()
        self.get_logger().info(f'Waiting for ESP32 ready signal (PKT,...) within {timeout}s...')

        while time.time() - start < timeout:
            if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                if 'PKT,' in data:
                    self.get_logger().info('ESP32 ready! Received telemetry PKT,')
                    self.ser.reset_input_buffer()
                    return True
            time.sleep(0.05)  # Poll every 50ms

        self.get_logger().warn('Timeout waiting for ESP32 PKT telemetry!')
        return False

    def drive_cb(self, msg: Twist):
        """(5) Chỉ lưu target RPM, KHÔNG gửi ngay.
        Commands sẽ được gửi liên tục trong publish_loop_30hz."""
        vx, wz = msg.linear.x, msg.angular.z
        v_left = vx - wz * self.wheelbase / 2.0
        v_right = vx + wz * self.wheelbase / 2.0

        rpm_left = (v_left / (2.0 * math.pi * self.wheel_radius)) * 60.0
        rpm_right = (v_right / (2.0 * math.pi * self.wheel_radius)) * 60.0

        self.last_rpm_left = max(min(rpm_left, self.max_rpm), -self.max_rpm)
        self.last_rpm_right = max(min(rpm_right, self.max_rpm), -self.max_rpm)

    def _send_drive_commands(self):
        """Gửi lệnh L/R liên tục mỗi 30Hz (từ publish_loop)."""
        if not self.ser or not self.ser.is_open:
            return
        try:
            self.ser.write(f"L{self.last_rpm_left:.2f}\n".encode('utf-8'))
            self.ser.write(f"R{self.last_rpm_right:.2f}\n".encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Failed to send drive cmd: {e}')

    def lift_cb(self, msg: Twist):
        if not self.ser or not self.ser.is_open:
            return
        try:
            if abs(msg.angular.z) > 0.01:
                self.ser.write(f'M{msg.angular.z:.1f}\n'.encode('utf-8'))
            elif abs(msg.linear.z) > 0.001:
                self.ser.write(f'H{msg.linear.z * 100:.1f}\n'.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f'Failed to send lift cmd: {e}')

    def raw_cb(self, msg: String):
        if self.ser and self.ser.is_open:
            try:
                cmd = msg.data.strip().upper() + '\n'
                self.ser.write(cmd.encode('utf-8'))
                self.get_logger().info(f"Sent Serial Raw Command: {cmd.strip()}")
            except Exception as e:
                self.get_logger().error(f'Failed to send raw cmd: {e}')

    def read_loop(self):
        buf = ""
        while self.running:
            if self.ser and self.ser.is_open:
                try:
                    if self.ser.in_waiting > 0:
                        buf += self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                        while '\n' in buf:
                            line, buf = buf.split('\n', 1)
                            self.parse_line(line.strip())
                except Exception:
                    pass
            time.sleep(0.002)

    @staticmethod
    def _parse_shelf(s):
        s = s.strip()
        if len(s) == 4 and all(c in '01' for c in s):
            return [int(c) for c in s]
        return None   # "NAN" hoac sai dinh dang

    def parse_line(self, line):
        if not line.startswith('PKT'):
            return
        p = line.split(',')
        if len(p) < 8:
            return
        try:
            el = int(p[1])
            er = int(p[2])
            em3 = int(p[3])

            # ESP32 gui "NAN" khi BNO055 loi/mat ket noi.
            if p[4].strip().upper() == 'NAN':
                gz = math.nan
            else:
                gz = float(p[4])

            battery_percent = float(p[6])

            # PKT mac dinh:
            #   8 truong: PKT,el,er,em3,gz,dist_cm,batt,timestamp            (firmware cu)
            #   9 truong: PKT,el,er,em3,gz,dist_cm,batt,timestamp,shelf      (firmware co ke)
            # Cac truong khac (tog_yaw/tog_acc/tog_dist) lam lech index -> bo qua dist va ke.
            shelf = None
            if len(p) in (8, 9):
                dist_cm = float(p[5])
                if len(p) == 9:
                    shelf = self._parse_shelf(p[8])
            else:
                dist_cm = -1.0
                if not self._warned_pkt_len:
                    self._warned_pkt_len = True
                    self.get_logger().warn(
                        f'PKT co {len(p)} truong (mong doi 8 hoac 9): bo qua khoang cach '
                        'sieu am va trang thai ke. Kiem tra tog_yaw/tog_acc/tog_dist tren firmware.')

            with self.data_lock:
                self.curr_el = el
                self.curr_er = er
                self.curr_em3 = em3
                self.curr_gz = gz
                self.curr_battery_percent = battery_percent
                self.curr_dist_cm = dist_cm
                self.curr_shelf = shelf
                self.last_pkt_time = time.monotonic()
        except Exception:
            pass

    def _publish_shelf(self, shelf, last_pkt):
        now_m = time.monotonic()
        if shelf is None or (now_m - last_pkt) > self.shelf_timeout:
            state = [-1, -1, -1, -1]
        else:
            state = shelf
        if state != self.last_shelf_sent or (now_m - self.last_shelf_pub) >= self.shelf_heartbeat:
            msg = Int8MultiArray()
            msg.data = state
            self.pub_shelf.publish(msg)
            self.last_shelf_sent = list(state)
            self.last_shelf_pub = now_m

    def publish_loop_100hz(self):
        now_time = self.get_clock().now()
        now = now_time.to_msg()

        dt_sec = (now_time - self.last_time).nanoseconds / 1e9
        if dt_sec <= 0:
            dt_sec = 0.01  # <-- ĐÃ SỬA THÀNH 0.01s (tương ứng 100 Hz)
        self.last_time = now_time

        # Gửi lệnh drive liên tục mỗi 100Hz
        self._send_drive_commands()

        with self.data_lock:
            el = self.curr_el
            er = self.curr_er
            em3 = self.curr_em3
            gz = self.curr_gz
            battery_percent = self.curr_battery_percent
            dist_cm = self.curr_dist_cm
            shelf = self.curr_shelf
            last_pkt = self.last_pkt_time
        self._publish_shelf(shelf, last_pkt)
        if self.first_run:
            self.last_el = el
            self.last_er = er
            self.first_run = False
            return

        # 1. Publish IMU
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'

        if math.isnan(gz):
            # NOTE: gyro_z = NAN từ ESP32 -> không cung cấp angular velocity cho ROS/EKF.
            # Covariance[0] = -1 báo sensor_msgs/Imu rằng angular velocity không khả dụng.
            imu.angular_velocity_covariance[0] = -1.0
        else:
            imu.angular_velocity.z = gz * self.gyro_scale_z
            imu.angular_velocity_covariance = [
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.001
            ]

        self.pub_imu.publish(imu)

        # 2. Publish Joint States
        js = JointState()
        js.header.stamp = now
        js.name = self.joint_names
        js.position = [
            (el / self.cpr) * 2 * math.pi,
            (er / self.cpr) * 2 * math.pi,
            (em3 / self.cpr) * 0.004
        ]
        self.pub_js.publish(js)

        # 3. Publish Odometry Raw
        wheel_circ = 2 * math.pi * self.wheel_radius
        dl = ((el - self.last_el) / self.cpr) * wheel_circ
        dr = ((er - self.last_er) / self.cpr) * wheel_circ
        dt = (dr - dl) / self.wheelbase           # yaw: giu nguyen, khong nhan odom_scale
        dc = ((dl + dr) / 2.0) * self.odom_scale  # tinh tien: da hieu chinh

        self.x += dc * math.cos(self.theta + dt / 2.0)
        self.y += dc * math.sin(self.theta + dt / 2.0)
        self.theta = (self.theta + dt) % (2 * math.pi)
        if self.theta > math.pi:
            self.theta -= 2 * math.pi

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.pose.covariance = [
            0.001, 0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.001, 0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   99.0,  0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   99.0,  0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   99.0,  0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.01
        ]

        odom.twist.twist.linear.x = dc / dt_sec
        odom.twist.twist.angular.z = dt / dt_sec
        odom.twist.covariance = [
            0.001, 0.0,   0.0,   0.0,   0.0,   0.0,
            0.0,   0.001, 0.0,   0.0,   0.0,   0.0,
            0.0,   0.0,   99.0,  0.0,   0.0,   0.0,
            0.0,   0.0,   0.0,   99.0,  0.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   99.0,   0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.005
        ]
        self.pub_odom.publish(odom)

        # 4. Publish Battery State
        battery = BatteryState()
        battery.header.stamp = now
        battery.header.frame_id = 'base_link'
        battery.percentage = battery_percent
        self.pub_battery.publish(battery)

        # 4b. Publish Ultrasonic Range (HC-SR04)
        # REP-117: khong co echo trong tam do -> +inf (GUI hien 'khong co echo')
        rng = Range()
        rng.header.stamp = now
        rng.header.frame_id = self.us_frame
        rng.radiation_type = Range.ULTRASOUND
        rng.field_of_view = self.us_fov
        rng.min_range = self.us_min
        rng.max_range = self.us_max
        if math.isnan(dist_cm) or dist_cm < 0.0:
            rng.range = math.inf
        else:
            rng.range = dist_cm / 100.0   # cm -> m
        self.pub_ultrasonic.publish(rng)

        # 5. TF Broadcaster
        if self.publish_tf and self.tf_broadcaster:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = math.sin(self.theta / 2.0)
            t.transform.rotation.w = math.cos(self.theta / 2.0)
            self.tf_broadcaster.sendTransform(t)

        self.last_el = el
        self.last_er = er

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(b'L0\nR0\nM0\n')
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()