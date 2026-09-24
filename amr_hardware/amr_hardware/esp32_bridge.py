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
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String
import tf2_ros


class ESP32Bridge(Node):
    def __init__(self):
        super().__init__('esp32_bridge')

        # --- PARAMETERS ---
        self.declare_parameter('port', '/dev/esp32')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius', 0.0325)    # Bán kính 32.5mm
        self.declare_parameter('wheelbase', 0.472)         # Khoảng cách 2 bánh 472mm
        self.declare_parameter('counts_per_rev', 3960.0)  # 3960 xung/vòng
        self.declare_parameter('max_rpm', 110.0)           # Tốc độ tối đa 110 RPM
        self.declare_parameter('publish_tf', False)
        self.declare_parameter('joint_names', ['front_encoder_wheel_left_joint', 'front_encoder_wheel_right_joint', 'encoder_lift_joint'])

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.cpr = self.get_parameter('counts_per_rev').value
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.publish_tf = self.get_parameter('publish_tf').value
        self.joint_names = self.get_parameter('joint_names').value

        # Biến lưu trữ dữ liệu phần cứng từ ESP32
        self.curr_el = 0
        self.curr_er = 0
        self.curr_em3 = 0
        self.curr_gz = 0.0
        self.data_lock = threading.Lock()

        self.ser = None
        self.connect_serial()

        # Publishers
        self.pub_imu = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom_raw', 10)

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

        # Luồng đọc Serial riêng biệt
        threading.Thread(target=self.read_loop, daemon=True).start()

        # Timer cố định 30Hz
        self.timer = self.create_timer(1.0 / 30.0, self.publish_loop_30hz)
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
        try:
            if not self.check_symlink_conflict():
                self.get_logger().error('[FATAL] Xung đột Symlink USB!')
                return
            if not os.path.exists(self.port):
                self.get_logger().error(f'[ERROR] Không tìm thấy cổng {self.port}!')
                return
            self.get_logger().info(f'Connecting to serial: {self.port}')
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            time.sleep(1.0)
            self.get_logger().info('Serial connection established successfully.')
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')

    def drive_cb(self, msg: Twist):
        if not self.ser or not self.ser.is_open:
            return
        vx, wz = msg.linear.x, msg.angular.z
        v_left = vx - wz * self.wheelbase / 2.0
        v_right = vx + wz * self.wheelbase / 2.0

        rpm_left = (v_left / (2.0 * math.pi * self.wheel_radius)) * 60.0
        rpm_right = (v_right / (2.0 * math.pi * self.wheel_radius)) * 60.0

        rpm_left = max(min(rpm_left, self.max_rpm), -self.max_rpm)
        rpm_right = max(min(rpm_right, self.max_rpm), -self.max_rpm)

        try:
            self.ser.write(f"L{rpm_left:.2f}\n".encode('utf-8'))
            self.ser.write(f"R{rpm_right:.2f}\n".encode('utf-8'))
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

    def parse_line(self, line):
        if line.startswith('PKT'):
            p = line.split(',')
            if len(p) >= 5:
                try:
                    el = int(p[1])
                    er = int(p[2])
                    em3 = int(p[3])
                    gz = float(p[4])

                    with self.data_lock:
                        self.curr_el = el
                        self.curr_er = er
                        self.curr_em3 = em3
                        self.curr_gz = gz
                except Exception:
                    pass

    def publish_loop_30hz(self):
        now_time = self.get_clock().now()
        now = now_time.to_msg()

        dt_sec = (now_time - self.last_time).nanoseconds / 1e9
        if dt_sec <= 0:
            dt_sec = 0.033
        self.last_time = now_time

        with self.data_lock:
            el = self.curr_el
            er = self.curr_er
            em3 = self.curr_em3
            gz = self.curr_gz

        if self.first_run:
            self.last_el = el
            self.last_er = er
            self.first_run = False
            return

        # 1. Publish IMU
        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'imu_link'
        imu.angular_velocity.z = gz
        imu.angular_velocity_covariance = [
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.02
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
        dc = (dl + dr) / 2.0
        dt = (dr - dl) / self.wheelbase

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
            0.0,   0.0,   0.0,   0.0,   99.0,  0.0,
            0.0,   0.0,   0.0,   0.0,   0.0,   0.01
        ]
        self.pub_odom.publish(odom)

        # 4. TF Broadcaster
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