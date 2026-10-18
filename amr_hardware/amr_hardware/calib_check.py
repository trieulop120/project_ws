#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class Calib(Node):
    def __init__(self):
        super().__init__('calib_check')
        self.create_subscription(Odometry, '/odom_raw', self.odom_cb, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 50)
        self.x0 = self.y0 = None
        self.dist = 0.0
        self.yaw_w = 0.0; self.last_yaw = None
        self.yaw_g = 0.0; self.last_t = None
        self.create_timer(1.0, self.report)

    def odom_cb(self, m):
        p = m.pose.pose
        yaw = 2 * math.atan2(p.orientation.z, p.orientation.w)
        if self.x0 is None:
            self.x0, self.y0, self.last_yaw = p.position.x, p.position.y, yaw
        self.dist = math.hypot(p.position.x - self.x0, p.position.y - self.y0)
        d = math.atan2(math.sin(yaw - self.last_yaw), math.cos(yaw - self.last_yaw))
        self.yaw_w += d
        self.last_yaw = yaw

    def imu_cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if self.last_t is not None and m.angular_velocity_covariance[0] >= 0:
            self.yaw_g += m.angular_velocity.z * (t - self.last_t)
        self.last_t = t

    def report(self):
        print(f"dist(odom)={self.dist:.3f} m | yaw bánh={math.degrees(self.yaw_w):8.1f}° | yaw gyro={math.degrees(self.yaw_g):8.1f}°")

rclpy.init(); n = Calib()
try: rclpy.spin(n)
except KeyboardInterrupt: pass