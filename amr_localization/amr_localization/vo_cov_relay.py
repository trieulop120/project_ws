#!/usr/bin/env python3
"""
Visual Odometry Covariance Relay Node
File location: amr_localization/amr_localization/vo_cov_relay.py
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class VoCovRelayNode(Node):
    def __init__(self):
        super().__init__('vo_cov_relay')

        # Declare parameters
        self.declare_parameter('vx_covariance', 0.015)       # Trọng số vx (mơ hồ hơn Wheel = 0.001)
        self.declare_parameter('vyaw_covariance', 0.05)      # Trọng số vyaw (nếu bật)
        self.declare_parameter('max_valid_covariance', 1.0) # Ngưỡng vứt bỏ frame mất track
        self.declare_parameter('reset_skip_frames', 5)      # Bỏ N frame đầu sau khi track hồi phục

        # Fetch parameters
        self.vx_cov = self.get_parameter('vx_covariance').value
        self.vyaw_cov = self.get_parameter('vyaw_covariance').value
        self.max_valid_cov = self.get_parameter('max_valid_covariance').value
        self.skip_count = self.get_parameter('reset_skip_frames').value

        self.skip_counter = 0

        # ROS 2 Subscriptions & Publishers
        self.sub = self.create_subscription(
            Odometry,
            '/rtabmap/odom',
            self.odom_callback,
            10
        )
        self.pub = self.create_publisher(
            Odometry,
            '/rtabmap/odom_fused',
            10
        )

        self.get_logger().info('VO Covariance Relay Node initialized successfully.')

    def odom_callback(self, msg: Odometry):
        vx = msg.twist.twist.linear.x
        vyaw = msg.twist.twist.angular.z
        raw_cov_vx = msg.twist.covariance[0]

        # 1. Lọc lỗi NaN / Inf / Covariance vọt cao do mất track
        if math.isnan(vx) or math.isnan(vyaw) or math.isinf(vx) or math.isinf(vyaw):
            self.skip_counter = self.skip_count
            return

        if raw_cov_vx >= self.max_valid_cov or raw_cov_vx <= 0.0:
            self.skip_counter = self.skip_count
            return

        # 2. QUY TẮC CHỐNG LỆCH X-Y KHI XOAY:
        # Nếu robot đang quay (|vyaw| > 0.05 rad/s ~ 2.8 deg/s) 
        # HOẶC robot đang đứng yên (|vx| < 0.02 m/s)
        # -> BỎ QUA VO HOÀN TOÀN để không làm lệch X, Y của EKF
        if abs(vyaw) > 0.05 or abs(vx) < 0.02:
            self.skip_counter = self.skip_count
            return

        # 3. Bỏ qua N frame đầu sau khi vừa hết quay/mất track để ổn định
        if self.skip_counter > 0:
            self.skip_counter -= 1
            return

        # 4. Chỉ khi xe ĐI THẲNG ỔN ĐỊNH mới gửi vx cho EKF bù trượt
        fused_msg = Odometry()
        fused_msg.header = msg.header
        fused_msg.child_frame_id = msg.child_frame_id
        fused_msg.pose = msg.pose
        fused_msg.twist = msg.twist

        cov = [1000.0] * 36
        cov[0] = self.vx_cov  # Chỉ truyền vx_covariance (0.015)
        fused_msg.twist.covariance = cov

        self.pub.publish(fused_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VoCovRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()