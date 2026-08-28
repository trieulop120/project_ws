#!/usr/bin/env python3
"""
cmd_vel_splitter.py

Tách /cmd_vel thành 2 topic:
- /diff_drive/cmd_vel: drive (x, y, angular.z) - cho 2 bánh trước
- /lift/cmd_vel: chỉ linear.z (t/b) - cho lift joint

Phím t/b trong teleop_twist_keyboard sẽ chỉ điều khiển lift.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, Float64MultiArray


class CmdVelSplitter(Node):
    def __init__(self):
        super().__init__('cmd_vel_splitter')

        # Parameters
        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.87)
        self.declare_parameter('lift_speed', 0.04)  # m/step

        self.lift_min = self.get_parameter('lift_min').value
        self.lift_max = self.get_parameter('lift_max').value
        self.lift_speed = self.get_parameter('lift_speed').value

        # Current lift position (simulated)
        self.current_lift_pos = 0.0

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publishers
        self.diff_drive_pub = self.create_publisher(
            Twist,
            '/diff_drive/cmd_vel',
            10
        )

        self.lift_pub = self.create_publisher(
            Float64MultiArray,
            '/lift_controller/commands',
            10
        )

        self.get_logger().info('cmd_vel_splitter started')
        self.get_logger().info(f'Lift range: {self.lift_min} to {self.lift_max} m')

    def cmd_vel_callback(self, msg: Twist):
        # === Drive (x, y, angular.z) -> /diff_drive/cmd_vel ===
        # Chỉ lấy x/y linear và angular.z, bỏ qua linear.z
        drive_cmd = Twist()
        drive_cmd.linear.x = msg.linear.x
        drive_cmd.linear.y = msg.linear.y
        drive_cmd.linear.z = 0.0  # Bỏ qua z
        drive_cmd.angular.x = 0.0
        drive_cmd.angular.y = 0.0
        drive_cmd.angular.z = msg.angular.z
        self.diff_drive_pub.publish(drive_cmd)

        # === Lift (linear.z) -> /lift/command ===
        # t (z > 0) -> nâng lên
        # b (z < 0) -> hạ xuống
        if abs(msg.linear.z) > 0.01:  # Có tín hiệu z
            direction = 1.0 if msg.linear.z > 0 else -1.0
            new_pos = self.current_lift_pos + (direction * self.lift_speed)

            # Clamp
            new_pos = max(self.lift_min, min(self.lift_max, new_pos))

            if new_pos != self.current_lift_pos:
                self.current_lift_pos = new_pos
                lift_cmd = Float64MultiArray()
                lift_cmd.data = [self.current_lift_pos]
                self.lift_pub.publish(lift_cmd)
                self.get_logger().debug(f'Lift: {self.current_lift_pos:.3f}m')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSplitter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
