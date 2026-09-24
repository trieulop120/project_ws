#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('amr_static_tf_broadcaster')
        self._tf_publisher = StaticTransformBroadcaster(self)
        self.publish_transforms()

    def publish_transforms(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_footprint'
        t.child_frame_id = 'base_link'
        t.transform.translation.z = 0.0925  # Độ cao từ URDF base_footprint_joint
        t.transform.rotation.w = 1.0

        self._tf_publisher.sendTransform([t])
        self.get_logger().info('Đã phát base_footprint -> base_link! Tất cả TF khác do URDF quản lý.')

def main():
    rclpy.init()
    node = StaticTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
