#!/usr/bin/env python3
"""
Static Transform Publisher for AMR
Publishes all static transforms for rear casters.
Note: Camera optical frames are published automatically by astra_camera driver.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class StaticTransformsPublisher(Node):
    def __init__(self):
        super().__init__('amr_static_transforms')
        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_transforms()

    def _publish_transforms(self):
        transforms = []

        # =====================================================================
        # Rear Caster Left — Steer + Wheel
        # =====================================================================

        # base_link -> rear_caster_steer_left_link
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'rear_caster_steer_left_link'
        t1.transform.translation.x = -0.1332
        t1.transform.translation.y = 0.1931
        t1.transform.translation.z = -0.027503
        t1.transform.rotation.x = 0.0
        t1.transform.rotation.y = 0.0
        t1.transform.rotation.z = 0.0
        t1.transform.rotation.w = 1.0
        transforms.append(t1)

        # rear_caster_steer_left_link -> rear_caster_wheel_left_link
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'rear_caster_steer_left_link'
        t2.child_frame_id = 'rear_caster_wheel_left_link'
        t2.transform.translation.x = 0.017
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = -0.0400
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        transforms.append(t2)

        # =====================================================================
        # Rear Caster Right — Steer + Wheel
        # =====================================================================

        # base_link -> rear_caster_steer_right_link
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'base_link'
        t3.child_frame_id = 'rear_caster_steer_right_link'
        t3.transform.translation.x = -0.1332
        t3.transform.translation.y = -0.1931
        t3.transform.translation.z = -0.027503
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0
        transforms.append(t3)

        # rear_caster_steer_right_link -> rear_caster_wheel_right_link
        t4 = TransformStamped()
        t4.header.stamp = self.get_clock().now().to_msg()
        t4.header.frame_id = 'rear_caster_steer_right_link'
        t4.child_frame_id = 'rear_caster_wheel_right_link'
        t4.transform.translation.x = 0.017
        t4.transform.translation.y = 0.0
        t4.transform.translation.z = -0.0400
        t4.transform.rotation.x = 0.0
        t4.transform.rotation.y = 0.0
        t4.transform.rotation.z = 0.0
        t4.transform.rotation.w = 1.0
        transforms.append(t4)

        # Publish all at once
        self._tf_broadcaster.sendTransform(transforms)
        self.get_logger().info(
            f'Published {len(transforms)} static transforms: 4 caster TFs'
        )


def main(args=None):
    rclpy.init(args=args)
    node = StaticTransformsPublisher()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
