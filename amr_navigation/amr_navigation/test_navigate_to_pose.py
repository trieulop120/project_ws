#!/usr/bin/env python3
"""
Test script to send NavigateToPose goal with custom BT for HOME → P1

Usage:
    ros2 run amr_navigation test_navigate_to_pose
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import math


def create_pose(x, y, yaw=0.0):
    """Create a PoseStamped message"""
    pose = PoseStamped()
    pose.header.stamp.sec = 0
    pose.header.stamp.nanosec = 0
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0

    # Convert yaw to quaternion
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    return pose


class NavigateToPoseTest(Node):
    def __init__(self):
        super().__init__('navigate_to_pose_test')

        # P1 coordinates from route_graph.geojson
        self.p1_x = 1.047
        self.p1_y = 1.56

        # BT XML path
        self.bt_xml_path = '/home/trieu/project_ws/install/amr_navigation/share/amr_navigation/behavior_trees/bt_home_to_p1.xml'

        # Action client
        self.action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.get_logger().info('NavigateToPose Test Node initialized')
        self.get_logger().info(f'Waiting for /navigate_to_pose action server...')

        if not self.action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Timeout waiting for /navigate_to_pose action server')
            return

        self.get_logger().info('/navigate_to_pose action server is ready')

    def send_goal(self):
        """Send NavigateToPose goal"""
        goal = NavigateToPose.Goal()
        goal.pose = create_pose(self.p1_x, self.p1_y, yaw=0.0)
        goal.behavior_tree = self.bt_xml_path

        self.get_logger().info(f'Sending NavigateToPose goal:')
        self.get_logger().info(f'  Pose: x={self.p1_x}, y={self.p1_y}')
        self.get_logger().info(f'  BT: {self.bt_xml_path}')

        self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'[FEEDBACK] Distance remaining: {feedback.distance_remaining:.2f}m'
        )

    def goal_response_callback(self, future):
        """Handle goal response"""
        goal_handle = future.result()

        if goal_handle is None:
            self.get_logger().error('[GOAL] Goal handle is None')
            return

        if not goal_handle.accepted:
            self.get_logger().error('[GOAL] Goal rejected')
            return

        self.get_logger().info('[GOAL] Goal accepted, waiting for result...')

        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        """Handle result"""
        result = future.result()

        self.get_logger().info('========================================')
        self.get_logger().info('[RESULT] Navigation completed')
        self.get_logger().info('========================================')

        # Log result wrapper type and fields for debugging
        self.get_logger().info(f'[DEBUG] result type: {type(result)}')
        result_fields = [attr for attr in dir(result) if not attr.startswith('_')]
        self.get_logger().info(f'[DEBUG] result fields: {result_fields}')

        # Check status - NavigateToPose uses result.status directly
        if hasattr(result, 'status'):
            status = result.status
            self.get_logger().info(f'[STATUS] Goal status: {status}')

            status_names = {
                0: 'UNKNOWN',
                1: 'ACCEPTED',
                2: 'EXECUTING',
                3: 'CANCELING',
                4: 'SUCCEEDED',
                5: 'CANCELED',
                6: 'ABORTED'
            }
            status_name = status_names.get(status, f'UNKNOWN_{status}')
            self.get_logger().info(f'[STATUS] Status name: {status_name}')

            if status == 4:
                self.get_logger().info('[SUCCESS] Navigation SUCCEEDED!')
            elif status == 6:
                self.get_logger().error('[FAILED] Navigation ABORTED')
            elif status == 5:
                self.get_logger().warn('[CANCELED] Navigation was CANCELED')
            else:
                self.get_logger().warn(f'[UNKNOWN] Navigation status: {status_name}')
        else:
            self.get_logger().warn('[RESULT] No status in result')

        # Check for result.result (FollowPath-style result)
        if hasattr(result, 'result'):
            self.get_logger().info(f'[DEBUG] result.result type: {type(result.result)}')


def main():
    rclpy.init()

    test_node = NavigateToPoseTest()

    try:
        test_node.send_goal()

        # Spin until done
        while rclpy.ok():
            rclpy.spin_once(test_node, timeout_sec=0.1)

    except KeyboardInterrupt:
        test_node.get_logger().info('Shutting down...')
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
