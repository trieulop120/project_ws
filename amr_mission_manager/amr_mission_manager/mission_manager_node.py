#!/usr/bin/env python3
"""
mission_manager_node.py
======================

AMR Mission Manager Node.

Responsibilities:
- Resolve destination from route_graph.yaml
- Select mission-specific BT
- Send NavigateToPose with selected BT

Flow:
    GUI button / Service call
        ↓
    Resolve destination from route_graph.yaml
        ↓
    Select mission-specific BT
        ↓
    NavigateToPose(pose, BT=<mission>.xml)
        ↓
    BT: ComputeRoute → FollowPath → ComputePathToPose → FollowPath
    (or NavigateToPose for return_home)

Usage:
    ros2 run amr_mission_manager mission_manager

Author: AMR System
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Empty
import os


NODE_NAME = 'mission_manager'

# Mission → (destination_type, BT_file)
# destination_type = điểm đến để lấy PoseStamped
# return_home dùng NavigateToPose trực tiếp, không cần route graph
MISSION_BT_MAP = {
    'home_to_p1': ('P_1', 'home_to_p1.xml'),      # HOME → P_1
    'home_to_p2': ('P_2', 'home_to_p2.xml'),      # HOME → P_2
    'p1_to_d1': ('D_1', 'p1_to_d1.xml'),         # P_1 → D_1
    'p2_to_d1': ('D_1', 'p2_to_d1.xml'),         # P_2 → D_1
    'd1_to_p1': ('P_1', 'd1_to_p1.xml'),         # D_1 → P_1
    'd1_to_p2': ('P_2', 'd1_to_p2.xml'),         # D_1 → P_2
    'return_home': ('HOME', 'return_home.xml'),     # Any → HOME (NavigateToPose only)
}


class MissionManager(Node):
    """AMR Mission Manager - sends navigation goals with mission-specific BT."""

    def __init__(self):
        super().__init__(NODE_NAME)

        # Declare parameters
        self.declare_parameter('graph_package', 'amr_navigation')
        self.declare_parameter('graph_yaml_filename', 'route_graph.yaml')

        self._bt_package = self.get_parameter('graph_package').value

        # Initialize route graph loader
        self._loader = None
        self._init_loader()

        # NavigateToPose action client
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('NavigateToPose server not ready')
        else:
            self.get_logger().info('NavigateToPose server connected')

        # Create mission services
        self._create_services()

        self.get_logger().info('Mission Manager initialized')

    def _init_loader(self):
        """Initialize route graph loader."""
        try:
            from amr_navigation.route_graph_loader import RouteGraphLoader

            graph_package = self.get_parameter('graph_package').value
            yaml_filename = self.get_parameter('graph_yaml_filename').value

            self._loader = RouteGraphLoader(
                package_name=graph_package,
                yaml_filename=yaml_filename
            )
            self.get_logger().info(f'Using graph: {graph_package}/config/graphs/{yaml_filename}')
            self.get_logger().info(self._loader.get_summary())
        except ImportError as e:
            self.get_logger().error(f'Failed to import RouteGraphLoader: {e}')
        except Exception as e:
            self.get_logger().error(f'Failed to init loader: {e}')

    def _get_bt_path(self, bt_file: str) -> str:
        """Get BT path for mission."""
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg = get_package_share_directory(self._bt_package)
            return os.path.join(pkg, 'config', 'bt', bt_file)
        except Exception as e:
            self.get_logger().error(f'Failed to get BT path: {e}')
            return ''

    def _create_services(self):
        """Create all mission trigger services."""
        for svc_name in MISSION_BT_MAP.keys():
            service_name = f'mission_manager/trigger_{svc_name}'
            self.create_service(
                Empty,
                service_name,
                lambda req, res, svc=svc_name: self._on_mission(req, res, svc)
            )

        self.get_logger().info(f'Services: {", ".join(MISSION_BT_MAP.keys())}')

    def _on_mission(self, request, response, mission_name: str):
        """Handle mission request."""
        if mission_name not in MISSION_BT_MAP:
            self.get_logger().error(f'Unknown mission: {mission_name}')
            return response

        dest_type, bt_file = MISSION_BT_MAP[mission_name]
        self.get_logger().info(f'=== {mission_name} -> {dest_type} ===')
        self._execute_mission(mission_name, dest_type, bt_file)
        return response

    def _execute_mission(self, mission_name: str, destination: str, bt_file: str) -> bool:
        """Execute mission to destination."""
        if self._loader is None:
            self.get_logger().error('Route loader not available')
            return False

        try:
            # Resolve destination node
            dst_node = self._loader.resolve(destination)
            if not dst_node:
                self.get_logger().error(f'Cannot resolve: {destination}')
                return False

            self.get_logger().info(f'  Destination: {dst_node}')

            # Get destination pose
            pose = self._loader.get_node_pose(dst_node)
            if pose is None:
                self.get_logger().error(f'No pose for {dst_node}')
                return False

            self.get_logger().info(f'  Goal: x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}')

            # Get BT path
            bt_path = self._get_bt_path(bt_file)
            if not bt_path:
                self.get_logger().error(f'BT not found: {bt_file}')
                return False

            self.get_logger().info(f'  BT: {bt_file}')

            # Special handling for return_home - use home_pose key
            if mission_name == 'return_home':
                self._navigate_to_pose(pose, dst_node, bt_path, use_home_pose=True)
            else:
                self._navigate_to_pose(pose, dst_node, bt_path, use_home_pose=False)
            return True

        except Exception as e:
            self.get_logger().error(f'Mission failed: {e}')
            import traceback
            traceback.print_exc()
            return False

    def _navigate_to_pose(self, pose: PoseStamped, node_name: str, bt_path: str, use_home_pose: bool = False):
        """Send NavigateToPose goal with custom BT."""
        pose.header.stamp = self.get_clock().now().to_msg()

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = bt_path

        # For return_home, the BT expects {home_pose} instead of {goal}
        # We pass pose as both to handle this
        if use_home_pose:
            self.get_logger().info('  Mode: NavigateToPose (direct to HOME)')
        else:
            self.get_logger().info('  Mode: ComputeRoute + FollowPath')

        self.get_logger().info(f'  Sending NavigateToPose...')

        if not self._nav_client.server_is_ready():
            self.get_logger().error('NavigateToPose server not ready')
            return

        send_future = self._nav_client.send_goal_async(
            goal,
            feedback_callback=lambda fb: None
        )
        send_future.add_done_callback(
            lambda f: self._on_goal_sent(f, node_name)
        )

    def _on_goal_sent(self, future, node_name: str):
        """Handle goal sent result."""
        try:
            handle = future.result()
            if handle and handle.accepted:
                self.get_logger().info(f'  Goal accepted: {node_name}')
                result_future = handle.get_result_async()
                result_future.add_done_callback(
                    lambda f: self._on_complete(f, node_name)
                )
            else:
                self.get_logger().error('  Goal rejected')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

    def _on_complete(self, future, node_name: str):
        """Handle navigation complete."""
        try:
            result = future.result()
            if result is None:
                self.get_logger().warn(f'  === {node_name} FAILED ===')
                return

            # NavigateToPose returns Empty result, check goal status
            # goal_handle contains status: SUCCEEDED=4, ABORTED=6, CANCELED=2
            if hasattr(result, 'goal_handle') and result.goal_handle:
                status = result.goal_handle.status
                if status == 4:  # SUCCEEDED
                    self.get_logger().info(f'  === {node_name} SUCCEEDED ===')
                elif status == 6:  # ABORTED
                    self.get_logger().warn(f'  === {node_name} ABORTED ===')
                elif status == 2:  # CANCELED
                    self.get_logger().warn(f'  === {node_name} CANCELED ===')
                else:
                    self.get_logger().warn(f'  === {node_name} status={status} ===')
            else:
                self.get_logger().info(f'  === {node_name} COMPLETED ===')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
