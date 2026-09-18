#!/usr/bin/env python3
"""
AMR Mission Manager Node

Handles high-level mission execution for AMR:
- Receives Dropoff/Pickup missions
- Calls Route Server for path planning via async action client
- Calls FollowPath to execute navigation (GOAL 4)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import Empty, Trigger
from nav2_msgs.action import ComputeRoute, FollowPath
import math


# =============================================================================
# SEMANTIC LOCATION MAPPING
# =============================================================================
# Mapping từ semantic location (tên nghiệp vụ) sang graph node ID.
# Chỉ Mission Manager cần biết mapping này.
# J nodes (transit nodes) do Route Server quản lý - không đưa vào đây.
# =============================================================================
SEMANTIC_LOCATIONS = {
    "NODE_HOME": 0,
    "P_1": 1,
    "D_1": 3,
    "SAFE_1": 7,
}

# Reverse mapping để hiển thị node name khi nhận route từ Route Server.
# NOTE: NODE_ID_TO_LOCATION có thể chứa thêm J nodes nếu Route Server
# trả về - nhưng Mission Manager không dùng J nodes trong logic mission.
NODE_ID_TO_LOCATION = {
    0: "NODE_HOME",
    1: "P_1",
    3: "D_1",
    7: "SAFE_1",
    # J nodes được thêm khi Route Server trả về route (xem display_route_result)
}

# Alias để tương thích ngược
LOCATION_TO_NODE_ID = SEMANTIC_LOCATIONS


class MissionManager(Node):
    """
    AMR Mission Manager
    
    Uses async action client pattern:
    trigger_dropoff_callback()
            ↓
    send_goal_async()
            ↓
    goal_response_callback()
            ↓
    get_result_async()
            ↓
    result_callback()
    """
    
    def __init__(self):
        super().__init__('mission_manager')

        # Track mission state
        self.mission_in_progress = False

        # Action client for /compute_route
        self.route_action_client = ActionClient(
            self,
            ComputeRoute,
            '/compute_route'
        )

        # Action client for /follow_path
        self.follow_action_client = ActionClient(
            self,
            FollowPath,
            '/follow_path'
        )

        # Service to trigger dropoff
        self.trigger_service = self.create_service(
            Empty,
            '/mission_manager/trigger_dropoff',
            self.trigger_dropoff_callback
        )

        # Service to trigger pickup
        self.pickup_trigger_service = self.create_service(
            Empty,
            '/mission_manager/trigger_pickup',
            self.trigger_pickup_callback
        )

        # Service to trigger HOME -> P_1 (test only)
        self.home_to_p1_service = self.create_service(
            Trigger,
            '/mission_manager/trigger_home_to_p1',
            self.trigger_home_to_p1_callback
        )

        self.get_logger().info('Mission Manager initialized')
        self.get_logger().info('Waiting for /compute_route action server...')

        if not self.route_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Timeout waiting for /compute_route action server')
            return

        self.get_logger().info('/compute_route action server is ready')
        self.get_logger().info('Waiting for /follow_path action server...')

        if not self.follow_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Timeout waiting for /follow_path action server')
            return

        self.get_logger().info('/follow_path action server is ready')
    
    def trigger_dropoff_callback(self, request, response):
        """
        Service callback - triggers dropoff mission asynchronously.
        Does NOT wait for route result here.
        """
        # Check if mission already in progress
        if self.mission_in_progress:
            self.get_logger().warn('[MISSION_MANAGER] Dropoff mission already in progress')
            return response
        
        self.get_logger().info('========================================')
        self.get_logger().info('[MISSION_MANAGER] Dropoff mission received')
        self.get_logger().info('========================================')
        
        # Dropoff: P_1 -> D_1
        start_id = LOCATION_TO_NODE_ID["P_1"]  # 1
        goal_id = LOCATION_TO_NODE_ID["D_1"]   # 3
        
        self.get_logger().info('')
        self.get_logger().info('[MISSION_MANAGER] Start:')
        self.get_logger().info(f'    P_1 (node_id={start_id})')
        self.get_logger().info('')
        self.get_logger().info('[MISSION_MANAGER] Goal:')
        self.get_logger().info(f'    D_1 (node_id={goal_id})')
        self.get_logger().info('')
        self.get_logger().info('[MISSION_MANAGER] Calling Route Server...')
        
        # Mark mission as in progress
        self.mission_in_progress = True
        
        # Send goal asynchronously
        self.send_route_goal(start_id, goal_id)
        
        # Return immediately - result comes via callback
        return response

    def trigger_pickup_callback(self, request, response):
        """
        Service callback - triggers pickup mission asynchronously.
        Does NOT wait for route result here.
        """
        # Check if mission already in progress
        if self.mission_in_progress:
            self.get_logger().warn('[MISSION_MANAGER] Pickup mission already in progress')
            return response

        self.get_logger().info('========================================')
        self.get_logger().info('[MISSION_MANAGER] Pickup mission received')
        self.get_logger().info('========================================')

        # Pickup: D_1 -> P_1
        start_id = LOCATION_TO_NODE_ID["D_1"]  # 3
        goal_id = LOCATION_TO_NODE_ID["P_1"]   # 1

        self.get_logger().info('')
        self.get_logger().info('[MISSION_MANAGER] Start:')
        self.get_logger().info(f'    D_1 (node_id={start_id})')
        self.get_logger().info('')
        self.get_logger().info('[MISSION_MANAGER] Goal:')
        self.get_logger().info(f'    P_1 (node_id={goal_id})')
        self.get_logger().info('')
        self.get_logger().info('[MISSION_MANAGER] Calling Route Server...')

        # Mark mission as in progress
        self.mission_in_progress = True

        # Send goal asynchronously
        self.send_route_goal(start_id, goal_id)

        # Return immediately - result comes via callback
        return response

    def trigger_home_to_p1_callback(self, request, response):
        """
        Service callback - triggers HOME -> P_1 mission asynchronously.
        For testing FollowPath layer with route: 0 -> 2 -> 1
        """
        # Check if mission already in progress
        if self.mission_in_progress:
            self.get_logger().warn('[HOME_TO_P1] Mission already in progress')
            return response

        self.get_logger().info('========================================')
        self.get_logger().info('[HOME_TO_P1] HOME -> P_1 mission received')
        self.get_logger().info('========================================')

        # HOME -> P_1: start_id=0 (NODE_HOME), goal_id=1 (P_1)
        start_id = LOCATION_TO_NODE_ID["NODE_HOME"]  # 0
        goal_id = LOCATION_TO_NODE_ID["P_1"]         # 1

        self.get_logger().info('')
        self.get_logger().info('[HOME_TO_P1] Start:')
        self.get_logger().info(f'    NODE_HOME (node_id={start_id})')
        self.get_logger().info('')
        self.get_logger().info('[HOME_TO_P1] Goal:')
        self.get_logger().info(f'    P_1 (node_id={goal_id})')
        self.get_logger().info('')
        self.get_logger().info('[HOME_TO_P1] Expected route: 0 -> 2 -> 1')
        self.get_logger().info('[HOME_TO_P1] Calling Route Server...')

        # Mark mission as in progress
        self.mission_in_progress = True

        # Send goal asynchronously - uses same send_route_goal as other missions
        self.send_route_goal(start_id, goal_id)

        # Return immediately - result comes via callback
        return response

    def send_route_goal(self, start_id, goal_id):
        """Send route goal asynchronously"""
        goal_msg = ComputeRoute.Goal()
        goal_msg.start_id = start_id
        goal_msg.goal_id = goal_id
        goal_msg.use_start = False
        goal_msg.use_poses = False
        
        self.get_logger().info(
            f'[MISSION_MANAGER] Sending: start_id={start_id}, goal_id={goal_id}'
        )
        
        # Send goal async - goal_response_callback will be called
        self.route_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.goal_feedback_callback
        ).add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """
        Called when goal is accepted/rejected.
        If accepted, requests result asynchronously.
        """
        try:
            goal_handle = future.result()
            
            if goal_handle is None:
                self.get_logger().error('[MISSION_MANAGER] Goal handle is None')
                self.mission_in_progress = False
                return
            
            if not goal_handle.accepted:
                self.get_logger().error('[MISSION_MANAGER] Goal was rejected by Route Server')
                self.mission_in_progress = False
                return
            
            self.get_logger().info('[MISSION_MANAGER] Route Server accepted goal')
            self.get_logger().info('[MISSION_MANAGER] Waiting for route result...')
            
            # Request result asynchronously - result_callback will be called
            goal_handle.get_result_async().add_done_callback(self.result_callback)
            
        except Exception as e:
            self.get_logger().error(f'[MISSION_MANAGER] Goal response error: {e}')
            self.mission_in_progress = False
    
    def goal_feedback_callback(self, feedback_msg):
        """Handle route planning feedback (optional)"""
        self.get_logger().debug('[FEEDBACK] Planning...')

    def add_orientation_to_path(self, path):
        """
        Add meaningful orientation to path poses based on geometry.
        Uses atan2 to compute yaw from consecutive pose vectors.

        Args:
            path: nav_msgs/Path from Route Server result

        Returns:
            nav_msgs/Path with valid orientations
        """
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped, Quaternion

        if not path.poses or len(path.poses) == 0:
            self.get_logger().warn('[MISSION_MANAGER] Empty path, cannot add orientation')
            return path

        # Create a new path with orientations
        processed_path = Path()
        processed_path.header = path.header

        # Find first valid segment to determine initial orientation
        initial_yaw = None
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i + 1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                initial_yaw = math.atan2(dy, dx)
                break

        if initial_yaw is None:
            # All poses are at the same location, use 0 as default
            self.get_logger().warn('[MISSION_MANAGER] All poses at same location, using default orientation')
            initial_yaw = 0.0

        current_yaw = initial_yaw

        for i, original_pose in enumerate(path.poses):
            # Create new pose with orientation
            new_pose = PoseStamped()
            new_pose.header = original_pose.header
            new_pose.pose.position = original_pose.pose.position

            # Calculate yaw from next segment if available
            if i < len(path.poses) - 1:
                p1 = path.poses[i].pose.position
                p2 = path.poses[i + 1].pose.position
                dx = p2.x - p1.x
                dy = p2.y - p1.y
                if abs(dx) > 1e-6 or abs(dy) > 1e-6:
                    current_yaw = math.atan2(dy, dx)

            # Convert yaw to quaternion (only yaw rotation around Z axis)
            # qx = 0, qy = 0, qz = sin(yaw/2), qw = cos(yaw/2)
            qz = math.sin(current_yaw / 2.0)
            qw = math.cos(current_yaw / 2.0)

            new_pose.pose.orientation.x = 0.0
            new_pose.pose.orientation.y = 0.0
            new_pose.pose.orientation.z = qz
            new_pose.pose.orientation.w = qw

            processed_path.poses.append(new_pose)

        self.get_logger().info(f'[MISSION_MANAGER] Path orientation generated: {len(processed_path.poses)} poses')
        return processed_path

    def send_follow_path_goal(self, path):
        """
        Send FollowPath goal to Controller Server.

        Args:
            path: nav_msgs/Path with valid orientations
        """
        goal_msg = FollowPath.Goal()
        goal_msg.path = path
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'general_goal_checker'

        self.get_logger().info('[MISSION_MANAGER] Sending FollowPath goal...')
        self.get_logger().info(f'    Path: {len(path.poses)} poses')
        self.get_logger().info(f'    Frame: {path.header.frame_id}')
        self.get_logger().info(f'    Controller: FollowPath')

        # Send goal async
        self.follow_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.follow_feedback_callback
        ).add_done_callback(self.follow_goal_response_callback)

    def follow_feedback_callback(self, feedback_msg):
        """Handle FollowPath feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().debug(
            f'[FOLLOW_PATH] Distance to goal: {feedback.distance_to_goal:.2f}m, '
            f'Speed: {feedback.speed:.2f}m/s'
        )

    def follow_goal_response_callback(self, future):
        """
        Called when FollowPath goal is accepted/rejected.
        """
        try:
            goal_handle = future.result()

            if goal_handle is None:
                self.get_logger().error('[MISSION_MANAGER] FollowPath goal handle is None')
                self.mission_in_progress = False
                return

            if not goal_handle.accepted:
                self.get_logger().error('[MISSION_MANAGER] FollowPath goal was rejected')
                self.mission_in_progress = False
                return

            self.get_logger().info('[MISSION_MANAGER] FollowPath goal accepted')
            self.get_logger().info('[MISSION_MANAGER] Waiting for navigation result...')

            # Request result asynchronously
            goal_handle.get_result_async().add_done_callback(self.follow_result_callback)

        except Exception as e:
            self.get_logger().error(f'[MISSION_MANAGER] FollowPath response error: {e}')
            self.mission_in_progress = False

    def follow_result_callback(self, future):
        """
        Handle FollowPath result from Controller Server.
        """
        try:
            result = future.result()

            if result is None:
                self.get_logger().error('[MISSION_MANAGER] FollowPath result is None')
                self.mission_in_progress = False
                return

            # DEBUG: Inspect runtime structure
            self.get_logger().info('')
            self.get_logger().info('========================================')
            self.get_logger().info('[MISSION_MANAGER] Navigation result received')
            self.get_logger().info('========================================')
            self.get_logger().info('')

            # Log result wrapper type
            self.get_logger().info(f'[DEBUG] result type: {type(result)}')

            # Log all fields of result wrapper
            result_fields = [attr for attr in dir(result) if not attr.startswith('_')]
            self.get_logger().info(f'[DEBUG] result fields: {result_fields}')

            # Check for status field
            if hasattr(result, 'status'):
                status = result.status
                self.get_logger().info(f'[DEBUG] result.status = {status}')

                # Map status to name
                status_map = {
                    0: 'UNKNOWN',
                    1: 'ACCEPTED',
                    2: 'EXECUTING',
                    3: 'CANCELING',
                    4: 'SUCCEEDED',
                    5: 'CANCELED',
                    6: 'ABORTED'
                }
                status_name = status_map.get(status, f'UNKNOWN_VALUE_{status}')
                self.get_logger().info(f'[DEBUG] status_name: {status_name}')

                # Log final status
                if status == 4:
                    self.get_logger().info('[MISSION_MANAGER] Navigation completed successfully!')
                elif status == 6:
                    self.get_logger().error('[MISSION_MANAGER] Navigation ABORTED')
                elif status == 5:
                    self.get_logger().warn('[MISSION_MANAGER] Navigation CANCELED')
                else:
                    self.get_logger().warn(f'[MISSION_MANAGER] Navigation status: {status} ({status_name})')
            else:
                self.get_logger().warn('[DEBUG] result has NO status field')

            # Check for result field (FollowPath Result)
            if hasattr(result, 'result'):
                follow_result = result.result
                self.get_logger().info(f'[DEBUG] result.result type: {type(follow_result)}')
                result_fields = [attr for attr in dir(follow_result) if not attr.startswith('_')]
                self.get_logger().info(f'[DEBUG] result.result fields: {result_fields}')

                # Check for error_code and error_msg
                if hasattr(follow_result, 'error_code'):
                    self.get_logger().info(f'[DEBUG] error_code = {follow_result.error_code}')
                else:
                    self.get_logger().info('[DEBUG] NO error_code field')

                if hasattr(follow_result, 'error_msg'):
                    self.get_logger().info(f'[DEBUG] error_msg = {follow_result.error_msg}')
                else:
                    self.get_logger().info('[DEBUG] NO error_msg field')
            else:
                self.get_logger().warn('[DEBUG] result has NO result field')

            self.get_logger().info('')

            self.mission_in_progress = False

        except Exception as e:
            self.get_logger().error(f'[MISSION_MANAGER] FollowPath result error: {e}')
            self.get_logger().info(f'[DEBUG] Exception details: {type(e).__name__}: {e}')
            self.mission_in_progress = False
    
    def result_callback(self, future):
        """
        Handle route result from Route Server.
        After processing, calls FollowPath to execute navigation.
        """
        try:
            result = future.result()

            if result is None:
                self.get_logger().error('[MISSION_MANAGER] Result is None')
                self.mission_in_progress = False
                return

            # Get the actual result from the action result wrapper
            route_result = result.result

            if route_result is None:
                self.get_logger().error('[MISSION_MANAGER] Route result is None')
                self.mission_in_progress = False
                return

            # Display the route
            self.display_route_result(route_result)

            # Get the path from result
            if not hasattr(route_result, 'path') or not route_result.path:
                self.get_logger().error('[MISSION_MANAGER] No path in route result')
                self.mission_in_progress = False
                return

            path = route_result.path
            self.get_logger().info('')
            self.get_logger().info('[MISSION_MANAGER] Route path received:')
            self.get_logger().info(f'    {len(path.poses)} poses')
            self.get_logger().info(f'    Frame: {path.header.frame_id}')

            # Add orientation to path
            processed_path = self.add_orientation_to_path(path)

            # Send to FollowPath
            self.send_follow_path_goal(processed_path)

            # Note: mission_in_progress remains True until FollowPath completes

        except Exception as e:
            self.get_logger().error(f'[MISSION_MANAGER] Result callback error: {e}')
            self.mission_in_progress = False
    
    def display_route_result(self, result):
        """Display route result from Route Server (not hardcoded)"""
        self.get_logger().info('')
        self.get_logger().info('========================================')
        self.get_logger().info('[MISSION_MANAGER] Route planning SUCCEEDED')
        self.get_logger().info('========================================')
        self.get_logger().info('')
        self.get_logger().info('[MISSION_MANAGER] Route received:')
        self.get_logger().info('')
        
        route = result.route
        
        if not route or not route.nodes:
            self.get_logger().warn('[MISSION_MANAGER] Empty route received')
            return
        
        # Get node IDs from Route Server response
        node_ids = [node.nodeid for node in route.nodes]
        
        self.get_logger().info('    ' + '-' * 40)
        
        # Display route with node names
        for i, node in enumerate(route.nodes):
            node_id = node.nodeid
            node_name = NODE_ID_TO_LOCATION.get(node_id, f'Unknown_{node_id}')
            
            if i > 0:
                self.get_logger().info(f'      |')
                self.get_logger().info(f'      v')
            
            self.get_logger().info(f'    {node_name} (id={node_id})')
        
        self.get_logger().info('    ' + '-' * 40)
        
        self.get_logger().info('')
        self.get_logger().info('[MISSION_MANAGER] Node IDs (from Route Server):')
        self.get_logger().info(f'    {" -> ".join(str(n) for n in node_ids)}')
        self.get_logger().info('')
        
        # Route names for display
        route_names = [NODE_ID_TO_LOCATION.get(n.nodeid, f'Unknown_{n.nodeid}') for n in route.nodes]
        self.get_logger().info('[MISSION_MANAGER] Full Route:')
        self.get_logger().info(f'    {" -> ".join(route_names)}')
        self.get_logger().info('')
        
        if route.edges:
            self.get_logger().info(f'[MISSION_MANAGER] Number of edges: {len(route.edges)}')


def main(args=None):
    rclpy.init(args=args)
    
    mission_manager = MissionManager()
    
    try:
        mission_manager.get_logger().info('Mission Manager running. Press Ctrl+C to exit.')
        rclpy.spin(mission_manager)
    except KeyboardInterrupt:
        mission_manager.get_logger().info('Shutting down Mission Manager...')
    finally:
        mission_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
