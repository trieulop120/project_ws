#!/usr/bin/env python3
"""
nav_bridge.py - Navigation Bridge Module
========================================
Kết nối Web Frontend với Nav2 Stack qua Action Client.

Phase 2 Fixes:
- P0-1: RLock thay vì Lock để tránh deadlock
- P0-2: Thread-safe emit qua callback
- P0-3: Goal result check đúng cách (status code)
- P1-1: Blocking calls đã được xử lý ở web_server (to_thread)
- P1-5: Nav2 process monitoring
- P1-6: pkill -9 -f thay bằng exact match
"""

import json
import math
import os
import signal
import subprocess
import threading
import time
from typing import Optional, Dict, Any

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
# History (HISTORY_RESULTS_PLAN.md §6, Phase 3): nav-run lifecycle.
try:
    from amr_web.history import events as _history_events
    from amr_web.history import runs as _history_runs
    _HISTORY_OK = True
except Exception:
    _HISTORY_OK = False
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

# Nav2 action message
try:
    from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
except ImportError:
    NavigateToPose = None
    NavigateThroughPoses = None


class NavBridge:
    """
    Navigation Bridge - Action Client cho Nav2 NavigateToPose.
    """

    # Navigation states
    STATE_IDLE = 'IDLE'
    STATE_STARTING = 'STARTING'
    STATE_ACTIVE = 'ACTIVE'
    STATE_SUCCEEDED = 'SUCCEEDED'
    STATE_FAILED = 'FAILED'
    STATE_CANCELED = 'CANCELED'

    def __init__(self, node, ws_broadcast_callback=None):
        self._node = node
        self._logger = node.get_logger() if node else None
        self._ws_broadcast = ws_broadcast_callback

        # Nav2 Process
        self._nav_process: Optional[subprocess.Popen] = None
        self._nav_active = False

        # Nav2 Action Client
        self._action_client = None
        self._waypoint_action_client = None
        self._goal_handle = None
        self._goal_pending = False
        self._cancel_requested = False

        # Navigation state
        self._current_state = self.STATE_IDLE
        self._current_goal_pose: Optional[Dict] = None
        self._feedback: Optional[Dict] = None
        self._initial_distance: Optional[float] = None

        # Waypoint navigation state
        self._waypoint_poses: list = []
        self._current_waypoint_index: int = 0
        self._mission_type: str = 'single'

        # Session info
        self._session = {'map_id': None, 'route_id': None}

        # Vấn đề 3 Fix: Thêm biến lưu trữ op_mode để đồng bộ giữa các client
        self._op_mode = 'MANUAL'

        # B21 Fix: Cờ báo đang dừng - ngăn frontend nạp lại map trong lúc stop
        self._stopping = False

        # P0-1 Fix: Use RLock for deadlock-free locking
        self._lock = threading.RLock()

        # Emergency Stop Publisher
        self._cmd_vel_pub = None
        self._cmd_vel_nav_pub = None

        # B02 Fix: Tạo ActionClient trong __init__ (không lazy init)
        self._cb_group = ReentrantCallbackGroup()
        if node is not None and NavigateToPose is not None:
            self._action_client = ActionClient(
                node, NavigateToPose, 'navigate_to_pose',
                callback_group=self._cb_group)
        else:
            self._action_client = None
        if node is not None and NavigateThroughPoses is not None:
            self._waypoint_action_client = ActionClient(
                node, NavigateThroughPoses, 'navigate_through_poses',
                callback_group=self._cb_group)
        else:
            self._waypoint_action_client = None

        # Initial Pose Publisher (B03 Fix: tạo trực tiếp, không dùng property)
        if node is not None:
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
            self._init_pose_pub = node.create_publisher(
                PoseWithCovarianceStamped, '/initialpose',
                QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    history=HistoryPolicy.KEEP_LAST, depth=1))
        else:
            self._init_pose_pub = None

        self._logger.info('NavBridge initialized') if self._logger else None

    # ============================================
    # Thread-Safe Emit (P0-2 Fix)
    # ============================================

    def _emit(self, payload: dict) -> None:
        """Thread-safe emit via callback."""
        if not self._ws_broadcast:
            return
        try:
            self._ws_broadcast(json.dumps(payload))
        except Exception as e:
            if self._logger:
                self._logger.warn(f'WS emit error: {e}')

    def _broadcast_state(self) -> None:
        """Broadcast navigation state to WebSocket."""
        self._emit(self.get_navigation_state())

    def set_op_mode(self, mode: str) -> None:
        """Vấn đề 3 Fix: Đặt chế độ vận hành AUTO/MANUAL."""
        with self._lock:
            self._op_mode = mode if mode in ('AUTO', 'MANUAL') else 'MANUAL'
        self._broadcast_state()

    def _broadcast_nav_result(self, state: str, message: str) -> None:
        """Broadcast navigation result to WebSocket."""
        payload = {
            'type': 'nav_result',
            'state': state,
            'message': message,
            'timestamp': time.time(),
        }
        if self._current_goal_pose:
            payload['target'] = self._current_goal_pose.get('name', 'Goal')
        self._emit(payload)
        # [history] Record nav_result event and close the active run. Soft-fail.
        if _HISTORY_OK:
            try:
                sess = _history_runs.get_active()
                _history_events.record_nav_result(
                    state, message, run_id=(sess.run_id if sess is not None else None))
                _history_runs.finish_active_run(
                    state if state in ('SUCCEEDED', 'FAILED', 'CANCELED') else 'FAILED',
                    message, self._capture_current_pose())
            except Exception:
                pass

    # ============================================
    # Process Management - Nav2 Launch
    # ============================================

    def is_nav_running(self) -> bool:
        """Kiểm tra Nav2 process có đang chạy không."""
        if self._nav_process and self._nav_process.poll() is None:
            return True
        return False

    def is_nav_active(self) -> bool:
        """Kiểm tra navigation state."""
        return self._nav_active

    def is_goal_active(self) -> bool:
        """Phase C: True neu hien dang co goal ACTIVE (gui den Nav2 va chua ket thuc).
        Doc duoi lock, KHONG dung de gan state.
        """
        with self._lock:
            return self._current_state == self.STATE_ACTIVE

    def _nav_ready(self) -> bool:
        """Check if navigate_to_pose action server is ready."""
        if not self._action_client:
            return False
        try:
            return self._action_client.server_is_ready()
        except Exception:
            return False

    def _capture_current_pose(self) -> Optional[dict]:
        """Read the latest pose from the bridge's state_bridge (if any)."""
        try:
            node = getattr(self, "_node", None)
            if node is None:
                return None
            state_bridge = getattr(node, "_state_bridge", None)
            if state_bridge is None:
                return None
            pose = state_bridge.get_latest_pose()
            if not pose:
                return None
            pos = pose.get("position") or {}
            ori = pose.get("orientation") or {}
            return {
                "x": float(pos.get("x", 0.0)),
                "y": float(pos.get("y", 0.0)),
                "yaw": float(ori.get("yaw", 0.0)),
            }
        except Exception:
            return None

    def _waypoint_ready(self) -> bool:
        """Check if navigate_through_poses action server is ready."""
        if not self._waypoint_action_client:
            return False
        try:
            return self._waypoint_action_client.server_is_ready()
        except Exception:
            return False

    def start_nav2(self, map_path: str, route_path: Optional[str] = None,
                    map_id: str = None, route_id: str = None) -> tuple:
        """Start Nav2 launch process with monitoring."""
        with self._lock:
            # B21 Fix: Chặn start khi đang dừng
            if self._stopping:
                return False, "Nav2 is stopping, retry in a few seconds"
            if self.is_nav_running():
                return False, "Nav2 process already running"

        map_abs = os.path.abspath(os.path.expanduser(map_path))
        if not os.path.exists(map_abs):
            return False, f"Map file not found: {map_abs}"

        route_abs = ""
        if route_path:
            route_abs = os.path.abspath(os.path.expanduser(route_path))
            if not route_abs.endswith('.geojson'):
                return False, "Route must be .geojson file"
            if not os.path.exists(route_abs):
                return False, f"Route file not found: {route_abs}"

        ws_setup = os.path.expanduser('~/project_ws/install/setup.bash')
        cmd = f'source "{ws_setup}" && exec ros2 launch amr_navigation nav_web.launch.py map:="{map_abs}"'
        if route_abs:
            cmd += f' route:="{route_abs}"'

        # Da xoa log file nav2_launch.log theo yeu cau
        self._nav_process = subprocess.Popen(
            ['bash', '-c', cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid
        )

        with self._lock:
            self._nav_active = True
            self._session = {'map_id': map_id, 'route_id': route_id}
            self._current_state = self.STATE_IDLE

        # P1-5 Fix: Start monitoring thread instead of just sleep(1.5)
        threading.Thread(target=self._watch_process, args=(self._nav_process,), daemon=True).start()
        self._broadcast_state()
        return True, "Nav2 is starting"

    def _watch_process(self, proc) -> None:
        """Monitor Nav2 process and broadcast when it dies."""
        rc = proc.wait()
        with self._lock:
            if self._nav_process is not proc:
                return  # Already stopped by stop_nav2
            self._nav_process = None
            self._nav_active = False
            self._current_state = self.STATE_IDLE
            self._goal_handle = None
            self._goal_pending = False

        self._emit({
            'type': 'nav_result',
            'state': 'FAILED',
            'message': f'Nav2 exited unexpectedly (code {rc})',
            'timestamp': time.time()
        })
        # [history] This path bypasses _broadcast_nav_result: record it and close the run.
        if _HISTORY_OK:
            try:
                _msg = f'Nav2 exited unexpectedly (code {rc})'
                sess = _history_runs.get_active()
                _history_events.record_nav_result(
                    'FAILED', _msg, run_id=(sess.run_id if sess is not None else None))
                _history_runs.finish_active_run('FAILED', _msg, self._capture_current_pose())
            except Exception:
                pass
        self._broadcast_state()

    def _terminate_process_group(self, proc) -> None:
        """Kill process group gracefully."""
        try:
            pgid = os.getpgid(proc.pid)
        except ProcessLookupError:
            return
        for sig, wait_s in ((signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                return
            try:
                proc.wait(timeout=wait_s)
                return
            except subprocess.TimeoutExpired:
                continue

    def stop_nav2(self) -> tuple:
        """Stop Nav2 launch process and all child nodes."""
        # B21 Fix: Bật cờ dừng ngay để nav_state báo IDLE trong lúc shutdown
        with self._lock:
            self._stopping = True

        # Cancel any active goal first
        try:
            self.cancel_navigation(emergency_stop=False)
        except Exception as e:
            if self._logger:
                self._logger.warn(f'Cancel before stop failed: {e}')

        with self._lock:
            proc, self._nav_process = self._nav_process, None  # Signal _watch_process

        if proc:
            self._terminate_process_group(proc)

        # P1-6 Fix: Kill specific nodes with exact match, not -f pattern
        try:
            nodes = [
                'lifecycle_manager_navigation',
                'lifecycle_manager_localization',
                'map_server', 'amcl',
                'controller_server', 'planner_server',
                'bt_navigator', 'waypoint_follower',
                'velocity_smoother', 'behavior_server',
                'route_server', 'route_graph_publisher'
            ]
            for node_name in nodes:
                subprocess.run(['pkill', '-x', node_name],
                             capture_output=True, timeout=1.0)
        except Exception as e:
            if self._logger:
                self._logger.warn(f'Node cleanup warning: {e}')

        with self._lock:
            self._nav_active = False
            self._current_state = self.STATE_IDLE
            self._goal_handle = None
            self._goal_pending = False
            self._feedback = None
            self._current_goal_pose = None
            self._session = {'map_id': None, 'route_id': None}

        # [history] Nav2 was stopped by the user: close a run that is still open (idempotent).
        if _HISTORY_OK:
            try:
                _history_runs.finish_active_run(
                    'CANCELED', 'Navigation stopped', self._capture_current_pose())
            except Exception:
                pass

        # B21 Fix: Tắt cờ dừng sau khi cleanup xong
        with self._lock:
            self._stopping = False

        self._broadcast_state()
        return True, "Nav2 stopped"

    # ============================================
    # Action Client Management
    # ============================================

    def _ensure_action_client(self) -> bool:
        """Ensure navigate_to_pose action client exists and is ready."""
        if NavigateToPose is None:
            return False
        if self._action_client is None:
            self._action_client = self._node.create_client(
                NavigateToPose, 'navigate_to_pose')
        return self._action_client.wait_for_server(timeout_sec=0.5)

    def _ensure_waypoint_client(self) -> bool:
        """Ensure navigate_through_poses action client exists and is ready."""
        if NavigateThroughPoses is None:
            return False
        if self._waypoint_action_client is None:
            self._waypoint_action_client = self._node.create_client(
                NavigateThroughPoses, 'navigate_through_poses')
        return self._waypoint_action_client.wait_for_server(timeout_sec=0.5)

    # ============================================
    # NavigateToPose
    # ============================================

    def send_navigation_goal(self, pose: Dict[str, float],
                            route_id: Optional[str] = None) -> tuple:
        """Send navigation goal to Nav2."""
        with self._lock:
            if not self.is_nav_running():
                return False, "Nav2 is not running"

            if not self._nav_ready():
                return False, "Nav2 is not ready"

            if self._goal_pending or (self._goal_handle and
                    self._goal_handle.status in (GoalStatus.STATUS_ACCEPTED,
                                                 GoalStatus.STATUS_EXECUTING,
                                                 GoalStatus.STATUS_CANCELING)):
                return False, "A goal is running – cancel it first"

            self._initial_distance = None
            self._cancel_requested = False
            self._mission_type = 'single'
            self._goal_pending = True

            # Create goal
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = float(pose.get('x', 0.0))
            goal_msg.pose.pose.position.y = float(pose.get('y', 0.0))
            goal_msg.pose.pose.position.z = 0.0

            yaw = float(pose.get('yaw', 0.0))
            q = self._yaw_to_quaternion(yaw)
            goal_msg.pose.pose.orientation.x = q[0]
            goal_msg.pose.pose.orientation.y = q[1]
            goal_msg.pose.pose.orientation.z = q[2]
            goal_msg.pose.pose.orientation.w = q[3]

            self._current_goal_pose = pose.copy()
            self._current_goal_pose['route_id'] = route_id

            # P0-2 Fix: Use sync send_goal + async callbacks
            send_future = self._action_client.send_goal_async(
                goal_msg, feedback_callback=self._on_navigation_feedback)
            send_future.add_done_callback(self._on_goal_response)

            self._current_state = self.STATE_ACTIVE
            # [history] Open a run for this navigation goal. Use the
            # current pose as the start. Soft-fail; an exception here
            # must not affect robot control.
            if _HISTORY_OK:
                try:
                    _history_runs.begin_goal_run(
                        goal_pose={"x": float(pose.get("x", 0.0)),
                                   "y": float(pose.get("y", 0.0)),
                                   "yaw": float(pose.get("yaw", 0.0))},
                        start_pose=self._capture_current_pose(),
                        map_id=(self._session.get("map_id")
                                if isinstance(self._session, dict) else None),
                        route_id=route_id,
                    )
                except Exception:
                    pass
            self._broadcast_state()
            return True, "Goal accepted"

    def _on_navigation_feedback(self, feedback_msg) -> None:
        """Handle navigation feedback."""
        try:
            fb = feedback_msg.feedback
            remaining = float(fb.distance_remaining)

            with self._lock:
                if self._initial_distance is None or remaining > self._initial_distance:
                    self._initial_distance = remaining
                self._feedback = {
                    'distance_remaining': remaining,
                    'time_remaining': fb.estimated_time_remaining.sec if fb.estimated_time_remaining else 0,
                }

            self._broadcast_state()
        except Exception as e:
            if self._logger:
                self._logger.warn(f'Feedback error: {e}')

    def _on_goal_response(self, future) -> None:
        """Handle goal acceptance/rejection."""
        try:
            self._goal_handle = future.result()

            if not self._goal_handle.accepted:
                with self._lock:
                    self._goal_pending = False
                self._current_state = self.STATE_FAILED
                self._broadcast_nav_result('FAILED', 'Goal rejected')
                return

            # Check if cancel was requested while waiting
            with self._lock:
                if self._cancel_requested:
                    self._goal_handle.cancel_goal_async().add_done_callback(
                        self._on_cancel_response)
                    self._cancel_requested = False
                    return

            # Get result
            result_future = self._goal_handle.get_result_async()
            result_future.add_done_callback(self._on_result_received)

        except Exception as e:
            if self._logger:
                self._logger.error(f'Goal response error: {e}')
            with self._lock:
                self._goal_pending = False
            self._current_state = self.STATE_FAILED
            self._broadcast_nav_result('FAILED', str(e))

    def _finish_goal(self, status: int, error_code: int, error_msg: str) -> None:
        """P0-3 Fix: Process goal result using status code correctly."""
        with self._lock:
            if status == GoalStatus.STATUS_SUCCEEDED and error_code == 0:
                state, msg = self.STATE_SUCCEEDED, 'Goal reached'
            elif status == GoalStatus.STATUS_CANCELED:
                state, msg = self.STATE_CANCELED, 'Navigation canceled'
            else:
                state, msg = self.STATE_FAILED, f'Navigation failed: {error_msg}'

            self._current_state = state
            self._broadcast_nav_result(state, msg)

            self._goal_handle = None
            self._goal_pending = False
            self._feedback = None
            self._current_goal_pose = None
            self._initial_distance = None
            self._waypoint_poses = []
            self._current_waypoint_index = 0
            self._mission_type = 'single'

        self._broadcast_state()

    def _on_result_received(self, future) -> None:
        """Handle navigation result."""
        try:
            wrapped = future.result()
            status = wrapped.status
            error_code = getattr(wrapped.result, 'error_code', 0)
            error_msg = getattr(wrapped.result, 'error_msg', '') or f'status={status}'

            self._finish_goal(status, error_code, error_msg)
        except Exception as e:
            if self._logger:
                self._logger.error(f'Result error: {e}')
            with self._lock:
                self._current_state = self.STATE_FAILED
                self._goal_pending = False
            self._broadcast_nav_result('FAILED', str(e))
            self._broadcast_state()

    def cancel_navigation(self, emergency_stop: bool = True) -> tuple:
        """Cancel active navigation goal."""
        with self._lock:
            if emergency_stop:
                self._emergency_stop()

            handle = self._goal_handle
            if handle and handle.status in (GoalStatus.STATUS_ACCEPTED,
                                            GoalStatus.STATUS_EXECUTING):
                self._cancel_future = handle.cancel_goal_async()
                self._cancel_future.add_done_callback(self._on_cancel_response)
                msg = "Cancel request sent"
            elif self._goal_pending:
                self._cancel_requested = True
                msg = "Will cancel when Nav2 accepts the goal"
            else:
                msg = "No goal is running"

            self._current_state = self.STATE_CANCELED
            self._broadcast_state()
            return True, msg

    def _on_cancel_response(self, future) -> None:
        """Handle cancel response."""
        try:
            result = future.result()
            if result.return_code == 0:
                self._logger.info('Cancel successful') if self._logger else None
            else:
                self._logger.warn(f'Cancel returned: {result.return_code}') if self._logger else None
        except Exception as e:
            self._logger.warn(f'Cancel response error: {e}') if self._logger else None

    def _emergency_stop(self, duration: float = 0.6, rate_hz: float = 20.0) -> None:
        """P0-1/P1-4 Fix: Emergency stop on both cmd_vel topics."""
        def _spam():
            if self._cmd_vel_pub is None:
                self._cmd_vel_pub = self._node.create_publisher(Twist, '/cmd_vel', 10)
            if self._cmd_vel_nav_pub is None:
                self._cmd_vel_nav_pub = self._node.create_publisher(Twist, '/cmd_vel_nav', 10)

            zero = Twist()
            end = time.time() + duration
            while time.time() < end:
                self._cmd_vel_pub.publish(zero)
                self._cmd_vel_nav_pub.publish(zero)
                time.sleep(1.0 / rate_hz)

        threading.Thread(target=_spam, daemon=True).start()

    # ============================================
    # NavigateThroughPoses (Waypoints)
    # ============================================

    def send_waypoint_mission(self, waypoints: list,
                               mission_type: str = 'waypoints') -> tuple:
        """Send waypoint/route mission to Nav2."""
        with self._lock:
            if not waypoints:
                return False, "No waypoints provided"

            if not self.is_nav_running():
                return False, "Nav2 is not running"

            if not self._waypoint_ready():
                return False, "Nav2 is not ready"

            if self._goal_pending or (self._goal_handle and
                    self._goal_handle.status in (GoalStatus.STATUS_ACCEPTED,
                                                 GoalStatus.STATUS_EXECUTING)):
                return False, "A goal is running – cancel it first"

            # Build poses
            from geometry_msgs.msg import PoseStamped
            poses = []
            for wp in waypoints:
                pose = PoseStamped()
                pose.header.stamp = self._node.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(wp.get('x', 0.0))
                pose.pose.position.y = float(wp.get('y', 0.0))
                pose.pose.position.z = 0.0

                yaw = float(wp.get('yaw', 0.0))
                q = self._yaw_to_quaternion(yaw)
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                poses.append(pose)

            self._waypoint_poses = waypoints
            self._current_waypoint_index = 0
            self._mission_type = mission_type
            self._initial_distance = None
            self._cancel_requested = False
            self._goal_pending = True
            self._current_goal_pose = {
                'type': mission_type,
                'waypoints': waypoints,
                'total': len(waypoints)
            }

            # Send goal
            goal_msg = NavigateThroughPoses.Goal()
            goal_msg.poses = poses

            send_future = self._waypoint_action_client.send_goal_async(
                goal_msg, feedback_callback=self._on_waypoint_feedback)
            send_future.add_done_callback(self._on_waypoint_goal_response)

            self._current_state = self.STATE_ACTIVE
            # [history] Open a run for this waypoint/route mission (soft-fail).
            if _HISTORY_OK:
                try:
                    _history_runs.begin_waypoint_run(
                        waypoints=[{"x": float(wp.get("x", 0.0)),
                                    "y": float(wp.get("y", 0.0)),
                                    "yaw": float(wp.get("yaw", 0.0)),
                                    "name": wp.get("name")} for wp in waypoints],
                        start_pose=self._capture_current_pose(),
                        map_id=(self._session.get("map_id")
                                if isinstance(self._session, dict) else None),
                        route_id=(self._session.get("route_id")
                                  if isinstance(self._session, dict) else None),
                        mission_type=mission_type,
                    )
                except Exception:
                    pass
            self._broadcast_state()
            return True, f"Mission started: {len(waypoints)} waypoints"

    def _on_waypoint_feedback(self, feedback_msg) -> None:
        """Handle waypoint navigation feedback."""
        try:
            fb = feedback_msg.feedback
            with self._lock:
                total = len(self._waypoint_poses)
                remaining = int(getattr(fb, 'number_of_poses_remaining', total))
                self._current_waypoint_index = max(0, min(total - 1, total - remaining))
                self._feedback = {
                    'distance_remaining': float(getattr(fb, 'distance_remaining', 0.0)),
                    'time_remaining': getattr(getattr(fb, 'estimated_time_remaining', None), 'sec', 0),
                }
            # [history] waypoint_index increased => the previous waypoint(s) were reached.
            if _HISTORY_OK:
                try:
                    _history_runs.on_waypoint_progress(
                        self._current_waypoint_index, self._capture_current_pose())
                except Exception:
                    pass
            self._broadcast_state()
        except Exception as e:
            if self._logger:
                self._logger.warn(f'Waypoint feedback error: {e}')

    def _on_waypoint_goal_response(self, future) -> None:
        """Handle waypoint goal response."""
        try:
            self._goal_handle = future.result()

            if not self._goal_handle.accepted:
                with self._lock:
                    self._goal_pending = False
                self._current_state = self.STATE_FAILED
                self._broadcast_nav_result('FAILED', 'Waypoint goal rejected')
                return

            with self._lock:
                if self._cancel_requested:
                    self._goal_handle.cancel_goal_async().add_done_callback(
                        self._on_cancel_response)
                    self._cancel_requested = False
                    return

            result_future = self._goal_handle.get_result_async()
            result_future.add_done_callback(self._on_waypoint_result_received)

        except Exception as e:
            if self._logger:
                self._logger.error(f'Waypoint goal response error: {e}')
            with self._lock:
                self._goal_pending = False
            self._current_state = self.STATE_FAILED
            self._broadcast_nav_result('FAILED', str(e))

    def _on_waypoint_result_received(self, future) -> None:
        """Handle waypoint navigation result."""
        try:
            wrapped = future.result()
            status = wrapped.status
            error_code = getattr(wrapped.result, 'error_code', 0)
            error_msg = getattr(wrapped.result, 'error_msg', '') or f'status={status}'

            with self._lock:
                if status == GoalStatus.STATUS_CANCELED:
                    self._current_state = self.STATE_CANCELED
                    self._broadcast_nav_result('CANCELED', 'Waypoint mission canceled')
                elif status == GoalStatus.STATUS_SUCCEEDED and error_code == 0:
                    self._current_state = self.STATE_SUCCEEDED
                    self._logger.info(f'Waypoint mission succeeded') if self._logger else None
                    self._broadcast_nav_result('SUCCEEDED',
                        f"All {len(self._waypoint_poses)} waypoints reached")
                else:
                    self._current_state = self.STATE_FAILED
                    self._logger.warn('Waypoint mission failed') if self._logger else None
                    self._broadcast_nav_result('FAILED', error_msg)

                self._goal_handle = None
                self._goal_pending = False
                self._feedback = None
                self._waypoint_poses = []
                self._current_waypoint_index = 0
                self._mission_type = 'single'

            self._broadcast_state()

        except Exception as e:
            if self._logger:
                self._logger.error(f'Waypoint result error: {e}')
            with self._lock:
                self._current_state = self.STATE_FAILED
                self._goal_pending = False
            self._broadcast_nav_result('FAILED', str(e))
            self._broadcast_state()

    # ============================================
    # Initial Pose (AMCL)
    # ============================================

    def set_initial_pose(self, pose: Dict[str, float]) -> tuple:
        """Set initial pose for AMCL localization."""
        try:
            # B03 Fix: Publisher đã được tạo trong __init__, không cần lazy init
            if self._init_pose_pub is None:
                return False, "Initial pose publisher not available"

            # P1-7 Fix: Check if AMCL is ready - chờ tối đa 1.0s
            max_wait = 1.0
            wait_interval = 0.1
            waited = 0.0
            while self._init_pose_pub.get_subscription_count() == 0 and waited < max_wait:
                time.sleep(wait_interval)
                waited += wait_interval

            if self._init_pose_pub.get_subscription_count() == 0:
                return False, "AMCL is not ready"

            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = float(pose.get('x', 0.0))
            msg.pose.pose.position.y = float(pose.get('y', 0.0))
            msg.pose.pose.position.z = 0.0

            # Ensure yaw is a valid float
            yaw_raw = pose.get('yaw', 0.0)
            try:
                yaw = float(yaw_raw)
                if not (-3.14159 <= yaw <= 3.14159):
                    yaw = 0.0  # Clamp invalid yaw
            except (ValueError, TypeError):
                yaw = 0.0
            q = self._yaw_to_quaternion(yaw)
            msg.pose.pose.orientation.x = q[0]
            msg.pose.pose.orientation.y = q[1]
            msg.pose.pose.orientation.z = q[2]
            msg.pose.pose.orientation.w = q[3]

            # Covariance as tuple for ROS2 compatibility
            msg.pose.covariance = tuple([
                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 0.0, 0.0, 0.130
            ])

            self._init_pose_pub.publish(msg)
            self._logger.info(f'Initial pose set: ({pose.get("x", 0):.2f}, {pose.get("y", 0):.2f})') if self._logger else None
            return True, "Initial pose set"

        except Exception as e:
            error_msg = f'Failed to set initial pose: {e}'
            if self._logger:
                self._logger.error(error_msg)
            return False, error_msg

    # ============================================
    # State Access
    # ============================================

    def get_navigation_state(self) -> Dict[str, Any]:
        """Get current navigation state."""
        with self._lock:
            stopping = self._stopping
            running = self.is_nav_running() and not stopping
            state = {
                'type': 'nav_state',
                'state': self._current_state,
                'timestamp': time.time(),
                'nav2_running': running,
                'nav2_ready': bool(running and self._nav_ready()),
                'map_id': None if stopping else self._session.get('map_id'),
                'route_id': None if stopping else self._session.get('route_id'),
                'op_mode': self._op_mode,
            }

            if stopping or (not running and self._current_state not in (
                    self.STATE_SUCCEEDED, self.STATE_FAILED, self.STATE_CANCELED)):
                state['state'] = self.STATE_IDLE

            if self._mission_type in ('waypoints', 'route') and self._waypoint_poses:
                total = len(self._waypoint_poses)
                state['mission_type'] = self._mission_type
                state['waypoint_index'] = self._current_waypoint_index
                state['waypoint_total'] = total
                state['progress'] = max(0.0, min(1.0, self._current_waypoint_index / total))

            elif self._current_goal_pose:
                state['target_pose'] = self._current_goal_pose
                if self._feedback:
                    rem = self._feedback.get('distance_remaining', 0.0)
                    ini = self._initial_distance or 0.0
                    state['distance_remaining'] = rem
                    state['estimated_time_remaining'] = self._feedback.get('time_remaining', 0)
                    state['progress'] = 0.0 if ini <= 0 else max(0.0, min(1.0, 1.0 - rem / ini))

            return state

    # ============================================
    # Helpers
    # ============================================

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> tuple:
        """Convert yaw (radians) to quaternion (x, y, z, w)."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (0.0, 0.0, sy, cy)