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
import glob
import math
import os
import signal
import shlex
import subprocess
import threading
import time
import uuid
from functools import wraps
from typing import Optional, Dict, Any

from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from lifecycle_msgs.srv import GetState
# History (HISTORY_RESULTS_PLAN.md §6, Phase 3): nav-run lifecycle.
try:
    from amr_web.history import events as _history_events
    from amr_web.history import runs as _history_runs
    _HISTORY_OK = True
except Exception:
    _HISTORY_OK = False
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

# Tự đặt initial pose AMCL theo node HOME của route khi Nav2 sẵn sàng.
# Đặt False để quay về hành vi cũ (initial pose do cấu hình Nav2 hoặc nút Set Pose trên web quyết định).
AUTO_HOME_INITIAL_POSE = True

# Nav2 action message
try:
    from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
except ImportError:
    NavigateToPose = None
    NavigateThroughPoses = None


def _serialize_cancel_and_goal_submission(method):
    """Keep goal submission from racing a cancel state update."""
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        with self._lock:
            generation = self._cancel_generation
            if self._cancel_in_progress:
                return False, 'Cancel is pending; wait for the navigation result'
        with self._cancel_lock:
            if generation != self._cancel_generation:
                return False, 'Command superseded by Cancel'
            return method(self, *args, **kwargs)
    return wrapped


def _serialize_process_management(method):
    @wraps(method)
    def wrapped(self, *args, **kwargs):
        with self._process_lock:
            return method(self, *args, **kwargs)
    return wrapped


class NavBridge:
    """
    Navigation Bridge - Action Client cho Nav2 NavigateToPose.
    """

    # Navigation states
    STATE_IDLE = 'IDLE'
    STATE_STARTING = 'STARTING'
    STATE_SENDING = 'SENDING'
    STATE_ACCEPTED = 'ACCEPTED'
    STATE_CANCELING = 'CANCELING'
    STATE_UNKNOWN = 'UNKNOWN'
    STATE_ACTIVE = 'ACTIVE'
    STATE_SUCCEEDED = 'SUCCEEDED'
    STATE_FAILED = 'FAILED'
    STATE_CANCELED = 'CANCELED'

    def __init__(self, node, ws_broadcast_callback=None, on_session_end=None):
        self._node = node
        self._logger = node.get_logger() if node else None
        self._ws_broadcast = ws_broadcast_callback
        self._on_session_end_callback = on_session_end

        # Nav2 Process
        self._nav_process: Optional[subprocess.Popen] = None
        self._nav_log_handles = {}
        self._nav_active = False

        # Nav2 Action Client
        self._action_client = None
        self._waypoint_action_client = None
        self._goal_handle = None
        self._goal_pending = False
        self._cancel_requested = False
        self._cancel_generation = 0
        self._cancel_in_progress = False
        self._goal_context = None
        self._observed_goals = {}
        self._last_goal_id = None
        self._last_result = None
        self._last_elapsed_s = 0.0
        self._bridge_session_id = uuid.uuid4().hex
        self._state_sequence = 0

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
        self._initial_pose_pending = False

        # P0-1 Fix: Use RLock for deadlock-free locking
        self._lock = threading.RLock()
        # Serialize cancel requests without blocking navigation-state readers.
        self._cancel_lock = threading.Lock()
        self._process_lock = threading.Lock()
        self._goal_changed = threading.Condition(self._lock)

        # Discovery of an action server alone does not establish lifecycle readiness.
        self._lifecycle = {}
        self._lifecycle_pending = {}
        self._lifecycle_requested_at = {}
        self._lifecycle_clients = {
            name: node.create_client(GetState, f'/{name}/get_state',
                                     callback_group=ReentrantCallbackGroup())
            for name in ('bt_navigator', 'controller_server', 'planner_server',
                         'behavior_server', 'velocity_smoother', 'amcl', 'map_server')
        } if node is not None else {}
        self._readiness_timer = (node.create_timer(1.0, self._poll_readiness,
                                callback_group=ReentrantCallbackGroup()) if node else None)

        # Emergency Stop Publisher
        self._cmd_vel_pub = (node.create_publisher(
            Twist, '/cmd_vel_emergency', 10) if node is not None else None)
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
        # Web Cancel must also reach goals sent by mission_manager or LLM,
        # whose goal handles are owned by another ROS node.
        self._cancel_goal_client = (node.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal',
            callback_group=self._cb_group) if node is not None else None)
        self._cancel_waypoint_goal_client = (node.create_client(
            CancelGoal, '/navigate_through_poses/_action/cancel_goal',
            callback_group=self._cb_group) if node is not None else None)

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
            'goal_id': self._last_goal_id,
            'bridge_session_id': self._bridge_session_id,
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

    def _notify_session_end(self) -> None:
        callback = self._on_session_end_callback
        if callback is None:
            return
        try:
            callback()
        except Exception as exc:  # noqa: BLE001
            if self._logger:
                self._logger.warning(f'Navigation session cleanup failed: {exc}')

    def _close_nav_log(self, proc) -> None:
        handle = self._nav_log_handles.pop(proc, None)
        if handle is not None:
            try:
                handle.close()
            except OSError:
                pass

    def _open_nav_log(self):
        """Open a per-run log; logging failures must not prevent Nav2 startup."""
        handle = None
        try:
            log_dir = os.path.expanduser('~/project_ws/logs')
            os.makedirs(log_dir, exist_ok=True)
            stamp = time.strftime('%Y%m%d_%H%M%S') + f'_{time.time_ns() % 1_000_000_000:09d}'
            path = os.path.join(log_dir, f'nav2_{stamp}.log')
            handle = open(path, 'w', encoding='utf-8')
            logs = sorted(glob.glob(os.path.join(log_dir, 'nav2_*.log')),
                          key=os.path.getmtime, reverse=True)
            for old_path in logs[5:]:
                try:
                    os.remove(old_path)
                except OSError:
                    pass
            return handle
        except OSError as exc:
            if handle is not None:
                try:
                    handle.close()
                except OSError:
                    pass
            if self._logger:
                self._logger.warning(f'Could not open Nav2 log; using DEVNULL: {exc}')
            return None

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
            return self._goal_context is not None or bool(self._active_observed_goals())

    @staticmethod
    def validate_pose(pose):
        if not isinstance(pose, dict):
            raise ValueError('Pose must contain finite x, y and yaw numbers')
        for key in ('x', 'y', 'yaw'):
            value = pose.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError('Pose must contain finite x, y and yaw numbers')
        return dict(pose)

    def _poll_readiness(self):
        with self._lock:
            for name, client in self._lifecycle_clients.items():
                pending = self._lifecycle_pending.get(name)
                if pending and not pending.done():
                    if time.monotonic() - self._lifecycle_requested_at.get(name, 0) <= 3.0:
                        continue
                    self._lifecycle_pending.pop(name, None)
                    client.remove_pending_request(pending)
                    pending.cancel()
                if not client.service_is_ready():
                    self._lifecycle.pop(name, None)
                    continue
                try:
                    future = client.call_async(GetState.Request())
                except Exception as exc:
                    self._lifecycle.pop(name, None)
                    if self._logger:
                        self._logger.warning(f'Lifecycle request {name} failed: {exc}')
                    continue
                self._lifecycle_pending[name] = future
                self._lifecycle_requested_at[name] = time.monotonic()
                def receive(done, name=name):
                    with self._lock:
                        if self._lifecycle_pending.get(name) is not done:
                            return
                        try:
                            self._lifecycle[name] = (done.result().current_state.id, time.monotonic())
                        except Exception as exc:
                            self._lifecycle.pop(name, None)
                            if self._logger:
                                self._logger.warning(f'Lifecycle query {name} failed: {exc}')
                future.add_done_callback(receive)

    def _readiness_reason(self):
        if not self._nav_ready():
            return 'Nav2 action server unavailable'
        now = time.monotonic()
        if any(self._lifecycle.get(name, (None, 0))[0] != 3 or
               now - self._lifecycle.get(name, (None, 0))[1] > 3.0
               for name in self._lifecycle_clients):
            return 'Waiting for active Nav2 lifecycle nodes'
        check = getattr(self._node, 'navigation_pose_ready', None)
        if check and not check():
            return 'Waiting for a fresh map transform and odometry'
        if self._initial_pose_pending:
            return 'Initializing localization at HOME'
        return None

    def _active_observed_goals(self):
        # Never release ownership merely because a status stream becomes stale.
        return {key: status for key, status in self._observed_goals.items() if status in (1, 2, 3)}

    def observe_goal_status(self, name, msg):
        with self._goal_changed:
            for item in msg.status_list:
                key = (name, bytes(item.goal_info.goal_id.uuid).hex())
                old = self._observed_goals.get(key)
                # Terminal status is final even if an older callback arrives late.
                if old not in (4, 5, 6):
                    self._observed_goals[key] = item.status
                if old in (1, 2, 3) and item.status in (4, 5, 6) and self._goal_context is None:
                    self._current_state = {4: self.STATE_SUCCEEDED, 5: self.STATE_CANCELED,
                                           6: self.STATE_FAILED}[item.status]
                    self._last_goal_id = key[1]
            # Retain active entries until an explicit terminal status arrives.
            terminal = [k for k, v in self._observed_goals.items() if v in (4, 5, 6)]
            for key in terminal[:-256]:
                self._observed_goals.pop(key, None)
            self._goal_changed.notify_all()

    def _new_goal_context(self):
        ctx = {'id': uuid.uuid4().hex, 'handle': None, 'created': time.monotonic()}
        self._goal_context = ctx
        self._last_goal_id = ctx['id']
        self._last_result = None
        self._goal_pending = True
        self._cancel_requested = False
        self._feedback = None
        self._current_state = self.STATE_SENDING
        return ctx

    def _goal_uncertain(self, ctx, exc):
        with self._lock:
            if self._goal_context is not ctx:
                return
            self._current_state = self.STATE_UNKNOWN
            ctx['message'] = 'Navigation outcome unconfirmed; verify robot state before retrying. Request Cancel or stop the owned Nav2 session.'
            if self._logger:
                self._logger.error(f'Navigation goal={ctx["id"]} unconfirmed: {exc}')
            self._broadcast_state()

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

    @_serialize_process_management
    def start_nav2(self, map_path: str, route_path: Optional[str] = None,
                    map_id: str = None, route_id: str = None) -> tuple:
        """Start Nav2 launch process with monitoring."""
        with self._lock:
            # B21 Fix: Chặn start khi đang dừng
            if self._stopping:
                return False, "Nav2 is stopping, retry in a few seconds"
            if self.is_nav_running():
                return False, "Nav2 process already running"
            if self._nav_ready() or self._waypoint_ready() or self.is_goal_active():
                return False, 'Nav2 already detected outside this bridge session; verify its owner first'

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

        self._notify_session_end()

        ws_setup = os.path.expanduser('~/project_ws/install/setup.bash')
        cmd = f'source {shlex.quote(ws_setup)} && exec ros2 launch amr_navigation nav_web.launch.py {shlex.quote("map:=" + map_abs)}'
        if route_abs:
            cmd += ' ' + shlex.quote('route:=' + route_abs)

        log_handle = self._open_nav_log()
        try:
            proc = subprocess.Popen(
                ['bash', '-c', cmd],
                stdout=log_handle if log_handle else subprocess.DEVNULL,
                stderr=log_handle if log_handle else subprocess.DEVNULL,
                start_new_session=True
            )
        except Exception:
            if log_handle:
                log_handle.close()
            raise
        self._nav_process = proc
        if log_handle:
            self._nav_log_handles[proc] = log_handle

        with self._lock:
            self._nav_active = True
            self._session = {'map_id': map_id, 'route_id': route_id}
            self._current_state = self.STATE_IDLE
            self._lifecycle.clear()
            self._initial_pose_pending = AUTO_HOME_INITIAL_POSE

        # P1-5 Fix: Start monitoring thread instead of just sleep(1.5)
        threading.Thread(target=self._watch_process, args=(self._nav_process,), daemon=True).start()
        if AUTO_HOME_INITIAL_POSE:
            threading.Thread(target=self._apply_home_initial_pose,
                             args=(self._nav_process, map_id, route_id), daemon=True).start()
        self._broadcast_state()
        return True, "Nav2 is starting"

    def _watch_process(self, proc) -> None:
        """Monitor Nav2 process and broadcast when it dies."""
        rc = proc.wait()
        self._close_nav_log(proc)
        with self._lock:
            if self._nav_process is not proc:
                return  # Already stopped by stop_nav2
            self._nav_process = None
            self._nav_active = False
            self._op_mode = 'MANUAL'
            # Process exit is not an action terminal result or proof of standstill.
            self._current_state = self.STATE_UNKNOWN if self.is_goal_active() else self.STATE_FAILED
            self._initial_pose_pending = False

        self._notify_session_end()
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
        if proc.poll() is None:
            raise RuntimeError('Nav2 process termination unconfirmed')

    @_serialize_process_management
    def stop_nav2(self) -> tuple:
        """Stop Nav2 launch process and all child nodes."""
        # B21 Fix: Bật cờ dừng ngay để nav_state báo IDLE trong lúc shutdown
        with self._lock:
            if not self._nav_process:
                return False, 'No Nav2 process owned by this bridge; external processes were not stopped'
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
            try:
                self._terminate_process_group(proc)
            except Exception as exc:
                with self._lock:
                    self._nav_process = proc
                    self._stopping = False
                    self._current_state = self.STATE_UNKNOWN
                self._broadcast_state()
                return False, f'Nav2 shutdown unconfirmed: {exc}'

        with self._lock:
            self._nav_active = False
            self._op_mode = 'MANUAL'
            self._current_state = self.STATE_IDLE
            self._goal_handle = None
            self._goal_pending = False
            self._goal_context = None
            self._observed_goals.clear()
            self._cancel_requested = False
            self._initial_pose_pending = False
            self._goal_changed.notify_all()
            self._feedback = None
            self._current_goal_pose = None
            self._session = {'map_id': None, 'route_id': None}

        # [history] Nav2 was stopped by the user: close a run that is still open (idempotent).
        if _HISTORY_OK:
            try:
                _history_runs.finish_active_run(
                    'INTERRUPTED', 'Nav2 session stopped; action outcome not confirmed', self._capture_current_pose())
            except Exception:
                pass

        # B21 Fix: Tắt cờ dừng sau khi cleanup xong
        with self._lock:
            self._stopping = False

        self._notify_session_end()
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

    @_serialize_cancel_and_goal_submission
    def send_navigation_goal(self, pose, route_id=None):
        try:
            pose = self.validate_pose(pose)
        except ValueError as exc:
            return False, str(exc)
        with self._lock:
            reason = self._submission_block_reason()
            if reason:
                return False, reason
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.pose.position.x = float(pose['x'])
            goal_msg.pose.pose.position.y = float(pose['y'])
            q = self._yaw_to_quaternion(pose['yaw'])
            (goal_msg.pose.pose.orientation.x, goal_msg.pose.pose.orientation.y,
             goal_msg.pose.pose.orientation.z, goal_msg.pose.pose.orientation.w) = q
            self._mission_type = 'single'
            self._initial_distance = None
            self._current_goal_pose = {**pose, 'route_id': route_id}
            ctx = self._new_goal_context()
            if _HISTORY_OK:
                _history_runs.begin_goal_run(
                    goal_pose=pose, start_pose=self._capture_current_pose(),
                    map_id=self._session.get('map_id'), route_id=route_id)
            try:
                future = self._action_client.send_goal_async(goal_msg,
                    feedback_callback=lambda msg: self._on_navigation_feedback(msg, ctx))
                future.add_done_callback(lambda done: self._on_goal_response(done, ctx))
            except Exception as exc:
                self._goal_uncertain(ctx, exc)
                return False, 'Goal submission unconfirmed; verify navigation state before retrying'
            self._broadcast_state()
            return True, 'Goal submitted; awaiting Nav2 acceptance'

    def _submission_block_reason(self):
        if self._stopping:
            return 'Nav2 is stopping'
        if not self.is_nav_running():
            return 'No Nav2 session owned by this bridge'
        if self.is_goal_active():
            return 'A goal is pending, active or unconfirmed; cancel it first'
        return self._readiness_reason()

    def _on_navigation_feedback(self, feedback_msg, ctx) -> None:
        """Handle navigation feedback."""
        try:
            fb = feedback_msg.feedback
            remaining = float(fb.distance_remaining)
            if not math.isfinite(remaining) or remaining < 0:
                raise ValueError('Invalid Nav2 distance feedback')

            with self._lock:
                if self._goal_context is not ctx or self._cancel_requested:
                    return
                ctx['feedback_received'] = True
                self._current_state = self.STATE_ACTIVE
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

    def _on_goal_response(self, future, ctx):
        with self._lock:
            try:
                handle = future.result()
                if self._goal_context is not ctx:
                    if handle.accepted:
                        handle.cancel_goal_async().add_done_callback(self._on_cancel_response)
                    return
                if not handle.accepted:
                    self._finish_goal(GoalStatus.STATUS_ABORTED, 1, 'Nav2 rejected the goal', ctx)
                    return
                ctx['handle'] = self._goal_handle = handle
                ctx['ros_id'] = bytes(handle.goal_id.uuid).hex()
                self._goal_pending = False
                self._current_state = (self.STATE_CANCELING if self._cancel_requested else
                                       self.STATE_ACTIVE if ctx.get('feedback_received') else self.STATE_ACCEPTED)
                # Always attach the result, including when Cancel preceded acceptance.
                handle.get_result_async().add_done_callback(lambda done: self._on_result_received(done, ctx))
                if self._cancel_requested:
                    handle.cancel_goal_async().add_done_callback(self._on_cancel_response)
                self._broadcast_state()
            except Exception as exc:
                self._goal_uncertain(ctx, exc)

    def _finish_goal(self, status: int, error_code: int, error_msg: str, ctx) -> None:
        """P0-3 Fix: Process goal result using status code correctly."""
        with self._goal_changed:
            if self._goal_context is not ctx:
                return
            if status == GoalStatus.STATUS_SUCCEEDED and error_code == 0:
                state, msg = self.STATE_SUCCEEDED, 'Goal reached'
            elif status == GoalStatus.STATUS_CANCELED:
                state, msg = self.STATE_CANCELED, 'Navigation canceled'
            else:
                state, msg = self.STATE_FAILED, f'Navigation failed: {error_msg}'

            self._current_state = state
            self._last_result = {'state': state, 'message': msg, 'goal_id': ctx['id']}
            self._last_elapsed_s = max(0.0, time.monotonic() - ctx['created'])
            ros_id = ctx.get('ros_id')
            if ros_id:
                name = 'navigate_to_pose' if self._mission_type == 'single' else 'navigate_through_poses'
                self._observed_goals[(name, ros_id)] = status
                for key in list(self._observed_goals):
                    if key[1] == ros_id:
                        self._observed_goals[key] = status
            self._broadcast_nav_result(state, msg)
            self._goal_context = None
            self._cancel_requested = False
            self._goal_changed.notify_all()
            self._goal_handle = None
            self._goal_pending = False
            self._feedback = None
            self._current_goal_pose = None
            self._initial_distance = None
            self._waypoint_poses = []
            self._current_waypoint_index = 0
            self._mission_type = 'single'

        self._broadcast_state()

    def _on_result_received(self, future, ctx):
        try:
            wrapped = future.result()
            self._finish_goal(wrapped.status, getattr(wrapped.result, 'error_code', 0),
                getattr(wrapped.result, 'error_msg', '') or f'status={wrapped.status}', ctx)
        except Exception as exc:
            self._goal_uncertain(ctx, exc)

    def cancel_navigation(self, emergency_stop=True, timeout_s=3.0):
        with self._lock:
            if self._cancel_in_progress:
                return False, 'Cancel already pending; await terminal navigation state'
            self._cancel_in_progress = True
            self._cancel_generation += 1
        try:
            return self._cancel_navigation(emergency_stop, timeout_s)
        finally:
            with self._lock:
                self._cancel_in_progress = False

    def _cancel_navigation(self, emergency_stop, timeout_s):
        with self._cancel_lock:
            with self._lock:
                ctx = self._goal_context
                targets = set(self._active_observed_goals())
                self._cancel_requested = ctx is not None
                if ctx or targets:
                    self._current_state = self.STATE_CANCELING
            self._broadcast_state()
            if emergency_stop:
                self._emergency_stop()
            failures = []
            for name, client in (('navigate_to_pose', self._cancel_goal_client),
                                 ('navigate_through_poses', self._cancel_waypoint_goal_client)):
                if client is None or not client.wait_for_service(timeout_sec=0.5):
                    if ctx or any(key[0] == name for key in targets):
                        failures.append(f'{name}: cancel service unavailable')
                    continue
                # Each future owns its storage; a late response must never wake a
                # subsequent service wait or replace that service's response.
                done, reply = threading.Event(), {}
                def receive(future, done=done, reply=reply):
                    try:
                        reply['response'] = future.result()
                    except Exception as exc:
                        reply['error'] = str(exc)
                    finally:
                        done.set()
                try:
                    client.call_async(CancelGoal.Request()).add_done_callback(receive)
                    if not done.wait(timeout=2.0):
                        failures.append(f'{name}: cancel response timeout')
                        continue
                    response = reply.get('response')
                    if response is None or response.return_code != 0:
                        failures.append(f'{name}: cancel rejected or unavailable')
                        continue
                    with self._lock:
                        for info in response.goals_canceling:
                            key = (name, bytes(info.goal_id.uuid).hex())
                            targets.add(key)
                            if self._observed_goals.get(key) not in (4, 5, 6):
                                self._observed_goals[key] = 3
                except Exception as exc:
                    failures.append(f'{name}: {exc}')
            deadline = time.monotonic() + timeout_s
            with self._goal_changed:
                def outstanding():
                    return (ctx is not None and self._goal_context is ctx or
                            any(self._observed_goals.get(key) not in (4, 5, 6) for key in targets))
                while outstanding():
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        self._current_state = self.STATE_UNKNOWN
                        self._broadcast_state()
                        return False, 'Nav2 cancel unconfirmed: awaiting terminal action result; do not retry Start'
                    self._goal_changed.wait(remaining)
                if failures and not (ctx or targets):
                    return False, 'Nav2 cancel unconfirmed: ' + '; '.join(failures)
                # A service error can race a real terminal result. That terminal
                # evidence settles the task, without claiming it was CANCELED.
                self._broadcast_state()
                return True, 'Navigation task ended' if ctx or targets else 'No active navigation goal'

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
        """Synchronously feed zero to the highest-priority mux input."""
        if self._cmd_vel_pub is None:
            raise RuntimeError('Emergency zero-velocity publisher unavailable')
        if self._cmd_vel_pub.get_subscription_count() == 0:
            raise RuntimeError('No subscriber on /cmd_vel_emergency')
        zero = Twist()
        end = time.monotonic() + duration
        while time.monotonic() < end:
            self._cmd_vel_pub.publish(zero)
            time.sleep(1.0 / rate_hz)

    # ============================================
    # NavigateThroughPoses (Waypoints)
    # ============================================

    @_serialize_cancel_and_goal_submission
    def send_waypoint_mission(self, waypoints, mission_type='waypoints'):
        if mission_type not in ('waypoints', 'route'):
            return False, 'Invalid mission type'
        if not isinstance(waypoints, list) or not waypoints:
            return False, 'Provide a non-empty waypoint list'
        try:
            waypoints = [self.validate_pose(wp) for wp in waypoints]
        except ValueError as exc:
            return False, str(exc)
        with self._lock:
            reason = self._submission_block_reason()
            if reason:
                return False, reason
            if not self._waypoint_ready():
                return False, 'NavigateThroughPoses action server unavailable'
            from geometry_msgs.msg import PoseStamped
            poses = []
            for wp in waypoints:
                pose = PoseStamped()
                pose.header.stamp = self._node.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = float(wp['x'])
                pose.pose.position.y = float(wp['y'])
                (pose.pose.orientation.x, pose.pose.orientation.y,
                 pose.pose.orientation.z, pose.pose.orientation.w) = self._yaw_to_quaternion(wp['yaw'])
                poses.append(pose)
            self._waypoint_poses = waypoints
            self._current_waypoint_index = 0
            self._mission_type = mission_type
            self._initial_distance = None
            self._current_goal_pose = {'type': mission_type, 'waypoints': waypoints, 'total': len(waypoints)}
            ctx = self._new_goal_context()
            goal = NavigateThroughPoses.Goal()
            goal.poses = poses
            if _HISTORY_OK:
                _history_runs.begin_waypoint_run(waypoints=waypoints, start_pose=self._capture_current_pose(),
                    map_id=self._session.get('map_id'), route_id=self._session.get('route_id'), mission_type=mission_type)
            try:
                future = self._waypoint_action_client.send_goal_async(goal,
                    feedback_callback=lambda msg: self._on_waypoint_feedback(msg, ctx))
                future.add_done_callback(lambda done: self._on_goal_response(done, ctx))
            except Exception as exc:
                self._goal_uncertain(ctx, exc)
                return False, 'Waypoint submission unconfirmed; verify navigation state before retrying'
            self._broadcast_state()
            return True, 'Waypoint mission submitted; awaiting Nav2 acceptance'

    def _on_waypoint_feedback(self, feedback_msg, ctx) -> None:
        """Handle waypoint navigation feedback."""
        try:
            fb = feedback_msg.feedback
            with self._lock:
                if self._goal_context is not ctx or self._cancel_requested:
                    return
                ctx['feedback_received'] = True
                self._current_state = self.STATE_ACTIVE
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


    # ============================================
    # Initial Pose (AMCL)
    # ============================================

    @_serialize_cancel_and_goal_submission
    def set_initial_pose(self, pose: Dict[str, float], *, initializing=False) -> tuple:
        """Publish a localization request; publication is not AMCL confirmation."""
        try:
            pose = self.validate_pose(pose)
            if not self.is_nav_running():
                return False, 'No Nav2 session owned by this bridge'
            if self._initial_pose_pending and not initializing:
                return False, 'Wait for automatic HOME localization initialization'
            if self._stopping or self.is_goal_active():
                return False, 'Cannot reset localization while navigation is pending, active or unconfirmed'
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

            yaw = pose['yaw']
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
            return True, "Initial pose published; verify localization on the map"

        except Exception as e:
            error_msg = f'Failed to set initial pose: {e}'
            if self._logger:
                self._logger.error(error_msg)
            return False, error_msg

    # ============================================
    # Initial pose tự động theo node HOME của route
    # ============================================

    def _load_home_pose(self, map_id: Optional[str], route_id: Optional[str]) -> Optional[Dict[str, float]]:
        """Đọc node HOME (tên chứa 'HOME') từ ~/project_ws/maps/<map_id>/routes/<route_id>.yaml.
        Trả về {'x','y','yaw'} trong khung map, hoặc None nếu không có."""
        if not map_id or not route_id:
            return None
        try:
            import yaml
            path = os.path.expanduser(f'~/project_ws/maps/{map_id}/routes/{route_id}.yaml')
            with open(path, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            raw = data.get('nodes', {})
            if isinstance(raw, dict):
                items = [(str(k), v) for k, v in raw.items() if isinstance(v, dict)]
            elif isinstance(raw, list):
                items = [(str(n.get('name', '')), n) for n in raw if isinstance(n, dict)]
            else:
                items = []
            homes = [(name, n) for name, n in items
                     if 'HOME' in str(n.get('name', name)).upper()]
            if not homes:
                return None
            # Ưu tiên node tên đúng NODE_HOME, nếu không thì node HOME đầu tiên
            chosen = next((h for h in homes if h[0].upper() == 'NODE_HOME'), homes[0])
            if len(homes) > 1 and self._logger:
                self._logger.warning(f'Route {route_id} có {len(homes)} node HOME, dùng "{chosen[0]}"')
            node = chosen[1]
            pos = node.get('position') or {}
            ori = node.get('orientation') or {}
            return {'x': float(pos.get('x', 0.0)),
                    'y': float(pos.get('y', 0.0)),
                    'yaw': float(ori.get('yaw', 0.0))}
        except Exception as e:
            if self._logger:
                self._logger.warning(f'Không đọc được node HOME ({map_id}/{route_id}): {e}')
            return None

    def _apply_home_initial_pose(self, proc, map_id, route_id, timeout=120.0):
        try:
            self._initialize_home_pose(proc, map_id, route_id, timeout)
        finally:
            with self._lock:
                if self._nav_process is proc:
                    self._initial_pose_pending = False
            self._broadcast_state()

    def _initialize_home_pose(self, proc, map_id, route_id, timeout):
        """Chờ Nav2 sẵn sàng rồi publish /initialpose tại node HOME (chạy trong thread riêng)."""
        home = self._load_home_pose(map_id, route_id)
        if home is None:
            if self._logger:
                self._logger.info('Không có node HOME: giữ initial pose mặc định của Nav2')
            return

        def _still_current() -> bool:
            with self._lock:
                return self._nav_process is proc and not self._stopping

        deadline = time.time() + timeout
        while time.time() < deadline:
            if not _still_current() or proc.poll() is not None:
                return
            if self._nav_ready():
                break
            time.sleep(0.5)
        else:
            if self._logger:
                self._logger.warning('Nav2 chưa sẵn sàng sau %.0fs: bỏ qua initial pose theo HOME' % timeout)
            return

        ok, msg = False, ''
        for _ in range(3):
            if not _still_current():
                return
            ok, msg = self.set_initial_pose(home, initializing=True)
            if ok:
                break
            time.sleep(1.0)
        if self._logger:
            log = self._logger.info if ok else self._logger.warning
            log(f'Initial pose theo HOME x={home["x"]:.3f} y={home["y"]:.3f} '
                f'yaw={home["yaw"]:.3f}: {"OK" if ok else msg}')

    # ============================================
    # State Access
    # ============================================

    def get_navigation_state(self) -> Dict[str, Any]:
        """Get current navigation state."""
        with self._lock:
            stopping = self._stopping
            process = self._nav_process
            current_state = self._current_state
            session = dict(self._session)
            mission_type = self._mission_type
            waypoint_poses = list(self._waypoint_poses)
            waypoint_index = self._current_waypoint_index
            current_goal_pose = self._current_goal_pose
            feedback = self._feedback
            initial_distance = self._initial_distance
            op_mode = self._op_mode
            self._state_sequence += 1
            revision = self._state_sequence
            active = self._active_observed_goals()
            busy = self._goal_context is not None or bool(active)
            owned = bool(process and process.poll() is None)
            detected = self._nav_ready() or self._waypoint_ready()
            reason = self._readiness_reason()
            goal_id = self._last_goal_id
            if self._goal_context is not None:
                # A send response timeout never means the request was rejected.
                if self._goal_pending and time.monotonic() - self._goal_context['created'] > 10.0:
                    self._current_state = current_state = self.STATE_UNKNOWN
            elif active:
                current_state = (self.STATE_UNKNOWN if not detected else
                    self.STATE_CANCELING if 3 in active.values() else
                    self.STATE_ACTIVE if 2 in active.values() else self.STATE_ACCEPTED)
                goal_id = next(iter(active))[1]
            elif detected and not owned:
                current_state = self.STATE_UNKNOWN
            ctx = self._goal_context
            last_result = dict(self._last_result) if self._last_result else None
            task_message = (ctx or {}).get('message') or (
                'Navigation outcome unknown; verify robot state and request cancellation before starting another task.'
                if current_state == self.STATE_UNKNOWN else '')
            elapsed_s = (max(0.0, time.monotonic() - ctx['created']) if ctx else
                         None if active else self._last_elapsed_s)
            state_source = 'web_action' if ctx else 'action_status' if active else 'bridge'
            can_set_pose = (owned and not stopping and not busy and not self._initial_pose_pending
                            and self._init_pose_pub is not None and self._init_pose_pub.get_subscription_count() > 0)

        running = owned or detected
        nav_ready = bool(owned and reason is None and not stopping)
        state = {
            'type': 'nav_state',
            'bridge_session_id': self._bridge_session_id,
            'revision': revision,
            'state': current_state,
            'timestamp': time.time(),
            'nav2_running': running,
            'nav2_ready': nav_ready,
            'process_owned': owned,
            'nav2_detected': detected,
            'stopping': stopping,
            'goal_active': busy,
            'goal_id': goal_id,
            'last_result': last_result,
            'can_send_goal': owned and nav_ready and not busy,
            'can_set_initial_pose': can_set_pose,
            'readiness_reason': reason,
            'state_source': state_source,
            'task_message': task_message,
            'elapsed_s': elapsed_s,
            'map_id': None if stopping else session.get('map_id'),
            'route_id': None if stopping else session.get('route_id'),
            'op_mode': op_mode,
        }

        if not running and busy:
            state['state'] = self.STATE_UNKNOWN

        if mission_type in ('waypoints', 'route') and waypoint_poses:
            state['target_pose'] = current_goal_pose
            total = len(waypoint_poses)
            state['mission_type'] = mission_type
            state['waypoint_index'] = waypoint_index
            state['waypoint_total'] = total
            state['progress'] = max(0.0, min(1.0, waypoint_index / total))

        elif current_goal_pose:
            state['target_pose'] = current_goal_pose
            if feedback:
                rem = feedback.get('distance_remaining', 0.0)
                ini = initial_distance or 0.0
                state['distance_remaining'] = rem
                state['estimated_time_remaining'] = feedback.get('time_remaining', 0)
                state['progress'] = 0.0 if ini <= 0 else max(0.0, min(1.0, 1.0 - rem / ini))
        if state['state'] == self.STATE_SUCCEEDED:
            state['progress'] = 1.0
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
