#!/usr/bin/env python3
"""
amr_web/bridge_node.py
======================
ROS 2 Entry Point Node - Optimized for Overview Dashboard
- Heartbeat monitoring cho 4 thiết bị phần cứng
- MultiThreadedExecutor cho ROS callbacks
- Demand-driven Camera Streaming (RGB & DEPTH) qua WebSocket
- SLAM web control: start/stop SLAM, save map, teleop, lift
"""

import asyncio
import copy
from collections import deque
from datetime import datetime
import json
import math
import os
import signal
import subprocess
import threading
import time
from typing import Optional, Tuple
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from sensor_msgs.msg import BatteryState, Image, LaserScan, JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from std_srvs.srv import Trigger
import uvicorn

# Phase B05: tf2_ros for pose re-frame (odom -> map)
from tf2_ros import Buffer, TransformListener

from amr_web.websocket_manager import WebSocketManager
from amr_web.state_bridge import StateBridge
from amr_web.converters import (
    odom_to_json, map_to_json, laser_to_json, image_to_json,
    quaternion_to_yaw,
    costmap_to_json, plan_to_json, compose_planar,
)
from amr_web.web_server import create_app
from amr_web.nav_bridge import NavBridge
# History (HISTORY_RESULTS_PLAN.md §6): edge-triggered system events.
# Import is guarded so a broken history package cannot kill the bridge.
try:
    from amr_web.history import db as _history_db
    from amr_web.history import system_hooks as _history_system_hooks
    _HISTORY_OK = True
except Exception:
    _HISTORY_OK = False


def sensor_qos():
    """QoS Profile cho sensor data: Best Effort, depth=1, không giữ history."""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE
    )


def plan_qos():
    """QoS Profile cho /plan: RELIABLE + VOLATILE, KEEP_LAST depth=10.
    Planner_server va route_server deu publish /plan, can RELIABLE.
    Khong dung TRANSIENT_LOCAL vi plan chi ton tai khi co goal.
    """
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        durability=DurabilityPolicy.VOLATILE
    )


class WebBridge(Node):
    """
    ROS 2 Web Bridge Node - Optimized for Overview Dashboard.
    Giám sát heartbeat 4 thiết bị: ESP32, LiDAR, Camera, Lift.
    """

    _CYCLE_RUNNING = {
        'GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING',
        'WAITING_SLOT', 'RETURN_HOME',
    }
    _CYCLE_BLOCKED_COMMANDS = _CYCLE_RUNNING | {'RECOVERY'}

    def __init__(self):
        super().__init__('web_bridge')

        # Parameters
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('pose_topic', '/odometry/filtered')
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('camera_topic', '/color/image_raw')
        self.declare_parameter('depth_camera_topic', '/depth/image_raw')
        self.declare_parameter('lidar_topic', '/scan_filtered')
        self.declare_parameter('joint_topic', '/joint_states')
        self.declare_parameter('pose_rate', 5.0)
        # Phase B05: frames for tf lookup (map -> base)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('map_frame', 'map')
        # Phase C: costmap + plan topics + broadcast intervals
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('local_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('plan_topic', '/plan')
        self.declare_parameter('costmap_global_interval', 1.0)
        self.declare_parameter('costmap_local_interval', 0.33)
        self.declare_parameter('plan_interval', 0.25)
        self.declare_parameter('battery_rate', 1.0)
        self.declare_parameter('system_status_rate', 1.0)
        self.declare_parameter('map_on_change', True)
        self.declare_parameter('verbose', True)

        self._host = self.get_parameter('host').value
        self._port = self.get_parameter('port').value
        self._pose_topic = self.get_parameter('pose_topic').value
        self._battery_topic = self.get_parameter('battery_topic').value
        self._map_topic = self.get_parameter('map_topic').value
        self._camera_topic = self.get_parameter('camera_topic').value
        self._depth_camera_topic = self.get_parameter('depth_camera_topic').value
        # Pallet detection overlay stream (drawn by pallet_detector_node):
        # its /pallet/detections + target OBB + box/hole companion overlays.
        self.declare_parameter('debug_camera_topic', '/perception/pallet_debug_image')
        self._debug_camera_topic = self.get_parameter('debug_camera_topic').value
        # pallet_mission status (JSON string) -> web cargo card.
        self.declare_parameter('mission_status_topic', '/pallet_mission/status')
        self._mission_status_topic = self.get_parameter('mission_status_topic').value
        self.declare_parameter('supervisor_status_topic', '/mission_supervisor/status')
        self._supervisor_status_topic = self.get_parameter('supervisor_status_topic').value
        self._lidar_topic = self.get_parameter('lidar_topic').value
        self._joint_topic = self.get_parameter('joint_topic').value
        self._pose_rate = self.get_parameter('pose_rate').value
        self._base_frame = self.get_parameter('base_frame').value
        self._map_frame = self.get_parameter('map_frame').value
        # Phase C: costmap + plan
        self._global_costmap_topic = self.get_parameter('global_costmap_topic').value
        self._local_costmap_topic = self.get_parameter('local_costmap_topic').value
        self._plan_topic = self.get_parameter('plan_topic').value
        self._costmap_global_interval = self.get_parameter('costmap_global_interval').value
        self._costmap_local_interval = self.get_parameter('costmap_local_interval').value
        self._plan_interval = self.get_parameter('plan_interval').value
        self._battery_rate = self.get_parameter('battery_rate').value
        self._system_status_rate = self.get_parameter('system_status_rate').value
        self._map_on_change = self.get_parameter('map_on_change').value
        self._verbose = self.get_parameter('verbose').value

        # Camera Streaming Control Flag
        self._camera_streaming = False
        self._camera_mode = 'off'  # 'rgb', 'depth', 'detect', 'off'

        # Pallet mission status (JSON from /pallet_mission/status). None until
        # the mission node publishes; exposed to web as mission{state,cargo,
        # us_cm,lift_cm,ts} piggybacked on the 1Hz system_status broadcast.
        self._mission_status = None
        self._mission_supervisor_status = None
        self._recent_supervisor_missions = deque(maxlen=50)
        self._recent_mission_baseline_pending = False
        self._nav2_goal_active = {
            'navigate_to_pose': False,
            'navigate_through_poses': False,
        }
        self._plan_received_t = 0.0

        # State Managers
        self._state_bridge = StateBridge()
        self._ws_manager = WebSocketManager()
        self._last_map_hash = None

        # Lift state tracking
        self._lift_height_mm: int = 0
        self._lift_initialized: bool = False

        # SLAM Process Management
        self._slam_process: Optional[subprocess.Popen] = None
        self._slam_active: bool = False

        # Navigation Bridge (Nav2 Action Client)
        self._nav_bridge: Optional[NavBridge] = None

        # Callback groups tach rieng: mac dinh moi callback cua node nam chung 1 nhom
        # MutuallyExclusive -> chi chay tung cai mot, map/camera nang se lam doi
        # pose/laser/heartbeat/timer. Tach nhom de 4 thread cua executor chay song song.
        self._cb_sensor = MutuallyExclusiveCallbackGroup()
        self._cb_map = MutuallyExclusiveCallbackGroup()
        self._cb_cam = MutuallyExclusiveCallbackGroup()

        # Initialize laser callback state before starting the TF listener thread
        # or creating subscriptions. The executor may invoke callbacks as soon
        # as subscriptions exist, while __init__ is still running.
        self._laser_interval = 0.1  # 10 Hz for laser scan
        self._last_laser_convert = 0.0

        # Phase B05: tf2_ros buffer + listener for pose re-frame
        # spin_thread=True: /tf (tan so cao khi Nav2 chay) duoc xu ly o thread rieng,
        # khong tranh voi callback cua node nay.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)
        self._last_map_pose_t = 0.0   # timestamp of last successful map-frame pose
        self._map_pose_logged = False  # log once when first map pose acquired
        # Bản pose odom THÔ (chưa đổi khung). Chỉ _on_pose được ghi vào đây,
        # tuyệt đối không đọc lại từ state_bridge (vì state_bridge có thể đang chứa map pose).
        self._last_odom_pose: Optional[dict] = None
        # Timer cung nhip pose_rate (5 Hz), goi _tf_lookup_timer de lay TF map <- base
        self._tf_lookup_timer_obj = self.create_timer(
            1.0 / max(self._pose_rate, 0.1), self._tf_lookup_timer)

        # [history] Pose sampler at 5 Hz. Pushes (t, x, y, yaw) into the
        # active RunSession, if any. Non-blocking; never touches the
        # network or DB directly.
        if _HISTORY_OK:
            self._history_pose_timer = self.create_timer(
                0.2, self._history_sample_pose)

        # Publishers cho teleop và lift
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)
        self._esp32_cmd_pub = self.create_publisher(String, '/esp32_cmd', 10)

        # Laser scan subscription
        self._laser_sub = self.create_subscription(
            LaserScan, self._lidar_topic, self._on_laser, qos_profile=sensor_qos(),
            callback_group=self._cb_sensor)

        # FastAPI Server Setup
        self._app = create_app(
            state_bridge=self._state_bridge,
            ws_manager=self._ws_manager,
            node_logger=self.get_logger() if self._verbose else None,
            bridge_node=self
        )

        # ROS Subscriptions
        self._pose_sub = self.create_subscription(
            Odometry, self._pose_topic, self._on_pose, qos_profile=sensor_qos(),
            callback_group=self._cb_sensor)
        self._battery_sub = self.create_subscription(
            BatteryState, self._battery_topic, self._on_battery, qos_profile=sensor_qos(),
            callback_group=self._cb_sensor)
        # P0-5 Fix: Map QoS - TRANSIENT_LOCAL for reliable map delivery
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self._map_sub = self.create_subscription(
            OccupancyGrid, self._map_topic, self._on_map, qos_profile=map_qos,
            callback_group=self._cb_map)
        # Phase C: 2 costmap (global/local) + plan. Dung _cb_map da co (B05).
        # Local costmap phai quy doi odom -> map bang _lookup_map_tf.
        self._global_costmap_sub = self.create_subscription(
            OccupancyGrid, self._global_costmap_topic, self._on_global_costmap,
            qos_profile=map_qos, callback_group=self._cb_map)
        self._local_costmap_sub = self.create_subscription(
            OccupancyGrid, self._local_costmap_topic, self._on_local_costmap,
            qos_profile=map_qos, callback_group=self._cb_map)
        self._plan_sub = self.create_subscription(
            Path, self._plan_topic, self._on_plan,
            qos_profile=plan_qos(), callback_group=self._cb_map)
        for action_name in self._nav2_goal_active:
            self.create_subscription(
                GoalStatusArray,
                f'/{action_name}/_action/status',
                lambda msg, name=action_name: self._on_nav2_goal_status(name, msg),
                10,
                callback_group=self._cb_sensor)
        self._camera_sub = self.create_subscription(
            Image, self._camera_topic, self._on_camera_rgb, qos_profile=sensor_qos(),
            callback_group=self._cb_cam)
        self._depth_camera_sub = self.create_subscription(
            Image, self._depth_camera_topic, self._on_camera_depth, qos_profile=sensor_qos(),
            callback_group=self._cb_cam)
        self._debug_camera_sub = self.create_subscription(
            Image, self._debug_camera_topic, self._on_camera_debug, qos_profile=sensor_qos(),
            callback_group=self._cb_cam)
        # Mission status: String JSON, ~2Hz from pallet_mission. The publisher
        # is plain reliable (depth 10), so match it with the default profile.
        self._mission_status_sub = self.create_subscription(
            String, self._mission_status_topic, self._on_mission_status, 10)
        self._mission_supervisor_status_sub = self.create_subscription(
            String, self._supervisor_status_topic, self._on_mission_supervisor_status, 10)
        self._mission_start_client = self.create_client(Trigger, '/mission/start')
        self._mission_stop_client = self.create_client(Trigger, '/mission/stop')
        self._mission_reset_client = self.create_client(Trigger, '/mission/reset')
        self._mission_resume_client = self.create_client(Trigger, '/mission/resume')
        self._mission_params_client = self.create_client(
            SetParameters, '/mission_supervisor/set_parameters')
        self._pallet_stop_client = self.create_client(Trigger, '/pallet_mission/stop')
        self._lidar_sub = self.create_subscription(
            LaserScan, self._lidar_topic, self._on_lidar, qos_profile=sensor_qos(),
            callback_group=self._cb_sensor)
        self._joint_sub = self.create_subscription(
            JointState, self._joint_topic, self._on_joint, qos_profile=10,
            callback_group=self._cb_sensor)

        # Threading
        self._running = True
        self._ws_thread: Optional[threading.Thread] = None
        self._broadcast_thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._broadcast_task: Optional[asyncio.Task] = None

        # Rate control
        self._pose_interval = 1.0 / self._pose_rate
        self._battery_interval = 1.0 / self._battery_rate
        self._system_status_interval = 1.0 / self._system_status_rate
        self._last_pose_send = 0.0
        self._last_battery_send = 0.0
        self._last_system_status_send = 0.0
        self._last_laser_send = 0.0
        self._last_map_send = 0.0
        self._map_interval = 0.5  # 2 Hz for map broadcast
        self._map_pending = False

        # Phase C: costmap + plan broadcast state
        self._costmap_global_pending = False
        self._costmap_local_pending = False
        self._plan_pending = False
        self._last_costmap_global_send = 0.0
        self._last_costmap_local_send = 0.0
        self._last_plan_send = 0.0
        # Dedupe signatures: tranh emit lien tuc neu du lieu khong doi that su
        self._costmap_global_sig = None
        self._costmap_local_sig = None

        if self._verbose:
            self.get_logger().info(f'Web Bridge Running on {self._host}:{self._port}')
            self.get_logger().info(f'Lift tracking: joint={self._joint_topic} (position in meters → mm)')

        # Initialize Navigation Bridge
        self._init_nav_bridge()

        # Initialize History (HISTORY_RESULTS_PLAN.md §6). Soft-fail.
        self._init_history()

        # LLM voice/command bridge
        self._init_llm()

    def set_camera_stream(self, enable: bool, mode: str = 'off') -> None:
        """Bật/tắt camera stream dynamic theo yêu cầu từ Web SCADA."""
        self._camera_streaming = enable
        self._camera_mode = mode if enable else 'off'
        if self._verbose:
            self.get_logger().info(f"Camera stream set: enable={enable}, mode={self._camera_mode}")

    def _init_nav_bridge(self) -> None:
        """Initialize Navigation Bridge."""
        try:
            self._nav_bridge = NavBridge(
                node=self,
                ws_broadcast_callback=self.broadcast_threadsafe,
                on_session_end=self._on_nav_session_end,
            )
            if self._verbose:
                self.get_logger().info('NavBridge initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to init NavBridge: {e}')
            self._nav_bridge = None

    def _on_nav_session_end(self) -> None:
        """Clear per-session dashboard caches without touching persistent history."""
        self._mission_status = None
        llm = getattr(self, 'llm', None)
        if llm is not None:
            try:
                llm.reset()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f'LLM session reset failed: {exc}')

    def _init_llm(self) -> None:
        """LLM bridge (lệnh tiếng Việt/giọng nói). Soft-fail: dashboard vẫn chạy nếu lỗi."""
        self.llm = None
        try:
            from amr_llm.llm_bridge import LLMBridge
            self.llm = LLMBridge(
                node=self,
                broadcast=lambda d: self.broadcast_threadsafe(json.dumps(d)),
                precheck=self._llm_precheck)
            if self._verbose:
                self.get_logger().info('LLMBridge initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to init LLMBridge: {e}')
            self.llm = None

    def _llm_precheck(self):
        nav = self._nav_bridge.get_navigation_state() if self._nav_bridge else {}
        if nav.get('nav2_ready') is not True:
            return 'Nav2 chưa sẵn sàng; hãy khởi động navigation trước.'
        status = self.get_mission_supervisor_status()
        if status is None or status.get('stale'):
            return 'Mission supervisor chưa có status mới.'
        graph = status.get('route_graph') or {}
        if graph.get('loaded') is not True:
            return f"Route graph chưa sẵn sàng: {graph.get('error') or 'chưa được nạp'}"
        return None

    def _init_history(self) -> None:
        """Initialise the operation history subsystem (additive, soft-fail)."""
        if not _HISTORY_OK:
            return
        try:
            ok = _history_db.init()
            if ok and self._verbose:
                self.get_logger().info(
                    f"History DB enabled at {_history_db.db_path()}")
            elif not ok and self._verbose:
                self.get_logger().warn(
                    "History DB disabled (could not open DB). "
                    "Robot operation is unaffected.")
            # Mark any runs left open by a previous backend instance.
            try:
                _history_db.mark_open_runs_interrupted()
            except Exception:
                pass
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f"History init failed (ignored): {e}")

    # ============================================
    # Event Loop Attachment (Phase 1 Fix)
    # ============================================

    def attach_server_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Attach to uvicorn's event loop for thread-safe broadcast."""
        self._loop = loop
        self._running = True
        self._broadcast_task = loop.create_task(self._broadcast_forever())
        if self._verbose:
            self.get_logger().info('Attached to server event loop')

    def detach_server_loop(self) -> None:
        """Detach from event loop on shutdown."""
        self._running = False
        if self._broadcast_task:
            self._broadcast_task.cancel()
        self._loop = None
        if self._verbose:
            self.get_logger().info('Detached from server event loop')

    def broadcast_threadsafe(self, message: str) -> None:
        """Thread-safe broadcast via server loop. Called from any thread."""
        loop = self._loop
        if loop is None or loop.is_closed():
            return
        asyncio.run_coroutine_threadsafe(
            self._ws_manager.broadcast(message),
            loop
        )

    async def _broadcast_forever(self) -> None:
        """Broadcast loop running in uvicorn's event loop."""
        last_nav = 0.0
        while self._running:
            try:
                now = time.time()

                if now - self._last_pose_send >= self._pose_interval:
                    pose = self._state_bridge.get_latest_pose()
                    if pose:
                        await self._ws_manager.broadcast(json.dumps(pose))
                        self._last_pose_send = now

                if now - self._last_battery_send >= self._battery_interval:
                    battery = self._state_bridge.get_latest_battery()
                    if battery:
                        await self._ws_manager.broadcast(json.dumps(battery))
                        self._last_battery_send = now

                if now - self._last_system_status_send >= self._system_status_interval:
                    system_status = self._state_bridge.get_system_overview_state()
                    # Piggyback the pallet-mission cargo card data (1Hz is
                    # plenty; the mission publishes at ~2Hz, we just forward
                    # the freshest parse). `stale` lets the UI dim when the
                    # mission node has gone quiet (stopped / not running).
                    m = self._mission_status
                    if m is not None:
                        m = dict(m)
                        m['stale'] = (now - m.get('ts', 0.0)) > 3.0
                    system_status['mission'] = m
                    supervisor = self.get_mission_supervisor_status()
                    system_status['mission_supervisor'] = supervisor
                    await self._ws_manager.broadcast(json.dumps(system_status))
                    # [history] Feed the edge-trigger monitor (plan §6).
                    if _HISTORY_OK:
                        try:
                            _history_system_hooks.tick(
                                system_status=system_status,
                                nav_state={"running": bool(self.is_navigation_running())},
                                slam_active=self.get_slam_state(),
                                battery=self._state_bridge.get_latest_battery(),
                            )
                        except Exception:
                            pass
                    self._last_system_status_send = now

                if now - self._last_laser_send >= self._laser_interval:
                    laser = self._state_bridge.get_latest_laser()
                    if laser:
                        await self._ws_manager.broadcast(json.dumps(laser))
                        self._last_laser_send = now

                if self._map_pending and (now - self._last_map_send) >= self._map_interval:
                    map_data = self._state_bridge.get_latest_map()
                    if map_data:
                        await self._ws_manager.broadcast(json.dumps(map_data))
                        self._last_map_send = now
                        self._map_pending = False

                # Phase C: costmap + plan broadcast (gate theo nav running)
                nav_running = bool(self._nav_bridge and self._nav_bridge.is_nav_running())
                goal_active = self._is_navigation_goal_active()

                if nav_running and self._costmap_global_pending and \
                        (now - self._last_costmap_global_send) >= self._costmap_global_interval:
                    cm = self._state_bridge.get_latest_costmap('global')
                    if cm:
                        await self._ws_manager.broadcast(json.dumps(cm))
                        self._last_costmap_global_send = now
                        self._costmap_global_pending = False

                if nav_running and self._costmap_local_pending and \
                        (now - self._last_costmap_local_send) >= self._costmap_local_interval:
                    cm = self._state_bridge.get_latest_costmap('local')
                    if cm:
                        await self._ws_manager.broadcast(json.dumps(cm))
                        self._last_costmap_local_send = now
                        self._costmap_local_pending = False

                if nav_running and self._plan_pending and \
                        (now - self._last_plan_send) >= self._plan_interval:
                    plan = self._state_bridge.get_latest_plan()
                    if plan:
                        await self._ws_manager.broadcast(json.dumps(plan))
                        self._last_plan_send = now
                        self._plan_pending = False

                # Phase C cleanup: xoa costmap/plan khi Nav2 tat hoac goal ket thuc
                # (1 cho duy nhat theo spec, tranh phai sua cac diem gan _current_state)
                if not nav_running:
                    any_cached = (
                        self._state_bridge.get_latest_costmap('global') is not None
                        or self._state_bridge.get_latest_costmap('local') is not None
                        or self._state_bridge.get_latest_plan() is not None
                    )
                    if any_cached:
                        self._state_bridge.clear_layers()
                        # Reset pending/sig de khong emit ban tin cu
                        self._costmap_global_pending = False
                        self._costmap_local_pending = False
                        self._plan_pending = False
                        self._costmap_global_sig = None
                        self._costmap_local_sig = None
                        # Broadcast 3 ban tin rong de frontend xoa
                        await self._ws_manager.broadcast(json.dumps(
                            {"type": "costmap", "layer": "global",
                             "frame_id": "map", "data": None}))
                        await self._ws_manager.broadcast(json.dumps(
                            {"type": "costmap", "layer": "local",
                             "frame_id": "map", "data": None}))
                        await self._ws_manager.broadcast(json.dumps(
                            {"type": "plan", "frame_id": "map", "points": []}))
                        if self._verbose:
                            self.get_logger().info('[PhaseC] Cleared costmap/plan (Nav2 stopped)')
                elif nav_running and not goal_active \
                        and (time.monotonic() - self._plan_received_t) > 1.5:
                    # Goal vua ket thuc nhung Nav2 van chay -> chi xoa plan
                    cached_plan = self._state_bridge.get_latest_plan()
                    # PhaseC fix: chi broadcast neu plan cu CO noi dung; neu da rong
                    # (broadcast o lan truoc da set diem = []) thi khong gui nua.
                    if cached_plan is not None and cached_plan.get("points"):
                        self._state_bridge.update_plan(
                            {"type": "plan", "frame_id": "map", "points": []})
                        self._plan_pending = False
                        await self._ws_manager.broadcast(json.dumps(
                            {"type": "plan", "frame_id": "map", "points": []}))
                        if self._verbose:
                            self.get_logger().info('[PhaseC] Cleared plan (no active goal)')

                # P1-13 Fix: Broadcast nav_state periodically (1 Hz)
                if now - last_nav >= 1.0:
                    if self._nav_bridge:
                        await self._ws_manager.broadcast(
                            json.dumps(self._nav_bridge.get_navigation_state())
                        )
                    last_nav = now

                await asyncio.sleep(0.1)

            except asyncio.CancelledError:
                raise
            except Exception as e:
                if self._verbose:
                    self.get_logger().error(f'Broadcast tick error: {e}')
                # Tranh vong lap busy khong nhuong event loop khi loi lap lai
                await asyncio.sleep(0.5)

    async def _broadcast_nav_state(self, message: str) -> None:
        """Callback để broadcast navigation state qua WebSocket (legacy compatibility)."""
        if self._ws_manager and self._loop and not self._loop.is_closed():
            await self._ws_manager.broadcast(message)

    def _process_and_broadcast_cam(self, msg: Image, cam_type: str) -> None:
        """Hàm chung cập nhật heartbeat và encode ảnh khi có yêu cầu từ client."""
        self._state_bridge.update_heartbeat('camera')

        # Nếu không stream hoặc client đang yêu cầu mode khác -> Bỏ qua ngay (0% CPU encode)
        if not self._camera_streaming or self._camera_mode != cam_type:
            return

        try:
            img_data = image_to_json(msg, max_width=480, quality=60)
            if img_data and self._loop and self._loop.is_running():
                img_data['cam_mode'] = cam_type
                asyncio.run_coroutine_threadsafe(
                    self._ws_manager.broadcast(json.dumps(img_data)),
                    self._loop
                )
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Camera encode error ({cam_type}): {e}')

    def _on_camera_rgb(self, msg: Image) -> None:
        self._process_and_broadcast_cam(msg, 'rgb')

    def _on_camera_depth(self, msg: Image) -> None:
        self._process_and_broadcast_cam(msg, 'depth')

    def _on_camera_debug(self, msg: Image) -> None:
        # Pallet detection overlay (bgr8). Reuses the demand-driven encode path
        # so it only costs CPU when a client selected the 'detect' cam mode.
        self._process_and_broadcast_cam(msg, 'detect')

    def _on_mission_status(self, msg: String) -> None:
        """Parse /pallet_mission/status JSON; keep a compact view for the web
        cargo card. A malformed/absent payload never blocks the bridge."""
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        self._mission_status = {
            'state': d.get('state'),
            'cargo': d.get('cargo'),       # None | True | False
            'us_cm': d.get('us_cm'),       # raw ultrasonic reading, cm (or None)
            'us_fresh': d.get('us_fresh') is True,
            'us_pick_confirmed': d.get('us_pick_confirmed') is True,
            'mode': d.get('mode'),         # pick | drop
            'cargo_class': d.get('cargo_class'),   # medium | tall | unknown | None
            'cargo_type': d.get('cargo_type'),     # A | B | None
            'cargo_h_cm': d.get('cargo_h_cm'),     # measured box height, cm (or None)
            'lift_cm': d.get('lift_cm'),
            'ts': time.time(),
        }

    def _on_mission_supervisor_status(self, msg: String) -> None:
        """Keep the full mission cycle payload for Web and LLM status views."""
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        if isinstance(data, dict):
            previous = self._mission_supervisor_status
            if self._recent_mission_baseline_pending:
                self._recent_mission_baseline_pending = False
            elif previous:
                self._record_supervisor_mission(previous, data)
            self._mission_supervisor_status = {**data, 'ts': time.time()}

    def _clear_recent_supervisor_missions(self) -> None:
        """Start a fresh Recent Missions view for the next navigation session."""
        self._recent_supervisor_missions.clear()
        # The next supervisor payload establishes the new session baseline;
        # do not turn a transition left over from the previous map into history.
        self._recent_mission_baseline_pending = True

    @staticmethod
    def _mission_label(name: str, phase: str = '', destination: str = '') -> str:
        """Operator-facing name for a navigation or pallet-handling step."""
        phase = (phase or '').upper()
        dest = (destination or '').upper()
        if phase == 'GOTO_PICK':
            return {'P_1': 'Go to Pick 1', 'P_2': 'Go to Pick 2'}.get(
                dest, 'Go to pickup')
        if phase == 'PICKING':
            return 'Pick pallet'
        if phase == 'GOTO_D':
            return 'Go to shelf (D_1)'
        if phase == 'DROPPING':
            return 'Place pallet on shelf'
        if phase == 'WAITING_SLOT':
            return 'Wait for shelf space'
        if phase == 'RETURN_HOME':
            return 'Return to HOME'
        mission = (name or '').lower()
        if mission == 'return_home':
            return 'Return to HOME'
        if mission in {'p1_to_d1', 'p2_to_d1'}:
            return 'Go to shelf (D_1)'
        if mission in {'home_to_p1', 'home_to_p2', 'd1_to_p1', 'd1_to_p2',
                        'p1_to_p2'}:
            return 'Go to pickup'
        return 'Pallet mission'

    def _record_supervisor_mission(self, previous: dict, current: dict) -> None:
        old_state, new_state = previous.get('state'), current.get('state')
        completed = {
            ('GOTO_PICK', 'PICKING'): ('GOTO_PICK', 'Completed'),
            ('PICKING', 'GOTO_D'): ('PICKING', 'Completed'),
            ('GOTO_D', 'DROPPING'): ('GOTO_D', 'Completed'),
            ('DROPPING', 'RETURN_HOME'): ('DROPPING', 'Completed'),
            ('DROPPING', 'GOTO_PICK'): ('DROPPING', 'Completed'),
            ('DROPPING', 'DONE'): ('DROPPING', 'Completed'),
            ('RETURN_HOME', 'DONE'): ('RETURN_HOME', 'Completed'),
        }
        active_states = {'GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING',
                         'WAITING_SLOT', 'RETURN_HOME'}
        transition = completed.get((old_state, new_state))
        if transition:
            phase, outcome = transition
        elif old_state in active_states and new_state == 'RECOVERY':
            outcome = ('Canceled' if 'OPERATOR_CANCEL' in str(current.get('msg', ''))
                       else 'Recovery Required')
            phase = old_state
        elif old_state in active_states and new_state == 'STOPPED':
            outcome = 'Canceled'
            phase = old_state
        elif (old_state == 'RECOVERY' and previous.get('resume_available') is True
              and new_state == previous.get('resume_state')):
            outcome = 'Resumed'
            phase = previous.get('resume_state')
        else:
            phase = None
        # Detection reaches READY before the physical approach/pick starts.
        # Record this as its own completed task rather than hiding it under the
        # navigation leg to the pickup point.
        events = []
        if phase:
            mission = previous.get('active_mission') or current.get('active_mission')
            dest = previous.get('active_dest') or current.get('active_dest') or ''
            task_name = self._mission_label(mission, phase, dest)
            if phase == 'PICKING' and previous.get('pm_state') == 'DETECTING':
                task_name = 'Detect pallet'
            events.append((task_name, outcome,
                           previous.get('cycle') or {}))
        if (new_state == 'PICKING' and current.get('pm_state') == 'READY'
                and previous.get('pm_state') != 'READY'):
            events.append(('Detect pallet', 'Completed', current.get('cycle') or {}))
        # appendleft reverses insertion order, so emit older events first.
        for label, event_status, cycle in events:
            self._recent_supervisor_missions.appendleft({
                'time': datetime.now().strftime('%H:%M:%S'),
                'name': label,
                'status': event_status,
                'cycle': cycle.get('number', 0),
            })

    def get_mission_supervisor_status(self):
        """Return a copy of supervisor status with transport freshness metadata."""
        data = self._mission_supervisor_status
        if data is None:
            return None
        result = dict(data)
        result['stale'] = (time.time() - result.get('ts', 0.0)) > 3.0
        result['recent_missions'] = list(self._recent_supervisor_missions)
        return result

    def _is_navigation_goal_active(self) -> bool:
        """True for both Web Quick Goals and goals issued by Auto missions.

        Auto-cycle goals are sent by mission_manager, so NavBridge's own action
        client does not own their goal handle. Use fresh supervisor state to
        recognize those navigation phases for the plan overlay.
        """
        if self._nav_bridge and self._nav_bridge.is_goal_active():
            return True
        if any(self._nav2_goal_active.values()):
            return True
        status = self.get_mission_supervisor_status()
        if status is None or status.get('stale'):
            return False
        return status.get('state') in {'GOTO_PICK', 'GOTO_D', 'RETURN_HOME'}

    def _on_nav2_goal_status(self, name: str, msg: GoalStatusArray) -> None:
        active_statuses = (1, 2, 3)  # ACCEPTED, EXECUTING, CANCELING
        self._nav2_goal_active[name] = any(
            goal.status in active_statuses for goal in msg.status_list)

    def mission_cycle_conflict(self):
        """Return a reason when another web Nav2 goal would conflict with a cycle."""
        status = self.get_mission_supervisor_status()
        if status is None:
            return None
        state = status.get('state')
        if status.get('stale') and state in self._CYCLE_BLOCKED_COMMANDS:
            return 'Mission status is stale during an active cycle; verify the supervisor before sending another goal'
        if status.get('stale'):
            return None
        if state in self._CYCLE_BLOCKED_COMMANDS:
            return f'Mission cycle is active ({state}); stop it first'
        return None

    def _mark_supervisor_reset(self) -> None:
        """Update the local cache before the next periodic supervisor status."""
        data = self._mission_supervisor_status
        if data:
            self._mission_supervisor_status = {
                **data,
                'state': 'IDLE',
                'can_start': data.get('cargo') is not True,
                'resume_available': False,
                'resume_state': None,
                'ts': time.time(),
            }

    @staticmethod
    def _call_service_wait(client, request, timeout_s=3.0):
        """Wait for a service result from a non-ROS worker thread."""
        if not client.wait_for_service(timeout_sec=min(timeout_s, 1.0)):
            return None, f'Service unavailable: {client.srv_name}'
        done = threading.Event()
        result = {}

        def _on_done(future):
            try:
                result['value'] = future.result()
            except Exception as exc:  # noqa: BLE001
                result['error'] = str(exc)
            done.set()

        client.call_async(request).add_done_callback(_on_done)
        if not done.wait(timeout_s):
            return None, f'Service timed out: {client.srv_name}'
        if 'error' in result:
            return None, result['error']
        return result.get('value'), ''

    def mission_cycle_command(self, command: str, run_mode: str = 'auto'):
        """Control the mission supervisor without bypassing its state machine."""
        clients = {
            'mission_cycle_stop': self._mission_stop_client,
            'mission_cycle_resume': self._mission_resume_client,
        }
        if command == 'mission_cycle_start':
            if run_mode != 'auto':
                return False, 'Only Auto pallet cycles are supported'
            current = self.get_mission_supervisor_status()
            if current is None or current.get('stale'):
                return False, 'No fresh mission status from the supervisor'
            if current.get('can_start') is False:
                return False, f"Supervisor is busy ({current.get('state', 'unknown')})"
            if current.get('nav_status_fresh') and current.get('nav_goal_active') is True:
                return False, 'Nav2 has another active goal; finish it before starting the cycle'
            if self._nav_bridge:
                self._nav_bridge.set_op_mode('AUTO')
            req = SetParameters.Request()
            req.parameters = [
                Parameter('run_mode', Parameter.Type.STRING, run_mode).to_parameter_msg(),
                Parameter('target_cargo_class', Parameter.Type.STRING, '').to_parameter_msg(),
                Parameter('target_quantity', Parameter.Type.INTEGER, 1).to_parameter_msg(),
                Parameter('target_drop_slot', Parameter.Type.STRING, '').to_parameter_msg(),
            ]
            response, error = self._call_service_wait(
                self._mission_params_client, req)
            if response is None:
                return False, error
            if (len(response.results) != len(req.parameters) or
                    not all(item.successful for item in response.results)):
                return False, f'Could not set mission parameters for run_mode={run_mode}'
            client = self._mission_start_client
        else:
            client = clients.get(command)
            if client is None:
                return False, f'Invalid mission command: {command}'
            if command == 'mission_cycle_stop':
                stopped, stop_message = self._stop_robot_and_lift()
                if not stopped:
                    return False, stop_message
        response, error = self._call_service_wait(client, Trigger.Request())
        if response is None:
            return False, error
        if not response.success:
            state = (self.get_mission_supervisor_status() or {}).get('state', 'unknown')
            return False, f'Mission supervisor rejected the command ({state})'
        return True, response.message or 'ok'

    def set_operating_mode(self, mode: str):
        """Entering MANUAL from AUTO stops and resets the auto mission first."""
        if mode not in ('AUTO', 'MANUAL'):
            return False, 'Operating mode must be AUTO or MANUAL'
        stop_message = ''
        if mode == 'MANUAL' and self._nav_bridge \
                and self._nav_bridge._op_mode == 'AUTO':
            stopped, stop_message = self._stop_robot_and_lift()
            if not stopped:
                return False, stop_message
            status = self.get_mission_supervisor_status()
            if status is None or status.get('stale'):
                return False, 'stop not confirmed: cannot reset with missing/stale supervisor status'
            active_states = {'GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING',
                             'WAITING_SLOT', 'RETURN_HOME', 'RECOVERY'}
            if status.get('state') in active_states:
                response, error = self._call_service_wait(
                    self._mission_stop_client, Trigger.Request(), timeout_s=2.0)
                if response is None or not response.success:
                    return False, ('stop not confirmed: supervisor stop failed: '
                                   f'{error or getattr(response, "message", "") or "rejected"}')
            response, error = self._call_service_wait(
                self._mission_reset_client, Trigger.Request(), timeout_s=2.0)
            if response is None or not response.success:
                return False, ('Auto mission reset failed; mode remains AUTO: '
                               f'{error or getattr(response, "message", "") or "rejected"}')
            self._mark_supervisor_reset()
        if self._nav_bridge:
            self._nav_bridge.set_op_mode(mode)
        return True, stop_message or f'Operating mode set to {mode}'

    def _lookup_map_tf(self, frame_id: str) -> Optional[Tuple[float, float, float]]:
        """Phase B05: tra TF map_frame -> frame_id. Tra ve (x, y, yaw) hoac None neu loi.

        Voi frame_id = base_frame (base_footprint) ket qua CHINH LA pose cua robot
        trong khung map (AMCL: map->odom, EKF: odom->base_footprint, tf2 tu noi chuoi).
        Khong block callback (lookup_transform tra exception khi TF chua san sang).
        """
        if frame_id == self._map_frame:
            return (0.0, 0.0, 0.0)
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, frame_id, rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        p = t.transform.translation
        tyaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
        return (p.x, p.y, tyaw)

    def _on_pose(self, msg: Odometry) -> None:
        try:
            pose_data = odom_to_json(msg)
            # Luon luu ban odom tho de timer B05 dung lam "khuon" (giu nguyen
            # cac field phu nhu van toc), khong doc lai tu state_bridge.
            self._last_odom_pose = pose_data
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Pose error: {e}')
            return

        # Phase B05: neu pose theo khung map vua duoc publish gan day
        # (tu timer tf_lookup) thi khong ghi de bang pose odom.
        if time.time() - self._last_map_pose_t < 1.0:
            return
        self._state_bridge.update_pose(pose_data)

    def _on_battery(self, msg: BatteryState) -> None:
        try:
            percentage = None
            if not math.isnan(msg.percentage):
                percentage = round(msg.percentage)

            battery_data = {
                "type": "battery",
                "timestamp": time.time(),
                "percentage": percentage,
                "voltage": round(msg.voltage, 1) if not math.isnan(msg.voltage) else None,
                "current": round(msg.current, 1) if not math.isnan(msg.current) else None,
                "charging": msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            }
            self._state_bridge.update_battery(battery_data)
            self._state_bridge.update_heartbeat('esp32')
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Battery error: {e}')

    def _on_map(self, msg: OccupancyGrid) -> None:
        try:
            map_data = map_to_json(msg)
            self._state_bridge.update_map(map_data)
            self._map_pending = True
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Map error: {e}')

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        """Phase C: nhan global costmap, convert + luu + dat co pending.
        KHONG broadcast truc tiep tu callback (theo pattern cua _on_map).
        """
        if not (self._nav_bridge and self._nav_bridge.is_nav_running()):
            return
        try:
            data = costmap_to_json(msg, 'global')  # frame_id='map', no TF needed
            # Dedupe signature: hash data + origin
            import zlib
            sig = (zlib.crc32(data['data'].encode('ascii')),
                   data['info']['origin']['x'], data['info']['origin']['y'],
                   data['info']['yaw'], data['info']['width'], data['info']['height'])
            if sig == self._costmap_global_sig:
                return
            self._costmap_global_sig = sig
            self._state_bridge.update_costmap('global', data)
            self._costmap_global_pending = True
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Global costmap error: {e}')

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        """Phase C: nhan local costmap (frame_id='odom'), quy doi odom -> map qua TF.
        Neu TF lookup loi (AMCL chua san sang) -> bo qua ban tin.
        """
        if not (self._nav_bridge and self._nav_bridge.is_nav_running()):
            return
        try:
            tf = self._lookup_map_tf(msg.header.frame_id)
            if tf is None:
                return  # giu ban tin cu trong cache
            tx, ty, tyaw = tf
            # Quy doi origin (lower-left) tu odom -> map bang compose_planar
            ox, oy, oyaw = compose_planar(
                tx, ty, tyaw,
                msg.info.origin.position.x,
                msg.info.origin.position.y,
                quaternion_to_yaw(
                    msg.info.origin.orientation.x,
                    msg.info.origin.orientation.y,
                    msg.info.origin.orientation.z,
                    msg.info.origin.orientation.w,
                ),
            )
            data = costmap_to_json(msg, 'local', origin=(ox, oy, oyaw))
            # Dedupe signature: data goc + origin goc + TF da lam tron
            import zlib
            sig = (zlib.crc32(data['data'].encode('ascii')),
                   msg.info.origin.position.x, msg.info.origin.position.y,
                   msg.info.width, msg.info.height,
                   round(tx, 2), round(ty, 2), round(tyaw, 3))
            if sig == self._costmap_local_sig:
                return
            self._costmap_local_sig = sig
            self._state_bridge.update_costmap('local', data)
            self._costmap_local_pending = True
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Local costmap error: {e}')

    def _on_plan(self, msg: Path) -> None:
        """Cache a valid plan while Nav2 runs; route BTs may publish only once."""
        if not (self._nav_bridge and self._nav_bridge.is_nav_running()):
            return
        try:
            tf = None
            if msg.header.frame_id != self._map_frame:
                tf = self._lookup_map_tf(msg.header.frame_id)
                if tf is None:
                    return
            data = plan_to_json(msg, tf=tf)
            if not data['points']:
                return
            self._state_bridge.update_plan(data)
            self._plan_received_t = time.monotonic()
            self._plan_pending = True
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Plan error: {e}')

    def _on_lidar(self, msg: LaserScan) -> None:
        self._state_bridge.update_heartbeat('lidar')

    def _on_laser(self, msg: LaserScan) -> None:
        # Chi convert theo nhip broadcast (10 Hz), bo qua cac ban tin thua
        now = time.time()
        if now - self._last_laser_convert < self._laser_interval:
            return
        self._last_laser_convert = now
        try:
            laser_data = laser_to_json(msg)
            self._state_bridge.update_laser(laser_data)
        except Exception as e:
            if self._verbose:
                self.get_logger().warn(f'Laser error: {e}')

    def _on_joint(self, msg: JointState) -> None:
        if 'encoder_lift_joint' in msg.name:
            self._state_bridge.update_heartbeat('lift')
            try:
                joint_idx = msg.name.index('encoder_lift_joint')
                lift_height_m = max(0.0, float(msg.position[joint_idx]))
                height_mm = int(round(lift_height_m * 1000.0))

                if not self._lift_initialized:
                    self._lift_initialized = True
                    self._state_bridge.set_lift_initialized(True)

                self._lift_height_mm = height_mm
                self._state_bridge.update_lift_height(self._lift_height_mm)
            except (ValueError, IndexError):
                pass

    def start_websocket_server(self) -> None:
        self._ws_thread = threading.Thread(target=self._run_server, daemon=True)
        self._ws_thread.start()

    # Phase B05 (FIXED): Timer tra TF map <- base_frame theo pose_rate.
    # Khi Nav2/AMCL dang chay va co TF, publish pose theo khung 'map' de frontend
    # dung cho robot icon + lidar scan.
    #
    # QUAN TRONG: lookup_transform(map, base_footprint) DA LA pose robot trong khung map
    # (tf2 tu noi chuoi map->odom (AMCL) va odom->base_footprint (EKF)).
    # Vi vay KHONG nhan them voi pose odom (code cu nhan doi, va con tu doc lai
    # chinh pose map do tick truoc ghi ra -> cong don -> troi vang).
    def _tf_lookup_timer(self) -> None:
        # Cho phép cập nhật Pose từ TF Map khi Nav2 chạy HOẶC khi đang mở SLAM
        nav_is_running = self._nav_bridge and self._nav_bridge.is_nav_running()
        slam_is_running = self.get_slam_state()

        if not (nav_is_running or slam_is_running):
            return

        tf = self._lookup_map_tf(self._base_frame)
        if tf is None:
            # Chưa lấy được TF map->base (ví dụ SLAM/AMCL chưa phát TF)
            return

        mx, my, myaw = tf
        myaw = math.atan2(math.sin(myaw), math.cos(myaw))  # Chuẩn hóa góc [-pi, pi]

        # Dùng bản odom thô làm khuôn giữ các thông số phụ
        if self._last_odom_pose is not None:
            map_pose = copy.deepcopy(self._last_odom_pose)
        else:
            map_pose = {}

        map_pose["type"] = "pose"
        map_pose["timestamp"] = time.time()
        map_pose["frame_id"] = self._map_frame
        map_pose["position"] = {"x": round(mx, 4), "y": round(my, 4), "z": 0.0}
        map_pose["orientation"] = {"yaw": round(myaw, 4)}

        self._state_bridge.update_pose(map_pose)
        self._last_map_pose_t = time.time()

        if not self._map_pose_logged:
            self._map_pose_logged = True
            if self._verbose:
                self.get_logger().info(
                    f'[B05] Map-frame pose active: frame={self._map_frame}, '
                    f'base={self._base_frame}, pose=({mx:.3f},{my:.3f},{myaw:.3f})'
                )

    def _history_sample_pose(self) -> None:
        """Timer callback (5 Hz). Pushes current pose into the active
        RunSession, if any. Soft-fail: never raises."""
        if not _HISTORY_OK:
            return
        try:
            from amr_web.history import runs as _history_runs
            session = _history_runs.get_active()
            if session is None:
                return
            pose = self._state_bridge.get_latest_pose()
            if not pose:
                return
            pos = pose.get("position") or {}
            ori = pose.get("orientation") or {}
            x = pos.get("x")
            y = pos.get("y")
            yaw = ori.get("yaw")
            if x is None or y is None or yaw is None:
                return
            # Wall-clock time on purpose: the pose "timestamp" is ROS/odom time, which is
            # not guaranteed to be on the same epoch as the run's start time.
            session.sample_pose(time.time(), float(x), float(y), float(yaw))
        except Exception:
            pass

    def _run_server(self) -> None:
        config = uvicorn.Config(
            self._app, host=self._host, port=self._port,
            log_level="info" if self._verbose else "warning"
        )
        server = uvicorn.Server(config)
        server.run()

    # SLAM Process Management
    def start_slam(self) -> bool:
        if self._slam_process and self._slam_process.poll() is None:
            self.get_logger().warn('SLAM process still running')
            return False

        try:
            ws_setup = os.path.expanduser('~/project_ws/install/setup.bash')
            cmd = f'source {ws_setup} && ros2 launch amr_mapping slam_web.launch.py'

            self._slam_process = subprocess.Popen(
                ['bash', '-c', cmd],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )
            self._slam_active = True
            self.get_logger().info('SLAM started: amr_mapping slam_web.launch.py')

            time.sleep(0.5)
            if self._slam_process.poll() is not None:
                self.get_logger().error('SLAM process exited immediately')
                self._slam_active = False
                return False

            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start SLAM: {e}')
            self._slam_active = False
            self._slam_process = None
            return False

    def get_slam_state(self) -> bool:
        if self._slam_process and self._slam_process.poll() is None:
            return True
        return self._slam_active

    def stop_slam(self) -> None:
        try:
            subprocess.run(
                ['bash', '-c',
                 'source ~/project_ws/install/setup.bash 2>/dev/null && '
                 'ros2 node kill /slam_toolbox 2>/dev/null; '
                 'ros2 node kill /octomap_server 2>/dev/null; '
                 'ros2 lifecycle set /slam_toolbox shutdown 2>/dev/null || true'],
                capture_output=True, timeout=5.0
            )
            self.get_logger().info('ROS nodes killed')
        except Exception as e:
            self.get_logger().warn(f'Node kill warning: {e}')

        if self._slam_process:
            try:
                pgid = os.getpgid(self._slam_process.pid)
                os.killpg(pgid, signal.SIGKILL)
                self._slam_process.wait(timeout=2.0)
                self.get_logger().info('SLAM process killed (SIGKILL)')
            except ProcessLookupError:
                self.get_logger().warn('SLAM process already dead')
            except Exception as e:
                self.get_logger().error(f'Failed to kill SLAM: {e}')
                try:
                    self._slam_process.kill()
                except:
                    pass
            finally:
                self._slam_process = None
                self._slam_active = False

    def save_map(self, map_name: str) -> tuple:
        maps_dir = os.path.expanduser('~/project_ws/maps')
        os.makedirs(maps_dir, exist_ok=True)

        map_path = os.path.join(maps_dir, map_name)
        map_path_abs = os.path.expanduser(map_path)

        try:
            result = subprocess.run(
                [
                    'ros2', 'service', 'call',
                    '/slam_toolbox/save_map',
                    'slam_toolbox/srv/SaveMap',
                    f'{{name: {{data: "{map_path_abs}"}}}}'
                ],
                capture_output=True,
                text=True,
                timeout=30.0
            )

            if result.returncode == 0:
                yaml_path = f"{map_path_abs}.yaml"
                self.get_logger().info(f'Map saved: {yaml_path}')
                return True, yaml_path
            else:
                self.get_logger().error(f'Map save failed: {result.stderr}')
                return False, ""

        except subprocess.TimeoutExpired:
            self.get_logger().error('Map save timeout')
            return False, ""
        except Exception as e:
            self.get_logger().error(f'Map save error: {e}')
            return False, ""

    def get_available_maps(self) -> list:
        maps = []
        try:
            maps_dir = os.path.expanduser('~/project_ws/maps')
            if os.path.exists(maps_dir):
                for f in sorted(os.listdir(maps_dir)):
                    if f.endswith('.yaml'):
                        full_path = os.path.join(maps_dir, f)
                        map_id = f.replace('.yaml', '')
                        try:
                            created = time.strftime('%Y-%m-%d', time.localtime(os.path.getctime(full_path)))
                        except:
                            created = time.strftime('%Y-%m-%d')
                        maps.append({
                            'id': map_id,
                            'name': map_id,
                            'path': full_path,
                            'created': created
                        })
        except Exception as e:
            self.get_logger().warn(f'Maps error: {e}')

        return maps

    def get_available_routes(self, map_id: str = None) -> list:
        import yaml
        routes = []
        try:
            if map_id:
                routes_dir = os.path.expanduser(f'~/project_ws/maps/{map_id}/routes')
                if os.path.exists(routes_dir):
                    for f in sorted(os.listdir(routes_dir)):
                        if f.endswith('.yaml') and not f.endswith('_poses.yaml'):
                            full_path = os.path.join(routes_dir, f)
                            route_id = f.replace('.yaml', '')
                            try:
                                created = time.strftime('%Y-%m-%d', time.localtime(os.path.getctime(full_path)))
                            except:
                                created = time.strftime('%Y-%m-%d')

                            nodes = []
                            edges = []
                            try:
                                with open(full_path, 'r', encoding='utf-8') as rf:
                                    rdata = yaml.safe_load(rf) or {}
                                    raw_nodes = rdata.get('nodes', {})
                                    if isinstance(raw_nodes, dict):
                                        nodes = list(raw_nodes.values())
                                    elif isinstance(raw_nodes, list):
                                        nodes = raw_nodes
                                    edges = rdata.get('edges', [])
                            except Exception as re:
                                self.get_logger().warn(f'Error reading route file {full_path}: {re}')

                            routes.append({
                                'id': route_id,
                                'name': route_id,
                                'path': full_path,
                                'created': created,
                                'graph_name': route_id,
                                'nodes': nodes,
                                'edges': edges,
                                'nodeCount': len(nodes),
                                'edgeCount': len(edges)
                            })
        except Exception as e:
            self.get_logger().warn(f'Routes error: {e}')

        return routes

    def save_route(self, map_id: str, route_data: dict) -> tuple:
        import yaml
        try:
            routes_dir = os.path.expanduser(f'~/project_ws/maps/{map_id}/routes')
            os.makedirs(routes_dir, exist_ok=True)

            graph_name = route_data.get('graph_name', f'{map_id}_route')
            yaml_path = os.path.join(routes_dir, f'{graph_name}.yaml')
            geojson_path = os.path.join(routes_dir, f'{graph_name}.geojson')
            poses_path = os.path.join(routes_dir, f'{graph_name}_poses.yaml')

            yaml_content = {
                'graph_name': graph_name,
                'version': '1.0',
                'nodes': {},
                'edges': route_data.get('edges', [])
            }
            for idx, node in enumerate(route_data.get('nodes', [])):
                yaml_content['nodes'][node['name']] = node

            with open(yaml_path, 'w', encoding='utf-8') as f:
                yaml.dump(yaml_content, f, default_flow_style=False, sort_keys=False)

            node_id_map = {}
            features = []
            for idx, (name, n_data) in enumerate(yaml_content['nodes'].items()):
                node_id_map[name] = idx
                pos = n_data.get('position', {})
                features.append({
                    'type': 'Feature',
                    'properties': {'id': idx, 'frame': 'map'},
                    'geometry': {'type': 'Point', 'coordinates': [pos.get('x', 0.0), pos.get('y', 0.0)]}
                })

            for edge in yaml_content['edges']:
                from_name, to_name = edge.get('from'), edge.get('to')
                if from_name in node_id_map and to_name in node_id_map:
                    from_pos = yaml_content['nodes'][from_name]['position']
                    to_pos = yaml_content['nodes'][to_name]['position']
                    features.append({
                        'type': 'Feature',
                        'properties': {'id': len(features), 'startid': node_id_map[from_name], 'endid': node_id_map[to_name]},
                        'geometry': {'type': 'MultiLineString', 'coordinates': [[[from_pos['x'], from_pos['y']], [to_pos['x'], to_pos['y']]]]}
                    })

            geojson_data = {
                'type': 'FeatureCollection',
                'name': graph_name,
                'crs': {'type': 'name', 'properties': {'name': 'urn:ogc:def:crs:EPSG::4326'}},
                'features': features
            }
            with open(geojson_path, 'w', encoding='utf-8') as f:
                json.dump(geojson_data, f, indent=2)

            poses_data = {'frame_id': 'map', 'poses': {}}
            for name, n_data in yaml_content['nodes'].items():
                pos = n_data.get('position', {})
                yaw = n_data.get('orientation', {}).get('yaw', 0.0)
                poses_data['poses'][name] = {
                    'id': node_id_map[name],
                    'pose_stamped': {
                        'header': {'frame_id': 'map'},
                        'pose': {
                            'position': {'x': pos.get('x', 0.0), 'y': pos.get('y', 0.0), 'z': pos.get('z', 0.0)},
                            'orientation': {'x': 0.0, 'y': 0.0, 'z': math.sin(yaw / 2.0), 'w': math.cos(yaw / 2.0)}
                        }
                    }
                }
            with open(poses_path, 'w', encoding='utf-8') as f:
                yaml.safe_dump(poses_data, f, sort_keys=False)

            self.get_logger().info(f'Route saved successfully: {yaml_path}')
            return True, "Route and associated files saved successfully"

        except Exception as e:
            self.get_logger().error(f'Failed to save route: {e}')
            return False, str(e)

    def delete_map_and_routes(self, map_id: str) -> tuple:
        import shutil
        try:
            maps_dir = os.path.expanduser('~/project_ws/maps')
            yaml_path = os.path.join(maps_dir, f'{map_id}.yaml')
            pgm_path = os.path.join(maps_dir, f'{map_id}.pgm')
            routes_dir = os.path.join(maps_dir, map_id)

            if os.path.exists(yaml_path): os.remove(yaml_path)
            if os.path.exists(pgm_path): os.remove(pgm_path)
            if os.path.exists(routes_dir): shutil.rmtree(routes_dir)

            self.get_logger().info(f'Deleted map and routes for {map_id}')
            return True, "Map and routes deleted"
        except Exception as e:
            self.get_logger().error(f'Failed to delete map: {e}')
            return False, str(e)

    def publish_cmd_vel(self, linear: float, angular: float) -> None:
        twist = Twist()
        twist.linear.x = float(linear)
        twist.angular.z = float(angular)
        self._cmd_vel_pub.publish(twist)

    def publish_lift(self, action: str, target_mm: int) -> None:
        cmd = String()
        if action == "UP":
            cmd.data = "M30"
        elif action == "DOWN":
            cmd.data = "M-30"
        elif action == "HOME":
            cmd.data = "HOME"
        elif action == "SET_HEIGHT":
            cmd.data = f"H{target_mm / 10:.1f}"
        else:
            return

        self._esp32_cmd_pub.publish(cmd)
        if self._verbose:
            self.get_logger().info(f'Lift cmd: {cmd.data}')

    # ============================================
    # Navigation Methods (NavBridge wrapper)
    # ============================================

    def start_navigation(self, map_id: str, route_id: Optional[str] = None) -> tuple:
        if not self._nav_bridge:
            return False, "NavBridge not initialized"

        try:
            # P0-5 Fix: Clear old map before starting navigation
            self._state_bridge.clear_map()
            self._map_pending = False

            maps_dir = os.path.expanduser('~/project_ws/maps')
            map_yaml = os.path.join(maps_dir, f'{map_id}.yaml')

            if not os.path.exists(map_yaml):
                return False, f"Map file not found: {map_yaml}"

            route_file = None
            if route_id:
                # Trỏ ưu tiên tới file .geojson (vì route_server bắt buộc đọc .geojson)
                candidate_geojson = os.path.join(maps_dir, map_id, 'routes', f'{route_id}.geojson')
                candidate_yaml = os.path.join(maps_dir, map_id, 'routes', f'{route_id}.yaml')

                if os.path.exists(candidate_geojson):
                    route_file = candidate_geojson
                elif os.path.exists(candidate_yaml):
                    route_file = candidate_yaml
                else:
                    self.get_logger().warn(f"Route file not found for: {route_id}")

            # Gọi nav_bridge với file route chính xác (truyen day du map_id va route_id)
            success, msg = self._nav_bridge.start_nav2(
                map_path=map_yaml,
                route_path=route_file,
                map_id=map_id,
                route_id=route_id
            )

            if success:
                self._clear_recent_supervisor_missions()
            if success and self._verbose:
                self.get_logger().info(f'Navigation started: map={map_id}, route={route_id}')

            return success, msg

        except Exception as e:
            self.get_logger().error(f'Failed to start navigation: {e}')
            return False, str(e)

    def stop_navigation(self) -> tuple:
        """Stop Nav2 and stop/reset the supervisor cycle when in AUTO."""
        if not self._nav_bridge:
            return False, "NavBridge not initialized"

        warnings = []
        motion_stop_ok = True
        state = None
        supervisor_stopped = False
        if self._nav_bridge._op_mode == 'AUTO':
            stop_ok, stop_message = self._stop_robot_and_lift()
            motion_stop_ok = stop_ok
            status = self.get_mission_supervisor_status()
            state = (status.get('state')
                     if status is not None and not status.get('stale') else None)
            supervisor_stopped = True

            if state in self._CYCLE_RUNNING:
                response, error = self._call_service_wait(
                    self._mission_stop_client, Trigger.Request(), timeout_s=2.0)
                supervisor_stopped = bool(response and response.success)
                if not supervisor_stopped:
                    warnings.append(
                        'supervisor stop not confirmed: '
                        f'{error or getattr(response, "message", "") or "rejected"}')

            if not stop_ok:
                warnings.append(stop_message)
            # Stop Navigation ends the current cycle, including any pending
            # RECOVERY checkpoint. Only mission_cycle_stop (operator cancel)
            # preserves a checkpoint for a later Resume.
            if state == 'RECOVERY' or (state in self._CYCLE_RUNNING and supervisor_stopped):
                if supervisor_stopped:
                    response, error = self._call_service_wait(
                        self._mission_reset_client, Trigger.Request(), timeout_s=2.0)
                    if response and response.success:
                        self._mark_supervisor_reset()
                    else:
                        warnings.append(
                            'supervisor reset failed: '
                            f'{error or getattr(response, "message", "") or "rejected"}')
            elif state is None:
                warnings.append('supervisor status unavailable/stale; mission reset not attempted')

        # P0-5 Fix: Clear map when stopping navigation
        self._state_bridge.clear_map()
        self._map_pending = False
        self._nav2_goal_active = {key: False for key in self._nav2_goal_active}
        nav_ok, nav_message = self._nav_bridge.stop_nav2()
        if nav_ok:
            self._clear_recent_supervisor_missions()
        if warnings:
            nav_message += '; WARNING: ' + '; '.join(warnings)
        return (nav_ok and motion_stop_ok and self._nav_bridge._op_mode == 'MANUAL',
                nav_message)

    def send_nav_goal(self, pose: dict, route_id: Optional[str] = None) -> tuple:
        """
        Send navigation goal to Nav2.

        Args:
            pose: dict với keys 'x', 'y', 'yaw'
            route_id: Optional route identifier

        Returns:
            tuple: (success: bool, message: str)
        """
        if not self._nav_bridge:
            return False, "NavBridge not initialized"

        return self._nav_bridge.send_navigation_goal(pose, route_id)

    def cancel_nav_goal(self) -> tuple:
        """Stop Nav2, base motion and pallet lift; report any unconfirmed step."""
        if not self._nav_bridge:
            return False, "stop not confirmed: NavBridge not initialized"
        stopped, message = self._stop_robot_and_lift()
        if not stopped:
            return False, message
        if self._nav_bridge._op_mode == 'AUTO':
            status = self.get_mission_supervisor_status()
            if status is None or status.get('stale'):
                return False, 'stop not confirmed: Auto supervisor state is unavailable/stale'
            active_states = {'GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING',
                             'WAITING_SLOT', 'RETURN_HOME', 'RECOVERY'}
            if status.get('state') in active_states:
                response, error = self._call_service_wait(
                    self._mission_stop_client, Trigger.Request(), timeout_s=2.0)
                if response is None:
                    return False, f'stop not confirmed: supervisor Recovery transition failed: {error}'
                if not response.success:
                    return False, ('stop not confirmed: supervisor rejected Recovery transition: '
                                   f'{response.message or "rejected"}')
                message += '; supervisor Recovery acknowledged'
        return True, message

    def _stop_robot_and_lift(self) -> tuple:
        failures = []
        try:
            self._nav_bridge._emergency_stop()
        except Exception as exc:  # noqa: BLE001
            failures.append(f'zero velocity not confirmed: {exc}')
        try:
            nav_ok, nav_msg = self._nav_bridge.cancel_navigation(emergency_stop=False)
            if not nav_ok:
                failures.append(nav_msg)
        except Exception as exc:  # noqa: BLE001
            failures.append(f'Nav2 cancel unconfirmed: {exc}')

        pallet_note = ''
        try:
            response, error = self._call_service_wait(
                self._pallet_stop_client, Trigger.Request(), timeout_s=2.0)
        except Exception as exc:  # noqa: BLE001
            response, error = None, str(exc)
        if response is None and error.startswith('Service unavailable'):
            if self._esp32_cmd_pub.get_subscription_count() == 0:
                failures.append('lift stop not confirmed: no subscriber on /esp32_cmd')
            else:
                for _ in range(2):
                    self._esp32_cmd_pub.publish(String(data='M0'))
                pallet_note = 'pallet_mission offline; lift M0 sent directly'
        elif response is None:
            failures.append(f'pallet stop not confirmed: {error}')
        elif not response.success:
            failures.append(f'pallet stop not confirmed: {response.message or "rejected"}')
        else:
            pallet_note = f'pallet stop confirmed: {response.message or "stopped"}'

        if failures:
            return False, 'stop not confirmed: ' + '; '.join(failures)
        return True, f'{nav_msg}; {pallet_note}; zero velocity published'

    def set_nav_initial_pose(self, pose: dict) -> tuple:
        """
        Set initial pose cho AMCL.

        Args:
            pose: dict với keys 'x', 'y', 'yaw'

        Returns:
            tuple: (success: bool, message: str)
        """
        if not self._nav_bridge:
            return False, "NavBridge not initialized"

        return self._nav_bridge.set_initial_pose(pose)

    def get_nav_state(self) -> dict:
        """
        Get current navigation state.

        Returns:
            dict: Navigation state info
        """
        if not self._nav_bridge:
            return {'type': 'nav_state', 'state': 'UNAVAILABLE'}

        return self._nav_bridge.get_navigation_state()

    def is_navigation_running(self) -> bool:
        """Check if Nav2 is running."""
        if not self._nav_bridge:
            return False
        return self._nav_bridge.is_nav_running()

    def is_navigation_active(self) -> bool:
        """Check if navigation goal is active."""
        if not self._nav_bridge:
            return False
        return self._nav_bridge.is_nav_active()

    def send_waypoint_mission(self, waypoints: list, mission_type: str = 'waypoints') -> tuple:
        """
        Send waypoint/route mission to Nav2.

        Args:
            waypoints: List of pose dicts [{x, y, yaw, name}, ...]
            mission_type: 'waypoints' hoặc 'route'

        Returns:
            tuple: (success: bool, message: str)
        """
        if not self._nav_bridge:
            return False, "NavBridge not initialized"

        return self._nav_bridge.send_waypoint_mission(waypoints, mission_type)

    def emergency_stop_robot(self) -> tuple:
        """
        Cancel Nav2, publish zero velocity and stop the pallet lift.

        Returns:
            tuple: (success: bool, message: str)
        """
        if not self._nav_bridge:
            return False, "stop not confirmed: NavBridge not initialized"

        try:
            success, message = self._stop_robot_and_lift()
            # [history] Flag the open run (if any) so its result reflects the E-stop.
            if _HISTORY_OK:
                try:
                    from amr_web.history import runs as _history_runs
                    _history_runs.mark_estop()
                except Exception:
                    pass
            return success, message
        except Exception as e:
            self.get_logger().error(f'Emergency stop failed: {e}')
            return False, f'stop not confirmed: {e}'


def main(args=None):
    rclpy.init(args=args)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    bridge = WebBridge()
    executor.add_node(bridge)

    bridge.start_websocket_server()
    time.sleep(1.0)

    try:
        # Phase 1 Fix: No more _broadcast_thread - using uvicorn's event loop instead
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        bridge._running = False
        bridge.stop_slam()
        if bridge._nav_bridge:
            bridge._nav_bridge.stop_nav2()
        executor.remove_node(bridge)
        bridge.destroy_node()
        # [history] Flush writer thread + mark open runs INTERRUPTED.
        if _HISTORY_OK:
            try:
                _history_db.shutdown()
            except Exception:
                pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
