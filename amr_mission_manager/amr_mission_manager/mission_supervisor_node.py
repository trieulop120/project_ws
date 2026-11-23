#!/usr/bin/env python3
"""
mission_supervisor_node.py
==========================
Mission Supervisor: integrates existing mission_manager and pallet_mission services.

    single: HOME -> P_1 -> adaptive detect -> pick -> D_1 -> drop, repeat
            until the requested quantity is placed, then HOME -> DONE
    auto:   repeat the cycle until no pallet is detected at P_1 or no compatible
            known-empty shelf slot remains; then return HOME.

Detection ends when enough samples have arrived and the live pallet pose has
remained within configured position/yaw tolerances for a settling interval,
or when the maximum sampling timeout expires. Pick/drop mechanics remain owned
by the existing pallet_mission node. STOP is available throughout; RECOVERY
stores a supervisor checkpoint and requires an explicit Resume command.

Web-ready (muc 7 report):
    - Trạng thái: 1 topic JSON duy nhất /mission/status
      {state, msg, cycle:{...}, pm_state, pm_phase, lift_cm}
    - Lệnh: services std_srvs (start/stop) - command DOWN, ack UP.

Chạy (cùng pallet_mission.launch.py, KHONG chay them mission_manager rieng):
    ros2 launch amr_mission_manager mission_supervisor.launch.py
    ros2 service call /mission/start std_srvs/srv/Trigger '{}'
    # override mission TRUOC khi start (hot, khong can restart):
    ros2 param set /mission_supervisor pick_mission home_to_p1
    ros2 service call /mission/stop std_srvs/srv/Trigger '{}'
"""

import json
import math
from amr_mission_manager.pallet_recovery import navigation_stopped
from amr_mission_manager.aruco_route_gate import ArucoRouteGate
import time
import threading
import uuid
from functools import wraps

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String, Bool, Int8MultiArray
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, SetBool, Trigger
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       qos_profile_sensor_data)

from amr_mission_manager.shelf_model import ShelfModel, UNKNOWN, EMPTY, FULL
from amr_mission_manager.mission_queue import normalize_mission_queue, task_after_placement, mission_start_blockers

NODE_NAME = 'mission_supervisor'


def status_counter(status, field):
    value = status.get(field) if isinstance(status, dict) else None
    return value if isinstance(value, int) and not isinstance(value, bool) and value >= 0 else None

IDLE = 'IDLE'
GOTO_PICK = 'GOTO_PICK'
PICKING = 'PICKING'
GOTO_D = 'GOTO_D'
DROPPING = 'DROPPING'
DONE = 'DONE'
FAILED = 'FAILED'
STOPPED = 'STOPPED'
# Ke da day (xe dang cam pallet, khong con o trong dung loai): DUNG TAI D_1,
# tu tham do lai o trong. KHONG phai loi -> KHONG vao RECOVERY.
WAITING_SLOT = 'WAITING_SLOT'

# Sub-phases trong PICKING
S_SAMPLING = 'sampling'
S_APPROACHING = 'approaching'

DEFAULTS = {
    # Mission manager (names must match MISSION_BT_MAP of mission_manager_node)
    'pick_mission': 'home_to_p1',
    'drop_mission': 'p1_to_d1',
    'pick_dest': 'P_1',            # node graph (map frame) duy nhat de lay pallet
    'run_mode': 'single',          # bounded quantity + HOME; auto: loop until source/slots empty
    # Bounded LLM mission: require exactly N successfully placed items of the
    # requested detector class. Empty class means any shelf-compatible cargo.
    'target_cargo_class': '',      # '' | 'medium' | 'tall'
    'target_quantity': 1,          # applied to run_mode=single
    'target_drop_slot': '',        # '' = supervisor priority | exact A1/A2/B1/B2 for one LLM cycle
    'target_queue_json': '',       # ordered bounded tasks owned by this supervisor run
    # W4 (loi 2 ESP serial -> /shelf/occupancy): 4 bit o ke, 1 = co hang;
    # -1 = mat lien lac. Anh xa bit -> o va thiet ke ke: config/shelf.yaml
    # (doc bang shelf_model). KHONG CON drop_bin_fallback: khong ro (mat
    # lien lac / het o tron / chua chon loai hang) -> KHONG dat hang,
    # bao len status/GUI (dung tai cho, khong tu ve Home).
    'shelf_topic': '/shelf/occupancy',
    'shelf_stale_s': 10.0,       # du lieu ke doi cham: > 10s khong co dat = coi mat lien lac
    'shelf_config': '',          # '' = config/shelf.yaml (mac dinh)
    'camera_topic': '/color/image_raw',
    'camera_stale_s': 4.0,
    'mission_manager_node': 'mission_manager',
    'pallet_mission_node': 'pallet_mission',
    # Frames
    'map_frame': 'map',
    'base_frame': 'base_footprint',
    'odom_topic': '/odometry/filtered',
    'nav_status_topic': '/navigate_to_pose/_action/status',
    # Den choi GOTO_PICK
    'arrive_tol_m': 0.30,
    'arrive_settle_s': 1.0,
    'arrive_max_linear_speed_mps': 0.05,
    'arrive_max_angular_speed_rps': 0.12,
    'goto_timeout_s': 180.0,
    # PICKING
    'pick_sample_wait_s': 20.0,  # hard timeout; includes model load only when prewarm is disabled
    'sample_stable_s': 2.0,
    'no_pallet_conclude_s': 10.0,  # detector DA CHAY ma sau ngan nay van 0 mau -> ket luan khong co pallet
    'no_pallet_retries': 1,      # 1 = ket luan ngay lan quet trong dau tien (-> ve HOME)
    'detector_restarts': 2,      # pallet_mission FAILED luc detect (detector chet...) -> tu bat lai tu ngan nay lan
    'pick_retries': 3,
    # --- tu phuc hoi (truoc khi can nguoi bam RESUME) ---
    'approach_retries': 2,       # so lan tu thu lai pick/drop khi approach FAILED TRUOC khi cang vao
    'shelf_wait_s': 30.0,        # du lieu ke stale: cho toi da bay nhieu giay roi moi RECOVERY
    'retry_backoff_s': 2.0,      # nghi giua cac lan thu lai
    'start_retries': 5,          # pallet_mission tu choi Drop (khong chuyen dong) -> thu lai moi 2s
    'nav_finish_wait_s': 8.0,    # xe da dung o D_1 nhung Nav2 goal van active > nguong -> huy goal
    'pick_timeout_s': 420.0,
    'drop_timeout_s': 360.0,     # cho toi bao lau o DROPPING truoc khi can nguoi
    # Ke da day: xe DUNG TAI D_1, tu tham do lai /shelf/occupancy moi
    # slot_poll_s giay. Co o trong -> dat ngay (khong can ngui).
    # Qua slot_wait_max_s -> bao ve nguoi van hanh (van giu pallet).
    'slot_poll_s': 5.0,
    'slot_wait_max_s': 900.0,
    # Bo tham so SAMPLING rong (demo 1-1.5 Hz) ap len pallet_mission (HOT,
    # khong sua node cu - chi la parameter service, pallet_mission self._reload_params)
    'pm_final_window_s': 4.0,
    'pm_min_samples': 3,
    'pm_max_pos_std_m': 0.05,
    'pm_max_yaw_std_deg': 5.0,
    'pm_max_sample_age_s': 3.0,
    'pm_require_stable': True,   # nua cuoi cung: false = bo luon kiem tra std
    'pm_after_pick': 'hold',     # supervisor BAT BUOC 'hold' (pallet_mission giu pallet sau khi lui)
}


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _state_locked(method):
    @wraps(method)
    def locked(self, *args, **kwargs):
        with self._transition_lock:
            return method(self, *args, **kwargs)
    return locked


class MissionSupervisor(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in DEFAULTS}
        self._client_group = ReentrantCallbackGroup()
        self._control_group = ReentrantCallbackGroup()
        # _tick CHAN (time.sleep cho service) -> phai nam group RIENG, neu khong no
        # chan cac subscription (shelf/pm status/TF listener) cung default group.
        self._tick_group = MutuallyExclusiveCallbackGroup()
        self._shelf_bad_t0 = None
        self._approach_retries = 0
        self._drop_retries = 0
        self._pm_fail_phase = None
        self._pm_insertion_only = False
        self._retry_wait_until = 0.0
        self._stuck_t0 = None
        self._start_refusals = 0
        self._detector_restarts = 0
        self._detect_on_t = 0.0
        self._scan_t0 = None
        self._pm_msg = ''
        # Cache service clients. Creating clients on every 10 Hz timer retry
        # leaks ROS entities and makes startup/recovery progressively less
        # responsive when a server is offline.
        self._service_clients = {}
        # Separate grants; this node alone publishes their safe combination.
        self._aruco_route_gate = ArucoRouteGate()
        self._retrieval_aruco_gate = ArucoRouteGate()
        self._retrieval_gate_status = None
        self._retrieval_gate_t = 0.0

        # ---------------- State ----------------
        self._state = IDLE
        self._msg = 'San sang (goi /mission/start)'
        self._pick_phase = S_SAMPLING
        self._t_state = time.monotonic()
        self._triggered = False           # da goi trigger mission_manager?
        self._detect_on_called = False
        self._go_called = False
        self._retries = 0
        self._near_t0 = None
        self._pm_state = None             # state cuoi cua pallet_mission
        self._pm_phase = None
        self._pm_lift = None
        self._pm_sampling_applied = False
        self._detector_filter_applied = False
        self._pm_samples = 0
        self._pm_live = None
        self._pm_cargo_type = None
        self._pm_cargo_why = ''
        self._pm_cargo = None
        self._pm_status_t = 0.0
        self._detector_status = None
        self._detector_status_t = 0.0
        self._stable_since = None
        self._stable_anchor = None
        self._active_mission = self.p['pick_mission']
        self._active_dest = self.p['pick_dest']
        # Diem lay duy nhat trong mot chu ky: P_1 (da bo nhanh lay thu hai P_2/D_2).
        self._pick_queue = []
        self._pick_idx = 0
        self._route_loader = None
        self._route_path_loaded = None
        self._route_path = None
        self._route_path_error = ''
        self._route_path_last_received = 0.0
        self._cycle = 0
        self._resume_state = None
        self._pm_session_id = None
        self._pm_operation_id = -1
        self._await_operation_after = None
        self._uncertain_child_request = None
        self._uncertain_child_session = None
        self._await_resume_checkpoint = None
        self._pm_recovery = None
        self._pm_handling_status = None
        self._pm_waiting_for = None
        self._recovery_stop_future = None
        self._recovery_cancel_future = None
        self._recovery_still_since = None
        self._recovery_started = 0.0
        self._recovery_origin = None
        self._recovery_route = None
        self._mission_trigger_pending = False
        self._navigation_stop_requests = []
        self._manager_busy = None
        self._manager_status_t = 0.0
        self._manager_status_wall_t = 0.0
        self._manager_status_run_id = None
        self._mission_queue = []
        self._mission_task_index = 0
        self._task_placed = 0
        self._mission_target_total = 0
        self._empty_scans = 0
        self._cancel_requested = False
        self._mission_epoch = 0
        self._tick_context = threading.local()
        self._transition_lock = threading.RLock()
        self._tick_inflight = False
        self._pending_service_futures = []
        self._run_id = None
        self._supervisor_session_id = uuid.uuid4().hex
        self._revision = 0
        self._resume_revision = 0
        self._active_since = None
        self._elapsed_active_s = 0.0
        self._stop_confirmed = False
        self._discard_requested = False
        self._discard_future = None
        self._stop_request_serial = 0
        self._discard_stop_serial = None
        self._resume_inflight = False
        self._recovery_cause = None
        self._mission_nav_run_id = None
        self._camera_last_t = 0.0
        self._odom_t = 0.0
        self._odom_linear_speed = None
        self._odom_angular_speed = None
        self._nav_status_t = 0.0
        self._nav_goal_active = None
        self._mission_nav_status = None
        self._mission_nav_status_t = 0.0
        self._mission_nav_attempt_t = 0.0
        self._mission_nav_expected = None
        self._mission_nav_consumed_t = 0.0
        self._resume_free_nav = False
        self._terminal_after_home = DONE
        self._terminal_reason = ''
        # WAITING_SLOT: lan cuoi tham do /shelf/occupancy, va ly do khong co o
        self._last_slot_poll = 0.0
        self._slot_wait_why = ''
        self._slot_operator_notified = False

        # ---------------- TF ----------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---------------- ROS I/O ----------------
        self.pub_status = self.create_publisher(String, f'/{NODE_NAME}/status', 10)
        gate_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_aruco_gate = self.create_publisher(Bool, '/aruco_b1/enable', gate_qos)
        self._publish_aruco_gate()  # Fail closed on launch/restart.
        route_qos = QoSProfile(depth=1)
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        self._route_path_sub = self.create_subscription(
            String, '/navigation/route_yaml_path', self._on_route_path, route_qos)
        self.create_subscription(
            String, f'/{self.p["pallet_mission_node"]}/status', self._on_pm_status, 10)
        self.create_subscription(String, '/pallet_retrieval/status',
                                 self._on_retrieval_gate_status, 10)
        self.create_subscription(String, '/pallet/target_status',
                                 self._on_detector_status, 10)
        self.create_subscription(Image, self.p['camera_topic'], self._on_camera,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['odom_topic'], self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            GoalStatusArray, self.p['nav_status_topic'], self._on_nav_status, 10)
        # mission_manager reports the actual NavigateToPose result here. The
        # action status array only says whether a goal is active, so it cannot
        # explain ABORTED or give the reason needed by web/recovery.
        self.create_subscription(String, '/mission/status', self._on_mission_nav_status, 10)

        # W4 (loi 2 ESP serial -> /shelf/occupancy): 4 bit o ke, 1 = co hang;
        # [-1]*4 = mat ket noi. Doc truc tiep tu esp32_bridge. Anh xa
        # bit -> o (A1/A2/B1/B2) lay tu config/shelf.yaml (shelf_model).
        self._shelf_bits = None          # [0/1,0/1,0/1,0/1] hoac None (stale)
        self._shelf_input = None         # raw Int8MultiArray values for diagnostics
        self._shelf_last_t = 0.0
        self.create_subscription(
            Int8MultiArray, self.p['shelf_topic'], self._on_shelf, 10)

        # Mo hinh ke (doc shelf.yaml 1 lan; anchor + priority + anh xa cam bien)
        self._shelf_model = self._load_shelf_model()

        # Services (khong GUI: GUI chinh la nut goi 2 service nay)
        self.create_service(
            Trigger, '/mission/start', self._srv_start,
            callback_group=self._control_group)
        self.create_service(
            Trigger, '/mission/stop', self._srv_stop,
            callback_group=self._control_group)
        self.create_service(Trigger, '/mission/cancel', self._srv_cancel, callback_group=self._control_group)
        self.create_service(
            Trigger, '/mission/reset', self._srv_reset,
            callback_group=self._control_group)
        self.create_service(
            Trigger, '/mission/resume', self._srv_resume,
            callback_group=self._control_group)

        self._cancel_cli = self.create_client(CancelGoal,
                                              '/navigate_to_pose/_action/cancel_goal',
                                              callback_group=self._client_group)

        # GIỮ reference timer trên self: executor của rclpy giữ callback bằng
        # weak reference -> không lưu thì timer bị GC ngay sau __init__ và
        # node đứng im IDLE (khong publish status, khong tick duoc).
        self._tick_timer = self.create_timer(0.1, self._tick,
                                             callback_group=self._tick_group)
        self._status_timer = self.create_timer(0.5, self._publish_status,
                                               callback_group=self._control_group)

        # O da chon cho drop (id, frame_tf) tu chu trinh goc - null khi
        # chua chu trinh nao chon duoc o (GUI hien o dang chon)
        self._drop_slot_chosen = None
        self._reserved_drop_slots = set()

        self.get_logger().info(
            f'Mission Supervisor ready: pick={self.p["pick_mission"]} '
            f'(dest {self.p["pick_dest"]}), drop={self.p["drop_mission"]} '
            f'run_mode={self.p["run_mode"]}')

    # ==================================================================
    # Helpers
    # ==================================================================
    @_state_locked
    def _set_state(self, state: str, msg: str = ''):
        if self._stale_step():
            return
        previous = self._state
        now = time.monotonic()
        if state in ('STOPPING', 'RECOVERY', IDLE, DONE, FAILED, STOPPED):
            if self._active_since is not None:
                self._elapsed_active_s += max(0., now - self._active_since)
                self._active_since = None
        elif self._active_since is None:
            self._active_since = now
        self._revision += 1
        if state == 'RECOVERY' and previous not in ('RECOVERY', 'STOPPING'):
            self._recovery_cause = 'error'
            self._recovery_origin = self._state
            self._recovery_pm_session = self._pm_session_id
            self._recovery_route = self._route_path_loaded
            self._request_recovery_stop()
        self._state = state
        self._aruco_route_gate.transition(previous, state, self._pm_cargo)
        self._publish_aruco_gate()  # Push OFF immediately for STOP/RECOVERY/DROP.
        self._msg = msg
        self._t_state = time.monotonic()
        self._shelf_bad_t0 = None
        self._stuck_t0 = None
        self._start_refusals = 0
        self.get_logger().info(f'[{state}] {msg}')

    def _publish_aruco_gate(self):
        # Repeated at supervisor status heartbeat (0.5s) so ArUco detects a
        # vanished supervisor via its lease timeout, not a stale latched True.
        publisher = getattr(self, 'pub_aruco_gate', None)
        if publisher is not None:
            now = time.monotonic()
            status = self._retrieval_gate_status
            fresh = self._retrieval_gate_t > 0.0 and now - self._retrieval_gate_t <= 2.0
            self._retrieval_aruco_gate.retrieval_transition(
                status, nav_goal_active=status.get('nav_goal_active') if isinstance(status, dict) else False,
                fresh=fresh)
            placement_fresh = self._pm_status_t > 0.0 and now - self._pm_status_t <= 2.0
            enabled = ArucoRouteGate.combine(
                self._state, self._aruco_route_gate.enabled, placement_fresh,
                status, self._retrieval_aruco_gate.enabled, fresh)
            self._aruco_gate_effective = bool(enabled)
            publisher.publish(Bool(data=bool(enabled)))

    @_state_locked
    def _on_retrieval_gate_status(self, msg):
        try:
            status = json.loads(msg.data)
        except (TypeError, ValueError):
            status = None
        self._retrieval_gate_status = status if isinstance(status, dict) else None
        self._retrieval_gate_t = time.monotonic() if self._retrieval_gate_status else 0.0
        self._publish_aruco_gate()  # In particular, d1 -> slot sends OFF immediately.

    def _handling_status(self):
        """Fine-grained safety activity while keeping the legacy mission state."""
        if (self._pm_status_t > 0.0 and time.monotonic() - self._pm_status_t <= 1.5
                and self._pm_handling_status in ('AUTO_RECOVERING', 'WAITING_FOR_SENSOR')):
            return self._pm_handling_status
        if (self._pm_handling_status == 'NEED_OPERATOR'
                and (self._stop_confirmed is True
                     or (self._pm_recovery or {}).get('stop_confirmed') is True)):
            return 'NEED_OPERATOR'
        if self._state == FAILED:
            return FAILED
        return self._state

    def _base_pose(self):
        """Pose cua xe trong map: (x, y, yaw) hoac None."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self.p['map_frame'], self.p['base_frame'], Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quat(q)

    def _dest_pose(self):
        """Pose cua node cuoi cua pick_mission trong map (tu route graph)."""
        try:
            from amr_navigation.route_graph_loader import RouteGraphLoader
            loader = self._route_loader
            if loader is None:
                return None
            name = loader.resolve(self._active_dest)
            pose = loader.get_node_pose(name)
            if pose is None:
                return None
            return (pose.pose.position.x, pose.pose.position.y,
                    yaw_from_quat(pose.pose.orientation))
        except Exception as e:
            self.get_logger().warn(f'Khong doc duoc route graph ({self._active_dest}): {e}')
            return None

    def _on_route_path(self, msg: String):
        """Use only the graph announced by the active Nav2 launch."""
        self._route_path_last_received = time.monotonic()
        self._route_path = msg.data
        if msg.data == self._route_path_loaded:
            self._route_path_error = ''
            return
        try:
            from amr_navigation.route_graph_loader import RouteGraphLoader
            loader = RouteGraphLoader(yaml_path=msg.data)
            loader.get_all_node_names()  # validate/load before replacing current graph
            self._route_loader = loader
            self._route_path_loaded = msg.data
            self._route_path_error = ''
            self.get_logger().info(f'Using active Nav2 route graph: {msg.data}')
        except Exception as e:
            self._route_loader = None
            self._route_path_error = str(e)
            self.get_logger().error(f'Failed to load active Nav2 route graph {msg.data}: {e}')

    def _stale_step(self):
        epoch = getattr(getattr(self, '_tick_context', None), 'epoch', None)
        return epoch is not None and epoch != getattr(self, '_mission_epoch', 0)

    @_state_locked
    def _late_service_result(self, epoch):
        if epoch != self._mission_epoch:
            self.get_logger().warning(f'Late service result from Auto epoch={epoch}; current={self._mission_epoch}')
            if self._cancel_requested and self._state in ('STOPPING', 'RECOVERY'):
                self._request_recovery_stop()
            return
        self._stop_after_late_request()

    def _call_service_result(self, cli, req, timeout: float = 5.0, on_late=None,
                             retain_on_timeout=False):
        epoch = self._mission_epoch
        if self._stale_step() or not cli.wait_for_service(timeout_sec=min(max(timeout, 0.0), 0.2)):
            return None
        with self._transition_lock:
            if epoch != self._mission_epoch or self._stale_step():
                return None
            fut = cli.call_async(req)
            self._pending_service_futures = [f for f in self._pending_service_futures if not f.done()]
            self._pending_service_futures.append(fut)
        t_end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.02)
        if not fut.done():
            if retain_on_timeout:
                self._uncertain_child_request = fut
                self._uncertain_child_session = self._pm_session_id
                self.get_logger().warn(
                    f'Child service {cli.srv_name} timed out; keeping operation pending until response/status')
                return None
            def late(_):
                with self._transition_lock:
                    if epoch != self._mission_epoch:
                        self._late_service_result(epoch)
                    elif on_late:
                        on_late(_)
                    else:
                        self._stop_after_late_request()
            fut.add_done_callback(late)
            self._late_service_result(epoch)
            return None
        if epoch != self._mission_epoch:
            self._late_service_result(epoch)
            return None
        try:
            return fut.result()
        except Exception as exc:
            self.get_logger().error(f'Auto service result unconfirmed epoch={epoch}: {exc}')
            if retain_on_timeout:
                self._uncertain_child_request = fut
                self._uncertain_child_session = self._pm_session_id
                return None
            self._late_service_result(epoch)
            return None

    def _pending_child_result(self):
        future = self._uncertain_child_request
        if future is None:
            return True, None
        if (self._pm_session_id == self._uncertain_child_session
                and self._await_operation_after is not None
                and self._pm_operation_id > self._await_operation_after
                and self._pm_state not in (None, IDLE, 'READY')):
            # A fresh status from the new operation confirms acceptance even
            # when the original Trigger response was lost.
            self._uncertain_child_request = None
            self._uncertain_child_session = None
            return True, None
        if not future.done():
            return False, None
        self._uncertain_child_request = None
        self._uncertain_child_session = None
        try:
            return True, future.result()
        except Exception as exc:
            self.get_logger().warn(f'Child service response uncertain: {exc}')
            return False, None

    def _service_client(self, srv_type, service_name):
        key = (srv_type, service_name)
        client = self._service_clients.get(key)
        if client is None:
            client = self.create_client(srv_type, service_name,
                                        callback_group=self._client_group)
            self._service_clients[key] = client
        return client

    def _call_service(self, cli, req) -> bool:
        return self._call_service_result(cli, req) is not None

    def _request_owned_navigation_stop(self):
        client = self._service_client(Trigger, '/mission_manager/stop')
        if client.service_is_ready():
            self._navigation_stop_requests.append(client.call_async(Trigger.Request()))
        elif self._cancel_cli.service_is_ready():
            self._navigation_stop_requests.append(self._cancel_cli.call_async(CancelGoal.Request()))

    def _trigger_mission(self, mission: str, force_free: bool = False) -> bool:
        """Call the acknowledged mission trigger; success means Nav2 accepted its goal."""
        if self._cancel_requested or any(not f.done() for f in self._navigation_stop_requests):
            return False
        self._navigation_stop_requests = []
        self._mission_nav_expected = mission
        self._mission_nav_run_id = None
        self._mission_nav_status = None
        self._mission_nav_status_t = 0.0
        self._mission_nav_attempt_t = time.time()
        self._mission_nav_consumed_t = 0.0
        suffix = '_free_ack' if force_free else '_ack'
        svc = f'/{self.p["mission_manager_node"]}/trigger_{mission}{suffix}'
        cli = self._service_client(Trigger, svc)
        req = Trigger.Request()
        self._mission_trigger_pending = True
        try:
            result = self._call_service_result(cli, req, timeout=12.0)
        finally:
            self._mission_trigger_pending = False
        if self._cancel_requested or result is None or not getattr(result, 'success', True):
            return False
        self.get_logger().info(f'Da goi {svc}')
        return True

    def _on_mission_nav_status(self, msg: String):
        try:
            data = json.loads(msg.data)
            mission = str(data.get('mission') or '')
            state = str(data.get('state') or '').upper()
            stamp = float(data.get('timestamp') or time.time())
            # Mission Manager may publish terminal/idle before an older busy
            # heartbeat is delivered. Never let that delayed sample re-lock Start.
            if stamp < self._manager_status_wall_t:
                return
            self._manager_status_wall_t = stamp
            self._manager_busy = data.get('busy')
            self._manager_status_run_id = data.get('run_id')
            self._manager_status_t = time.monotonic()
            if mission != self._mission_nav_expected or stamp < self._mission_nav_attempt_t:
                return
            run_id = data.get('run_id')
            if run_id is not None:
                if self._mission_nav_run_id is None:
                    if state not in ('PENDING', 'EXECUTING'):
                        return
                    self._mission_nav_run_id = run_id
                elif run_id != self._mission_nav_run_id:
                    return
            self._mission_nav_status = {
                'run_id': run_id,
                'busy': data.get('busy'),
                'mission': mission, 'state': state,
                'reason': str(data.get('reason') or ''),
                'nav_mode': data.get('nav_mode'),
            }
            self._mission_nav_status_t = stamp
        except (TypeError, ValueError, json.JSONDecodeError):
            return

    # ==================================================================
    # Services
    # ==================================================================
    def _srv_start(self, req, res):
        with self._transition_lock:
            return self._start_operation(req, res)

    def _start_operation(self, req, res):
        # Keep unresolved service outcomes as hard blockers. A stale manager
        # busy flag, however, can be superseded only by same-run terminal idle.
        unresolved = sum(not f.done() for f in self._pending_service_futures)
        terminal_idle = self._confirmed_terminal_idle()
        blockers = mission_start_blockers(
            stopping=(self._state == 'STOPPING'),
            discarding=getattr(self, '_discard_requested', False),
            tick_inflight=getattr(self, '_tick_inflight', False),
            unresolved_services=unresolved, manager_busy=self._manager_busy,
            terminal_idle_confirmed=terminal_idle)
        if blockers:
            detail = '; '.join(blockers)
            self.get_logger().warning(
                f'[mission/start] rejected: {detail}; state={self._state}, '
                f'busy={self._manager_busy}, manager_run={self._manager_status_run_id}, '
                f'run_id={self._run_id}, nav={self._mission_nav_status}, '
                f'terminal_idle={terminal_idle}')
            res.success, res.message = False, f'Không thể bắt đầu mission: {detail}'
            return res

        route_age = (time.monotonic() - self._route_path_last_received
                     if self._route_path_last_received > 0.0 else float('inf'))
        if self._route_loader is None or route_age > 6.0:
            res.success = False
            detail = self._route_path_error or 'route graph chưa được nạp'
            res.message = f'Từ chối start: {detail}; route status age={route_age:.1f}s'
            return res
        if (self._pm_recovery or {}).get('checkpoint'):
            res.success, res.message = False, 'Pallet operation checkpoint retained; Resume or explicitly reset after inspection'
            return res
        requested_mode = self.get_parameter('run_mode').value
        if requested_mode not in ('single', 'auto'):
            res.success = False
            res.message = "run_mode phai la 'single' hoac 'auto'"
            return res
        if self._state not in (IDLE, DONE, FAILED, STOPPED):
            res.message = f'dang ban ({self._state}), dung /mission/stop truoc'
            res.success = False
            return res
        if (self._nav_status_t > 0.0
                and time.monotonic() - self._nav_status_t <= 2.0
                and self._nav_goal_active is True):
            res.success = False
            res.message = 'Nav2 dang co goal khac; hay ket thuc/huy goal truoc khi bat dau chu trinh'
            return res

        # Nạp lại các tham số mission (web/GUI/CLI có thể đã set qua
        # ros2 param set mission_supervisor ... TRƯỚC khi gọi /mission/start)
        # - override hot, không cần restart.
        for k in ('pick_mission', 'drop_mission', 'pick_dest', 'run_mode',
                  'target_cargo_class', 'target_quantity', 'target_drop_slot', 'target_queue_json',
                  'pm_final_window_s', 'pm_min_samples', 'pm_max_pos_std_m',
                  'pm_max_yaw_std_deg', 'pm_max_sample_age_s',
                  'pm_require_stable'):
            try:
                self.p[k] = self.get_parameter(k).value
            except Exception:
                pass

        target_class = str(self.p.get('target_cargo_class') or '').strip().lower()
        target_quantity = self.p.get('target_quantity', 1)
        target_drop_slot = str(self.p.get('target_drop_slot') or '').strip().upper()
        if requested_mode == 'auto':
            target_class, target_quantity, target_drop_slot = '', 1, ''
        if target_class not in ('', 'medium', 'tall'):
            res.success = False
            res.message = 'target_cargo_class must be empty, medium, or tall'
            return res
        if (isinstance(target_quantity, bool) or not isinstance(target_quantity, int)
                or not 1 <= target_quantity <= 10):
            res.success = False
            res.message = 'target_quantity must be an integer from 1 to 10'
            return res
        if target_drop_slot and target_drop_slot not in ('A1', 'A2', 'B1', 'B2'):
            res.success = False
            res.message = 'target_drop_slot must be empty, A1, A2, B1, or B2'
            return res
        if target_drop_slot and target_quantity != 1 and not self.p.get('target_queue_json'):
            res.success = False
            res.message = 'A fixed shelf slot only accepts one pallet per mission'
            return res
        try:
            self._mission_queue = normalize_mission_queue(
                self.p.get('target_queue_json', ''),
                {'cargo_class': target_class or 'any', 'quantity': target_quantity,
                 'drop_slot': target_drop_slot or 'auto'}) if requested_mode == 'single' else []
        except ValueError as exc:
            res.success, res.message = False, str(exc)
            self.get_logger().warning(f'[mission/start] rejected: invalid task queue: {exc}')
            return res
        self._mission_task_index = 0
        self._task_placed = 0
        self._mission_target_total = sum(t['quantity'] for t in self._mission_queue)
        if self._mission_queue:
            first = self._mission_queue[0]
            target_class, target_quantity, target_drop_slot = (
                '' if first['cargo_class'] == 'any' else first['cargo_class'],
                first['quantity'], '' if first['drop_slot'] == 'auto' else first['drop_slot'])
        if target_drop_slot and self._shelf_model is not None:
            slot_cfg = self._shelf_model._slot_by_id.get(target_drop_slot)
            slot_class = ('medium' if slot_cfg and slot_cfg.product_type == 'A'
                          else 'tall' if slot_cfg and slot_cfg.product_type == 'B' else None)
            if slot_class is None or (target_class and target_class != slot_class):
                res.success = False
                res.message = f'target_drop_slot {target_drop_slot} is incompatible with {target_class or "any"}'
                return res
            target_class = slot_class
        self.p['target_cargo_class'] = target_class
        self.p['target_quantity'] = target_quantity
        self.p['target_drop_slot'] = target_drop_slot
        if target_class and (target_drop_slot or self._shelf_ok() and self._shelf_bits is not None):
            cargo_type = 'A' if target_class == 'medium' else 'B'
            slot, _, why = self._select_drop_slot_for(cargo_type)
            if slot is None:
                res.success = False
                res.message = f'No compatible shelf slot for {target_class}: {why}'
                return res

        # The queue belongs to this accepted run. Consume the parameter so a
        # later GUI/CLI start that does not provide a queue cannot replay it.
        if self.p.get('target_queue_json'):
            cleared = self.set_parameters([Parameter('target_queue_json', value='')])
            if not cleared or not cleared[0].successful:
                res.success = False
                res.message = 'Could not consume target_queue_json; refusing to start with reusable queue state'
                self.get_logger().error('[mission/start] rejected: failed to clear consumed target_queue_json')
                return res
            self.p['target_queue_json'] = ''

        # Canh bao nhat quan: `pick_dest` va `pick_mission` phai tro cung mot
        # diem. Khong tu sua (nguoi van hanh can thay), chi bao ro.
        if self.p['pick_mission'] == 'home_to_p1' and self.p['pick_dest'] != 'P_1':
            self.get_logger().warn(
                f'[mission] pick_mission={self.p["pick_mission"]} nhung pick_dest='
                f'{self.p["pick_dest"]} -> khong khop. Sua pick_dest cho khop.')

        self._mission_epoch += 1
        self._run_id = uuid.uuid4().hex
        self._resume_revision = 0
        self._elapsed_active_s = 0.0
        self._active_since = None
        self._recovery_cause = None
        self._stop_confirmed = False
        self._pick_phase = S_SAMPLING
        self._detect_on_called = False
        self._go_called = False
        self._retries = 0
        self._approach_retries = 0
        self._drop_retries = 0
        self._detector_restarts = 0
        self._near_t0 = None
        self._triggered = False
        self._pm_state = None
        self._pm_sampling_applied = False
        self._detector_filter_applied = False
        self._drop_slot_chosen = None
        # Hang doi diem lay: P_1, va them P_2 CHI khi graph co edge noi P_2.
        self._pick_queue = self._build_pick_queue()
        self._pick_idx = 0
        self._active_mission = self._mission_for_pick_dest(self._current_pick_dest())
        self._active_dest = self._current_pick_dest()
        self._cycle = 0
        self._terminal_after_home = DONE
        self._terminal_reason = ''
        self._mission_nav_expected = None
        self._mission_nav_status = None
        self._mission_nav_status_t = 0.0
        self._resume_free_nav = False
        self._empty_scans = 0
        self._resume_state = None
        self._cancel_requested = False
        self._last_slot_poll = 0.0
        self._slot_wait_why = ''
        queue_txt = ' -> '.join(self._pick_queue)
        self._set_state(GOTO_PICK, f'Mode {self.p["run_mode"]}: {queue_txt} '
                                   f'(bang {self._active_mission})')
        res.success = True
        res.message = 'start'
        return res

    def _confirmed_terminal_idle(self):
        """Whether same Mission Manager run ended and Nav2 freshly reports idle."""
        return (self._state in (DONE, FAILED, STOPPED)
                and self._mission_nav_status is not None
                and self._mission_nav_run_id is not None
                and self._mission_nav_status.get('run_id') == self._mission_nav_run_id
                and self._manager_status_run_id == self._mission_nav_run_id
                and self._mission_nav_status.get('state') in ('SUCCEEDED', 'ABORTED', 'CANCELED')
                and self._mission_nav_status.get('busy') is False
                and self._manager_status_t > 0.0
                and time.monotonic() - self._manager_status_t <= 5.0
                and self._nav_goal_active is False
                and self._nav_status_t > 0.0
                and time.monotonic() - self._nav_status_t <= 2.0)

    def _stop_after_late_request(self):
        if self._cancel_requested and self._state in ('STOPPING', 'RECOVERY'):
            self._request_recovery_stop()
            return
        if self._state != 'RECOVERY':
            self._resume_state = self._state if self._state not in (IDLE, DONE, FAILED, STOPPED) else None
            self._set_state('RECOVERY', 'Service outcome uncertain; STOP requested before any retry')
        else:
            self._request_recovery_stop()

    @_state_locked
    def _request_recovery_stop(self):
        """Request STOP; completion is established from telemetry before Resume."""
        self._cancel_requested = True
        self._stop_request_serial += 1
        self._stop_confirmed = False
        self._recovery_started = time.monotonic()
        self._recovery_cancel_future = None
        self._recovery_stop_future = None
        self._manager_stop_request = None
        self._recovery_still_since = None
        try:
            manager = self._service_client(Trigger, '/mission_manager/stop')
            if manager.service_is_ready():
                self._manager_stop_request = manager.call_async(Trigger.Request())
                self._pending_service_futures.append(self._manager_stop_request)
            if self._cancel_cli.service_is_ready():
                self._recovery_cancel_future = self._cancel_cli.call_async(CancelGoal.Request())
                self._pending_service_futures.append(self._recovery_cancel_future)
            cli = self._service_client(Trigger, f'/{self.p["pallet_mission_node"]}/stop')
            if cli.service_is_ready():
                self._recovery_stop_future = cli.call_async(Trigger.Request())
                self._pending_service_futures.append(self._recovery_stop_future)
        except Exception as exc:
            self.get_logger().warn(f'Recovery stop request failed: {exc}')

    def _srv_stop(self, req, res):
        with self._transition_lock:
            if self._state in (IDLE, DONE, FAILED, STOPPED):
                res.success, res.message = True, 'No active supervisor mission'
                return res
            if self._state == 'STOPPING':
                if self._manager_stop_request is None or self._recovery_stop_future is None:
                    self._request_recovery_stop()
                res.success, res.message = True, 'Stopping mission'
                return res
            if self._state == 'RECOVERY' and self._stop_confirmed and not self._resume_inflight:
                res.success, res.message = True, 'Mission paused'
                return res
            self._begin_user_stop(discard=False)
        res.success, res.message = True, 'Stopping mission'
        return res

    def _srv_cancel(self, req, res):
        with self._transition_lock:
            if self._state == IDLE:
                res.success, res.message = True, 'Mission canceled'
                return res
            if self._discard_requested:
                if self._discard_future is not None and self._discard_future.done():
                    try:
                        if not self._discard_future.result().success:
                            self._discard_future = None
                    except Exception as exc:
                        self.get_logger().warning(f'Discard response unconfirmed: {exc}')
                        self._discard_future = None
                if self._manager_stop_request is None or self._recovery_stop_future is None:
                    self._request_recovery_stop()
                res.success, res.message = True, 'Canceling mission'
                return res
            self._begin_user_stop(discard=True)
        res.success, res.message = True, 'Canceling mission'
        return res

    def _begin_user_stop(self, discard):
        previous = self._state
        if previous not in ('STOPPING', 'RECOVERY'):
            self._resume_state = previous if previous not in (IDLE, DONE, FAILED, STOPPED) else None
            self._recovery_origin = self._resume_state
            self._recovery_pm_session = self._pm_session_id
            self._recovery_route = self._route_path_loaded
        self._mission_epoch += 1
        self._cancel_requested = True  # fence timer/results BEFORE sending any child cancel
        self._discard_requested = discard
        self._discard_future = None
        self._stop_confirmed = False
        if previous != 'RECOVERY' or self._recovery_cause != 'error':
            self._recovery_cause = 'user'
        self._set_state('STOPPING', 'Operator requested full cancellation' if discard else 'Operator requested pause')
        self._request_recovery_stop()

    def _stop_blocked(self):
        now = time.monotonic()
        if (getattr(self, '_tick_inflight', False) or getattr(self, '_mission_trigger_pending', False)
                or getattr(self, '_resume_inflight', False)):
            return 'Waiting for the previous mission step'
        if any(not f.done() for f in getattr(self, '_pending_service_futures', [])):
            return 'Waiting for outstanding mission service requests'
        for future in (getattr(self, '_manager_stop_request', None), getattr(self, '_recovery_stop_future', None)):
            if future is None or not future.done():
                return 'Waiting for owner STOP acknowledgement'
            try:
                if not future.result().success:
                    return 'Owner STOP rejected'
            except Exception as exc:
                return f'Owner STOP failed: {exc}'
        if self._manager_busy is not False or self._manager_status_t < self._recovery_started:
            return 'Waiting for pending/active manager goal to end'
        if any(not f.done() for f in self._navigation_stop_requests):
            return 'Waiting for earlier navigation stop'
        if not navigation_stopped(self._nav_goal_active, self._nav_status_t, self._recovery_started,
                                  getattr(self, '_recovery_cancel_future', None)):
            return 'Waiting for terminal navigation'
        if (not self._robot_stopped(now) or self._recovery_still_since is None or
                now - self._recovery_still_since < self.p['arrive_settle_s']):
            return 'Waiting for stationary odometry'
        if (now - self._pm_status_t > 1.5 or self._pm_status_t < self._recovery_started or
                (self._pm_recovery or {}).get('stop_confirmed') is not True):
            return 'Waiting for pallet/lift stop confirmation'
        return ''

    def _advance_stop(self):
        with self._transition_lock:
            reason = self._stop_blocked()
            if reason:
                if reason != getattr(self, '_stop_reason', ''):
                    self.get_logger().info(f'Auto stop run={self._run_id}: {reason}')
                self._stop_reason = reason
                return
            self._stop_confirmed = True
            if not self._discard_requested:
                self._set_state('RECOVERY', 'User-requested recovery; mission checkpoint retained')
                return
            if self._discard_future is None:
                client = self._service_client(Trigger, f'/{self.p["pallet_mission_node"]}/discard_recovery')
                if not client.service_is_ready():
                    self._stop_reason = 'Pallet discard service unavailable'
                    return
                self._discard_future = client.call_async(Trigger.Request())
                self._discard_stop_serial = self._stop_request_serial
                self._pending_service_futures.append(self._discard_future)
                return
            if not self._discard_future.done():
                return
            if self._discard_stop_serial != self._stop_request_serial:
                self._discard_future = None
                return
            try:
                response = self._discard_future.result()
            except Exception as exc:
                self._stop_reason = f'Pallet discard failed: {exc}'
                self.get_logger().error(self._stop_reason)
                return
            if not response.success:
                self._stop_reason = response.message
                return
            self._clear_auto_cycle()
            self._set_state(IDLE, 'Auto mission canceled; physical observations preserved')

    def _clear_auto_cycle(self):
        self._mission_epoch += 1
        self._aruco_route_gate.reset()
        self._publish_aruco_gate()
        self._cancel_requested = self._discard_requested = self._resume_inflight = False
        self._resume_state = self._recovery_origin = None
        self._active_mission = self._active_dest = None
        self._mission_nav_expected = self._mission_nav_status = self._mission_nav_run_id = None
        self._mission_nav_status_t = self._mission_nav_consumed_t = 0.0
        self._pick_queue = []
        self._pick_idx = self._cycle = 0
        self._triggered = self._go_called = self._detect_on_called = False
        self._near_t0 = self._stable_since = self._stable_anchor = None
        self._await_resume_checkpoint = self._await_operation_after = None
        self._drop_slot_chosen = None
        self._active_since = None
        self._elapsed_active_s = 0.0
        self._resume_free_nav = False
        self._stop_reason = ''
        self._recovery_cause = None
        self._retry_wait_until = self._last_slot_poll = 0.0
        self._slot_wait_why = ''
        self._slot_operator_notified = False
        self._pick_phase = S_SAMPLING
        self._pm_sampling_applied = self._detector_filter_applied = False
        self._retries = self._approach_retries = self._drop_retries = 0
        self._detector_restarts = self._start_refusals = self._empty_scans = 0
        self._terminal_after_home, self._terminal_reason = DONE, ''

    def _resume_blocked(self):
        if self._state != 'RECOVERY' or not self._resume_state:
            return 'No supervisor recovery checkpoint'
        if getattr(self, '_discard_requested', False):
            return 'Mission cancellation is pending'
        if getattr(self, '_tick_inflight', False):
            return 'Previous mission step has not ended'
        now = time.monotonic()
        future = getattr(self, '_recovery_stop_future', None)
        if future is None or not future.done():
            return 'Waiting for pallet STOP response; retry Stop if its service was unavailable'
        try:
            if not future.result().success:
                return 'Pallet STOP was not acknowledged; retry Stop'
        except Exception:
            return 'Pallet STOP failed; retry Stop'
        if any(not f.done() for f in self._navigation_stop_requests):
            return 'Waiting for earlier navigation STOP requests'
        if getattr(self, '_recovery_cancel_future', None) is not None and not self._recovery_cancel_future.done():
            return 'Waiting for navigation cancel response'
        if getattr(self, '_manager_stop_request', None) is not None and not self._manager_stop_request.done():
            return 'Waiting for mission_manager STOP acknowledgement'
        if self._manager_busy is not False or self._manager_status_t < self._recovery_started:
            return 'Waiting for mission_manager to release pending navigation'
        if getattr(self, '_mission_trigger_pending', False):
            return 'Waiting for the previous mission trigger to finish'
        if not navigation_stopped(self._nav_goal_active, self._nav_status_t, self._recovery_started,
                                  getattr(self, '_recovery_cancel_future', None)):
            return 'Waiting for Nav2 idle status'
        if (not self._robot_stopped(now) or self._recovery_still_since is None
                or now - self._recovery_still_since < float(self.p['arrive_settle_s'])):
            return 'Waiting for fresh odometry and a stationary robot'
        if (now - self._pm_status_t > 1.5 or self._pm_status_t < self._recovery_started
                or not isinstance(self._pm_cargo, bool)):
            return 'Waiting for fresh pallet/cargo status after Stop'
        if self._pm_session_id != self._recovery_pm_session:
            return 'Pallet node restarted during recovery; inspect and reset mission'
        if (self._pm_recovery or {}).get('stop_pending'):
            return 'Waiting for all child STOP requests to finish'
        if self._route_loader is None or now - self._route_path_last_received > 6.0:
            return 'Route graph missing or stale'
        if self._route_path_loaded != self._recovery_route:
            return 'Route changed since recovery checkpoint'
        if 'stop_confirmed' in (self._pm_recovery or {}) and self._pm_recovery['stop_confirmed'] is not True:
            return 'Pallet/lift stop is not confirmed'
        origin = self._recovery_origin
        if origin == GOTO_PICK and self._pm_cargo is not False:
            return 'Cannot resume a pick navigation while carrying cargo'
        if origin == GOTO_D and self._pm_cargo is not True:
            return 'Cannot resume shelf navigation without confirmed cargo'
        cp = (self._pm_recovery or {}).get('checkpoint')
        if origin in (PICKING, DROPPING) and cp:
            if not self._pm_recovery.get('available'):
                return self._pm_recovery.get('reason') or 'Pallet operation cannot resume'
        elif origin == PICKING and self._pm_state not in ('IDLE', 'READY', 'CARRYING'):
            return 'No recoverable PICK operation; inspect robot'
        elif origin == DROPPING and self._pm_state not in ('CARRYING', DONE):
            return 'No recoverable DROP operation; inspect robot'
        if origin in (GOTO_PICK, PICKING) and not self._pm_cargo and not self._camera_ok():
            return 'Camera is not ready'
        return ''

    def _srv_resume(self, req, res):
        with self._transition_lock:
            if self._resume_inflight:
                res.success, res.message = False, 'Resume is already pending'
                return res
            self._resume_inflight = True
        try:
            return self._resume_operation(req, res)
        finally:
            self._resume_inflight = False

    def _resume_operation(self, req, res):
        resume_epoch = self._mission_epoch
        reason = self._resume_blocked()
        if reason:
            self.get_logger().warning(f'Auto Resume rejected run={self._run_id}: {reason}')
            res.success, res.message = False, reason
            return res
        recovery_started = self._recovery_started
        origin = self._recovery_origin
        state = self._resume_state
        cp = (self._pm_recovery or {}).get('checkpoint')
        if origin in (PICKING, DROPPING):
            state = origin  # finish the interrupted child before considering cargo/navigation
            if cp:
                cli = self._service_client(Trigger, f'/{self.p["pallet_mission_node"]}/resume')
                result = self._call_service_result(cli, Trigger.Request(), timeout=3.0,
                                                   on_late=lambda _: self._request_recovery_stop())
                if self._recovery_started != recovery_started:
                    self._request_recovery_stop()
                    res.success, res.message = False, 'Resume interrupted by a newer Stop'
                    return res
                if result is None or not result.success:
                    if result is None:
                        self._request_recovery_stop()
                    res.success, res.message = False, result.message if result else 'Pallet Resume unconfirmed; STOP requested'
                    return res
                self._await_resume_checkpoint = cp['id']
                self._go_called = cp['state'] not in ('READY', 'DETECTING')
                if origin == DROPPING and cp['state'] == 'CARRYING':
                    self._go_called = False
                self._detect_on_called = cp['state'] == 'DETECTING'
                self._pick_phase = S_SAMPLING if cp['state'] == 'DETECTING' else S_APPROACHING
                self._pm_state = None
            elif origin == PICKING:
                self._go_called = self._pm_state == 'CARRYING'
                self._detect_on_called = False
                self._pick_phase = S_APPROACHING if self._pm_state == 'READY' else S_SAMPLING
            else:
                self._go_called = self._pm_state == DONE
        else:
            self._triggered = False
        with self._transition_lock:
            if resume_epoch != self._mission_epoch:
                res.success, res.message = False, 'Resume interrupted by a newer stop'
                return res
            self._resume_free_nav = bool(self._mission_nav_status and self._mission_nav_status.get('state') == 'ABORTED')
            self._mission_epoch += 1
            self._resume_revision += 1
            self._stop_confirmed = False
            self._cancel_requested = False
            self._mission_nav_status = None
            self._mission_nav_expected = None
            self._mission_nav_consumed_t = 0.0
            self._near_t0 = None
            self._resume_state = None
            self._set_state(state, 'Resumed interrupted operation; completed steps and budgets retained')
            res.success, res.message = True, 'resumed'
            return res

    def _srv_reset(self, req, res):
        if self._state not in (IDLE, STOPPED, 'RECOVERY', 'DONE', 'FAILED'):
            res.success = False
            res.message = f'Cannot reset active mission state {self._state}; stop first'
            return res
        cli = self._service_client(
            Trigger, f'/{self.p["pallet_mission_node"]}/reset')
        result = self._call_service_result(cli, Trigger.Request(), timeout=1.0)
        if result is None or not result.success:
            res.success = False
            res.message = ('Pallet mission reset not confirmed' if result is None
                           else result.message or 'Pallet mission reset rejected')
            return res
        if 'cargo=true' in result.message:
            self._pm_cargo = True
        elif 'cargo=false' in result.message:
            self._pm_cargo = False
        elif 'cargo=unknown' in result.message:
            self._pm_cargo = None

        self._clear_auto_cycle()
        self._pick_phase = S_SAMPLING
        self._detect_on_called = False
        self._go_called = False
        self._retries = self._approach_retries = self._drop_retries = 0
        self._detector_restarts = self._start_refusals = self._empty_scans = 0
        self._near_t0 = self._stable_since = self._stable_anchor = None
        self._triggered = self._pm_sampling_applied = False
        self._detector_filter_applied = False
        self._pick_queue = []
        self._pick_idx = self._cycle = 0
        self._active_mission = self.p['pick_mission']
        self._active_dest = self.p['pick_dest']
        self._drop_slot_chosen = None
        self._resume_state = None
        self._cancel_requested = False
        self._retry_wait_until = 0.0
        self._last_slot_poll = 0.0
        self._slot_wait_why = ''
        self._slot_operator_notified = False
        self._set_state(IDLE, 'Mission reset by operator entering MANUAL')
        res.success, res.message = True, 'Mission reset; cargo state cleared, lift position preserved'
        return res

    # ==================================================================
    # Pallet mission sampling params (HOT, khong sua node cu)
    # ==================================================================
    def _apply_pm_sampling(self):
        """Ap bo tham so sampling rong len pallet_mission (SetParameters srv).

        Node cu co self._reload_params (hot reload) nen chi can goi 1 lan
        SetParameters; khong can restart node. Map: pm_<param> -> <param>.
        """
        pm = self.p['pallet_mission_node']
        mapping = {
            'pm_final_window_s': ('final_window_s', Parameter.Type.DOUBLE),
            'pm_min_samples': ('min_samples', Parameter.Type.INTEGER),
            'pm_max_pos_std_m': ('max_pos_std_m', Parameter.Type.DOUBLE),
            'pm_max_yaw_std_deg': ('max_yaw_std_deg', Parameter.Type.DOUBLE),
            'pm_max_sample_age_s': ('max_sample_age_s', Parameter.Type.DOUBLE),
            'pm_require_stable': ('require_stable', Parameter.Type.BOOL),
            'pm_after_pick': ('after_pick', Parameter.Type.STRING),
        }
        params = []
        for src_key, (dst_key, ptype) in mapping.items():
            v = self.get_parameter(src_key).value
            params.append(Parameter(dst_key, ptype, v).to_parameter_msg())
        cli = self._service_client(SetParameters, f'/{pm}/set_parameters')
        if not cli.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('SetParameters (pallet_mission) khong du - '
                                   'dung tham so mac dinh cua no')
            return
        req = SetParameters.Request()
        req.parameters = params
        fut = cli.call_async(req)
        t_end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.02)
        if fut.done():
            res = fut.result()
            ok = all(r.successful for r in res.results)
            self.get_logger().info(
                f'[sampling] ap len pallet_mission: '
                f'window={self.p["pm_final_window_s"]}s min={self.p["pm_min_samples"]} '
                f'std={self.p["pm_max_pos_std_m"]}m/{self.p["pm_max_yaw_std_deg"]}deg '
                f'stable={self.p["pm_require_stable"]} -> {ok}')
        else:
            self.get_logger().warn('[sampling] SetParameters timeout')

    def _apply_detector_filter(self) -> bool:
        """Pass shelf-compatible cargo classes to the detector child process.

        Keep pallet_mission untouched: update its existing detector_extra_args
        parameter through ROS, preserving any launch-time detector arguments.
        pallet_mission reloads those args when Detect ON starts the child.
        """
        pm = self.p['pallet_mission_node']
        classes = []
        if self._select_drop_slot_for('A')[0] is not None:
            classes.append('medium')
        if self._select_drop_slot_for('B')[0] is not None:
            classes.append('tall')
        requested = str(self.p.get('target_cargo_class') or '').strip().lower()
        if requested:
            classes = [requested] if requested in classes else []
        if not classes:
            self.get_logger().warn(
                '[detector] no compatible shelf class is available for requested '
                f'class {requested or "any"}')
            return False

        get_cli = self._service_client(GetParameters, f'/{pm}/get_parameters')
        if not get_cli.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('[detector] pallet_mission get_parameters unavailable')
            return False
        get_req = GetParameters.Request()
        get_req.names = ['detector_extra_args']
        get_future = get_cli.call_async(get_req)
        deadline = time.monotonic() + 3.0
        while not get_future.done() and time.monotonic() < deadline:
            if self._cancel_requested:
                return False
            time.sleep(0.01)
        if not get_future.done():
            return False
        try:
            parameter_value = get_future.result().values[0]
            # ParameterValue.type is the generated message's integer enum;
            # rclpy.Parameter.Type.STRING_ARRAY is a Python Enum wrapper.
            if parameter_value.type != Parameter.Type.STRING_ARRAY.value:
                raise TypeError(
                    'expected STRING_ARRAY, got parameter type '
                    f'{parameter_value.type}')
            # rcl_interfaces exposes string_array_value as a plain Python
            # list in current ROS 2 releases. Some generated wrappers expose
            # an object with a `.values` member, so accept both forms.
            raw_args = parameter_value.string_array_value
            args = list(getattr(raw_args, 'values', raw_args))
            if not all(isinstance(arg, str) for arg in args):
                raise TypeError('string array contains a non-string value')
        except Exception as exc:
            self.get_logger().error(f'[detector] cannot read detector_extra_args: {exc}')
            return False

        # Replace a prior filter pair if present; leave all other launch args.
        clean, i = [], 0
        while i < len(args):
            arg = args[i]
            if arg == '-p' and i + 1 < len(args) and args[i + 1].startswith('target_cargo_classes:='):
                i += 2
                continue
            if arg.startswith('target_cargo_classes:='):
                i += 1
                continue
            clean.append(arg)
            i += 1
        clean.extend(['-p', 'target_cargo_classes:=' + ','.join(classes)])

        set_cli = self._service_client(SetParameters, f'/{pm}/set_parameters')
        if not set_cli.wait_for_service(timeout_sec=0.2):
            return False
        set_req = SetParameters.Request()
        set_req.parameters = [Parameter('detector_extra_args',
                                        Parameter.Type.STRING_ARRAY,
                                        clean).to_parameter_msg()]
        set_future = set_cli.call_async(set_req)
        deadline = time.monotonic() + 3.0
        while not set_future.done() and time.monotonic() < deadline:
            if self._cancel_requested:
                return False
            time.sleep(0.01)
        if not set_future.done():
            return False
        ok = all(result.successful for result in set_future.result().results)
        self.get_logger().info(f'[detector] target cargo filter={classes}, applied={ok}')
        return ok

    # ==================================================================
    # Pallet mission status
    # ==================================================================
    def _on_pm_status(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        operation_id = status_counter(d, 'operation_id')
        resume_id = status_counter(d, 'last_resumed_checkpoint_id')
        if operation_id is None or resume_id is None:
            self.get_logger().warn('Ignoring pallet status with invalid operation/checkpoint ID',
                                   throttle_duration_sec=5.0)
            return
        same_session = self._pm_session_id == d.get('session_id')
        if (same_session and self._await_resume_checkpoint is not None
                and resume_id < self._await_resume_checkpoint):
            return
        if (same_session and self._go_called and self._await_operation_after is not None
                and operation_id <= self._await_operation_after):
            return
        self._await_resume_checkpoint = None
        self._pm_operation_id = operation_id
        self._pm_session_id = d.get('session_id')
        self._pm_recovery = d.get('recovery')
        self._pm_handling_status = d.get('handling_status')
        self._pm_waiting_for = d.get('waiting_for')
        self._pm_state = d.get('state')
        self._pm_phase = d.get('phase')
        self._pm_lift = d.get('lift_cm')
        self._pm_samples = int(d.get('samples') or 0)
        self._pm_live = d.get('live')
        self._pm_cargo_type = d.get('cargo_type')
        self._pm_cargo_why = d.get('cargo_why') or ''
        self._pm_cargo = d.get('cargo')
        self._pm_fail_phase = d.get('fail_phase')
        self._pm_insertion_only = d.get('insertion_only') is True
        self._pm_msg = d.get('msg') or ''
        self._pm_err = d.get('err') if isinstance(d.get('err'), dict) else None
        self._pm_retries = d.get('retries') if isinstance(d.get('retries'), dict) else None
        self._pm_status_t = time.monotonic()

    def _on_camera(self, _msg: Image):
        self._camera_last_t = time.monotonic()

    def _on_odom(self, msg: Odometry):
        twist = msg.twist.twist
        self._odom_linear_speed = math.hypot(twist.linear.x, twist.linear.y)
        self._odom_angular_speed = abs(twist.angular.z)
        now = time.monotonic()
        previous = self._odom_t
        self._odom_t = now
        if (now - previous > 0.5 or not self._robot_stopped(now)):
            self._recovery_still_since = None
        elif self._recovery_still_since is None:
            self._recovery_still_since = now

    def _robot_stopped(self, now: float) -> bool:
        """Require fresh odometry and low measured speed before pallet detection."""
        return (
            self._odom_t > 0.0
            and now - self._odom_t <= 0.5
            and self._odom_linear_speed is not None
            and self._odom_angular_speed is not None
            and self._odom_linear_speed <= float(self.p['arrive_max_linear_speed_mps'])
            and self._odom_angular_speed <= float(self.p['arrive_max_angular_speed_rps'])
        )

    def _navigation_idle(self, now: float) -> bool:
        """Only hand control to pallet_mission after the Nav2 goal has ended."""
        # GoalStatusArray is periodic and can be slower than the 10 Hz mission
        # tick. A stale-but-idle sample must not deadlock a completed pallet
        # operation; the pallet action's terminal DONE plus latest inactive
        # state establishes the handoff without accepting an active goal.
        return (self._nav_status_t > 0.0 and self._nav_goal_active is False
                and not any(not f.done() for f in self._navigation_stop_requests))

    def _on_nav_status(self, msg: GoalStatusArray):
        # GoalStatus constants: ACCEPTED=1, EXECUTING=2, CANCELING=3.
        self._nav_goal_active = any(s.status in (1, 2, 3) for s in msg.status_list)
        self._nav_status_t = time.monotonic()

    def _handle_mission_nav_result(self):
        result = self._mission_nav_status
        if not result or self._mission_nav_status_t <= self._mission_nav_consumed_t:
            return
        if result.get('state') not in ('ABORTED', 'CANCELED', 'FAILED'):
            return
        if self._state not in (GOTO_PICK, GOTO_D, 'RETURN_HOME'):
            return
        if result.get('mission') != self._active_mission:
            return
        if self._cancel_requested:
            return
        checkpoint = self._state
        self._mission_nav_consumed_t = self._mission_nav_status_t
        self._triggered = False
        reason = result.get('reason') or result['state']
        mode = result.get('nav_mode') or 'unknown'
        self._set_state(
            'RECOVERY',
            f'Nav2 {result["state"]} for {result["mission"]} '
            f'(mode={mode}): {reason}')
        self._resume_state = checkpoint

    def _on_detector_status(self, msg: String):
        try:
            self._detector_status = json.loads(msg.data)
            self._detector_status_t = time.monotonic()
        except Exception:
            return

    def _camera_ok(self):
        return (self._camera_last_t > 0.0 and
                time.monotonic() - self._camera_last_t <= float(self.p['camera_stale_s']))

    # ==================================================================
    # Shelf occupancy (loi 2 ESP serial -> /shelf/occupancy)
    # ==================================================================
    def _on_shelf(self, msg: Int8MultiArray):
        # 4 bit 0/1 theo thu tu o (bin co hang = 1). esp32_bridge gui
        # [-1,-1,-1,-1] khi stale -> coi la mat ket noi (ghi None).
        # [0,0,0,0] (tat ca tron) la tinh TRANG HOP LE, khong phai stale.
        data = list(msg.data)
        self._shelf_input = data
        if any(v < 0 for v in data):
            self._shelf_bits = None
        else:
            self._shelf_bits = [1 if v > 0 else 0 for v in data]
        self._shelf_last_t = time.monotonic()

    def _shelf_ok(self) -> bool:
        """True neu van chap nhan du lieu khoang /shelf/occupancy (khong stale)."""
        if self._shelf_last_t == 0.0:
            return False
        return (time.monotonic() - self._shelf_last_t) <= float(self.p['shelf_stale_s'])

    def _shelf_expired(self, now) -> bool:
        """Du lieu ke tuoi -> False. Stale: bat dau dem; chi tra True (-> RECOVERY)
        khi stale lien tuc > shelf_wait_s. Tranh dung xe vi 1 nhip tre."""
        if self._shelf_ok() and self._shelf_bits is not None:
            self._shelf_bad_t0 = None
            return False
        if self._shelf_bad_t0 is None:
            self._shelf_bad_t0 = now
            self.get_logger().warn('[shelf] du lieu ke stale - cho tuoi tro lai '
                                   f'(toi da {self.p["shelf_wait_s"]:.0f}s) truoc khi RECOVERY')
        return (now - self._shelf_bad_t0) > float(self.p['shelf_wait_s'])

    # Pha ApproachController ma khi FAILED thi cang CHUA vao pallet/ke
    # (ENTER va LOCK bi loai: mui cang da sat/qua mat, can nguoi kiem tra)
    # DETECT_DRIFT xay ra tai diem lay, robot dung yen, chua cham vao pallet.
    # ENTER va LOCK co tinh khong an toan de tu dong retry, khong them vao day.
    SAFE_RETRY_PHASES = ('DETECT_DRIFT',)

    def _drop_refused(self, now, why: str):
        """pallet_mission tu choi Drop (xe KHONG chuyen dong): thu lai vai lan roi moi RECOVERY,
        va hien dung ly do (truoc day ly do bi bo, chi con 'can kiem tra pallet/lift')."""
        self._start_refusals += 1
        self.get_logger().warn(f'[DROPPING] start deferred ({self._start_refusals}): {why}',
                               throttle_duration_sec=5.0)
        self._retry_wait_until = now + max(2.0, float(self.p['retry_backoff_s']))

    def _can_auto_retry(self, used: int) -> bool:
        blockers = []
        if (self._pm_recovery or {}).get('checkpoint'):
            blockers.append('child checkpoint owns the interrupted operation')
        if self._pm_insertion_only:
            blockers.append('raised forks/insertion-only; a new operation could duplicate DROP')
        if self._pm_fail_phase not in self.SAFE_RETRY_PHASES:
            blockers.append(f'phase {self._pm_fail_phase!r} is not a supervisor retry phase')
        if used >= int(self.p['approach_retries']):
            blockers.append('supervisor retry budget exhausted')
        if self._cancel_requested:
            blockers.append('cancel requested')
        if blockers:
            self.get_logger().warn(
                f'[AUTO_RETRY_DENIED] phase={self._pm_fail_phase} '
                f'operation_id={self._pm_operation_id}: ' + '; '.join(blockers),
                throttle_duration_sec=5.0)
            return False
        return True

    def _shelf_states(self) -> dict:
        """Trang thai 4 o (A1/A2/B1/B2) tu /shelf/occupancy + shelf.yaml.

        Anh xa bit -> o do shelf_model lo (o 'unknown' khi -1 hoac du lieu
        stale - KHONG co 'empty' tu du lieu khong that). Ket qua:
        {id_o: 'full' | 'empty' | 'unknown'}.
        """
        if self._shelf_model is None:
            return {}
        fresh = self._shelf_ok() and self._shelf_bits is not None
        data = self._shelf_bits if self._shelf_bits is not None else [-1] * 4
        return self._shelf_model.slot_states(data, fresh=fresh)

    def _load_shelf_model(self):
        """Doc config/shelf.yaml (mac dinh tu share package). None = loi."""
        try:
            import os
            path = self.p['shelf_config']
            if not path:
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(
                    get_package_share_directory('amr_mission_manager'),
                    'config', 'shelf.yaml')
            model = ShelfModel.from_yaml(path)
            for w in model.provisional_sensor_warnings():
                self.get_logger().warn(f'[ke][CAM BIEN TAM] {w}')
            self.get_logger().info(
                f'[ke] Da doc shelf config: {path} (anchor={model.anchor_node}, '
                f'{len(model.slots)} o)')
            return model
        except Exception as e:
            self.get_logger().error(
                f'KHONG doc duoc shelf config: {e} - chuc nang dat hang bi chon')
            return None

    # ==================================================================
    # State machine
    # ==================================================================
    def _sample_window_stable(self):
        live = self._pm_live
        if not live or time.monotonic() - self._pm_status_t > 1.5:
            self._stable_since, self._stable_anchor = None, None
            return False
        point = (float(live.get('x', 0.0)), float(live.get('y', 0.0)),
                 math.radians(float(live.get('yaw_deg', 0.0))))
        if self._stable_anchor is None:
            self._stable_anchor, self._stable_since = point, time.monotonic()
            return False
        dyaw = math.atan2(math.sin(point[2] - self._stable_anchor[2]),
                          math.cos(point[2] - self._stable_anchor[2]))
        stable = (math.hypot(point[0] - self._stable_anchor[0],
                             point[1] - self._stable_anchor[1]) <= float(self.p['pm_max_pos_std_m'])
                  and abs(math.degrees(dyaw)) <= float(self.p['pm_max_yaw_std_deg']))
        if not stable:
            self._stable_anchor, self._stable_since = point, time.monotonic()
            return False
        return self._stable_since is not None and (
            time.monotonic() - self._stable_since >= float(self.p['sample_stable_s']))

    def _any_slot_available(self):
        return self._shelf_model is not None and any(
            v == EMPTY for v in self._shelf_states().values())

    def _requested_slot_available(self):
        requested = str(self.p.get('target_cargo_class') or '').strip().lower()
        if requested == 'medium':
            return self._select_drop_slot_for('A')[0] is not None
        if requested == 'tall':
            return self._select_drop_slot_for('B')[0] is not None
        return (self._select_drop_slot_for('A')[0] is not None
                or self._select_drop_slot_for('B')[0] is not None)

    def _select_drop_slot_for(self, cargo_type):
        if self._shelf_model is None:
            return None, None, 'Khong doc duoc shelf.yaml'
        states = dict(self._shelf_states())
        for sid in tuple(self._reserved_drop_slots):
            if states.get(sid) == FULL:
                self._reserved_drop_slots.discard(sid)
        # A just-placed box can take a sensor update cycle to appear. Reserve
        # confirmed drop slots for this mission so a multi-item request cannot
        # select the same apparently-empty slot twice.
        for sid in self._reserved_drop_slots:
            states[sid] = FULL
        requested = str(self.p.get('target_drop_slot') or '').strip().upper()
        if requested:
            slot = self._shelf_model._slot_by_id.get(requested)
            if slot is None or slot.product_type != cargo_type:
                return None, None, f'O ke {requested} khong phu hop loai hang {cargo_type}'
            if states.get(requested) != EMPTY:
                return None, None, f'O ke {requested} khong duoc xac nhan la trong'
            return requested, slot.tf_frame, ''
        sid, why = self._shelf_model.select_drop_slot(cargo_type, states)
        if sid is None:
            return None, None, why
        return sid, self._shelf_model._slot_by_id[sid].tf_frame, ''

    # ==================================================================
    # Diem lay duy nhat (P_1) - da bo nhanh lay thu hai (P_2/D_2).
    # ==================================================================
    def _build_pick_queue(self) -> list:
        """Hang doi diem lay cho chu ky: luon chi [P_1] (diem lay duy nhat)."""
        return [self.p['pick_dest']]

    def _current_pick_dest(self):
        """Diem lay duy nhat."""
        if not self._pick_queue:
            return self.p['pick_dest']
        return self._pick_queue[0]

    def _mission_for_pick_dest(self, dest: str) -> str:
        """Mission de di toi `dest`.

        - Diem lay (P_1) khi bat dau chu ky (tu HOME): dung `pick_mission`.
        - Khi bat dau chu ky moi (tu D_1 sau khi da dat len ke): dung
          `d1_to_p1` neu co, nguoc lai `pick_mission`.
        """
        if dest == self.p['pick_dest']:
            return self.p['pick_mission']
        return self._mission_from_d1(dest)

    @staticmethod
    def _mission_from_d1(dest: str) -> str:
        """Return the registered mission ID for traveling from D_1 to a pick node."""
        # Mission service ID d1_to_p1 (khong bien doi ten bang lower()).
        d1_missions = {'P_1': 'd1_to_p1'}
        return d1_missions.get(dest, '')

    def _start_return_home(self, why):
        incomplete = (self.p.get('run_mode') == 'single'
                      and self._cycle < int(self._mission_target_total or self.p.get('target_quantity', 1)))
        self._terminal_after_home = FAILED if incomplete else DONE
        if incomplete:
            requested = self.p.get('target_cargo_class') or 'any'
            why = (f'Mission incomplete: placed {self._cycle}/'
                   f'{self._mission_target_total or self.p.get("target_quantity", 1)} {requested}; {why}')
        self._terminal_reason = why
        self._active_mission, self._active_dest = 'return_home', 'NODE_HOME'
        self._triggered, self._near_t0 = False, None
        self._set_state('RETURN_HOME', why + ' -> tu dong ve HOME')

    # ==================================================================
    # Ke da day: DUNG TAI D_1, tu tham do lai o trong
    # ==================================================================
    def _enter_waiting_slot(self, why: str):
        """Xe dang CAM PALLET ma ke khong con o trong dung loai.

        Day KHONG phai loi -> KHONG vao RECOVERY. Xe DUNG TAI D_1, tu tham
        do lai /shelf/occupancy moi `slot_poll_s` giay; co o trong thi dat
        vao o do ngay. Qua `slot_wait_max_s` chi BAO ve nguoi van hanh
        (van dung chos o D_1 de cho o tro lai, khong di chuyen khac).
        """
        self._last_slot_poll = time.monotonic()
        self._slot_wait_why = why
        self._slot_operator_notified = False
        self._triggered, self._near_t0 = False, None
        self._go_called = False
        self._t_state = time.monotonic()
        self._set_state(WAITING_SLOT, why)

    def _do_waiting_slot(self, now):
        """Dung tai D_1, cho co o trong (xe van giu pallet)."""
        # Mat lien lac cam bien ke -> khong biet khi nao co o -> can nguoi.
        if self._shelf_expired(now):
            self._set_state('RECOVERY', 'Cam bien ke mat ket noi khi dang cho o trong; '
                                        'xe van giu pallet tai D_1')
            self._resume_state = DROPPING
            return

        waited = now - self._t_state
        # Qua thoi gian cho: BAO ve nguoi van hanh (de web/GUI hien thi).
        # Van dung chos o D_1 de cho o tro lai - khong di chuyen khac.
        if (waited > float(self.p['slot_wait_max_s'])
                and not getattr(self, '_slot_operator_notified', False)):
            self._slot_operator_notified = True
            self.get_logger().warn(
                f'[WAITING_SLOT] Da cho {waited:.0f}s (hon gioi han '
                f'{self.p["slot_wait_max_s"]:.0f}s) - bao nguoi van hanh '
                f'doa keo/rap hang bo cho. {self._slot_wait_why}')

        # Tham do o trong moi `slot_poll_s` giay.
        if now - self._last_slot_poll < float(self.p['slot_poll_s']):
            return
        self._last_slot_poll = now

        sid, frame, why = self._select_drop_slot_for(self._pm_cargo_type)
        if sid is None:
            self._slot_wait_why = why
            # Cap nhat status nhung giu moc thoi gian vao WAITING_SLOT de gioi
            # han slot_wait_max_s tinh tu luc bat dau cho, khong bi reset moi poll.
            self._msg = (f'Ke da day, dang cho o trong (da cho {waited:.0f}s): '
                         f'{why}')
            return
        # Co o trong -> dat ngay, khong can nguoi bam RESUME.
        # Lan ve GOTO_D de (neu da di chuyen ve HOME trong truoc do) lai D_1
        # roi chon o; neu xe da dung o D_1 thi GOTO_D tu nhay qua DROPPING.
        self._drop_slot_chosen = (sid, frame)
        self._go_called = False
        self._t_state = now
        self._triggered = False
        self._set_state(GOTO_D, f'Da co o trong {sid} -> di dat pallet '
                                f'loai {self._pm_cargo_type} vao {sid}')

    def _do_goto_d(self, now):
        if self._shelf_expired(now):
            try:
                self._request_owned_navigation_stop()
            except Exception:
                pass
            self._set_state('RECOVERY', 'Du lieu cam bien ke stale/mat ket noi tren duong toi D_1')
            self._resume_state = GOTO_D
            return
        if self._shelf_ok() and self._shelf_bits is not None:
            # Ke chi can tuoi luc chon o; tren duong di thoang stale thi KHONG cat Nav2.
            sid, frame, why = self._select_drop_slot_for(self._pm_cargo_type)
            if sid is None:
                try:
                    self._request_owned_navigation_stop()
                except Exception:
                    pass
                # Dang CAM PALLET + ke khong con o phu hop -> cho tai D_1,
                # khong ve HOME (ve HOME se lam cycle chet: xe cam pallet ->
                # /mission/start bi tu choi vinh vien).
                self._enter_waiting_slot('Ke khong con o phu hop tren duong toi D_1: ' + why)
                return
            self._drop_slot_chosen = (sid, frame)
        if not self._triggered:
            self._active_dest = 'D_1'
            d0, p0 = self._dest_pose(), self._base_pose()
            if (d0 is not None and p0 is not None
                    and math.hypot(p0[0] - d0[0], p0[1] - d0[1]) <= float(self.p['arrive_tol_m'])
                    and self._nav_goal_active is not True):
                # RESUME khi xe da dung o D_1: KHONG gui lai Nav2 (se di chuyen them va
                # lam lech pose truoc khi vao ke).
                self.get_logger().info('[GOTO_D] Xe da o D_1 -> bo qua gui lai mission Nav2')
                self._aruco_route_gate.arrived_d1()
                self._publish_aruco_gate()
                self._triggered = True
                self._resume_free_nav = False
                return
            if self._trigger_mission(self.p['drop_mission'], self._resume_free_nav):
                self._triggered = True
                self._resume_free_nav = False
            elif now - self._t_state > 30.0:
                self._set_state('RECOVERY', 'Khong goi duoc mission toi D_1')
                self._resume_state = GOTO_D
            return
        self._active_dest = 'D_1'
        dest, pose = self._dest_pose(), self._base_pose()
        if dest is None or pose is None:
            if now - self._t_state > 10.0:
                try:
                    self._request_owned_navigation_stop()
                except Exception:
                    pass
                self._set_state('RECOVERY', 'Khong doc duoc pose D_1/robot')
                self._resume_state = GOTO_D
            return
        d = math.hypot(pose[0] - dest[0], pose[1] - dest[1])
        if d <= float(self.p['arrive_tol_m']):
            # Truoc day chi can vao ban kinh arrive_tol_m 1s la bat dau Drop, du Nav2 van dang
            # chay -> hai nguon lenh cmd_vel va pose xuat phat Drop chua on dinh. Giong GOTO_PICK:
            # cho xe DUNG va Nav2 khong con goal active.
            if not self._robot_stopped(now) or self._nav_goal_active is True:
                self._near_t0 = None
                if self._robot_stopped(now) and self._nav_goal_active is True:
                    self._stuck_t0 = self._stuck_t0 or now
                    if now - self._stuck_t0 > float(self.p['nav_finish_wait_s']):
                        self.get_logger().warn('[GOTO_D] Xe da dung o D_1 nhung Nav2 goal chua ket thuc -> huy goal')
                        try:
                            self._request_owned_navigation_stop()
                        except Exception:
                            pass
                        self._stuck_t0 = None
                else:
                    self._stuck_t0 = None
            else:
                self._stuck_t0 = None
                self._near_t0 = self._near_t0 or now
            if self._near_t0 is not None and now - self._near_t0 >= float(self.p['arrive_settle_s']):
                self._aruco_route_gate.arrived_d1()
                self._publish_aruco_gate()
                if not (self._shelf_ok() and self._shelf_bits is not None):
                    if self._shelf_expired(now):
                        self._set_state('RECOVERY', 'Mat ket noi/cam bien ke stale khi den D_1')
                        self._resume_state = GOTO_D
                    # else: dung o D_1 cho du lieu ke tuoi tro lai
                else:
                    sid, frame, why = self._select_drop_slot_for(self._pm_cargo_type)
                    if sid is None:
                        self._enter_waiting_slot('Ke khong con o phu hop: ' + why)
                    else:
                        self._drop_slot_chosen = (sid, frame)
                        self._set_state(DROPPING, f'Dat pallet loai {self._pm_cargo_type} vao {sid}')
                        self._go_called = False
        else:
            self._near_t0 = None
        if now - self._t_state > float(self.p['goto_timeout_s']):
            try:
                self._request_owned_navigation_stop()
            except Exception:
                pass
            self._set_state('RECOVERY', 'Di toi D_1 qua timeout; dang giu pallet')
            self._resume_state = GOTO_D

    def _do_dropping(self, now):
        pm = self.p['pallet_mission_node']
        if not self._go_called:
            if now < self._retry_wait_until:
                return
            if not (self._shelf_ok() and self._shelf_bits is not None):
                if self._shelf_expired(now):
                    self._enter_waiting_slot('Du lieu ke stale/mat ket noi truoc khi bat dau drop')
                return
            sid, frame, why = self._select_drop_slot_for(self._pm_cargo_type)
            if sid is None:
                self._enter_waiting_slot('O ke khong con phu hop: ' + why)
                return
            # Re-select while travelling; pass the selected slot through the
            # existing pallet_mission parameter/service contract.
            self._drop_slot_chosen = (sid, frame)
            cli = self._service_client(SetParameters, f'/{pm}/set_parameters')
            if not cli.wait_for_service(timeout_sec=0.2):
                self._drop_refused(now, 'Pallet mission set_parameters chua san sang')
                return
            req = SetParameters.Request()
            req.parameters = [
                Parameter('drop_slot', Parameter.Type.STRING, sid).to_parameter_msg(),
                Parameter('cargo_type_override', Parameter.Type.STRING, '').to_parameter_msg(),
            ]
            result = self._call_service_result(cli, req, timeout=2.)
            if self._cancel_requested:
                return
            if result is None or not all(x.successful for x in result.results):
                self._drop_refused(now, 'Khong dat duoc drop_slot cho pallet_mission')
                return
            drop = self._service_client(Trigger, f'/{pm}/drop')
            if not drop.wait_for_service(timeout_sec=0.2):
                self._drop_refused(now, 'Pallet mission drop service chua san sang')
                return
            if self._cancel_requested:
                return
            self._await_operation_after = self._pm_operation_id
            result = self._call_service_result(
                drop, Trigger.Request(), timeout=3., retain_on_timeout=True)
            if self._cancel_requested:
                return
            if result is None and self._uncertain_child_request is not None:
                self._go_called = True
                self._pm_state = None
                self._t_state = now
                return
            if result is None or not result.success:
                why = result.message if result else 'service khong tra loi trong 3s'
                self._drop_refused(now, f'Drop chua khoi dong: {why}')
                return
            self._start_refusals = 0
            self._go_called = True
            self._pm_state = None      # bo status cu (FAILED lan truoc)
            self._t_state = now
            return
        ready, pending_result = self._pending_child_result()
        if not ready:
            return
        if pending_result is not None and not pending_result.success:
            self._go_called = False
            self._drop_refused(now, pending_result.message)
            return
        if self._pm_state == DONE and self._pm_cargo is False:
            # The pallet operation reaches DONE only after its owned Nav2 goal
            # is terminal; also require the supervisor's latest status not to
            # show an active goal before changing task or reporting completion.
            if not self._navigation_idle(now):
                nav_age = (now - self._nav_status_t if self._nav_status_t > 0.0
                           else float('inf'))
                pending_stops = sum(not f.done() for f in self._navigation_stop_requests)
                self.get_logger().warn(
                    f'[DROPPING] pallet operation DONE/cargo=false; waiting for Nav2 idle: '
                    f'goal_active={self._nav_goal_active}, status_age={nav_age:.1f}s, '
                    f'pending_stop_services={pending_stops}', throttle_duration_sec=5.0)
                return
            if not self._drop_slot_chosen:
                self._set_state('RECOVERY', 'DROP reported DONE without a validated shelf slot')
                self._resume_state = DROPPING
                return
            if self._drop_slot_chosen:
                self._reserved_drop_slots.add(self._drop_slot_chosen[0])
            self._cycle += 1
            self._task_placed += 1
            self._drop_retries = 0
            self._go_called = False
            transition, next_index = (task_after_placement(self._mission_queue,
                                  self._mission_task_index, self._task_placed)
                                  if self.p.get('run_mode') == 'single' and self._mission_queue
                                  else ('complete', self._mission_task_index))
            if transition == 'next':
                self._mission_task_index = next_index
                self._task_placed = 0
                task = self._mission_queue[next_index]
                self.p['target_cargo_class'] = '' if task['cargo_class'] == 'any' else task['cargo_class']
                self.p['target_quantity'] = task['quantity']
                self.p['target_drop_slot'] = '' if task['drop_slot'] == 'auto' else task['drop_slot']
                if not self._requested_slot_available():
                    self._start_return_home('Mission queue task has no compatible empty shelf slot')
                    return
                self._pick_queue = self._build_pick_queue()
                self._pick_idx = 0
                self._active_dest = self._current_pick_dest()
                self._active_mission = self._mission_from_d1(self._active_dest)
                self._triggered, self._near_t0 = False, None
                self._set_state(GOTO_PICK,
                    f'Queue task {next_index + 1}/{len(self._mission_queue)}: '
                    f'{task["cargo_class"]} -> {task["drop_slot"]}; D_1 to {self._active_dest}')
                return
            if self.p.get('run_mode') == 'auto' and (not self._shelf_ok() or self._shelf_bits is None):
                self._start_return_home('Da dat pallet; cam bien ke mat ket noi nen dung Auto')
            elif (self.p.get('run_mode') == 'auto'
                  or transition == 'repeat'):
                if not self._requested_slot_available():
                    self._start_return_home('Da het o ke phu hop voi loai hang can lay')
                    return
                self._pick_queue = self._build_pick_queue()
                self._pick_idx = 0
                self._active_dest = self._current_pick_dest()
                self._active_mission = self._mission_from_d1(self._active_dest)
                self._triggered, self._near_t0 = False, None
                self._set_state(GOTO_PICK,
                                f'Auto: da dat xong pallet {self._cycle}, quay lai '
                                f'{self._active_dest} bang {self._active_mission}')
            else:
                self._start_return_home('Hoan tat chu ky' if self.p.get('run_mode') != 'auto'
                                         else 'Ke da day')
        elif self._pm_state == DONE and self._pm_cargo is not False:
            self.get_logger().warn(
                f'[DROPPING] pallet node DONE but cargo is not confirmed false: '
                f'cargo={self._pm_cargo}, operation_id={self._pm_operation_id}, '
                f'pm_status_age={now - self._pm_status_t:.1f}s',
                throttle_duration_sec=5.0)
        elif self._pm_state == FAILED and now - self._t_state > 1.5:
            if self._can_auto_retry(self._drop_retries):
                self._drop_retries += 1
                self.get_logger().warn(
                    f'[DROPPING] Drop FAILED o pha {self._pm_fail_phase} (cang chua vao ke) '
                    f'-> tu thu lai {self._drop_retries}/{self.p["approach_retries"]}')
                self._pm_state = None
                self._go_called = False
                self._t_state = now
                self._retry_wait_until = now + float(self.p['retry_backoff_s'])
                return
            self._set_state('RECOVERY', 'Drop FAILED; dung tai checkpoint, kiem tra vi tri pallet '
                                        f'(pha loi: {self._pm_fail_phase})')
            self._resume_state = DROPPING
        elif self._pm_state == STOPPED:
            self._set_state('RECOVERY', 'DROP stopped; inspect pallet and staging before Resume')
            self._resume_state = DROPPING
        elif now - self._t_state > float(self.p.get('drop_timeout_s', 360.0)):
            self.get_logger().warn('DROP duration exceeded warning deadline; child operation retained',
                                   throttle_duration_sec=5.0)
            self._t_state = now

    def _do_return_home(self, now):
        if not self._triggered:
            if self._trigger_mission('return_home', self._resume_free_nav):
                self._triggered = True
                self._resume_free_nav = False
                self._active_dest = 'NODE_HOME'
                self._near_t0 = None
            elif now - self._t_state > 30.0:
                self._set_state('RECOVERY', 'Khong goi duoc return_home')
                self._resume_state = 'RETURN_HOME'
            return
        if now - self._t_state > float(self.p['goto_timeout_s']):
            try:
                self._request_owned_navigation_stop()
            except Exception:
                pass
            self._set_state('RECOVERY', 'Ve HOME qua timeout; da yeu cau huy goal Nav2')
            self._resume_state = 'RETURN_HOME'
            return
        dest, pose = self._dest_pose(), self._base_pose()
        if dest is None or pose is None:
            if now - self._t_state > 10.0:
                self._set_state('RECOVERY', 'Khong doc duoc pose HOME/robot de xac nhan ve nha')
                self._resume_state = 'RETURN_HOME'
            return
        if math.hypot(pose[0] - dest[0], pose[1] - dest[1]) <= float(self.p['arrive_tol_m']):
            result = self._mission_nav_status or {}
            confirmed = (self._mission_nav_run_id is not None
                         and result.get('run_id') == self._mission_nav_run_id
                         and result.get('mission') == 'return_home'
                         and result.get('state') == 'SUCCEEDED'
                         and result.get('busy') is False
                         and self._manager_status_run_id == self._mission_nav_run_id
                         and self._manager_busy is False
                         and self._manager_status_t > 0.0
                         and now - self._manager_status_t <= 5.0
                         and self._nav_status_t > 0.0
                         and now - self._nav_status_t <= 2.0
                         and self._navigation_idle(now)
                         and self._robot_stopped(now))
            if not confirmed:
                self._near_t0 = None
                return
            self._near_t0 = self._near_t0 or now
            if now - self._near_t0 >= float(self.p['arrive_settle_s']):
                # Ve HOME xong NHUNG van giu pallet: khong co o ke de dat, va
                # /mission/start se bi tu choi (dang mang pallet) -> cycle chet.
                # Vao WAITING_SLOT de xe dung cho o trong (khong phai loi).
                if self._pm_cargo is True:
                    self._enter_waiting_slot(
                        'Da ve HOME nhung van giu pallet; ke chua co o trong de dat')
                else:
                    terminal = self._terminal_after_home
                    if terminal == FAILED:
                        self._set_state(FAILED, self._terminal_reason or
                                        f'Mission incomplete: placed {self._cycle}/'
                                        f'{self.p.get("target_quantity", 1)}')
                    else:
                        self._set_state(DONE, f'Ve HOME xong; da dat {self._cycle} pallet len ke')
        else:
            self._near_t0 = None

    def _tick(self):
        if self._state == 'STOPPING':
            self._advance_stop()
            return
        if self._cancel_requested:
            return
        self._tick_inflight = True
        self._tick_context.epoch = self._mission_epoch
        try:
            self._run_mission_tick()
        finally:
            self._tick_context.epoch = None
            self._tick_inflight = False

    def _run_mission_tick(self):
        now = time.monotonic()

        self._handle_mission_nav_result()

        if self._state == GOTO_PICK:
            self._do_goto_pick(now)
        elif self._state == PICKING:
            self._do_picking(now)
        elif self._state == GOTO_D:
            self._do_goto_d(now)
        elif self._state == DROPPING:
            self._do_dropping(now)
        elif self._state == WAITING_SLOT:
            self._do_waiting_slot(now)
        elif self._state == 'RETURN_HOME':
            self._do_return_home(now)

    def _do_goto_pick(self, now):
        if self._shelf_expired(now):
            try:
                self._request_owned_navigation_stop()
            except Exception:
                pass
            self._set_state('RECOVERY', 'Du lieu cam bien ke stale/mat ket noi truoc khi di pick')
            self._resume_state = GOTO_PICK
            return
        if (self._shelf_ok() and self._shelf_bits is not None
                and not self._requested_slot_available()):
            self._start_return_home('Khong con o ke trong phu hop')
            return
        if not self._triggered:
            d0, p0 = self._dest_pose(), self._base_pose()
            if (d0 is not None and p0 is not None
                    and math.hypot(p0[0] - d0[0], p0[1] - d0[1]) <= float(self.p['arrive_tol_m'])
                    and self._nav_goal_active is not True):
                self.get_logger().info('[GOTO_PICK] Xe da o diem pick -> bo qua gui lai mission Nav2')
                self._triggered = True
                self._resume_free_nav = False
                return
            if self._trigger_mission(self._active_mission, self._resume_free_nav):
                self._triggered = True
                self._resume_free_nav = False
            if now - self._t_state > 30.0:
                self._set_state('RECOVERY', 'Service mission_manager khong san sang')
                self._resume_state = GOTO_PICK
            return
        # Choi den choi: TF map->base trong ban kinh + on dinh
        dest = self._dest_pose()
        pose = self._base_pose()
        if dest is None or pose is None:
            if now - self._t_state > 10.0:
                try:
                    self._request_owned_navigation_stop()
                except Exception:
                    pass
                self._set_state('RECOVERY', f'Khong doc duoc pose (dest={dest is not None}, '
                                        f'base={pose is not None})')
                self._resume_state = GOTO_PICK
            return
        d = math.hypot(pose[0] - dest[0], pose[1] - dest[1])
        if d <= float(self.p['arrive_tol_m']):
            if not self._navigation_idle(now) or not self._robot_stopped(now):
                # Position alone is insufficient. Wait for the current Nav2
                # action to finish and for the base to stop before detection.
                self._near_t0 = None
            elif self._near_t0 is None:
                self._near_t0 = now
            elif now - self._near_t0 >= float(self.p['arrive_settle_s']):
                self._near_t0 = None
                if not (self._shelf_ok() and self._shelf_bits is not None):
                    if self._shelf_expired(now):
                        self._set_state('RECOVERY', 'Du lieu cam bien ke stale/mat ket noi tai P_1')
                        self._resume_state = GOTO_PICK
                    # else: cho du lieu ke tuoi tro lai, thu lai o nhip sau
                elif not self._requested_slot_available():
                    self._start_return_home('Khong con o ke trong hoac du lieu ke khong hop le')
                else:
                    self._enter_picking()
        else:
            self._near_t0 = None
        if now - self._t_state > float(self.p['goto_timeout_s']):
            try:
                self._request_owned_navigation_stop()
            except Exception:
                pass
            self._set_state('RECOVERY', f'Ve {self._active_dest} qua thoi gian '
                                    f'(con {d * 100:.0f} cm)')
            self._resume_state = GOTO_PICK

    def _enter_picking(self):
        if not self._camera_ok():
            self._set_state('RECOVERY', 'Camera pallet khong co frame moi tai P_1')
            self._resume_state = PICKING
            return
        self._pick_phase = S_SAMPLING
        self._detector_filter_applied = False
        self._set_state(PICKING, 'Bat dau chu trinh lay pallet')

    def _do_picking(self, now):
        pm = self.p['pallet_mission_node']

        if not self._pm_sampling_applied:
            self._apply_pm_sampling()
            self._pm_sampling_applied = True

        if not self._detector_filter_applied:
            if not self._apply_detector_filter():
                self._set_state('RECOVERY', 'Khong cau hinh duoc detector theo loai pallet con o trong ke')
                self._resume_state = PICKING
                return
            self._detector_filter_applied = True

        if self._pick_phase == S_SAMPLING:
            if not self._detect_on_called:
                if now < self._retry_wait_until:
                    return
                cli = self._service_client(SetBool, f'/{pm}/detect')
                req = SetBool.Request()
                req.data = True
                response = self._call_service_result(cli, req)
                if self._cancel_requested:
                    return
                self._detect_on_called = bool(response and response.success)
                self._pm_samples = 0
                self._stable_since, self._stable_anchor = None, None
                if self._detect_on_called:
                    self._t_state = now
                    self._detect_on_t = now
                    self._scan_t0 = None
                    self._pm_state = None          # bo status cu (FAILED lan truoc)
                elif not self._camera_ok():
                    self._set_state('RECOVERY', 'Detect service tu choi va camera stale/mat ket noi')
                    self._resume_state = PICKING
                else:
                    self._detector_restarts += 1
                    why = response.message if response else 'service khong tra loi'
                    if self._detector_restarts > int(self.p['detector_restarts']):
                        self._set_state('RECOVERY', f'Bat detect that bai {self._detector_restarts} lan: {why}')
                        self._resume_state = PICKING
                    else:
                        self.get_logger().warn(f'[PICKING] Bat detect that bai ({why}) -> thu lai '
                                               f'{self._detector_restarts}/{self.p["detector_restarts"]}')
                        self._retry_wait_until = now + float(self.p['retry_backoff_s'])
                return
            # pallet_mission FAILED ngay trong luc lay mau (detector tu thoat, khong len duoc...):
            # tu bat lai detector vai lan truoc khi vao RECOVERY; khong cho het thoi gian roi doan.
            if self._pm_state == FAILED and now - self._detect_on_t > 1.5:
                self._detector_restarts += 1
                if self._detector_restarts > int(self.p['detector_restarts']):
                    self._set_state('RECOVERY', 'pallet_mission FAILED khi detect (da tu bat lai detector '
                                                f'{self._detector_restarts - 1} lan): {self._pm_msg}')
                    self._resume_state = PICKING
                    return
                self.get_logger().warn(f'[PICKING] pallet_mission FAILED khi detect ({self._pm_msg}) -> '
                                       f'bat lai detector {self._detector_restarts}/{self.p["detector_restarts"]}')
                self._pm_state = None
                self._detect_on_called = False
                self._retry_wait_until = now + float(self.p['retry_backoff_s'])
                return
            enough = self._pm_samples >= int(self.p['pm_min_samples'])
            stable = self._sample_window_stable()
            # Dong ho "khong thay pallet" chi chay tu luc DETECTOR THAT SU bao status (da nap model
            # xong va con song), nen 10 s la 10 s nhin that chu khong gom thoi gian nap model.
            det_live = (self._detector_status_t > self._detect_on_t
                        and now - self._detector_status_t <= 2.0)
            if det_live and self._scan_t0 is None:
                self._scan_t0 = now
            no_pallet = (self._pm_samples == 0 and self._scan_t0 is not None
                         and now - self._scan_t0 >= float(self.p['no_pallet_conclude_s']))
            if (enough and stable) or no_pallet or now - self._t_state >= float(self.p['pick_sample_wait_s']):
                # dat -> detect OFF (chot median; success <=> state READY)
                self._t_state = now
                self._detect_on_called = False
                self._go_called = False
                cli = self._service_client(SetBool, f'/{pm}/detect')
                req = SetBool.Request()
                req.data = False
                res = self._call_service_result(cli, req)
                if self._cancel_requested:
                    return
                if res is None:
                    self._pick_retry('Service detect khong du')
                    return
                self.get_logger().info(f'[PICKING] detect OFF: {res.message}')
                if res.success:
                    self._detector_restarts = 0
                    self._pick_phase = S_APPROACHING
                    self._t_state = now
                else:
                    if self._pm_samples == 0:
                        if not self._camera_ok():
                            self._set_state('RECOVERY', 'Khong co mau va camera stale; khong ket luan P_1 het pallet')
                            self._resume_state = PICKING
                            return
                        if (self._detector_status is None or
                                time.monotonic() - self._detector_status_t > 2.0):
                            self._set_state('RECOVERY', 'Camera con frame nhung detector khong gui target status')
                            self._resume_state = PICKING
                            return
                        unknown = status_counter(self._detector_status, 'unknown_loaded')
                        compatible = status_counter(self._detector_status, 'compatible_pallets')
                        unlocalized = status_counter(self._detector_status, 'unlocalized_loaded')
                        if None in (unknown, compatible, unlocalized):
                            self.get_logger().warn(
                                'Detector target status has null/missing counters; waiting for valid data',
                                throttle_duration_sec=5.0)
                            return
                        if unknown > 0 or compatible > 0 or unlocalized > 0:
                            self._pick_retry(
                                'Detector thay pallet nhung chua the khoa class-compatible target '
                                f'(unknown={unknown}, unlocalized={unlocalized}, compatible={compatible})')
                            return
                        self._empty_scans += 1
                    if self._empty_scans >= int(self.p['no_pallet_retries']):
                        # Diem lay duy nhat la P_1 (da bo nhanh thu hai P_2/D_2):
                        # het pallet -> ket thuc chu ky, ve HOME.
                        self._start_return_home(
                            f'{self._active_dest} khong detect duoc pallet sau '
                            f'{self._empty_scans} lan quet')
                    else:
                        self._pick_retry(res.message)
            return

        # S_APPROACHING
        if not self._go_called:
            if self._pm_state not in ('READY', 'IDLE') or time.monotonic() - self._pm_status_t > 1.5:
                if now - self._t_state > 4.0:
                    self._pick_retry('Cho status detect OFF tu pallet_mission qua han')
                return
            if self._pm_cargo_type not in ('A', 'B'):
                self._pick_retry('Khong phan loai duoc chieu cao pallet: ' + (self._pm_cargo_why or 'unknown'))
                return
            sid, frame, why = self._select_drop_slot_for(self._pm_cargo_type)
            if sid is None:
                self._start_return_home(
                    f'Khong co o phu hop cho pallet {self._pm_cargo_type or "?"}: {why}')
                return
            self._drop_slot_chosen = (sid, frame)
            cli = self._service_client(Trigger, f'/{pm}/go_to_pallet')
            self._await_operation_after = self._pm_operation_id
            res = self._call_service_result(cli, Trigger.Request(), retain_on_timeout=True)
            if self._cancel_requested:
                return
            if res is None:
                if self._uncertain_child_request is not None:
                    self._go_called = True
                    self._pm_state = None
                    self._t_state = now
                return
            if res.success:
                self._go_called = True
                self._pm_state = None      # bo status cu
                self.get_logger().info(f'[PICKING] {res.message}')
            else:
                self._pick_retry(res.message)
            return

        # Wait pallet_mission terminal state (qua status topic)
        ready, pending_result = self._pending_child_result()
        if not ready:
            return
        if pending_result is not None and not pending_result.success:
            self._pick_retry(pending_result.message)
            return
        if self._pm_state in (DONE, FAILED, STOPPED, 'CARRYING'):
            self._finish_picking(self._pm_state)
        if now - self._t_state > float(self.p['pick_timeout_s']):
            self.get_logger().warn('PICK duration exceeded warning deadline; child operation retained',
                                   throttle_duration_sec=5.0)
            self._t_state = now

    def _pick_retry(self, reason: str):
        self._retries += 1
        self._detect_on_called = False
        self._go_called = False
        self._detector_filter_applied = False
        self._pick_phase = S_SAMPLING
        if self._retries > int(self.p['pick_retries']):
            self._set_state('RECOVERY', f'Detect/approach that bai sau {self._retries} lan: {reason}')
            self._resume_state = PICKING
            return
        self._t_state = time.monotonic()
        self.get_logger().info(f'[PICKING] Detect that bai ({reason}) -> thu '
                               f'{self._retries}/{self.p["pick_retries"]}')

    def _finish_picking(self, pm_state: str):
        if (pm_state == FAILED and self._pm_cargo is not True
                and self._can_auto_retry(self._approach_retries)):
            self._approach_retries += 1
            self.get_logger().warn(
                f'[PICKING] Approach FAILED o pha {self._pm_fail_phase} (cang chua vao pallet) '
                f'-> detect lai va thu lai {self._approach_retries}/{self.p["approach_retries"]}')
            self._pm_state = None
            self._detect_on_called = False
            self._go_called = False
            self._detector_filter_applied = False
            self._pick_phase = S_SAMPLING
            self._t_state = time.monotonic()
            return
        if pm_state == FAILED:
            self._set_state('RECOVERY', 'pallet_mission FAILED; kiem tra tai va vi tri xe truoc Resume')
            self._resume_state = GOTO_D if self._pm_cargo is True else GOTO_PICK
        elif pm_state == STOPPED:
            self._set_state(STOPPED, 'pallet_mission STOPPED')
        elif pm_state == 'CARRYING':
            self._empty_scans = 0
            self._retries = 0
            self._approach_retries = 0
            ctype = self._pm_cargo_type
            sid, frame, why = self._select_drop_slot_for(ctype)
            if sid is None:
                self._set_state('RECOVERY', f'Dang mang pallet loai {ctype or "?"}, khong co o phu hop: {why}')
                self._resume_state = GOTO_D
                return
            self._drop_slot_chosen = (sid, frame)
            drop_mission = self.p['drop_mission']
            self._active_mission, self._active_dest = drop_mission, 'D_1'
            self._triggered, self._near_t0 = False, None
            self._set_state(GOTO_D, f'Da pick pallet {ctype}; di toi D_1, o du kien {sid}')
        else:  # Pick finished without a confirmed CARRYING state.
            self._set_state('RECOVERY',
                            'Pick ket thuc nhung pallet_mission khong bao CARRYING; kiem tra after_pick=hold va tai')
            self._resume_state = None

    # ==================================================================
    # Status (web-ready: JSON duy nhat)
    # ==================================================================
    @_state_locked
    def _publish_status(self):
        self._publish_aruco_gate()
        detector = self._detector_status or {}
        now = time.monotonic()
        nav_fresh = self._nav_status_t > 0.0 and now - self._nav_status_t <= 2.0
        shelf_age = (round(now - self._shelf_last_t, 2)
                     if self._shelf_last_t > 0.0 else None)
        camera_age = (round(now - self._camera_last_t, 2)
                      if self._camera_last_t > 0.0 else None)
        detector_age = (round(now - self._detector_status_t, 2)
                        if self._detector_status_t > 0.0 else None)
        pm_fresh = self._pm_status_t > 0.0 and now - self._pm_status_t <= 1.5
        route_loader = self._route_loader
        route_age = (now - self._route_path_last_received
                     if self._route_path_last_received > 0.0 else float('inf'))
        route_stale = route_age > 6.0
        try:
            route_nodes = route_loader.get_all_node_names() if route_loader else []
        except Exception:
            route_nodes = []
        # AUTO Start does not depend on cargo/fork heartbeat. A retained
        # operation checkpoint, unresolved stop, or active Nav2 goal still blocks.
        can_start = (self._state in (IDLE, DONE, FAILED, STOPPED)
                     and not (self._pm_recovery or {}).get('checkpoint')
                     and not self._discard_requested and not self._tick_inflight
                     and not any(not f.done() for f in self._pending_service_futures)
                     and (self._manager_busy is not True or self._confirmed_terminal_idle())
                     and self._nav_goal_active is not True)
        stop_reason = self._stop_blocked() if self._state in ('STOPPING', 'RECOVERY') else ''
        elapsed = self._elapsed_active_s + (max(0., now - self._active_since) if self._active_since is not None else 0.)
        # Milestone progress for the current pallet cycle. Nav2 route feedback
        # is combined with these phase boundaries in the web UI; Auto has no
        # fixed total number of pallet cycles, so this is per cycle.
        progress_state = (self._resume_state if self._state == 'RECOVERY'
                          and self._resume_state else self._state)
        progress_percent = None
        progress_phase = None
        if self._state == 'STOPPING' and self._resume_state:
            progress_state = self._resume_state
        if progress_state == GOTO_PICK:
            progress_percent, progress_phase = 0.0, 'to_pick'
        elif progress_state == PICKING:
            if self._pick_phase == S_SAMPLING:
                sample_total = max(1, int(self.p['pm_min_samples']))
                sample_ratio = min(1.0, max(0.0, self._pm_samples / sample_total))
                progress_percent = 25.0 + 15.0 * sample_ratio
                progress_phase = 'sampling'
            else:
                progress_percent, progress_phase = 45.0, 'approaching_pallet'
        elif progress_state == GOTO_D:
            progress_percent, progress_phase = 50.0, 'to_dropoff'
        elif progress_state in (DROPPING, WAITING_SLOT):
            progress_percent = 75.0 if progress_state == DROPPING else 80.0
            progress_phase = 'dropping' if progress_state == DROPPING else 'waiting_for_slot'
        elif progress_state == 'RETURN_HOME':
            progress_percent, progress_phase = 90.0, 'returning_home'
        elif self._state == DONE:
            progress_percent, progress_phase = 100.0, 'complete'
        d = {
            'supervisor_session_id': self._supervisor_session_id,
            'run_id': self._run_id,
            'operation_id': self._pm_operation_id,
            'revision': self._revision,
            'resume_revision': self._resume_revision,
            'elapsed_s': round(elapsed, 1),
            'recovery_cause': self._recovery_cause,
            'stop_confirmed': (self._state == 'RECOVERY' and not stop_reason or self._state == IDLE and self._stop_confirmed),
            'cancel_pending': self._discard_requested,
            'stop_reason': stop_reason or getattr(self, '_stop_reason', ''),
            'state': self._state,
            'aruco_detection_enabled': bool(getattr(self, '_aruco_gate_effective', False)),
            'handling_status': self._handling_status(),
            'msg': self._msg,
            'progress_percent': progress_percent,
            'progress_phase': progress_phase,
            'state_elapsed_s': round(now - self._t_state, 1),
            'can_start': can_start,
            'can_stop': not can_start,
            'active_mission': self._active_mission,
            'active_dest': self._active_dest,
            'cycle': {
                'run_mode': self.p.get('run_mode', 'single'),
                'number': self._cycle,
                'placed': self._task_placed if self._mission_queue else self._cycle,
                'target_quantity': (int(self.p.get('target_quantity', 1))
                                    if self.p.get('run_mode') == 'single' else None),
                'target_cargo_class': (self.p.get('target_cargo_class') or 'any'
                                       if self.p.get('run_mode') == 'single' else None),
                'target_drop_slot': (self.p.get('target_drop_slot') or 'auto'
                                     if self.p.get('run_mode') == 'single' else None),
                'task_index': (self._mission_task_index + 1 if self._mission_queue else None),
                'task_count': len(self._mission_queue) if self._mission_queue else None,
                'task_placed': self._task_placed if self._mission_queue else None,
                'pick_mission': self.p['pick_mission'],
                'drop_mission': self.p['drop_mission'],
            },
            'route_graph': {
                'loaded': route_loader is not None and not route_stale,
                'reason': 'stale' if route_stale else '',
                'path': self._route_path,
                'error': 'stale' if route_stale else self._route_path_error,
                'age_s': round(route_age, 2) if self._route_path_last_received > 0.0 else None,
                'nodes': route_nodes,
            },
            'pick_queue': self._pick_queue[self._pick_idx:],
            'waiting_slot': self._state == WAITING_SLOT,
            # Trạng thái 4 ô kệ (A1/A2/B1/B2) từ /shelf/occupancy theo
            # anh xa trong shelf.yaml - cho GUI/web ('unknown' = mat lien
            # lac / cua; 'empty'/'full' = du lieu that)
            'slots': self._shelf_states(),
            # 4 bit thô /shelf/occupancy (None khi chưa nhận được) - GUI
            # hiện dòng riêng để user kiểm tra khi che cảm biến.
            'shelf_raw': self._shelf_input,
            'shelf_bits': self._shelf_bits,
            # Loại hàng đang mang ('A'/'B'/''), o đã chọn cho drop (tu
            # chu trinh goc, giu lai de GUI hien); null = chua chon
            'cargo_type': self._pm_cargo_type,
            'cargo': self._pm_cargo,
            'drop_slot': (self._drop_slot_chosen[0]
                          if getattr(self, '_drop_slot_chosen', None) else None),
            'drop_slot_frame': (self._drop_slot_chosen[1]
                                if getattr(self, '_drop_slot_chosen', None) else None),
            'shelf_message_fresh': self._shelf_ok(),
            'shelf_ok': self._shelf_ok() and self._shelf_bits is not None,
            'shelf_age_s': shelf_age,
            'camera_ok': self._camera_ok(),
            'camera_age_s': camera_age,
            'detector': {
                'state': detector.get('state'),
                'seen_pallets': detector.get('seen_pallets', 0),
                'loaded_pallets': detector.get('loaded_pallets', 0),
                'unlocalized_loaded': detector.get('unlocalized_loaded', 0),
                'compatible_pallets': detector.get('compatible_pallets', 0),
                'unknown_loaded': detector.get('unknown_loaded', 0),
                'incompatible_loaded': detector.get('incompatible_loaded', 0),
                'target_cargo_classes': detector.get('target_cargo_classes', []),
                'fresh': self._detector_status_t > 0.0 and now - self._detector_status_t <= 2.0,
                'age_s': detector_age,
            },
            'pm_state': self._pm_state,
            'pm_handling_status': self._pm_handling_status,
            'pm_waiting_for': self._pm_waiting_for,
            'pm_status_fresh': pm_fresh,
            'pm_status_age_s': (round(now - self._pm_status_t, 2)
                                if self._pm_status_t > 0.0 else None),
            'pm_phase': self._pm_phase,
            'pm_live': self._pm_live,
            'pm_msg': self._pm_msg,
            'pm_err': getattr(self, '_pm_err', None),
            'pm_retries': getattr(self, '_pm_retries', None),
            'pm_fail_phase': self._pm_fail_phase,
            'pm_insertion_only': self._pm_insertion_only,
            'cargo_why': self._pm_cargo_why,
            'pm_samples': self._pm_samples,
            'pm_min_samples': int(self.p['pm_min_samples']),
            'pm_lift_cm': self._pm_lift,
            'nav_status_fresh': nav_fresh,
            'nav_goal_active': self._nav_goal_active if nav_fresh else None,
            'nav_mission': dict(self._mission_nav_status)
                if self._mission_nav_status else None,
            'odom_fresh': self._odom_t > 0.0 and now - self._odom_t <= 0.5,
            'robot_stopped': self._robot_stopped(now),
            'resume_available': self._state == 'RECOVERY' and not self._resume_inflight and not self._resume_blocked(),
            'resume_reason': self._resume_blocked() if self._state == 'RECOVERY' else '',
            'operation_checkpoint': (self._pm_recovery or {}).get('checkpoint'),
            'resume_state': self._resume_state,
            'retries': {
                'pick': self._retries,
                'approach': self._approach_retries,
                'drop': self._drop_retries,
                'detector_restart': self._detector_restarts,
                'empty_scans': self._empty_scans,
            },
        }
        m = String()
        m.data = json.dumps(d)
        self.pub_status.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
