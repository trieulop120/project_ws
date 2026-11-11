#!/usr/bin/env python3
"""
mission_supervisor_node.py
==========================
Mission Supervisor: integrates existing mission_manager and pallet_mission services.

    single: HOME -> P_1 -> adaptive detect -> pick -> D_1 -> drop -> HOME -> DONE
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
    ros2 param set /mission_supervisor pick_mission home_to_p2
    ros2 service call /mission/stop std_srvs/srv/Trigger '{}'
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String, Int8MultiArray
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty, SetBool, Trigger
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (QoSProfile, DurabilityPolicy, ReliabilityPolicy,
                       qos_profile_sensor_data)

from amr_mission_manager.shelf_model import ShelfModel, UNKNOWN, EMPTY, FULL

NODE_NAME = 'mission_supervisor'

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
    'pick_dest': 'P_1',            # node graph (map frame) de soi den choi
    # Nhanh lay thu hai: chi duoc dung khi graph THAT co edge noi P_2 voi
    # node nao do (xem _p2_available). P_1 het pallet -> di P_1 -> P_2.
    'pick_dest_2': 'P_2',
    'pick_mission_2_from_p1': 'p1_to_p2',
    'run_mode': 'single',          # single: 1 pallet + HOME; auto: loop until source/compatible slots empty
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
    'pick_sample_wait_s': 20.0,  # hard timeout (tinh tu Detect ON, gom ca thoi gian detector nap model)
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
        # Hang doi diem lay trong mot chu ky: ['P_1'] hoac ['P_1','P_2'].
        # P_2 chi duoc them khi graph THAT co edge noi P_2 (_p2_available).
        self._pick_queue = []
        self._pick_idx = 0
        self._route_loader = None
        self._route_path_loaded = None
        self._route_path = None
        self._route_path_error = ''
        self._route_path_last_received = 0.0
        self._cycle = 0
        self._resume_state = None
        self._empty_scans = 0
        self._cancel_requested = False
        self._camera_last_t = 0.0
        self._odom_t = 0.0
        self._odom_linear_speed = None
        self._odom_angular_speed = None
        self._nav_status_t = 0.0
        self._nav_goal_active = None
        # WAITING_SLOT: lan cuoi tham do /shelf/occupancy, va ly do khong co o
        self._last_slot_poll = 0.0
        self._slot_wait_why = ''
        self._slot_operator_notified = False

        # ---------------- TF ----------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---------------- ROS I/O ----------------
        self.pub_status = self.create_publisher(String, f'/{NODE_NAME}/status', 10)
        route_qos = QoSProfile(depth=1)
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        self._route_path_sub = self.create_subscription(
            String, '/navigation/route_yaml_path', self._on_route_path, route_qos)
        self.create_subscription(
            String, f'/{self.p["pallet_mission_node"]}/status', self._on_pm_status, 10)
        self.create_subscription(String, '/pallet/target_status',
                                 self._on_detector_status, 10)
        self.create_subscription(Image, self.p['camera_topic'], self._on_camera,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, self.p['odom_topic'], self._on_odom, 10)
        self.create_subscription(
            GoalStatusArray, self.p['nav_status_topic'], self._on_nav_status, 10)

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

        self.get_logger().info(
            f'Mission Supervisor ready: pick={self.p["pick_mission"]} '
            f'(dest {self.p["pick_dest"]}), drop={self.p["drop_mission"]} '
            f'run_mode={self.p["run_mode"]}')

    # ==================================================================
    # Helpers
    # ==================================================================
    def _set_state(self, state: str, msg: str = ''):
        self._state = state
        self._msg = msg
        self._t_state = time.monotonic()
        self._shelf_bad_t0 = None
        self._stuck_t0 = None
        self._start_refusals = 0
        self.get_logger().info(f'[{state}] {msg}')

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

    def _call_service_result(self, cli, req, timeout: float = 5.0):
        # Availability checks happen from the state-machine timer. Keep them
        # short so a missing service cannot starve STOP/status/timer callbacks.
        if not cli.wait_for_service(timeout_sec=min(max(timeout, 0.0), 0.2)):
            return None
        fut = cli.call_async(req)
        t_end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

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

    def _trigger_mission(self, mission: str) -> bool:
        """Call the acknowledged mission trigger; success means Nav2 accepted its goal."""
        if self._cancel_requested:
            return False
        svc = f'/{self.p["mission_manager_node"]}/trigger_{mission}_ack'
        cli = self._service_client(Trigger, svc)
        req = Trigger.Request()
        result = self._call_service_result(cli, req, timeout=12.0)
        if self._cancel_requested or result is None or not getattr(result, 'success', True):
            return False
        self.get_logger().info(f'Da goi {svc}')
        return True

    # ==================================================================
    # Services
    # ==================================================================
    def _srv_start(self, req, res):
        route_age = (time.monotonic() - self._route_path_last_received
                     if self._route_path_last_received > 0.0 else float('inf'))
        if self._route_loader is None or route_age > 6.0:
            res.success = False
            detail = self._route_path_error or 'route graph chưa được nạp'
            res.message = f'Từ chối start: {detail}; route status age={route_age:.1f}s'
            return res
        requested_mode = self.get_parameter('run_mode').value
        if requested_mode not in ('single', 'auto'):
            res.success = False
            res.message = "run_mode phai la 'single' hoac 'auto'"
            return res
        if self._pm_cargo is True:
            res.success = False
            res.message = ('Dang mang pallet; khong the bat dau chu ky pick moi. '
                           'Dung /mission/resume sau khi kiem tra vi tri va tai.')
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
                  'pick_dest_2', 'pick_mission_2_from_p1',
                  'pm_final_window_s', 'pm_min_samples', 'pm_max_pos_std_m',
                  'pm_max_yaw_std_deg', 'pm_max_sample_age_s',
                  'pm_require_stable'):
            try:
                self.p[k] = self.get_parameter(k).value
            except Exception:
                pass

        # Canh bao nhat quan: `pick_dest` va `pick_mission` phai tro cung mot
        # diem. Khong tu sua (nguoi van hanh can thay), chi bao ro.
        if self.p['pick_mission'] == 'home_to_p2' and self.p['pick_dest'] != 'P_2':
            self.get_logger().warn(
                f'[mission] pick_mission={self.p["pick_mission"]} nhung pick_dest='
                f'{self.p["pick_dest"]} -> robot se toi {self.p["pick_dest"]} nhung '
                f'mission lai dich sang P_2. Sua pick_dest cho khop.')
        if self.p['pick_mission'] == 'home_to_p1' and self.p['pick_dest'] != 'P_1':
            self.get_logger().warn(
                f'[mission] pick_mission={self.p["pick_mission"]} nhung pick_dest='
                f'{self.p["pick_dest"]} -> khong khop. Sua pick_dest cho khop.')

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

    def _srv_stop(self, req, res):
        if self._state in (IDLE, DONE, FAILED, STOPPED):
            res.success, res.message = True, 'không có gì để dừng'
            return res
        # Huyen Nav2 (mission_manager) + pallet_mission (xe + lift)
        checkpoint = self._resume_state
        if checkpoint is None and self._state not in ('IDLE', STOPPED, 'RECOVERY', 'DONE', 'FAILED'):
            checkpoint = self._state
        self._cancel_requested = True
        try:
            if self._cancel_cli.wait_for_service(timeout_sec=0.2):
                self._cancel_cli.call_async(CancelGoal.Request())
        except Exception:
            pass
        cli = self._service_client(
            Trigger, f'/{self.p["pallet_mission_node"]}/stop')
        self._call_service_result(cli, Trigger.Request(), timeout=1.0)
        # A manual operator cancel is recoverable regardless of cargo state.
        # Preserve the active phase, queue, retries, and selected shelf slot.
        self._resume_state = checkpoint if checkpoint not in ('IDLE', STOPPED, 'DONE', 'FAILED') else None
        self._set_state('RECOVERY', 'OPERATOR_CANCEL: da dung xe/lift; kiem tra an toan truoc khi RESUME')
        res.success = True
        res.message = 'OPERATOR_CANCEL; RECOVERY'
        return res

    def _srv_resume(self, req, res):
        if self._state != 'RECOVERY' or not self._resume_state:
            res.success, res.message = False, 'Khong co checkpoint de tiep tuc'
            return res
        state = self._resume_state
        self._resume_state = None
        self._approach_retries = 0
        self._drop_retries = 0
        self._detector_restarts = 0
        self._triggered = False
        self._go_called = False
        self._cancel_requested = False
        self._set_state(state, 'Resume tu checkpoint; da kiem tra an toan truoc khi bam')
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
        res.success, res.message = True, 'Mission reset; physical cargo/lift state preserved'
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
        if not classes:
            self.get_logger().warn('[detector] no compatible shelf class is available')
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
        self._pm_state = d.get('state')
        self._pm_phase = d.get('phase')
        self._pm_lift = d.get('lift_cm')
        self._pm_samples = int(d.get('samples') or 0)
        self._pm_live = d.get('live')
        self._pm_cargo_type = d.get('cargo_type')
        self._pm_cargo_why = d.get('cargo_why') or ''
        self._pm_cargo = d.get('cargo')
        self._pm_fail_phase = d.get('fail_phase')
        self._pm_msg = d.get('msg') or ''
        self._pm_status_t = time.monotonic()

    def _on_camera(self, _msg: Image):
        self._camera_last_t = time.monotonic()

    def _on_odom(self, msg: Odometry):
        twist = msg.twist.twist
        self._odom_linear_speed = math.hypot(twist.linear.x, twist.linear.y)
        self._odom_angular_speed = abs(twist.angular.z)
        self._odom_t = time.monotonic()

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
        return (self._nav_status_t > 0.0 and now - self._nav_status_t <= 2.0
                and self._nav_goal_active is False)

    def _on_nav_status(self, msg: GoalStatusArray):
        # GoalStatus constants: ACCEPTED=1, EXECUTING=2, CANCELING=3.
        self._nav_goal_active = any(s.status in (1, 2, 3) for s in msg.status_list)
        self._nav_status_t = time.monotonic()

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
    SAFE_RETRY_PHASES = ('RETREAT', 'ALIGN', 'SIDE_TURN', 'SIDE_DRIVE',
                         'CONVERGE', 'GATE', 'DETECT_DRIFT')

    def _drop_refused(self, now, why: str):
        """pallet_mission tu choi Drop (xe KHONG chuyen dong): thu lai vai lan roi moi RECOVERY,
        va hien dung ly do (truoc day ly do bi bo, chi con 'can kiem tra pallet/lift')."""
        self._start_refusals += 1
        if self._start_refusals <= int(self.p['start_retries']):
            self.get_logger().warn(f'[DROPPING] {why} -> thu lai {self._start_refusals}/{self.p["start_retries"]}')
            self._retry_wait_until = now + 2.0
            return
        self._set_state('RECOVERY', why)
        self._resume_state = DROPPING

    def _can_auto_retry(self, used: int) -> bool:
        return (used < int(self.p['approach_retries'])
                and self._pm_fail_phase in self.SAFE_RETRY_PHASES
                and not self._cancel_requested)

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

    def _select_drop_slot_for(self, cargo_type):
        if self._shelf_model is None:
            return None, None, 'Khong doc duoc shelf.yaml'
        states = self._shelf_states()
        sid, why = self._shelf_model.select_drop_slot(cargo_type, states)
        if sid is None:
            return None, None, why
        return sid, self._shelf_model._slot_by_id[sid].tf_frame, ''

    # ==================================================================
    # Nhanh lay (P_1 -> P_2) - chi bat khi graph THAT co edge
    # ==================================================================
    def _p2_available(self) -> bool:
        """P_2 co dung duoc khong? Chi khi graph dang chay THAT co edge noi P_2.

        P_2 la diem lay thu hai. Khi graph khong noi P_2 voi node nao (node
        bi bo troi trong YAML, hoac graph hien tai khong co canh P_2) thi
        P_2 KHONG duoc dung -> nhanh nay tat hoan toan, robot chi lay o P_1.

        Doc lai moi lan quyet dinh (khong cache) de sua graph roi restart lai
    """
        loader = self._route_loader
        if loader is None:
            return False
        try:
            name = loader.resolve(self.p['pick_dest_2'])
            if not name:
                return False
            return bool(loader.get_connected_nodes(name))
        except Exception as e:
            self.get_logger().warn(f'[pick] khong kiem tra duoc edge cua '
                                   f'{self.p["pick_dest_2"]}: {e}')
            return False

    def _build_pick_queue(self) -> list:
        """Hang doi diem lay cho chu ky hien tai.

        Luon bat dau tu `pick_dest` (mac dinh P_1). Neu P_2 co edge thi them
        P_2 vao CUOI hang doi -> khi P_1 het pallet, robot moi di sang P_2.
        """
        queue = [self.p['pick_dest']]
        if self._p2_available() and self.p['pick_dest_2'] not in queue:
            queue.append(self.p['pick_dest_2'])
        return queue

    def _current_pick_dest(self):
        """Diem lay dang chay (theo hang doi)."""
        if not self._pick_queue:
            return self.p['pick_dest']
        idx = min(self._pick_idx, len(self._pick_queue) - 1)
        return self._pick_queue[idx]

    def _next_pick_dest(self):
        """Diem lay tiep theo trong hang doi, hoac None neu het."""
        nxt = self._pick_idx + 1
        if nxt < len(self._pick_queue):
            return self._pick_queue[nxt]
        return None

    def _mission_for_pick_dest(self, dest: str) -> str:
        """Mission de di toi `dest`.

        - Diem dau tien cua chu ky (P_1): dung `pick_mission` (HOME -> P_1).
        - Diem thu hai (P_2) khi dang o P_1: dung `pick_mission_2_from_p1`
          (P_1 -> P_2) de khong phai quay ve HOME.
        - Diem thu hai khi bat dau chu ky moi (tu D_1): dung `d1_to_p1` /
          `d1_to_p2` neu co, nguoc lai `pick_mission`.
        """
        if dest == self.p['pick_dest']:
            return self.p['pick_mission']
        # Diem thu hai.
        if self._pick_idx > 0:
            return self.p['pick_mission_2_from_p1']
        return self._mission_from_d1(dest)

    @staticmethod
    def _mission_from_d1(dest: str) -> str:
        """Return the registered mission ID for traveling from D_1 to a pick node."""
        # Mission service IDs omit the underscore in the numbered node name:
        # mission_manager registers d1_to_p1 / d1_to_p2, while graph nodes are
        # named P_1 / P_2. Do not derive the service ID with lower() alone.
        d1_missions = {'P_1': 'd1_to_p1', 'P_2': 'd1_to_p2'}
        return d1_missions.get(dest, '')

    def _start_return_home(self, why):
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
                self._cancel_cli.call_async(CancelGoal.Request())
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
                    self._cancel_cli.call_async(CancelGoal.Request())
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
                self._triggered = True
                return
            if self._trigger_mission(self.p['drop_mission']):
                self._triggered = True
            elif now - self._t_state > 30.0:
                self._set_state('RECOVERY', 'Khong goi duoc mission toi D_1')
                self._resume_state = GOTO_D
            return
        self._active_dest = 'D_1'
        dest, pose = self._dest_pose(), self._base_pose()
        if dest is None or pose is None:
            if now - self._t_state > 10.0:
                try:
                    self._cancel_cli.call_async(CancelGoal.Request())
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
                            self._cancel_cli.call_async(CancelGoal.Request())
                        except Exception:
                            pass
                        self._stuck_t0 = None
                else:
                    self._stuck_t0 = None
            else:
                self._stuck_t0 = None
                self._near_t0 = self._near_t0 or now
            if self._near_t0 is not None and now - self._near_t0 >= float(self.p['arrive_settle_s']):
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
                self._cancel_cli.call_async(CancelGoal.Request())
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
                    self._set_state('RECOVERY', 'Du lieu ke stale/mat ket noi truoc khi bat dau drop')
                    self._resume_state = DROPPING
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
                self._set_state('RECOVERY', 'Pallet mission set_parameters chua san sang')
                self._resume_state = DROPPING
                return
            req = SetParameters.Request()
            req.parameters = [
                Parameter('drop_slot', Parameter.Type.STRING, sid).to_parameter_msg(),
                Parameter('cargo_type_override', Parameter.Type.STRING, '').to_parameter_msg(),
            ]
            fut = cli.call_async(req)
            end = time.monotonic() + 2.0
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.01)
            if self._cancel_requested:
                return
            if not fut.done() or not all(x.successful for x in fut.result().results):
                self._set_state('RECOVERY', 'Khong dat duoc drop_slot cho pallet_mission')
                self._resume_state = DROPPING
                return
            drop = self._service_client(Trigger, f'/{pm}/drop')
            if not drop.wait_for_service(timeout_sec=0.2):
                self._drop_refused(now, 'Pallet mission drop service chua san sang')
                return
            if self._cancel_requested:
                return
            fut = drop.call_async(Trigger.Request())
            end = time.monotonic() + 3.0
            while not fut.done() and time.monotonic() < end:
                time.sleep(0.01)
            if self._cancel_requested:
                return
            if not fut.done() or not fut.result().success:
                why = fut.result().message if fut.done() else 'service khong tra loi trong 3s'
                self._drop_refused(now, f'Drop chua khoi dong: {why}')
                return
            self._start_refusals = 0
            self._go_called = True
            self._pm_state = None      # bo status cu (FAILED lan truoc)
            self._t_state = now
            return
        if self._pm_state == DONE and self._pm_cargo is False:
            self._cycle += 1
            self._drop_retries = 0
            self._go_called = False
            if self.p.get('run_mode') == 'auto' and (not self._shelf_ok() or self._shelf_bits is None):
                self._start_return_home('Da dat pallet; cam bien ke mat ket noi nen dung Auto')
            elif self.p.get('run_mode') == 'auto' and self._any_slot_available():
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
        elif now - self._t_state > float(self.p.get('drop_timeout_s', 360.0)):
            self._set_state('RECOVERY', 'Drop timeout; can kiem tra pallet va lift truoc Resume')
            self._resume_state = DROPPING

    def _do_return_home(self, now):
        if not self._triggered:
            if self._trigger_mission('return_home'):
                self._triggered = True
                self._active_dest = 'NODE_HOME'
                self._near_t0 = None
            elif now - self._t_state > 30.0:
                self._set_state('RECOVERY', 'Khong goi duoc return_home')
                self._resume_state = 'RETURN_HOME'
            return
        if now - self._t_state > float(self.p['goto_timeout_s']):
            try:
                self._cancel_cli.call_async(CancelGoal.Request())
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
            self._near_t0 = self._near_t0 or now
            if now - self._near_t0 >= float(self.p['arrive_settle_s']):
                # Ve HOME xong NHUNG van giu pallet: khong co o ke de dat, va
                # /mission/start se bi tu choi (dang mang pallet) -> cycle chet.
                # Vao WAITING_SLOT de xe dung cho o trong (khong phai loi).
                if self._pm_cargo is True:
                    self._enter_waiting_slot(
                        'Da ve HOME nhung van giu pallet; ke chua co o trong de dat')
                else:
                    self._set_state(DONE, f'Ve HOME xong sau {self._cycle} chu ky')

    def _tick(self):
        now = time.monotonic()

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
                self._cancel_cli.call_async(CancelGoal.Request())
            except Exception:
                pass
            self._set_state('RECOVERY', 'Du lieu cam bien ke stale/mat ket noi truoc khi di pick')
            self._resume_state = GOTO_PICK
            return
        if (self._shelf_ok() and self._shelf_bits is not None
                and not self._any_slot_available()):
            self._start_return_home('Khong con o ke trong')
            return
        if not self._triggered:
            d0, p0 = self._dest_pose(), self._base_pose()
            if (d0 is not None and p0 is not None
                    and math.hypot(p0[0] - d0[0], p0[1] - d0[1]) <= float(self.p['arrive_tol_m'])
                    and self._nav_goal_active is not True):
                self.get_logger().info('[GOTO_PICK] Xe da o diem pick -> bo qua gui lai mission Nav2')
                self._triggered = True
                return
            if self._trigger_mission(self._active_mission):
                self._triggered = True
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
                    self._cancel_cli.call_async(CancelGoal.Request())
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
                elif not self._any_slot_available():
                    self._start_return_home('Khong con o ke trong hoac du lieu ke khong hop le')
                else:
                    self._enter_picking()
        else:
            self._near_t0 = None
        if now - self._t_state > float(self.p['goto_timeout_s']):
            try:
                self._cancel_cli.call_async(CancelGoal.Request())
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
                        unknown = int(self._detector_status.get('unknown_loaded', 0))
                        compatible = int(self._detector_status.get('compatible_pallets', 0))
                        unlocalized = int(self._detector_status.get('unlocalized_loaded', 0))
                        if unknown > 0 or compatible > 0 or unlocalized > 0:
                            self._pick_retry(
                                'Detector thay pallet nhung chua the khoa class-compatible target '
                                f'(unknown={unknown}, unlocalized={unlocalized}, compatible={compatible})')
                            return
                        self._empty_scans += 1
                    if self._empty_scans >= int(self.p['no_pallet_retries']):
                        next_dest = self._next_pick_dest()
                        # Kiem tra lai edge ngay tai thoi diem re nhanh; graph
                        # khong con edge thi bo qua P_2 va ket thuc chu ky.
                        if (next_dest == self.p['pick_dest_2']
                                and self._p2_available()):
                            self._pick_idx += 1
                            self._active_dest = next_dest
                            self._active_mission = self.p['pick_mission_2_from_p1']
                            self._triggered, self._near_t0 = False, None
                            self._set_state(
                                GOTO_PICK,
                                f'{self._current_pick_dest()} het pallet -> di toi '
                                f'{next_dest} bang {self._active_mission}')
                        else:
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
            res = self._call_service_result(cli, Trigger.Request())
            if self._cancel_requested:
                return
            if res is None:
                if now - self._t_state > 30.0:
                    self._set_state('RECOVERY', 'Service go_to_pallet khong du')
                    self._resume_state = PICKING
                return
            if res.success:
                self._go_called = True
                self._pm_state = None      # bo status cu
                self.get_logger().info(f'[PICKING] {res.message}')
            else:
                self._pick_retry(res.message)
            return

        # Wait pallet_mission terminal state (qua status topic)
        if self._pm_state in (DONE, FAILED, STOPPED, 'CARRYING'):
            self._finish_picking(self._pm_state)
        if now - self._t_state > float(self.p['pick_timeout_s']):
            self._set_state('RECOVERY', f'PICKING qua thoi gian (pm_state={self._pm_state})')
            self._resume_state = GOTO_D if self._pm_cargo is True else PICKING

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
            drop_mission = ('p2_to_d1'
                            if self._current_pick_dest() == self.p['pick_dest_2']
                            else self.p['drop_mission'])
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
    def _publish_status(self):
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
            p2_node = route_loader.resolve(self.p['pick_dest_2']) if route_loader else None
            p2_connected_nodes = (route_loader.get_connected_nodes(p2_node)
                                  if p2_node else [])
        except Exception:
            route_nodes, p2_connected_nodes = [], []
        can_start = (self._state in (IDLE, DONE, FAILED, STOPPED)
                     and self._pm_cargo is not True)
        # Milestone progress for the current pallet cycle. Nav2 route feedback
        # is combined with these phase boundaries in the web UI; Auto has no
        # fixed total number of pallet cycles, so this is per cycle.
        progress_state = (self._resume_state if self._state == 'RECOVERY'
                          and self._resume_state else self._state)
        progress_percent = None
        progress_phase = None
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
            'state': self._state,
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
                'pick_dest_2': self.p['pick_dest_2'],
                'pick_dest_2_enabled': bool(p2_connected_nodes),
                'pick_dest_2_connected_nodes': p2_connected_nodes,
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
            'pm_status_fresh': pm_fresh,
            'pm_status_age_s': (round(now - self._pm_status_t, 2)
                                if self._pm_status_t > 0.0 else None),
            'pm_phase': self._pm_phase,
            'pm_live': self._pm_live,
            'pm_msg': self._pm_msg,
            'pm_fail_phase': self._pm_fail_phase,
            'cargo_why': self._pm_cargo_why,
            'pm_samples': self._pm_samples,
            'pm_min_samples': int(self.p['pm_min_samples']),
            'pm_lift_cm': self._pm_lift,
            'nav_status_fresh': nav_fresh,
            'nav_goal_active': self._nav_goal_active if nav_fresh else None,
            'odom_fresh': self._odom_t > 0.0 and now - self._odom_t <= 0.5,
            'robot_stopped': self._robot_stopped(now),
            'resume_available': self._state == 'RECOVERY' and bool(self._resume_state),
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
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
