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
from action_msgs.srv import CancelGoal
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String, Int8MultiArray
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool, Trigger
import tf2_ros
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

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

# Sub-phases trong PICKING
S_SAMPLING = 'sampling'
S_APPROACHING = 'approaching'

DEFAULTS = {
    # Mission manager (names must match MISSION_BT_MAP of mission_manager_node)
    'pick_mission': 'home_to_p1',
    'drop_mission': 'p1_to_d1',
    'pick_dest': 'P_1',            # node graph (map frame) de soi den choi
    'run_mode': 'single',          # single: 1 pallet + HOME; auto: loop until source/compatible slots empty
    # W4 (loi 2 ESP serial -> /shelf/occupancy): 4 bit o ke, 1 = co hang;
    # -1 = mat lien lac. Anh xa bit -> o va thiet ke ke: config/shelf.yaml
    # (doc bang shelf_model). KHONG CON drop_bin_fallback: khong ro (mat
    # lien lac / het o tron / chua chon loai hang) -> KHONG dat hang,
    # bao len status/GUI (dung tai cho, khong tu ve Home).
    'shelf_topic': '/shelf/occupancy',
    'shelf_stale_s': 2.0,        # > 2s khong co dat = coi mat lien lac
    'shelf_config': '',          # '' = config/shelf.yaml (mac dinh)
    'camera_topic': '/color/image_raw',
    'camera_stale_s': 2.0,
    'mission_manager_node': 'mission_manager',
    'pallet_mission_node': 'pallet_mission',
    # Frames
    'map_frame': 'map',
    'base_frame': 'base_footprint',
    # Den choi GOTO_PICK
    'arrive_tol_m': 0.30,
    'arrive_settle_s': 1.0,
    'goto_timeout_s': 120.0,
    # PICKING
    'pick_sample_wait_s': 12.0,  # hard timeout; stop earlier after enough stable samples
    'sample_stable_s': 2.0,
    'no_pallet_retries': 3,
    'pick_retries': 2,
    'pick_timeout_s': 300.0,
    # Bo tham so SAMPLING rong (demo 1-1.5 Hz) ap len pallet_mission (HOT,
    # khong sua node cu - chi la parameter service, pallet_mission self._reload_params)
    'pm_final_window_s': 4.0,
    'pm_min_samples': 3,
    'pm_max_pos_std_m': 0.05,
    'pm_max_yaw_std_deg': 5.0,
    'pm_max_sample_age_s': 3.0,
    'pm_require_stable': True,   # nua cuoi cung: false = bo luon kiem tra std
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
        self._cycle = 0
        self._resume_state = None
        self._empty_scans = 0
        self._cancel_requested = False
        self._camera_last_t = 0.0

        # ---------------- TF ----------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---------------- ROS I/O ----------------
        self.pub_status = self.create_publisher(String, f'/{NODE_NAME}/status', 10)
        self.create_subscription(
            String, f'/{self.p["pallet_mission_node"]}/status', self._on_pm_status, 10)
        self.create_subscription(String, '/pallet/target_status',
                                 self._on_detector_status, 10)
        self.create_subscription(Image, self.p['camera_topic'], self._on_camera,
                                 qos_profile_sensor_data)

        # W4 (loi 2 ESP serial -> /shelf/occupancy): 4 bit o ke, 1 = co hang;
        # [-1]*4 = mat ket noi. Doc truc tiep tu esp32_bridge. Anh xa
        # bit -> o (A1/A2/B1/B2) lay tu config/shelf.yaml (shelf_model).
        self._shelf_bits = None          # [0/1,0/1,0/1,0/1] hoac None (stale)
        self._shelf_last_t = 0.0
        self.create_subscription(
            Int8MultiArray, self.p['shelf_topic'], self._on_shelf, 10)

        # Mo hinh ke (doc shelf.yaml 1 lan; anchor + priority + anh xa cam bien)
        self._shelf_model = self._load_shelf_model()

        # Services (khong GUI: GUI chinh la nut goi 2 service nay)
        self.create_service(
            Trigger, f'/{NODE_NAME}/start', self._srv_start,
            callback_group=self._control_group)
        self.create_service(
            Trigger, f'/{NODE_NAME}/stop', self._srv_stop,
            callback_group=self._control_group)
        self.create_service(
            Trigger, f'/{NODE_NAME}/resume', self._srv_resume,
            callback_group=self._control_group)

        self._cancel_cli = self.create_client(CancelGoal,
                                              '/navigate_to_pose/_action/cancel_goal',
                                              callback_group=self._client_group)

        # GIỮ reference timer trên self: executor của rclpy giữ callback bằng
        # weak reference -> không lưu thì timer bị GC ngay sau __init__ và
        # node đứng im IDLE (khong publish status, khong tick duoc).
        self._tick_timer = self.create_timer(0.1, self._tick)
        self._status_timer = self.create_timer(0.5, self._publish_status)

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
            loader = RouteGraphLoader()
            name = loader.resolve(self._active_dest)
            pose = loader.get_node_pose(name)
            if pose is None:
                return None
            return (pose.pose.position.x, pose.pose.position.y,
                    yaw_from_quat(pose.pose.orientation))
        except Exception as e:
            self.get_logger().warn(f'Khong doc duoc route graph ({self._active_dest}): {e}')
            return None

    def _call_service_result(self, cli, req, timeout: float = 5.0):
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'Service {cli.srv_name} chua san sang')
            return None
        fut = cli.call_async(req)
        t_end = time.monotonic() + timeout
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def _call_service(self, cli, req) -> bool:
        return self._call_service_result(cli, req) is not None

    def _trigger_mission(self, mission: str) -> bool:
        """Goi 1 nut mission_manager (fire-and-forget trong node do)."""
        if self._cancel_requested:
            return False
        svc = f'/{self.p["mission_manager_node"]}/trigger_{mission}'
        cli = self.create_client(Trigger, svc, callback_group=self._client_group)
        req = Trigger.Request()
        result = self._call_service_result(cli, req)
        if self._cancel_requested or result is None or not getattr(result, 'success', True):
            return False
        self.get_logger().info(f'Da goi {svc}')
        return True

    # ==================================================================
    # Services
    # ==================================================================
    def _srv_start(self, req, res):
        requested_mode = self.get_parameter('run_mode').value
        if requested_mode not in ('single', 'auto'):
            res.success = False
            res.message = "run_mode phai la 'single' hoac 'auto'"
            return res
        if self._state not in (IDLE, DONE, FAILED, STOPPED):
            res.message = f'dang ban ({self._state}), dung /mission/stop truoc'
            res.success = False
            return res
        # Nạp lại các tham số mission (web/GUI/CLI có thể đã set qua
        # ros2 param set mission_supervisor ... TRƯỚC khi gọi /mission/start)
        # - override hot, không cần restart.
        for k in ('pick_mission', 'drop_mission', 'pick_dest', 'run_mode',
                  'pm_final_window_s', 'pm_min_samples', 'pm_max_pos_std_m',
                  'pm_max_yaw_std_deg', 'pm_max_sample_age_s',
                  'pm_require_stable'):
            try:
                self.p[k] = self.get_parameter(k).value
            except Exception:
                pass

        self._pick_phase = S_SAMPLING
        self._detect_on_called = False
        self._go_called = False
        self._retries = 0
        self._near_t0 = None
        self._triggered = False
        self._pm_state = None
        self._pm_sampling_applied = False
        self._detector_filter_applied = False
        self._drop_slot_chosen = None
        self._active_mission, self._active_dest = self.p['pick_mission'], self.p['pick_dest']
        self._cycle = 0
        self._empty_scans = 0
        self._resume_state = None
        self._cancel_requested = False
        self._set_state(GOTO_PICK, f'Mode {self.p["run_mode"]}: ve {self.p["pick_dest"]} bang {self.p["pick_mission"]} '
                                   f'(edge hoac set-goal)')
        res.success = True
        res.message = 'start'
        return res

    def _srv_stop(self, req, res):
        # Huyen Nav2 (mission_manager) + pallet_mission (xe + lift)
        self._cancel_requested = True
        try:
            if self._cancel_cli.wait_for_service(timeout_sec=2.0):
                self._cancel_cli.call_async(CancelGoal.Request())
        except Exception:
            pass
        cli = self.create_client(Trigger,
                                 f'/{self.p["pallet_mission_node"]}/stop',
                                 callback_group=self._client_group)
        self._call_service(cli, Trigger.Request())
        self._set_state(STOPPED, 'STOP (Nav2 + pallet_mission)')
        res.success = True
        res.message = 'stopped'
        return res

    def _srv_resume(self, req, res):
        if self._state != 'RECOVERY' or not self._resume_state:
            res.success, res.message = False, 'Khong co checkpoint de tiep tuc'
            return res
        state = self._resume_state
        self._resume_state = None
        self._triggered = False
        self._go_called = False
        self._set_state(state, 'Resume tu checkpoint; da kiem tra an toan truoc khi bam')
        res.success, res.message = True, 'resumed'
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
        }
        params = []
        for src_key, (dst_key, ptype) in mapping.items():
            v = self.get_parameter(src_key).value
            params.append(Parameter(dst_key, ptype, v).to_parameter_msg())
        cli = self.create_client(SetParameters, f'/{pm}/set_parameters', callback_group=self._client_group)
        if not cli.wait_for_service(timeout_sec=5.0):
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

        get_cli = self.create_client(GetParameters, f'/{pm}/get_parameters',
                                     callback_group=self._client_group)
        if not get_cli.wait_for_service(timeout_sec=3.0):
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
            args = list(get_future.result().values[0].string_array_value.values)
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

        set_cli = self.create_client(SetParameters, f'/{pm}/set_parameters',
                                     callback_group=self._client_group)
        if not set_cli.wait_for_service(timeout_sec=3.0):
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
        self._pm_status_t = time.monotonic()

    def _on_camera(self, _msg: Image):
        self._camera_last_t = time.monotonic()

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

    def _start_return_home(self, why):
        self._active_mission, self._active_dest = 'return_home', 'NODE_HOME'
        self._triggered, self._near_t0 = False, None
        self._set_state('RETURN_HOME', why + ' -> tu dong ve HOME')

    def _do_goto_d(self, now):
        if not self._shelf_ok() or self._shelf_bits is None:
            try:
                self._cancel_cli.call_async(CancelGoal.Request())
            except Exception:
                pass
            self._set_state('RECOVERY', 'Du lieu cam bien ke stale/mat ket noi tren duong toi D_1')
            self._resume_state = GOTO_D
            return
        sid, frame, why = self._select_drop_slot_for(self._pm_cargo_type)
        if sid is None:
            try:
                self._cancel_cli.call_async(CancelGoal.Request())
            except Exception:
                pass
            self._start_return_home('Khong con o ke phu hop tren duong toi D_1: ' + why)
            return
        self._drop_slot_chosen = (sid, frame)
        if not self._triggered:
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
            self._near_t0 = self._near_t0 or now
            if now - self._near_t0 >= float(self.p['arrive_settle_s']):
                if not self._shelf_ok() or self._shelf_bits is None:
                    self._set_state('RECOVERY', 'Mat ket noi/cam bien ke stale khi den D_1')
                    self._resume_state = GOTO_D
                else:
                    sid, frame, why = self._select_drop_slot_for(self._pm_cargo_type)
                    if sid is None:
                        self._start_return_home('Khong con o phu hop: ' + why)
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
            if not self._shelf_ok() or self._shelf_bits is None:
                self._set_state('RECOVERY', 'Du lieu ke stale/mat ket noi truoc khi bat dau drop')
                self._resume_state = DROPPING
                return
            sid, frame, why = self._select_drop_slot_for(self._pm_cargo_type)
            if sid is None:
                self._start_return_home('O ke khong con phu hop: ' + why)
                return
            # Re-select while travelling; pass the selected slot through the
            # existing pallet_mission parameter/service contract.
            self._drop_slot_chosen = (sid, frame)
            cli = self.create_client(SetParameters, f'/{pm}/set_parameters', callback_group=self._client_group)
            if not cli.wait_for_service(timeout_sec=1.0):
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
            drop = self.create_client(Trigger, f'/{pm}/drop', callback_group=self._client_group)
            if not drop.wait_for_service(timeout_sec=1.0):
                self._set_state('RECOVERY', 'Pallet mission drop service chua san sang')
                self._resume_state = DROPPING
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
                self._set_state('RECOVERY', 'Drop chua khoi dong; can kiem tra pallet/lift')
                self._resume_state = DROPPING
                return
            self._go_called = True
            self._t_state = now
            return
        if self._pm_state == DONE and self._pm_cargo is False:
            self._cycle += 1
            self._go_called = False
            if self.p.get('run_mode') == 'auto' and (not self._shelf_ok() or self._shelf_bits is None):
                self._start_return_home('Da dat pallet; cam bien ke mat ket noi nen dung Auto')
            elif self.p.get('run_mode') == 'auto' and self._any_slot_available():
                self._active_mission, self._active_dest = 'd1_to_p1', self.p['pick_dest']
                self._triggered, self._near_t0 = False, None
                self._set_state(GOTO_PICK, f'Auto: da dat xong pallet {self._cycle}, quay lai P_1')
            else:
                self._start_return_home('Hoan tat chu ky' if self.p.get('run_mode') != 'auto'
                                         else 'Ke da day')
        elif self._pm_state == FAILED:
            self._set_state('RECOVERY', 'Drop FAILED; dung tai checkpoint, kiem tra vi tri pallet')
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
        dest, pose = self._dest_pose(), self._base_pose()
        if dest is None or pose is None:
            if now - self._t_state > 10.0:
                self._set_state('RECOVERY', 'Khong doc duoc pose HOME/robot de xac nhan ve nha')
                self._resume_state = 'RETURN_HOME'
            return
        if math.hypot(pose[0] - dest[0], pose[1] - dest[1]) <= float(self.p['arrive_tol_m']):
            self._near_t0 = self._near_t0 or now
            if now - self._near_t0 >= float(self.p['arrive_settle_s']):
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
        elif self._state == 'RETURN_HOME':
            self._do_return_home(now)

    def _do_goto_pick(self, now):
        if not self._shelf_ok() or self._shelf_bits is None:
            try:
                self._cancel_cli.call_async(CancelGoal.Request())
            except Exception:
                pass
            self._set_state('RECOVERY', 'Du lieu cam bien ke stale/mat ket noi truoc khi di pick')
            self._resume_state = GOTO_PICK
            return
        if not self._any_slot_available():
            self._start_return_home('Khong con o ke trong')
            return
        if not self._triggered:
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
            if self._near_t0 is None:
                self._near_t0 = now
            if now - self._near_t0 >= float(self.p['arrive_settle_s']):
                self._near_t0 = None
                if not self._shelf_ok() or self._shelf_bits is None:
                    self._set_state('RECOVERY', 'Du lieu cam bien ke stale/mat ket noi tai P_1')
                    self._resume_state = GOTO_PICK
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
                cli = self.create_client(SetBool, f'/{pm}/detect', callback_group=self._client_group)
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
                elif not self._camera_ok():
                    self._set_state('RECOVERY', 'Detect service tu choi va camera stale/mat ket noi')
                    self._resume_state = PICKING
                return
            enough = self._pm_samples >= int(self.p['pm_min_samples'])
            stable = self._sample_window_stable()
            if (enough and stable) or now - self._t_state >= float(self.p['pick_sample_wait_s']):
                # dat -> detect OFF (chot median; success <=> state READY)
                self._t_state = now
                self._detect_on_called = False
                self._go_called = False
                cli = self.create_client(SetBool, f'/{pm}/detect', callback_group=self._client_group)
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
                        self._start_return_home(f'P_1 khong detect duoc pallet sau {self._empty_scans} lan quet')
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
                self._start_return_home(f'Khong co o phu hop cho pallet {self._pm_cargo_type or "?"}: {why}')
                return
            self._drop_slot_chosen = (sid, frame)
            cli = self.create_client(Trigger, f'/{pm}/go_to_pallet', callback_group=self._client_group)
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
        if pm_state == FAILED:
            self._set_state('RECOVERY', 'pallet_mission FAILED; kiem tra tai va vi tri xe truoc Resume')
            self._resume_state = GOTO_D if self._pm_cargo is True else GOTO_PICK
        elif pm_state == STOPPED:
            self._set_state(STOPPED, 'pallet_mission STOPPED')
        elif pm_state == 'CARRYING':
            self._empty_scans = 0
            ctype = self._pm_cargo_type
            sid, frame, why = self._select_drop_slot_for(ctype)
            if sid is None:
                self._set_state('RECOVERY', f'Dang mang pallet loai {ctype or "?"}, khong co o phu hop: {why}')
                self._resume_state = GOTO_D
                return
            self._drop_slot_chosen = (sid, frame)
            self._active_mission, self._active_dest = self.p['drop_mission'], 'D_1'
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
        d = {
            'state': self._state,
            'msg': self._msg,
            'cycle': {
                'run_mode': self.p.get('run_mode', 'single'),
                'number': self._cycle,
                'pick_mission': self.p['pick_mission'],
                'drop_mission': self.p['drop_mission'],
            },
            # Trạng thái 4 ô kệ (A1/A2/B1/B2) từ /shelf/occupancy theo
            # anh xa trong shelf.yaml - cho GUI/web ('unknown' = mat lien
            # lac / cua; 'empty'/'full' = du lieu that)
            'slots': self._shelf_states(),
            # 4 bit thô /shelf/occupancy (None khi chưa nhận được) - GUI
            # hiện dòng riêng để user kiểm tra khi che cảm biến.
            'shelf_raw': self._shelf_bits,
            # Loại hàng đang mang ('A'/'B'/''), o đã chọn cho drop (tu
            # chu trinh goc, giu lai de GUI hien); null = chua chon
            'cargo_type': self._pm_cargo_type,
            'cargo': self._pm_cargo,
            'drop_slot': (self._drop_slot_chosen[0]
                          if getattr(self, '_drop_slot_chosen', None) else None),
            'drop_slot_frame': (self._drop_slot_chosen[1]
                                if getattr(self, '_drop_slot_chosen', None) else None),
            'shelf_ok': self._shelf_ok(),
            'camera_ok': self._camera_ok(),
            'detector': {
                'state': detector.get('state'),
                'seen_pallets': detector.get('seen_pallets', 0),
                'loaded_pallets': detector.get('loaded_pallets', 0),
                'unlocalized_loaded': detector.get('unlocalized_loaded', 0),
                'compatible_pallets': detector.get('compatible_pallets', 0),
                'unknown_loaded': detector.get('unknown_loaded', 0),
                'incompatible_loaded': detector.get('incompatible_loaded', 0),
                'target_cargo_classes': detector.get('target_cargo_classes', []),
                'fresh': (self._detector_status_t > 0.0 and
                          time.monotonic() - self._detector_status_t <= 2.0),
            },
            'pm_state': self._pm_state,
            'pm_phase': self._pm_phase,
            'pm_samples': self._pm_samples,
            'pm_min_samples': int(self.p['pm_min_samples']),
            'pm_lift_cm': self._pm_lift,
            'resume_available': self._state == 'RECOVERY' and bool(self._resume_state),
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
