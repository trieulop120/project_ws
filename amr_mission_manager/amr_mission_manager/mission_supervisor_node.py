#!/usr/bin/env python3
"""
mission_supervisor_node.py
==========================
Mission Supervisor (node MỚI - KHÔNG sửa pallet_mission_node / mission_manager).

Demo flow "lấy pallet" (mốc M4 của docs/REPORT_amr_pallet_mission_design.md):

    IDLE -> GOTO_PICK -> PICKING -> DONE
           |             |
           +-- GOTO_D / DROPPING (dự phòng: run_drop:=true, chờ pallet_mission
                có mode drop - mốc M5; hiện tại báo chưa hỗ trợ -> DONE)

    GOTO_PICK : gọi mission_manager /mission_manager/trigger_{pick_mission}
               (BT ComputeRoute theo edge -> fallback set-goal, bên trong
                mission_manager - không cần code lại ở đây).
                Chờ robot TỚI node đích bằng TF map->base (trong ban kinh
                arrive_tol_m, on dinh arrive_settle_s) va den gioi hang.
    PICKING   : gọi các service HIỆN CÓ của pallet_mission, đúng thứ tự:
                1. /pallet_mission/detect true   (bat gom mau ~1-1.5 Hz)
                2. cho `pick_sample_wait_s` (du mau)
                3. /pallet_mission/detect false (chot median -> READY)
                   that bai -> thuchien lai (pick_retries lan)
                4. /pallet_mission/go_to_pallet (align/insert/lift/back/home)
                5. theo /pallet_mission/status cho den DONE/FAILED/STOPPED.

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
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
import tf2_ros

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
    'drop_bin': 'bin_0',          # frame TF cua ke (rack_tf_publisher)
    'run_drop': False,            # True: sau PICKING -> GOTO_D + DROP
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
    'pick_sample_wait_s': 6.0,   # gom mau ~1-1.5 Hz -> 6-9 mau; cua so estimate 4s
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

        # ---------------- TF ----------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ---------------- ROS I/O ----------------
        self.pub_status = self.create_publisher(String, f'/{NODE_NAME}/status', 10)
        self.create_subscription(
            String, f'/{self.p["pallet_mission_node"]}/status', self._on_pm_status, 10)

        self.create_service(
            Trigger, f'/{NODE_NAME}/start', self._srv_start)
        self.create_service(
            Trigger, f'/{NODE_NAME}/stop', self._srv_stop)

        self._cancel_cli = self.create_client(CancelGoal,
                                              '/navigate_to_pose/_action/cancel_goal')

        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_status)

        self.get_logger().info(
            f'Mission Supervisor ready: pick={self.p["pick_mission"]} '
            f'(dest {self.p["pick_dest"]}), drop={self.p["drop_mission"]} '
            f'bin={self.p["drop_bin"]} run_drop={self.p["run_drop"]}')

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
            name = loader.resolve(self.p['pick_dest'])
            pose = loader.get_node_pose(name)
            if pose is None:
                return None
            return (pose.pose.position.x, pose.pose.position.y,
                    yaw_from_quat(pose.pose.orientation))
        except Exception as e:
            self.get_logger().warn(f'Khong doc duoc route graph ({self.p["pick_dest"]}): {e}')
            return None

    def _call_service(self, cli, req) -> bool:
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f'Service {cli.srv_name} chua san sang')
            return False
        fut = cli.call_async(req)
        t_end = time.monotonic() + 5.0
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.02)
        return fut.done()

    def _trigger_mission(self, mission: str) -> bool:
        """Goi 1 nut mission_manager (fire-and-forget trong node do)."""
        svc = f'/{self.p["mission_manager_node"]}/trigger_{mission}'
        cli = self.create_client(Trigger, svc)
        req = Trigger.Request()
        if not self._call_service(cli, req):
            return False
        self.get_logger().info(f'Da goi {svc}')
        return True

    # ==================================================================
    # Services
    # ==================================================================
    def _srv_start(self, req, res):
        if self._state not in (IDLE, DONE, FAILED, STOPPED):
            res.message = f'dang ban ({self._state}), dung /mission/stop truoc'
            res.success = False
            return res
        # Nạp lại các tham số mission (web/GUI/CLI có thể đã set qua
        # ros2 param set mission_supervisor ... TRƯỚC khi gọi /mission/start)
        # - override hot, không cần restart.
        for k in ('pick_mission', 'drop_mission', 'drop_bin',
                  'pick_dest', 'run_drop',
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
        self._set_state(GOTO_PICK, f'Ve {self.p["pick_dest"]} bang {self.p["pick_mission"]} '
                                   f'(edge hoac set-goal)')
        res.success = True
        res.message = 'start'
        return res

    def _srv_stop(self, req, res):
        # Huyen Nav2 (mission_manager) + pallet_mission (xe + lift)
        try:
            if self._cancel_cli.wait_for_service(timeout_sec=2.0):
                self._cancel_cli.call_async(CancelGoal.Request())
        except Exception:
            pass
        cli = self.create_client(Trigger,
                                 f'/{self.p["pallet_mission_node"]}/stop')
        self._call_service(cli, Trigger.Request())
        self._set_state(STOPPED, 'STOP (Nav2 + pallet_mission)')
        res.success = True
        res.message = 'stopped'
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
            params.append(Parameter(dst_key, ptype, v))
        cli = self.create_client(SetParameters, f'/{pm}/set_parameters')
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

    # ==================================================================
    # State machine
    # ==================================================================
    def _tick(self):
        now = time.monotonic()

        if self._state == GOTO_PICK:
            self._do_goto_pick(now)
        elif self._state == PICKING:
            self._do_picking(now)
        # GOTO_D / DROPPING: don duoc goi _enter_drop_khi can (muc M5)

    def _do_goto_pick(self, now):
        if not self._triggered:
            if self._trigger_mission(self.p['pick_mission']):
                self._triggered = True
            if now - self._t_state > 30.0:
                self._set_state(FAILED, 'Service mission_manager khong bao gio du (trigger)')
            return
        # Choi den choi: TF map->base trong ban kinh + on dinh
        dest = self._dest_pose()
        pose = self._base_pose()
        if dest is None or pose is None:
            if now - self._t_state > 10.0:
                self._set_state(FAILED, f'Khong doc duoc pose (dest={dest is not None}, '
                                        f'base={pose is not None})')
            return
        d = math.hypot(pose[0] - dest[0], pose[1] - dest[1])
        if d <= float(self.p['arrive_tol_m']):
            if self._near_t0 is None:
                self._near_t0 = now
            if now - self._near_t0 >= float(self.p['arrive_settle_s']):
                self._near_t0 = None
                self._enter_picking()
        else:
            self._near_t0 = None
        if now - self._t_state > float(self.p['goto_timeout_s']):
            self._set_state(FAILED, f'Ve {self.p["pick_dest"]} qua thoi gian '
                                    f'(con {d * 100:.0f} cm)')

    def _enter_picking(self):
        self._pick_phase = S_SAMPLING
        self._set_state(PICKING, 'Bat dau chu trinh lay pallet')

    def _do_picking(self, now):
        pm = self.p['pallet_mission_node']

        if not self._pm_sampling_applied:
            self._apply_pm_sampling()
            self._pm_sampling_applied = True

        if self._pick_phase == S_SAMPLING:
            if not self._detect_on_called:
                cli = self.create_client(SetBool, f'/{pm}/detect')
                req = SetBool.Request()
                req.data = True
                self._detect_on_called = self._call_service(cli, req)
                return
            if now - self._t_state >= float(self.p['pick_sample_wait_s']):
                # dat -> detect OFF (chot median; success <=> state READY)
                self._t_state = now
                self._detect_on_called = False
                self._go_called = False
                cli = self.create_client(SetBool, f'/{pm}/detect')
                req = SetBool.Request()
                req.data = False
                fut = cli.call_async(req) if cli.wait_for_service(timeout_sec=5.0) else None
                if fut is None:
                    self._pick_retry('Service detect khong du')
                    return
                res = fut.result()
                self.get_logger().info(f'[PICKING] detect OFF: {res.message}')
                if res.success:
                    self._pick_phase = S_APPROACHING
                    self._t_state = now
                else:
                    self._pick_retry(res.message)
            return

        # S_APPROACHING
        if not self._go_called:
            cli = self.create_client(Trigger, f'/{pm}/go_to_pallet')
            fut = cli.call_async(Trigger.Request()) \
                if cli.wait_for_service(timeout_sec=5.0) else None
            if fut is None:
                if now - self._t_state > 30.0:
                    self._set_state(FAILED, 'Service go_to_pallet khong du')
                return
            res = fut.result()
            if res.success:
                self._go_called = True
                self.get_logger().info(f'[PICKING] {res.message}')
            else:
                self._pick_retry(res.message)
            return

        # Wait pallet_mission terminal state (qua status topic)
        if self._pm_state in (DONE, FAILED, STOPPED):
            self._finish_picking(self._pm_state)
        if now - self._t_state > float(self.p['pick_timeout_s']):
            self._set_state(FAILED, f'PICKING qua thoi gian (pm_state={self._pm_state})')

    def _pick_retry(self, reason: str):
        self._retries += 1
        self._detect_on_called = False
        self._go_called = False
        if self._retries > int(self.p['pick_retries']):
            self._set_state(FAILED, f'Detect that {self._retries + 1} lan: {reason}')
            return
        self._pick_phase = S_SAMPLING
        self._t_state = time.monotonic()
        self.get_logger().info(f'[PICKING] Detect that bai ({reason}) -> thu '
                               f'{self._retries}/{self.p["pick_retries"]}')

    def _finish_picking(self, pm_state: str):
        if pm_state == FAILED:
            self._set_state(FAILED, f'pallet_mission FAILED')
        elif pm_state == STOPPED:
            self._set_state(STOPPED, 'pallet_mission STOPPED')
        else:  # DONE
            if self.p['run_drop']:
                # Don duoc: pallet_mission can co mode 'post_lift: hold' + 'drop'
                # (muc M5 report) - hien tai moi chu trinh ket thuc o Home.
                self._set_state(DONE, 'Pick xong (run_drop=true nhung chua co mode '
                                      'drop -> xe ve Home, khong drop)')
            else:
                self._set_state(DONE, f'XONG chu trinh: {self.p["pick_mission"]} + pick')

    # ==================================================================
    # Status (web-ready: JSON duy nhat)
    # ==================================================================
    def _publish_status(self):
        d = {
            'state': self._state,
            'msg': self._msg,
            'cycle': {
                'pick_mission': self.p['pick_mission'],
                'drop_mission': self.p['drop_mission'],
                'drop_bin': self.p['drop_bin'],
                'run_drop': bool(self.p['run_drop']),
            },
            'pm_state': self._pm_state,
            'pm_phase': self._pm_phase,
            'pm_lift_cm': self._pm_lift,
        }
        m = String()
        m.data = json.dumps(d)
        self.pub_status.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MissionSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
