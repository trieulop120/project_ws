#!/usr/bin/env python3
"""
pallet_mission_node.py
======================

Pallet approach mission for amr_mission_manager.

Luong:
    [Detect ON]  -> chay pallet_detector (tien trinh con), gom /pallet/pose, ve marker tren map
    [Detect OFF] -> tat detector, chot x, y, yaw (map) tu cac mau cuoi, tra ve qua service
    [Go to pallet] -> NavigateToPose toi diem cach pallet `standoff_m`, huong ve pallet
                      -> canh lai goc (xoay tai cho neu lech)
                      -> tien thang 0.1 m/s, tu chinh huong + lech ngang
                      -> dung khi sieu am < stop_dist_cm lien tiep `confirm_count` mau
    [STOP]       -> huy Nav2, dung xe, tat detector

Services (std_srvs, giong mission_manager - khong can package interface moi):
    pallet_mission/detect        SetBool  data=true: bat | data=false: tat + chot ket qua
    pallet_mission/go_to_pallet  Trigger
    pallet_mission/stop          Trigger  (dung khan cap)

Topics:
    out  /pallet_mission/status      std_msgs/String  (JSON: state, msg, pallet, pre, us_cm ...)
    out  /pallet_mission/markers     visualization_msgs/MarkerArray   (them display vao RViz)
    out  /pallet_mission/pallet_pose geometry_msgs/PoseStamped (map, latched)
    out  /pallet_mission/pre_pose    geometry_msgs/PoseStamped (map, latched)
    out  /cmd_vel                    geometry_msgs/Twist (CHI khi ALIGNING / INSERTING)
    in   /pallet/pose                geometry_msgs/PoseStamped (tu detector, frame map)
    in   /ultrasonic/range           sensor_msgs/Range (tu esp32_bridge)

Quy uoc: yaw cua pallet = huong +X cua pallet (phap tuyen mat truoc, huong VAO pallet),
nen xe quay mat vao pallet khi yaw_xe == yaw_pallet.
"""

import json
import math
import os
import shutil
import signal
import subprocess
import time
from collections import deque
from typing import Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Range
from std_msgs.msg import String, ColorRGBA
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


NODE_NAME = 'pallet_mission'

# States
IDLE = 'IDLE'
DETECTING = 'DETECTING'
READY = 'READY'              # da co ket qua hop le, san sang Go to pallet
NAVIGATING = 'NAVIGATING'
SETTLING = 'SETTLING'        # cho Nav2 ngung publish cmd_vel
ALIGNING = 'ALIGNING'
INSERTING = 'INSERTING'
DONE = 'DONE'
FAILED = 'FAILED'
STOPPED = 'STOPPED'

BUSY_STATES = (NAVIGATING, SETTLING, ALIGNING, INSERTING)


# ----------------------------------------------------------------------
# Pure helpers (khong phu thuoc ROS -> de test)
# ----------------------------------------------------------------------
def norm_angle(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def estimate_pallet(samples, now, window_s, min_samples, max_age_s,
                    max_pos_std_m, max_yaw_std_deg):
    """Chot x, y, yaw tu cac mau (t, x, y, yaw).

    Return (dict | None, reason). dict luon co 'stable'.
    """
    n_all = len(samples)
    if n_all < min_samples:
        return None, f'chua du mau ({n_all}/{min_samples})'
    t_last = samples[-1][0]
    if now - t_last > max_age_s:
        return None, f'pallet khong con trong tam nhin (mau cuoi {now - t_last:.1f}s truoc)'
    win = [s for s in samples if s[0] >= t_last - window_s]
    if len(win) < min_samples:
        return None, f'chua du mau trong {window_s:.0f}s cuoi ({len(win)}/{min_samples})'

    arr = np.array([(s[1], s[2], s[3]) for s in win], dtype=np.float64)
    x = float(np.median(arr[:, 0]))
    y = float(np.median(arr[:, 1]))
    yaws = arr[:, 2]
    ref = math.atan2(float(np.mean(np.sin(yaws))), float(np.mean(np.cos(yaws))))
    d = np.array([norm_angle(v - ref) for v in yaws])
    yaw = norm_angle(ref + float(np.mean(d)))
    yaw_std = math.degrees(float(np.std(d)))
    pos_std = math.sqrt(float(np.std(arr[:, 0])) ** 2 + float(np.std(arr[:, 1])) ** 2)
    stable = (pos_std <= max_pos_std_m) and (yaw_std <= max_yaw_std_deg)
    return {
        'x': x, 'y': y, 'yaw': yaw,
        'n': len(win), 'pos_std': pos_std, 'yaw_std_deg': yaw_std,
        'stable': stable,
    }, ''


def approach_pose(x, y, yaw, standoff):
    """Diem cach pallet `standoff` (doc theo phap tuyen), huong xe = yaw pallet."""
    return x - standoff * math.cos(yaw), y - standoff * math.sin(yaw), yaw


def pallet_line_errors(rx, ry, ryaw, px, py, psi):
    """Sai so cua xe so voi duong thang qua (px,py) huong psi.

    e_yaw : psi - yaw_xe                 (>0: xe can quay trai)
    e_lat : lech ngang, >0 = xe nam ben TRAI duong
    along : toa do doc duong (am = chua toi pallet)
    """
    e_yaw = norm_angle(psi - ryaw)
    dx, dy = rx - px, ry - py
    e_lat = -math.sin(psi) * dx + math.cos(psi) * dy
    along = math.cos(psi) * dx + math.sin(psi) * dy
    return e_yaw, e_lat, along


def steer_wz(e_lat, ryaw, psi, k_yaw, k_lat, max_offset_rad, max_wz):
    """Stanley don gian: huong muc tieu = psi - atan(k_lat * e_lat) (gioi han)."""
    off = clamp(math.atan(k_lat * e_lat), -max_offset_rad, max_offset_rad)
    desired = psi - off
    e_h = norm_angle(desired - ryaw)
    return clamp(k_yaw * e_h, -max_wz, max_wz)


def rotate_wz(e_yaw, k_yaw, min_wz, max_wz):
    """Xoay tai cho: ti le theo sai so nhung khong duoi min_wz (tranh ket ma sat)."""
    mag = clamp(k_yaw * abs(e_yaw), min_wz, max_wz)
    return math.copysign(mag, e_yaw)


class PalletMission(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        # ---------------- Parameters ----------------
        P = self.declare_parameter
        # detector
        P('detector_package', 'amr_perception')
        P('detector_executable', 'pallet_detector')
        P('detector_camera_info_in', '/camera/color/camera_info')
        P('detector_camera_info_out', '/color/camera_info')
        P('detector_extra_args', [''])          # vd: ['-p', 'conf_threshold:=0.5']
        # topics / frames
        P('pose_topic', '/pallet/pose')
        P('range_topic', '/ultrasonic/range')
        P('cmd_vel_topic', '/cmd_vel')
        P('map_frame', 'map')
        P('base_frame', 'base_footprint')
        P('insertion_frame', 'odom')            # khung muot de tien vao (khong bi AMCL nhay)
        # chot ket qua
        P('final_window_s', 3.0)
        P('min_samples', 10)
        P('max_sample_age_s', 2.0)
        P('max_pos_std_m', 0.03)
        P('max_yaw_std_deg', 3.0)
        P('require_stable', True)
        # approach
        P('standoff_m', 0.40)                   # tu base_footprint toi diem pallet
        P('nav_bt_package', 'amr_navigation')
        P('nav_bt_file', 'return_home.xml')
        P('settle_s', 1.0)
        P('auto_insert', True)
        # canh chinh + tien vao
        P('insert_speed', 0.10)
        P('stop_dist_cm', 3.0)
        P('confirm_count', 10)
        P('max_insert_m', 1.0)
        P('insert_timeout_s', 30.0)
        P('align_timeout_s', 20.0)
        P('align_tol_deg', 2.0)
        P('realign_deg', 6.0)
        P('k_yaw', 1.5)
        P('k_lat', 1.0)
        P('max_heading_offset_deg', 10.0)
        P('min_wz', 0.20)
        P('max_wz', 0.35)
        P('range_stale_s', 0.5)
        P('tf_stale_s', 0.5)
        # marker
        P('face_width_m', 1.2)

        g = lambda n: self.get_parameter(n).value
        self.p = {n: g(n) for n in [
            'detector_package', 'detector_executable', 'detector_camera_info_in',
            'detector_camera_info_out', 'detector_extra_args', 'pose_topic',
            'range_topic', 'cmd_vel_topic', 'map_frame', 'base_frame',
            'insertion_frame', 'final_window_s', 'min_samples', 'max_sample_age_s',
            'max_pos_std_m', 'max_yaw_std_deg', 'require_stable', 'standoff_m',
            'nav_bt_package', 'nav_bt_file', 'settle_s', 'auto_insert',
            'insert_speed', 'stop_dist_cm', 'confirm_count', 'max_insert_m',
            'insert_timeout_s', 'align_timeout_s', 'align_tol_deg', 'realign_deg',
            'k_yaw', 'k_lat', 'max_heading_offset_deg', 'min_wz', 'max_wz',
            'range_stale_s', 'tf_stale_s', 'face_width_m']}

        # ---------------- State ----------------
        self._state = IDLE
        self._msg = 'San sang'
        self._samples = deque(maxlen=600)
        self._result = None           # dict x,y,yaw,... (map)
        self._pre = None              # (x, y, yaw) map
        self._det_proc = None
        self._det_kill_at = None

        self._goal_handle = None
        self._settle_t0 = 0.0

        self._ins_p = None            # (x, y) trong insertion_frame
        self._ins_psi = 0.0
        self._rotating = False
        self._align_t0 = 0.0
        self._insert_t0 = 0.0
        self._along0 = 0.0

        self._us_cm = None
        self._us_t = 0.0
        self._us_count = 0

        # ---------------- ROS I/O ----------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, f'/{NODE_NAME}/status', 10)
        self.pub_markers = self.create_publisher(MarkerArray, f'/{NODE_NAME}/markers', 10)
        self.pub_pallet = self.create_publisher(PoseStamped, f'/{NODE_NAME}/pallet_pose', latched)
        self.pub_pre = self.create_publisher(PoseStamped, f'/{NODE_NAME}/pre_pose', latched)
        self.pub_cmd = self.create_publisher(Twist, self.p['cmd_vel_topic'], 10)

        self.create_subscription(PoseStamped, self.p['pose_topic'], self._on_pose, 10)
        self.create_subscription(Range, self.p['range_topic'], self._on_range, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_service(SetBool, f'{NODE_NAME}/detect', self._srv_detect)
        self.create_service(Trigger, f'{NODE_NAME}/go_to_pallet', self._srv_go)
        self.create_service(Trigger, f'{NODE_NAME}/stop', self._srv_stop)

        self.create_timer(0.05, self._tick)          # 20 Hz dieu khien
        self.create_timer(0.5, self._slow_tick)      # 2 Hz status + marker

        self.get_logger().info('Pallet Mission ready')
        self._publish_status()

    # ==================================================================
    # State helpers
    # ==================================================================
    def _set_state(self, state: str, msg: str = ''):
        self._state = state
        self._msg = msg
        self.get_logger().info(f'[{state}] {msg}')
        self._publish_status()

    def _publish_status(self):
        pallet = None
        if self._result:
            pallet = {'x': round(self._result['x'], 4), 'y': round(self._result['y'], 4),
                      'yaw_deg': round(math.degrees(self._result['yaw']), 2),
                      'stable': bool(self._result['stable'])}
        pre = None
        if self._pre:
            pre = {'x': round(self._pre[0], 4), 'y': round(self._pre[1], 4),
                   'yaw_deg': round(math.degrees(self._pre[2]), 2)}
        us = None
        if self._us_cm is not None and math.isfinite(self._us_cm):
            us = round(self._us_cm, 1)
        live = None
        if self._state == DETECTING and self._samples:
            s = self._samples[-1]
            live = {'x': round(s[1], 3), 'y': round(s[2], 3),
                    'yaw_deg': round(math.degrees(s[3]), 1)}
        d = {'state': self._state, 'msg': self._msg, 'samples': len(self._samples),
             'live': live, 'pallet': pallet, 'pre': pre, 'us_cm': us,
             'us_count': self._us_count}
        m = String()
        m.data = json.dumps(d)
        self.pub_status.publish(m)

    # ==================================================================
    # Subscriptions
    # ==================================================================
    def _on_pose(self, msg: PoseStamped):
        if self._state != DETECTING:
            return
        if msg.header.frame_id != self.p['map_frame']:
            self.get_logger().warn(
                f"/pallet/pose frame '{msg.header.frame_id}' != '{self.p['map_frame']}'",
                throttle_duration_sec=3.0)
            return
        self._samples.append((time.monotonic(), msg.pose.position.x,
                              msg.pose.position.y, yaw_from_quat(msg.pose.orientation)))

    def _on_range(self, msg: Range):
        cm = msg.range * 100.0 if math.isfinite(msg.range) else float('inf')
        self._us_cm = cm
        self._us_t = time.monotonic()
        lo = msg.min_range * 100.0 if msg.min_range > 0.0 else 0.0
        if math.isfinite(cm) and lo <= cm < self.p['stop_dist_cm']:
            self._us_count += 1
        else:
            self._us_count = 0

    # ==================================================================
    # Services
    # ==================================================================
    def _srv_detect(self, req, res):
        if req.data:
            res.success, res.message = self._start_detect()
        else:
            res.success, res.message = self._stop_detect()
        return res

    def _srv_go(self, req, res):
        res.success, res.message = self._go_to_pallet()
        return res

    def _srv_stop(self, req, res):
        self._emergency_stop()
        res.success, res.message = True, 'STOPPED'
        return res

    # ==================================================================
    # Detector process
    # ==================================================================
    def _detector_alive(self) -> bool:
        return self._det_proc is not None and self._det_proc.poll() is None

    def _start_detector(self) -> Tuple[bool, str]:
        if self._detector_alive():
            return True, 'detector dang chay'
        if shutil.which('ros2') is None:
            return False, "khong tim thay lenh 'ros2' (chua source workspace?)"
        cmd = ['ros2', 'run', self.p['detector_package'], self.p['detector_executable'],
               '--ros-args',
               '-r', f"{self.p['detector_camera_info_in']}:={self.p['detector_camera_info_out']}"]
        extra = [a for a in (self.p['detector_extra_args'] or []) if a]
        cmd += extra
        try:
            self._det_proc = subprocess.Popen(cmd, start_new_session=True)
        except Exception as e:
            self._det_proc = None
            return False, f'khong chay duoc detector: {e}'
        self._det_kill_at = None
        self.get_logger().info('Detector started: ' + ' '.join(cmd))
        return True, 'detector started'

    def _terminate_detector(self):
        """Gui SIGINT toi ca nhom tien trinh; tick se SIGKILL neu qua han."""
        if not self._detector_alive():
            self._det_proc = None
            return
        try:
            os.killpg(os.getpgid(self._det_proc.pid), signal.SIGINT)
        except Exception as e:
            self.get_logger().warn(f'SIGINT detector failed: {e}')
        self._det_kill_at = time.monotonic() + 4.0

    def _reap_detector(self):
        if self._det_proc is None:
            return
        if self._det_proc.poll() is not None:
            self._det_proc = None
            self._det_kill_at = None
        elif self._det_kill_at is not None and time.monotonic() > self._det_kill_at:
            try:
                os.killpg(os.getpgid(self._det_proc.pid), signal.SIGKILL)
            except Exception:
                pass
            self._det_kill_at = time.monotonic() + 2.0

    # ==================================================================
    # Detect on / off
    # ==================================================================
    def _start_detect(self) -> Tuple[bool, str]:
        if self._state in BUSY_STATES:
            return False, f'dang ban ({self._state}), khong the detect'
        if self._state == DETECTING:
            return True, 'dang detect'
        ok, why = self._start_detector()
        if not ok:
            self._set_state(FAILED, why)
            return False, why
        self._samples.clear()
        self._result = None
        self._pre = None
        self._set_state(DETECTING, 'Dang detect... (cho detector nap model)')
        return True, 'detect ON'

    def _stop_detect(self) -> Tuple[bool, str]:
        if self._state != DETECTING:
            return False, 'khong o trang thai detect'
        self._terminate_detector()
        est, why = estimate_pallet(
            list(self._samples), time.monotonic(),
            self.p['final_window_s'], int(self.p['min_samples']),
            self.p['max_sample_age_s'], self.p['max_pos_std_m'],
            self.p['max_yaw_std_deg'])
        if est is None:
            self._set_state(IDLE, f'Detect that bai: {why}')
            return False, f'Detect that bai: {why}'

        txt = (f"x={est['x']:.3f} y={est['y']:.3f} yaw={math.degrees(est['yaw']):.1f}deg "
               f"(n={est['n']}, std xy={est['pos_std'] * 100:.1f}cm, "
               f"yaw={est['yaw_std_deg']:.1f}deg)")
        usable = est['stable'] or not self.p['require_stable']
        self._result = est
        px, py, pyaw = approach_pose(est['x'], est['y'], est['yaw'], self.p['standoff_m'])
        self._pre = (px, py, pyaw)
        self._publish_pose(self.pub_pallet, est['x'], est['y'], est['yaw'])
        self._publish_pose(self.pub_pre, px, py, pyaw)
        if usable:
            self._set_state(READY, txt)
            return True, txt
        self._set_state(IDLE, 'KHONG ON DINH: ' + txt)
        return False, 'KHONG ON DINH (detect lai): ' + txt

    # ==================================================================
    # Go to pallet (Nav2)
    # ==================================================================
    def _go_to_pallet(self) -> Tuple[bool, str]:
        if self._state != READY or self._pre is None:
            return False, f'chua co ket qua hop le (state={self._state})'
        if not self._nav.server_is_ready():
            return False, 'NavigateToPose server chua san sang'

        px, py, pyaw = self._pre
        pose = PoseStamped()
        pose.header.frame_id = self.p['map_frame']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = px, py
        pose.pose.orientation.z = math.sin(pyaw / 2.0)
        pose.pose.orientation.w = math.cos(pyaw / 2.0)

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = self._bt_path()

        fut = self._nav.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_sent)
        self._set_state(NAVIGATING,
                        f'Dang toi diem truoc pallet ({px:.2f}, {py:.2f}, '
                        f'{math.degrees(pyaw):.0f}deg)')
        return True, 'dang toi pallet'

    def _bt_path(self) -> str:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory(self.p['nav_bt_package']),
                                'config', 'bt', self.p['nav_bt_file'])
            if os.path.exists(path):
                return path
            self.get_logger().warn(f'BT khong ton tai: {path} -> dung BT mac dinh cua Nav2')
        except Exception as e:
            self.get_logger().warn(f'Khong lay duoc BT path: {e} -> dung BT mac dinh')
        return ''

    def _on_goal_sent(self, future):
        try:
            handle = future.result()
        except Exception as e:
            if self._state == NAVIGATING:
                self._set_state(FAILED, f'Loi gui goal: {e}')
            return
        if self._state != NAVIGATING:        # da bi STOP trong luc cho
            if handle and handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle or not handle.accepted:
            self._set_state(FAILED, 'Nav2 tu choi goal')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        self._goal_handle = None
        if self._state != NAVIGATING:
            return
        try:
            status = future.result().status
        except Exception as e:
            self._set_state(FAILED, f'Loi ket qua Nav2: {e}')
            return
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._settle_t0 = time.monotonic()
            self._set_state(SETTLING, 'Da toi diem truoc pallet, cho on dinh...')
        elif status == GoalStatus.STATUS_CANCELED:
            self._set_state(STOPPED, 'Nav2 bi huy')
        else:
            self._set_state(FAILED, f'Nav2 that bai (status={status})')

    # ==================================================================
    # Alignment + insertion
    # ==================================================================
    def _robot_pose(self, frame: str) -> Optional[Tuple[float, float, float]]:
        try:
            tf = self.tf_buffer.lookup_transform(frame, self.p['base_frame'], Time())
        except Exception:
            return None
        age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
        if tf.header.stamp.sec != 0 and age > self.p['tf_stale_s']:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quat(q)

    def _pallet_in_insertion_frame(self) -> Optional[Tuple[float, float, float]]:
        x, y, psi = self._result['x'], self._result['y'], self._result['yaw']
        ins, mp = self.p['insertion_frame'], self.p['map_frame']
        if ins == mp:
            return x, y, psi
        try:
            tf = self.tf_buffer.lookup_transform(ins, mp, Time())
        except Exception:
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        th = yaw_from_quat(q)
        c, s = math.cos(th), math.sin(th)
        return c * x - s * y + t.x, s * x + c * y + t.y, norm_angle(psi + th)

    def _begin_alignment(self):
        res = self._pallet_in_insertion_frame()
        if res is None:
            self._set_state(FAILED, 'Khong doi duoc pose pallet sang khung odom (TF)')
            return
        self._ins_p = (res[0], res[1])
        self._ins_psi = res[2]
        if not self.p['auto_insert']:
            self._set_state(DONE, 'Da toi diem truoc pallet (auto_insert=false)')
            return
        self._rotating = False
        self._align_t0 = time.monotonic()
        self._set_state(ALIGNING, 'Canh lai goc voi pallet...')

    def _cmd(self, vx: float, wz: float):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self.pub_cmd.publish(t)

    def _halt(self):
        for _ in range(3):
            self._cmd(0.0, 0.0)

    def _finish(self, state: str, msg: str):
        self._halt()
        self._set_state(state, msg)

    def _control(self, now: float):
        pose = self._robot_pose(self.p['insertion_frame'])
        if pose is None:
            self._finish(FAILED, 'Mat TF robot -> dung xe')
            return
        rx, ry, ryaw = pose
        px, py = self._ins_p
        psi = self._ins_psi
        e_yaw, e_lat, along = pallet_line_errors(rx, ry, ryaw, px, py, psi)
        tol = math.radians(self.p['align_tol_deg'])

        # ---------------- ALIGNING: xoay tai cho ----------------
        if self._state == ALIGNING:
            if now - self._align_t0 > self.p['align_timeout_s']:
                self._finish(FAILED, 'Canh goc qua thoi gian')
                return
            if abs(e_yaw) <= tol:
                self._halt()
                self._us_count = 0
                self._insert_t0 = now
                self._along0 = along
                self._rotating = False
                self._set_state(INSERTING, f'Tien vao {self.p["insert_speed"]:.2f} m/s')
                return
            self._cmd(0.0, rotate_wz(e_yaw, self.p['k_yaw'],
                                     self.p['min_wz'], self.p['max_wz']))
            return

        # ---------------- INSERTING ----------------
        if now - self._us_t > self.p['range_stale_s']:
            self._finish(FAILED, 'Mat du lieu sieu am -> dung xe')
            return
        if self._us_count >= int(self.p['confirm_count']):
            cm = self._us_cm if self._us_cm is not None else float('nan')
            self._finish(DONE, f'XONG: sieu am {cm:.1f} cm '
                               f'(< {self.p["stop_dist_cm"]:.0f} cm x{self._us_count} mau)')
            return
        if along - self._along0 > self.p['max_insert_m']:
            self._finish(FAILED, f'Da tien {self.p["max_insert_m"]:.2f} m ma sieu am chua '
                                 f'dat nguong -> dung')
            return
        if now - self._insert_t0 > self.p['insert_timeout_s']:
            self._finish(FAILED, 'Tien vao qua thoi gian')
            return

        # lech goc lon -> dung tien, xoay lai (co hysteresis)
        if self._rotating:
            if abs(e_yaw) <= tol:
                self._rotating = False
        elif abs(e_yaw) > math.radians(self.p['realign_deg']):
            self._rotating = True
        if self._rotating:
            self._cmd(0.0, rotate_wz(e_yaw, self.p['k_yaw'],
                                     self.p['min_wz'], self.p['max_wz']))
            return

        wz = steer_wz(e_lat, ryaw, psi, self.p['k_yaw'], self.p['k_lat'],
                      math.radians(self.p['max_heading_offset_deg']), self.p['max_wz'])
        self._cmd(self.p['insert_speed'], wz)

    # ==================================================================
    # Emergency stop
    # ==================================================================
    def _emergency_stop(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
        self._terminate_detector()
        self._halt()
        self._set_state(STOPPED, 'DUNG KHAN CAP')

    # ==================================================================
    # Timers
    # ==================================================================
    def _tick(self):
        self._reap_detector()
        if self._state == DETECTING and self._det_proc is None:
            self._set_state(FAILED, 'Detector tu thoat bat ngo')
            return
        if self._state == SETTLING:
            if time.monotonic() - self._settle_t0 >= self.p['settle_s']:
                self._begin_alignment()
        elif self._state in (ALIGNING, INSERTING):
            self._control(time.monotonic())

    def _slow_tick(self):
        self._publish_status()
        self._publish_markers()

    # ==================================================================
    # Markers
    # ==================================================================
    @staticmethod
    def _color(r, g, b, a=1.0):
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = r, g, b, a
        return c

    def _publish_pose(self, pub, x, y, yaw):
        m = PoseStamped()
        m.header.frame_id = self.p['map_frame']
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.position.x, m.pose.position.y = x, y
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        pub.publish(m)

    def _publish_markers(self):
        cur = None
        if self._result:
            cur = (self._result['x'], self._result['y'], self._result['yaw'])
        elif self._state == DETECTING and self._samples:
            s = self._samples[-1]
            if time.monotonic() - s[0] < 1.5:
                cur = (s[1], s[2], s[3])

        arr = MarkerArray()
        now = self.get_clock().now().to_msg()
        frame = self.p['map_frame']

        def base(ns, i, mtype):
            m = Marker()
            m.header.frame_id = frame
            m.header.stamp = now
            m.ns, m.id, m.type, m.action = ns, i, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            return m

        if cur is None:
            d = Marker()
            d.action = Marker.DELETEALL
            arr.markers.append(d)
            self.pub_markers.publish(arr)
            return

        x, y, yaw = cur
        z = 0.15
        col = self._color(1.0, 0.85, 0.0)

        # diem pallet
        m = base('pallet', 0, Marker.SPHERE)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.x = m.scale.y = m.scale.z = 0.07
        m.color = col
        arr.markers.append(m)

        # mat truoc pallet (duong vuong goc phap tuyen)
        hw = self.p['face_width_m'] / 2.0
        nx, ny = -math.sin(yaw), math.cos(yaw)
        m = base('pallet_face', 1, Marker.LINE_STRIP)
        m.points = [Point(x=x - hw * nx, y=y - hw * ny, z=z),
                    Point(x=x + hw * nx, y=y + hw * ny, z=z)]
        m.scale.x = 0.04
        m.color = col
        arr.markers.append(m)

        # huong vao pallet (+X)
        m = base('pallet_dir', 2, Marker.ARROW)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        m.scale.x, m.scale.y, m.scale.z = 0.35, 0.05, 0.05
        m.color = col
        arr.markers.append(m)

        m = base('pallet_text', 3, Marker.TEXT_VIEW_FACING)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z + 0.25
        m.scale.z = 0.12
        m.color = self._color(1.0, 1.0, 1.0)
        m.text = f'pallet ({x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}deg)'
        arr.markers.append(m)

        # diem dem
        if self._pre:
            px, py, pyaw = self._pre
            m = base('pre_pose', 4, Marker.ARROW)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = px, py, z
            m.pose.orientation.z = math.sin(pyaw / 2.0)
            m.pose.orientation.w = math.cos(pyaw / 2.0)
            m.scale.x, m.scale.y, m.scale.z = 0.35, 0.08, 0.08
            m.color = self._color(0.1, 0.9, 0.2)
            arr.markers.append(m)

        self.pub_markers.publish(arr)

    # ==================================================================
    def destroy_node(self):
        try:
            if self._state in (ALIGNING, INSERTING):
                self._halt()
            if self._detector_alive():
                os.killpg(os.getpgid(self._det_proc.pid), signal.SIGINT)
                try:
                    self._det_proc.wait(timeout=3.0)
                except Exception:
                    os.killpg(os.getpgid(self._det_proc.pid), signal.SIGKILL)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PalletMission()
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