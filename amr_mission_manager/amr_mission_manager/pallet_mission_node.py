#!/usr/bin/env python3
"""
pallet_mission_node.py
======================

Pallet approach mission for amr_mission_manager.

Luong:
    [Detect ON]  -> chay pallet_detector (tien trinh con), gom /pallet/pose
    [Detect OFF] -> tat detector, chot x, y, yaw (map). DONG BANG pallet vao khung odom
                    ngay luc nay (xe dang dung yen) -> sai so AMCL sau do khong con anh huong.
    [Go to pallet] -> Nav2 toi diem dem (staging) cach MAT TRUOC pallet `staging` m
                      (cat Nav2 som khi con <= handoff_m de tranh "nhich tung ti")
                   -> ALIGNING: lui ra neu qua gan, xoay thang huong, DI NGANG (sidestep)
                      cho het lech ngang x,y (con cho de xoay), roi xoay lai
                   -> INSERTING: tien, DUNG o cong (cach mat pallet gate_margin_m), do lai
                      lech ngang + lech goc; KHONG dat thi lui ra lam lai (khong lao vao)
                      qua cong: giu huong, siêu am dung
                   -> LIFTING : nang lift len lift_height_cm (mac dinh 5 cm)
                   -> BACKING : lui thang backoff_m = 0.40 m (neu backoff_m = 0 thi dung handoff_m)
                   -> HOMING  : Nav2 ve Home (set_home hoac home_x/home_y/home_yaw_deg)
                   -> LOWERING: ha lift ve 0 (lenh HOME)
                   -> DONE    : xong 1 chu trinh
    [STOP]       -> huy Nav2, dung xe, DUNG LIFT (M0), tat detector

Bo dieu khien (class ApproachController) la python thuan, khong phu thuoc ROS,
da duoc mo phong kiem tra rieng.

GIOI HAN TRUNG THUC: sau Detect OFF khong co cam bien nao nhin lai pallet
(camera chi thay tu >= 1.6 m). Xe "bam" duong thang da luu trong odom, KHONG phai bam
pallet that. Sai so con lai = sai so detector/camera tai luc detect + troi odom
(vai mm .. 1-2 cm tren < 1.5 m). Dung pallet_lat_offset_m / pallet_yaw_offset_deg de
bu sai so he thong (luon lech cung mot chieu).

Quy uoc: yaw cua pallet = huong +X (huong VAO pallet). Diem (x, y) cua detector duoc coi
la diem tren MAT TRUOC pallet (detector dung depth mat truoc). Neu thuc te la TAM pallet:
dat face_offset_m = -(nua chieu sau pallet), vi du -0.5.

Services (std_srvs):
    pallet_mission/detect        SetBool  true: bat | false: tat + chot
    pallet_mission/go_to_pallet  Trigger
    pallet_mission/lift_home     Trigger
    pallet_mission/stop          Trigger
    pallet_mission/calibrate     Trigger
    pallet_mission/set_home      Trigger  dat Home = pose hien tai cua xe (khung map)
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
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import String, ColorRGBA
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


NODE_NAME = 'pallet_mission'

# States (GUI dung cac ten nay -> KHONG doi)
IDLE = 'IDLE'
DETECTING = 'DETECTING'
READY = 'READY'
NAVIGATING = 'NAVIGATING'
SETTLING = 'SETTLING'
ALIGNING = 'ALIGNING'
INSERTING = 'INSERTING'
LIFTING = 'LIFTING'
BACKING = 'BACKING'         # moi: lui ra sau khi nang
HOMING = 'HOMING'           # moi: ve Home
LOWERING = 'LOWERING'       # moi: ha lift ve 0
DONE = 'DONE'
FAILED = 'FAILED'
STOPPED = 'STOPPED'

BUSY_STATES = (NAVIGATING, SETTLING, ALIGNING, INSERTING, LIFTING,
               BACKING, HOMING, LOWERING)

# Tham so (gia tri mac dinh). Launch chi truyen mot phan; phan con lai lay o day.
DEFAULTS = {
    # detector
    'detector_package': 'amr_perception',
    'detector_executable': 'pallet_detector',
    'detector_camera_info_in': '/camera/color/camera_info',
    'detector_camera_info_out': '/color/camera_info',
    'detector_extra_args': [''],
    # topics / frames
    'pose_topic': '/pallet/pose',
    'range_topic': '/ultrasonic/range',
    'cmd_vel_topic': '/cmd_vel',
    'map_frame': 'map',
    'base_frame': 'base_footprint',
    'insertion_frame': 'odom',
    # chot ket qua
    'final_window_s': 3.0,
    'min_samples': 10,
    'max_sample_age_s': 2.0,
    'max_pos_std_m': 0.03,
    'max_yaw_std_deg': 3.0,
    'require_stable': True,
    'max_tf_jump_m': 0.03,          # map->odom nhay > nguong trong luc detect -> khong on dinh
    'max_tf_jump_deg': 1.5,
    # hieu chinh he thong (khung pallet: +x vao pallet, +y trai)
    'pallet_lat_offset_m': 0.0,     # >0 dich diem dich sang TRAI huong vao pallet
    'pallet_yaw_offset_deg': 0.0,
    'face_offset_m': 0.0,           # vi tri mat truoc so voi diem detector, doc +X
    # Hieu chinh LECH DO NGHIENG: detector lay tia qua pixel TAM OBB nhung do sau = percentile
    # (gan nhat) -> khi pallet nghieng, diem bi keo ve phia xe va lech ngang cung chieu nghieng.
    'skew_corr_gain': 1.0,          # 0 = tat; 1 = bu day du theo mo hinh; chinh 0.5..1.3 theo thuc te
    'pallet_width_m': 0.4,          # chieu dai mat truoc pallet (m)
    'obb_shrink': 0.6,              # = shrink trong extract_depth_in_obb cua detector
    'depth_percentile': 25.0,       # = depth_percentile cua detector
    'camera_x_m': 0.20,             # camera cach base_footprint ve phia truoc (m), uoc luong
    # hinh hoc xe
    'fork_front_m': 0.42,           # base_footprint -> mui cang
    'rot_clear_m': 0.60,            # can >= nua dai xe + du de xoay tai cho (bk goc ~0.50)
    # approach
    'standoff_m': 0.50,             # khoang cach base_footprint -> mat pallet (se nang len min_staging_m)
    'min_staging_m': 0.65,
    'nav_bt_package': 'amr_navigation',
    'nav_bt_file': 'return_home.xml',
    'handoff_m': 0.20,              # trong ban kinh nay MA Nav2 chua xong sau handoff_hold_s -> cat Nav2 (0 = tat)
    'handoff_hold_s': 4.0,          # Nav2 tu toi dich duoc thi khong bi cat (khong con 'cancel')
    'nav_abort_ok_m': 0.35,         # Nav2 abort nhung xe trong ban kinh nay -> van tiep tuc
    'settle_s': 1.0,
    'auto_insert': True,
    # canh chinh (xoay + di ngang)
    'align_tol_deg': 1.5,           # dung xoay khi vao dung sai nay
    'align_accept_deg': 3.0,        # sau khi dung + cho on dinh: chap nhan neu <= nguong nay
                                    # (CONVERGE + Stanley tu keo yaw ve ~0 truoc cong, cong kiem lai <= gate_yaw_deg)
    'pulse_zone_deg': 8.0,          # lech < nguong nay: xoay tung NHIP ngan roi dung do (khong xoay lien tuc)
    'pulse_gain': 0.6,              # moi nhip xoay ~60% goc con lai; tu giam 1/2 neu phat hien vot qua
    'max_pulses': 10,
    'rot_settle_s': 0.5,
    'lat_tol_cm': 1.5,              # lech ngang > nguong -> di ngang
    'max_sidestep_m': 0.30,
    'max_sidesteps': 3,
    'side_speed': 0.05,
    'side_tol_deg': 3.0,
    'side_accept_deg': 6.0,
    'side_done_cm': 0.5,
    'retreat_speed': 0.08,
    'retreat_max_m': 0.80,
    'k_yaw': 1.5,
    'min_wz': 0.20,                 # toc do xoay TAI CHO toi thieu (phai > ma sat tinh)
    'max_wz': 0.35,
    'drive_min_wz': 0.0,            # 0 = tat. Khi dang CHAY, lenh xoay nho hon ma sat tinh khong lam xe
                                    # quay -> dat = wz nho nhat that su lam xe quay (do bang cmd_vel)
    'drive_db_on_deg': 1.0,         # lech huong > nguong nay moi bat bu
    'drive_db_off_deg': 0.4,        # < nguong nay tat (tranh chop)
    # tien vao
    'insert_speed': 0.10,
    'gate_margin_m': 0.05,          # cong cach mat pallet (tinh tu mui cang)
    'gate_pause_s': 0.4,
    'gate_lat_cm': 2.0,
    'gate_yaw_deg': 2.0,
    'max_gate_retries': 2,
    'realign_deg': 6.0,             # trong vung hoi tu: lech goc > nguong -> dung, xoay lai
    'k_lat_conv': 2.0,
    'conv_max_offset_deg': 8.0,
    'lock_k_lat': 0.5,              # sau cong: gan nhu chi giu huong
    'lock_max_offset_deg': 2.0,
    'lock_lat_abort_cm': 3.0,
    'lock_yaw_abort_deg': 4.0,
    # dung (sieu am)
    'stop_dist_cm': 3.0,
    'confirm_count': 10,
    'us_arm_m': 0.10,               # chi tin sieu am sau khi da di vao >= 10 cm tu cong
    'us_below_min_is_close': True,  # doc < min_range cung tinh la "sat"
    'max_insert_m': 0.90,           # quang toi da tu cong (= chieu dai cang + du)
    'insert_timeout_s': 40.0,
    'align_timeout_s': 40.0,
    'total_timeout_s': 180.0,
    'range_stale_s': 0.5,
    'tf_stale_s': 0.5,
    # lift
    'esp32_cmd_topic': '/esp32_cmd',
    'joint_states_topic': '/joint_states',
    'lift_joint_name': 'encoder_lift_joint',
    'auto_lift': True,              # True: vao pallet xong tu nang -> lui -> Home -> ha lift
    'lift_height_cm': 5.0,          # chieu cao nang (cm)
    'lift_tol_cm': 1.0,
    'lift_timeout_s': 60.0,
    'lift_start_timeout_s': 4.0,
    'lift_move_eps_cm': 0.3,
    'joint_stale_s': 1.0,
    # sau khi nang: lui -> ve Home -> ha lift
    'backoff_m': 0.40,              # quang lui (m); 0 = dung handoff_m
    'backoff_speed': 0.08,          # toc do lui (m/s)
    'backoff_timeout_s': 20.0,
    # Home = NODE_HOME trong route graph (khung map). Dung cung BT return_home.xml nhu mission_manager.
    'home_set': True,               # True: dung home_x/home_y/home_yaw_deg (khung map)
    'home_x': 0.055,
    'home_y': -0.061,
    'home_yaw_deg': 2.086,          # yaw 0.0364 rad
    'lower_on_detect': True,        # bam Detect ON thi ha lift ve Home (lenh lift_home_cmd)
    'lift_home_cmd': 'HOME',        # lenh ha lift ve 0 (giong nut Lift HOME)
    'lift_home_tol_cm': 1.0,        # lift <= nguong nay thi coi la da ve Home
    # kiem tra nhat quan
    'amcl_shift_warn_m': 0.10,
    'yaw_warn_deg': 1.5,            # map->odom xoay > nguong tu luc detect -> canh bao
    'map_yaw_trim': False,          # True: truoc khi tien vao, chinh lai huong theo yaw MAP hien tai
                                    # (AMCL) thay vi yaw odom da dong bang luc detect
    'amcl_shift_fail_m': 0.60,
    # marker
    'face_width_m': 1.2,
}


# ----------------------------------------------------------------------
# Pure helpers
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
    """Chot x, y, yaw tu cac mau (t, x, y, yaw). Return (dict | None, reason)."""
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
    return {'x': x, 'y': y, 'yaw': yaw, 'n': len(win), 'pos_std': pos_std,
            'yaw_std_deg': yaw_std, 'stable': stable}, ''


def _detector_bias(dist, yc, phi, W, shrink, pct):
    """Mo hinh lech cua diem detector so voi tam mat truoc that (he truc xe: x truoc, y trai).

    Detector: tia qua pixel tam OBB, do sau = percentile `pct` trong vung OBB thu `shrink`.
    Mat truoc nghieng phi -> do sau thay doi doc mat -> diem bi keo ve phia xe + lech ngang."""
    c = np.array([dist, yc], dtype=np.float64)
    yp = np.array([-math.sin(phi), math.cos(phi)])
    s = np.linspace(-W / 2.0, W / 2.0, 801)
    P = c[None, :] + s[:, None] * yp[None, :]
    if np.any(P[:, 0] < 0.3):
        return np.zeros(2)
    u = P[:, 1] / P[:, 0]
    uc = 0.5 * (u.min() + u.max())
    half = 0.5 * (u.max() - u.min())
    m = np.abs(u - uc) <= shrink * half
    d = float(np.percentile(P[m, 0], pct))
    return np.array([d, d * uc]) - c


def correct_skew_bias(xb, yb, phi, cam_x, W, shrink, pct, gain):
    """Diem tam mat truoc DA HIEU CHINH (khung xe) tu diem detector (xb, yb), phi = yaw pallet so voi xe."""
    if gain == 0.0:
        return xb, yb
    meas = np.array([xb - cam_x, yb], dtype=np.float64)
    c = meas.copy()
    for _ in range(8):
        c = meas - gain * _detector_bias(float(c[0]), float(c[1]), phi, W, shrink, pct)
    return float(c[0]) + cam_x, float(c[1])


def apply_offsets(x, y, yaw, lat_off, yaw_off_deg):
    """Bu sai so he thong: dich ngang (khung pallet) + cong yaw."""
    x += -math.sin(yaw) * lat_off
    y += math.cos(yaw) * lat_off
    return x, y, norm_angle(yaw + math.radians(yaw_off_deg))


def approach_pose(x, y, yaw, dist_back):
    """Diem cach diem pallet `dist_back` (doc phap tuyen), huong xe = yaw pallet."""
    return x - dist_back * math.cos(yaw), y - dist_back * math.sin(yaw), yaw


def line_errors(rx, ry, ryaw, px, py, psi, face_off=0.0):
    """Sai so cua xe so voi duong thang qua (px,py) huong psi.

    e_yaw : psi - yaw_xe          (>0: xe can quay trai)
    e_lat : >0 = xe nam ben TRAI duong (can di sang phai)
    a     : toa do doc duong tinh tu MAT TRUOC pallet (am = chua toi mat pallet)
    """
    e_yaw = norm_angle(psi - ryaw)
    dx, dy = rx - px, ry - py
    e_lat = -math.sin(psi) * dx + math.cos(psi) * dy
    a = math.cos(psi) * dx + math.sin(psi) * dy - face_off
    return e_yaw, e_lat, a


def steer_wz(e_lat, ryaw, psi, k_yaw, k_lat, max_offset_rad, max_wz):
    """Stanley: huong muc tieu = psi - atan(k_lat * e_lat) (gioi han)."""
    off = clamp(math.atan(k_lat * e_lat), -max_offset_rad, max_offset_rad)
    e_h = norm_angle(psi - off - ryaw)
    return clamp(k_yaw * e_h, -max_wz, max_wz)


def rotate_wz(e, k_yaw, min_wz, max_wz):
    mag = clamp(k_yaw * abs(e), min_wz, max_wz)
    return math.copysign(mag, e)


# ----------------------------------------------------------------------
# Bo dieu khien tiep can (python thuan -> test duoc ngoai ROS)
# ----------------------------------------------------------------------
class ApproachController:
    """Lui (neu gan) -> xoay -> di ngang -> xoay lai -> tien toi cong -> kiem tra ->
    tien thang -> dung theo sieu am. Moi pose o khung `insertion_frame`."""

    ENTER, RETREAT, ALIGN, SIDE_TURN, SIDE_DRIVE = \
        'ENTER', 'RETREAT', 'ALIGN', 'SIDE_TURN', 'SIDE_DRIVE'
    CONVERGE, GATE, LOCK = 'CONVERGE', 'GATE', 'LOCK'
    ALIGN_PHASES = (ENTER, RETREAT, ALIGN, SIDE_TURN, SIDE_DRIVE)

    def __init__(self, p: dict, stage_m: float):
        self.p = p
        self.stage_m = stage_m
        self.log = []
        self.reset()

    def reset(self):
        self.phase = self.ENTER
        self.t_start = None
        self.t_phase = 0.0
        self._wait_t0 = None
        self._rot_tries = 0
        self._steer_on = False
        self._pulse_until = None
        self._pulse_wz = 0.0
        self._pgain = None
        self._last_e = None
        self.sidesteps = 0
        self.gate_retries = 0
        self.err = (0.0, 0.0, 0.0)      # e_lat, e_yaw, a
        self._ref = (0.0, 0.0)
        self._side_dir = 1.0
        self._side_h = 0.0
        self._side_e0 = 0.0
        self._a_lock0 = 0.0
        self.log = []

    # ---- helpers ----
    def _goto(self, phase, now):
        self.phase = phase
        self.t_phase = now
        self._wait_t0 = None
        self._rot_tries = 0
        self._steer_on = False
        self._pulse_until = None
        self._pgain = None
        self._last_e = None

    def _fail(self, msg):
        return 0.0, 0.0, None, msg

    def restart_align(self, now):
        self._goto(self.ALIGN, now)

    def _min_wz(self, wz, e_h):
        """Bu vung chet ma sat khi dang chay: co tre hai nguong de khong chop."""
        m = self.p['drive_min_wz']
        if m <= 0.0:
            return wz
        e = abs(e_h)
        if self._steer_on:
            if e < math.radians(self.p['drive_db_off_deg']):
                self._steer_on = False
        elif e > math.radians(self.p['drive_db_on_deg']):
            self._steer_on = True
        if self._steer_on and abs(wz) < m:
            return math.copysign(m, e_h if wz == 0.0 else wz)
        return wz

    def _steer(self, e_lat, ryaw, psi, k_lat, max_off_deg):
        p = self.p
        off = clamp(math.atan(k_lat * e_lat), -math.radians(max_off_deg), math.radians(max_off_deg))
        e_h = norm_angle(psi - off - ryaw)
        wz = clamp(p['k_yaw'] * e_h, -p['max_wz'], p['max_wz'])
        return self._min_wz(wz, e_h)

    def _turn(self, now, e, stop, accept):
        """Xoay tai cho ve e=0. Return (wz, state): 'run' | 'ok' | 'fail'.

        Lech lon: xoay lien tuc. Lech nho (< pulse_zone): xoay tung NHIP ngan o min_wz,
        dung, cho het quan tinh, DO LAI roi moi quyet dinh nhip tiep. Neu sau nhip
        sai so doi dau (vot qua) -> nhip sau ngan di mot nua. Khong bao gio xoay lien tuc
        khi gan dich nen khong the lac trai-phai."""
        p = self.p
        if self._pgain is None:
            self._pgain = float(p['pulse_gain'])
        if self._pulse_until is not None:                 # dang trong 1 nhip
            if now < self._pulse_until:
                return self._pulse_wz, 'run'
            self._pulse_until = None
            self._wait_t0 = now
            return 0.0, 'run'
        if self._wait_t0 is not None:                     # cho on dinh
            if now - self._wait_t0 < p['rot_settle_s']:
                return 0.0, 'run'
            self._wait_t0 = None
            if self._last_e is not None and e * self._last_e < 0.0 and abs(e) > 0.3 * stop:
                self._pgain = max(0.15, self._pgain * 0.5)   # vua vot qua -> nhip ngan hon
            self._last_e = e
            if abs(e) <= stop:
                return 0.0, 'ok'
            self._rot_tries += 1
            if self._rot_tries > int(p['max_pulses']):
                return (0.0, 'ok') if abs(e) <= accept else (0.0, 'fail')
            dur = clamp(self._pgain * abs(e) / p['min_wz'], 0.05, 0.5)
            self._pulse_wz = math.copysign(p['min_wz'], e)
            self._pulse_until = now + dur
            return self._pulse_wz, 'run'
        if abs(e) <= math.radians(p['pulse_zone_deg']):   # vao vung nhip: dung lai, do
            self._wait_t0 = now
            return 0.0, 'run'
        return rotate_wz(e, p['k_yaw'], p['min_wz'], p['max_wz']), 'run'

    # ---- main step: (vx, wz, done_msg|None, fail_msg|None) ----
    def step(self, now, pose, line, us_count, us_fresh):
        p = self.p
        rx, ry, ryaw = pose
        px, py, psi = line
        e_yaw, e_lat, a = line_errors(rx, ry, ryaw, px, py, psi, p['face_offset_m'])
        self.err = (e_lat, e_yaw, a)
        fork = p['fork_front_m']
        a_gate = -(fork + p['gate_margin_m'])
        tol = math.radians(p['align_tol_deg'])

        if self.t_start is None:
            self.t_start = now
        if now - self.t_start > p['total_timeout_s']:
            return self._fail('Qua thoi gian tong')

        ph = self.phase

        if ph == self.ENTER:
            if a + fork > -0.02:
                return self._fail(f'Mui cang da toi/qua mat pallet (a={a*100:.0f} cm) -> khong tu dong')
            if abs(e_lat) > p['max_sidestep_m']:
                return self._fail(f'Lech ngang {e_lat*100:.0f} cm > {p["max_sidestep_m"]*100:.0f} cm '
                                  f'(pose pallet hoac Nav2 sai) -> detect lai')
            self._ref = (rx, ry)
            self._goto(self.RETREAT if a > -p['rot_clear_m'] else self.ALIGN, now)
            if self.phase == self.RETREAT:
                self.log.append(f'Qua gan pallet (a={a*100:.0f} cm) -> lui ra de co cho xoay')
            return 0.0, 0.0, None, None

        if ph == self.RETREAT:
            if a <= -self.stage_m:
                self._goto(self.ALIGN, now)
                return 0.0, 0.0, None, None
            if math.hypot(rx - self._ref[0], ry - self._ref[1]) > p['retreat_max_m'] \
                    or now - self.t_phase > 25.0:
                return self._fail('Lui qua xa/qua thoi gian ma van chua du cho xoay')
            wz = self._min_wz(clamp(p['k_yaw'] * e_yaw, -0.2, 0.2), e_yaw)
            return -p['retreat_speed'], wz, None, None

        if ph == self.ALIGN:
            if now - self.t_phase > p['align_timeout_s']:
                return self._fail('Canh goc qua thoi gian')
            wz, st = self._turn(now, e_yaw, tol, math.radians(p['align_accept_deg']))
            if st == 'fail':
                return self._fail(f'Xoay {self._rot_tries} nhip van lech {math.degrees(e_yaw):+.1f} deg '
                                  f'(min_wz qua lon/qua nho? xem odom)')
            if st != 'ok':
                return 0.0, wz, None, None
            lat_tol = p['lat_tol_cm'] / 100.0
            if abs(e_lat) > lat_tol:
                if self.sidesteps < int(p['max_sidesteps']):
                    self.sidesteps += 1
                    self._side_dir = 1.0 if e_lat > 0 else -1.0
                    self._side_h = norm_angle(psi - self._side_dir * math.pi / 2.0)
                    self._side_e0 = abs(e_lat)
                    self.log.append(f'Lech ngang {e_lat*100:+.1f} cm -> di ngang (lan {self.sidesteps})')
                    self._goto(self.SIDE_TURN, now)
                    return 0.0, 0.0, None, None
                if abs(e_lat) > p['gate_lat_cm'] / 100.0:
                    return self._fail(f'Di ngang {self.sidesteps} lan van lech {e_lat*100:+.1f} cm')
            self._goto(self.CONVERGE, now)
            return 0.0, 0.0, None, None

        if ph == self.SIDE_TURN:
            if now - self.t_phase > p['align_timeout_s']:
                return self._fail('Xoay de di ngang qua thoi gian')
            e_h = norm_angle(self._side_h - ryaw)
            wz, st = self._turn(now, e_h, math.radians(p['side_tol_deg']),
                                math.radians(p['side_accept_deg']))
            if st == 'fail':
                return self._fail('Xoay de di ngang khong dat huong')
            if st != 'ok':
                return 0.0, wz, None, None
            self._ref = (rx, ry)
            self._goto(self.SIDE_DRIVE, now)
            return 0.0, 0.0, None, None

        if ph == self.SIDE_DRIVE:
            traveled = math.hypot(rx - self._ref[0], ry - self._ref[1])
            if e_lat * self._side_dir <= p['side_done_cm'] / 100.0:
                self._goto(self.ALIGN, now)
                return 0.0, 0.0, None, None
            if traveled > self._side_e0 + 0.05 or now - self.t_phase > 20.0:
                return self._fail('Di ngang khong het lech (odom/ban xe?)')
            e_h = norm_angle(self._side_h - ryaw)
            wz = self._min_wz(clamp(p['k_yaw'] * e_h, -0.2, 0.2), e_h)
            return p['side_speed'], wz, None, None

        if ph == self.CONVERGE:
            if now - self.t_phase > p['insert_timeout_s']:
                return self._fail('Tien toi cong qua thoi gian')
            if a >= a_gate:
                self._goto(self.GATE, now)
                return 0.0, 0.0, None, None
            if abs(e_yaw) > math.radians(p['realign_deg']):
                self.log.append(f'Lech goc {math.degrees(e_yaw):+.1f} deg khi tien -> dung xoay lai')
                self._goto(self.ALIGN, now)
                return 0.0, 0.0, None, None
            wz = self._steer(e_lat, ryaw, psi, p['k_lat_conv'], p['conv_max_offset_deg'])
            return p['insert_speed'], wz, None, None

        if ph == self.GATE:
            if now - self.t_phase < p['gate_pause_s']:
                return 0.0, 0.0, None, None
            lat_cm, yaw_deg = e_lat * 100.0, math.degrees(e_yaw)
            if abs(lat_cm) <= p['gate_lat_cm'] and abs(yaw_deg) <= p['gate_yaw_deg']:
                self.log.append(f'QUA CONG: lat={lat_cm:+.1f} cm yaw={yaw_deg:+.1f} deg '
                                f'(mui cang cach mat pallet {-(a + fork)*100:.0f} cm)')
                self._a_lock0 = a
                self._goto(self.LOCK, now)
                return 0.0, 0.0, None, None
            if self.gate_retries < int(p['max_gate_retries']):
                self.gate_retries += 1
                self.sidesteps = 0
                self.log.append(f'KHONG QUA CONG: lat={lat_cm:+.1f} cm yaw={yaw_deg:+.1f} deg '
                                f'-> lui ra lam lai (lan {self.gate_retries})')
                self._goto(self.RETREAT, now)
                self._ref = (rx, ry)
                return 0.0, 0.0, None, None
            return self._fail(f'Khong qua cong sau {self.gate_retries} lan lui: '
                              f'lat={lat_cm:+.1f} cm yaw={yaw_deg:+.1f} deg')

        if ph == self.LOCK:
            depth = a - self._a_lock0                    # quang da tien tu cong
            if abs(e_lat) * 100.0 > p['lock_lat_abort_cm'] or \
                    abs(math.degrees(e_yaw)) > p['lock_yaw_abort_deg']:
                return self._fail(f'Lech trong luc vao: lat={e_lat*100:+.1f} cm '
                                  f'yaw={math.degrees(e_yaw):+.1f} deg -> dung')
            if depth >= p['us_arm_m']:
                if not us_fresh:
                    return self._fail('Mat du lieu sieu am -> dung xe')
                if us_count >= int(p['confirm_count']):
                    return 0.0, 0.0, f'Vao pallet {depth*100:.0f} cm tu cong, sieu am du mau', None
            if depth > p['max_insert_m']:
                return self._fail(f'Da tien {depth:.2f} m tu cong ma sieu am chua dat nguong -> dung')
            if now - self.t_phase > p['insert_timeout_s']:
                return self._fail('Tien vao qua thoi gian')
            wz = self._steer(e_lat, ryaw, psi, p['lock_k_lat'], p['lock_max_offset_deg'])
            return p['insert_speed'], wz, None, None

        return self._fail(f'phase la: {ph}')


PHASE_TEXT = {
    'ENTER': 'Kiem tra vi tri...',
    'RETREAT': 'Lui ra de co cho xoay',
    'ALIGN': 'Xoay thang huong pallet',
    'SIDE_TURN': 'Xoay de di ngang',
    'SIDE_DRIVE': 'Di ngang cho het lech',
    'CONVERGE': 'Tien toi cong pallet',
    'GATE': 'Dung o cong, do lai x,y,yaw',
    'LOCK': 'Tien vao pallet (giu huong)',
}


class PalletMission(Node):

    def __init__(self):
        super().__init__(NODE_NAME)

        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in DEFAULTS}

        # diem dem (khoang cach base_footprint -> mat pallet)
        self.stage_m = max(float(self.p['standoff_m']), float(self.p['min_staging_m']),
                           float(self.p['rot_clear_m']) + 0.03)
        if self.stage_m > float(self.p['standoff_m']) + 1e-6:
            self.get_logger().warn(
                f"standoff_m={self.p['standoff_m']:.2f} qua gan de xoay/di ngang an toan "
                f"-> dung {self.stage_m:.2f} m (mat pallet -> base_footprint)")

        # ---------------- State ----------------
        self._state = IDLE
        self._msg = 'San sang'
        self._samples = deque(maxlen=600)
        self._tf_hist = deque(maxlen=120)   # (t, x, y, yaw) map->odom luc detect
        self._result = None
        self._pre = None
        self._pallet_ins = None             # (x, y, yaw) trong insertion_frame, DONG BANG luc Detect OFF
        self._line = None
        self._det_proc = None
        self._det_kill_at = None

        self._goal_handle = None
        self._settle_t0 = 0.0
        self._settle_tf = []
        self._handoff_t0 = None
        self._tf_miss = 0
        self._ctl = ApproachController(self.p, self.stage_m)
        self._phase_shown = None
        self._trimmed = False

        self._calib = None              # (y pallet trong khung xe, yaw pallet so voi xe) luc Detect OFF
        self._us_cm = None
        self._us_t = 0.0
        self._us_count = 0

        self._lift_cm = None
        self._lift_t = 0.0
        self._lift_t0 = 0.0
        self._lift_h0 = 0.0
        self._lift_moved = False
        self._ins_msg = ''
        self._lift_goal = 0.0           # chieu cao dich cua lan nang/ha hien tai (cm)
        self._lift_is_home = False      # True: dang ha ve Home (LOWERING)

        self._bo_ref = (0.0, 0.0)       # diem bat dau lui (insertion_frame)
        self._bo_t0 = 0.0
        self._bo_dist = 0.0
        self._bo_tf_miss = 0
        self._home = None               # (x, y, yaw) trong map, dat bang set_home

        # ---------------- ROS I/O ----------------
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.pub_status = self.create_publisher(String, f'/{NODE_NAME}/status', 10)
        self.pub_markers = self.create_publisher(MarkerArray, f'/{NODE_NAME}/markers', 10)
        self.pub_pallet = self.create_publisher(PoseStamped, f'/{NODE_NAME}/pallet_pose', latched)
        self.pub_pre = self.create_publisher(PoseStamped, f'/{NODE_NAME}/pre_pose', latched)
        self.pub_cmd = self.create_publisher(Twist, self.p['cmd_vel_topic'], 10)
        self.pub_esp32 = self.create_publisher(String, self.p['esp32_cmd_topic'], 10)

        self.create_subscription(PoseStamped, self.p['pose_topic'], self._on_pose, 10)
        self.create_subscription(Range, self.p['range_topic'], self._on_range, 10)
        self.create_subscription(JointState, self.p['joint_states_topic'],
                                 self._on_joint_states, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cancel_all_cli = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal')

        self.create_service(SetBool, f'{NODE_NAME}/detect', self._srv_detect)
        self.create_service(Trigger, f'{NODE_NAME}/go_to_pallet', self._srv_go)
        self.create_service(Trigger, f'{NODE_NAME}/lift_home', self._srv_lift_home)
        self.create_service(Trigger, f'{NODE_NAME}/stop', self._srv_stop)
        self.create_service(Trigger, f'{NODE_NAME}/calibrate', self._srv_calibrate)
        self.create_service(Trigger, f'{NODE_NAME}/set_home', self._srv_set_home)

        self.create_timer(0.05, self._tick)
        self.create_timer(0.5, self._slow_tick)

        self.get_logger().info(f'Pallet Mission ready (staging {self.stage_m:.2f} m)')
        self._publish_status()

    def _reload_params(self, recompute_stage: bool):
        """Doc lai tham so (ros2 param set ... co hieu luc ngay, khong can khoi dong lai node)."""
        try:
            self.p.update({k: self.get_parameter(k).value for k in DEFAULTS})
        except Exception as e:
            self.get_logger().warn(f'reload params failed: {e}')
            return
        if recompute_stage:
            self.stage_m = max(float(self.p['standoff_m']), float(self.p['min_staging_m']),
                               float(self.p['rot_clear_m']) + 0.03)

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
        lift = self._lift_height_now()
        err = None
        if self._state in (ALIGNING, INSERTING):
            e_lat, e_yaw, a = self._ctl.err
            my = self._map_yaw_err()
            err = {'lat_cm': round(e_lat * 100, 1), 'yaw_deg': round(math.degrees(e_yaw), 1),
                   'to_face_cm': round(-a * 100, 0),
                   'yaw_map_deg': None if my is None else round(my, 1)}
        d = {'state': self._state, 'msg': self._msg, 'samples': len(self._samples),
             'live': live, 'pallet': pallet, 'pre': pre, 'us_cm': us,
             'us_count': self._us_count,
             'lift_cm': None if lift is None else round(lift, 1),
             'phase': self._ctl.phase if self._state in (ALIGNING, INSERTING) else None,
             'err': err}
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
        close = False
        if math.isfinite(cm):
            if cm < self.p['stop_dist_cm'] and (cm >= lo or self.p['us_below_min_is_close']):
                close = True
        self._us_count = self._us_count + 1 if close else 0

    def _on_joint_states(self, msg: JointState):
        try:
            i = msg.name.index(self.p['lift_joint_name'])
            self._lift_cm = float(msg.position[i]) * 100.0
            self._lift_t = time.monotonic()
        except (ValueError, IndexError):
            pass

    def _lift_height_now(self) -> Optional[float]:
        if self._lift_cm is None:
            return None
        if time.monotonic() - self._lift_t > self.p['joint_stale_s']:
            return None
        return self._lift_cm

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

    def _srv_lift_home(self, req, res):
        if self._state in BUSY_STATES:
            res.success = False
            res.message = f'dang ban ({self._state}), khong the ha lift'
        else:
            self._esp32('HOME')
            res.success = True
            res.message = 'Da gui HOME: lift dang ha ve 0 cm (bam STOP de dung)'
        return res

    def _srv_calibrate(self, req, res):
        """Dat xe DONG TRUC that voi pallet (hai cang thang hai khe), pallet THANG, Detect ON -> OFF,
        roi goi dich vu nay: node tu tinh pallet_lat_offset_m = -(y pallet trong khung xe)."""
        if self._state != READY or self._calib is None:
            res.success, res.message = False, 'Can Detect ON/OFF xong (state READY) truoc'
            return res
        yb, phi = self._calib
        if abs(math.degrees(phi)) > 4.0:
            res.success = False
            res.message = (f'Pallet dang nghieng {math.degrees(phi):+.1f} deg so voi xe: dat pallet THANG '
                           f'(<4 deg) roi detect lai de calibrate')
            return res
        off = -yb
        if abs(off) > 0.15:
            res.success = False
            res.message = f'Offset tinh ra {off * 100:+.1f} cm qua lon (>15 cm): kiem tra lai xe co dong truc that khong'
            return res
        try:
            self.set_parameters([Parameter('pallet_lat_offset_m', Parameter.Type.DOUBLE, float(off))])
        except Exception as e:
            res.success, res.message = False, f'khong dat duoc tham so: {e}'
            return res
        self.p['pallet_lat_offset_m'] = float(off)
        msg = (f'pallet_lat_offset_m = {off:+.4f} m (y pallet trong khung xe luc dong truc = {yb * 100:+.1f} cm). '
               f'Detect ON/OFF lai de ap dung; ghi so nay vao DEFAULTS de giu lau dai.')
        self.get_logger().info('[calib] ' + msg)
        res.success, res.message = True, msg
        return res

    def _srv_stop(self, req, res):
        self._emergency_stop()
        res.success, res.message = True, 'STOPPED'
        return res

    def _srv_set_home(self, req, res):
        """Dat Home = pose hien tai cua xe trong khung map (dung xe o vi tri Home roi goi)."""
        r = self._robot_pose(self.p['map_frame'])
        if r is None:
            res.success, res.message = False, 'Khong doc duoc TF map->base'
            return res
        self._home = r
        msg = (f'Home = ({r[0]:.3f}, {r[1]:.3f}, {math.degrees(r[2]):.1f}deg). '
               f'Ghi vao DEFAULTS: home_set=True, home_x={r[0]:.3f}, home_y={r[1]:.3f}, '
               f'home_yaw_deg={math.degrees(r[2]):.1f}')
        self.get_logger().info('[home] ' + msg)
        res.success, res.message = True, msg
        return res

    def _get_home(self):
        """Home dang dung: tu set_home, neu khong thi tu tham so (home_set=True)."""
        if self._home is not None:
            return self._home
        if self.p['home_set']:
            return (float(self.p['home_x']), float(self.p['home_y']),
                    math.radians(float(self.p['home_yaw_deg'])))
        return None

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
        cmd += [a for a in (self.p['detector_extra_args'] or []) if a]
        try:
            self._det_proc = subprocess.Popen(cmd, start_new_session=True)
        except Exception as e:
            self._det_proc = None
            return False, f'khong chay duoc detector: {e}'
        self._det_kill_at = None
        self.get_logger().info('Detector started: ' + ' '.join(cmd))
        return True, 'detector started'

    def _terminate_detector(self):
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
    # TF helpers
    # ==================================================================
    def _tf_pose(self, parent: str, child: str, check_stale=False):
        """Pose cua `child` trong `parent`: (x, y, yaw) hoac None."""
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, Time())
        except Exception:
            return None
        if check_stale:
            age = (self.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds * 1e-9
            if tf.header.stamp.sec != 0 and age > self.p['tf_stale_s']:
                return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, yaw_from_quat(q)

    def _robot_pose(self, frame: str):
        return self._tf_pose(frame, self.p['base_frame'], check_stale=True)

    def _map_yaw_err(self) -> Optional[float]:
        """yaw pallet (map, luc chot) - yaw xe trong map hien tai (deg). ~0 = xe song song pallet
        THEO AMCL. Khac so voi err.yaw (theo odom dong bang) neu odom troi hoac AMCL doi."""
        if not self._result:
            return None
        r = self._robot_pose(self.p['map_frame'])
        if r is None:
            return None
        return math.degrees(norm_angle(self._result['yaw'] - r[2]))

    def _map_to_ins(self, x, y, yaw):
        """Doi pose tu map sang insertion_frame bang TF map->odom hien tai."""
        ins, mp = self.p['insertion_frame'], self.p['map_frame']
        if ins == mp:
            return x, y, yaw
        tf = self._tf_pose(ins, mp)
        if tf is None:
            return None
        tx, ty, th = tf
        c, s = math.cos(th), math.sin(th)
        return c * x - s * y + tx, s * x + c * y + ty, norm_angle(yaw + th)

    # ==================================================================
    # Detect on / off
    # ==================================================================
    def _start_detect(self) -> Tuple[bool, str]:
        if self._state in BUSY_STATES:
            return False, f'dang ban ({self._state}), khong the detect'
        if self._state == DETECTING:
            return True, 'dang detect'
        self._reload_params(recompute_stage=True)
        ok, why = self._start_detector()
        if not ok:
            self._set_state(FAILED, why)
            return False, why
        self._samples.clear()
        self._tf_hist.clear()
        self._result = None
        self._pre = None
        self._pallet_ins = None
        if self.p['lower_on_detect']:
            self._esp32(self.p['lift_home_cmd'])        # ha lift ve Home truoc khi di vao pallet
        self._set_state(DETECTING, 'Dang detect + ha lift ve Home... (xe phai DUNG YEN, cho detector nap model)')
        return True, 'detect ON'

    def _tf_spread(self, window_s: float):
        """Do nhay cua map->odom trong cua so cuoi luc detect: (m, deg)."""
        h = list(self._tf_hist)
        if len(h) < 2:
            return 0.0, 0.0
        t_last = h[-1][0]
        w = [s for s in h if s[0] >= t_last - window_s]
        xs = [s[1] for s in w]
        ys = [s[2] for s in w]
        yw = [s[3] for s in w]
        ref = yw[-1]
        dy = [abs(norm_angle(v - ref)) for v in yw]
        return math.hypot(max(xs) - min(xs), max(ys) - min(ys)), math.degrees(max(dy))

    def _stop_detect(self) -> Tuple[bool, str]:
        if self._state != DETECTING:
            return False, 'khong o trang thai detect'
        self._terminate_detector()
        self._reload_params(recompute_stage=True)
        est, why = estimate_pallet(
            list(self._samples), time.monotonic(),
            self.p['final_window_s'], int(self.p['min_samples']),
            self.p['max_sample_age_s'], self.p['max_pos_std_m'],
            self.p['max_yaw_std_deg'])
        if est is None:
            self._set_state(IDLE, f'Detect that bai: {why}')
            return False, f'Detect that bai: {why}'

        # ---- vi tri pallet trong khung xe + hieu chinh lech do pallet NGHIENG (detector) ----
        skew_txt = ''
        self._calib = None
        rp = self._robot_pose(self.p['map_frame'])
        if rp is None:
            skew_txt = ' [thieu TF map->base: khong bu nghieng, khong calibrate duoc]'
        else:
            xr, yr, th = rp
            dx, dy = est['x'] - xr, est['y'] - yr
            xb = math.cos(th) * dx + math.sin(th) * dy
            yb = -math.sin(th) * dx + math.cos(th) * dy
            phi = norm_angle(est['yaw'] - th)
            xb2, yb2 = xb, yb
            if self.p['skew_corr_gain'] != 0.0:
                xb2, yb2 = correct_skew_bias(
                    xb, yb, phi, self.p['camera_x_m'], self.p['pallet_width_m'],
                    self.p['obb_shrink'], self.p['depth_percentile'], self.p['skew_corr_gain'])
                est['x'] = xr + math.cos(th) * xb2 - math.sin(th) * yb2
                est['y'] = yr + math.sin(th) * xb2 + math.cos(th) * yb2
                skew_txt = (f' [bu nghieng {math.degrees(phi):+.0f}deg: doc {(xb2 - xb) * 100:+.1f}cm '
                            f'ngang {(yb2 - yb) * 100:+.1f}cm]')
                self.get_logger().info(f'[diag] pallet nghieng {math.degrees(phi):+.1f} deg so voi xe, '
                                       f'bu detector: truoc {xb:.3f},{yb:.3f} -> sau {xb2:.3f},{yb2:.3f} (khung xe)')
            self._calib = (yb2, phi)

        # ---- bu sai so he thong ----
        x, y, yaw = apply_offsets(est['x'], est['y'], est['yaw'],
                                  self.p['pallet_lat_offset_m'], self.p['pallet_yaw_offset_deg'])
        est['x'], est['y'], est['yaw'] = x, y, yaw

        # ---- AMCL (map->odom) co nhay trong luc detect khong? ----
        jm, jd = self._tf_spread(self.p['final_window_s'])
        tf_ok = jm <= self.p['max_tf_jump_m'] and jd <= self.p['max_tf_jump_deg']
        if not tf_ok:
            est['stable'] = False

        txt = (f"x={x:.3f} y={y:.3f} yaw={math.degrees(yaw):.1f}deg{skew_txt} "
               f"[bu ngang {self.p['pallet_lat_offset_m'] * 100:+.1f}cm, bu yaw {self.p['pallet_yaw_offset_deg']:+.1f}deg] "
               f"(n={est['n']}, std xy={est['pos_std'] * 100:.1f}cm, "
               f"yaw={est['yaw_std_deg']:.1f}deg, AMCL nhay {jm * 100:.1f}cm/{jd:.1f}deg)")
        usable = est['stable'] or not self.p['require_stable']

        # ---- DONG BANG pallet vao khung odom ngay luc xe dang dung yen ----
        ins = self._map_to_ins(x, y, yaw)
        if ins is None and usable:
            self._result = est
            self._set_state(IDLE, 'Khong doc duoc TF map->odom luc chot -> detect lai')
            return False, 'Khong doc duoc TF map->odom luc chot'
        self._pallet_ins = ins

        self._result = est
        d_back = self.stage_m - self.p['face_offset_m']
        px, py, pyaw = approach_pose(x, y, yaw, d_back)
        self._pre = (px, py, pyaw)
        self._publish_pose(self.pub_pallet, x, y, yaw)
        self._publish_pose(self.pub_pre, px, py, pyaw)
        if usable:
            self._set_state(READY, txt)
            return True, txt
        reason = 'AMCL dang nhay' if not tf_ok else 'KHONG ON DINH'
        self._set_state(IDLE, f'{reason}: ' + txt)
        return False, f'{reason} (detect lai): ' + txt

    # ==================================================================
    # Go to pallet (Nav2)
    # ==================================================================
    def _go_to_pallet(self) -> Tuple[bool, str]:
        if self._state != READY or self._pre is None or self._pallet_ins is None:
            return False, f'chua co ket qua hop le (state={self._state})'
        self._reload_params(recompute_stage=False)
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
        self._handoff_t0 = None
        self._set_state(NAVIGATING,
                        f'Dang toi diem dem ({px:.2f}, {py:.2f}, {math.degrees(pyaw):.0f}deg), '
                        f'cach mat pallet {self.stage_m:.2f} m')
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
        if self._state != NAVIGATING:
            if handle and handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle or not handle.accepted:
            self._set_state(FAILED, 'Nav2 tu choi goal')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _dist_to_pre(self) -> Optional[float]:
        pose = self._robot_pose(self.p['map_frame'])
        if pose is None or self._pre is None:
            return None
        return math.hypot(pose[0] - self._pre[0], pose[1] - self._pre[1])

    def _start_settling(self, msg: str):
        self._settle_t0 = time.monotonic()
        self._settle_tf = []
        self._set_state(SETTLING, msg)

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
            self._start_settling('Da toi diem dem, cho on dinh...')
        elif status == GoalStatus.STATUS_CANCELED:
            self._set_state(STOPPED, 'Nav2 bi huy')
        else:
            d = self._dist_to_pre()
            if d is not None and d <= self.p['nav_abort_ok_m']:
                self.get_logger().warn(f'Nav2 status={status} nhung xe chi cach diem dem {d:.2f} m '
                                       f'-> tu canh chinh tiep')
                self._start_settling(f'Nav2 khong dat dich chinh xac (con {d * 100:.0f} cm) -> tu canh')
            else:
                self._set_state(FAILED, f'Nav2 that bai (status={status}, cach diem dem '
                                        f'{"?" if d is None else f"{d:.2f}"} m)')

    def _handoff_check(self):
        """Nav2 nhich tung ti gan diem dem (>= handoff_hold_s ma chua xong) -> cat Nav2,
        controller rieng lam tiep. Nav2 tu toi dich duoc thi KHONG bi cat (khong cancel)."""
        h = self.p['handoff_m']
        if h <= 0.0 or self._goal_handle is None:
            self._handoff_t0 = None
            return
        d = self._dist_to_pre()
        if d is None or d > h:
            self._handoff_t0 = None
            return
        now = time.monotonic()
        if self._handoff_t0 is None:
            self._handoff_t0 = now
            return
        if now - self._handoff_t0 < self.p['handoff_hold_s']:
            return
        handle = self._goal_handle
        self._goal_handle = None
        self._handoff_t0 = None
        self._start_settling(f'Nav2 nhich o {d * 100:.0f} cm cach diem dem > '
                             f'{self.p["handoff_hold_s"]:.0f}s -> cat Nav2, tu canh chinh')
        try:
            handle.cancel_goal_async()
        except Exception:
            pass

    # ==================================================================
    # Alignment + insertion (dung ApproachController)
    # ==================================================================
    def _begin_alignment(self):
        if not self.p['auto_insert']:
            self._set_state(DONE, 'Da toi diem dem (auto_insert=false)')
            return
        stored = self._pallet_ins
        # kiem tra nhat quan: AMCL co nhay lon / odom bi reset tu luc detect khong
        now_ins = self._map_to_ins(self._result['x'], self._result['y'], self._result['yaw'])
        if now_ins is not None and stored is not None:
            shift = math.hypot(now_ins[0] - stored[0], now_ins[1] - stored[1])
            dyaw = math.degrees(abs(norm_angle(now_ins[2] - stored[2])))
            if shift > self.p['amcl_shift_fail_m']:
                self._set_state(FAILED, f'map->odom doi {shift:.2f} m tu luc detect '
                                        f'(odom reset?) -> detect lai')
                return
            self.get_logger().info(f'[diag] map->odom doi {shift * 100:.1f} cm / {dyaw:.1f} deg '
                                   f'tu luc detect (yaw pallet map={math.degrees(self._result["yaw"]):.1f})')
            if shift > self.p['amcl_shift_warn_m'] or dyaw > self.p['yaw_warn_deg']:
                self.get_logger().warn(f'AMCL da dich {shift * 100:.0f} cm / {dyaw:.1f} deg tu luc '
                                       f'detect. Dang dung pose luc detect (map_yaw_trim='
                                       f'{self.p["map_yaw_trim"]}).')
        if stored is None:
            stored = now_ins
        if stored is None:
            self._set_state(FAILED, 'Khong doi duoc pose pallet sang khung odom (TF)')
            return
        self._line = stored
        self._ctl = ApproachController(self.p, self.stage_m)
        self._tf_miss = 0
        self._phase_shown = None
        self._trimmed = False
        self._set_state(ALIGNING, 'Bat dau canh chinh x, y, yaw...')

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
            self._cmd(0.0, 0.0)
            self._tf_miss += 1
            if self._tf_miss >= 6:
                self._finish(FAILED, 'Mat TF robot -> dung xe')
            return
        self._tf_miss = 0
        us_fresh = (now - self._us_t) <= self.p['range_stale_s']
        vx, wz, done, fail = self._ctl.step(now, pose, self._line, self._us_count, us_fresh)

        for line in self._ctl.log:
            self.get_logger().info(line)
        self._ctl.log.clear()

        if fail:
            e_lat, e_yaw, a = self._ctl.err
            self._finish(FAILED, f'{fail} [lat={e_lat * 100:+.1f}cm yaw={math.degrees(e_yaw):+.1f}deg]')
            return
        if done:
            self._ins_msg = done
            self._halt()
            if self.p['auto_lift']:
                self._begin_lift()          # nang -> lui -> Home -> ha lift -> DONE
            else:
                self._set_state(DONE, 'XONG: ' + done)
            return

        ph = self._ctl.phase
        if ph == 'CONVERGE' and not self._trimmed:
            self._trimmed = True
            my = self._map_yaw_err()
            now_ins = self._map_to_ins(self._result['x'], self._result['y'], self._result['yaw'])
            if now_ins is not None:
                delta = math.degrees(norm_angle(now_ins[2] - self._line[2]))
                self.get_logger().info(
                    f'[diag] xe canh xong theo odom; yaw MAP: pallet - xe = '
                    f'{"?" if my is None else f"{my:+.1f}"} deg | duong pallet doi theo map->odom hien tai: {delta:+.1f} deg')
                if self.p['map_yaw_trim'] and abs(delta) > 0.5:
                    self._line = (self._line[0], self._line[1], now_ins[2])
                    self._halt()
                    self._ctl.restart_align(now)
                    self.get_logger().warn(f'map_yaw_trim: chinh huong duong pallet {delta:+.1f} deg theo yaw MAP, canh lai')
                    self._phase_shown = None
                    return
        if ph in ('GATE', 'LOCK') and ph != self._phase_shown:
            my = self._map_yaw_err()
            self.get_logger().info(f'[diag] {ph}: lech yaw theo odom={math.degrees(self._ctl.err[1]):+.1f} deg, '
                                   f'theo MAP={"?" if my is None else f"{my:+.1f}"} deg, '
                                   f'lat={self._ctl.err[0] * 100:+.1f} cm')
        if ph != self._phase_shown:
            self._phase_shown = ph
            top = ALIGNING if ph in ApproachController.ALIGN_PHASES else INSERTING
            self._set_state(top, PHASE_TEXT.get(ph, ph))
        self._cmd(vx, wz)

    # ==================================================================
    # Lift (nang / ha)
    # ==================================================================
    def _esp32(self, cmd: str):
        m = String()
        m.data = cmd
        self.pub_esp32.publish(m)

    def _lift_stop(self):
        self._esp32('M0')
        self._esp32('M0')

    def _begin_lift(self, target=None, home: bool = False):
        """home=False: nang len `target` cm (mac dinh lift_height_cm). home=True: ha ve 0."""
        h = self._lift_height_now()
        if h is None:
            self._set_state(FAILED, f'{self._ins_msg} | KHONG co /joint_states cua lift '
                                    f'-> khong dieu khien lift')
            return
        target = float(self.p['lift_height_cm'] if target is None else target)
        self._lift_goal = target
        self._lift_is_home = home
        self._lift_h0 = h
        self._lift_moved = False
        self._lift_t0 = time.monotonic()
        if home:
            self._esp32(self.p['lift_home_cmd'])
            self._set_state(LOWERING, f'Ha lift ve Home (dang o {h:.1f} cm)...')
        else:
            self._esp32(f'H{target:g}')
            self._set_state(LIFTING, f'Nang lift len {target:.0f} cm (dang o {h:.1f} cm)...')

    def _lift_control(self, now: float):
        h = self._lift_height_now()
        if h is None:
            self._lift_stop()
            self._set_state(FAILED, 'Mat du lieu /joint_states (lift) -> dung lift')
            return
        tol = self.p['lift_home_tol_cm'] if self._lift_is_home else self.p['lift_tol_cm']
        if abs(h - self._lift_goal) <= tol:
            if self._lift_is_home:
                self._set_state(DONE, f'XONG chu trinh: {self._ins_msg} | lift ve {h:.1f} cm')
            else:
                self._begin_backoff()       # du chieu cao -> lui ra
            return
        if abs(h - self._lift_h0) > self.p['lift_move_eps_cm']:
            self._lift_moved = True
        if not self._lift_moved and now - self._lift_t0 > self.p['lift_start_timeout_s']:
            self._lift_stop()
            self._set_state(FAILED, 'Lift khong chuyen dong - da bam Lift HOME chua '
                                    '(ESP32 reset thi mat HOME)?')
            return
        if now - self._lift_t0 > self.p['lift_timeout_s']:
            self._lift_stop()
            self._set_state(FAILED, f'Lift qua thoi gian (dang o {h:.1f} cm, can {self._lift_goal:.1f})')

    # ==================================================================
    # Lui ra sau khi nang
    # ==================================================================
    def _begin_backoff(self):
        pose = self._robot_pose(self.p['insertion_frame'])
        if pose is None:
            self._set_state(FAILED, 'Mat TF robot, khong lui duoc')
            return
        d = float(self.p['backoff_m'])
        self._bo_dist = d if d > 0.0 else float(self.p['handoff_m'])
        self._bo_ref = (pose[0], pose[1])
        self._bo_t0 = time.monotonic()
        self._bo_tf_miss = 0
        self._set_state(BACKING, f'Da nang du {self._lift_goal:.0f} cm -> lui {self._bo_dist * 100:.0f} cm')

    def _backoff_control(self, now: float):
        pose = self._robot_pose(self.p['insertion_frame'])
        if pose is None:
            self._cmd(0.0, 0.0)
            self._bo_tf_miss += 1
            if self._bo_tf_miss >= 6:
                self._finish(FAILED, 'Mat TF robot khi lui -> dung xe')
            return
        self._bo_tf_miss = 0
        d = math.hypot(pose[0] - self._bo_ref[0], pose[1] - self._bo_ref[1])
        if d >= self._bo_dist:
            self._halt()
            self._begin_home()
            return
        if now - self._bo_t0 > self.p['backoff_timeout_s']:
            self._finish(FAILED, f'Lui qua thoi gian (moi lui {d * 100:.0f} cm)')
            return
        self._cmd(-float(self.p['backoff_speed']), 0.0)

    # ==================================================================
    # Ve Home bang Nav2
    # ==================================================================
    def _begin_home(self):
        home = self._get_home()
        if home is None:
            self._set_state(FAILED, 'Chua dat Home: goi pallet_mission/set_home hoac dat '
                                    'home_set/home_x/home_y/home_yaw_deg')
            return
        if not self._nav.server_is_ready():
            self._set_state(FAILED, 'NavigateToPose server chua san sang, khong ve Home duoc')
            return
        hx, hy, hyaw = home
        pose = PoseStamped()
        pose.header.frame_id = self.p['map_frame']
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = hx, hy
        pose.pose.orientation.z = math.sin(hyaw / 2.0)
        pose.pose.orientation.w = math.cos(hyaw / 2.0)
        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = self._bt_path()
        fut = self._nav.send_goal_async(goal)
        fut.add_done_callback(self._on_home_goal_sent)
        self._set_state(HOMING, f'Ve Home ({hx:.2f}, {hy:.2f}, {math.degrees(hyaw):.0f}deg)')

    def _on_home_goal_sent(self, future):
        try:
            handle = future.result()
        except Exception as e:
            if self._state == HOMING:
                self._set_state(FAILED, f'Loi gui goal Home: {e}')
            return
        if self._state != HOMING:
            if handle and handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle or not handle.accepted:
            self._set_state(FAILED, 'Nav2 tu choi goal Home')
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_home_result)

    def _on_home_result(self, future):
        self._goal_handle = None
        if self._state != HOMING:
            return
        try:
            status = future.result().status
        except Exception as e:
            self._set_state(FAILED, f'Loi ket qua Nav2 (Home): {e}')
            return
        home = self._get_home()
        pose = self._robot_pose(self.p['map_frame'])
        d = None
        if home is not None and pose is not None:
            d = math.hypot(pose[0] - home[0], pose[1] - home[1])
        if status == GoalStatus.STATUS_SUCCEEDED or \
                (status != GoalStatus.STATUS_CANCELED and d is not None
                 and d <= self.p['nav_abort_ok_m']):
            self._begin_lift(0.0, home=True)    # den Home -> ha lift
        elif status == GoalStatus.STATUS_CANCELED:
            self._set_state(STOPPED, 'Nav2 (Home) bi huy')
        else:
            self._set_state(FAILED, f'Ve Home that bai (status={status}, '
                                    f'cach Home {"?" if d is None else f"{d:.2f}"} m)')

    # ==================================================================
    # Emergency stop
    # ==================================================================
    def _cancel_all_nav(self):
        try:
            if self._cancel_all_cli.service_is_ready():
                self._cancel_all_cli.call_async(CancelGoal.Request())
        except Exception as e:
            self.get_logger().warn(f'Cancel all Nav2 goals failed: {e}')

    def _emergency_stop(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
        self._cancel_all_nav()
        self._terminate_detector()
        self._halt()
        self._lift_stop()
        self._set_state(STOPPED, 'DUNG KHAN CAP (xe + lift)')

    # ==================================================================
    # Timers
    # ==================================================================
    def _tick(self):
        self._reap_detector()
        now = time.monotonic()
        if self._state == DETECTING and self._det_proc is None:
            self._set_state(FAILED, 'Detector tu thoat bat ngo')
            return
        if self._state == NAVIGATING:
            self._handoff_check()
        elif self._state == SETTLING:
            # lay mau map->odom trong luc cho on dinh (khong dung lai, chi de log)
            if now - self._settle_t0 >= self.p['settle_s']:
                self._begin_alignment()
        elif self._state in (ALIGNING, INSERTING):
            self._control(now)
        elif self._state in (LIFTING, LOWERING):
            self._lift_control(now)
        elif self._state == BACKING:
            self._backoff_control(now)
        # HOMING: do callback cua Nav2 xu ly, khong can lam gi trong tick

    def _slow_tick(self):
        if self._state == DETECTING:
            tf = self._tf_pose(self.p['insertion_frame'], self.p['map_frame'])
            if tf is not None:
                self._tf_hist.append((time.monotonic(), tf[0], tf[1], tf[2]))
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

        m = base('pallet', 0, Marker.SPHERE)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        m.scale.x = m.scale.y = m.scale.z = 0.07
        m.color = col
        arr.markers.append(m)

        hw = self.p['face_width_m'] / 2.0
        nx, ny = -math.sin(yaw), math.cos(yaw)
        m = base('pallet_face', 1, Marker.LINE_STRIP)
        m.points = [Point(x=x - hw * nx, y=y - hw * ny, z=z),
                    Point(x=x + hw * nx, y=y + hw * ny, z=z)]
        m.scale.x = 0.04
        m.color = col
        arr.markers.append(m)

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
            if self._state in (ALIGNING, INSERTING, BACKING):
                self._halt()
            if self._state in (LIFTING, LOWERING):
                self._lift_stop()
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