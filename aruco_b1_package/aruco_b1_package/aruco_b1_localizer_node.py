#!/usr/bin/env python3
"""Estimate a planar robot pose from a fixed ArUco marker on shelf slot_B1.

This node publishes a separate map -> base_footprint_aruco TF for comparison.
When enabled, it sends gated marker poses to AMCL through /initialpose.
It never publishes map -> odom.

Conventions:
 slot_B1: +X into shelf, +Y robot's left, +Z up
 marker: +X right as seen by robot, +Y up, +Z toward robot
 T_slot_marker rotation: marker X = -slot Y; marker Y = +slot Z;
                         marker Z = -slot X.
"""
import json
import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                      ReliabilityPolicy, qos_profile_sensor_data)
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import (Buffer, StaticTransformBroadcaster, TransformBroadcaster,
                     TransformListener, ExtrapolationException,
                     LookupException, ConnectivityException)

from aruco_b1_package.anchor_policy import (
    ObservationWindow, acknowledgement_matches, bounded_pose_update,
    compose_pose, correction_reason, map_odom_from_marker)


def quat_matrix(q):
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    n = x*x + y*y + z*z + w*w
    if n < 1e-12:
        raise ValueError('Invalid zero quaternion')
    s = 2.0/n
    return np.array([
        [1-s*(y*y+z*z), s*(x*y-z*w), s*(x*z+y*w)],
        [s*(x*y+z*w), 1-s*(x*x+z*z), s*(y*z-x*w)],
        [s*(x*z-y*w), s*(y*z+x*w), 1-s*(x*x+y*y)]
    ], dtype=np.float64)


def tf_matrix(trans):
    out = np.eye(4)
    out[:3, :3] = quat_matrix(trans.transform.rotation)
    t = trans.transform.translation
    out[:3, 3] = (t.x, t.y, t.z)
    return out


def pose_matrix(R, t):
    out = np.eye(4)
    out[:3, :3] = R
    out[:3, 3] = np.asarray(t).reshape(3)
    return out


def yaw_of(R):
    return math.atan2(float(R[1, 0]), float(R[0, 0]))


def planar_of(T):
    return float(T[0, 3]), float(T[1, 3]), yaw_of(T[:3, :3])


def matrix_quat(R):
    """Return normalized quaternion [x,y,z,w] from a rotation matrix."""
    m = np.asarray(R, dtype=np.float64)
    tr = float(np.trace(m))
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        q = np.array([(m[2, 1]-m[1, 2])/s, (m[0, 2]-m[2, 0])/s,
                      (m[1, 0]-m[0, 1])/s, s/4.0])
    else:
        i = int(np.argmax(np.diag(m)))
        if i == 0:
            s = math.sqrt(1.0+m[0, 0]-m[1, 1]-m[2, 2])*2.0
            q = np.array([s/4.0, (m[0, 1]+m[1, 0])/s,
                          (m[0, 2]+m[2, 0])/s, (m[2, 1]-m[1, 2])/s])
        elif i == 1:
            s = math.sqrt(1.0+m[1, 1]-m[0, 0]-m[2, 2])*2.0
            q = np.array([(m[0, 1]+m[1, 0])/s, s/4.0,
                          (m[1, 2]+m[2, 1])/s, (m[0, 2]-m[2, 0])/s])
        else:
            s = math.sqrt(1.0+m[2, 2]-m[0, 0]-m[1, 1])*2.0
            q = np.array([(m[0, 2]+m[2, 0])/s, (m[1, 2]+m[2, 1])/s,
                          s/4.0, (m[1, 0]-m[0, 1])/s])
    return q / np.linalg.norm(q)


def angle_diff(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def angular_mean(xs):
    return math.atan2(float(np.mean(np.sin(xs))), float(np.mean(np.cos(xs))))


def bottom_roi_start(image_height, bottom_fraction):
    """Top Y of a fixed bottom-image ROI; pixel coordinates remain full-frame.

    A bottom_fraction of 0.40 processes rows [0.60*H, H), while 1.0
    processes the entire image. Never change CameraInfo K or D for this ROI.
    """
    fraction = float(bottom_fraction)
    if (image_height <= 0 or not math.isfinite(fraction)
            or not 0.10 <= fraction <= 1.0):
        raise ValueError('roi_bottom_fraction must be between 0.10 and 1.0')
    return min(image_height - 1, max(0, int(round(image_height * (1.0 - fraction)))))


class ArucoB1Localizer(Node):
    def __init__(self):
        super().__init__('aruco_b1_localizer')
        p = self.declare_parameter
        p('image_topic', '/color/image_raw')
        p('camera_info_topic', '/color/camera_info')
        p('map_frame', 'map')
        p('base_frame', 'base_footprint')
        p('odom_frame', 'odom')
        p('slot_frame', 'slot_B1')
        p('marker_frame', 'aruco_B1')
        p('aruco_base_frame', 'base_footprint_aruco')
        p('marker_id', 0)
        p('dictionary', 'DICT_4X4_50')
        p('marker_size_m', 0.15)  # printed BLACK SQUARE, not paper including white margin
        p('marker_xyz_slot_m', [0.0, 0.0, 0.115])
        p('search_hz', 5.0)
        p('track_hz', 12.0)
        # Fixed vertical crop: search only the bottom 40% of the 640x480 RGB.
        # Set 0.30 for the bottom 30%, or 1.0 to search the full frame.
        # CAUTION: physical marker height does not guarantee image row.
        p('roi_bottom_fraction', 0.40)
        # Fail-closed detection: mission supervisor enables the one PICK -> D_1
        # travel segment only. The gate must be refreshed periodically.
        p('mission_gated_detection', True)
        p('mission_enable_topic', '/aruco_b1/enable')
        p('mission_enable_timeout_s', 3.0)
        p('track_hold_s', 1.5)
        p('max_reproj_px', 1.5)
        p('min_edge_px', 35.0)
        p('max_base_z_abs_m', 0.20)
        p('max_base_tilt_deg', 10.0)
        p('confirm_frames', 4)
        p('confirm_window_s', 0.9)
        p('max_spread_xy_m', 0.10)
        p('max_spread_yaw_deg', 6.0)
        p('auto_update_amcl', False)
        p('anchor_min_xy_m', 0.05)
        p('anchor_min_yaw_deg', 3.0)
        p('anchor_max_xy_m', 0.30)
        p('anchor_max_yaw_deg', 15.0)
        p('anchor_max_age_s', 0.8)
        p('anchor_min_interval_s', 2.0)
        p('anchor_ack_timeout_s', 5.0)
        p('anchor_ack_stamp_slack_s', 1.0)
        p('anchor_retry_interval_s', 30.0)
        p('anchor_ack_xy_m', 0.12)
        p('anchor_ack_yaw_deg', 8.0)
        p('large_confirm_frames', 6)
        p('large_confirm_window_s', 1.2)
        p('large_max_spread_xy_m', 0.075)
        p('large_max_spread_yaw_deg', 4.0)
        p('large_max_reproj_px', 1.2)
        p('odom_max_age_s', 0.5)
        p('current_tf_max_age_s', 0.35)
        p('moving_max_linear_mps', 0.35)
        p('moving_max_angular_rps', 0.60)
        # A moving robot may receive small, *acknowledged* AMCL resets.
        # Nav2 active uses the same bounded step path (never a full pose jump).
        p('moving_anchor_enabled', True)
        p('allow_full_stationary_reset', False)  # opt-in only after validating mission idle
        p('moving_anchor_max_step_xy_m', 0.05)
        p('moving_anchor_max_step_yaw_deg', 2.0)
        p('moving_anchor_min_interval_s', 5.0)
        p('moving_anchor_retry_interval_s', 5.0)
        p('moving_anchor_stationary_linear_mps', 0.02)
        p('moving_anchor_stationary_angular_rps', 0.04)
        p('moving_odom_tf_max_age_s', 0.12)
        # AMCL publishes map->odom post-dated by transform_tolerance (~1 s).
        p('amcl_tf_future_tolerance_s', 1.5)
        p('moving_large_max_linear_mps', 0.05)
        p('moving_large_max_angular_rps', 0.10)
        p('nav_active_max_correction_xy_m', 0.15)
        p('nav_active_max_correction_yaw_deg', 5.0)
        p('nav_status_max_age_s', 2.0)
        p('anchor_xy_std_m', 0.06)
        p('anchor_yaw_std_deg', 4.0)
        p('tf_timeout_s', 0.0)
        # The EKF often lags image timestamps by 5-25 ms. Keep a small
        # camera queue and wait for the exact-time transform without blocking
        # the single-threaded ROS executor. Never extrapolate odom.
        p('tf_wait_max_s', 0.30)
        p('tf_image_queue_size', 8)

        gp = lambda k: self.get_parameter(k).value
        self.marker_size = float(gp('marker_size_m'))
        if self.marker_size <= 0:
            raise ValueError('marker_size_m must be positive')
        self.image_topic = str(gp('image_topic'))
        self.map_frame = str(gp('map_frame'))
        self.base_frame = str(gp('base_frame'))
        self.odom_frame = str(gp('odom_frame'))
        self.slot_frame = str(gp('slot_frame'))
        self.marker_frame = str(gp('marker_frame'))
        self.aruco_base_frame = str(gp('aruco_base_frame'))
        self.id = int(gp('marker_id'))
        self.roi_y_start_px = None
        bottom_roi_start(480, gp('roi_bottom_fraction'))  # validate at startup
        self.search_hz, self.track_hz = float(gp('search_hz')), float(gp('track_hz'))
        if self.search_hz <= 0 or self.track_hz <= self.search_hz:
            raise ValueError('Require 0 < search_hz < track_hz')
        self.bridge = CvBridge()
        self.mission_gated_detection = bool(gp('mission_gated_detection'))
        self.mission_gate_timeout_s = float(gp('mission_enable_timeout_s'))
        if self.mission_gate_timeout_s <= 0:
            raise ValueError('mission_enable_timeout_s must be positive')
        self.mission_enabled = False
        self.mission_gate_last_wall = -1e30
        self.mission_gate_effective = False
        dic_name = str(gp('dictionary'))
        if not hasattr(cv2, 'aruco'):
            raise RuntimeError('OpenCV ArUco module missing; install an OpenCV contrib build')
        if not hasattr(cv2.aruco, dic_name):
            raise ValueError('Unknown OpenCV ArUco dictionary: ' + dic_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dic_name))
        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.det_params = cv2.aruco.DetectorParameters()
        else:
            self.det_params = cv2.aruco.DetectorParameters_create()
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.det_params)
        else:
            self.detector = None
        h = self.marker_size/2.0
        # SOLVEPNP_IPPE_SQUARE requires precisely this 3D corner order.
        self.obj = np.array([[-h,h,0], [h,h,0], [h,-h,0], [-h,-h,0]], np.float64)
        self.K, self.D, self.info_frame, self.info_size = None, None, None, None
        self.last_img = None
        self.last_img_key = None  # last image processed by detection
        self.last_received_image_key = None
        self.image_queue = deque(maxlen=max(3, int(gp('tf_image_queue_size'))))
        self.frames_waiting_for_tf = 0
        self.frames_tf_expired = 0
        self.last_detect_wall = -1e30
        self.last_found_wall = -1e30
        self.observations = ObservationWindow()
        self.odom_last_wall = -1e30
        self.odom_speed = None
        self.nav_last_wall = -1e30
        self.nav_active = None
        self.static_map_slot = None
        self.static_tf_conflict = False
        self.pending_anchor = None
        self.pending_amcl_ack = None
        self.last_attempt_stamp = 0.0
        self.retry_block_until = -1e30
        self.last_correction_wall = None
        self.last_correction_ros_time = None
        self.last_anchor_result = 'no_measurement'
        self.last_relocalization_accepted = False
        self.last_delta_xy = None
        self.last_delta_yaw_deg = None
        self.last_anchor_mode = None
        self.last_requested_xy_m = None
        self.last_requested_yaw_deg = None
        self.detected = False
        self.confirmed = False
        self.last_frame_stamp = None
        self.last_quality = {}
        self.last_status_wall = -1e30
        self.last_error_wall = -1e30
        self.last_debug_pub_wall = -1e30
        self.last_anchor_wall = -1e30
        self.frames_processed = 0
        self.frames_dropped = 0
        self.processing_seconds_total = 0.0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._publish_marker_tf()
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_b1/pose', 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.status_pub = self.create_publisher(String, '/aruco_b1/status', 10)
        self.comparison_pub = self.create_publisher(String, '/aruco_b1/comparison', 10)
        self.debug_pub = self.create_publisher(
            Image, '/aruco_b1/debug_image',
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT))
        gate_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, str(gp('mission_enable_topic')),
                                 self.on_mission_enable, gate_qos)
        # Avoid even deserializing 30Hz RGB frames while this leg is OFF.
        self.image_sub = None
        self.image_qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                    reliability=ReliabilityPolicy.BEST_EFFORT)
        if not self.mission_gated_detection:
            self._ensure_image_subscription()
        self.create_subscription(CameraInfo, str(gp('camera_info_topic')), self.on_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, qos_profile_sensor_data)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                 self.on_nav_status, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl_pose, 10)
        self.create_subscription(TFMessage, '/tf_static', self.on_static_tf,
                                 QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info('B1 ArUco ID=%d side=%.3fm; SEARCH %.1f Hz / TRACK %.1f Hz; auto_update_amcl=%s' %
                               (self.id, self.marker_size, self.search_hz, self.track_hz, gp('auto_update_amcl')))
        if gp('auto_update_amcl'):
            self.get_logger().info('ArUco B1 anchor enabled; AMCL remains the map -> odom publisher')

    def on_info(self, msg):
        if msg.width <= 0 or msg.height <= 0 or msg.distortion_model != 'plumb_bob':
            return
        K = np.asarray(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.asarray(msg.d, dtype=np.float64)
        if len(D) not in (4, 5, 8, 12, 14) or not np.all(np.isfinite(K)) or not np.all(np.isfinite(D)) or K[0, 0] <= 0 or K[1, 1] <= 0:
            self.warn_throttled('Invalid RGB CameraInfo; refusing ArUco PnP')
            return
        self.K, self.D = K, D
        self.info_frame = msg.header.frame_id
        self.info_size = (int(msg.width), int(msg.height))

    def _ensure_image_subscription(self):
        if self.image_sub is None:
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self.on_image, self.image_qos)
            self.get_logger().info('ArUco RGB subscription ON (PICK -> D_1)')

    def _remove_image_subscription(self):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            self.get_logger().info('ArUco RGB subscription OFF (outside PICK -> D_1)')

    def _gate_open(self):
        if not self.mission_gated_detection:
            return True
        return (self.mission_enabled and
                time.monotonic() - self.mission_gate_last_wall <= self.mission_gate_timeout_s)

    def _clear_detection_for_gate(self):
        # Do not retain a camera frame or confirmed pose from before the next
        # mission leg. /initialpose is never sent while disabled.
        self.image_queue.clear()
        self.last_img = None
        self.observations.clear()
        self.detected = False
        self.confirmed = False
        self.last_frame_stamp = None
        self.last_quality = {}
        self.last_found_wall = -1e30
        self.pending_anchor = None
        self.pending_amcl_ack = None
        self.last_anchor_result = 'mission_disabled'
        self.mission_gate_effective = False

    def on_mission_enable(self, msg):
        self.mission_enabled = bool(msg.data)
        self.mission_gate_last_wall = time.monotonic()
        if self._gate_open():
            if not self.mission_gate_effective:
                # Start with empty evidence before subscribing to live frames.
                self._clear_detection_for_gate()
                self.mission_gate_effective = True
                self.last_detect_wall = -1e30
            self._ensure_image_subscription()
        else:
            self._remove_image_subscription()
            if self.mission_gate_effective:
                self._clear_detection_for_gate()

    def on_image(self, msg):
        """Bounded queue: retain images briefly until image-time EKF TF arrives.

        Drop oldest images under load. Do not process duplicate/out-of-order
        camera timestamps or build an unbounded camera backlog.
        """
        if not self._gate_open():
            return  # No image queue, OpenCV, PnP or TF work outside PICK -> D_1.
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if key <= (self.last_received_image_key or (-1, -1)):
            return
        self.last_received_image_key = key
        if len(self.image_queue) == self.image_queue.maxlen:
            self.frames_dropped += 1
        self.image_queue.append(msg)
        self.last_img = msg

    def on_odom(self, msg):
        now = time.monotonic()
        gp = lambda name: float(self.get_parameter(name).value)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        twist = msg.twist.twist
        linear = math.hypot(float(twist.linear.x), float(twist.linear.y))
        angular = abs(float(twist.angular.z))
        self.odom_last_wall = now
        if (stamp <= 0 or age < -0.1 or age > gp('odom_max_age_s')
                or not all(math.isfinite(v) for v in (linear, angular))):
            self.odom_speed = None
        else:
            self.odom_speed = linear, angular

    def on_nav_status(self, msg):
        self.nav_last_wall = time.monotonic()
        self.nav_active = any(s.status in (1, 2, 3) for s in msg.status_list)

    def on_static_tf(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id == self.map_frame
                    and transform.child_frame_id == self.slot_frame):
                if self.static_tf_conflict:
                    continue
                matrix = tf_matrix(transform)
                if self.static_map_slot is not None and not np.allclose(
                        self.static_map_slot, matrix, atol=1e-6):
                    self.static_map_slot = None
                    self.static_tf_conflict = True
                    self.last_anchor_result = 'fixed_marker_tf_conflict'
                else:
                    self.static_map_slot = matrix

    def on_amcl_pose(self, msg):
        """Only ACK a reset when the AMCL pose actually moved towards it.

        Compare expected poses at the *AMCL message timestamp*, preventing
        robot movement between publish and feedback from looking like a reset.
        """
        pending = self.pending_anchor
        if pending is None or msg.header.frame_id != self.map_frame:
            return
        if time.monotonic() <= pending['sent_wall']:
            return
        # Save this ACK until the EKF transform at its timestamp arrives.
        # A future-extrapolation error of a few milliseconds is not an ACK
        # failure, nor a reason to reset the confirmation window.
        self.pending_amcl_ack = msg
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp < pending['sent_ros'] - float(
                self.get_parameter('anchor_ack_stamp_slack_s').value):
            return
        q = msg.pose.pose.orientation
        amcl_pose = (float(msg.pose.pose.position.x),
                     float(msg.pose.pose.position.y),
                     2.0 * math.atan2(float(q.z), float(q.w)))
        try:
            odom_at_ack = planar_of(self.matrix_tf(
                self.odom_frame, self.base_frame, msg.header.stamp))
        except Exception:
            return
        before = compose_pose(pending['old_map_odom'], odom_at_ack)
        desired = compose_pose(pending['map_odom'], odom_at_ack)
        gp = lambda name: float(self.get_parameter(name).value)
        if acknowledgement_matches(amcl_pose, before, desired,
                                   gp('anchor_ack_xy_m'), gp('anchor_ack_yaw_deg')):
            self.last_correction_wall = time.monotonic()
            self.last_correction_ros_time = self.get_clock().now().nanoseconds * 1e-9
            retry_interval = (gp('moving_anchor_retry_interval_s')
                              if pending['bounded'] else gp('anchor_retry_interval_s'))
            self.retry_block_until = self.last_correction_wall + retry_interval
            self.last_relocalization_accepted = True
            self.last_anchor_result = 'accepted_bounded_step' if pending['bounded'] else 'accepted'
            self.pending_anchor = None
            self.pending_amcl_ack = None

    def warn_throttled(self, message):
        now = time.monotonic()
        if now-self.last_error_wall > 4.0:
            self.get_logger().warn(message)
            self.last_error_wall = now

    def status(self, **data):
        now = time.monotonic()
        if now-self.last_status_wall < 0.4:
            return
        self.last_status_wall = now
        out = String()
        data.setdefault('avg_processing_ms', self.average_processing_ms)
        data.setdefault('roi_bottom_fraction', self.get_parameter('roi_bottom_fraction').value)
        data.setdefault('roi_y_start_px', self.roi_y_start_px)
        data.setdefault('mission_detection_enabled', bool(self._gate_open()))
        data.setdefault('mission_gated_detection', self.mission_gated_detection)
        data.setdefault('mission_gate_age_s', None if self.mission_gate_last_wall < 0
                        else round(now - self.mission_gate_last_wall, 3))
        data.setdefault('frames_dropped', self.frames_dropped)
        data.setdefault('frames_waiting_tf', self.frames_waiting_for_tf)
        data.setdefault('frames_tf_expired', self.frames_tf_expired)
        data.setdefault('queued_images', len(self.image_queue))
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        data.setdefault('marker_detected', self.detected)
        data.setdefault('confirmed', self.confirmed and self.last_frame_stamp is not None
                        and 0 <= ros_now - self.last_frame_stamp <=
                        float(self.get_parameter('anchor_max_age_s').value))
        data.setdefault('sample_age_s', None if self.last_frame_stamp is None else
                        round(ros_now - self.last_frame_stamp, 3))
        data.setdefault('quality', self.last_quality)
        data.setdefault('delta_xy_m', self.last_delta_xy)
        data.setdefault('delta_yaw_deg', self.last_delta_yaw_deg)
        data.setdefault('correction_mode', self.last_anchor_mode)
        data.setdefault('requested_step_xy_m', self.last_requested_xy_m)
        data.setdefault('requested_step_yaw_deg', self.last_requested_yaw_deg)
        data.setdefault('relocalization_attempted', self.pending_anchor is not None or
                        self.last_attempt_stamp > 0)
        data.setdefault('relocalization_accepted', self.last_relocalization_accepted)
        data.setdefault('rejection_reason', None if self.last_anchor_result in
                        ('accepted', 'accepted_bounded_step', 'anchor_sent') else self.last_anchor_result)
        data.setdefault('last_correction_time', None if self.last_correction_wall is None
                        else self.last_correction_ros_time)
        data.setdefault('amcl', self.last_anchor_result)
        out.data = json.dumps(data, ensure_ascii=False)
        self.status_pub.publish(out)

    @property
    def average_processing_ms(self):
        if self.frames_processed == 0:
            return 0.0
        return 1000.0 * self.processing_seconds_total / self.frames_processed

    def matrix_tf(self, target, source, stamp=None):
        query_time = Time.from_msg(stamp) if stamp is not None else Time()
        # Do not wait synchronously inside a single-threaded executor: that
        # prevents incoming /tf callbacks from filling the Buffer.
        trans = self.tf_buffer.lookup_transform(target, source, query_time)
        return tf_matrix(trans)

    def _current_tf_pose(self, target, source):
        trans = self.tf_buffer.lookup_transform(
            target, source, Time())
        stamp = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if (stamp <= 0 or age < -0.1 or
                age > float(self.get_parameter('current_tf_max_age_s').value)):
            raise ValueError('current_tf_stale')
        pose = planar_of(tf_matrix(trans))
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('current_tf_invalid')
        return pose

    def _current_map_odom_amcl(self):
        """AMCL's map->odom edge, not map->base at an unrelated TF timestamp."""
        trans = self.tf_buffer.lookup_transform(
            self.map_frame, self.odom_frame, Time())
        stamp = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        # AMCL stamps this edge in the future by transform_tolerance.
        if (stamp <= 0 or age > float(self.get_parameter('current_tf_max_age_s').value)
                or age < -float(self.get_parameter('amcl_tf_future_tolerance_s').value)):
            raise ValueError('amcl_map_odom_tf_stale')
        pose = planar_of(tf_matrix(trans))
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('amcl_map_odom_tf_invalid')
        return pose

    def _current_odom_pose_with_age(self):
        trans = self.tf_buffer.lookup_transform(
            self.odom_frame, self.base_frame, Time())
        stamp = trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if stamp <= 0 or age < -0.1 or age > float(
                self.get_parameter('current_tf_max_age_s').value):
            raise ValueError('odom_tf_stale')
        pose = planar_of(tf_matrix(trans))
        if not all(math.isfinite(v) for v in pose):
            raise ValueError('odom_tf_invalid')
        return pose, age, trans.header.stamp

    def _publish_marker_tf(self):
        """Publish the measured fixed marker pose as slot_B1 -> aruco_B1."""
        xyz = np.asarray(self.get_parameter('marker_xyz_slot_m').value, dtype=np.float64)
        if xyz.size != 3 or not np.all(np.isfinite(xyz)):
            raise ValueError('marker_xyz_slot_m must contain three finite values')
        # Marker +X = slot -Y; marker +Y = slot +Z; marker +Z = slot -X.
        R_slot_marker = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])
        q = matrix_quat(R_slot_marker)
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.slot_frame
        msg.child_frame_id = self.marker_frame
        msg.transform.translation.x = float(xyz[0])
        msg.transform.translation.y = float(xyz[1])
        msg.transform.translation.z = float(xyz[2])
        msg.transform.rotation.x, msg.transform.rotation.y = float(q[0]), float(q[1])
        msg.transform.rotation.z, msg.transform.rotation.w = float(q[2]), float(q[3])
        self.static_tf_broadcaster.sendTransform(msg)

    def detect_corners(self, gray, y_offset=0):
        """Detect in the cropped image but return full-camera pixel corners.

        `solvePnPGeneric` is given the original CameraInfo K and D, so the
        pixel coordinates MUST have their original image origin restored.
        """
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.det_params)
        if ids is None:
            return None
        for points, detected_id in zip(corners, ids.reshape(-1)):
            if int(detected_id) == self.id:
                full_corners = np.asarray(points, dtype=np.float64).reshape(4, 2).copy()
                full_corners[:, 1] += y_offset
                return full_corners
        return None

    def solve_marker_pose(self, corners):
        try:
            result = cv2.solvePnPGeneric(
                self.obj, corners, self.K, self.D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not result[0]:
                return None
            rvecs, tvecs = result[1], result[2]
        except cv2.error:
            return None
        candidates = []
        for rvec, tvec in zip(rvecs, tvecs):
            if not np.all(np.isfinite(rvec)) or not np.all(np.isfinite(tvec)):
                continue
            R_cam_marker, _ = cv2.Rodrigues(rvec)
            t = np.asarray(tvec, dtype=np.float64).reshape(3)
            if np.min((R_cam_marker @ self.obj.T + t[:, None])[2, :]) <= 0:
                continue
            projection, _ = cv2.projectPoints(self.obj, rvec, tvec, self.K, self.D)
            error = float(np.sqrt(np.mean(np.sum(
                (projection.reshape(-1, 2)-corners)**2, axis=1))))
            if np.isfinite(error) and error <= float(self.get_parameter('max_reproj_px').value):
                candidates.append((error, rvec, tvec, R_cam_marker))
        return min(candidates, key=lambda item: item[0]) if candidates else None

    def candidate(self, marker_pose, camera_frame, image_stamp):
        error, rvec, tvec, R_cam_marker = marker_pose
        T_base_cam = self.matrix_tf(self.base_frame, camera_frame, image_stamp)
        # Require the direct map -> slot_B1 /tf_static edge. A transform routed
        # through AMCL's base/odom would make this anchor self-referential.
        if self.static_map_slot is None:
            raise ValueError('fixed_marker_tf_unavailable')
        T_map_slot = self.static_map_slot
        # Columns are marker axes expressed in slot_B1 coordinates.
        R_sm = np.array([[0.,0.,-1.], [-1.,0.,0.], [0.,1.,0.]])
        marker_xyz = np.asarray(self.get_parameter('marker_xyz_slot_m').value, dtype=np.float64)
        if marker_xyz.size != 3 or not np.all(np.isfinite(marker_xyz)):
            raise ValueError('marker_xyz_slot_m must be three finite numbers')
        T_map_marker = T_map_slot @ pose_matrix(R_sm, marker_xyz)
        T_map_base = T_map_marker @ np.linalg.inv(
            T_base_cam @ pose_matrix(R_cam_marker, tvec))
        R = T_map_base[:3, :3]
        # Preserve the full 3D estimate for validation, then publish planar pose.
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        pitch = math.asin(max(-1.0, min(1.0, -float(R[2, 0]))))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))
        z = float(T_map_base[2, 3])
        if (abs(z) > float(self.get_parameter('max_base_z_abs_m').value) or
                tilt > float(self.get_parameter('max_base_tilt_deg').value)):
            return None
        x, y = map(float, T_map_base[:2, 3])
        return (error, x, y, yaw_of(R), z, math.degrees(roll),
                math.degrees(pitch), tilt, rvec, tvec)

    def stable(self, now):
        gp = lambda name: float(self.get_parameter(name).value)
        estimate, reason, quality = self.observations.evaluate(
            now, int(gp('confirm_frames')), gp('confirm_window_s'),
            gp('anchor_max_age_s'), gp('max_spread_xy_m'),
            gp('max_spread_yaw_deg'), gp('max_reproj_px'),
            after_stamp=self.last_attempt_stamp)
        self.last_quality = quality
        if estimate is None:
            self.last_anchor_result = reason
        return estimate

    def _tf_ready_for_image(self, msg):
        """Non-blocking exact-stamp TF availability check.

        No lookup at time=0 is allowed for image-time odometry; that would
        make moving-robot pose estimation inconsistent.
        """
        if self.K is None:
            return True  # process_frame will report waiting_camera_info
        camera_frame = msg.header.frame_id or self.info_frame
        if not camera_frame:
            return True  # process_frame reports the camera frame mismatch
        stamp = Time.from_msg(msg.header.stamp)
        try:
            return (self.tf_buffer.can_transform(self.base_frame, camera_frame, stamp)
                    and self.tf_buffer.can_transform(self.odom_frame, self.base_frame, stamp))
        except (LookupException, ExtrapolationException, ConnectivityException):
            return False

    def tick(self):
        now = time.monotonic()
        if not self._gate_open():
            self._remove_image_subscription()
            if self.mission_gate_effective:
                self._clear_detection_for_gate()
            self.status(state='mission_disabled', hz=0.0)
            return
        self._ensure_image_subscription()
        if not self.mission_gate_effective:
            # New travel leg: do not mix old samples with this observation.
            self._clear_detection_for_gate()
            self.mission_gate_effective = True
            self.last_detect_wall = -1e30
        self._check_pending_anchor(now)
        if self.pending_anchor is not None and self.pending_amcl_ack is not None:
            self.on_amcl_pose(self.pending_amcl_ack)
        active = now-self.last_found_wall < float(self.get_parameter('track_hold_s').value)
        hz = self.track_hz if active else self.search_hz
        if now-self.last_detect_wall < 1.0/hz:
            return
        if not self.image_queue:
            if now-self.last_found_wall > float(self.get_parameter('track_hold_s').value):
                self.detected = False
                self.confirmed = False
                self.observations.clear()
            self.status(state='waiting_camera', hz=hz)
            return

        ros_now = self.get_clock().now().nanoseconds * 1e-9
        max_age = float(self.get_parameter('anchor_max_age_s').value)
        max_wait = float(self.get_parameter('tf_wait_max_s').value)
        # The newest available image avoids processing old camera backlog.
        # Retain not-yet-ready images, but at most max_wait/max_age seconds.
        selected = None
        for msg in reversed(self.image_queue):
            stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            age = ros_now - stamp_s
            if 0 <= age <= max_age and self._tf_ready_for_image(msg):
                selected = msg
                break

        if selected is None:
            while self.image_queue:
                oldest = self.image_queue[0]
                stamp_s = oldest.header.stamp.sec + oldest.header.stamp.nanosec * 1e-9
                age = ros_now - stamp_s
                if -0.1 <= age <= min(max_age, max_wait):
                    break
                self.image_queue.popleft()
                self.frames_tf_expired += 1
            self.frames_waiting_for_tf += 1
            self.last_anchor_result = 'waiting_image_time_tf'
            self.status(state='waiting_image_time_tf', hz=hz)
            return

        selected_key = (selected.header.stamp.sec, selected.header.stamp.nanosec)
        while self.image_queue:
            head = self.image_queue.popleft()
            key = (head.header.stamp.sec, head.header.stamp.nanosec)
            if key == selected_key:
                break
            self.frames_dropped += 1
        self.last_img_key = selected_key
        self.last_detect_wall = now
        start = time.monotonic()
        try:
            if self.K is None:
                bgr = self.bridge.imgmsg_to_cv2(selected, desired_encoding='bgr8')
                self.status(state='waiting_camera_info', hz=hz)
                self._publish_debug_image(bgr, selected.header,
                                          'SEARCHING - WAITING CAMERA INFO')
                return
            self.process_frame(selected, now, hz)
        finally:
            self.frames_processed += 1
            self.processing_seconds_total += time.monotonic() - start

    def process_frame(self, msg, now, hz):
        image_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        age = ros_now - image_stamp
        if image_stamp <= 0 or age < -0.1 or age > float(self.get_parameter('anchor_max_age_s').value):
            self.detected = False
            self.confirmed = False
            self.last_anchor_result = 'stale_image'
            self.status(state='stale_image', hz=hz, sample_age_s=round(age, 3))
            return
        if self.info_size != (int(msg.width), int(msg.height)):
            self.warn_throttled('CameraInfo resolution does not match RGB image')
            self.observations.clear()
            self.confirmed = False
            return
        camera_frame = msg.header.frame_id or self.info_frame
        if self.info_frame and camera_frame != self.info_frame:
            self.warn_throttled('RGB image frame does not match RGB CameraInfo frame')
            self.observations.clear()
            self.confirmed = False
            return
        bgr = None
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Hard ROI: do NOT convert or run ArUco detection over the top
            # of the frame. Keep image and CameraInfo coordinates unchanged.
            y_start = bottom_roi_start(
                bgr.shape[0], self.get_parameter('roi_bottom_fraction').value)
            self.roi_y_start_px = y_start
            gray_roi = cv2.cvtColor(bgr[y_start:, :], cv2.COLOR_BGR2GRAY)
            corners = self.detect_corners(gray_roi, y_offset=y_start)
            if corners is None:
                self.detected = False
                if now-self.last_found_wall > float(self.get_parameter('track_hold_s').value):
                    self.observations.clear()
                    self.confirmed = False
                self.status(state='searching', hz=hz)
                self._publish_debug_image(bgr, msg.header, 'SEARCHING')
                return
            self.detected = True
            self.last_frame_stamp = image_stamp
            side = float(np.mean(np.linalg.norm(corners-np.roll(corners,-1,axis=0),axis=1)))
            if side < float(self.get_parameter('min_edge_px').value):
                self.confirmed = False
                self.status(state='marker_too_small', hz=hz, edge_px=round(side,1))
                self._publish_debug_image(bgr, msg.header, 'SEARCHING - MARKER TOO SMALL')
                return
            self.last_found_wall = now
            marker_pose = self.solve_marker_pose(corners)
            if marker_pose is None:
                self.confirmed = False
                debug = self._draw_marker(bgr, corners, None)
                self.status(state='geometry_rejected', hz=hz, edge_px=round(side,1))
                self._publish_debug_image(debug, msg.header, 'CONFIRMING - PNP REJECTED')
                return
            debug = self._draw_marker(
                bgr, corners, (marker_pose[1], marker_pose[2]))
            estimate_raw = self.candidate(marker_pose, camera_frame, msg.header.stamp)
            if estimate_raw is None:
                self.confirmed = False
                self.status(state='geometry_rejected', hz=hz, edge_px=round(side,1))
                self._publish_debug_image(debug, msg.header, 'CONFIRMING - GEOMETRY REJECTED')
                return
            error, x, y, yaw, z, roll_deg, pitch_deg, tilt, _, _ = estimate_raw
            odom_at_image = planar_of(self.matrix_tf(
                self.odom_frame, self.base_frame, msg.header.stamp))
            if not all(math.isfinite(v) for v in odom_at_image):
                raise ValueError('odom_tf_at_image_invalid')
            map_odom = map_odom_from_marker((x, y, yaw), odom_at_image)
            if not self.observations.add(image_stamp, *map_odom, error):
                self.last_anchor_result = 'duplicate_or_invalid_timestamp'
                self.status(state='timestamp_rejected', hz=hz)
                return
            est = self.stable(ros_now)
            if est is None:
                self.status(state='confirming', hz=hz, n=len(self.observations.samples), rmse_px=round(error,3),
                            z_residual_m=round(z,3), roll_deg=round(roll_deg,2),
                            pitch_deg=round(pitch_deg,2), tilt_deg=round(tilt,1))
                self._publish_debug_image(
                    debug, msg.header, 'CONFIRMING', (x, y, yaw))
                return
            self.confirmed = True
            x2, y2, yaw2 = compose_pose(est[:3], odom_at_image)
            result = PoseStamped()
            result.header.frame_id = self.map_frame
            result.header.stamp = msg.header.stamp
            result.pose.position.x = x2
            result.pose.position.y = y2
            result.pose.position.z = 0.0
            result.pose.orientation.z = math.sin(yaw2 / 2.0)
            result.pose.orientation.w = math.cos(yaw2 / 2.0)
            self.pose_pub.publish(result)
            tf_msg = TransformStamped()
            tf_msg.header = result.header
            tf_msg.child_frame_id = self.aruco_base_frame
            tf_msg.transform.translation.x = x2
            tf_msg.transform.translation.y = y2
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation.z = result.pose.orientation.z
            tf_msg.transform.rotation.w = result.pose.orientation.w
            self.tf_broadcaster.sendTransform(tf_msg)
            self._publish_comparison(x2, y2, yaw2, msg.header.stamp)
            anchor_state = self._maybe_update_amcl(result, now)
            self.status(state='stable', hz=hz, x=round(x2,3), y=round(y2,3),
                        z_residual_m=round(z,3), roll_deg=round(roll_deg,2),
                        pitch_deg=round(pitch_deg,2), yaw_deg=round(math.degrees(yaw2),2),
                        rmse_px=round(error,3), amcl=anchor_state)
            self._publish_debug_image(
                debug, msg.header, 'STABLE', (x2, y2, yaw2))
        except (LookupException, ExtrapolationException, ConnectivityException) as exc:
            # An otherwise valid camera frame may race an EKF TF publication.
            # Keep previous observations; the next frame will be processed when
            # its exact-time TF is available. Do not spam warnings.
            self.last_anchor_result = 'waiting_image_time_tf'
            self.frames_waiting_for_tf += 1
            self.status(state='waiting_image_time_tf', hz=hz)
        except Exception as exc:
            self.warn_throttled('ArUco B1 error: %s: %s' % (type(exc).__name__, exc))
            self.confirmed = False
            self.last_anchor_result = (str(exc) if str(exc) == 'fixed_marker_tf_unavailable'
                                       else 'tf_or_geometry_error')
            if bgr is not None:
                self._publish_debug_image(bgr, msg.header, 'SEARCHING - TF/GEOMETRY UNAVAILABLE')

    def _check_pending_anchor(self, now):
        if (self.pending_anchor is not None and
                now - self.pending_anchor['sent_wall'] >
                float(self.get_parameter('anchor_ack_timeout_s').value)):
            self.pending_anchor = None
            self.pending_amcl_ack = None
            self.last_anchor_result = 'amcl_ack_timeout'
            self.retry_block_until = now + float(self.get_parameter('anchor_retry_interval_s').value)

    def _maybe_update_amcl(self, pose, now):
        if not self._gate_open():
            return 'mission_disabled'
        """Publish AMCL relocalization with timestamp compensation and step limits.

        Marker localization is published independently of this function: rejected
        AMCL resets must not hide a valid ArUco pose from RViz/consumers.
        """
        if not bool(self.get_parameter('auto_update_amcl').value):
            self.last_anchor_result = 'observe_only'
            return 'observe_only'
        image_stamp = pose.header.stamp.sec + pose.header.stamp.nanosec * 1e-9
        if image_stamp <= self.last_attempt_stamp:
            self.last_anchor_result = 'already_used_measurement'
            return self.last_anchor_result
        self._check_pending_anchor(now)
        if self.pending_anchor is not None:
            self.last_anchor_result = 'awaiting_amcl_ack'
            return self.last_anchor_result
        if now < self.retry_block_until:
            self.last_anchor_result = 'retry_cooldown'
            return self.last_anchor_result
        if self.initialpose_pub.get_subscription_count() == 0:
            self.last_anchor_result = 'amcl_unavailable'
            return self.last_anchor_result
        gp = lambda name: float(self.get_parameter(name).value)
        if self.static_map_slot is None:
            self.last_anchor_result = 'fixed_marker_tf_unavailable'
            return self.last_anchor_result
        if now - self.odom_last_wall > gp('odom_max_age_s') or self.odom_speed is None:
            self.last_anchor_result = 'waiting_fresh_odometry'
            return self.last_anchor_result
        linear, angular = self.odom_speed
        if linear > gp('moving_max_linear_mps') or angular > gp('moving_max_angular_rps'):
            # ArUco /pose & TF still publish, but /initialpose is deferred.
            self.last_anchor_result = 'moving_too_fast_for_reset'
            return self.last_anchor_result

        fresh_est, fresh_reason, quality = self.observations.evaluate(
            self.get_clock().now().nanoseconds * 1e-9,
            int(gp('confirm_frames')), gp('confirm_window_s'),
            gp('anchor_max_age_s'), gp('max_spread_xy_m'),
            gp('max_spread_yaw_deg'), gp('max_reproj_px'),
            after_stamp=self.last_attempt_stamp)
        self.last_quality = quality
        if fresh_est is None:
            self.last_anchor_result = fresh_reason
            return self.last_anchor_result

        age_s = (self.get_clock().now().nanoseconds -
                 Time.from_msg(pose.header.stamp).nanoseconds) * 1e-9
        try:
            odom_now, odom_tf_age, odom_tf_stamp = self._current_odom_pose_with_age()
            map_odom_amcl = self._current_map_odom_amcl()
        except Exception:
            self.last_anchor_result = 'current_tf_unavailable'
            return self.last_anchor_result

        map_odom_est = fresh_est[:3]
        marker_pose_now = compose_pose(map_odom_est, odom_now)
        amcl_pose_now = compose_pose(map_odom_amcl, odom_now)
        error_xy = math.hypot(marker_pose_now[0] - amcl_pose_now[0],
                              marker_pose_now[1] - amcl_pose_now[1])
        error_yaw_deg = math.degrees(angle_diff(marker_pose_now[2], amcl_pose_now[2]))
        self.last_delta_xy = error_xy
        self.last_delta_yaw_deg = error_yaw_deg

        nav_fresh = now - self.nav_last_wall <= gp('nav_status_max_age_s')
        nav_active = nav_fresh and self.nav_active is True
        moving = (nav_active or linear > gp('moving_anchor_stationary_linear_mps')
                  or angular > gp('moving_anchor_stationary_angular_rps'))
        if moving and not bool(self.get_parameter('moving_anchor_enabled').value):
            self.last_anchor_result = 'moving_anchor_disabled'
            return self.last_anchor_result
        if moving and odom_tf_age > gp('moving_odom_tf_max_age_s'):
            self.last_anchor_result = 'waiting_fresh_moving_tf'
            return self.last_anchor_result
        large = (error_xy > gp('anchor_max_xy_m') or
                 abs(error_yaw_deg) > gp('anchor_max_yaw_deg'))
        strong = False
        if large:
            strong_est, strong_reason, quality = self.observations.evaluate(
                self.get_clock().now().nanoseconds * 1e-9,
                int(gp('large_confirm_frames')), gp('large_confirm_window_s'),
                gp('anchor_max_age_s'), gp('large_max_spread_xy_m'),
                gp('large_max_spread_yaw_deg'), gp('large_max_reproj_px'),
                after_stamp=self.last_attempt_stamp)
            self.last_quality = quality
            if strong_est is None:
                self.last_anchor_result = 'large_' + strong_reason
                return self.last_anchor_result
            map_odom_est = strong_est[:3]
            marker_pose_now = compose_pose(map_odom_est, odom_now)
            error_xy = math.hypot(marker_pose_now[0] - amcl_pose_now[0],
                                  marker_pose_now[1] - amcl_pose_now[1])
            error_yaw_deg = math.degrees(angle_diff(marker_pose_now[2], amcl_pose_now[2]))
            self.last_delta_xy = error_xy
            self.last_delta_yaw_deg = error_yaw_deg
            strong = True

        # A stopped robot can still be inside a pallet operation. Without a
        # reliable mission-idle signal, never assume a large reset is safe.
        bounded = moving or not bool(self.get_parameter('allow_full_stationary_reset').value)
        min_interval = (gp('moving_anchor_min_interval_s') if bounded
                        else gp('anchor_min_interval_s'))
        reason = correction_reason(
            error_xy, error_yaw_deg, age_s, now - self.last_anchor_wall,
            gp('anchor_min_xy_m'), gp('anchor_min_yaw_deg'),
            gp('anchor_max_xy_m'), gp('anchor_max_yaw_deg'),
            gp('anchor_max_age_s'), min_interval,
            strong_confirmation=strong)
        if reason != 'correct':
            self.last_anchor_result = reason
            return reason

        requested_base = marker_pose_now
        mode = 'full_stationary'
        if bounded:
            # Never inject a large map->odom jump during a mission by default.
            step_xy = gp('moving_anchor_max_step_xy_m')
            step_yaw_deg = gp('moving_anchor_max_step_yaw_deg')
            if nav_active:
                step_xy = min(step_xy, gp('nav_active_max_correction_xy_m'))
                step_yaw_deg = min(step_yaw_deg, gp('nav_active_max_correction_yaw_deg'))
            try:
                requested_base, _, step_actual_xy, step_actual_yaw = bounded_pose_update(
                    amcl_pose_now, marker_pose_now, step_xy, step_yaw_deg)
            except ValueError:
                self.last_anchor_result = 'invalid_step_limit'
                return self.last_anchor_result
            mode = 'bounded_moving' if moving else 'bounded_stationary'
        else:
            step_actual_xy = error_xy
            step_actual_yaw = abs(error_yaw_deg)

        requested_map_odom = map_odom_from_marker(requested_base, odom_now)
        update = PoseWithCovarianceStamped()
        update.header.frame_id = self.map_frame
        # Pose was computed using odom at odom_tf_stamp. Nav2 Humble AMCL
        # compensates wheel motion from this stamp to reception time.
        update.header.stamp = odom_tf_stamp
        update.pose.pose.position.x = requested_base[0]
        update.pose.pose.position.y = requested_base[1]
        update.pose.pose.position.z = 0.0
        update.pose.pose.orientation.x = 0.0
        update.pose.pose.orientation.y = 0.0
        update.pose.pose.orientation.z = math.sin(requested_base[2] / 2.0)
        update.pose.pose.orientation.w = math.cos(requested_base[2] / 2.0)
        update.pose.covariance[0] = gp('anchor_xy_std_m') ** 2
        update.pose.covariance[7] = gp('anchor_xy_std_m') ** 2
        update.pose.covariance[35] = math.radians(gp('anchor_yaw_std_deg')) ** 2
        self.initialpose_pub.publish(update)
        self.last_anchor_wall = now
        self.last_attempt_stamp = image_stamp
        self.pending_amcl_ack = None
        self.pending_anchor = {
            'sent_wall': now,
            'sent_ros': self.get_clock().now().nanoseconds * 1e-9,
            'map_odom': requested_map_odom,
            'old_map_odom': map_odom_amcl,
            'moving': moving,
            'bounded': bounded,
        }
        self.last_anchor_mode = mode
        self.last_requested_xy_m = step_actual_xy
        self.last_requested_yaw_deg = step_actual_yaw
        self.last_relocalization_accepted = False
        self.last_anchor_result = 'anchor_sent'
        self.get_logger().info(
            'ArUco B1 -> AMCL (%s): full_delta=%.3fm/%.1fdeg; request=%.3fm/%.1fdeg' %
            (mode, error_xy, error_yaw_deg, step_actual_xy, step_actual_yaw))
        return 'anchor_sent'

    def _draw_marker(self, image, corners, pose):
        if self.debug_pub.get_subscription_count() == 0:
            return image
        debug = image.copy()
        points = np.asarray(corners, dtype=np.float32).reshape(1, 4, 2)
        ids = np.array([[self.id]], dtype=np.int32)
        cv2.aruco.drawDetectedMarkers(debug, [points], ids, (0, 255, 0))
        if pose is not None:
            rvec, tvec = pose
            cv2.drawFrameAxes(debug, self.K, self.D, rvec, tvec,
                              self.marker_size * 0.5, 2)
        return debug

    def _publish_debug_image(self, image, header, state, pose=None):
        if self.debug_pub.get_subscription_count() == 0:
            return
        now = time.monotonic()
        if now-self.last_debug_pub_wall < 0.5:
            return
        self.last_debug_pub_wall = now
        debug = image.copy()
        if self.roi_y_start_px is not None:
            # Full-size debug image makes the hard ROI clearly visible.
            cv2.line(debug, (0, self.roi_y_start_px),
                     (debug.shape[1] - 1, self.roi_y_start_px),
                     (255, 160, 40), 2)
        color = (0, 255, 0) if state == 'STABLE' else (0, 220, 255)
        cv2.rectangle(debug, (0, 0), (debug.shape[1], 56), (20, 20, 20), -1)
        cv2.putText(debug, state, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62, color, 2, cv2.LINE_AA)
        if pose is not None:
            x, y, yaw = pose
            cv2.putText(debug, f'Robot map: X={x:.3f} m Y={y:.3f} m '
                        f'Yaw={math.degrees(yaw):.1f} deg',
                        (8, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.48,
                        (255, 255, 255), 1, cv2.LINE_AA)
        out = Image()
        out.header = header
        out.height, out.width = debug.shape[:2]
        out.encoding = 'bgr8'
        out.is_bigendian = 0
        out.step = out.width * 3
        out.data = np.ascontiguousarray(debug).tobytes()
        self.debug_pub.publish(out)

    def _publish_comparison(self, aruco_x, aruco_y, aruco_yaw, image_stamp):
        try:
            T_map_amcl = self.matrix_tf(self.map_frame, self.base_frame, image_stamp)
        except Exception as exc:
            self.status(state='aruco_valid_amcl_tf_unavailable', error=str(exc))
            return
        delta_x = aruco_x - float(T_map_amcl[0, 3])
        delta_y = aruco_y - float(T_map_amcl[1, 3])
        delta_yaw = math.degrees(angle_diff(aruco_yaw, yaw_of(T_map_amcl[:3, :3])))
        out = String()
        out.data = json.dumps({
            'stamp': {'sec': image_stamp.sec, 'nanosec': image_stamp.nanosec},
            'delta_x': delta_x,
            'delta_y': delta_y,
            'delta_yaw': delta_yaw,
            'error_xy': math.hypot(delta_x, delta_y),
        })
        self.comparison_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoB1Localizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
