#!/usr/bin/env python3
"""Estimate a planar robot pose from a fixed ArUco marker on shelf slot_B1.

This node publishes a separate map -> base_footprint_aruco TF for comparison.
It never updates AMCL or publishes map -> odom.

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
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformBroadcaster, TransformListener


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


class ArucoB1Localizer(Node):
    def __init__(self):
        super().__init__('aruco_b1_localizer')
        p = self.declare_parameter
        p('image_topic', '/color/image_raw')
        p('camera_info_topic', '/color/camera_info')
        p('map_frame', 'map')
        p('base_frame', 'base_footprint')
        p('slot_frame', 'slot_B1')
        p('marker_frame', 'aruco_B1')
        p('aruco_base_frame', 'base_footprint_aruco')
        p('marker_id', 0)
        p('dictionary', 'DICT_4X4_50')
        p('marker_size_m', 0.10)  # printed BLACK SQUARE, not paper including white margin
        p('marker_xyz_slot_m', [0.0, 0.0, 0.122])
        p('search_hz', 1.5)
        p('track_hz', 8.0)
        p('track_hold_s', 2.0)
        p('max_reproj_px', 1.5)
        p('min_edge_px', 35.0)
        p('max_base_z_abs_m', 0.20)
        p('max_base_tilt_deg', 10.0)
        p('confirm_frames', 6)
        p('confirm_window_s', 2.0)
        p('max_spread_xy_m', 0.08)
        p('max_spread_yaw_deg', 5.0)
        # Retained for launch compatibility; AMCL updates are not implemented.
        p('auto_update_amcl', False)
        p('tf_timeout_s', 0.12)

        gp = lambda k: self.get_parameter(k).value
        self.marker_size = float(gp('marker_size_m'))
        if self.marker_size <= 0:
            raise ValueError('marker_size_m must be positive')
        self.image_topic = str(gp('image_topic'))
        self.map_frame = str(gp('map_frame'))
        self.base_frame = str(gp('base_frame'))
        self.slot_frame = str(gp('slot_frame'))
        self.marker_frame = str(gp('marker_frame'))
        self.aruco_base_frame = str(gp('aruco_base_frame'))
        self.id = int(gp('marker_id'))
        self.search_hz, self.track_hz = float(gp('search_hz')), float(gp('track_hz'))
        if self.search_hz <= 0 or self.track_hz <= self.search_hz:
            raise ValueError('Require 0 < search_hz < track_hz')
        self.bridge = CvBridge()
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
        self.last_img_key = None
        self.last_detect_wall = -1e30
        self.last_found_wall = -1e30
        self.history = deque(maxlen=60)
        self.last_status_wall = -1e30
        self.last_error_wall = -1e30
        self.last_debug_pub_wall = -1e30
        self.frames_processed = 0
        self.frames_dropped = 0
        self.processing_seconds_total = 0.0
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._publish_marker_tf()
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco_b1/pose', 10)
        self.status_pub = self.create_publisher(String, '/aruco_b1/status', 10)
        self.comparison_pub = self.create_publisher(String, '/aruco_b1/comparison', 10)
        self.debug_pub = self.create_publisher(
            Image, '/aruco_b1/debug_image',
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(
            Image, self.image_topic, self.on_image,
            QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                       reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(CameraInfo, str(gp('camera_info_topic')), self.on_info, qos_profile_sensor_data)
        self.timer = self.create_timer(0.05, self.tick)
        self.get_logger().info('B1 ArUco ID=%d side=%.3fm; SEARCH %.1f Hz / TRACK %.1f Hz; auto_update_amcl=%s' %
                               (self.id, self.marker_size, self.search_hz, self.track_hz, gp('auto_update_amcl')))
        if gp('auto_update_amcl'):
            self.get_logger().warn('AMCL updates are intentionally disabled; only comparison outputs are published')

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

    def on_image(self, msg):
        if self.last_img is not None:
            previous = (self.last_img.header.stamp.sec, self.last_img.header.stamp.nanosec)
            if previous != self.last_img_key:
                self.frames_dropped += 1
        self.last_img = msg  # Keep only the newest frame; never queue camera backlog.

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
        data.setdefault('frames_dropped', self.frames_dropped)
        out.data = json.dumps(data, ensure_ascii=False)
        self.status_pub.publish(out)

    @property
    def average_processing_ms(self):
        if self.frames_processed == 0:
            return 0.0
        return 1000.0 * self.processing_seconds_total / self.frames_processed

    def matrix_tf(self, target, source, stamp=None):
        query_time = Time.from_msg(stamp) if stamp is not None else Time()
        trans = self.tf_buffer.lookup_transform(
            target, source, query_time,
            timeout=Duration(seconds=float(self.get_parameter('tf_timeout_s').value)))
        return tf_matrix(trans)

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

    def detect_corners(self, gray):
        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.det_params)
        if ids is None:
            return None
        for points, detected_id in zip(corners, ids.reshape(-1)):
            if int(detected_id) == self.id:
                return np.asarray(points, dtype=np.float64).reshape(4, 2)
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
        T_map_slot = self.matrix_tf(self.map_frame, self.slot_frame, image_stamp)
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
        window = float(self.get_parameter('confirm_window_s').value)
        while self.history and now-self.history[0][0] > window:
            self.history.popleft()
        n = int(self.get_parameter('confirm_frames').value)
        if len(self.history) < n:
            return None
        arr = np.array([item[1:4] for item in self.history], dtype=float)
        x, y = np.median(arr[:, :2], axis=0)
        yaw = angular_mean(arr[:, 2])
        dxy = np.linalg.norm(arr[:, :2]-np.array([x,y]), axis=1)
        dyaw = np.array([abs(angle_diff(a, yaw)) for a in arr[:, 2]])
        if max(dxy) > float(self.get_parameter('max_spread_xy_m').value):
            return None
        if math.degrees(max(dyaw)) > float(self.get_parameter('max_spread_yaw_deg').value):
            return None
        return float(x), float(y), float(yaw)

    def tick(self):
        now = time.monotonic()
        active = now-self.last_found_wall < float(self.get_parameter('track_hold_s').value)
        hz = self.track_hz if active else self.search_hz
        if now-self.last_detect_wall < 1.0/hz:
            return
        self.last_detect_wall = now
        msg = self.last_img
        if msg is None:
            self.status(state='waiting_camera', hz=hz)
            return
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if key == self.last_img_key:
            return
        self.last_img_key = key
        start = time.monotonic()
        try:
            if self.K is None:
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                self.status(state='waiting_camera_info', hz=hz)
                self._publish_debug_image(bgr, msg.header, 'SEARCHING - WAITING CAMERA INFO')
                return
            self.process_frame(msg, now, hz)
        finally:
            self.frames_processed += 1
            self.processing_seconds_total += time.monotonic() - start

    def process_frame(self, msg, now, hz):
        if self.info_size != (int(msg.width), int(msg.height)):
            self.warn_throttled('CameraInfo resolution does not match RGB image')
            self.history.clear()
            return
        camera_frame = msg.header.frame_id or self.info_frame
        if self.info_frame and camera_frame != self.info_frame:
            self.warn_throttled('RGB image frame does not match RGB CameraInfo frame')
            self.history.clear()
            return
        bgr = None
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            corners = self.detect_corners(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
            if corners is None:
                if now-self.last_found_wall > float(self.get_parameter('track_hold_s').value):
                    self.history.clear()
                self.status(state='searching', hz=hz)
                self._publish_debug_image(bgr, msg.header, 'SEARCHING')
                return
            side = float(np.mean(np.linalg.norm(corners-np.roll(corners,-1,axis=0),axis=1)))
            if side < float(self.get_parameter('min_edge_px').value):
                self.history.clear()
                self.status(state='marker_too_small', hz=hz, edge_px=round(side,1))
                self._publish_debug_image(bgr, msg.header, 'SEARCHING - MARKER TOO SMALL')
                return
            self.last_found_wall = now
            marker_pose = self.solve_marker_pose(corners)
            if marker_pose is None:
                self.history.clear()
                debug = self._draw_marker(bgr, corners, None)
                self.status(state='geometry_rejected', hz=hz, edge_px=round(side,1))
                self._publish_debug_image(debug, msg.header, 'CONFIRMING - PNP REJECTED')
                return
            debug = self._draw_marker(
                bgr, corners, (marker_pose[1], marker_pose[2]))
            estimate_raw = self.candidate(marker_pose, camera_frame, msg.header.stamp)
            if estimate_raw is None:
                self.history.clear()
                self.status(state='geometry_rejected', hz=hz, edge_px=round(side,1))
                self._publish_debug_image(debug, msg.header, 'CONFIRMING - GEOMETRY REJECTED')
                return
            error, x, y, yaw, z, roll_deg, pitch_deg, tilt, _, _ = estimate_raw
            self.history.append((now,x,y,yaw))
            est = self.stable(now)
            if est is None:
                self.status(state='confirming', hz=hz, n=len(self.history), rmse_px=round(error,3),
                            z_residual_m=round(z,3), roll_deg=round(roll_deg,2),
                            pitch_deg=round(pitch_deg,2), tilt_deg=round(tilt,1))
                self._publish_debug_image(
                    debug, msg.header, 'CONFIRMING', (x, y, yaw))
                return
            x2, y2, yaw2 = est[:3]
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
            self.status(state='stable', hz=hz, x=round(est[0],3), y=round(est[1],3),
                        z_residual_m=round(z,3), roll_deg=round(roll_deg,2),
                        pitch_deg=round(pitch_deg,2), yaw_deg=round(math.degrees(est[2]),2),
                        rmse_px=round(error,3), amcl='observe_only')
            self._publish_debug_image(
                debug, msg.header, 'STABLE', (x2, y2, yaw2))
        except Exception as exc:
            self.warn_throttled('ArUco B1 error: %s: %s' % (type(exc).__name__, exc))
            self.history.clear()
            if bgr is not None:
                self._publish_debug_image(bgr, msg.header, 'SEARCHING - TF/GEOMETRY UNAVAILABLE')

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
