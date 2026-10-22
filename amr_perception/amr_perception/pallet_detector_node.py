#!/usr/bin/env python3
"""
Pallet Detector Node
====================
YOLO OBB-based pallet detection using RGB-D camera.
Publishes pallet pose in map / base_footprint / camera frames and dynamic TF.

Subscribes:
  - /color/image_raw (Image)
  - /depth/image_raw (Image)
  - /camera/color/camera_info (CameraInfo)

Publishes:
  - /pallet/pose (geometry_msgs/PoseStamped)        - absolute pose in `map` frame
  - /pallet/pose_base (geometry_msgs/PoseStamped)   - robot-relative pose in `base_footprint`
  - /pallet/pose_camera (geometry_msgs/PoseStamped) - raw pose in camera optical frame
  - /pallet/detections (vision_msgs/Detection2DArray)
  - /perception/pallet_debug_image (sensor_msgs/Image)

Broadcasts:
  - pallet_frame (dynamic TF, parent = map frame)
  - pallet_frame_base (dynamic TF, parent = base_footprint)

Frame conventions
-----------------
- Camera optical frame: X right, Y down, Z forward (what pixel_to_3d returns).
- ROS body frame (REP-103): X forward, Y left, Z up.
  The optical -> ROS conversion is done by TF (camera_color_optical_frame is
  linked to camera_link / base_footprint by the robot's TF tree), so every
  point/orientation is carried through the real TF rotation + translation.
- Camera tilt: the physical camera looks down by `camera_pitch_deg` (default 15)
  but the URDF has camera_link rpy = 0, so the node adds that tilt itself when
  going optical -> base/map. Depth Z is distance ALONG the optical axis, not the
  horizontal distance, so with a tilted camera Base X < depth.
- Pallet frame: origin at pallet centre, +X is the normal of the pallet's long
  (front) face pointing INTO the pallet (away from the robot), +Z up.
  Yaw is the angle of pallet +X relative to base_footprint +X, around the
  vertical axis, normalized to [-pi, pi]:
      yaw = 0      -> robot is facing the pallet squarely
      yaw > 0      -> pallet is rotated counter-clockwise (left) vs robot heading
      yaw < 0      -> pallet is rotated clockwise (right) vs robot heading
  The yaw is estimated from the 3D edges of the OBB using depth. If depth at
  the corners is not usable, it falls back to the line-of-sight direction
  (pallet assumed perpendicular to the ray robot -> pallet).
"""

import math
import os
import traceback
import threading
from typing import Optional, List, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from message_filters import Subscriber, ApproximateTimeSynchronizer
from cv_bridge import CvBridge

from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


# ----------------------------------------------------------------------
# Small math helpers (no extra dependencies)
# ----------------------------------------------------------------------
def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Quaternion -> 3x3 rotation matrix."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def rot_to_quat(R: np.ndarray) -> Quaternion:
    """3x3 rotation matrix -> Quaternion msg."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = Quaternion()
    q.x, q.y, q.z, q.w = float(x), float(y), float(z), float(w)
    return q


def normalize_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_to_rot(yaw: float) -> np.ndarray:
    """Rotation around Z."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PalletDetectorNode(Node):
    """YOLO OBB Pallet Detector with RGB-Depth fusion."""

    # Camera intrinsics (Astra Pro default)
    DEFAULT_FX = 570.0
    DEFAULT_FY = 570.0
    DEFAULT_CX = 320.0
    DEFAULT_CY = 240.0

    MIN_DEPTH_M = 0.1
    MAX_DEPTH_M = 10.0

    def __init__(self):
        super().__init__('pallet_detector_node')

        # ============================================================
        # Parameters
        # ============================================================
        self.declare_parameter('model_path', '')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('pallet_frame_id', 'pallet_frame')
        self.declare_parameter('tf_timeout', 0.2)
        self.declare_parameter('debug_image', True)
        # percentile of depth inside the OBB (low = nearest = pallet front face)
        self.declare_parameter('depth_percentile', 25.0)
        self.declare_parameter('use_depth_yaw', True)
        # Real downward tilt of the camera (deg, positive = looking down) that is
        # NOT in the URDF/TF (camera_joint rpy = 0 0 0). Default 15 deg.
        # Set to 0.0 once camera_joint rpy="0 <pitch_rad> 0" is fixed in the URDF,
        # otherwise the tilt would be applied twice.
        self.declare_parameter('camera_pitch_deg', 5.3)

        self.model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.target_frame = self.get_parameter('target_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame_param = self.get_parameter('camera_frame').value
        self.pallet_frame_id = self.get_parameter('pallet_frame_id').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        self.publish_debug = self.get_parameter('debug_image').value
        self.depth_percentile = self.get_parameter('depth_percentile').value
        self.use_depth_yaw = self.get_parameter('use_depth_yaw').value
        self.camera_pitch_deg = self.get_parameter('camera_pitch_deg').value

        # Rotation correcting the tilt missing from TF, expressed in the optical frame.
        # camera_link pitched down by theta about its own Y axis:
        #   R_corr = R_co^T @ Ry(theta) @ R_co,  R_co: optical -> camera_link
        theta = math.radians(self.camera_pitch_deg)
        ct, st = math.cos(theta), math.sin(theta)
        R_pitch = np.array([[ct, 0.0, st], [0.0, 1.0, 0.0], [-st, 0.0, ct]])
        R_co = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        self.R_cam_corr = R_co.T @ R_pitch @ R_co

        # Camera intrinsics
        self.fx = self.DEFAULT_FX
        self.fy = self.DEFAULT_FY
        self.cx = self.DEFAULT_CX
        self.cy = self.DEFAULT_CY
        self.camera_info_received = False

        self.bridge = CvBridge()

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # YOLO model
        self.model = None
        self._load_model()

        # Thread-safe latest data
        self.latest_color = None
        self.latest_depth = None
        self.latest_depth_encoding = None
        self.latest_color_header = None
        self.frame_id_counter = 0
        self.last_processed_frame = -1
        self.lock = threading.Lock()

        # ============================================================
        # Publishers
        # ============================================================
        self.pose_pub = self.create_publisher(
            PoseStamped, '/pallet/pose', QoSProfile(depth=10))
        self.pose_base_pub = self.create_publisher(
            PoseStamped, '/pallet/pose_base', QoSProfile(depth=10))
        self.pose_camera_pub = self.create_publisher(
            PoseStamped, '/pallet/pose_camera', QoSProfile(depth=10))
        self.detections_pub = self.create_publisher(
            Detection2DArray, '/pallet/detections', QoSProfile(depth=10))

        if self.publish_debug:
            self.debug_image_pub = self.create_publisher(
                Image,
                '/perception/pallet_debug_image',
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            )

        # ============================================================
        # Subscribers with Time Synchronizer
        # ============================================================
        self.color_sub = Subscriber(self, Image, '/color/image_raw')
        self.depth_sub = Subscriber(self, Image, '/depth/image_raw')

        self.ts = ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            QoSProfile(depth=1)
        )

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.get_logger().info('Pallet Detector Node initialized')
        self.get_logger().info(f'Model path: {self.model_path}')
        self.get_logger().info(f'Conf threshold: {self.conf_threshold}')
        self.get_logger().info(
            f'Frames: camera={self.camera_frame_param}, base={self.base_frame}, '
            f'target={self.target_frame}')

    # ==================================================================
    # Model
    # ==================================================================
    def _load_model(self):
        """Load YOLO OBB model."""
        if YOLO is None:
            self.get_logger().error('Ultralytics not installed!')
            return

        if not self.model_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                pkg_dir = get_package_share_directory('amr_perception')
                self.model_path = os.path.join(pkg_dir, 'weights', 'best.pt')
                if not os.path.exists(self.model_path):
                    self.model_path = os.path.join(
                        os.path.dirname(os.path.dirname(os.path.dirname(__file__))),
                        'weights', 'best.pt'
                    )
            except Exception:
                self.model_path = os.path.join(
                    os.path.dirname(os.path.dirname(__file__)),
                    'weights', 'best.pt'
                )
        else:
            self.model_path = os.path.expanduser(self.model_path)

        if not os.path.exists(self.model_path):
            self.get_logger().warn(f'Model file not found: {self.model_path}')
            self.get_logger().warn('Using default YOLO8n OBB model for testing')
            try:
                self.model = YOLO('yolov8n-obb.pt')
            except Exception as e:
                self.get_logger().error(f'Failed to load default model: {e}')
            return

        try:
            self.get_logger().info(f'Loading model from: {self.model_path}')
            self.model = YOLO(self.model_path)
            self.get_logger().info('YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            self.model = None

    # ==================================================================
    # Callbacks
    # ==================================================================
    def camera_info_callback(self, msg: CameraInfo):
        """Update camera intrinsics from CameraInfo topic."""
        if not self.camera_info_received:
            self.fx = msg.k[0] if msg.k[0] != 0 else self.DEFAULT_FX
            self.fy = msg.k[4] if msg.k[4] != 0 else self.DEFAULT_FY
            self.cx = msg.k[2] if msg.k[2] != 0 else self.DEFAULT_CX
            self.cy = msg.k[5] if msg.k[5] != 0 else self.DEFAULT_CY
            self.camera_info_received = True
            self.get_logger().info(
                f'Camera intrinsics updated: fx={self.fx:.1f}, fy={self.fy:.1f}, '
                f'cx={self.cx:.1f}, cy={self.cy:.1f}'
            )

    def synced_callback(self, color_msg: Image, depth_msg: Image):
        """Synchronized callback for color and depth images."""
        try:
            color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            with self.lock:
                self.latest_color = color
                self.latest_depth = depth
                self.latest_depth_encoding = depth_msg.encoding
                self.latest_color_header = color_msg.header
                self.frame_id_counter += 1
        except Exception as e:
            self.get_logger().error(f'Error converting images: {e}')

    # ==================================================================
    # Geometry / depth
    # ==================================================================
    def pixel_to_3d(self, u: float, v: float, depth: float) -> np.ndarray:
        """Pixel + depth -> 3D point in camera OPTICAL frame (X right, Y down, Z fwd)."""
        x = (u - self.cx) * depth / self.fx
        y = (v - self.cy) * depth / self.fy
        return np.array([x, y, depth], dtype=np.float64)

    @staticmethod
    def _to_meters(values: np.ndarray, depth_encoding: str) -> np.ndarray:
        """Keep only valid depths and convert them to meters."""
        values = values.astype(np.float32)
        values = values[np.isfinite(values) & (values > 0)]
        # '16UC1' / 'mono16': millimeters; '32FC1': meters
        if depth_encoding in ('16UC1', 'mono16'):
            values = values / 1000.0
        return values

    def extract_depth_at_point(self, depth_img: np.ndarray, u: int, v: int,
                               depth_encoding: str, window: int = 3) -> float:
        """Median depth (m) in a small window around (u, v). 0.0 if invalid."""
        h, w = depth_img.shape[:2]
        x_min, x_max = max(0, u - window), min(w, u + window + 1)
        y_min, y_max = max(0, v - window), min(h, v + window + 1)
        if x_min >= x_max or y_min >= y_max:
            return 0.0

        vals = self._to_meters(depth_img[y_min:y_max, x_min:x_max], depth_encoding)
        vals = vals[(vals > self.MIN_DEPTH_M) & (vals < self.MAX_DEPTH_M)]
        if len(vals) == 0:
            return 0.0
        return float(np.median(vals))

    def extract_depth_in_obb(self, depth_img: np.ndarray, box: np.ndarray,
                             depth_encoding: str, shrink: float = 0.6,
                             min_pixels: int = 20) -> float:
        """Robust depth (m) of the pallet from pixels inside the (shrunk) OBB.

        Uses a low percentile so gaps between planks (which show the floor /
        background further away) do not pull the depth backwards.
        """
        h, w = depth_img.shape[:2]
        center = box.mean(axis=0)
        shrunk = center + (box - center) * shrink

        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.fillPoly(mask, [shrunk.astype(np.int32).reshape((-1, 1, 2))], 1)

        vals = self._to_meters(depth_img[mask > 0], depth_encoding)
        vals = vals[(vals > self.MIN_DEPTH_M) & (vals < self.MAX_DEPTH_M)]
        if len(vals) < min_pixels:
            return 0.0
        return float(np.percentile(vals, self.depth_percentile))

    def lookup_rt(self, target: str, source: str,
                  cam_corr: bool = False) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Return (R, t) such that p_target = R @ p_source + t (latest TF).

        cam_corr=True adds the camera tilt that is missing from the TF tree
        (param camera_pitch_deg); only use it when source is the camera optical frame.
        """
        try:
            tf = self.tf_buffer.lookup_transform(
                target, source, Time(),
                timeout=Duration(seconds=self.tf_timeout))
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().debug(f'TF {source} -> {target} failed: {e}')
            return None
        q = tf.transform.rotation
        t = tf.transform.translation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        if cam_corr and self.camera_pitch_deg != 0.0:
            R = R @ self.R_cam_corr
        return R, np.array([t.x, t.y, t.z])

    def estimate_yaw_from_depth(self, depth_img: np.ndarray, det: dict,
                                depth_encoding: str, center_depth: float,
                                camera_frame: str,
                                p_base: np.ndarray) -> Optional[float]:
        """Pallet yaw in base frame from the 3D OBB edges.

        Returns yaw (rad, in [-pi, pi]) of the pallet +X axis: the normal of
        the long face pointing INTO the pallet (away from the robot).
        0 means the robot faces the pallet squarely.
        Returns None if corner depths are unusable.
        """
        rt = self.lookup_rt(self.base_frame, camera_frame, cam_corr=True)
        if rt is None:
            return None
        R_bc, t_bc = rt

        box = det['box']
        center = box.mean(axis=0)
        pts_base = []
        for c in box:
            # Move the sample point a bit toward the centre (stay on the pallet),
            # the edge direction is preserved because the quad stays similar.
            p = center + (c - center) * 0.85
            d = self.extract_depth_at_point(
                depth_img, int(round(p[0])), int(round(p[1])), depth_encoding, window=4)
            if d <= 0.0 or abs(d - center_depth) > 0.5:  # background / invalid
                return None
            p_cam = self.pixel_to_3d(float(p[0]), float(p[1]), d)
            pts_base.append(R_bc @ p_cam + t_bc)

        c0, c1, c2, c3 = pts_base
        e1 = np.linalg.norm((c1 - c0)[:2])
        e2 = np.linalg.norm((c2 - c1)[:2])
        if e1 >= e2:
            d_long = (c1 - c0) + (c2 - c3)
        else:
            d_long = (c2 - c1) + (c3 - c0)

        dx, dy = float(d_long[0]), float(d_long[1])
        norm = math.hypot(dx, dy)
        if norm < 1e-3:
            return None

        # Normal to the long edge, pointing AWAY from the robot (robot at base origin),
        # i.e. same side as the pallet position vector.
        nx, ny = -dy / norm, dx / norm
        if nx * p_base[0] + ny * p_base[1] < 0.0:
            nx, ny = -nx, -ny
        return normalize_angle(math.atan2(ny, nx))

    # ==================================================================
    # Detection
    # ==================================================================
    def detect_pallets(self, color_img: np.ndarray) -> List[dict]:
        """Run YOLO OBB detection on color image."""
        if self.model is None:
            return []

        try:
            results = self.model.predict(
                color_img,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                device='cpu',
                verbose=False
            )

            detections = []
            for result in results:
                obb = result.obb
                if obb is None or len(obb) == 0:
                    continue

                corners = obb.xyxyxyxy.cpu().numpy()       # (N, 4, 2)
                xywhr = obb.xywhr.cpu().numpy()            # (N, 5): cx, cy, w, h, angle(rad)
                confs = obb.conf.cpu().numpy()
                clss = obb.cls.cpu().numpy().astype(int)

                for i in range(len(corners)):
                    cx, cy = float(xywhr[i][0]), float(xywhr[i][1])
                    cls = int(clss[i])
                    detections.append({
                        'center': (cx, cy),
                        'angle': float(xywhr[i][4]),
                        'conf': float(confs[i]),
                        'class': result.names[cls],
                        'class_id': cls,
                        'box': corners[i],
                        'u': int(cx),
                        'v': int(cy),
                    })

            return detections

        except Exception as e:
            self.get_logger().error(f'Detection error: {e}')
            return []

    # ==================================================================
    # Publishing helpers
    # ==================================================================
    @staticmethod
    def make_pose(frame_id: str, stamp, p: np.ndarray, R: np.ndarray) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp
        msg.pose.position.x = float(p[0])
        msg.pose.position.y = float(p[1])
        msg.pose.position.z = float(p[2])
        msg.pose.orientation = rot_to_quat(R)
        return msg

    def make_tf(self, pose: PoseStamped, child_frame: str) -> TransformStamped:
        t = TransformStamped()
        t.header = pose.header
        t.child_frame_id = child_frame
        t.transform.translation.x = pose.pose.position.x
        t.transform.translation.y = pose.pose.position.y
        t.transform.translation.z = pose.pose.position.z
        t.transform.rotation = pose.pose.orientation
        return t

    def compute_pallet_poses(self, det: dict, depth_img: np.ndarray,
                             depth_encoding: str, header) -> bool:
        """Compute the 3D pose of one detection and publish pose/TF.

        Fills det['x_3d'..] (camera optical frame) for the HUD / detections.
        """
        camera_frame = header.frame_id or self.camera_frame_param

        depth_m = self.extract_depth_in_obb(depth_img, det['box'], depth_encoding)
        if depth_m <= 0.0:
            # fallback: small window around the centre
            depth_m = self.extract_depth_at_point(
                depth_img, det['u'], det['v'], depth_encoding)
        if not (self.MIN_DEPTH_M < depth_m < self.MAX_DEPTH_M):
            return False

        # --- 1. Position in camera OPTICAL frame -------------------------
        p_cam = self.pixel_to_3d(float(det['u']), float(det['v']), depth_m)

        # --- 2. Camera optical -> base_footprint (ROS convention) ---------
        rt_base = self.lookup_rt(self.base_frame, camera_frame, cam_corr=True)
        if rt_base is None:
            self.get_logger().warn(
                f'No TF {camera_frame} -> {self.base_frame}', throttle_duration_sec=2.0)
            return False
        R_bc, t_bc = rt_base
        p_base = R_bc @ p_cam + t_bc

        # --- 3. Yaw around vertical axis (base frame) ---------------------
        yaw_base = None
        if self.use_depth_yaw:
            yaw_base = self.estimate_yaw_from_depth(
                depth_img, det, depth_encoding, depth_m, camera_frame, p_base)
        if yaw_base is None:
            # Fallback: pallet perpendicular to the line of sight robot -> pallet
            yaw_base = math.atan2(p_base[1], p_base[0])
        yaw_base = normalize_angle(yaw_base)

        R_pb = yaw_to_rot(yaw_base)       # pallet orientation in base
        R_pc = R_bc.T @ R_pb              # pallet orientation in camera optical

        stamp = header.stamp

        # --- 4. Publish: camera / base --------------------------------------
        pose_cam = self.make_pose(camera_frame, stamp, p_cam, R_pc)
        pose_base = self.make_pose(self.base_frame, stamp, p_base, R_pb)
        self.pose_camera_pub.publish(pose_cam)
        self.pose_base_pub.publish(pose_base)

        tfs = [self.make_tf(pose_base, f'{self.pallet_frame_id}_base')]

        # --- 5. Publish: map --------------------------------------------------
        rt_map = self.lookup_rt(self.target_frame, camera_frame, cam_corr=True)
        if rt_map is not None:
            R_mc, t_mc = rt_map
            p_map = R_mc @ p_cam + t_mc
            R_pm = R_mc @ R_pc
            pose_map = self.make_pose(self.target_frame, stamp, p_map, R_pm)
            self.pose_pub.publish(pose_map)
            tfs.append(self.make_tf(pose_map, self.pallet_frame_id))
            det['x_map'], det['y_map'] = float(p_map[0]), float(p_map[1])
        else:
            self.get_logger().debug(
                f'No TF {camera_frame} -> {self.target_frame}; skip map pose')

        self.tf_broadcaster.sendTransform(tfs)

        # --- 6. Fill detection info ---------------------------------------
        det['x_3d'], det['y_3d'], det['z_3d'] = (float(p_cam[0]), float(p_cam[1]),
                                                 float(p_cam[2]))
        det['x_base'], det['y_base'], det['z_base'] = (float(p_base[0]), float(p_base[1]),
                                                       float(p_base[2]))
        det['yaw'] = float(yaw_base)
        det['yaw_deg'] = math.degrees(yaw_base)
        det['depth_m'] = depth_m
        det['pose_cam'] = pose_cam.pose
        return True

    # ==================================================================
    # Debug image
    # ==================================================================
    def create_debug_image(self, color_img: np.ndarray,
                           detections: List[dict],
                           best_detection: Optional[dict] = None) -> np.ndarray:
        """Create debug image with OBB overlays and HUD."""
        debug_img = color_img.copy()
        h, w = debug_img.shape[:2]

        for det in detections:
            box = det['box']
            color = (0, 255, 0) if det['class'] == 'pallet' else (255, 0, 0)

            pts = box.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(debug_img, [pts], True, color, 2)
            cv2.circle(debug_img, (int(det['center'][0]), int(det['center'][1])),
                       5, color, -1)

            label = f"{det['class']}: {det['conf']:.2f}"
            cv2.putText(debug_img, label,
                        (int(det['center'][0]) + 10, int(det['center'][1]) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        if best_detection and 'x_3d' in best_detection:
            hud_x, hud_w, hud_h = 10, 300, 200
            hud_y = h - hud_h - 10

            cv2.rectangle(debug_img, (hud_x, hud_y),
                          (hud_x + hud_w, hud_y + hud_h), (0, 0, 0), -1)
            cv2.rectangle(debug_img, (hud_x, hud_y),
                          (hud_x + hud_w, hud_y + hud_h), (0, 255, 0), 2)

            b = best_detection
            lines = [
                "=== PALLET DETECTION ===",
                f"Class: {b['class']}  Conf: {b['conf']:.2f}",
                f"Depth: {b.get('depth_m', 0):.3f} m",
                f"Cam  X:{b.get('x_3d', 0):.2f} Y:{b.get('y_3d', 0):.2f} Z:{b.get('z_3d', 0):.2f}",
                f"Base X:{b.get('x_base', 0):.2f} Y:{b.get('y_base', 0):.2f} Z:{b.get('z_base', 0):.2f}",
                f"Map  X:{b.get('x_map', float('nan')):.2f} Y:{b.get('y_map', float('nan')):.2f}",
                f"Yaw(base): {b.get('yaw_deg', 0):.1f} deg",
            ]
            for i, line in enumerate(lines):
                cv2.putText(debug_img, line,
                            (hud_x + 10, hud_y + 25 + i * 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1)
        else:
            cv2.putText(debug_img, "No pallet detected",
                        (w // 2 - 100, h // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        return debug_img

    def publish_debug_image(self, debug_img: np.ndarray, header):
        try:
            if len(debug_img.shape) == 2:
                debug_img = cv2.cvtColor(debug_img, cv2.COLOR_GRAY2BGR)
            debug_img = np.ascontiguousarray(debug_img, dtype=np.uint8)

            msg = Image()
            msg.header = header
            msg.height = debug_img.shape[0]
            msg.width = debug_img.shape[1]
            msg.encoding = 'bgr8'
            msg.is_bigendian = 0
            msg.step = debug_img.shape[1] * 3
            msg.data = debug_img.tobytes()
            self.debug_image_pub.publish(msg)
        except Exception:
            self.get_logger().error(f'Debug image error: {traceback.format_exc()}')

    # ==================================================================
    # Main loop
    # ==================================================================
    def timer_callback(self):
        """Process the newest frame: detect, localize, publish."""
        with self.lock:
            if self.latest_color is None or self.latest_depth is None:
                return
            if self.frame_id_counter == self.last_processed_frame:
                return  # no new frame
            self.last_processed_frame = self.frame_id_counter

            color_img = self.latest_color.copy()
            depth_img = self.latest_depth.copy()
            depth_encoding = self.latest_depth_encoding or '16UC1'
            header = self.latest_color_header

        detections = self.detect_pallets(color_img)

        # Pick the best pallet and localize it
        best_detection = None
        best_conf = 0.0
        for det in detections:
            if det['class'] == 'pallet' and det['conf'] > best_conf:
                best_conf = det['conf']
                best_detection = det

        if best_detection is not None:
            ok = self.compute_pallet_poses(best_detection, depth_img,
                                           depth_encoding, header)
            if ok:
                self.get_logger().debug(
                    f"Pallet: base=({best_detection['x_base']:.2f}, "
                    f"{best_detection['y_base']:.2f}) "
                    f"yaw={best_detection['yaw_deg']:.1f}deg conf={best_conf:.2f}")
            else:
                best_detection = None

        # Detections message
        det_msg = Detection2DArray()
        det_msg.header = header
        for det in detections:
            d = Detection2D()
            d.header = header

            result = ObjectHypothesisWithPose()
            result.hypothesis.class_id = det['class']
            result.hypothesis.score = det['conf']
            if 'pose_cam' in det:
                result.pose.pose = det['pose_cam']
            d.results.append(result)

            bbox = det['box']
            d.bbox.center.position.x = float(det['center'][0])
            d.bbox.center.position.y = float(det['center'][1])
            d.bbox.center.theta = float(det['angle'])
            d.bbox.size_x = float(np.max(bbox[:, 0]) - np.min(bbox[:, 0]))
            d.bbox.size_y = float(np.max(bbox[:, 1]) - np.min(bbox[:, 1]))

            det_msg.detections.append(d)

        self.detections_pub.publish(det_msg)

        if self.publish_debug:
            debug_img = self.create_debug_image(color_img, detections, best_detection)
            self.publish_debug_image(debug_img, header)


def main(args=None):
    rclpy.init(args=args)
    node = PalletDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()