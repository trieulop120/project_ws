#!/usr/bin/env python3
"""
rack_tf_publisher_node.py
=========================
Phat TF TICH + marker RViz cho ke 4 o (A1/A2/B1/B2) theo config/shelf.yaml,
neo theo node D_1 trong route graph (khung map).

Nguyen ly (giai doan 1 - ke moi):
    - D_1 la diem duy nhat xac dinh vi tri ke: truc +X (yaw) cua node D_1
      chi VAO ke (robot dung o D_1 nhan thang vao ke).
    - Khung `shelf_front`: goc GIUA mat truoc ke (tam cot 2), cach D_1
      `front_distance_m` doc theo +X, y=0, cung yaw - doc tu shelf.yaml.
    - Khung tung o (con cua shelf_front, z = 0 cho tat ca):
        slot_A  = cot 1 (A1 duoi + A2 tren, CHUNG khung o)  y = +col_sign*col_pitch
        slot_B1 = cot 2                                      y = 0
        slot_B2 = cot 3                                      y = -col_sign*col_pitch
      (A1/A2 khong phan biet trong TF - chinh la ch y do cao,
       giai doan 1 moi o deu z = 0.)

    - Tat ca hinh hoc doc tu `shelf_config` (mac dinh config/shelf.yaml)
      bang shelf_model (logic thuan, co test don vi - test/test_shelf_model.py).
      Node chi lo doc graph + phat TF/marker.

KHONG SUA GRAPH: graph chay that (mac dinh 'D203B Route.yaml') phai co
node D_1 do ngui dung tao bang graph builder. Neu graph THU THI EU D_1:
KHONG phat TF/marker ke, in canh bao 1 lan, node van chay (quan sat
moi check_interval_s de "phat" khi D_1 xuat hien).

Luu y (TF static, latched):
    SUA DIEM D_1 TRONG GRAPH HOAC SHIELD.YAML -> phai KHOI DONG LAI
    node day (TF static da latch va khong xoa duoc tu /tf_static;
    node chi xoa duoc marker).

Chay:
    ros2 launch amr_mission_manager shelf_tf.launch.py
Hoac thuc nghiem:
    # Start nav2.launch.py first so /navigation/route_yaml_path is available.
    ros2 run tf2_ros tf2_echo map shelf_front slot_A slot_B1 slot_B2
"""

import math
import os
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Point
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, String
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

NODE_NAME = 'rack_tf_publisher'

DEFAULTS = {
    # Mo hinh ke (front_distance_m, col_pitch_m, 4 o, anh xa cam bien,
    # uu tien, anchor_node): tat ca nam trong 1 file duy nhat shelf.yaml
    'shelf_config': '',    # '' = config/shelf.yaml (share sau build -> src)
    'map_frame': 'map',
    # Phat marker + chuc nang
    'publish_markers': True,
    'publish_once': False,   # True: kiem tra + phat 1 lan roi thoat
    'check_interval_s': 5.0, # chu ki quan sat neu publish_once=False
}


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _default_shelf_path() -> str:
    """Duong dan shelf.yaml: share (sau colcon build) -> src (chay thu nga)."""
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(
            get_package_share_directory('amr_mission_manager'),
            'config', 'shelf.yaml')
    except Exception:
        return os.path.expanduser(
            '~/project_ws/src/amr_mission_manager/config/shelf.yaml')


class RackTfPublisher(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in DEFAULTS}
        self._active_graph_path = None
        self._route_path_loaded = None
        self._route_path_last_received = 0.0

        # Nap moi hinh ke tu shelf.yaml (1 lan, tai khoi dong)
        from amr_mission_manager.shelf_model import ShelfModel, ShelfError
        shelf_path = self.p['shelf_config'] or _default_shelf_path()
        self._model_path = shelf_path
        try:
            self._model = ShelfModel.from_yaml(shelf_path)
        except (ShelfError, Exception) as e:
            self.get_logger().error(f'KHONG doc duoc {shelf_path}: {e}')
            self._model = None

        # Log 1 lan / state
        self._log_once = False          # da in startup log (pose + canh bao TAM)
        self._warned_no_anchor = False  # da canh bao graph thieu D_1
        self._done_boxes = set()        # frame da add box (marker update)
        self._last_tf_key = None        # khoa pose da phat TF (chi phat lai khi doi)

        # TF static: PHAI dung StaticTransformBroadcaster (tf2_msgs/TFMessage, QoS
        # transient_local). KHONG publish TransformStamped len /tf_static: sai kieu
        # nen RViz/tf2 khong thay frame (marker van hien vi la topic rieng).
        self._static_tf = StaticTransformBroadcaster(self)
        self._bcast_m = self.create_publisher(MarkerArray, '/rack/markers', 10)

        route_qos = QoSProfile(depth=1)
        route_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        route_qos.reliability = ReliabilityPolicy.RELIABLE
        self._route_path_sub = self.create_subscription(
            String, '/navigation/route_yaml_path', self._on_route_path, route_qos)

        # Lan dau: kiem tra va phat ngay.
        if self._model is None:
            return  # da log loi: di ngui, khong co gi de quan sat
        self._publish_if_anchor()
        if not self.p['publish_once']:
            self.create_timer(self.p['check_interval_s'], self._publish_if_anchor)

    # ------------------------------------------------------------------
    # Doc D_1 tu route graph (khung map; yaw file = RADIAN)
    # ------------------------------------------------------------------
    def _anchor_pose(self):
        """(x, y, yaw_rad) cua anchor (D_1) tu graph, hoac None.

        Yaml graph luu orientation.yaw theo RADIAN (route_graph_loader
        chuyen yaw rad -> quaternion); get_node_pose tra PoseStamped
        frame 'map' -> quay quaternion ve rad de goi shelf_model
        (map_poses nhan radian).
        """
        from amr_navigation.route_graph_loader import RouteGraphLoader
        if not self._active_graph_path:
            return None
        loader = RouteGraphLoader(yaml_path=self._active_graph_path)
        name = loader.resolve(self._model.anchor_node)
        if name is None:
            return None
        pose = loader.get_node_pose(name)
        if pose is None:
            return None
        p = pose.pose.position
        return p.x, p.y, _yaw_from_quat(pose.pose.orientation)

    def _on_route_path(self, msg: String):
        """Use the route YAML announced by the active Nav2 launch."""
        self._route_path_last_received = time.monotonic()
        if msg.data == self._route_path_loaded:
            return
        try:
            from amr_navigation.route_graph_loader import RouteGraphLoader
            loader = RouteGraphLoader(yaml_path=msg.data)
            loader.get_all_node_names()
            self._active_graph_path = msg.data
            self._route_path_loaded = msg.data
            self._last_tf_key = None
            self._warned_no_anchor = False
            self.get_logger().info(f'Using active Nav2 route graph: {msg.data}')
            if self._model is not None:
                self._publish_if_anchor()
        except Exception as e:
            self._active_graph_path = None
            self.get_logger().error(f'Failed to load active Nav2 route graph {msg.data}: {e}')

    # ------------------------------------------------------------------
    # Kiem tra + phat
    # ------------------------------------------------------------------
    def _publish_if_anchor(self) -> bool:
        if not self._active_graph_path:
            return False
        d1 = self._anchor_pose()
        if d1 is None:
            if not self._warned_no_anchor:
                self._warned_no_anchor = True
                self.get_logger().warn(
                    f"Khong thay node '{self._model.anchor_node}' trong graph "
                    f"'{self._active_graph_path}'. KHONG phat "
                    f"TF/marker ke, dat hang se bi chon voi ly do 'khong co "
                    f"{self._model.anchor_node}' (khong tu chuyen sang graph "
                    f"khac). Quan sat moi {self.p['check_interval_s']:.0f}s.")
                if self.p['publish_markers']:
                    self._clear_markers()
            return False

        poses = self._model.map_poses(d1[0], d1[1], d1[2])
        key = tuple(round(v, 6) for p in poses.values() for v in p)
        if key != self._last_tf_key:      # TF static: chi phat khi pose doi
            self._publish_tf(poses)
            self._last_tf_key = key
        if self.p['publish_markers']:
            self._publish_markers(poses)
        if not self._log_once:
            self._log_once = True
            self._startup_log(d1, poses)
        return True

    def _startup_log(self, d1, poses):
        """In MOTT lan (moi lan khoi dong node): pose D_1, vi tri
        shelf_front + 3 khung o trong map, va canh bao cac so TAM."""
        x_d, y_d, yaw_d = d1
        self.get_logger().info(
            f'[ke] Neo {self._model.anchor_node} = ({x_d:.3f}, {y_d:.3f}, '
            f'{math.degrees(yaw_d):.1f} deg) tu graph '
            f"'{self._active_graph_path}' "
            f"(yaw trong graph theo RADIAN - da doi ve rad cho shelf_model)")
        fx, fy, _ = poses['shelf_front']
        self.get_logger().info(
            f'[ke] shelf_front = ({fx:.3f}, {fy:.3f}) trong map '
            f'(front_distance_m={self._model.front_distance_m:.2f}, '
            f'col_pitch_m={self._model.col_pitch_m:.2f}, '
            f'col_sign={self._model.col_sign})')
        for s in self._model.slots:
            px, py, _ = poses[s.tf_frame]
            self.get_logger().info(
                f'[ke] {s.tf_frame} (o {s.id}, cam bien chi so '
                f'{s.sensor_index}) = ({px:.3f}, {py:.3f}) trong map')
        # Canh bao cac so TAM (moi loi in 1 lan - o day)
        cfg = self._load_cfg_raw()
        for key in ('front_distance_status', 'col_pitch_status'):
            if cfg.get(key):
                self.get_logger().warn(f'[ke][TAM] {key}: {cfg[key]}')
        for w in self._model.provisional_sensor_warnings():
            self.get_logger().warn(f'[ke][CAM BIEN TAM] {w}')

    def _load_cfg_raw(self) -> dict:
        try:
            import yaml
            with open(self._model_path, 'r') as f:
                return yaml.safe_load(f) or {}
        except Exception:
            return {}

    def _publish_tf(self, poses):
        """TF STATIC map -> shelf_front / slot_A / slot_B1 / slot_B2.

        SUA DIEM D_1 TRONG GRAPH HOAC CHI SO TRONG SHIELD.YAML ->
        KHOI DONG LAI node day (TF static da latch, khong che duoc).
        """
        now = self.get_clock().now().to_msg()
        msgs = []
        for name, (x, y, yaw) in poses.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.p['map_frame']
            t.child_frame_id = name
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0  # giai doan 1: moi o deu z = 0
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)
            msgs.append(t)
        self._static_tf.sendTransform(msgs)

    # ------------------------------------------------------------------
    # Marker (RViz)
    # ------------------------------------------------------------------
    @staticmethod
    def _color(r, g, b, a=1.0):
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = r, g, b, a
        return c

    def _slot_label(self, tf_frame: str) -> str:
        """Nhan cho 1 khung o: 'A (A1, A2)' neu chung khung duoi/tren,
        con lai la ten o (B1 / B2)."""
        ids = [s.id for s in self._model.slots if s.tf_frame == tf_frame]
        if len(ids) > 1:
            typ = self._model.slots[0].product_type
            for s in self._model.slots:
                if s.tf_frame == tf_frame:
                    typ = s.product_type
                    break
            return f'{typ} ({", ".join(sorted(ids))})'
        return ids[0]

    def _publish_markers(self, poses):
        """Mat truoc ke (3 cot) + hop 3 cot + nhan. Mau don gian;
        CHUA hien trang thai trong/co hang (chua thuoc giai doan 1)."""
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        fx, fy, fyaw = poses['shelf_front']
        cp = self._model.col_pitch_m

        # (1) Mat truoc ke: doan thang theo +Y cua shelf_front,
        #     cam cot 1 -> cam cot 3 (doi xung tam = cam cot 2)
        nx, ny = -math.sin(fyaw), math.cos(fyaw)   # +Y trong map
        line = Marker()
        line.header.frame_id = self.p['map_frame']
        line.header.stamp = now
        line.ns = 'rack'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.points = [Point(x=fx + cp * nx, y=fy + cp * ny, z=0.05),
                       Point(x=fx - cp * nx, y=fy - cp * ny, z=0.05)]
        line.scale.x = 0.02
        line.color = self._color(0.9, 0.9, 0.9, 0.8)
        arr.markers.append(line)

        # (2) Khoen + nhan tung cot (moi khung o RIENG LEU)
        colors = {
            'slot_A':  (0.1, 0.6, 0.9),   # cot 1 = A (xanh nuoc)
            'slot_B1': (0.9, 0.7, 0.1),   # cot 2 = B1 (vang)
            'slot_B2': (0.9, 0.2, 0.2),   # cot 3 = B2 (do)
        }
        i = 10
        for tf_frame in self._model.frames:
            x, y, yaw = poses[tf_frame]
            z = 0.0
            m = Marker()
            m.header.frame_id = self.p['map_frame']
            m.header.stamp = now
            m.ns = 'rack'
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z + 0.05
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)
            m.scale.x = m.scale.y = 0.35
            m.scale.z = 0.10
            m.color = self._color(*colors.get(tf_frame, (0.5, 0.5, 0.5)), 0.5)
            arr.markers.append(m)

            txt = Marker()
            txt.header.frame_id = self.p['map_frame']
            txt.header.stamp = now
            txt.ns = 'rack'
            txt.id = i + 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x, y, z + 0.25
            txt.scale.z = 0.12
            txt.color = self._color(1.0, 1.0, 1.0)
            txt.text = self._slot_label(tf_frame)
            arr.markers.append(txt)
            i += 2
        self._bcast_m.publish(arr)

    def _clear_markers(self):
        m = Marker()
        m.header.frame_id = self.p['map_frame']
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'rack'
        m.id = 0
        m.type = Marker.CUBE
        m.action = Marker.DELETEALL
        arr = MarkerArray()
        arr.markers.append(m)
        self._bcast_m.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = RackTfPublisher()
    try:
        if node.get_parameter('publish_once').value:
            rclpy.spin_once(node, timeout_sec=0.5)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
