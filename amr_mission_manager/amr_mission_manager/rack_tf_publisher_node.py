#!/usr/bin/env python3
"""
rack_tf_publisher_node.py
=========================
Phat TF tinh cua 3 cot ke (san pham thap / vua / cao) trong khung map
+ marker RViz. Neo theo mot NODE ANCHOR trong route graph (mac dinh D_1).

Nguyen ly (thiet ke, 1 ke duy nhat):
    - Xe can chinh tu thi di thang vao ke theo huong yaw cua anchor.
      TF chieu dung tu the de "vao" ke, KHONG dung lam du lieu that.
    - KHOANG_X (mac 0.6m) theo +X (huong yaw anchor) tu anchor
      = tam cot GIUA (san pham vua).
    - 3 cot cach nhau COL_OFFSET (mac 0.6m = rong 1 cot) theo +Y:
        * cot THAP  (bin_low)    : ben TRAI (do xe nho vao) = +Y
        * cot VUA   (bin_medium) : GIUA = 0
        * cot CAO   (bin_high)   : ben PHAI = -Y
    - z = 0 cho tat ca: do cao (ot 2 thap, tap san pham) do LIFT +
      cam bien siu am lo, khong nham trong TF nay.
    - Gate theo nguon graph: graph co ANCHOR (D_1) -> phat TF;
      khong co -> khong phat (log, de ngui). Quan sat timer de "theo
      doi" khi nguon graph / vi tri D_1 doi ma khong can sua link.

Chay:
    ros2 launch amr_mission_manager rack_tf_publisher.launch.py
Hoac thuc nghiem:
    ros2 run amr_mission_manager rack_tf_publisher anchor_node:=D_1
    ros2 view_tf   # xem map -> bin_low / bin_medium / bin_high
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

NODE_NAME = 'rack_tf_publisher'

DEFAULTS = {
    # Neo: ten node trong route graph (chi 1 ke = chi 1 anchor)
    'anchor_node': 'D_1',
    'graph_yaml_filename': 'route_graph.yaml',
    'map_frame': 'map',

    # Hinh hoc ke (so do / thiet ke, co the tinh luon de chinh)
    'bin_x_m': 0.60,        # khoang +X tu anchor -> tam cot gia (san pham vua)
    'col_offset_m': 0.60,   # khoang +Y giua tam cot ke nhau (= rong 1 cot)
    'col_sign': 1.0,        # flip +Y khi lap ke: 1.0 = cot thap tren +Y (trai)

    # Phat marker + chuc nang
    'publish_markers': True,
    'publish_once': False,   # True: kiem tra 1 lan roi thoat (ke fix)
    'check_interval_s': 5.0, # chu ki quan sat neu publish_once=False
}

# Ten frame 3 cot (tuot xuoi, khop ban van)
BIN_LOW = 'bin_low'         # san pham thap, 2 ot chi dung 1 TF
BIN_MED = 'bin_medium'      # san pham vua, cot gia = tam 60cm
BIN_HIGH = 'bin_high'       # san pham cao


def _rot_z(vx: float, vy: float, yaw: float):
    """Xoay diem local (vx,vy) theo yaw ra map (chung gốc)."""
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx - s * vy, s * vx + c * vy


def _yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RackTfPublisher(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in DEFAULTS}

        self._bcast_tf = self.create_publisher(TransformStamped, '/tf_static', 10)
        self._bcast_m = self.create_publisher(MarkerArray, '/rack/markers', 10)

        # Lan dau: kiem tra va phat ngay.
        if not self._publish_if_anchor():
            # Khong co anchor -> de ngui; chi chuc timer quan sat (trừ
            # publish_once). Neu co anchor trong tuong lai se tu phat.
            self.get_logger().warn(
                f"Khong thay node '{self.p['anchor_node']}' trong graph "
                f"'{self.p['graph_yaml_filename']}'. Khong phat rack TF. "
                f"(quan sat moi {self.p['check_interval_s']:.0f}s)")

        if not self.p['publish_once']:
            self.create_timer(self.p['check_interval_s'],
                              self._publish_if_anchor)

    # ------------------------------------------------------------------
    # Anh xa local -> map
    # ------------------------------------------------------------------
    def _anchor_pose(self):
        """Doc (x, y, yaw) cua anchor node tu route graph.

        Tra (x, y, yaw) hoac None (graph khong co anchor node).
        """
        from amr_navigation.route_graph_loader import RouteGraphLoader
        # Instan moi moi lan de luon doc file hien hanh (khong che).
        loader = RouteGraphLoader(yaml_filename=self.p['graph_yaml_filename'])
        name = loader.resolve(self.p['anchor_node'])
        if name is None:
            return None
        pose = loader.get_node_pose(name)
        if pose is None:
            return None
        p = pose.pose.position
        return p.x, p.y, _yaw_of(pose.pose.orientation)

    def _local_bins(self):
        """Tọa độ local (khe) cua 3 cot: cot gia = tam bin_x_m theo +X.

        cot thap = +Y (trai), cot cao = -Y (phai) -> col_sign de flip.
        """
        bx, co = float(self.p['bin_x_m']), float(self.p['col_offset_m'])
        s = float(self.p['col_sign'])
        return {
            BIN_MED: (bx, 0.0),
            BIN_LOW: (bx, s * co),
            BIN_HIGH: (bx, -s * co),
        }

    # ------------------------------------------------------------------
    # Kiem tra + phat
    # ------------------------------------------------------------------
    def _publish_if_anchor(self) -> bool:
        anchor = self._anchor_pose()
        if anchor is None:
            # Kho khong co anchor -> xoa markers (TF tu bi latched,
            # chua xoa /tf_static duoc; chi ngung phat).
            if self.p['publish_markers']:
                self._clear_markers()
            return False

        x_d, y_d, yaw_d = anchor
        local = self._local_bins()
        tf = {}
        for name, (lx, ly) in local.items():
            mx, my = _rot_z(lx, ly, yaw_d)
            # z = 0: do cao do lift lo
            tf[name] = (x_d + mx, y_d + my, 0.0, yaw_d)

        self._publish_tf(tf)
        if self.p['publish_markers']:
            self._publish_markers(tf)
        self.get_logger().info(
            f"Rack TF neo {self.p['anchor_node']}=({x_d:.3f},{y_d:.3f},"
            f"{math.degrees(yaw_d):.1f}deg) "
            f"bx={self.p['bin_x_m']:.2f} col={self.p['col_offset_m']:.2f}m")
        for name, (x, y, z, yaw) in tf.items():
            self.get_logger().info(f'  {name}: ({x:.3f}, {y:.3f}) map')
        return True

    def _publish_tf(self, tf):
        now = self.get_clock().now().to_msg()
        for name, (x, y, z, yaw) in tf.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.p['map_frame']
            t.child_frame_id = name
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = z
            t.transform.rotation.z = math.sin(yaw / 2.0)
            t.transform.rotation.w = math.cos(yaw / 2.0)
            self._bcast_tf.publish(t)

    @staticmethod
    def _color(r, g, b, a=1.0):
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = r, g, b, a
        return c

    def _publish_markers(self, tf):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        # Mau theo cot: thap (xanh nuoc) / vua (vang) / cao (do)
        col_color = {
            BIN_LOW: (0.1, 0.6, 0.9, 0.5),
            BIN_MED: (0.9, 0.7, 0.1, 0.5),
            BIN_HIGH: (0.9, 0.2, 0.2, 0.5),
        }
        for i, (name, (x, y, z, yaw)) in enumerate(sorted(tf.items())):
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
            m.color = self._color(*col_color.get(name, (0.5, 0.5, 0.5, 0.5)))
            arr.markers.append(m)

            txt = Marker()
            txt.header.frame_id = self.p['map_frame']
            txt.header.stamp = now
            txt.ns = 'rack'
            txt.id = 100 + i
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x, txt.pose.position.y, txt.pose.position.z = x, y, z + 0.25
            txt.scale.z = 0.12
            txt.color = self._color(1.0, 1.0, 1.0)
            txt.text = name
            arr.markers.append(txt)
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
