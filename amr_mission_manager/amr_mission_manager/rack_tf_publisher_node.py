#!/usr/bin/env python3
"""
rack_tf_publisher_node.py
=========================
Phat TF tinh cua ke 4 o (lưới 2x2) trong khung map + marker RViz.

Ket cau TF (doc: docs/REPORT_amr_pallet_mission_design.md, muc 5):

    map ─ rack_base ─ bin_0
                  ├─ bin_1  (bin_0 + pitch_row theo +X cua ke)
                  ├─ bin_2  (bin_0 + pitch_col theo +Y cua ke)
                  └─ bin_3  (bin_1 + bin_2)

Chi 3 so do mot lan khi lap ke:
    rack_x / rack_y / rack_yaw_deg : goc ke (bin_0) trong map
    pitch_row_m                    : khoang cach 2 o cung hang
    pitch_col_m                    : khoang cach 2 o cung cot

Convention: +X cua ke huong toi bin_1, +Y huong toi bin_2 (xac nhan lai
voi ke that khi lap).

So mock mac dinh (bat buoc dat trong launch de gap vi tri thuc):
    rack_x=0.0  rack_y=1.5  rack_yaw_deg=0.0  pitch_row=0.4  pitch_col=0.4

Chay:
    ros2 launch amr_mission_manager rack_tf_publisher.launch.py
Hoac thuc nghi nhan nhanh:
    ros2 run amr_mission_manager rack_tf_publisher rack_x:=0.0 rack_y:=1.5 rack_yaw_deg:=0.0
    ros2 view_tf        # xem map -> bin_0..3
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

NODE_NAME = 'rack_tf_publisher'

DEFAULTS = {
    'map_frame': 'map',
    'rack_x': 0.0,
    'rack_y': 1.5,
    'rack_yaw_deg': 0.0,
    'pitch_row_m': 0.4,
    'pitch_col_m': 0.4,
    'rack_z_m': 0.0,          # cao do mat ke (bin o z nay)
    'publish_markers': True,
    'publish_once': False,    # True: publish 1 lan roi thoat (static_transforms style)
}


def _rot_z(vx: float, vy: float, yaw: float):
    c, s = math.cos(yaw), math.sin(yaw)
    return c * vx - s * vy, s * vx + c * vy


class RackTfPublisher(Node):

    def __init__(self):
        super().__init__(NODE_NAME)
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.p = {k: self.get_parameter(k).value for k in DEFAULTS}

        yaw = math.radians(float(self.p['rack_yaw_deg']))
        rx, ry, rz = float(self.p['rack_x']), float(self.p['rack_y']), float(self.p['rack_z_m'])
        pr, pc = float(self.p['pitch_row_m']), float(self.p['pitch_col_m'])

        # Bo TF bin_i trong khung rack_base (bin_0 goc) -> ra map
        local = {
            'bin_0': (0.0, 0.0),
            'bin_1': (pr, 0.0),
            'bin_2': (0.0, pc),
            'bin_3': (pr, pc),
        }
        self._tf = {}
        for name, (lx, ly) in local.items():
            mx, my = _rot_z(lx, ly, yaw)
            self._tf[name] = (rx + mx, ry + my, rz, yaw)

        self._bcast_tf = self.create_publisher(
            TransformStamped, '/tf_static', 10)
        self._bcast_m = self.create_publisher(MarkerArray, '/rack/markers', 10)

        self._publish_once()
        # Dam bao 1 lan de TF marker den RViz; chi chuc timer neu khong co
        # publish_once (publish_once -> spin_once roi thoat, khong can timer).
        if not self.p['publish_once']:
            self.create_timer(1.0, self._publish_once)

        self.get_logger().info(
            f'Rack TF: base=({rx:.3f},{ry:.3f},{math.degrees(yaw):.1f}deg) '
            f'pitch=({pr:.2f},{pc:.2f})m')
        for name, t in self._tf.items():
            self.get_logger().info(f'  {name}: ({t[0]:.3f}, {t[1]:.3f}) map')

    def _publish_once(self):
        now = self.get_clock().now().to_msg()
        for name, (x, y, z, yaw) in self._tf.items():
            self._bcast_tf.publish(self._make_tf(now, x, y, z, yaw, name))
        if self.p['publish_markers']:
            self._bcast_m.publish(self._make_markers())

    def _make_tf(self, stamp, x, y, z, yaw, child):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.p['map_frame']
        t.child_frame_id = child
        t.transform.translation.x, t.transform.translation.y, \
            t.transform.translation.z = x, y, z
        t.transform.rotation.z = math.sin(yaw / 2.0)
        t.transform.rotation.w = math.cos(yaw / 2.0)
        return t

    @staticmethod
    def _color(r, g, b, a=1.0):
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = r, g, b, a
        return c

    def _make_markers(self) -> MarkerArray:
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for name, (x, y, z, yaw) in self._tf.items():
            m = Marker()
            m.header.frame_id = self.p['map_frame']
            m.header.stamp = now
            m.ns = 'rack'
            m.id = int(name[-1])
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z + 0.05
            m.pose.orientation.z = math.sin(yaw / 2.0)
            m.pose.orientation.w = math.cos(yaw / 2.0)
            m.scale.x = m.scale.y = 0.35
            m.scale.z = 0.10
            m.color = self._color(0.1, 0.9, 0.2, 0.5)
            arr.markers.append(m)
            txt = Marker()
            txt.header.frame_id = self.p['map_frame']
            txt.header.stamp = now
            txt.ns = 'rack'
            txt.id = 100 + int(name[-1])
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x, txt.pose.position.y = x, y
            txt.pose.position.z = z + 0.25
            txt.scale.z = 0.12
            txt.color = self._color(1.0, 1.0, 1.0)
            txt.text = name
            arr.markers.append(txt)
        return arr


def main(args=None):
    rclpy.init(args=args)
    node = RackTfPublisher()
    try:
        if node.get_parameter('publish_once').value:
            # /tf_static la latched: 1 spin la du (giong static_transforms.py)
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
