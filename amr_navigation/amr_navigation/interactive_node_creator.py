#!/usr/bin/env python3
"""
interactive_node_creator.py
==========================
ROS2 Node tao route graph tren RViz2.

Tinh nang:
- Subscribe /clicked_point (Publish Point - lay x,y, yaw=0)
- Subscribe /goal_pose (2D Goal Pose - lay x,y,yaw)
- Chi can 1 lenh: ros2 launch amr_mission_manager graph_builder.launch.py

Su dung:
    ros2 launch amr_navigation graph_builder.launch.py

Author: AMR System
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, Empty
import yaml
import os
import math
import sys
import select
from ament_index_python.packages import get_package_share_directory


# ============================================================================
# CONSTANTS
# ============================================================================

HOME_NODE_NAME = 'NODE_HOME'

HOME_NODE = {
    'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
    'orientation': {'yaw': 0.0},
    'type': 'home',
    'floor': 1,
    'description': 'Diem home - vi tri mac dinh',
    'properties': {
        'is_home': True,
        'is_parking': True,
    }
}


# ============================================================================
# CAU HINH MARKER
# ============================================================================

Z_OFFSET = 0.5

# Kich thuoc node
NODE_SCALE = 0.15          # Node PICKUP / DROP
TRANSIT_SCALE = 0.10       # Node trung gian nho hon
HOME_SCALE = 0.22          # HOME lon hon

# Do day duong
EDGE_SCALE = 0.06

# Kich thuoc mui ten huong cua node
NODE_ARROW_LENGTH = 0.18
HOME_ARROW_LENGTH = 0.24

NODE_ARROW_WIDTH = 0.045
HOME_ARROW_WIDTH = 0.055
NODE_ARROW_HEIGHT = 0.04

# Vi tri hien thi ten node
# Dat gan node hon nhung van tach khoi node
LABEL_OFFSET_Y = 0.17
LABEL_OFFSET_Z = 0.05

# Co chu
LABEL_SCALE = 0.22
HOME_LABEL_SCALE = 0.24
TRANSIT_LABEL_SCALE = 0.18


# ============================================================================
# MAU SAC
# ============================================================================

# HOME / CHARGE - Cam
COLOR_HOME = (1.0, 0.5, 0.0)

# PICKUP - Xanh duong
COLOR_PICKUP = (0.0, 0.45, 1.0)

# DROP - Do
COLOR_DROPOFF = (1.0, 0.0, 0.0)

# NODE TRUNG GIAN - Xanh la
# Giong mau voi EDGE 2 chieu
COLOR_TRANSIT = (0.2, 0.8, 0.4)

# SAFE - Xam
COLOR_SAFE_ZONE = (0.5, 0.5, 0.5)

# Mac dinh
COLOR_DEFAULT = (0.5, 0.5, 0.5)


# Mau duong giu nguyen nhu code cu
COLOR_EDGE_BIDIRECTIONAL = (0.2, 0.8, 0.4)
COLOR_EDGE_UNIDIRECTIONAL = (0.9, 0.5, 0.2)


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================

def make_color(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = r, g, b, a
    return c


def yaw_to_quaternion(yaw):
    q = Quaternion()
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    q.x, q.y, q.z, q.w = 0.0, 0.0, sy, cy
    return q


def quaternion_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def calc_distance(p1, p2):
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    return math.sqrt(dx * dx + dy * dy)


def parse_args():
    """Parse command line arguments"""
    load_existing = '--load-existing' in sys.argv or '--load' in sys.argv

    if load_existing:
        print('[INFO] Che do: LOAD EXISTING')
    else:
        print('[INFO] Che do: CLEAN SLATE')

    return load_existing


def infer_node_type_from_name(name: str) -> str:
    """Tu dong gan class dua tren ten node"""
    upper = name.upper()

    # Kiem tra truoc cac truong hop dac biet
    if upper.startswith('HOME'):
        return 'home'

    elif upper.startswith('PICKUP') or upper.startswith('P_'):
        return 'pickup_approach'

    elif upper.startswith('DROP') or upper.startswith('D_'):
        return 'dropoff_approach'

    elif upper.startswith('SAFE'):
        return 'safe_zone'

    elif upper.startswith('CHARGE') or upper.startswith('C_'):
        return 'charging'

    else:
        # J*, WAYPOINT*, va cac ten khac -> transit
        return 'transit'


def get_node_color(ntype: str):
    """Tra ve mau RGB theo loai node"""

    if ntype in ('home', 'charging'):
        return COLOR_HOME

    elif ntype in ('pickup_approach', 'pickup'):
        return COLOR_PICKUP

    elif ntype in ('dropoff_approach', 'dropoff'):
        return COLOR_DROPOFF

    elif ntype == 'safe_zone':
        return COLOR_SAFE_ZONE

    elif ntype in ('transit', 'junction'):
        return COLOR_TRANSIT

    else:
        return COLOR_DEFAULT


def get_node_scale(ntype, is_home=False):
    """Lay kich thuoc marker theo loai"""

    if is_home:
        return HOME_SCALE

    # Node trung gian nho hon
    if ntype in ('transit', 'junction'):
        return TRANSIT_SCALE

    return NODE_SCALE


def get_label_scale(ntype, is_home=False):
    """Lay kich thuoc chu theo loai node"""

    if is_home:
        return HOME_LABEL_SCALE

    if ntype in ('transit', 'junction'):
        return TRANSIT_LABEL_SCALE

    return LABEL_SCALE


def generate_default_name(existing_nodes: dict) -> str:
    """Sinh ten mac dinh NODE_01, NODE_02..."""
    i = 1

    while f'NODE_{i:02d}' in existing_nodes:
        i += 1

    return f'NODE_{i:02d}'


# ============================================================================
# INTERACTIVE NODE CREATOR
# ============================================================================

class InteractiveNodeCreator(Node):

    def __init__(self):
        super().__init__('interactive_node_creator')

        # === SỬA: Nhận graph_filepath từ Launch File ===
        default_yaml_path = os.path.join(
            get_package_share_directory('amr_navigation'),
            'config', 'graphs', 'route_graph.yaml'
        )

        self.declare_parameter('graph_filepath', default_yaml_path)
        self.yaml_file = self.get_parameter('graph_filepath').value

        # === SỬA: Nhận load_existing từ Launch File ===
        self.declare_parameter('load_existing', False)
        self.load_existing = bool(
            self.get_parameter('load_existing').value
        )

        config_dir = os.path.dirname(self.yaml_file)

        if not os.path.exists(config_dir):
            try:
                os.makedirs(config_dir, exist_ok=True)

            except Exception as e:
                self.get_logger().error(
                    f"Khong the tao thu muc: {config_dir}. Loi: {e}"
                )

        self.nodes = {}
        self.edges = []
        self.pending = None

        self._inject_home_node()

        from rclpy.qos import QoSProfile, HistoryPolicy

        # Match RViz config: Volatile
        qos_profile = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/route_graph/markers',
            qos_profile=qos_profile
        )

        # Publisher: thong bao khi graph duoc save
        self.save_pub = self.create_publisher(
            Empty,
            '/graph_saved',
            10
        )

        # === SUBSCRIBERS ===

        # /clicked_point: Publish Point tren RViz2
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self._on_clicked_point,
            10
        )

        # /goal_pose: 2D Goal Pose tren RViz2
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self._on_goal_pose,
            10
        )

        if self.load_existing:
            self._load_graph()

        self._publish_markers()

        self.create_timer(
            1.0,
            self._publish_markers
        )

        self._print_help()
        self._print_pending_prompt()

        self.get_logger().info(
            'Khoi tao thanh cong - NODE_HOME tai (0, 0)'
        )

    # =========================================================================
    # HOME NODE
    # =========================================================================

    def _inject_home_node(self):
        self.nodes[HOME_NODE_NAME] = HOME_NODE.copy()

        print(
            f'[HOME] Da tao {HOME_NODE_NAME} tai (0.0, 0.0)'
        )

    def _set_home_to_pending(self):
        """Dua NODE_HOME den vi tri pending"""

        if not self.pending:
            print('  [ERROR] Chua co vi tri de dat HOME')
            return False

        pos = self.pending

        self.nodes[HOME_NODE_NAME]['position'] = {
            'x': round(pos['x'], 3),
            'y': round(pos['y'], 3),
            'z': round(pos['z'], 3)
        }

        self.nodes[HOME_NODE_NAME]['orientation']['yaw'] = round(
            pos['yaw'],
            4
        )

        self._publish_markers()

        new_pos = self.nodes[HOME_NODE_NAME]['position']

        print(
            f'  [OK] NODE_HOME -> '
            f'({new_pos["x"]}, {new_pos["y"]})'
        )

        return True

    # =========================================================================
    # DATA LOAD/SAVE
    # =========================================================================

    def _load_graph(self):

        if os.path.exists(self.yaml_file):

            try:
                with open(self.yaml_file, 'r') as f:
                    data = yaml.safe_load(f)

                    if data:
                        loaded_nodes = data.get(
                            'nodes',
                            {}
                        )

                        loaded_edges = data.get(
                            'edges',
                            []
                        )

                        for name, node_data in loaded_nodes.items():

                            self.nodes[name] = node_data

                        self.edges = loaded_edges

                print(
                    f'[LOAD] Tai: '
                    f'{len(self.nodes)} nodes, '
                    f'{len(self.edges)} edges'
                )

            except Exception as e:
                print(f'[WARN] Loi load: {e}')

    def _save_graph(self):

        data = {
            'graph_name': 'my_route_graph',
            'version': '1.0',
            'description':
                'Route graph - Generated by Interactive Node Creator',
            'nodes': self.nodes,
            'edges': self.edges,
        }

        with open(self.yaml_file, 'w') as f:
            yaml.dump(
                data,
                f,
                default_flow_style=False,
                sort_keys=False
            )

        print(f'[SAVE] {self.yaml_file}')

        print(
            f'       Nodes: {len(self.nodes)} | '
            f'Edges: {len(self.edges)}'
        )

        # Publish de yaml_to_geojson tu dong convert
        self.save_pub.publish(Empty())

        print(
            '       [AUTO-CONVERT] '
            'Dang chuyen doi GeoJSON...'
        )

    # =========================================================================
    # CALLBACKS - Chi dat pending, KHONG block
    # =========================================================================

    def _on_clicked_point(self, msg: PointStamped):
        """Callback khi nguoi dung click Publish Point"""

        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        yaw = 0.0

        print('\n' + '=' * 55)

        print(
            '  [PUBLISH POINT] Toa do: '
            'x={:.3f}, y={:.3f} | Yaw: 0.0 deg'
            .format(x, y)
        )

        print('=' * 55)

        print(
            '  Nhap ten node '
            '(VD: J1, PICKUP_A): ',
            end='',
            flush=True
        )

        self.pending = {
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw
        }

    def _on_goal_pose(self, msg: PoseStamped):
        """Callback khi nguoi dung click 2D Goal Pose"""

        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        yaw = quaternion_to_yaw(
            msg.pose.orientation
        )

        yaw_deg = math.degrees(yaw)

        print('\n' + '=' * 55)

        print(
            '  [2D GOAL POSE] Toa do: '
            'x={:.3f}, y={:.3f} | Yaw: {:.1f} deg'
            .format(x, y, yaw_deg)
        )

        print('=' * 55)

        print(
            '  Nhap ten node '
            '(VD: J1, PICKUP_A): ',
            end='',
            flush=True
        )

        self.pending = {
            'x': x,
            'y': y,
            'z': z,
            'yaw': yaw
        }

    def _print_pending_prompt(self):
        """In prompt cho viec nhap ten node"""

        if self.pending:

            pos = self.pending

            yaw_deg = math.degrees(
                pos['yaw']
            )

            # Xac dinh nguon trigger
            if abs(pos['yaw']) < 0.001:
                source = '[PUBLISH POINT]'
            else:
                source = '[2D GOAL POSE]'

            print('\n' + '=' * 55)

            print(
                f'  {source} - CLICK DETECTED'
            )

            print('-' * 55)

            print(
                f'  TOA DO: '
                f'x={pos["x"]:.3f}, '
                f'y={pos["y"]:.3f}, '
                f'z={pos["z"]:.3f}'
            )

            print(
                f'  YAW:    '
                f'{pos["yaw"]:.4f} rad '
                f'({yaw_deg:.1f} deg)'
            )

            print('-' * 55)

            print(
                '  Nhap ten: '
                'PICKUP_01, DROP_A, J1... '
                '(Enter = mac dinh)'
            )

            print(
                '  Huy:      c hoac cancel'
            )

            print(
                '  Dat HOME: sh'
            )

            print('=' * 55)

            print(
                '  >>> ',
                end='',
                flush=True
            )

    # =========================================================================
    # XU LY NODE
    # =========================================================================

    def _process_node_name(self, raw_input: str):
        """Xu ly input ten node"""

        if not self.pending:
            return

        # Strip cac ky tu rac
        name = raw_input.strip()

        # Huy bo
        if name.lower() in (
            'c',
            'cancel',
            'huy'
        ):
            print('  [CANCEL] Da huy bo')

            self.pending = None

            return

        # Dat HOME
        if name.lower() in (
            'sh',
            'sethome',
            'home'
        ):
            self._set_home_to_pending()

            self.pending = None

            return

        # Ten rong -> sinh ten mac dinh
        if not name:
            name = generate_default_name(
                self.nodes
            )

            print(
                f'  [AUTO] Ten mac dinh: {name}'
            )

        # Kiem tra trung ten
        if name in self.nodes:

            print(
                f'  [ERROR] Node "{name}" da ton tai!'
            )

            self.pending = None

            return

        # Tu dong xac dinh loai tu ten
        ntype = infer_node_type_from_name(name)

        self._add_node(
            name,
            ntype
        )

    def _add_node(self, name: str, ntype: str):
        """Them node moi"""

        if not self.pending:
            return

        pos = self.pending

        node = {
            'position': {
                'x': round(pos['x'], 3),
                'y': round(pos['y'], 3),
                'z': round(pos['z'], 3)
            },
            'orientation': {
                'yaw': round(
                    pos['yaw'],
                    4
                )
            },
            'type': ntype,
            'floor': 1,
            'description': f'Node {name}',
        }

        self.nodes[name] = node

        self.pending = None

        self._publish_markers()

        pos_out = node['position']

        print(
            f'  [OK] {name} [{ntype}] at '
            f'({pos_out["x"]}, {pos_out["y"]})'
        )

    def _add_edge(
        self,
        from_name: str,
        to_name: str,
        bidirectional: bool = True
    ):

        if from_name not in self.nodes:
            print(
                f'  [ERROR] Node "{from_name}" khong ton tai'
            )

            return False

        if to_name not in self.nodes:
            print(
                f'  [ERROR] Node "{to_name}" khong ton tai'
            )

            return False

        for e in self.edges:

            if (
                e['from'] == from_name
                and
                e['to'] == to_name
            ):
                print(
                    '  [ERROR] Edge da ton tai'
                )

                return False

        p1 = self.nodes[from_name]['position']
        p2 = self.nodes[to_name]['position']

        dist = calc_distance(
            (
                p1['x'],
                p1['y']
            ),
            (
                p2['x'],
                p2['y']
            )
        )

        edge = {
            'from': from_name,
            'to': to_name,
            'bidirectional': bidirectional,
            'weight': 1.0,
            'distance': round(dist, 2),
        }

        self.edges.append(edge)

        self._publish_markers()

        direction = (
            '<->'
            if bidirectional
            else
            '->'
        )

        print(
            f'  [OK] Edge: '
            f'{from_name} {direction} {to_name} '
            f'({dist:.2f}m)'
        )

        return True

    def _delete_node(self, name: str):

        if name == HOME_NODE_NAME:
            print(
                '  [ERROR] Khong the xoa NODE_HOME!'
            )

            return False

        if name not in self.nodes:
            print(
                f'  [ERROR] Node "{name}" khong ton tai'
            )

            return False

        # Dem so edge bi anh huong
        edges_removed = [
            e
            for e in self.edges
            if e['from'] == name
            or e['to'] == name
        ]

        edge_count = len(edges_removed)

        # Xoa node
        del self.nodes[name]

        # Xoa tat ca edge lien quan
        self.edges = [
            e
            for e in self.edges
            if e['from'] != name
            and e['to'] != name
        ]

        # Refresh markers tren RViz2
        self._publish_markers()

        if edge_count > 0:
            print(
                f'  [OK] Da xoa Node [{name}] '
                f'va {edge_count} Edge lien quan.'
            )

        else:
            print(
                f'  [OK] Da xoa Node [{name}].'
            )

        return True

    def _delete_edge(
        self,
        from_name: str,
        to_name: str
    ):
        """Xoa edge giua 2 node"""

        # Tim edge tu A -> B
        edge_ab = None
        edge_ba = None

        for e in self.edges:

            if (
                e['from'] == from_name
                and
                e['to'] == to_name
            ):
                edge_ab = e

            if (
                e['from'] == to_name
                and
                e['to'] == from_name
            ):
                edge_ba = e

        if (
            edge_ab is None
            and
            edge_ba is None
        ):
            print(
                f'  [ERROR] Edge giua '
                f'"{from_name}" va "{to_name}" '
                f'khong ton tai'
            )

            return False

        # Xoa edge A -> B
        if edge_ab:

            self.edges.remove(edge_ab)

            bidir_ab = (
                '⟷'
                if edge_ab.get(
                    'bidirectional',
                    True
                )
                else
                '→'
            )

            print(
                f'  [OK] Da xoa Edge: '
                f'{from_name} {bidir_ab} {to_name}'
            )

        # Xoa edge B -> A
        if edge_ba:

            self.edges.remove(edge_ba)

            bidir_ba = (
                '⟷'
                if edge_ba.get(
                    'bidirectional',
                    True
                )
                else
                '→'
            )

            print(
                f'  [OK] Da xoa Edge: '
                f'{to_name} {bidir_ba} {from_name}'
            )

        # Refresh markers tren RViz2
        self._publish_markers()

        print(
            f'  [OK] Da xoa Edge giua '
            f'"{from_name}" va "{to_name}".'
        )

        return True

    def _list_nodes(self):

        if not self.nodes:
            print(
                '\n  Chua co node nao (chi co HOME)'
            )

            return

        print('\n' + '=' * 65)

        print(
            f'  NODES: {len(self.nodes)} | '
            f'EDGES: {len(self.edges)}'
        )

        print('=' * 65)

        for name in sorted(
            self.nodes.keys()
        ):

            data = self.nodes[name]

            pos = data['position']

            yaw = data.get(
                'orientation',
                {}
            ).get(
                'yaw',
                0
            )

            ntype = data.get(
                'type',
                'unknown'
            )

            marker = (
                '🏠 '
                if name == HOME_NODE_NAME
                else
                '  '
            )

            print(
                f'{marker}'
                f'{name:20} '
                f'({pos["x"]:7.2f}, '
                f'{pos["y"]:7.2f}) '
                f'yaw={math.degrees(yaw):5.0f}deg '
                f'[{ntype}]'
            )

        print('\n  EDGES:')

        if self.edges:

            for e in self.edges:

                bidir = (
                    '⟷'
                    if e.get(
                        'bidirectional',
                        True
                    )
                    else
                    '→'
                )

                print(
                    f'    {e["from"]:15} '
                    f'{bidir} '
                    f'{e["to"]:15} '
                    f'({e.get("distance", 0):.2f}m)'
                )

        else:
            print(
                '    (chua co edge)'
            )

        print(
            '=' * 65 + '\n'
        )

    # =========================================================================
    # MARKER PUBLISHING
    # =========================================================================

    def _publish_markers(self):
        """
        Publish markers tren RViz2.

        Quy tac hien thi:
        - HOME: node lon, mau cam, mui ten nho vua phai
        - PICKUP: node xanh duong
        - DROP: node do
        - TRANSIT/JUNCTION: node nho, mau xanh la
        - Node trung gian khong hien mui ten huong
        - Ten node dat gan node nhung khong de len node
        - Edge 2 chieu: xanh la
        - Edge 1 chieu: cam
        """

        markers = MarkerArray()

        mid = 0

        # =====================================================================
        # NODES
        # =====================================================================

        for i, (name, data) in enumerate(
            self.nodes.items()
        ):

            x = data['position']['x']
            y = data['position']['y']

            z = (
                data['position']['z']
                +
                Z_OFFSET
            )

            yaw = data.get(
                'orientation',
                {}
            ).get(
                'yaw',
                0
            )

            ntype = data.get(
                'type',
                'transit'
            )

            is_home = (
                name == HOME_NODE_NAME
            )

            color = get_node_color(
                ntype
            )

            scale = get_node_scale(
                ntype,
                is_home
            )

            label_scale = get_label_scale(
                ntype,
                is_home
            )

            # =================================================================
            # SPHERE NODE
            # =================================================================

            m = Marker()

            m.header.frame_id = 'map'
            m.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            m.ns = 'nodes'
            m.id = mid

            m.type = Marker.SPHERE
            m.action = Marker.ADD

            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z

            m.pose.orientation.w = 1.0

            m.scale.x = scale
            m.scale.y = scale
            m.scale.z = scale

            m.color = make_color(
                *color
            )

            markers.markers.append(m)

            mid += 1

            # =================================================================
            # TEXT LABEL
            # =================================================================

            m = Marker()

            m.header.frame_id = 'map'
            m.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            m.ns = 'labels'
            m.id = i

            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD

            m.pose.position.x = x

            # Dua chu gan node hon.
            # Van giu khoang cach de khong cham node.
            m.pose.position.y = (
                y
                +
                LABEL_OFFSET_Y
            )

            m.pose.position.z = (
                z
                +
                LABEL_OFFSET_Z
            )

            m.pose.orientation.w = 1.0

            m.text = (
                'HOME'
                if is_home
                else
                name
            )

            m.scale.z = label_scale

            # Chu dung cung mau voi node
            m.color = make_color(
                *color
            )

            markers.markers.append(m)

            # =================================================================
            # DIRECTION ARROW
            # =================================================================
            #
            # Node trung gian khong hien mui ten.
            # Chi HOME / PICKUP / DROP / cac node co yaw moi hien.
            #

            if ntype not in (
                'transit',
                'junction'
            ):

                m = Marker()

                m.header.frame_id = 'map'
                m.header.stamp = (
                    self.get_clock()
                    .now()
                    .to_msg()
                )

                m.ns = 'arrows'
                m.id = i

                m.type = Marker.ARROW
                m.action = Marker.ADD

                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = z

                m.pose.orientation = (
                    yaw_to_quaternion(yaw)
                )

                # Mui ten da duoc thu nho lai
                m.scale.x = (
                    HOME_ARROW_LENGTH
                    if is_home
                    else
                    NODE_ARROW_LENGTH
                )

                m.scale.y = (
                    HOME_ARROW_WIDTH
                    if is_home
                    else
                    NODE_ARROW_WIDTH
                )

                m.scale.z = NODE_ARROW_HEIGHT

                m.color = make_color(
                    *color
                )

                markers.markers.append(m)

        # =====================================================================
        # EDGES
        # =====================================================================

        for e_idx, e in enumerate(
            self.edges
        ):

            from_node = self.nodes.get(
                e['from']
            )

            to_node = self.nodes.get(
                e['to']
            )

            if not from_node or not to_node:
                continue

            # ================================================================
            # ROAD
            # ================================================================

            m = Marker()

            m.header.frame_id = 'map'
            m.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            m.ns = 'edges'
            m.id = e_idx

            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD

            m.points = [
                Point(
                    x=from_node['position']['x'],
                    y=from_node['position']['y'],
                    z=Z_OFFSET
                ),
                Point(
                    x=to_node['position']['x'],
                    y=to_node['position']['y'],
                    z=Z_OFFSET
                )
            ]

            m.scale.x = EDGE_SCALE

            # ================================================================
            # MAU DUONG
            # ================================================================

            if e.get(
                'bidirectional',
                True
            ):

                m.color = make_color(
                    *COLOR_EDGE_BIDIRECTIONAL
                )

            else:

                m.color = make_color(
                    *COLOR_EDGE_UNIDIRECTIONAL
                )

            markers.markers.append(m)

        # =====================================================================
        # PUBLISH
        # =====================================================================

        self.marker_pub.publish(
            markers
        )

    # =========================================================================
    # COMMAND LINE - NON-BLOCKING
    # =========================================================================

    def _print_help(self):

        print('\n' + '=' * 65)

        print(
            '  INTERACTIVE NODE CREATOR'
        )

        print('=' * 65)

        print()

        print('  TREN RVIZ2:')

        print(
            '    • Publish Point (🔵) - '
            'Lay (x, y), yaw=0'
        )

        print(
            '    • 2D Goal Pose (🎯)  - '
            'Lay (x, y, yaw)'
        )

        print()

        print(
            '  QUY TAC DAT TEN '
            '(tu dong gan class):'
        )

        print(
            '    PICKUP_* / P_*  -> '
            'pickup_approach (xanh duong)'
        )

        print(
            '    DROP_*   / D_*  -> '
            'dropoff_approach (do)'
        )

        print(
            '    HOME*          -> '
            'home (cam)'
        )

        print(
            '    SAFE_*         -> '
            'safe_zone (xam)'
        )

        print(
            '    CHARGE* / C_*  -> '
            'charging (cam)'
        )

        print(
            '    J*, WAYPOINT*  -> '
            'transit (xanh la)'
        )

        print()

        print('  LENH:')

        print(
            '    l           '
            'Danh sach nodes/edges'
        )

        print(
            '    e A B       '
            'Edge A <-> B (2 chieu)'
        )

        print(
            '    e A B n     '
            'Edge A -> B (1 chieu)'
        )

        print(
            '    de A B      '
            'Xoa edge giua A va B'
        )

        print(
            '    d ten       '
            'Xoa node + cac edge lien quan'
        )

        print(
            '    sh          '
            'Dat HOME tai vi tri vua click'
        )

        print(
            '    s           '
            'Luu file YAML'
        )

        print(
            '    h           '
            'Help'
        )

        print(
            '    q           '
            'Thoat (tu dong luu)'
        )

        print()

        print(
            f'  HIENTAI: '
            f'{len(self.nodes)} nodes | '
            f'{len(self.edges)} edges'
        )

        print(
            f'  FILE: {self.yaml_file}'
        )

        print(
            f'  CHE DO: '
            f'{"LOAD" if self.load_existing else "CLEAN SLATE"}'
        )

        print(
            '=' * 65 + '\n'
        )

    def run_command_loop(self):
        """Vong lap lenh non-blocking - xu ly tat ca input"""

        print(
            'Dang cho input... '
            '(Go "h" de xem help)\n'
        )

        while rclpy.ok():

            # Non-blocking check stdin
            if select.select(
                [sys.stdin],
                [],
                [],
                0.1
            )[0]:

                try:

                    line = sys.stdin.readline()

                    if not line:
                        break

                    # Strip ky tu rac
                    raw = line.strip()

                    # Neu co pending -> xu ly ten node
                    if self.pending:

                        self._process_node_name(
                            raw
                        )

                    else:

                        # Xu ly lenh binh thuong
                        cmd = raw.split()

                        if not cmd:
                            continue

                        op = cmd[0].lower()

                        if op == 'q':
                            break

                        elif op == 'h':
                            self._print_help()

                        elif op == 'l':
                            self._list_nodes()

                        elif op == 's':
                            self._save_graph()

                        elif (
                            op == 'sh'
                            or
                            op == 'sethome'
                        ):

                            print(
                                '  [INFO] Click tren RViz2 '
                                'truoc, sau do "sh"'
                            )

                        elif (
                            op == 'e'
                            and
                            len(cmd) >= 3
                        ):

                            bidir = (
                                cmd[3].lower() != 'n'
                                if len(cmd) > 3
                                else
                                True
                            )

                            self._add_edge(
                                cmd[1],
                                cmd[2],
                                bidir
                            )

                        elif (
                            op == 'd'
                            and
                            len(cmd) >= 2
                        ):

                            self._delete_node(
                                cmd[1]
                            )

                        elif (
                            op == 'de'
                            and
                            len(cmd) >= 3
                        ):

                            self._delete_edge(
                                cmd[1],
                                cmd[2]
                            )

                        else:

                            if raw:

                                print(
                                    '  [INFO] '
                                    'Click/Set Goal tren RViz2 truoc, '
                                    'hoac go "h" de xem help'
                                )

                    # In lai prompt neu con pending
                    if self.pending:
                        self._print_pending_prompt()

                except (
                    EOFError,
                    KeyboardInterrupt
                ):
                    break

        self._save_graph()


# ============================================================================
# MAIN
# ============================================================================

def main():

    import logging

    rclpy_logger = logging.getLogger(
        'rclpy'
    )

    rclpy_logger.setLevel(
        logging.WARNING
    )

    rclpy.init()

    node = InteractiveNodeCreator()

    if node.load_existing:

        print(
            '[INFO] Che do: LOAD EXISTING'
        )

    else:

        print(
            '[INFO] Che do: CLEAN SLATE'
        )

    try:

        import threading

        spin_thread = threading.Thread(
            target=rclpy.spin,
            args=(node,),
            daemon=True
        )

        spin_thread.start()

        node.run_command_loop()

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()