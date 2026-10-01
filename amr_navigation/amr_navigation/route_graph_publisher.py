#!/usr/bin/env python3
"""
route_graph_publisher.py
========================
Publish route graph markers từ route_graph.yaml lên /route_graph/markers topic.
Dùng cho nav2_bringup để hiển thị route graph trên RViz.

Usage:
    ros2 run amr_navigation route_graph_publisher

Author: AMR System
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, Quaternion
import yaml
import math
import os
from ament_index_python.packages import get_package_share_directory


# ============================================================================
# CONSTANTS
# ============================================================================

HOME_NODE_NAME = 'NODE_HOME'

# Cau hinh marker
Z_OFFSET = 0.5

NODE_SCALE = 0.15
TRANSIT_SCALE = 0.11
HOME_SCALE = 0.22

EDGE_SCALE = 0.06

# Kich thuoc mui ten huong node
NODE_ARROW_LENGTH = 0.18
HOME_ARROW_LENGTH = 0.24

NODE_ARROW_WIDTH = 0.045
HOME_ARROW_WIDTH = 0.055

NODE_ARROW_HEIGHT = 0.04

# Khoang cach va kich thuoc ten node
LABEL_OFFSET_Y = 0.17
LABEL_OFFSET_Z = 0.06

LABEL_SCALE = 0.18
HOME_LABEL_SCALE = 0.20
TRANSIT_LABEL_SCALE = 0.16

# Mau sac theo class
# Giong interactive_node_creator.py
COLOR_HOME = (1.0, 0.5, 0.0)       # Cam - HOME, CHARGE
COLOR_PICKUP = (0.0, 0.45, 1.0)    # Xanh duong - PICKUP_*, P_*
COLOR_DROPOFF = (1.0, 0.0, 0.0)    # Do - DROP_*, D_*
COLOR_TRANSIT = (0.2, 0.8, 0.4)    # Xanh la - node trung gian
COLOR_SAFE_ZONE = (0.5, 0.5, 0.5)  # Xam - SAFE_*
COLOR_DEFAULT = (0.5, 0.5, 0.5)    # Xam mac dinh


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


def get_node_color(ntype):
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
    if is_home:
        return HOME_SCALE
    if ntype in ('transit', 'junction'):
        return TRANSIT_SCALE
    return NODE_SCALE


def get_route_graph_path():
    """Lay duong dan route_graph.yaml"""
    pkg_share = get_package_share_directory('amr_navigation')
    return os.path.join(pkg_share, 'config', 'graphs', 'route_graph.yaml')


def load_route_graph(yaml_path: str) -> tuple:
    """Load route graph tu YAML"""
    if not os.path.exists(yaml_path):
        return None, None

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    if not data:
        return {}, []

    nodes = data.get('nodes', {})
    edges = data.get('edges', [])

    return nodes, edges


# ============================================================================
# ROUTE GRAPH PUBLISHER
# ============================================================================

class RouteGraphPublisher(Node):

    def __init__(self):
        super().__init__('route_graph_publisher')

        # Declare parameter cho graph path
        default_path = get_route_graph_path()
        self.declare_parameter('graph_yaml_path', default_path)
        self.graph_yaml_path = self.get_parameter('graph_yaml_path').value

        self.get_logger().info(f'Graph YAML: {self.graph_yaml_path}')

        self.marker_pub = self.create_publisher(
            MarkerArray, '/route_graph/markers', 10
        )

        # Load graph
        self.nodes, self.edges = load_route_graph(self.graph_yaml_path)

        if self.nodes is None:
            self.get_logger().warn(
                f'Route graph not found: {self.graph_yaml_path}'
            )
            return

        self.get_logger().info(
            f'Loaded {len(self.nodes)} nodes, {len(self.edges)} edges'
        )

        # Publish markers
        self._publish_markers()

        # Republish periodically de dam bao RViz nhan
        self.timer = self.create_timer(1.0, self._publish_markers)

    def _publish_markers(self):
        """Publish all route graph markers"""
        if self.nodes is None:
            return

        markers = MarkerArray()
        mid = 0

        # Nodes
        for i, (name, data) in enumerate(self.nodes.items()):
            x = data['position']['x']
            y = data['position']['y']
            z = data['position']['z'] + Z_OFFSET
            yaw = data.get('orientation', {}).get('yaw', 0)
            ntype = data.get('type', 'transit')
            is_home = (name == HOME_NODE_NAME)
            is_transit = ntype in ('transit', 'junction')

            color = get_node_color(ntype)
            scale = get_node_scale(ntype, is_home)

            # SPHERE
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
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
            m.color = make_color(*color)
            markers.markers.append(m)
            mid += 1

            # TEXT LABEL
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = 'labels'
            m.id = i
            m.type = Marker.TEXT_VIEW_FACING
            m.action = Marker.ADD
            m.pose.position.x = x

            # Giu khoang cach text = 0.17 m
            m.pose.position.y = y + LABEL_OFFSET_Y

            m.pose.position.z = z + LABEL_OFFSET_Z
            m.pose.orientation.w = 1.0
            m.text = 'HOME' if is_home else name

            if is_home:
                m.scale.z = HOME_LABEL_SCALE
            elif is_transit:
                m.scale.z = TRANSIT_LABEL_SCALE
            else:
                m.scale.z = LABEL_SCALE

            m.color = make_color(*color)
            markers.markers.append(m)

            # DIRECTION ARROW
            # Giong interactive_node_creator:
            # - HOME / PICKUP / DROP / CHARGE: hien mui ten
            # - TRANSIT / JUNCTION: khong hien mui ten
            if not is_transit:
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = 'arrows'
                m.id = i
                m.type = Marker.ARROW
                m.action = Marker.ADD
                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = z
                m.pose.orientation = yaw_to_quaternion(yaw)

                if is_home:
                    m.scale.x = HOME_ARROW_LENGTH
                    m.scale.y = HOME_ARROW_WIDTH
                else:
                    m.scale.x = NODE_ARROW_LENGTH
                    m.scale.y = NODE_ARROW_WIDTH

                m.scale.z = NODE_ARROW_HEIGHT
                m.color = make_color(*color)
                markers.markers.append(m)

        # Edges
        for e_idx, e in enumerate(self.edges):
            from_node = self.nodes.get(e['from'])
            to_node = self.nodes.get(e['to'])

            if not from_node or not to_node:
                continue

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = self.get_clock().now().to_msg()
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

            if e.get('bidirectional', True):
                m.color = make_color(0.2, 0.8, 0.4)
            else:
                m.color = make_color(0.9, 0.5, 0.2)

            markers.markers.append(m)

        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = RouteGraphPublisher()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()