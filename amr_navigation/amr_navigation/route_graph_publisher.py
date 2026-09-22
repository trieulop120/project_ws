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

# Cấu hình marker
Z_OFFSET = 0.5
NODE_SCALE = 0.1
HOME_SCALE = 0.125
EDGE_SCALE = 0.04

# Màu sắc
COLOR_HOME = (1.0, 0.5, 0.0)
COLOR_PICKUP = (0.0, 1.0, 0.0)
COLOR_DROPOFF = (1.0, 0.0, 0.0)
COLOR_TRANSIT = (0.0, 0.4, 1.0)
COLOR_SAFE_ZONE = (0.5, 0.5, 0.5)
COLOR_DEFAULT = (0.5, 0.5, 0.5)


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
    return NODE_SCALE


def get_route_graph_path():
    """Lấy đường dẫn route_graph.yaml"""
    pkg_share = get_package_share_directory('amr_navigation')
    return os.path.join(pkg_share, 'config', 'graphs', 'route_graph.yaml')


def load_route_graph():
    """Load route graph từ YAML"""
    yaml_path = get_route_graph_path()

    if not os.path.exists(yaml_path):
        return None, None

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    nodes = data.get('nodes', {})
    edges = data.get('edges', [])

    return nodes, edges


# ============================================================================
# ROUTE GRAPH PUBLISHER
# ============================================================================

class RouteGraphPublisher(Node):

    def __init__(self):
        super().__init__('route_graph_publisher')

        self.marker_pub = self.create_publisher(
            MarkerArray, '/route_graph/markers', 10
        )

        # Load graph
        self.nodes, self.edges = load_route_graph()

        if self.nodes is None:
            self.get_logger().warn(f'Route graph not found, skipping')
            return

        self.get_logger().info(f'Loaded {len(self.nodes)} nodes, {len(self.edges)} edges')

        # Publish markers
        self._publish_markers()

        # Republish periodically để đảm bảo RViz nhận
        self.timer = self.create_timer(1.0, self._publish_markers)

    def _publish_markers(self):
        """Publish all markers"""
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
            m.pose.position.y = y + 0.35
            m.pose.position.z = z + 0.05
            m.pose.orientation.w = 1.0
            m.text = 'HOME' if is_home else name
            m.scale.z = 0.25
            m.color = make_color(*color)
            markers.markers.append(m)

            # DIRECTION ARROW
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
            m.scale.x = 0.4 if is_home else 0.3
            m.scale.y = 0.08 if is_home else 0.05
            m.scale.z = 0.05
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
                Point(x=from_node['position']['x'], y=from_node['position']['y'], z=Z_OFFSET),
                Point(x=to_node['position']['x'], y=to_node['position']['y'], z=Z_OFFSET)
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
