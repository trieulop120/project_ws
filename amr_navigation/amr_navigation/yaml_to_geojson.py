#!/usr/bin/env python3
"""
yaml_to_geojson.py
==================
ROS2 Node: Chuyển đổi route_graph.yaml sang GeoJSON cho nav2_route.

Nhận paths từ launch file qua parameters:
  - input_yaml: Đường dẫn file YAML input
  - output_geojson: Đường dẫn file GeoJSON output
  - output_poses: Đường dẫn file poses YAML output

Usage:
    ros2 run amr_navigation yaml_to_geojson

Hoặc từ launch file với custom paths:
    ros2 run amr_navigation yaml_to_geojson \
        --ros-args -p input_yaml:=/path/to/input.yaml ...

Author: AMR System
"""

import os
import json
import math
import yaml
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from ament_index_python.packages import get_package_share_directory


class YamlToGeojson(Node):
    """ROS2 Node chuyển đổi route_graph.yaml sang GeoJSON."""

    def __init__(self):
        super().__init__('yaml_to_geojson')

        # Declare parameters với defaults từ package share
        pkg_share = get_package_share_directory('amr_navigation')
        default_graph_dir = os.path.join(pkg_share, 'config', 'graphs')

        self.declare_parameter(
            'input_yaml',
            os.path.join(default_graph_dir, 'route_graph.yaml')
        )
        self.declare_parameter(
            'output_geojson',
            os.path.join(default_graph_dir, 'route_graph.geojson')
        )
        self.declare_parameter(
            'output_poses',
            os.path.join(default_graph_dir, 'route_poses.yaml')
        )
        self.declare_parameter('auto_run', False)  # Default: chờ trigger

        self.input_yaml = self.get_parameter('input_yaml').value
        self.output_geojson = self.get_parameter('output_geojson').value
        self.output_poses = self.get_parameter('output_poses').value
        auto_run = self.get_parameter('auto_run').value

        self.get_logger().info(f'Input YAML: {self.input_yaml}')
        self.get_logger().info(f'Output GeoJSON: {self.output_geojson}')
        self.get_logger().info(f'Output Poses: {self.output_poses}')

        # Subscribe: đợi /graph_saved trigger
        self.trigger_sub = self.create_subscription(
            Empty, '/graph_saved', self._on_graph_saved, 10
        )

        self.get_logger().info('Đợi /graph_saved trigger...')

    def _on_graph_saved(self, msg):
        """Callback khi interactive_node_creator save graph."""
        self.get_logger().info('Nhận /graph_saved - Chờ 2s để đảm bảo file được ghi...')
        # Chờ 2s trước khi convert (đảm bảo file được ghi xong)
        import threading
        threading.Timer(2.0, self.run_conversion).start()

    def load_yaml(self) -> dict:
        """Đọc route_graph.yaml."""
        if not os.path.exists(self.input_yaml):
            self.get_logger().error(f'File not found: {self.input_yaml}')
            return None

        with open(self.input_yaml, 'r') as f:
            data = yaml.safe_load(f)

        if not data:
            self.get_logger().error('Empty YAML file')
            return None

        return data

    def yaw_to_quaternion(self, yaw: float) -> dict:
        """Chuyển yaw sang quaternion."""
        return {
            'x': 0.0,
            'y': 0.0,
            'z': math.sin(yaw / 2.0),
            'w': math.cos(yaw / 2.0)
        }

    def convert_to_geojson(self, data: dict) -> tuple:
        """Chuyển sang định dạng nav2_route GeoJSON."""
        nodes = data.get('nodes', {})
        edges = data.get('edges', [])

        node_id_map = {}
        features = []

        # Convert nodes
        for name in nodes.keys():
            node_id = len(node_id_map)
            node_id_map[name] = node_id

            node_data = nodes[name]
            pos = node_data.get('position', {})

            feature = {
                'type': 'Feature',
                'properties': {
                    'id': node_id,
                    'frame': 'map',
                },
                'geometry': {
                    'type': 'Point',
                    'coordinates': [pos.get('x', 0.0), pos.get('y', 0.0)]
                }
            }
            features.append(feature)

        # Convert edges
        for edge in edges:
            from_name = edge.get('from')
            to_name = edge.get('to')

            if from_name not in node_id_map or to_name not in node_id_map:
                self.get_logger().warn(f'Skip edge {from_name} -> {to_name}: unknown nodes')
                continue

            startid = node_id_map[from_name]
            endid = node_id_map[to_name]

            from_pos = nodes[from_name].get('position', {})
            to_pos = nodes[to_name].get('position', {})

            feature = {
                'type': 'Feature',
                'properties': {
                    'id': len(features),
                    'startid': startid,
                    'endid': endid,
                },
                'geometry': {
                    'type': 'MultiLineString',
                    'coordinates': [[[
                        from_pos.get('x', 0.0),
                        from_pos.get('y', 0.0)
                    ], [
                        to_pos.get('x', 0.0),
                        to_pos.get('y', 0.0)
                    ]]]
                }
            }
            features.append(feature)

        graph_name = data.get('graph_name', 'route_graph')
        output = {
            'type': 'FeatureCollection',
            'name': graph_name,
            'crs': {
                'type': 'name',
                'properties': {'name': 'urn:ogc:def:crs:EPSG::4326'}
            },
            'date_generated': '',
            'features': features
        }

        return output, node_id_map

    def convert_to_poses(self, data: dict, node_id_map: dict) -> dict:
        """Tạo dữ liệu PoseStamped cho các node."""
        nodes = data.get('nodes', {})
        poses = {}

        for name in nodes.keys():
            node_data = nodes[name]
            pos = node_data.get('position', {})
            orientation = node_data.get('orientation', {})

            x = pos.get('x', 0.0)
            y = pos.get('y', 0.0)
            z = pos.get('z', 0.0)
            yaw = orientation.get('yaw', 0.0)
            quaternion = self.yaw_to_quaternion(yaw)

            poses[name] = {
                'id': node_id_map[name],
                'pose_stamped': {
                    'header': {'frame_id': 'map'},
                    'pose': {
                        'position': {'x': x, 'y': y, 'z': z},
                        'orientation': quaternion
                    }
                }
            }

        return poses

    def run_conversion(self):
        """Thực hiện chuyển đổi."""
        self.get_logger().info('Converting route_graph.yaml to GeoJSON...')

        # Load YAML
        data = self.load_yaml()
        if not data:
            return

        # Convert to GeoJSON
        geojson_data, node_id_map = self.convert_to_geojson(data)

        # Save GeoJSON
        os.makedirs(os.path.dirname(self.output_geojson), exist_ok=True)
        with open(self.output_geojson, 'w', encoding='utf-8') as f:
            json.dump(geojson_data, f, indent=2)
        self.get_logger().info(f'Saved GeoJSON: {self.output_geojson}')

        # Convert to poses
        pose_data = self.convert_to_poses(data, node_id_map)
        pose_output = {
            'frame_id': 'map',
            'poses': pose_data
        }

        # Save poses
        with open(self.output_poses, 'w', encoding='utf-8') as f:
            yaml.safe_dump(pose_output, f, sort_keys=False, allow_unicode=True)
        self.get_logger().info(f'Saved Poses: {self.output_poses}')

        # Summary
        node_count = len([f for f in geojson_data['features']
                         if f['geometry']['type'] == 'Point'])
        edge_count = len([f for f in geojson_data['features']
                         if f['geometry']['type'] == 'MultiLineString'])

        self.get_logger().info(f'Nodes: {node_count}, Edges: {edge_count}')
        self.get_logger().info('Done!')


def main():
    rclpy.init()
    node = YamlToGeojson()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
