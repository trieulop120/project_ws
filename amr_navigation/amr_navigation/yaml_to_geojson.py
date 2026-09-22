#!/usr/bin/env python3
"""
yaml_to_geojson.py
==================
Chuyen doi route_graph.yaml sang dinh dang GeoJSON cho nav2_route
va tao du lieu PoseStamped cho cac node.

Format chuan cua nav2_route:
- Nodes: id (so), frame, geometry: Point
- Edges: id, startid, endid, geometry: MultiLineString

Su dung:
    ros2 run amr_navigation yaml_to_geojson
    python3 -m amr_navigation.yaml_to_geojson

Dau ra:
    config/graphs/route_graph.geojson
    config/graphs/route_poses.yaml

Author: AMR System
"""

import os
import json
import math
import yaml
from ament_index_python.packages import get_package_share_directory


PACKAGE_NAME = 'amr_navigation'


def get_config_dir():
    """Lay duong dan thu muc config graphs trong package share."""
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    return os.path.join(pkg_share, 'config', 'graphs')


def get_yaml_path():
    return os.path.join(get_config_dir(), 'route_graph.yaml')


def get_geojson_path():
    # Luu truc tiep vao src/ de khong bi mat sau build
    return '/home/trieu/project_ws/src/amr_navigation/config/graphs/route_graph.geojson'


def get_pose_path():
    # Luu truc tiep vao src/ de khong bi mat sau build
    return '/home/trieu/project_ws/src/amr_navigation/config/graphs/route_poses.yaml'


def load_yaml():
    """Doc route_graph.yaml"""
    yaml_path = get_yaml_path()

    if not os.path.exists(yaml_path):
        print(f'[ERROR] File not found: {yaml_path}')
        print('[HINT] Chay graph_builder de tao route graph truoc!')
        return None

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    if not data:
        print('[ERROR] Empty YAML file')
        return None

    return data


def convert_to_nav2_route_geojson(data: dict) -> dict:
    """Chuyen doi sang dinh dang nav2_route GeoJSON."""

    nodes = data.get('nodes', {})
    edges = data.get('edges', [])

    # Build node id mapping (sequential)
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
            print(f'[WARN] Skip edge {from_name} -> {to_name}: unknown nodes')
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

    # Build output
    graph_name = data.get('graph_name', 'route_graph')

    output = {
        'type': 'FeatureCollection',
        'name': graph_name,
        'crs': {
            'type': 'name',
            'properties': {'name': 'urn:ogc:def:crs:EPSG::4326'}
        },
        'date_generated': '',  # Can be filled if needed
        'features': features
    }

    return output, node_id_map


def yaw_to_quaternion(yaw):
    """Chuyen yaw sang quaternion."""

    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(yaw / 2.0),
        'w': math.cos(yaw / 2.0)
    }


def convert_to_pose_stamped(data: dict, node_id_map: dict) -> dict:
    """Tao du lieu PoseStamped cho cac node."""

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

        quaternion = yaw_to_quaternion(yaw)

        poses[name] = {
            'id': node_id_map[name],
            'pose_stamped': {
                'header': {
                    'frame_id': 'map'
                },
                'pose': {
                    'position': {
                        'x': x,
                        'y': y,
                        'z': z
                    },
                    'orientation': quaternion
                }
            }
        }

    return poses


def main():
    print('[INFO] Converting route_graph.yaml to GeoJSON...')
    print(f'[INFO] Input:  {get_yaml_path()}')
    print(f'[INFO] Output: {get_geojson_path()}')
    print(f'[INFO] Pose:   {get_pose_path()}')

    # Load YAML
    data = load_yaml()
    if not data:
        return

    # Convert
    geojson_data, node_id_map = convert_to_nav2_route_geojson(data)

    # Save GeoJSON
    os.makedirs(get_config_dir(), exist_ok=True)
    with open(get_geojson_path(), 'w', encoding='utf-8') as f:
        json.dump(geojson_data, f, indent=2)

    print(f'[OK] Saved: {get_geojson_path()}')

    # Convert PoseStamped
    pose_data = convert_to_pose_stamped(data, node_id_map)

    pose_output = {
        'frame_id': 'map',
        'poses': pose_data
    }

    # Save PoseStamped
    with open(get_pose_path(), 'w', encoding='utf-8') as f:
        yaml.safe_dump(
            pose_output,
            f,
            sort_keys=False,
            allow_unicode=True
        )

    print(f'[OK] Saved: {get_pose_path()}')

    # Summary
    print('\n' + '='*50)
    print('  CONVERSION SUMMARY')
    print('='*50)
    print(f'  Graph: {geojson_data.get("name", "N/A")}')
    print(f'  Nodes: {len([f for f in geojson_data["features"] if f["geometry"]["type"] == "Point"])}')
    print(f'  Edges: {len([f for f in geojson_data["features"] if f["geometry"]["type"] == "MultiLineString"])}')
    print('='*50)

    print('\n  Node Mapping (name -> id):')
    for name, nid in sorted(node_id_map.items(), key=lambda x: x[1]):
        print(f'    {nid}: {name}')

    print('\n  PoseStamped:')
    for name, pose in pose_data.items():
        yaw = data['nodes'][name].get('orientation', {}).get('yaw', 0.0)
        print(f'    {name}: x={pose["pose_stamped"]["pose"]["position"]["x"]}, '
              f'y={pose["pose_stamped"]["pose"]["position"]["y"]}, '
              f'yaw={yaw}')

    print('\n[INFO] Done!')
    print(f'[INFO] GeoJSON ready for nav2_route: {get_geojson_path()}')
    print(f'[INFO] PoseStamped data: {get_pose_path()}')


if __name__ == '__main__':
    main()