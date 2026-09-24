#!/usr/bin/env python3
"""
Graph Builder Launch File (Simulation)

Tạo/sửa route graph trên simulation với map Gazebo.

Usage:
    ros2 launch amr_navigation graph_builder.launch.py
    ros2 launch amr_navigation graph_builder.launch.py load_existing:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('amr_navigation')
    pkg_mapping = get_package_share_directory('amr_mapping')

    # ============================================
    # PATHS - Chỉnh sửa ở đây
    # ============================================
    input_yaml = os.path.join(pkg_nav, 'config', 'graphs', 'route_graph.yaml')
    output_geojson = os.path.join(pkg_nav, 'config', 'graphs', 'route_graph.geojson')
    output_poses = os.path.join(pkg_nav, 'config', 'graphs', 'route_poses.yaml')
    map_yaml = os.path.join(pkg_mapping, 'maps', 'amr_map.yaml')
    rviz_config = os.path.join(pkg_nav, 'rviz', 'route_builder.rviz')

    # ============================================
    # Launch Arguments
    # ============================================
    load_existing_arg = DeclareLaunchArgument(
        'load_existing',
        default_value='false',
        description='Load existing graph (true) or start fresh (false)'
    )
    load_existing = LaunchConfiguration('load_existing')

    # ============================================
    # Return LaunchDescription
    # ============================================
    return LaunchDescription([
        SetEnvironmentVariable(name='MESA_GL_VERSION_OVERRIDE', value='3.3'),

        load_existing_arg,

        LogInfo(msg='[GRAPH_BUILDER] Khởi tạo simulation...'),
        LogInfo(msg=f'  Input YAML: {input_yaml}'),
        LogInfo(msg=f'  Output GeoJSON: {output_geojson}'),
        LogInfo(msg=f'  Map: {map_yaml}'),
        LogInfo(condition=IfCondition(load_existing), msg='  Mode: LOAD EXISTING'),
        LogInfo(condition=UnlessCondition(load_existing), msg='  Mode: CLEAN SLATE'),

        # Static TF: map -> base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map_basefootprint',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_footprint'],
            parameters=[{'use_sim_time': False}],
        ),

        # Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': False}],
            output='screen',
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[{'autostart': True, 'node_names': ['map_server'], 'use_sim_time': False}],
        ),

        # Interactive Node Creator
        Node(
            package='amr_navigation',
            executable='interactive_node_creator',
            name='interactive_node_creator',
            output='screen',
            parameters=[{
                'load_existing': load_existing,
                'graph_filepath': input_yaml,
            }],
            prefix=['gnome-terminal --title="Node Creator CLI" --'],
        ),

        # YAML to GeoJSON Converter
        Node(
            package='amr_navigation',
            executable='yaml_to_geojson',
            name='yaml_to_geojson',
            output='screen',
            parameters=[{
                'input_yaml': input_yaml,
                'output_geojson': output_geojson,
                'output_poses': output_poses,
                'auto_run': False,
            }],
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
        ),
    ])
