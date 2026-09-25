#!/usr/bin/env python3
"""
mission_manager.launch.py
=======================

Launch Mission Manager node + GUI.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# ============================================
# PATHS - Chỉnh sửa ở đây
# ============================================
GRAPH_PACKAGE = 'amr_navigation'              # Package chứa graph
GRAPH_YAML_FILENAME = 'real_route_graph.yaml'    # File YAML trong config/graphs/


def generate_launch_description():
    # Launch arguments - dùng giá trị từ PATHS
    graph_package_arg = DeclareLaunchArgument(
        'graph_package',
        default_value=GRAPH_PACKAGE,
        description='ROS package chứa route_graph.yaml'
    )

    graph_yaml_filename_arg = DeclareLaunchArgument(
        'graph_yaml_filename',
        default_value=GRAPH_YAML_FILENAME,
        description='Tên file YAML trong <package>/config/graphs/'
    )

    # Mission Manager node
    mission_manager_node = Node(
        package='amr_mission_manager',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        parameters=[{
            'graph_package': LaunchConfiguration('graph_package'),
            'graph_yaml_filename': LaunchConfiguration('graph_yaml_filename'),
        }],
    )

    # Mission GUI node
    mission_gui_node = Node(
        package='amr_mission_manager',
        executable='mission_gui',
        name='mission_gui',
        output='screen',
    )

    return LaunchDescription([
        graph_package_arg,
        graph_yaml_filename_arg,
        mission_manager_node,
        mission_gui_node,
    ])
