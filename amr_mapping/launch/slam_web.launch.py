#!/usr/bin/env python3
"""
slam_web.launch.py - SLAM Launch for Web Dashboard
==================================================
Giống slam.launch.py nhưng KHÔNG có RViz.
Dùng cho SLAM page trên web dashboard.

Usage:
  ros2 launch amr_mapping slam_web.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================
    # Package paths
    # ============================================
    pkg_amr_mapping = get_package_share_directory('amr_mapping')

    # Config Paths
    slam_config = os.path.join(pkg_amr_mapping, 'config', 'mapper_params_online_async_real.yaml')
    octomap_params = os.path.join(pkg_amr_mapping, 'config', 'octomap_params.yaml')

    # ============================================
    # SLAM Toolbox
    # ============================================
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            slam_config
        ],
    )



    # ============================================
    # Return LaunchDescription (NO RViz)
    # ============================================
    return LaunchDescription([

        # SLAM Toolbox - tạo bản đồ từ /scan_filtered
        slam_node,

    ])
