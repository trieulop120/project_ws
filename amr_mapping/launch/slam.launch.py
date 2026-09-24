#!/usr/bin/env python3
"""
SLAM Bringup Real Hardware - Launch EKF + SLAM + OctoMap + Controllers + RViz (No Gazebo)
File location: amr_mapping/launch/slam.launch.py
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
    rviz_config = os.path.join(pkg_amr_mapping, 'rviz', 'slam.rviz')

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
    # OctoMap Server
    # ============================================
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            octomap_params
        ],
        remappings=[
            ('cloud_in', '/points'),
        ],
    )

    # ============================================
    # RViz2
    # ============================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ============================================
    # Return LaunchDescription
    # ============================================
    return LaunchDescription([

        # SLAM
        slam_node,         # <-- Nhận /scan_filtered để dựng bản đồ

        # OctoMap Server
        octomap_server,

        # Visualization
        rviz,
    ])