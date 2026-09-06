"""Perception Launch - Depth Camera to LaserScan and Sensor Fusion

This launch file:
1. Converts depth camera image to LaserScan (depthimage_to_laserscan)
2. Optionally fuses LiDAR /scan with depth camera /scan_depth

Usage:
  ros2 launch amr_perception perception.launch.py          # Full fusion
  ros2 launch amr_perception perception.launch.py lidar_only:=true  # LiDAR only

Remapping:
  Input: /camera/astra_pro/depth/image_raw (from Gazebo simulation)
  Output: /scan_depth (depth camera scan)
         /scan_fused (fused with LiDAR)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    lidar_only_arg = DeclareLaunchArgument(
        'lidar_only',
        default_value='false',
        description='Use only LiDAR (skip depth camera conversion)'
    )

    use_fusion_arg = DeclareLaunchArgument(
        'use_fusion',
        default_value='true',
        description='Fuse LiDAR and depth camera scans'
    )

    lidar_only = LaunchConfiguration('lidar_only')
    use_fusion = LaunchConfiguration('use_fusion')

    # Package paths
    pkg_amr_perception = get_package_share_directory('amr_perception')
    config_file = os.path.join(pkg_amr_perception, 'config', 'depth_to_laserscan.yaml')

    # Environment variables
    env_vars = [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', '/home/trieu/project_ws/src/amr_description'),
    ]

    # ============================================
    # Depth Camera to LaserScan Node
    # ============================================
    depth_to_laserscan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan',
        output='screen',
        parameters=[{
            'scan_height': 10,
            'scan_time': 0.1,
            'range_min': 0.4,
            'range_max': 5.0,
            'output_frame_id': 'camera_link',
            'queue_size': 5,
        }],
        remappings=[
            # Input from Gazebo Astra Pro simulation
            ('depth', '/camera/astra_pro/depth/image_raw'),
            ('depth_camera_info', '/camera/astra_pro/depth/camera_info'),
            # Output scan topic
            ('scan', '/scan_depth'),
        ],
        condition=UnlessCondition(lidar_only),
    )

    # ============================================
    # Scan Fusion Node (Custom)
    # ============================================
    # This merges /scan (LiDAR) and /scan_depth (depth camera)
    # into /scan_fused for use in SLAM and Nav2
    scan_fusion_node = Node(
        package='amr_perception',
        executable='scan_fusion_node',
        name='scan_fusion',
        output='screen',
        parameters=[{
            'scan_topic_lidar': '/scan',
            'scan_topic_depth': '/scan_depth',
            'output_scan_topic': '/scan_fused',
            'queue_size': 10,
            'use_depth_scan': True,
        }],
        condition=IfCondition(use_fusion),
    )

    # ============================================
    # Return LaunchDescription
    # ============================================
    return LaunchDescription([
        # Arguments
        lidar_only_arg,
        use_fusion_arg,

        # Environment
        *env_vars,

        # Nodes
        depth_to_laserscan_node,
        scan_fusion_node,
    ])
