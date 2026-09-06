"""Launch SLAM toolbox for 2D mapping.

Subscribes to:
  - /scan (LiDAR scan from Sllidar A1M8)
  - /tf (odom -> base_footprint from EKF)

Publishes:
  - /map (occupancy grid)
  - /map_metadata
  - /slam_toolbox/scan_visualization
  - TF: map -> odom
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_mapping = get_package_share_directory('amr_mapping')
    config_file = os.path.join(pkg_mapping, 'config', 'slam.yaml')

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            config_file: config_file,
        }],
        remappings=[
            ('/scan', '/scan'),
        ]
    )

    return LaunchDescription([
        slam_node,
    ])
