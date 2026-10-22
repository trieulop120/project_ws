#!/usr/bin/env python3
"""
Pallet Detector Launch File
Launches the pallet_detector_node with configuration.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Get package paths
    pkg_dir = get_package_share_directory('amr_perception')
    config_file = os.path.join(pkg_dir, 'config', 'pallet_detector.yaml')

    # Fallback to source if not installed
    if not os.path.exists(config_file):
        config_file = os.path.join(
            os.path.dirname(os.path.dirname(__file__)),
            'config', 'pallet_detector.yaml'
        )

    # Node configuration
    pallet_detector_node = Node(
        package='amr_perception',
        executable='pallet_detector',
        name='pallet_detector_node',
        output='screen',
        parameters=[{
            'model_path': '',  # Auto-detect weights/best.pt
            'conf_threshold': 0.25,
            'iou_threshold': 0.45,
            'target_frame': 'map',
            'pallet_frame_id': 'pallet_frame',
            'tf_timeout': 1.0,
            'debug_image': True,
        }],
        remappings=[
            ('/color/image_raw', '/color/image_raw'),
            ('/depth/image_raw', '/depth/image_raw'),
            ('/camera/color/camera_info', '/camera/color/camera_info'),
        ],
    )

    return LaunchDescription([
        pallet_detector_node,
    ])
