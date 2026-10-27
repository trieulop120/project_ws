#!/usr/bin/env python3
"""Pallet Detector Launch File: only starts the node.

All settings live in `Config` at the top of amr_perception/pallet_detector_node.py.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_perception',
            executable='pallet_detector',
            name='pallet_detector_node',
            output='screen',
        ),
    ])