#!/usr/bin/env python3
"""
mission_manager.launch.py
=======================

Launch Mission Manager node + GUI for testing.

Usage:
    ros2 launch amr_mission_manager mission_manager.launch.py

Author: AMR System
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_manager_node = Node(
        package='amr_mission_manager',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
    )

    mission_gui_node = Node(
        package='amr_mission_manager',
        executable='mission_gui',
        name='mission_gui',
        output='screen',
    )

    return LaunchDescription([
        mission_manager_node,
        mission_gui_node,
    ])
