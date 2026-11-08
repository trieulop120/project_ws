#!/usr/bin/env python3
"""
mission_manager.launch.py
=======================

Launch Mission Manager node + GUI. Start the Nav2 launch first; the manager
loads the active graph from /navigation/route_yaml_path.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Mission Manager node
    mission_manager_node = Node(
        package='amr_mission_manager',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
    )

    # Mission GUI node
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
