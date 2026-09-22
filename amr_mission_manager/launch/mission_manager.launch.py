#!/usr/bin/env python3
"""
mission_manager.launch.py
=======================

Launch Mission Manager node + GUI for testing.

Usage:
    ros2 launch amr_mission_manager mission_manager.launch.py

Author: AMR System
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_name = 'amr_mission_manager'
    pkg_share = get_package_share_directory(pkg_name)
    pkg_prefix = os.path.dirname(os.path.dirname(pkg_share))

    # Executable paths
    mission_manager_exec = os.path.join(pkg_prefix, 'bin', 'mission_manager')
    mission_gui_exec = os.path.join(pkg_prefix, 'bin', 'mission_gui')

    # Mission Manager Node
    mission_manager_node = ExecuteProcess(
        cmd=[mission_manager_exec],
        output='screen',
        shell=False,
    )

    # Mission GUI
    mission_gui_node = ExecuteProcess(
        cmd=[mission_gui_exec],
        output='screen',
        shell=False,
    )

    return LaunchDescription([
        mission_manager_node,
        mission_gui_node,
    ])
