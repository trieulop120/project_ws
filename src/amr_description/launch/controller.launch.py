"""Launch controller manager and load lift controller"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_desc = get_package_share_directory('amr_description')

    # Controller config
    controller_config = os.path.join(pkg_amr_desc, 'config', 'lift_controller.yaml')

    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        output='screen',
        shell=True,
    )

    return LaunchDescription([
        controller_manager,
    ])
