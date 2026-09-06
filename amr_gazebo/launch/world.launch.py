"""Launch Gazebo world for map editing"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_gazebo')
    world_file = os.path.join(pkg_share, 'worlds', 'warehouse.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_init.so', world_file],
            output='screen'
        ),
    ])
