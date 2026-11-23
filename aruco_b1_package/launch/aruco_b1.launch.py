from typing import List
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('marker_x', default_value='0.0'),
        DeclareLaunchArgument('marker_y', default_value='0.0'),
        DeclareLaunchArgument('marker_z', default_value='0.115'),
        Node(
            package='aruco_b1_package',
            executable='aruco_b1_localizer',
            name='aruco_b1_localizer',
            output='screen',
            parameters=[{
                'marker_id': 0,
                'dictionary': 'DICT_4X4_50',
                'marker_size_m': 0.15,
                'marker_xyz_slot_m': ParameterValue(
                    PythonExpression([
                        '[', LaunchConfiguration('marker_x'), ', ',
                        LaunchConfiguration('marker_y'), ', ',
                        LaunchConfiguration('marker_z'), ']']),
                    value_type=List[float]),
                'slot_frame': 'slot_B1',
                'map_frame': 'map',
                'base_frame': 'base_footprint',
                'search_hz': 1.5,
                'track_hz': 8.0,
                'auto_update_amcl': False,
            }],
        )
    ])
