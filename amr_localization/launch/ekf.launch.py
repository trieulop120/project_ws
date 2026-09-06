"""Launch EKF odometry for AMR localization.

Fuses wheel odometry (/odom) with IMU gyro_z (/imu/data).
Outputs fused odometry to /localization/odom and TF odom -> base_link.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_localization = get_package_share_directory('amr_localization')
    config_file = os.path.join(pkg_localization, 'config', 'ekf.yaml')

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/odom', '/odom'),          # From differential drive plugin
            #('/imu/data', '/imu/data'),  # From IMU sensor plugin
        ]
    )

    return LaunchDescription([
        ekf_node,
    ])
