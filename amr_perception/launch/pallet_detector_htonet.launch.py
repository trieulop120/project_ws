import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('amr_perception')
    default_model_path = os.path.join(
        pkg_dir, 'weights', 'htonet_v5_openvino', 'htonet_v5_pallet_ov.xml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model_path,
            description='Path to HTONet-v5 OpenVINO IR (.xml) model'
        ),
        DeclareLaunchArgument(
            'ov_device',
            default_value='GPU',
            description='OpenVINO target device: GPU (Intel Iris Xe) or CPU'
        ),
        DeclareLaunchArgument(
            'conf_threshold', default_value='0.25',
            description='Base confidence threshold'
        ),
        DeclareLaunchArgument(
            'iou_threshold', default_value='0.45',
            description='Kept for compatibility with detector settings'
        ),
        DeclareLaunchArgument(
            'pallet_conf',
            default_value='0.50',
            description='Confidence threshold for pallet class'
        ),
        DeclareLaunchArgument(
            'box_conf',
            default_value='0.50',
            description='Confidence threshold for box class'
        ),
        DeclareLaunchArgument(
            'hole_conf',
            default_value='0.40',
            description='Confidence threshold for hole class'
        ),
        DeclareLaunchArgument(
            'debug_image',
            default_value='true',
            description='Publish annotated debug image'
        ),
        DeclareLaunchArgument(
            'use_roi', default_value='true',
            description='Use the configured inference ROI'
        ),
        DeclareLaunchArgument(
            'timer_period', default_value='0.1',
            description='Detector processing period in seconds'
        ),
        DeclareLaunchArgument(
            'mt_executor', default_value='false',
            description='Use a multi-threaded executor'
        ),

        Node(
            package='amr_perception',
            executable='pallet_detector_htonet',
            name='pallet_detector_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'ov_device': LaunchConfiguration('ov_device'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'iou_threshold': LaunchConfiguration('iou_threshold'),
                'pallet_conf': LaunchConfiguration('pallet_conf'),
                'box_conf': LaunchConfiguration('box_conf'),
                'hole_conf': LaunchConfiguration('hole_conf'),
                'debug_image': LaunchConfiguration('debug_image'),
                'use_roi': LaunchConfiguration('use_roi'),
                'timer_period': LaunchConfiguration('timer_period'),
                'mt_executor': LaunchConfiguration('mt_executor'),
            }]
        )
    ])
