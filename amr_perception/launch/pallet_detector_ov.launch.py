#!/usr/bin/env python3
"""Run the OpenVINO detector on Intel GPU; run only one pallet detector."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'inference_device', default_value='intel:gpu',
            description='OpenVINO device: intel:gpu (Intel GPU) or intel:cpu'),
        Node(
            package='amr_perception',
            executable='pallet_detector_ov',
            name='pallet_detector_node',
            output='screen',
            additional_env={
                'OMP_NUM_THREADS': '4',
            },
            parameters=[{
                'inference_device': ParameterValue(
                    LaunchConfiguration('inference_device'), value_type=str),
                'mt_executor': True,
                'timer_period': 0.066,
                'tf_timeout': 0.05,
            }],
        ),
    ])
