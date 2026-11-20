#!/usr/bin/env python3
"""Run the PyTorch .pt detector on CPU for testing."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'inference_device', default_value='cpu',
            description='PyTorch inference device (CPU by default for testing)'),
        Node(
            package='amr_perception',
            executable='pallet_detector',
            name='pallet_detector_node',
            output='screen',
            parameters=[{
                'inference_device': ParameterValue(
                    LaunchConfiguration('inference_device'), value_type=str),
            }],
        ),
    ])
