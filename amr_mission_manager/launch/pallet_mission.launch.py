#!/usr/bin/env python3
"""
pallet_mission.launch.py
========================
Chay pallet_mission_node (va GUI neu gui:=true) trong package amr_mission_manager.

Dung chung voi bringup + Nav2 dang chay:
    ros2 launch amr_mission_manager pallet_mission.launch.py

Launch nay chay luon mission_manager (cho nut HOME -> P_1) va GUI pallet.
KHONG chay kem mission_manager.launch.py (se bi trung node).
Neu da chay mission_manager rieng: with_mission_manager:=false

Vi du doi tham so:
    ros2 launch amr_mission_manager pallet_mission.launch.py standoff_m:=0.55
    ros2 launch amr_mission_manager pallet_mission.launch.py auto_insert:=false

Luu y: pallet_mission_node tu bat detector (ros2 run amr_perception pallet_detector)
bang tien trinh con khi bam Detect, nen KHONG chay detector rieng o day.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _arg(name, default, desc):
    return DeclareLaunchArgument(name, default_value=default, description=desc)


def generate_launch_description():
    # Tham so hay phai chinh khi test tren xe that
    args = [
        _arg('use_sim_time', 'false', 'true neu chay mo phong'),
        _arg('gui', 'true', 'bat GUI pallet_gui'),
        _arg('with_mission_manager', 'true',
             'true: chay luon mission_manager (nut HOME -> P_1). '
             'false neu mission_manager da chay rieng'),
        _arg('graph_package', 'amr_navigation', 'package chua route graph'),
        _arg('graph_yaml_filename', 'D203B Route.yaml',
             'ten file YAML trong <package>/config/graphs/'),
        _arg('standoff_m', '0.40',
             'khoang cach tu base_footprint toi diem dem (m). '
             '= 0.40 + khoang cach tu truc banh toi mui cang'),
        _arg('insert_speed', '0.10', 'van toc tien vao pallet (m/s)'),
        _arg('stop_dist_cm', '3.5', 'nguong sieu am de dung (cm)'),
        _arg('confirm_count', '10', 'so mau sieu am lien tiep duoi nguong'),
        _arg('auto_insert', 'true', 'true: toi diem dem xong tu canh goc + tien vao'),
        _arg('require_stable', 'true', 'true: Detect OFF chi nhan ket qua on dinh'),
        _arg('final_window_s', '5.0',
             'cua so (s) lay mau cuoi de chot ket qua'),
        _arg('min_samples', '1',
             'so mau toi thieu (tong va trong cua so). Detector CPU ~2 mau/s'),
        _arg('range_topic', '/ultrasonic/range', 'topic sieu am (sensor_msgs/Range)'),
    ]

    def pv(name, typ):
        return ParameterValue(LaunchConfiguration(name), value_type=typ)

    mission_node = Node(
        package='amr_mission_manager',
        # Trung voi RENAME pallet_mission_node trong CMakeLists.txt
        executable='pallet_mission_node',
        name='pallet_mission',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': pv('use_sim_time', bool),
            'standoff_m': pv('standoff_m', float),
            'insert_speed': pv('insert_speed', float),
            'stop_dist_cm': pv('stop_dist_cm', float),
            'confirm_count': pv('confirm_count', int),
            'auto_insert': pv('auto_insert', bool),
            'require_stable': pv('require_stable', bool),
            'final_window_s': pv('final_window_s', float),
            'min_samples': pv('min_samples', int),
            'range_topic': LaunchConfiguration('range_topic'),
        }],
    )

    mission_manager_node = Node(
        package='amr_mission_manager',
        executable='mission_manager',
        name='mission_manager',
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_mission_manager')),
        parameters=[{
            'graph_package': LaunchConfiguration('graph_package'),
            'graph_yaml_filename': LaunchConfiguration('graph_yaml_filename'),
        }],
    )

    gui_node = Node(
        package='amr_mission_manager',
        executable='pallet_gui',
        name='pallet_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
        parameters=[{'use_sim_time': pv('use_sim_time', bool)}],
    )

    return LaunchDescription(args + [mission_manager_node, mission_node, gui_node])