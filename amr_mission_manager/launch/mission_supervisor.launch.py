#!/usr/bin/env python3
"""
mission_supervisor.launch.py
============================
Chi khoi dong cac node mission; map/route do nav2.launch nap va phat context.
Supervisor, Mission Manager, pallet_mission va rack_tf_publisher nhan graph tu
/navigation/route_yaml_path, khong tu chon file graph. Tham so mission, timeout,
retry... nam trong DEFAULTS cua tung node. Muon doi:
  - sua DEFAULTS trong node roi build lai, hoac
  - doi luc dang chay:  ros2 param set /mission_supervisor <ten> <gia tri>

Khoi dong:
    mission_manager + pallet_mission (qua pallet_mission.launch.py)
    mission_supervisor, rack_tf_publisher, supervisor_gui

Vidu:
    ros2 launch amr_mission_manager mission_supervisor.launch.py
    ros2 launch amr_mission_manager mission_supervisor.launch.py gui:=false
    ros2 launch amr_mission_manager mission_supervisor.launch.py with_pallet_mission:=false
        (khi mission_manager + pallet_mission da chay rieng)

Cac launch argument duoi day chi la CONG BAT/TAT node, khong phai parameter.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument('gui', default_value='true',
                              description='chay supervisor_gui (nut START/STOP)'),
        DeclareLaunchArgument('with_rack_tf', default_value='true',
                              description='chay rack_tf_publisher'),
        DeclareLaunchArgument('with_pallet_mission', default_value='true',
                              description='chay luon mission_manager + pallet_mission'),
    ]

    supervisor = Node(
        package='amr_mission_manager',
        executable='mission_supervisor',
        name='mission_supervisor',
        output='screen',
        emulate_tty=True,
    )

    rack_tf = Node(
        package='amr_mission_manager',
        executable='rack_tf_publisher',
        name='rack_tf_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('with_rack_tf')),
    )

    gui = Node(
        package='amr_mission_manager',
        executable='supervisor_gui',
        name='supervisor_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    pallet_stack = GroupAction(
        scoped=True,
        condition=IfCondition(LaunchConfiguration('with_pallet_mission')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('amr_mission_manager'),
                    'launch', 'pallet_mission.launch.py'])),
                # Chi cong bat/tat node; KHONG truyen parameter (after_pick='hold' do
                # mission_supervisor tu dat len pallet_mission luc bat dau pick).
                launch_arguments={
                    'gui': 'false',                 # khong bat pallet_gui (tranh chen lenh)
                    'with_rack_tf': 'false',        # rack_tf_publisher da chay o day
                    'with_mission_manager': 'true',
                }.items(),
            ),
        ],
    )

    return LaunchDescription(args + [pallet_stack, supervisor, rack_tf, gui])
