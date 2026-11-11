#!/usr/bin/env python3
"""
web_bridge.launch.py

Launch the AMR Web Bridge HTTP server, the headless mission supervisor,
and optionally rosbridge. Nav2 is started from the web Navigation page after
the operator selects a map and route.

Usage:
    ros2 launch amr_web web_bridge.launch.py

    # Or for full functionality with rosbridge:
    ros2 launch amr_web web_bridge.launch.py launch_rosbridge:=true

    # If mission_supervisor is already running elsewhere:
    ros2 launch amr_web web_bridge.launch.py launch_mission_supervisor:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # HTTP server arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='HTTP server host'
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8000',
        description='HTTP server port'
    )
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='true',
        description='Enable verbose logging'
    )

    # Rosbridge arguments
    rosbridge_arg = DeclareLaunchArgument(
        'launch_rosbridge',
        default_value='false',
        description='Launch rosbridge server'
    )
    rosbridge_port_arg = DeclareLaunchArgument(
        'rosbridge_port',
        default_value='9090',
        description='Rosbridge WebSocket port'
    )

    mission_supervisor_arg = DeclareLaunchArgument(
        'launch_mission_supervisor', default_value='true',
        description='Start the mission supervisor and pallet workflow with the web dashboard'
    )

    # HTTP server node (non-ROS executable)
    web_bridge_node = Node(
        package='amr_web',
        executable='web_bridge',
        name='web_bridge',
        output='screen',
        arguments=[
            PythonExpression(["'--host=' + '", LaunchConfiguration('host'), "'"]),
            PythonExpression(["'--port=' + '", LaunchConfiguration('port'), "'"]),
            PythonExpression(["'--verbose=' + '", LaunchConfiguration('verbose'), "'"]),
        ],
    )

    # Rosbridge server node
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('rosbridge_port'),
        }],
        condition=IfCondition(LaunchConfiguration('launch_rosbridge')),
    )

    mission_supervisor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('amr_mission_manager'),
            'launch', 'mission_supervisor.launch.py',
        ])),
        launch_arguments={'gui': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('launch_mission_supervisor')),
    )

    return LaunchDescription([
        host_arg,
        port_arg,
        verbose_arg,
        rosbridge_arg,
        rosbridge_port_arg,
        mission_supervisor_arg,
        web_bridge_node,
        rosbridge_node,
        mission_supervisor,
    ])

