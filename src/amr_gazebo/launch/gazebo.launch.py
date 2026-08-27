"""Launch Gazebo Classic with AMR robot"""
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_amr_desc = get_package_share_directory('amr_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Environment variables setup for meshes and plugins
    gz_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.dirname(pkg_amr_desc)
    )
    gz_plugin_path = AppendEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value='/opt/ros/humble/lib'
    )

    # File paths
    world_file = os.path.join(pkg_amr_gazebo, 'worlds', 'warehouse.world')
    xacro_file = os.path.join(pkg_amr_desc, 'urdf', 'amr.gazebo.xacro')

    # Process Xacro
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()

    # Launch arguments
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_z = DeclareLaunchArgument('z', default_value='0.0955')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_config
        }]
    )

    # Gazebo System - Tắt tính năng tự inject params bằng extra_gazebo_args
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
            'extra_gazebo_args': '--ros-args --disable-rosout'
        }.items()
    )

    # Spawn Entity chỉ đọc từ Topic robot_description
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'amr',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_x,
        declare_y,
        declare_z,
        declare_yaw,
        gz_model_path,
        gz_plugin_path,
        gazebo,
        rsp,
        TimerAction(period=3.0, actions=[spawn_robot]),
    ])