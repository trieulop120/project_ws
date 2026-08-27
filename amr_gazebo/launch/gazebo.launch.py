"""Launch Gazebo with AMR robot"""
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_amr_desc = get_package_share_directory('amr_description')

    # Paths
    world_file = os.path.join(pkg_amr_gazebo, 'worlds', 'warehouse.world')
    xacro_file = os.path.join(pkg_amr_desc, 'urdf', 'amr.gazebo.xacro')
    meshes_dir = os.path.join(pkg_amr_desc, 'meshes')
    urdf_out = '/tmp/amr_robot_gazebo.urdf'

    # Generate URDF from xacro
    robot_description = ""
    try:
        result = subprocess.run(
            ['xacro', xacro_file],
            capture_output=True, text=True, timeout=30
        )
        if result.returncode == 0:
            urdf_content = result.stdout
            # Replace package:// URLs with absolute paths for Gazebo
            urdf_content = urdf_content.replace(
                'package://amr_description/meshes/',
                meshes_dir + '/'
            )
            with open(urdf_out, 'w') as f:
                f.write(urdf_content)
            robot_description = urdf_content
            print(f"[gazebo.launch] URDF generated: {urdf_out}")
        else:
            print(f"[gazebo.launch] xacro error:\n{result.stderr}")
    except Exception as e:
        print(f"[gazebo.launch] xacro failed: {e}")

    # Launch arguments
    declare_x = DeclareLaunchArgument('x', default_value='0.0')
    declare_y = DeclareLaunchArgument('y', default_value='0.0')
    declare_z = DeclareLaunchArgument('z', default_value='0.0955')
    declare_yaw = DeclareLaunchArgument('yaw', default_value='0.0')

    # Robot State Publisher
    rsp_params = {
        'use_sim_time': True,
        'robot_description': robot_description,
    }
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[rsp_params]
    )

    # Gazebo server
    gazebo_server = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_init.so', world_file],
        output='screen'
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'amr_sim',
            '-file', urdf_out,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_x, declare_y, declare_z, declare_yaw,

        # 1. Start Gazebo server
        gazebo_server,

        # 2. Start robot_state_publisher + spawn robot
        TimerAction(period=2.0, actions=[rsp, spawn_robot]),
    ])
