"""SLAM Bringup - Launch EKF + SLAM + Controllers + RViz (no teleop)

Usage:
  ros2 launch amr_mapping slam_bringup.launch.py           # Gazebo headless (no GUI)
  ros2 launch amr_mapping slam_bringup.launch.py gui:=true # Gazebo GUI on
  
  Then run in separate terminal:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, 
    TimerAction, SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch Gazebo GUI (true/false)'
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.0925')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    gui = LaunchConfiguration('gui')

    # Package paths
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_amr_desc = get_package_share_directory('amr_description')
    pkg_amr_local = get_package_share_directory('amr_localization')
    pkg_amr_mapping = get_package_share_directory('amr_mapping')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Paths
    world_file = os.path.join(pkg_amr_gazebo, 'worlds', 'warehouse.world')
    xacro_file = os.path.join(pkg_amr_desc, 'urdf', 'amr.gazebo.xacro')
    ekf_config = os.path.join(pkg_amr_local, 'config', 'ekf.yaml')
    slam_config = os.path.join(pkg_amr_mapping, 'config', 'mapper_params_online_async.yaml')
    rviz_config = os.path.join(pkg_amr_mapping, 'rviz', 'slam.rviz')
    controller_config = os.path.join(pkg_amr_desc, 'config', 'lift_controller.yaml')

    # Xacro processing
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()

    # Environment variables
    env_vars = [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.dirname(pkg_amr_desc)),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', '/opt/ros/humble/lib'),
    ]

    # ============================================
    # GAZEBO (always spawn robot)
    # ============================================
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
        }.items(),
        condition=IfCondition(gui),
    )

    gazebo_headless = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen',
        condition=UnlessCondition(gui),
    )

    # ============================================
    # Robot State Publisher
    # ============================================
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description_config
        }],
    )

    # ============================================
    # Spawn Robot (always - no condition)
    # ============================================
    spawn_entity = Node(
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

    # ============================================
    # Controllers (delayed start - wait longer for ros2_control to load)
    # ============================================
    controller_spawner = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    '--param-file', controller_config,
                    'joint_state_broadcaster',
                    'lift_controller', 
                    '--controller-manager', '/controller_manager'
                ],
                output='screen',
            ),
        ],
    )

    # ============================================
    # EKF Odometry
    # ============================================
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            ekf_config
        ],
        remappings=[
            ('/odom', '/odom'),
            #('/imu/data', '/imu/data'),
        ],
    )

    # ============================================
    # SLAM Toolbox
    # ============================================
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            slam_config
        ],
    )

    # ============================================
    # cmd_vel_splitter
    # ============================================
    cmd_vel_splitter = Node(
        package='amr_manipulation',
        executable='cmd_vel_splitter',
        name='cmd_vel_splitter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
    )

    # ============================================
    # RViz
    # ============================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # ============================================
    # Return LaunchDescription
    # ============================================
    return LaunchDescription([
        gui_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,

        # Environment
        *env_vars,

        # Gazebo (GUI or headless)
        gazebo_gui,
        gazebo_headless,

        # Robot state publisher
        rsp,

        # Spawn entity (always)
        spawn_entity,

        # Delayed: Controllers
        controller_spawner,

        # EKF
        ekf_node,

        # SLAM
        slam_node,

        # cmd_vel_splitter
        cmd_vel_splitter,

        # RViz
        rviz,
    ])