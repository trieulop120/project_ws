"""Nav2 Bringup - Launch Gazebo + Nav2 + EKF + Controllers + RViz

Includes:
  - Gazebo simulation with robot
  - EKF odometry (wheel odometry + IMU)
  - Controller manager (joint_state_broadcaster, lift_controller)
  - Nav2 with AMCL localization + MPPI controller
  - PointCloud obstacle detection from depth camera

Usage:
  ros2 launch amr_navigation nav2_bringup.launch.py           # Headless
  ros2 launch amr_navigation nav2_bringup.launch.py gui:=true # GUI
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
    # ============================================
    # Launch Arguments
    # ============================================
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

    # ============================================
    # Package paths
    # ============================================
    pkg_amr_gazebo = get_package_share_directory('amr_gazebo')
    pkg_amr_desc = get_package_share_directory('amr_description')
    pkg_amr_local = get_package_share_directory('amr_localization')
    pkg_amr_mapping = get_package_share_directory('amr_mapping')
    pkg_amr_nav = get_package_share_directory('amr_navigation')

    # ============================================
    # Paths
    # ============================================
    world_file = os.path.join(pkg_amr_gazebo, 'worlds', 'warehouse.world')
    xacro_file = os.path.join(pkg_amr_desc, 'urdf', 'amr.gazebo.xacro')
    ekf_config = os.path.join(pkg_amr_local, 'config', 'ekf.yaml')
    nav2_params = os.path.join(pkg_amr_nav, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_amr_mapping, 'maps', 'amr_map.yaml')
    rviz_config = os.path.join(pkg_amr_nav, 'rviz', 'navigation.rviz')
    controller_config = os.path.join(pkg_amr_desc, 'config', 'lift_controller.yaml')

    # Xacro processing
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()

    # ============================================
    # Environment variables
    # ============================================
    env_vars = [
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.path.dirname(pkg_amr_desc)),
        SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', '/opt/ros/humble/lib'),
    ]

    # ============================================
    # Gazebo
    # ============================================
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
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
    # Spawn Robot
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
    # Controllers (delayed start - wait for ros2_control)
    # ============================================
    controller_spawner = TimerAction(
        period=5.0,
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
    # EKF Odometry (for localization)
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
            ('/odometry/filtered', '/odometry/filtered'),
        ],
    )

    # ============================================
    # cmd_vel_splitter (for lift mechanism)
    # ============================================
    cmd_vel_splitter = Node(
        package='amr_manipulation',
        executable='cmd_vel_splitter',
        name='cmd_vel_splitter',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ============================================
    # Nav2 Bringup (official)
    # Uses map for localization
    # ============================================
    nav2_bringup_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'bringup_launch.py'
    )

    # Delay Nav2 launch to ensure:
    # 1. Gazebo simulation is stable
    # 2. map_server is ready
    nav2_bringup_delayed = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_path),
                launch_arguments={
                    'map': map_file,
                    'use_sim_time': 'true',
                    'params_file': nav2_params,
                    'autostart': 'true',
                    'initial_pose_x': '0.0',
                    'initial_pose_y': '0.0',
                    'initial_pose_a': '0.0',
                }.items()
            )
        ]
    )

    # ============================================
    # RViz with Nav2 panel
    # ============================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(gui),
    )

    # ============================================
    # Return LaunchDescription
    # ============================================
    return LaunchDescription([
        # Arguments
        gui_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,

        # Environment
        *env_vars,

        # Gazebo
        gazebo_gui,
        gazebo_headless,

        # Robot state publisher
        rsp,

        # Spawn entity
        spawn_entity,

        # Delayed: Controllers
        controller_spawner,

        # EKF
        ekf_node,

        # Nav2 (includes map_server, amcl, controller, planner, etc.)
        nav2_bringup_delayed,

        # cmd_vel_splitter
        cmd_vel_splitter,

        # RViz
        rviz,
    ])
