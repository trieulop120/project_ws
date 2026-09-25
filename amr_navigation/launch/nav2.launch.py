"""Nav2 Launch for Real Hardware.

Usage:
  ros2 launch amr_navigation nav2.launch.py

Requires (đã chạy từ real_bringup.launch.py):
  - /scan_filtered (LiDAR đã lọc)
  - /odometry/filtered (EKF odometry)
  - /imu/data_raw (IMU)
  - /points (Camera PointCloud)
  - /points_filtered (downsampled PointCloud)
  - /map (từ SLAM)

Outputs:
  - /amcl_pose (vị trí robot trên map)
  - TF: map -> odom (từ AMCL)
  - /cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel (robot)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('amr_navigation')
    pkg_mapping = get_package_share_directory('amr_mapping')

    # ============================================
    # PATHS - Chỉnh sửa ở đây
    # ============================================
    params_file = os.path.join(pkg_nav, 'config', 'nav2_params_real.yaml')
    map_file = os.path.join(pkg_mapping, 'maps', 'real_amr_map.yaml')
    graph_file = os.path.join(pkg_nav, 'config', 'graphs', 'real_route_graph.geojson')
    rviz_config = os.path.join(pkg_nav, 'rviz', 'navigation.rviz')

    # ============================================
    # Map Server
    # ============================================
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': False,
        }],
    )

    # ============================================
    # AMCL Localization
    # ============================================
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/scan', '/scan_filtered'),
            ('/initialpose', '/initialpose'),
        ]
    )

    # ============================================
    # Lifecycle Manager — Localization
    # ============================================
    localization_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }]
    )

    # ============================================
    # Controller Server (MPPI)
    # ============================================
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
        ]
    )

    # ============================================
    # Planner Server (A* SmacPlanner2D)
    # ============================================
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
    )

    # ============================================
    # Behavior Server (spin, backup, wait)
    # ============================================
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
    )

    # ============================================
    # BT Navigator
    # ============================================
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
    )

    # ============================================
    # Waypoint Follower
    # ============================================
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
    )

    # ============================================
    # Velocity Smoother
    # ============================================
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel'),
        ]
    )

    # ============================================
    # Lifecycle Manager — Navigation
    # ============================================
    navigation_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'velocity_smoother',
            ],
        }]
    )

    # ============================================
    # Route Server (nav2_route)
    # ============================================
    route_server = Node(
        package='nav2_route',
        executable='route_server',
        name='route_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'graph_filepath': graph_file,
            'route_frame': 'map',
            'global_frame': 'map',
            'base_frame': 'base_link',
        }],
    )

    # ============================================
    # Route Server Lifecycle - Auto Configure + Activate
    # Lifecycle: unconfigured[1] -> configure -> inactive[2] -> activate -> active[3]
    # ============================================
    route_server_configure = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/route_server', 'configure'],
                output='screen',
            ),
        ],
    )

    route_server_activate = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/route_server', 'activate'],
                output='screen',
            ),
        ],
    )

    # ============================================
    # Route Graph Publisher
    # ============================================
    route_graph_publisher = Node(
        package='amr_navigation',
        executable='route_graph_publisher',
        name='route_graph_publisher',
        output='screen',
        parameters=[{
            'graph_yaml_path': graph_file.replace('.geojson', '.yaml'),
        }],
    )

    # ============================================
    # RViz (Navigation)
    # ============================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # ============================================
    # Return LaunchDescription
    # ============================================
    return LaunchDescription([
        # Localization
        map_server_node,
        amcl_node,
        localization_mgr,

        # Navigation
        controller_node,
        planner_node,
        behavior_node,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        navigation_mgr,

        # Route Server
        route_server,

        # Route Server Lifecycle
        route_server_configure,
        route_server_activate,

        # Route Graph Publisher
        route_graph_publisher,

        # Visualization
        rviz_node,
    ])
