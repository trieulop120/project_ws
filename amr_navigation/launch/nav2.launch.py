"""Nav2 Launch for Real Hardware.

Usage:
  ros2 launch amr_navigation nav2.launch.py
  ros2 launch amr_navigation nav2.launch.py map:=/maps/site.yaml route:=/maps/site/routes/graph.geojson

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
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('amr_navigation')
    pkg_mapping = get_package_share_directory('amr_mapping')

    # Nav2 launch owns the selected map and route files. Keep defaults for the
    # current D203B setup, but allow the operator/parent launch to select them.
    params_file = os.path.join(pkg_nav, 'config', 'nav2_params_real.yaml')
    map_file = os.path.join(pkg_mapping, 'maps', 'D203B.yaml')
    graph_file = os.path.join(pkg_nav, 'config', 'graphs', 'D203B Route.geojson')
    rviz_config = os.path.join(pkg_nav, 'rviz', 'navigation_real.rviz')
    map_arg = DeclareLaunchArgument(
        'map', default_value=map_file,
        description='Map YAML loaded by Nav2 map_server')
    route_arg = DeclareLaunchArgument(
        'route', default_value=graph_file,
        description='Route GeoJSON loaded by nav2_route route_server; matching YAML must be beside it')

    # ============================================
    # Map Server
    # ============================================
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': LaunchConfiguration('map'),
            'use_sim_time': False,
        }],
    )

    # ============================================
    # AMCL Localization
    # ============================================
    def _make_amcl(context):
        # AMCL's initial pose is a localization seed, not a navigation goal.
        # Read it from the YAML paired with the selected Route GeoJSON.
        route_path = context.perform_substitution(LaunchConfiguration('route'))
        route_yaml = os.path.splitext(route_path)[0] + '.yaml'
        amcl_parameters = [params_file]
        home_pose = None
        try:
            import yaml
            with open(route_yaml, 'r', encoding='utf-8') as graph_stream:
                graph_data = yaml.safe_load(graph_stream) or {}
            nodes = graph_data.get('nodes', {})
            if isinstance(nodes, dict):
                home_node = nodes.get('NODE_HOME')
                if home_node is None:
                    home_node = next(
                        (node for name, node in nodes.items()
                         if 'HOME' in str(name).upper() or
                         'HOME' in str(node.get('name', '')).upper()),
                        None,
                    )
                if isinstance(home_node, dict):
                    position = home_node.get('position') or {}
                    orientation = home_node.get('orientation') or {}
                    home_pose = (float(position.get('x', 0.0)),
                                 float(position.get('y', 0.0)),
                                 float(orientation.get('yaw', 0.0)))
        except (OSError, ValueError, TypeError, ImportError) as exc:
            return [LogInfo(msg=f'[nav2] Could not read HOME from {route_yaml}: {exc}'),
                    Node(package='nav2_amcl', executable='amcl', name='amcl',
                         output='screen', parameters=amcl_parameters,
                         remappings=[('/scan', '/scan_filtered'),
                                     ('/initialpose', '/initialpose')])]

        if home_pose is not None:
            home_x, home_y, home_yaw = home_pose
            amcl_parameters.append({
                'set_initial_pose': True,
                'initial_pose.x': home_x,
                'initial_pose.y': home_y,
                'initial_pose.z': 0.0,
                'initial_pose.yaw': home_yaw,
            })
            info = (f'[nav2] AMCL initial pose theo HOME trong {route_yaml}: '
                    f'x={home_x:.3f} y={home_y:.3f} yaw={home_yaw:.3f}')
        else:
            info = f'[nav2] Không tìm thấy HOME trong {route_yaml}; dùng initial_pose mặc định'

        return [LogInfo(msg=info), Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=amcl_parameters,
            remappings=[('/scan', '/scan_filtered'),
                        ('/initialpose', '/initialpose')])]

    amcl_node = OpaqueFunction(function=_make_amcl)

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
                'route_server',
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
            'graph_filepath': LaunchConfiguration('route'),
            'route_frame': 'map',
            'global_frame': 'map',
            'base_frame': 'base_link',
        }],
    )

    # ============================================
    # Route Graph Publisher
    # ============================================
    # LUON launch voi duong dan YAML suy ra tu route da chon (file .yaml
    # nam canh .geojson cung ten). Neu thieu file: publisher (fail-fast)
    # se log loi ro va thoat, khong im lang bo qua nhu cu.
    def _make_route_graph_publisher(context):
        route_path = context.perform_substitution(LaunchConfiguration('route'))
        route_yaml = os.path.splitext(route_path)[0] + '.yaml'
        return [Node(
            package='amr_navigation', executable='route_graph_publisher',
            name='route_graph_publisher', output='screen',
            parameters=[{'graph_yaml_path': route_yaml}]),
        ]

    route_graph_publisher = OpaqueFunction(function=_make_route_graph_publisher)

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
        map_arg,
        route_arg,
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

        # Route Graph Publisher
        route_graph_publisher,

        # Visualization
        rviz_node,
    ])
