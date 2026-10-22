#!/usr/bin/env python3
"""
nav_web.launch.py - Navigation Launch cho Web Dashboard
======================================================
Khởi chạy Nav2 Stack động theo Map & Route chọn từ Web SCADA.
Sử dụng tham số phần cứng thật (nav2_params_real.yaml).
KHÔNG khởi chạy RViz.

Usage:
  ros2 launch amr_navigation nav_web.launch.py \
      map:=/home/trieu/project_ws/maps/<map_id>.yaml \
      route:=/home/trieu/project_ws/maps/<map_id>/routes/<route_id>.geojson
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, ExecuteProcess, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_home_pose(route_geojson_path):
    """Đọc node HOME từ file route .yaml nằm cạnh file .geojson (cùng tên, đuôi .yaml).
    Node nào có tên chứa 'HOME' (ưu tiên NODE_HOME). Trả về (x, y, yaw) trong khung map, hoặc None."""
    try:
        import yaml
        if not route_geojson_path:
            return None
        yaml_path = os.path.splitext(route_geojson_path)[0] + '.yaml'
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        raw = data.get('nodes', {})
        if isinstance(raw, dict):
            items = [(str(k), v) for k, v in raw.items() if isinstance(v, dict)]
        elif isinstance(raw, list):
            items = [(str(n.get('name', '')), n) for n in raw if isinstance(n, dict)]
        else:
            items = []
        homes = [(name, n) for name, n in items if 'HOME' in str(n.get('name', name)).upper()]
        if not homes:
            return None
        chosen = next((h for h in homes if h[0].upper() == 'NODE_HOME'), homes[0])
        node = chosen[1]
        pos = node.get('position') or {}
        ori = node.get('orientation') or {}
        return (float(pos.get('x', 0.0)), float(pos.get('y', 0.0)), float(ori.get('yaw', 0.0)))
    except Exception:
        return None


def generate_launch_description():
    pkg_nav = get_package_share_directory('amr_navigation')
    pkg_mapping = get_package_share_directory('amr_mapping')

    # File cấu hình Nav2 chuẩn phần cứng
    params_file = os.path.join(pkg_nav, 'config', 'nav2_params_real.yaml')

    # ============================================
    # Launch Arguments (Nhận đường dẫn động từ Web)
    # ============================================
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/project_ws/maps/map_01.yaml'),
        description='Full path to map YAML file selected from Web'
    )

    route_arg = DeclareLaunchArgument(
        'route',
        default_value=os.path.expanduser('~/project_ws/maps/map_01/routes/map_01 Route.geojson'),
        description='Full path to route GeoJSON file selected from Web'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ============================================
    # AMCL Localization (Nạp nav2_params_real.yaml)
    # ============================================
    def _make_amcl(context):
        amcl_params = [params_file]
        actions = []
        # Initial pose lấy từ node HOME của route đã chọn (thay cho (0,0,0) cố định trong params).
        # Không có route hoặc route không có node HOME thì giữ nguyên initial_pose trong params.
        home = _load_home_pose(context.perform_substitution(LaunchConfiguration('route')))
        if home is not None:
            hx, hy, hyaw = home
            amcl_params.append({   # đặt sau params_file nên ghi đè
                'set_initial_pose': True,
                'initial_pose.x': hx,
                'initial_pose.y': hy,
                'initial_pose.z': 0.0,
                'initial_pose.yaw': hyaw,
            })
            actions.append(LogInfo(msg=f'[nav_web] AMCL initial pose theo HOME: '
                                       f'x={hx:.3f} y={hy:.3f} yaw={hyaw:.3f}'))
        else:
            actions.append(LogInfo(msg='[nav_web] Không có node HOME trong route: '
                                       'dùng initial_pose trong nav2_params_real.yaml'))
        actions.append(Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=amcl_params,
            remappings=[
                ('/scan', '/scan_filtered'),
                ('/initialpose', '/initialpose'),
            ]
        ))
        return actions

    amcl_node = OpaqueFunction(function=_make_amcl)

    # ============================================
    # Lifecycle Manager - Localization
    # ============================================
    localization_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
    # Behavior Server
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
    # Lifecycle Manager - Navigation
    # ============================================
    navigation_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'graph_filepath': LaunchConfiguration('route'),
            'route_frame': 'map',
            'global_frame': 'map',
            'base_frame': 'base_link',
        }],
    )

    # ============================================
    # Route Server Lifecycle
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
            'graph_yaml_path': LaunchConfiguration('route'),
        }],
    )

    # ============================================
    # Return Launch Description
    # ============================================
    return LaunchDescription([
        # Arguments
        map_arg,
        route_arg,
        use_sim_time_arg,

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
        route_server_configure,
        route_server_activate,

        # Route Graph Publisher
        route_graph_publisher,
    ])