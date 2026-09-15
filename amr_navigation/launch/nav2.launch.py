"""Launch Nav2 stack for AMR navigation (localization + navigation).

Standalone launch - chạy khi đã có Gazebo + EKF + sensor chạy ở chỗ khác.

Usage:
  ros2 launch amr_navigation nav2.launch.py

Requires (đã chạy ở chỗ khác):
  - /scan (LiDAR)
  - /odom (wheel odometry)
  - /imu/data (IMU)
  - /points (Camera PointCloud)
  - TF: odom -> base_footprint (từ EKF)

Outputs:
  - /map (từ map_server)
  - /amcl_pose (vị trí robot trên map)
  - TF: map -> odom (từ AMCL)
  - cmd_vel_nav -> cmd_vel (velocity commands đến robot)
  - /points_filtered (từ VoxelGrid, ~15Hz)
  - /octomap_point_cloud_centers (từ octomap_server, ~2Hz)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_nav = get_package_share_directory('amr_navigation')
    pkg_mapping = get_package_share_directory('amr_mapping')

    # Arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Paths
    params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    map_file = os.path.join(pkg_mapping, 'maps', 'amr_map.yaml')
    octomap_params = os.path.join(pkg_mapping, 'config', 'octomap_params.yaml')

    # Nav2 bringup launch from nav2_bringup package
    # Bao gồm: map_server, amcl, controller, planner, bt_navigator, etc.
    bringup_dir = get_package_share_directory('nav2_bringup')

    # Import nav2 bringup launch
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.actions import IncludeLaunchDescription

    # map_server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        remappings=[
            ('/map', '/map'),
        ]
    )

    # AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/scan', '/scan'),
            ('/initialpose', '/initialpose'),
        ]
    )

    # Lifecycle manager for localization
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

    # Controller server
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    # Planner server
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    # Behavior server (spin, backup, wait, etc.)
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )

    # Waypoint follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[params_file],
    )

    # Velocity smoother
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

    # Lifecycle manager for navigation
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
    # PointCloud Downsampler (C++ with PCL VoxelGrid)
    # Input: /points (raw from camera, ~57,000 pts)
    # Output: /points_filtered (~2,000-3,000 pts @ 10-15Hz)
    # Filters: Z-range 0.15-1.20m (remove floor/ceiling)
    # ============================================
    voxel_grid_node = Node(
        package='amr_perception',
        executable='pointcloud_downsampler',
        name='pointcloud_downsampler',
        output='screen',
        parameters=[{
            'leaf_size': 0.05,
            'z_min': 0.15,
            'z_max': 1.20,
        }],
    )

    # ============================================
    # OctoMap Server (for 3D visualization & global map)
    # Converts PointCloud2 -> Octree -> /octomap_point_cloud_centers
    # ============================================
    octomap_server = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        output='screen',
        parameters=[octomap_params],
        remappings=[
            ('cloud_in', '/points_filtered'),
            ('octomap_point_cloud_centers', '/octomap_point_cloud_centers'),
        ],
    )

    return LaunchDescription([
        use_sim_time,

        # Localization stack
        map_server_node,
        amcl_node,
        localization_mgr,

        # Navigation stack
        controller_node,
        planner_node,
        behavior_node,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        navigation_mgr,

        # 3D Perception
        voxel_grid_node,
        octomap_server,
    ])
