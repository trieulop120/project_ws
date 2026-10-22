#!/usr/bin/env python3
"""
Real Hardware Bringup Launch File
File location: amr_hardware/launch/real_bringup.launch.py
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ============================================
    # Launch Arguments
    # ============================================
    esp32_port_arg = DeclareLaunchArgument(
        'esp32_port',
        default_value='/dev/esp32',
        description='ESP32 serial port'
    )
    esp32_port = LaunchConfiguration('esp32_port')

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/rplidar',
        description='LiDAR serial port'
    )
    lidar_port = LaunchConfiguration('lidar_port')

    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='LiDAR serial baudrate'
    )
    serial_baudrate = LaunchConfiguration('serial_baudrate')

    scan_mode_arg = DeclareLaunchArgument(
        'scan_mode',
        default_value='',
        description='LiDAR scan mode'
    )
    scan_mode = LaunchConfiguration('scan_mode')

    camera_enable_arg = DeclareLaunchArgument(
        'camera_enable',
        default_value='true',
        description='Enable camera driver (true/false)'
    )
    camera_enable = LaunchConfiguration('camera_enable')

    use_rf2o_arg = DeclareLaunchArgument(
        'use_rf2o',
        default_value='false',
        description='Fuse the rf2o laser odometry (vx only) into the EKF '
                    'as a secondary odometry source (true/false)'
    )
    use_rf2o = LaunchConfiguration('use_rf2o')

    rf2o_delay_arg = DeclareLaunchArgument(
        'rf2o_startup_delay',
        default_value='5.0',
        description='Delay [s] before starting rf2o, so that the TF '
                    'base_footprint -> lidar_link is available'
    )
    rf2o_delay = LaunchConfiguration('rf2o_startup_delay')

    # ============================================
    # Package paths & Configs
    # ============================================
    pkg_amr_desc = get_package_share_directory('amr_description')
    pkg_amr_localization = get_package_share_directory('amr_localization')

    xacro_file = os.path.join(pkg_amr_desc, 'urdf', 'amr.gazebo.xacro')
    ekf_config = os.path.join(pkg_amr_localization, 'config', 'ekf_real_robot.yaml')
    rf2o_config = os.path.join(pkg_amr_localization, 'config', 'rf2o_params.yaml')

    # Process xacro
    with open(xacro_file) as f:
        doc = xacro.parse(f)
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()

    # ============================================
    # Info Logs
    # ============================================
    bringup_info = LogInfo(msg='[real_bringup] Starting REAL HARDWARE bringup')
    lidar_info = LogInfo(msg='[real_bringup] LiDAR: sllidar_ros2')
    ekf_info = LogInfo(msg='[real_bringup] Localization: EKF -> /odom')

    # ============================================
    # Robot State Publisher
    # ============================================
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description_config
        }],
    )

    # ============================================
    # Static Transform Publisher
    # ============================================
    static_transforms = Node(
        package='amr_hardware',
        executable='static_transforms',
        name='amr_static_transforms',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ============================================
    # ESP32 Hardware Bridge
    # ============================================
    esp32_bridge = Node(
        package='amr_hardware',
        executable='esp32_bridge',
        name='esp32_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'port': esp32_port,
            'baudrate': 115200,
            'wheel_radius': 0.05,
            'wheelbase': 0.477,
            'publish_tf': False,
        }],
    )

    # ============================================
    # CMD Velocity Splitter
    # ============================================
    cmd_vel_splitter = Node(
        package='amr_hardware',
        executable='cmd_vel_splitter',
        name='cmd_vel_splitter',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
    )

    # ============================================
    # LiDAR Driver (Sllidar A1M8)
    # ============================================
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'serial_port': lidar_port,
            'serial_baudrate': serial_baudrate,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': scan_mode,
        }],
        remappings=[
            ('/scan', '/scan'),
        ],
    )

    # ============================================
    # LiDAR Scan Filter Node
    # ============================================
    scan_filter_node = Node(
        package='amr_perception',
        executable='scan_filter_node',
        name='scan_filter_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    # ============================================
    # Camera Driver
    # ============================================
    camera_driver = Node(
        package='astra_camera',
        executable='astra_camera_node',
        name='camera_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'color_width': 640,
            'color_height': 480,
            'depth_width': 640,
            'depth_height': 480,
            # [FIXED]: Explicitly enable depth & color streams
            'enable_depth': True,
            'enable_color': True,
            # [CHANGED]: Try UVC mode for color stream
            'use_uvc_camera': True,
            # [FIXED]: Disable hardware registration to fix Device.setProperty(5) error
            'enable_depth_to_color_registration': False,
            'depth_registration': False,
            'color_depth_synchronization': True,
        }],
        remappings=[
            # Remap PointCloud2 output from Astra to /points
            ('/depth/points', '/points'),
            ('/camera/depth/image_raw', '/depth/image_raw'),
            ('/camera/color/image_raw', '/color/image_raw'),
        ],
        condition=IfCondition(camera_enable)
    )

    pointcloud_filter_node = Node(
        package='amr_perception',
        executable='pointcloud_downsampler',
        name='pointcloud_filter',
        output='screen',
        parameters=[{
            'leaf_size': 0.05,
            'z_min': 0.05,
            'z_max': 1.20,
        }],
        condition=IfCondition(camera_enable)
    )

    # ============================================
    # RF2O Laser Odometry (optional, vx source)
    # ============================================
    # Delayed so that the TF base_footprint -> base_link -> lidar_link is
    # already published by robot_state_publisher when rf2o performs its first
    # lookupTransform. Without the delay rf2o logs
    # "lookupTransform base_footprint -> lidar_link" failures and never
    # initialises.
    rf2o_node = TimerAction(
        period=rf2o_delay,
        actions=[
            Node(
                package='rf2o_laser_odometry',
                executable='rf2o_laser_odometry_node',
                name='rf2o_laser_odometry',
                output='screen',
                parameters=[
                    rf2o_config,
                    {'use_sim_time': False}
                ],
                arguments=['--ros-args', '--log-level', 'FATAL'],
                condition=IfCondition(use_rf2o),
            )
        ],
    )

    # ============================================
    # RF2O Covariance Relay (optional)
    # ============================================
    # Sets the covariance of the lidar vx measurement, rejects samples that
    # are inconsistent with the wheel encoders, and republishes the result
    # on /odom_rf2o_fused for the EKF.
    rf2o_relay_node = Node(
        package='amr_localization',
        executable='rf2o_cov_relay',
        name='rf2o_cov_relay',
        output='screen',
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rf2o),
    )

    # ============================================
    # EKF Localization Node
    # ============================================
    # The odom1 block (rf2o, vx only) lives in ekf_real_robot.yaml. When
    # use_rf2o is false, odom1_config is overridden with an all-false vector:
    # robot_localization then creates no subscription to /odom_rf2o_fused at
    # all, so the EKF behaves exactly as it did before rf2o was integrated.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {
                'use_sim_time': False,
                'odom1_config': [
                    False, False, False,
                    False, False, False,
                    False, False, False,
                    False, False, False,
                    False, False, False,
                ],
            }
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered')
        ],
        condition=UnlessCondition(use_rf2o),
    )

    ekf_node_rf2o = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_config,
            {'use_sim_time': False}
        ],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered')
        ],
        condition=IfCondition(use_rf2o),
    )

    # ============================================
    # Return LaunchDescription
    # ============================================
    return LaunchDescription([
        esp32_port_arg,
        lidar_port_arg,
        serial_baudrate_arg,
        scan_mode_arg,
        camera_enable_arg,
        use_rf2o_arg,
        rf2o_delay_arg,

        bringup_info,
        lidar_info,
        ekf_info,

        rsp,
        static_transforms,

        esp32_bridge,
        cmd_vel_splitter,
        lidar_node,
        scan_filter_node,
        camera_driver,
        pointcloud_filter_node,
        rf2o_node,
        rf2o_relay_node,
        ekf_node,
        ekf_node_rf2o,
    ])
