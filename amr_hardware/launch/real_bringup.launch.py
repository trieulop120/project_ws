#!/usr/bin/env python3
"""
Real Hardware Bringup Launch File
File location: amr_hardware/launch/real_bringup.launch.py
RTAB-Map VO tuned for 20-30Hz / low CPU
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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

    esp32_baudrate_arg = DeclareLaunchArgument(
        'esp32_baudrate',
        default_value='921600',
        description='ESP32 serial baudrate'
    )
    esp32_baudrate = LaunchConfiguration('esp32_baudrate')

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

    # Tat point cloud cua driver neu khong co node nao dung /points
    # (tiet kiem CPU dang ke: driver khong phai tao cloud 640x480 @30Hz)
    enable_pointcloud_arg = DeclareLaunchArgument(
        'enable_pointcloud',
        default_value='true',
        description='Enable astra driver point cloud (/points) + downsampler'
    )
    enable_pointcloud = LaunchConfiguration('enable_pointcloud')

    use_vo_arg = DeclareLaunchArgument(
        'use_vo',
        default_value='true',
        description='Enable RTAB-Map Visual Odometry & Relay nodes (true/false)'
    )
    use_vo = LaunchConfiguration('use_vo')

    # Giam do phan giai xu ly cua VO: 1 = full 640x480, 2 = 320x240
    vo_decimation_arg = DeclareLaunchArgument(
        'vo_decimation',
        default_value='2',
        description='Odom/ImageDecimation of RTAB-Map VO (1,2,4)'
    )
    vo_decimation = LaunchConfiguration('vo_decimation')

    vo_max_features_arg = DeclareLaunchArgument(
        'vo_max_features',
        default_value='400',
        description='Vis/MaxFeatures of RTAB-Map VO'
    )
    vo_max_features = LaunchConfiguration('vo_max_features')

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

    rf2o_log_level_arg = DeclareLaunchArgument(
        'rf2o_log_level',
        default_value='warn',
        description='Log level of the rf2o node (debug/info/warn/error/fatal)'
    )
    rf2o_log_level = LaunchConfiguration('rf2o_log_level')

    # Log level cho cac node hay spam (rgbd_odometry, relay, filter)
    # Doi khi chay: quiet_log_level:=info de debug
    quiet_log_level_arg = DeclareLaunchArgument(
        'quiet_log_level',
        default_value='error',
        description='Log level for noisy nodes (debug/info/warn/error/fatal)'
    )
    quiet_log_level = LaunchConfiguration('quiet_log_level')

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
    ekf_info = LogInfo(
        msg='[real_bringup] Localization: EKF -> /odometry/filtered '
            '(TF odom -> base_footprint)'
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
            'baudrate': ParameterValue(esp32_baudrate, value_type=int),
            'wheel_radius': 0.0509,
            'wheelbase': 0.4813,
            'odom_scale': 1.0,   # hieu chinh quang duong odometry (chi tinh tien)
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
            'serial_baudrate': ParameterValue(serial_baudrate, value_type=int),
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': ParameterValue(scan_mode, value_type=str),
        }],
    )

    # ============================================
    # LiDAR Scan Filter Node
    # ============================================
    scan_filter_node = Node(
        package='amr_perception',
        executable='scan_filter_node',
        name='scan_filter_node',
        output='screen',
        ros_arguments=['--log-level', quiet_log_level],
        parameters=[{'use_sim_time': False}],
    )

    # ============================================
    # Camera Driver (Astra Camera) - 30Hz
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
            'color_fps': 30,
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 30,
            'enable_depth': True,
            'enable_color': True,
            'use_uvc_camera': True,
            'uvc_camera_format': 'yuyv',   # dinh dang tho, giam CPU giai nen (MJPEG)
            'enable_depth_to_color_registration': False,
            'use_software_d2c': True,  # Bật Software D2C để dóng hàng Depth/Color
            'color_depth_synchronization': True,
            # TF camera (camera_link -> *_optical_frame) là TF DYNAMIC của driver,
            # mặc định chỉ 10 Hz trong khi ảnh stamp 30 Hz -> TF2 WARN
            # "extrapolation into the future" (rgbd_odometry) và pallet node block
            # ở tf_timeout. Ép TF camera chạy bằng tần số ảnh (30 Hz).
            # NOTE: driver đọc tham số này bằng double (default 10.0) — truyền int
            # sẽ bị type mismatch và driver giữ nguyên default (đã xảy ra, log "10 Hz").
            'tf_publish_rate': 0.0,
        }],
        remappings=[
            ('/depth/points', '/points'),
            ('/camera/depth/image_raw', '/depth/image_raw'),
            ('/camera/color/image_raw', '/color/image_raw'),
            ('/camera/color/camera_info', '/color/camera_info'),
        ],
        condition=IfCondition(camera_enable)
    )

    # ============================================
    # PointCloud Downsampler Node
    # ============================================
    pointcloud_filter_node = Node(
        package='amr_perception',
        executable='pointcloud_downsampler',
        name='pointcloud_filter',
        output='screen',
        ros_arguments=['--log-level', quiet_log_level],
        parameters=[{
            'use_sim_time': False,
            'leaf_size': 0.05,
            'z_min': 0.05,
            'z_max': 1.20,
        }],
        condition=IfCondition(camera_enable)
    )

    # ============================================
    # Visual Odometry Node (RTAB-Map RGB-D)
    # Muc tieu: 20-30Hz (= toc do camera) voi CPU thap
    # ============================================
    vo_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        ros_arguments=['--log-level', quiet_log_level],
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom_vo',
            'publish_tf': False,                 # Nhuong xuat TF cho EKF

            'guess_frame_id': 'odom',            # Dung odom cua EKF lam vi tri doan khi VO bi mu
            'always_process_most_recent_frame': True,  # Luon lay frame moi nhat, chong don hang doi
            'approx_sync': True,
            'approx_sync_max_interval': 0.09,    # ~1 frame @30Hz
            'queue_size': 5,                     # queue nho = it tre, it RAM

            # --- Tham so thuat toan RTAB-Map (tat ca phai la string) ---
            'Reg/Force3DoF': 'true',
            'Odom/Strategy': '0',                # 0=Frame-to-Map. Neu van nang -> doi '1' (Frame-to-Frame)
            'Odom/ResetCountdown': '1',
            'Odom/Holonomic': 'false',
            'Vis/MinInliers': '12',              # mat track nhieu -> thu ha xuong 10

            # --- Toi uu CPU ---
            'Odom/ImageDecimation': ParameterValue(vo_decimation, value_type=str),
            'Vis/MaxFeatures': ParameterValue(vo_max_features, value_type=str),
            'Vis/FeatureType': '6',              # GFTT/BRIEF: nhanh hon mac dinh 8 (GFTT/ORB)
            'GFTT/MinDistance': '7',
            'OdomF2M/MaxSize': '800',            # so diem toi da cua local map (mac dinh 2000)
            'OdomF2M/BundleAdjustment': '0',     # tat g2o BA: giam CPU manh, EKF da bu drift
            'Odom/KeyFrameThr': '0.3',
        }],
        remappings=[
            ('rgb/image', '/color/image_raw'),
            ('depth/image', '/depth/image_raw'),
            ('rgb/camera_info', '/color/camera_info'),
            ('odom', '/rtabmap/odom'),
        ],
        condition=IfCondition(use_vo)
    )

    # ============================================
    # Visual Odometry Covariance Relay Node
    # ============================================
    vo_relay_node = Node(
        package='amr_localization',
        executable='vo_cov_relay',
        name='vo_cov_relay',
        output='screen',
        ros_arguments=['--log-level', quiet_log_level],
        parameters=[{
            'use_sim_time': False,
            'vx_covariance': 0.015,       # Mo ho hon Wheel (0.001) de khong de Wheel
            'vyaw_covariance': 0.05,
            'vx_scale': 0.98,             # Hieu chinh vx cua VO (thuc/do)
            'max_valid_covariance': 1.0,  # Nguong vut bo frame mat track
            'reset_skip_frames': 5,
        }],
        condition=IfCondition(use_vo)
    )

    # ============================================
    # RF2O Laser Odometry (optional, vx source)
    # ============================================
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
                ros_arguments=['--log-level', rf2o_log_level],
                condition=IfCondition(use_rf2o),
            )
        ],
    )

    # ============================================
    # RF2O Covariance Relay (optional)
    # ============================================
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
        esp32_baudrate_arg,
        lidar_port_arg,
        serial_baudrate_arg,
        scan_mode_arg,
        camera_enable_arg,
        enable_pointcloud_arg,
        use_vo_arg,
        vo_decimation_arg,
        vo_max_features_arg,
        use_rf2o_arg,
        rf2o_delay_arg,
        rf2o_log_level_arg,
        quiet_log_level_arg,

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
        vo_node,
        vo_relay_node,
        rf2o_node,
        rf2o_relay_node,
        ekf_node,
        ekf_node_rf2o,
    ])