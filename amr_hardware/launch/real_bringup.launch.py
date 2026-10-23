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
        default_value='115200',
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

    use_vo_arg = DeclareLaunchArgument(
        'use_vo',
        default_value='true',
        description='Enable RTAB-Map Visual Odometry & Relay nodes (true/false)'
    )
    use_vo = LaunchConfiguration('use_vo')

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
        parameters=[{'use_sim_time': False}],
    )

    # ============================================
    # Camera Driver (Astra Camera)
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
            'color_fps': 15,           # Khóa 15fps tránh quá tải CPU
            'depth_width': 640,
            'depth_height': 480,
            'depth_fps': 15,
            'enable_depth': True,
            'enable_color': True,
            'use_uvc_camera': True,
            'enable_depth_to_color_registration': False,
            'use_software_d2c': True,  # Bật Software D2C để dóng hàng Depth/Color
            'color_depth_synchronization': True,
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
    # ============================================
    vo_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'frame_id': 'base_footprint',        # Khung gốc robot
            'odom_frame_id': 'odom_vo',          # Khung ảo của VO
            'publish_tf': False,                 # Nhường xuất TF cho EKF
            
            # --- CÁC THAM SỐ CHỦ CHỐT ĐỂ KHÔI PHỤC KHI MẤT TRACK ---
            'guess_frame_id': 'odom',            # Dùng Odom của EKF làm vị trí đoán khi VO bị mù
            'always_process_most_recent_frame': True, # Luôn lấy frame mới nhất, chống dồn hàng đợi
            'approx_sync': True,                 # Bật đồng bộ xấp xỉ nhãn thời gian RGB & Depth
            'queue_size': 10,
            
            # Param thuật toán RTAB-Map
            'Reg/Force3DoF': 'true',             # Ép chạy trên mặt phẳng 2D
            'Odom/Strategy': '0',                # 0=Frame-to-Map (rất ổn định)
            'Odom/ResetCountdown': '1',          # Tự reset keyframe sau 1 frame mất track
            'Odom/Holonomic': 'false',           # Robot xe nâng không đi ngang
            'Vis/MinInliers': '12',              # Số điểm đặc trưng tối thiểu để chấp nhận Odom
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
        parameters=[{
            'use_sim_time': False,
            'vx_covariance': 0.015,       # Mơ hồ hơn Wheel (0.001) để không đè Wheel
            'vyaw_covariance': 0.05,
            'max_valid_covariance': 1.0,  # Ngưỡng vứt bỏ frame mất track
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
        use_vo_arg,
        use_rf2o_arg,
        rf2o_delay_arg,
        rf2o_log_level_arg,

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