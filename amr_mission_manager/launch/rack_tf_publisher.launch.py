#!/usr/bin/env python3
"""
rack_tf_publisher.launch.py
===========================
Chay rack_tf_publisher - phat TF tinh 4 o ke (lưới 2x2) trong khung map.

CHI 3 SO do mot lan khi lap ke (khac ra = so mock, bat buoc chinh):
    rack_x, rack_y, rack_yaw_deg  : goc ke (bin_0) trong map
    pitch_row_m, pitch_col_m      : khoang cach 2 o cung hang / cung cot

Vidu:
    ros2 launch amr_mission_manager rack_tf_publisher.launch.py \
        rack_x:=0.0 rack_y:=1.5 rack_yaw_deg:=90
    ros2 view_tf   # xem map -> bin_0..3
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def _arg(name, default, desc):
    return DeclareLaunchArgument(name, default_value=default, description=desc)


def generate_launch_description():
    # ================================================================
    # PATHS / SO MOCK - chinh sua o day khi co ke that
    # ================================================================
    RACK_X = 0.0
    RACK_Y = 1.5
    RACK_YAW_DEG = 0.0
    PITCH_ROW_M = 0.4
    PITCH_COL_M = 0.4
    RACK_Z_M = 0.0

    # ==================== Launch arguments =========================
    args = [
        _arg('map_frame', 'map', 'frame cha cua ke'),
        _arg('rack_x', str(RACK_X), 'x goc ke (bin_0) trong map (m)'),
        _arg('rack_y', str(RACK_Y), 'y goc ke (bin_0) trong map (m)'),
        _arg('rack_yaw_deg', str(RACK_YAW_DEG), 'yaw ke trong map (deg)'),
        _arg('pitch_row_m', str(PITCH_ROW_M), 'khoang 2 o cung hang (m)'),
        _arg('pitch_col_m', str(PITCH_COL_M), 'khoang 2 o cung cot (m)'),
        _arg('rack_z_m', str(RACK_Z_M), 'cao do mat ke (m)'),
        _arg('publish_markers', 'true', 'hien marker bin_0..3 trong RViz'),
        _arg('publish_once', 'false',
             'true: publish /tf_static 1 lan roi thoat (khop ke fix, khong can node dau)'),
    ]

    from launch_ros.actions import Node
    from launch_ros.parameter_descriptions import ParameterValue

    def pv(name, typ):
        return ParameterValue(LaunchConfiguration(name), value_type=typ)

    node = Node(
        package='amr_mission_manager',
        executable='rack_tf_publisher',
        name='rack_tf_publisher',
        output='screen',
        parameters=[{
            'map_frame': LaunchConfiguration('map_frame'),
            'rack_x': pv('rack_x', float),
            'rack_y': pv('rack_y', float),
            'rack_yaw_deg': pv('rack_yaw_deg', float),
            'pitch_row_m': pv('pitch_row_m', float),
            'pitch_col_m': pv('pitch_col_m', float),
            'rack_z_m': pv('rack_z_m', float),
            'publish_markers': pv('publish_markers', bool),
            'publish_once': pv('publish_once', bool),
        }],
    )
    return LaunchDescription(args + [node])
