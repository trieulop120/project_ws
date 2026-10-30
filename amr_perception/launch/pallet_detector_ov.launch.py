#!/usr/bin/env python3
"""Pallet Detector (OpenVINO FP16 backend) launch file.

Starts ONLY the node — same convention as pallet_detector.launch.py. The OV
node pins OMP_NUM_THREADS=4 (measured optimum for a 4-physical-core i7; 8t is
slower). Run this INSTEAD of pallet_detector.launch.py, not alongside it
(both publish the same /pallet/* topics).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='amr_perception',
            executable='pallet_detector_ov',
            name='pallet_detector_node',   # cùng tên node -> giữ nguyên tên topic
            output='screen',
            # Giới hạn 4 luồng CPU cho hiệu năng tối ưu
            additional_env={'OMP_NUM_THREADS': '4',
                           'OPENVINO_CPU_THREADS': '4'},
            # --- Cấu hình tối ưu tốc độ xử lý ---
            # mt_executor: True  -> Chạy đa luồng (Multi-Threaded Executor)
            # timer_period 0.066 -> 15 Hz, khớp với tốc độ camera
            # tf_timeout 0.05    -> Giảm thời gian chờ TF xuống 50ms
            parameters=[
                {'mt_executor': True,
                 'timer_period': 0.066,
                 'tf_timeout': 0.05},
            ],
        ),
    ])