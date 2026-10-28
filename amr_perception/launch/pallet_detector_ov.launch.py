#!/usr/bin/env python3
"""Pallet Detector (OpenVINO INT8 backend) launch file.

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
            name='pallet_detector_node',   # same node name -> same topic names
            output='screen',
            # Measured: 4 threads is the optimum; do not raise to 8.
            additional_env={'OMP_NUM_THREADS': '4',
                           'OPENVINO_CPU_THREADS': '4'},
            # --- Rate boost (see report) ---------------------------------
            # mt_executor: true  -> multi-threaded executor (timer fires while
            #                          other callbacks run); removes the
            #                          single-thread blocking ceiling.
            # timer_period 0.066 -> 15 Hz, matching the camera sync rate.
            # tf_timeout 0.05    -> a TF gap waits 50ms, not 200ms; this is what
            #                          was dragging the full-stack rate to ~2 Hz.
            parameters=[
                {'mt_executor': True,
                 'timer_period': 0.066,
                 'tf_timeout': 0.05},
            ],
        ),
    ])
