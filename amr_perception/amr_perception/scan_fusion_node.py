#!/usr/bin/env python3
"""Scan Fusion Node - Merges LiDAR and Depth Camera LaserScan

This node subscribes to:
  - /scan (LiDAR) - 360° scan
  - /scan_depth (Depth Camera) - ~60° front-facing scan

And publishes:
  - /scan_fused - merged scan for SLAM and Nav2

Logic:
  - Use LiDAR scan as base
  - Add depth camera points where LiDAR has no data or depth is closer

Author: AMR Project
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan


class ScanFusionNode(Node):
    def __init__(self):
        super().__init__('scan_fusion_node')

        # Declare parameters
        self.declare_parameter('scan_topic_lidar', '/scan')
        self.declare_parameter('scan_topic_depth', '/scan_depth')
        self.declare_parameter('output_scan_topic', '/scan_fused')
        self.declare_parameter('queue_size', 10)
        self.declare_parameter('use_depth_scan', True)
        self.declare_parameter('target_frame', 'lidar_link')

        self.scan_lidar = None
        self.scan_depth = None

        # Get parameters
        self.scan_topic_lidar = self.get_parameter('scan_topic_lidar').value
        self.scan_topic_depth = self.get_parameter('scan_topic_depth').value
        self.output_scan_topic = self.get_parameter('output_scan_topic').value
        self.use_depth_scan = self.get_parameter('use_depth_scan').value
        self.target_frame = self.get_parameter('target_frame').value

        # Subscribers
        self.sub_lidar = self.create_subscription(
            LaserScan,
            self.scan_topic_lidar,
            self.lidar_callback,
            10
        )

        if self.use_depth_scan:
            self.sub_depth = self.create_subscription(
                LaserScan,
                self.scan_topic_depth,
                self.depth_callback,
                10
            )

        # Publisher
        self.pub_fused = self.create_publisher(LaserScan, self.output_scan_topic, 10)

        self.get_logger().info(f'Scan Fusion Node started')
        self.get_logger().info(f'  LiDAR: {self.scan_topic_lidar}')
        self.get_logger().info(f'  Depth: {self.scan_topic_depth}')
        self.get_logger().info(f'  Output: {self.output_scan_topic}')

    def lidar_callback(self, msg: LaserScan):
        """Store LiDAR scan, publish fused when depth is available"""
        self.scan_lidar = msg

        if self.scan_depth is None or not self.use_depth_scan:
            # No depth data, publish LiDAR only with correct frame
            fused = msg
            fused.header.frame_id = self.target_frame
            self.pub_fused.publish(fused)
        else:
            # Fuse scans
            fused = self.fuse_scans(self.scan_lidar, self.scan_depth)
            self.pub_fused.publish(fused)

    def depth_callback(self, msg: LaserScan):
        """Store depth scan"""
        self.scan_depth = msg

    def fuse_scans(self, lidar_scan: LaserScan, depth_scan: LaserScan) -> LaserScan:
        """Fuse LiDAR and depth camera scans using simple angular offset method"""
        # Create output scan based on LiDAR
        fused = LaserScan()
        fused.header = lidar_scan.header
        fused.header.frame_id = self.target_frame

        # Use LiDAR parameters as base
        fused.angle_min = lidar_scan.angle_min
        fused.angle_max = lidar_scan.angle_max
        fused.angle_increment = lidar_scan.angle_increment
        fused.time_increment = lidar_scan.time_increment
        fused.scan_time = lidar_scan.scan_time
        fused.range_min = max(lidar_scan.range_min, depth_scan.range_min)
        fused.range_max = min(lidar_scan.range_max, depth_scan.range_max)

        n_ranges = len(lidar_scan.ranges)
        fused.ranges = list(lidar_scan.ranges)
        fused.intensities = list(lidar_scan.intensities) if lidar_scan.intensities else []

        try:
            # Calculate angular offset between camera and LiDAR
            # Camera is mounted in front of LiDAR, so we need to offset
            # This is a simplified fusion - just blend the two scans

            # Get camera mounting angle offset (camera is ~30 degrees offset from front)
            camera_yaw_offset = 0.0  # radians - adjust based on your robot geometry

            depth_ranges = np.array(depth_scan.ranges)
            n_depth = len(depth_ranges)
            depth_angles = np.linspace(
                depth_scan.angle_min + camera_yaw_offset,
                depth_scan.angle_max + camera_yaw_offset,
                n_depth
            )

            # Filter valid depth points
            valid_mask = np.isfinite(depth_ranges) & \
                        (depth_ranges >= depth_scan.range_min) & \
                        (depth_ranges <= depth_scan.range_max)

            if np.any(valid_mask):
                # Map depth points to LiDAR angular bins
                for i, (angle, rng) in enumerate(zip(depth_angles, depth_ranges)):
                    if not valid_mask[i]:
                        continue

                    if angle >= lidar_scan.angle_min and angle <= lidar_scan.angle_max:
                        bin_idx = int((angle - lidar_scan.angle_min) / lidar_scan.angle_increment)
                        if 0 <= bin_idx < n_ranges:
                            current_range = fused.ranges[bin_idx]

                            # Use depth point if:
                            # 1. Current range is invalid (inf)
                            # 2. OR depth point is closer (for obstacle detection)
                            is_current_valid = np.isfinite(current_range)
                            if not is_current_valid or (np.isfinite(rng) and rng < current_range):
                                fused.ranges[bin_idx] = rng

        except Exception as e:
            self.get_logger().warn(f'Fusion error: {e}', throttle_duration_sec=5.0)

        return fused


def main(args=None):
    rclpy.init(args=args)
    node = ScanFusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
