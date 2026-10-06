#!/usr/bin/env python3
# amr_web/converters.py
import base64
import math
import time
from typing import Dict, Optional, List
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import BatteryState, Image, LaserScan


def system_status_to_json(
    devices: Dict[str, dict],
    battery_data: Optional[dict] = None
) -> dict:
    """
    Tạo JSON payload cho trạng thái hệ thống và thiết bị.
    Bao gồm: system_status, devices heartbeat, telemetry cơ bản.
    """
    # Đếm số thiết bị connected
    connected_count = sum(1 for d in devices.values() if d.get('connected', False))
    total_devices = len(devices)

    # Xác định system_status
    if connected_count == 0:
        system_status = "Offline"
    elif connected_count < total_devices:
        system_status = "Degraded"
    else:
        system_status = "Online"

    result = {
        "type": "system_status",
        "timestamp": time.time(),
        "system_status": system_status,
        "devices": {
            name: {
                "connected": data.get('connected', False),
                "hz": round(data.get('hz', 0.0), 1)
            }
            for name, data in devices.items()
        }
    }

    # Thêm battery telemetry nếu có
    if battery_data:
        result["battery"] = battery_data

    return result


def quaternion_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Convert quaternion to yaw in radians."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def odom_to_json(msg: Odometry) -> dict:
    """Convert Odometry message to application-level JSON."""
    pose = msg.pose.pose
    return {
        "type": "pose",
        "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "frame_id": msg.header.frame_id,
        "position": {
            "x": round(pose.position.x, 3),
            "y": round(pose.position.y, 3),
        },
        "orientation": {
            "yaw": round(quaternion_to_yaw(
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ), 3)
        }
    }


def battery_to_json(msg: BatteryState) -> dict:
    """Convert BatteryState message to application-level JSON."""
    percentage = None
    if not math.isnan(msg.percentage) and 0.0 <= msg.percentage <= 1.0:
        percentage = round(msg.percentage * 100, 1)

    voltage = None
    if not math.isnan(msg.voltage):
        voltage = round(msg.voltage, 1)

    current = None
    if not math.isnan(msg.current):
        current = round(msg.current, 1)

    charging = msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING

    return {
        "type": "battery",
        "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "percentage": percentage,
        "voltage": voltage,
        "current": current,
        "charging": charging,
    }


def map_to_json(msg: OccupancyGrid) -> dict:
    """Convert OccupancyGrid to application-level JSON (Compressed Base64)."""
    compressed = []
    for val in msg.data:
        if val == -1:
            compressed.append(0)  # Unknown
        elif val == 0:
            compressed.append(1)  # Free
        else:
            compressed.append(2)  # Occupied

    data_bytes = bytes(compressed)
    encoded = base64.b64encode(data_bytes).decode('ascii')

    return {
        "type": "map",
        "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "frame_id": msg.header.frame_id,
        "info": {
            "resolution": round(msg.info.resolution, 4),
            "width": msg.info.width,
            "height": msg.info.height,
            "origin": {
                "x": round(msg.info.origin.position.x, 4),
                "y": round(msg.info.origin.position.y, 4),
                "theta": round(quaternion_to_yaw(
                    msg.info.origin.orientation.x,
                    msg.info.origin.orientation.y,
                    msg.info.origin.orientation.z,
                    msg.info.origin.orientation.w
                ), 4)
            }
        },
        "data": encoded,
    }


def image_to_json(msg: Image, max_width: int = 640, quality: int = 75) -> dict:
    """Convert ROS Image message to JPEG base64 for web display."""
    try:
        import cv2
        import numpy as np

        if msg.encoding == 'rgb8':
            np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            np_image = np_image[:, :, ::-1]
        elif msg.encoding == 'bgr8':
            np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif msg.encoding == 'mono8':
            np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        elif msg.encoding == 'rgba8':
            np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
            np_image = np_image[:, :, :3]
            np_image = np_image[:, :, ::-1]
        else:
            np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)

        if max_width > 0 and np_image.shape[1] > max_width:
            scale = max_width / np_image.shape[1]
            new_height = int(np_image.shape[0] * scale)
            np_image = cv2.resize(np_image, (max_width, new_height))

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        _, buffer = cv2.imencode('.jpg', np_image, encode_param)
        encoded = base64.b64encode(buffer).decode('ascii')

        return {
            "type": "camera",
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "frame_id": msg.header.frame_id,
            "width": np_image.shape[1],
            "height": np_image.shape[0],
            "encoding": msg.encoding,
            "data": encoded,
        }
    except Exception as e:
        print(f"Image conversion error: {e}")
        return None


def laser_to_json(msg: LaserScan) -> dict:
    """Convert LaserScan to application-level JSON for web visualization."""
    return {
        "type": "laser",
        "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "frame_id": msg.header.frame_id,
        "angle_min": round(msg.angle_min, 4),
        "angle_max": round(msg.angle_max, 4),
        "angle_increment": round(msg.angle_increment, 4),
        "range_min": round(msg.range_min, 3),
        "range_max": round(msg.range_max, 3),
        "ranges": [round(r, 3) if not math.isinf(r) else msg.range_max for r in msg.ranges],
    }