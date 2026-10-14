#!/usr/bin/env python3
# amr_web/converters.py
import base64
import math
import time
from typing import Dict, Optional, List, Tuple
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from sensor_msgs.msg import BatteryState, Image, LaserScan


def system_status_to_json(
    devices: Dict[str, dict],
    battery_data: Optional[dict] = None
) -> dict:
    """
    Tạo JSON payload cho trạng thái hệ thống và thiết bị.
    Bao gồm: system_status, devices heartbeat, telemetry cơ bản.
    """
    connected_count = sum(1 for d in devices.values() if d.get('connected', False))
    total_devices = len(devices)

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


def image_to_json(msg: Image, max_width: int = 480, quality: int = 60) -> dict:
    """Convert ROS Image message (RGB/BGR/Mono/Depth) to JPEG base64."""
    try:
        import cv2
        import numpy as np

        encoding = msg.encoding.lower() if hasattr(msg, 'encoding') else 'rgb8'
        np_image = None

        # 1. Xử lý ảnh Depth 16UC1 / MONO16 (16-bit uint - mm)
        if '16u' in encoding or 'mono16' in encoding:
            depth_data = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
            depth_clipped = np.clip(depth_data, 0, 5000)  # Giới hạn 0 - 5000mm (5m)
            depth_normalized = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            np_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        # 2. Xử lý ảnh Depth 32FC1 (32-bit float - m)
        elif '32f' in encoding:
            depth_data = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            depth_data = np.nan_to_num(depth_data, nan=0.0)
            depth_clipped = np.clip(depth_data, 0.0, 5.0)  # Giới hạn 0 - 5.0m
            depth_normalized = (depth_clipped / 5.0 * 255.0).astype(np.uint8)
            np_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)

        # 3. Các định dạng RGB/BGR/Mono8 thông thường
        elif 'rgb' in encoding:
            np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            np_image = np_image[:, :, ::-1]  # RGB sang BGR
        elif 'bgr' in encoding:
            np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        elif 'mono8' in encoding or '8uc1' in encoding:
            np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
        elif 'rgba' in encoding:
            np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
            np_image = np_image[:, :, :3][:, :, ::-1]
        else:
            # Dự phòng tự động suy luận kiểu dữ liệu theo kích thước buffer
            total_elements = len(msg.data)
            num_pixels = msg.height * msg.width
            bytes_per_pixel = total_elements // num_pixels

            if bytes_per_pixel == 2:  # Ảnh 16-bit Depth
                depth_data = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                depth_clipped = np.clip(depth_data, 0, 5000)
                depth_normalized = cv2.normalize(depth_clipped, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                np_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            elif bytes_per_pixel == 4:  # Ảnh 32-bit Float Depth
                depth_data = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
                depth_data = np.nan_to_num(depth_data, nan=0.0)
                depth_clipped = np.clip(depth_data, 0.0, 5.0)
                depth_normalized = (depth_clipped / 5.0 * 255.0).astype(np.uint8)
                np_image = cv2.applyColorMap(depth_normalized, cv2.COLORMAP_JET)
            else:
                channels = max(1, bytes_per_pixel)
                if channels == 1:
                    np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                else:
                    np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, channels)[:, :, :3]

        if np_image is None:
            return None

        # Resize ảnh để tối ưu băng thông truyền qua WebSocket
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
    """Convert LaserScan to application-level JSON for web visualization.
    P1-8 Fix: Handle NaN values properly for valid JSON.
    """
    def clean_range(r):
        """Clean range value: replace NaN/inf with None (JSON null)."""
        if math.isnan(r) or math.isinf(r) or r < msg.range_min or r > msg.range_max:
            return None
        return round(r, 3)

    return {
        "type": "laser",
        "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "frame_id": msg.header.frame_id,
        "angle_min": round(msg.angle_min, 4),
        "angle_max": round(msg.angle_max, 4),
        "angle_increment": round(msg.angle_increment, 4),
        "range_min": round(msg.range_min, 3),
        "range_max": round(msg.range_max, 3),
        "ranges": [clean_range(r) for r in msg.ranges],
    }


# ============================================================================
# Phase C - Navigation Layers: costmap + plan
# ============================================================================

def compose_planar(tx: float, ty: float, tyaw: float,
                   x: float, y: float, yaw: float) -> Tuple[float, float, float]:
    """Compose planar transform: tra ve pose (x, y, yaw) cua diem (x,y,yaw)
    trong khung dich sau khi apply T = (tx, ty, tyaw).
    T_map_point = T_map_T . T_point (chinh xac theo Phase C docs 3.4).
    """
    c, s = math.cos(tyaw), math.sin(tyaw)
    return (tx + c * x - s * y, ty + s * x + c * y, tyaw + yaw)


def _costmap_bytes(msg: OccupancyGrid) -> bytes:
    """Convert OccupancyGrid int8 -> uint8 bytes (dung numpy neu co, fallback generator)."""
    try:
        import numpy as np
        return np.asarray(msg.data, dtype=np.int8).view(np.uint8).tobytes()
    except Exception:
        return bytes(v & 0xFF for v in msg.data)


def costmap_to_json(msg: OccupancyGrid, layer: str,
                    origin: Optional[Tuple[float, float, float]] = None) -> dict:
    """Convert OccupancyGrid (costmap) sang JSON payload.

    layer: 'global' hoac 'local'.
    origin: tuple (x_map, y_map, yaw_map) da quy doi ve khung 'map'. Neu None
            thi lay truc tiep tu msg.info.origin (gia su msg da o khung 'map').

    Schema:
        {"type":"costmap","layer":..,"timestamp":..,"frame_id":"map",
         "info":{"resolution":..,"width":W,"height":H,
                 "origin":{"x":..,"y":..},"yaw":..},
         "data":"<base64 uint8>"}
    """
    if origin is not None:
        ox, oy, oyaw = origin
    else:
        info = msg.info
        ox = info.origin.position.x
        oy = info.origin.position.y
        oyaw = quaternion_to_yaw(
            info.origin.orientation.x,
            info.origin.orientation.y,
            info.origin.orientation.z,
            info.origin.orientation.w,
        )

    data_b64 = base64.b64encode(_costmap_bytes(msg)).decode('ascii')

    return {
        "type": "costmap",
        "layer": layer,
        "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "frame_id": "map",  # Phase C: frontend chi lam viec voi khung 'map'
        "info": {
            "resolution": float(msg.info.resolution),
            "width": int(msg.info.width),
            "height": int(msg.info.height),
            "origin": {
                "x": round(float(ox), 4),
                "y": round(float(oy), 4),
            },
            "yaw": round(float(oyaw), 6),
        },
        "data": data_b64,
    }


def plan_to_json(msg: Path,
                 tf: Optional[Tuple[float, float, float]] = None) -> dict:
    """Convert nav_msgs/Path sang JSON payload (da o khung 'map').

    tf: neu plan o khung khac 'map', truyen (tx, ty, tyaw) = T_map_frame de
        quy doi moi diem qua compose_planar. Neu None, gia su msg da o 'map'.

    Giam mau: giu diem dau/cuoi + cac diem cach diem truoc >= 0.05 m,
    toi da 2000 diem, lam tron 3 chu so. < 2 diem -> points = [].
    """
    if tf is not None:
        tx, ty, tyaw = tf
    else:
        tx, ty, tyaw = (0.0, 0.0, 0.0)

    raw_points: List[Tuple[float, float]] = []
    for ps in msg.poses:
        p: Pose = ps.pose
        px, py = p.position.x, p.position.y
        if tf is not None:
            px, py, _ = compose_planar(tx, ty, tyaw, px, py, 0.0)
        raw_points.append((px, py))

    # Giam mau
    sampled: List[List[float]] = []
    if len(raw_points) >= 1:
        sampled.append([round(raw_points[0][0], 3), round(raw_points[0][1], 3)])
    for i in range(1, len(raw_points) - 1):
        x, y = raw_points[i]
        lx, ly = sampled[-1]
        if (x - lx) ** 2 + (y - ly) ** 2 >= 0.05 ** 2:
            sampled.append([round(x, 3), round(y, 3)])
            if len(sampled) >= 2000:
                break
    if len(raw_points) >= 2:
        lx, ly = raw_points[-1]
        if not sampled or sampled[-1] != [round(lx, 3), round(ly, 3)]:
            if len(sampled) < 2000:
                sampled.append([round(lx, 3), round(ly, 3)])

    return {
        "type": "plan",
        "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        "frame_id": "map",  # Phase C: frontend chi lam viec voi khung 'map'
        "points": sampled if len(sampled) >= 2 else [],
    }