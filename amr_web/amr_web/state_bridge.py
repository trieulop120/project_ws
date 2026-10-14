#!/usr/bin/env python3
# amr_web/state_bridge.py
"""
Thread-safe state storage for Web Bridge.
Phase 1 Fix: Use RLock instead of blocking=False to prevent stale data.
"""
import threading
import time
from typing import Optional, Dict


class StateBridge:
    """Thread-safe storage for heartbeat monitoring và state management."""

    def __init__(self):
        # P0-5 Fix: Use RLock instead of Lock, blocking=True for all operations
        self._lock = threading.RLock()

        # === Device Heartbeat Configuration (timeout = 3.0s) ===
        self._timeouts = {
            'esp32': 3.0,
            'lidar': 3.0,
            'camera': 3.0,
            'lift': 3.0,
        }

        self._last_seen: Dict[str, float] = {
            'esp32': 0.0,
            'lidar': 0.0,
            'camera': 0.0,
            'lift': 0.0,
        }

        # Message counters
        self._msg_counts: Dict[str, int] = {
            'esp32': 0,
            'lidar': 0,
            'camera': 0,
            'lift': 0,
        }
        self._last_hz_check = time.time()
        self._hz_values: Dict[str, float] = {
            'esp32': 0.0,
            'lidar': 0.0,
            'camera': 0.0,
            'lift': 0.0,
        }

        # === Lift State ===
        self._lift_height: Optional[int] = None  # mm
        self._lift_initialized: bool = False  # Đã home chưa

        # === Original State Storage ===
        self._pose: Optional[dict] = None
        self._battery: Optional[dict] = None
        self._map: Optional[dict] = None
        self._laser: Optional[dict] = None

        # === Phase C: Navigation Layers state ===
        self._costmap_global: Optional[dict] = None
        self._costmap_local: Optional[dict] = None
        self._plan: Optional[dict] = None

    # === Device Heartbeat Methods ===
    def update_heartbeat(self, device_name: str) -> None:
        """Cập nhật heartbeat của thiết bị."""
        with self._lock:
            self._last_seen[device_name] = time.time()
            self._msg_counts[device_name] += 1

    def check_device_statuses(self) -> Dict[str, dict]:
        """Kiểm tra trạng thái kết nối của tất cả thiết bị."""
        with self._lock:
            current_time = time.time()

            elapsed = current_time - self._last_hz_check
            if elapsed >= 1.0:
                for device in self._hz_values:
                    count = self._msg_counts[device]
                    self._hz_values[device] = count / elapsed
                    self._msg_counts[device] = 0
                self._last_hz_check = current_time

            devices = {}
            for device, timeout in self._timeouts.items():
                last_seen = self._last_seen.get(device, 0.0)
                connected = (current_time - last_seen) <= timeout
                devices[device] = {
                    'connected': connected,
                    'hz': round(self._hz_values.get(device, 0.0), 1)
                }

            return devices

    # === Lift State Methods ===
    def set_lift_initialized(self, initialized: bool) -> None:
        """Đánh dấu lift đã được home."""
        with self._lock:
            self._lift_initialized = initialized

    def update_lift_height(self, height_mm: int) -> None:
        """Cập nhật lift height."""
        with self._lock:
            self._lift_height = height_mm

    def get_lift_data(self) -> dict:
        """Lấy dữ liệu lift cho web."""
        with self._lock:
            height = self._lift_height
            initialized = self._lift_initialized

        # Chưa home → UNKNOWN
        if not initialized:
            return {
                "height": None,
                "state": "UNKNOWN"
            }

        # Đã home
        if height is not None and height == 0:
            return {
                "height": 0,
                "state": "HOME"
            }

        # Height > 0 → hiển thị số mm, state = None (không hiển thị)
        return {
            "height": height if height is not None else 0,
            "state": None  # Không hiển thị status
        }

    def get_system_overview_state(self) -> dict:
        """Trả về trạng thái tổng quan hệ thống."""
        devices = self.check_device_statuses()
        connected_count = sum(1 for d in devices.values() if d.get('connected', False))
        total_devices = len(devices)

        if connected_count == 0:
            system_status = "Offline"
        elif connected_count < total_devices:
            system_status = "Degraded"
        else:
            system_status = "Online"

        lift_data = self.get_lift_data()

        with self._lock:
            battery = self._battery if self._battery else None

        result = {
            "type": "system_status",
            "timestamp": time.time(),
            "system_status": system_status,
            "devices": devices,
            "lift": lift_data
        }

        if battery:
            result["battery"] = battery

        return result

    # === Map State Methods (P0-5: Add clear methods) ===
    def clear_map(self) -> None:
        """Xoá map state - gọi khi start/stop navigation hoặc SLAM."""
        with self._lock:
            self._map = None

    # === Original State Methods (P0-5 Fix: Use blocking RLock) ===
    def update_pose(self, pose_data: dict) -> None:
        with self._lock:
            self._pose = pose_data

    def update_battery(self, battery_data: dict) -> None:
        with self._lock:
            self._battery = battery_data

    def update_map(self, map_data: dict) -> None:
        with self._lock:
            self._map = map_data

    def update_laser(self, laser_data: dict) -> None:
        with self._lock:
            self._laser = laser_data

    def get_latest_pose(self) -> Optional[dict]:
        with self._lock:
            return self._pose

    def get_latest_battery(self) -> Optional[dict]:
        with self._lock:
            return self._battery

    def get_latest_map(self) -> Optional[dict]:
        with self._lock:
            return self._map

    def get_latest_laser(self) -> Optional[dict]:
        with self._lock:
            return self._laser

    # === Phase C: Costmap + Plan ===
    def update_costmap(self, layer: str, costmap_data: dict) -> None:
        """Luu costmap moi nhat theo layer ('global' hoac 'local')."""
        with self._lock:
            if layer == 'global':
                self._costmap_global = costmap_data
            elif layer == 'local':
                self._costmap_local = costmap_data
            else:
                raise ValueError(f"Unknown costmap layer: {layer}")

    def get_latest_costmap(self, layer: str) -> Optional[dict]:
        with self._lock:
            if layer == 'global':
                return self._costmap_global
            if layer == 'local':
                return self._costmap_local
            return None

    def update_plan(self, plan_data: dict) -> None:
        with self._lock:
            self._plan = plan_data

    def get_latest_plan(self) -> Optional[dict]:
        with self._lock:
            return self._plan

    def clear_layers(self) -> None:
        """Xoa costmap + plan. Goi khi Nav2 tat hoac goal ket thuc."""
        with self._lock:
            self._costmap_global = None
            self._costmap_local = None
            self._plan = None

    def get_all_state(self) -> dict:
        with self._lock:
            result = {}
            if self._pose:
                result["pose"] = self._pose
            if self._battery:
                result["battery"] = self._battery
            if self._map:
                result["map"] = self._map
            if self._costmap_global:
                result["costmap_global"] = self._costmap_global
            if self._costmap_local:
                result["costmap_local"] = self._costmap_local
            if self._plan:
                result["plan"] = self._plan
            return result
