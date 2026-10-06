#!/usr/bin/env python3
# amr_web/state_bridge.py
import threading
import time
from typing import Optional, Dict


class StateBridge:
    """Thread-safe storage for heartbeat monitoring và state management."""

    def __init__(self):
        self._lock = threading.Lock()

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

    # === Device Heartbeat Methods ===
    def update_heartbeat(self, device_name: str) -> None:
        """Cập nhật heartbeat của thiết bị."""
        if self._lock.acquire(blocking=False):
            try:
                self._last_seen[device_name] = time.time()
                self._msg_counts[device_name] += 1
            finally:
                self._lock.release()

    def check_device_statuses(self) -> Dict[str, dict]:
        """Kiểm tra trạng thái kết nối của tất cả thiết bị."""
        current_time = time.time()

        if self._lock.acquire(blocking=False):
            try:
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
            finally:
                self._lock.release()
        else:
            return {device: {'connected': False, 'hz': 0.0} for device in self._timeouts}

    # === Lift State Methods ===
    def set_lift_initialized(self, initialized: bool) -> None:
        """Đánh dấu lift đã được home."""
        if self._lock.acquire(blocking=False):
            try:
                self._lift_initialized = initialized
            finally:
                self._lock.release()

    def update_lift_height(self, height_mm: int) -> None:
        """Cập nhật lift height."""
        if self._lock.acquire(blocking=False):
            try:
                self._lift_height = height_mm
            finally:
                self._lock.release()

    def get_lift_data(self) -> dict:
        """Lấy dữ liệu lift cho web."""
        if self._lock.acquire(blocking=False):
            try:
                height = self._lift_height
                initialized = self._lift_initialized
            finally:
                self._lock.release()
        else:
            height = None
            initialized = False

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

        if self._lock.acquire(blocking=False):
            try:
                battery = self._battery if self._battery else None
            finally:
                self._lock.release()
        else:
            battery = None

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

    # === Original State Methods ===
    def update_pose(self, pose_data: dict) -> None:
        if self._lock.acquire(blocking=False):
            try:
                self._pose = pose_data
            finally:
                self._lock.release()

    def update_battery(self, battery_data: dict) -> None:
        if self._lock.acquire(blocking=False):
            try:
                self._battery = battery_data
            finally:
                self._lock.release()

    def update_map(self, map_data: dict) -> None:
        if self._lock.acquire(blocking=False):
            try:
                self._map = map_data
            finally:
                self._lock.release()

    def update_laser(self, laser_data: dict) -> None:
        if self._lock.acquire(blocking=False):
            try:
                self._laser = laser_data
            finally:
                self._lock.release()

    def get_latest_pose(self) -> Optional[dict]:
        if self._lock.acquire(blocking=False):
            try:
                return self._pose
            finally:
                self._lock.release()
        return None

    def get_latest_battery(self) -> Optional[dict]:
        if self._lock.acquire(blocking=False):
            try:
                return self._battery
            finally:
                self._lock.release()
        return None

    def get_latest_map(self) -> Optional[dict]:
        if self._lock.acquire(blocking=False):
            try:
                return self._map
            finally:
                self._lock.release()
        return None

    def get_latest_laser(self) -> Optional[dict]:
        if self._lock.acquire(blocking=False):
            try:
                return self._laser
            finally:
                self._lock.release()
        return None

    def get_all_state(self) -> dict:
        if self._lock.acquire(blocking=False):
            try:
                result = {}
                if self._pose:
                    result["pose"] = self._pose
                if self._battery:
                    result["battery"] = self._battery
                if self._map:
                    result["map"] = self._map
                return result
            finally:
                self._lock.release()
        return {}
