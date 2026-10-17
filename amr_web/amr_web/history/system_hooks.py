"""
amr_web/history/system_hooks.py
================================
Edge-triggered system event detection.

Plan §6: device connected/disconnected transitions from `system_status`
(esp32, lidar, camera, lift), Nav2 running state change, robot online/offline,
battery below 18% (once per episode, re-arm above 25%), and `nav_result`
outcomes.

The hooks poll the latest state snapshot from StateBridge once per tick. The
caller (`bridge_node._broadcast_forever`) calls `tick()` at most once per
100ms — same cadence as existing broadcasts. Each tick is O(1).

Pure module: no class. State lives in module-level dicts guarded by an
RLock. Soft-fail: every operation is wrapped in try/except, no side effects
on the rest of the system.
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from amr_web.history import events


_lock = threading.RLock()

# Last observed device connection states. Key: device name.
_device_last: dict = {}

# Last observed nav2 running state.
_nav2_last: Optional[bool] = None

# Last observed slam active state.
_slam_last: Optional[bool] = None

# Battery episode state: 'normal' (above 25%), 'low' (below 18%, event fired).
_battery_state: str = "normal"
_battery_last_pct: Optional[float] = None


# Thresholds — keep in sync with plan §6 wording.
_BATTERY_LOW_THRESHOLD = 18.0
_BATTERY_RECOVER_THRESHOLD = 25.0


def tick(
    system_status: Optional[dict] = None,
    nav_state: Optional[dict] = None,
    slam_active: Optional[bool] = None,
    battery: Optional[dict] = None,
) -> None:
    """Call from the broadcaster tick (or anywhere with the same cadence).

    All four arguments are optional. Missing args mean "no change" for that
    dimension. The function is cheap: at most a few dict lookups and a couple
    of event enqueues (when an edge actually fires).
    """
    try:
        if system_status is not None:
            _check_devices(system_status)
        if nav_state is not None:
            _check_nav_state(nav_state)
        if slam_active is not None:
            _check_slam(slam_active)
        if battery is not None:
            _check_battery(battery)
    except Exception:
        pass


def reset() -> None:
    """Forget all edge state. Used by tests; safe to call at startup too."""
    global _device_last, _nav2_last, _slam_last, _battery_state, _battery_last_pct
    with _lock:
        _device_last = {}
        _nav2_last = None
        _slam_last = None
        _battery_state = "normal"
        _battery_last_pct = None


# ---------------------------------------------------------------------------
# Internals
# ---------------------------------------------------------------------------

def _check_devices(system_status: dict) -> None:
    devices = system_status.get("devices") or {}
    with _lock:
        last = dict(_device_last)
    for name, info in devices.items():
        if not isinstance(info, dict):
            continue
        connected = bool(info.get("connected"))
        prev = last.get(name)
        if prev is None:
            # First observation: do not fire an edge event. The device may
            # have been connected since the backend started, before history
            # was enabled. We only fire on *transitions*.
            with _lock:
                _device_last[name] = connected
            continue
        if prev != connected:
            with _lock:
                _device_last[name] = connected
            events.record_device_transition(name, connected)


def _check_nav_state(nav_state: dict) -> None:
    """nav_state example (from nav_bridge.get_navigation_state):
    {"type": "nav_state", "state": "ACTIVE"|"IDLE"|"SUCCEEDED"|..., ...}

    'state' here is the goal state machine, NOT the Nav2 process. Plan §6
    asks for "Nav2 running state change" — that is `bridge_node.is_navigation_running()`,
    not the goal state. We accept both: the caller can pass either by
    setting `nav_state` to a dict with a 'running' key, or by providing
    `slam_active` (separate arg). The plain nav_state dict is for
    completeness and is currently unused.
    """
    global _nav2_last
    running = nav_state.get("running")
    if not isinstance(running, bool):
        return
    with _lock:
        last = _nav2_last
    if last is None or last != running:
        with _lock:
            _nav2_last = running
        events.record_nav_state_change(running)


def _check_slam(active: bool) -> None:
    global _slam_last
    with _lock:
        last = _slam_last
    if last is None or last != active:
        with _lock:
            _slam_last = active
        events.record_slam_state_change(active)


def _check_battery(battery: dict) -> None:
    global _battery_state, _battery_last_pct
    pct = battery.get("percentage")
    if not isinstance(pct, (int, float)):
        return
    pct = float(pct)
    with _lock:
        prev_state = _battery_state
        _battery_last_pct = pct
    if pct < _BATTERY_LOW_THRESHOLD and prev_state != "low":
        with _lock:
            _battery_state = "low"
        events.record_battery_low(pct)
    elif pct > _BATTERY_RECOVER_THRESHOLD and prev_state == "low":
        with _lock:
            _battery_state = "normal"
        events.record_battery_recovered(pct)
