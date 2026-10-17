"""
amr_web/history/events.py
=========================
Public helpers for logging operation events.

Design rules (HISTORY_RESULTS_PLAN.md §1.3):
- Every call is non-blocking. We enqueue to the writer thread; we never block
  the caller.
- Every call is wrapped in try/except so a history-side failure cannot affect
  robot control or websocket flow.
- High-rate sources (pose, laser, camera, costmap, plan, get_map, nav_state
  heartbeat) are NEVER logged here. Callers must filter before calling.
- The function names match the plan's "record_*" wording so the call sites
  read like English prose.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Optional

from amr_web.history import db


# ---------------------------------------------------------------------------
# Command logging
# ---------------------------------------------------------------------------
# db.log_event() allocates the row id in-process and returns it immediately, so the
# id we hand back here is the REAL events.id. update_command_result() then updates
# exactly that row (the writer queue is FIFO: the insert always runs before the update).

def record_command(
    cmd: dict,
    client_ip: Optional[str] = None,
) -> Optional[int]:
    """Enqueue a 'requested' event for an incoming WS command.

    Returns the event id (pass it to update_command_result later), or None when the
    command is empty or the history feature is disabled.
    """
    try:
        if not db.is_enabled():
            return None
        command = str(cmd.get("command", ""))[:64]
        if not command:
            return None
        return db.log_event(
            category=_category_for_command(command),
            level="info",
            action=command,
            detail=_build_command_detail(command, cmd),
            params=_sanitize_params(cmd),
            result="requested",
            client_ip=client_ip,
        )
    except Exception:
        return None


def update_command_result(
    event_id: Optional[int],
    success: bool,
    message: Optional[str] = None,
) -> None:
    """Set the final result ('success'/'failed') of the event created by
    record_command(). `message` is appended to the event detail.
    No-op when event_id is None or the feature is disabled."""
    try:
        if event_id is None or not db.is_enabled():
            return
        db.update_event_result(
            event_id, "success" if success else "failed", message)
    except Exception:
        pass


# ---------------------------------------------------------------------------
# System events
# ---------------------------------------------------------------------------

def record_system_event(
    category: str,
    action: str,
    level: str = "info",
    detail: Optional[str] = None,
    params: Optional[dict] = None,
    result: Optional[str] = None,
    client_ip: Optional[str] = None,
) -> None:
    """Generic system event. Used for ws_connected, device transitions, etc."""
    try:
        if not db.is_enabled():
            return
        db.log_event(
            category=category,
            level=level,
            action=action,
            detail=detail,
            params=params,
            result=result,
            client_ip=client_ip,
        )
    except Exception:
        pass


def record_ws_connect(client_ip: Optional[str]) -> None:
    record_system_event(
        category="CONNECTION",
        action="ws_connected",
        level="info",
        client_ip=client_ip,
    )


def record_ws_disconnect(client_ip: Optional[str], duration_s: Optional[float] = None) -> None:
    record_system_event(
        category="CONNECTION",
        action="ws_disconnected",
        level="info",
        detail=f"duration_s={duration_s:.1f}" if duration_s is not None else None,
        params={"duration_s": duration_s} if duration_s is not None else None,
        client_ip=client_ip,
    )


def record_device_transition(device: str, connected: bool) -> None:
    """Edge-trigger: only call when a device flips state, not every tick."""
    record_system_event(
        category="SYSTEM",
        action="device_connected" if connected else "device_disconnected",
        level="info" if connected else "warn",
        detail=f"device={device}",
        params={"device": device, "connected": connected},
    )


def record_nav_state_change(running: bool) -> None:
    record_system_event(
        category="SYSTEM",
        action="nav2_running" if running else "nav2_stopped",
        level="info",
        detail=f"running={running}",
        params={"running": running},
    )


def record_slam_state_change(active: bool) -> None:
    record_system_event(
        category="SYSTEM",
        action="slam_active" if active else "slam_inactive",
        level="info",
        detail=f"active={active}",
        params={"active": active},
    )


def record_battery_low(percentage: float) -> None:
    record_system_event(
        category="SYSTEM",
        action="battery_low",
        level="warn",
        detail=f"pct={percentage:.1f}",
        params={"percentage": percentage},
    )


def record_battery_recovered(percentage: float) -> None:
    record_system_event(
        category="SYSTEM",
        action="battery_recovered",
        level="info",
        detail=f"pct={percentage:.1f}",
        params={"percentage": percentage},
    )


def record_nav_result(result: str, message: Optional[str] = None,
                      run_id: Optional[int] = None) -> None:
    """Record a navigation result event (SUCCEEDED / FAILED / CANCELED / ESTOP)."""
    level = "error" if result == "FAILED" else "warn" if result in ("ESTOP",) else "info"
    record_system_event(
        category="NAVIGATION",
        action="nav_result",
        level=level,
        detail=message,
        params={"result": result, "run_id": run_id},
        result=result,
    )


# ---------------------------------------------------------------------------
# Teleop session aggregation
# ---------------------------------------------------------------------------
# One event per drive session (start when velocity becomes non-zero, end when
# zero for >1 s or the client disconnects).

_teleop_lock = threading.Lock()
_teleop_sessions: dict = {}  # client_ip -> {"start": ts, "max_lin": 0, "max_ang": 0, "samples": 0, "last_nonzero": ts}


def record_teleop_tick(client_ip: Optional[str], linear: float, angular: float) -> None:
    """Called from web_server on every 'teleop' command. Aggregates a session."""
    try:
        if not db.is_enabled():
            return
        ip = client_ip or "unknown"
        now = time.time()
        v = math_hypot(linear, angular) if (linear or angular) else 0.0
        with _teleop_lock:
            sess = _teleop_sessions.get(ip)
            if v > 1e-6:
                # If there's a stale open session (no velocity for >1s), close
                # it before starting a new one.
                if sess is not None and (now - sess["last_nonzero"]) > 1.0:
                    _flush_teleop_session(ip, sess)
                    _teleop_sessions.pop(ip, None)
                    sess = None
                if sess is None:
                    sess = {
                        "start": now, "max_lin": 0.0, "max_ang": 0.0,
                        "samples": 0, "last_nonzero": now,
                    }
                    _teleop_sessions[ip] = sess
                sess["max_lin"] = max(sess["max_lin"], abs(linear))
                sess["max_ang"] = max(sess["max_ang"], abs(angular))
                sess["samples"] += 1
                sess["last_nonzero"] = now
            else:
                if sess is not None and (now - sess["last_nonzero"]) > 1.0:
                    _flush_teleop_session(ip, sess)
                    _teleop_sessions.pop(ip, None)
    except Exception:
        pass


def end_teleop_session(client_ip: Optional[str]) -> None:
    """Force-close any open session for this client (e.g. on WS disconnect)."""
    try:
        if not db.is_enabled():
            return
        ip = client_ip or "unknown"
        with _teleop_lock:
            sess = _teleop_sessions.pop(ip, None)
        if sess is not None:
            _flush_teleop_session(ip, sess)
    except Exception:
        pass


def _flush_teleop_session(ip: str, sess: dict) -> None:
    duration = max(0.0, sess["last_nonzero"] - sess["start"])
    if duration < 0.05 or sess["samples"] == 0:
        return  # ignore accidental single zero
    record_system_event(
        category="MODE",
        action="teleop_session",
        level="info",
        detail=f"client={ip} dur={duration:.1f}s "
               f"max_lin={sess['max_lin']:.2f} max_ang={sess['max_ang']:.2f}",
        params={
            "client": ip,
            "duration_s": round(duration, 3),
            "max_linear": round(sess["max_lin"], 4),
            "max_angular": round(sess["max_ang"], 4),
            "samples": sess["samples"],
        },
        client_ip=ip,
    )


def math_hypot(a: float, b: float) -> float:
    return (a * a + b * b) ** 0.5


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

_COMMAND_CATEGORY = {
    # SLAM
    "start_slam": "SLAM",
    "stop_slam": "SLAM",
    "save_map": "SLAM",
    # NAVIGATION
    "start_navigation": "NAVIGATION",
    "stop_navigation": "NAVIGATION",
    "nav_goal": "NAVIGATION",
    "nav_cancel": "NAVIGATION",
    "set_initial_pose": "NAVIGATION",
    "waypoint_mission": "MISSION",
    # MODE
    "set_op_mode": "MODE",
    # SAFETY
    "emergency_stop": "SAFETY",
    # LIFT
    "lift": "LIFT",
    # MAP / ROUTE changes go through REST (logged at the REST endpoint).
}


def _category_for_command(command: str) -> str:
    return _COMMAND_CATEGORY.get(command, "SYSTEM")


def _build_command_detail(command: str, cmd: dict) -> Optional[str]:
    """Build a short, human-readable detail string for the given command."""
    try:
        if command in ("start_navigation",):
            return f"map={cmd.get('map_id', '')} route={cmd.get('route_id', '')}"
        if command == "stop_navigation":
            return None
        if command == "nav_goal":
            pose = cmd.get("pose") or {}
            return (f"x={float(pose.get('x', 0)):.3f} "
                    f"y={float(pose.get('y', 0)):.3f} "
                    f"yaw={float(pose.get('yaw', 0)):.3f}rad")
        if command == "waypoint_mission":
            n = len(cmd.get("waypoints") or [])
            return f"n={n}"
        if command == "set_initial_pose":
            pose = cmd.get("pose") or {}
            return (f"x={float(pose.get('x', 0)):.3f} "
                    f"y={float(pose.get('y', 0)):.3f} "
                    f"yaw={float(pose.get('yaw', 0)):.3f}rad")
        if command == "save_map":
            return f"map={cmd.get('map_name', '')}"
        if command == "lift":
            return f"action={cmd.get('action', '')} target_mm={cmd.get('target_mm', 0)}"
        if command == "set_op_mode":
            return f"mode={cmd.get('mode', '')}"
        if command == "emergency_stop":
            return None
        if command in ("start_slam", "stop_slam"):
            return None
    except Exception:
        return None
    return None


# Fields we never log verbatim (high-rate or large blobs).
_DROPPED_PARAMS = {
    "image", "image_data", "pgm", "jpeg", "b64", "data",
    "depth_image", "color_image", "camera_data",
}


def _sanitize_params(cmd: dict) -> Optional[dict]:
    """Return a small JSON-safe copy of cmd, dropping large/blob fields."""
    try:
        out: dict = {}
        for k, v in cmd.items():
            if k in _DROPPED_PARAMS:
                continue
            if k == "command":
                continue
            if isinstance(v, (str, int, float, bool)) or v is None:
                out[k] = v
            elif isinstance(v, dict):
                # Keep at most a few keys of a nested dict (e.g. pose).
                inner = {ik: iv for ik, iv in v.items()
                         if isinstance(iv, (str, int, float, bool)) or iv is None}
                if inner:
                    out[k] = inner
            elif isinstance(v, list):
                # For waypoints, keep count only.
                if k == "waypoints":
                    out["waypoint_count"] = len(v)
                else:
                    out[k] = v if len(v) <= 8 else f"<list len={len(v)}>"
        return out or None
    except Exception:
        return None