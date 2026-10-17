"""
amr_web/history/runs.py
========================
Navigation-run lifecycle (HISTORY_RESULTS_PLAN.md, Phase 3).

`db.py` provides the primitives (open_run / add_waypoint / add_trajectory / close_run).
This module wraps them into a `RunSession` that remembers the run id, the goals and the
running path-length, plus a small registry of the ONE active run (there is one robot).

Conventions (all poses are dicts {"x": m, "y": m, "yaw": radians}, `map` frame):
- start pose  = latest robot pose when the goal/mission command is accepted
- goal pose   = the goal (for a waypoint mission: the LAST waypoint)
- end pose    = latest robot pose when the run ends
- straight_dist_m = |start -> goal|                       (plan section 7)
- path_length_m   = sum of steps between sampled poses; steps < 0.01 m are ignored so
                    localisation jitter does not inflate the length
- pos_error_m     = |end -> goal|
- yaw_error_deg   = wrap(end_yaw - goal_yaw), signed, in [-180, 180)
  (computed from the robot's own localisation (AMCL); the tape-measured value lives in
   measured_error_cm and is entered from the History page)
- Waypoint rows: Nav2 NavigateThroughPoses reports `number_of_poses_remaining`; when the
  waypoint index goes i -> i+1, waypoint i has just been reached (within Nav2's goal
  tolerance, so its error is the distance at the moment Nav2 accepted it, not the final
  settle error). The last waypoint is recorded when the mission SUCCEEDS.

Everything is soft-fail: a session whose run could not be opened (feature disabled) is a
valid object whose methods are cheap no-ops. All public functions swallow exceptions.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

from amr_web.history import db


_MIN_STEP_M = 0.01                 # ignore pose steps smaller than this (jitter)
_MAX_WAYPOINTS = 256
_TRAJECTORY_FLUSH_THRESHOLD = 5_000
_PENDING_CLIENT_TTL = 5.0          # seconds


def _xy(p) -> Optional[tuple]:
    """(x, y) from a pose dict or a tuple/list; None if unavailable."""
    if p is None:
        return None
    try:
        if isinstance(p, dict):
            return (float(p["x"]), float(p["y"]))
        return (float(p[0]), float(p[1]))
    except Exception:
        return None


def _dist(a, b) -> Optional[float]:
    pa, pb = _xy(a), _xy(b)
    if pa is None or pb is None:
        return None
    return math.hypot(pa[0] - pb[0], pa[1] - pb[1])


def _yaw_err_deg(end_pose, goal_pose) -> Optional[float]:
    try:
        e = end_pose.get("yaw")
        g = goal_pose.get("yaw")
        if not isinstance(e, (int, float)) or not isinstance(g, (int, float)):
            return None
        return db.wrap_deg(math.degrees(e) - math.degrees(g))
    except Exception:
        return None


class RunSession:
    """One navigation run. Create with `open_goal` / `open_waypoint`, finish with `close()`."""

    __slots__ = (
        "_id", "_type", "_map_id", "_route_id",
        "_start_pose", "_goal_pose", "_waypoint_goals",
        "_waypoints_recorded", "_trajectory_buf",
        "_t0", "_path_len", "_last_xy", "_n_samples", "_last_wp_index", "_estop",
    )

    def __init__(self):
        self._id: Optional[int] = None          # None => no-op session
        self._type: str = ""
        self._map_id: Optional[str] = None
        self._route_id: Optional[str] = None
        self._start_pose: Optional[dict] = None
        self._goal_pose: Optional[dict] = None
        self._waypoint_goals: list = []
        self._waypoints_recorded: set = set()
        self._trajectory_buf: list = []
        self._t0: float = 0.0
        self._path_len: float = 0.0
        self._last_xy: Optional[tuple] = None
        self._n_samples: int = 0
        self._last_wp_index: int = 0
        self._estop: bool = False

    def mark_estop(self) -> None:
        """The operator pressed E-stop while this run was open (see close())."""
        self._estop = True

    # ------------------------------------------------------------------ open
    @classmethod
    def open_goal(
        cls,
        goal_pose: dict,
        start_pose: Optional[dict] = None,
        map_id: Optional[str] = None,
        route_id: Optional[str] = None,
        client_ip: Optional[str] = None,
    ) -> "RunSession":
        s = cls()
        s._type = "goal"
        s._map_id = map_id
        s._route_id = route_id
        s._start_pose = start_pose
        s._goal_pose = goal_pose
        s._t0 = time.time()
        s._id = db.open_run(
            run_type="goal",
            map_id=map_id,
            route_id=route_id,
            start_pose=start_pose,
            goal_pose=goal_pose,
            client_ip=client_ip or _take_pending_client(),
        )
        return s

    @classmethod
    def open_waypoint(
        cls,
        waypoints: list,
        start_pose: Optional[dict] = None,
        map_id: Optional[str] = None,
        route_id: Optional[str] = None,
        mission_type: str = "waypoints",
        client_ip: Optional[str] = None,
    ) -> "RunSession":
        s = cls()
        s._type = mission_type if mission_type in ("waypoints", "route") else "waypoints"
        s._map_id = map_id
        s._route_id = route_id
        s._start_pose = start_pose
        s._waypoint_goals = list(waypoints or [])[:_MAX_WAYPOINTS]
        if s._waypoint_goals:
            s._goal_pose = s._waypoint_goals[-1]
        s._t0 = time.time()
        s._id = db.open_run(
            run_type=s._type,
            map_id=map_id,
            route_id=route_id,
            start_pose=start_pose,
            goal_pose=s._goal_pose,
            waypoint_goals=s._waypoint_goals,
            client_ip=client_ip or _take_pending_client(),
        )
        return s

    # ------------------------------------------------------------ trajectory
    def sample_pose(self, t: float, x: float, y: float, yaw_rad: float) -> None:
        """Record one pose sample (called ~5 Hz). `t` is wall-clock epoch seconds; it is
        stored relative to the run start. Path length is accumulated incrementally
        (it must not depend on the buffer, which is flushed in batches)."""
        if self._id is None:
            return
        if self._last_xy is None:
            self._last_xy = (x, y)
        else:
            step = math.hypot(x - self._last_xy[0], y - self._last_xy[1])
            if step >= _MIN_STEP_M:
                self._path_len += step
                self._last_xy = (x, y)
        self._n_samples += 1
        self._trajectory_buf.append((max(0.0, t - self._t0), x, y, math.degrees(yaw_rad)))
        if len(self._trajectory_buf) >= _TRAJECTORY_FLUSH_THRESHOLD:
            self._flush_trajectory()

    def _flush_trajectory(self) -> None:
        if self._id is None or not self._trajectory_buf:
            return
        db.add_trajectory(self._id, self._trajectory_buf)
        self._trajectory_buf = []

    # ------------------------------------------------------------- waypoints
    def add_waypoint(
        self,
        idx: int,
        name: Optional[str],
        goal_pose: Optional[dict],
        status: str,
        reached_pose: Optional[dict] = None,
        pos_error_m: Optional[float] = None,
        yaw_error_deg: Optional[float] = None,
        reached_at: Optional[float] = None,
    ) -> None:
        if self._id is None or idx in self._waypoints_recorded:
            return
        if len(self._waypoints_recorded) >= _MAX_WAYPOINTS:
            return
        self._waypoints_recorded.add(idx)
        db.add_waypoint(
            run_id=self._id, idx=int(idx), name=name, goal_pose=goal_pose, status=status,
            reached_at=reached_at, reached_pose=reached_pose,
            pos_error_m=pos_error_m, yaw_error_deg=yaw_error_deg,
        )

    def _record_waypoint_reached(self, idx: int, pose: Optional[dict]) -> None:
        if idx < 0 or idx >= len(self._waypoint_goals):
            return
        goal = self._waypoint_goals[idx]
        gpose = {"x": goal.get("x"), "y": goal.get("y"), "yaw": goal.get("yaw")}
        pos_err = _dist(pose, gpose) if pose else None
        yaw_err = _yaw_err_deg(pose, gpose) if pose else None
        self.add_waypoint(
            idx=idx, name=goal.get("name"), goal_pose=gpose, status="REACHED",
            reached_pose=pose, pos_error_m=pos_err, yaw_error_deg=yaw_err,
            reached_at=time.time(),
        )

    def on_waypoint_progress(self, new_index: int, pose: Optional[dict]) -> None:
        """Feed the nav_bridge `waypoint_index` (0-based, the waypoint being driven TO).
        When it increases, every waypoint before it has been reached."""
        if self._id is None or not self._waypoint_goals:
            return
        if new_index > self._last_wp_index:
            for i in range(self._last_wp_index, new_index):
                self._record_waypoint_reached(i, pose)
            self._last_wp_index = new_index

    # ----------------------------------------------------------------- close
    def close(
        self,
        result: str,
        message: Optional[str] = None,
        end_pose: Optional[dict] = None,
        pos_error_m: Optional[float] = None,
        yaw_error_deg: Optional[float] = None,
        measured_error_cm: Optional[float] = None,
    ) -> None:
        if self._id is None:
            return
        run_id = self._id
        try:
            # E-stop pressed during the run: if Nav2 then ends the run as CANCELED/FAILED
            # the result is ESTOP; if the robot resumed and still arrived it stays
            # SUCCEEDED, but the message says that E-stop was pressed.
            if self._estop:
                message = "[E-stop pressed] " + (message or "")
                if result in ("CANCELED", "FAILED"):
                    result = "ESTOP"
            # Last waypoint(s) of a mission are reached exactly when the mission succeeds.
            if result == "SUCCEEDED" and self._waypoint_goals:
                for i in range(self._last_wp_index, len(self._waypoint_goals)):
                    self._record_waypoint_reached(i, end_pose)
            # Final pose sample so the plot ends at the real end pose.
            if end_pose is not None and _xy(end_pose) is not None:
                ex, ey = _xy(end_pose)
                if self._last_xy is not None:
                    step = math.hypot(ex - self._last_xy[0], ey - self._last_xy[1])
                    if step >= _MIN_STEP_M:
                        self._path_len += step
                yaw = end_pose.get("yaw")
                self._trajectory_buf.append(
                    (max(0.0, time.time() - self._t0), ex, ey,
                     math.degrees(yaw) if isinstance(yaw, (int, float)) else 0.0))
                self._n_samples += 1
            if self._trajectory_buf:
                db.add_trajectory(run_id, self._trajectory_buf)
                self._trajectory_buf = []

            straight = _dist(self._start_pose, self._goal_pose)
            path = self._path_len if self._n_samples > 0 else None
            if pos_error_m is None and end_pose is not None and self._goal_pose is not None:
                pos_error_m = _dist(end_pose, self._goal_pose)
            if yaw_error_deg is None and end_pose is not None and self._goal_pose is not None:
                yaw_error_deg = _yaw_err_deg(end_pose, self._goal_pose)

            db.close_run(
                run_id=run_id, result=result, message=message, end_pose=end_pose,
                straight_dist_m=straight, path_length_m=path,
                duration_s=(time.time() - self._t0) if self._t0 else None,
                pos_error_m=pos_error_m, yaw_error_deg=yaw_error_deg,
            )
            if measured_error_cm is not None:
                db.update_run_meta(run_id, measured_error_cm=measured_error_cm)
        except Exception:
            pass
        finally:
            self._id = None        # any later call on this session is a no-op

    def update_meta(self, scenario: Optional[str] = None, notes: Optional[str] = None) -> None:
        if self._id is None:
            return
        db.update_run_meta(self._id, scenario=scenario, notes=notes)

    @property
    def run_id(self) -> Optional[int]:
        return self._id

    @property
    def start_pose(self) -> Optional[dict]:
        return self._start_pose

    @property
    def goal_pose(self) -> Optional[dict]:
        return self._goal_pose


# ---------------------------------------------------------------------------
# Active-run registry (one robot => at most one open run)
# ---------------------------------------------------------------------------

_active_lock = threading.Lock()
_active: Optional[RunSession] = None


def set_active(session: Optional[RunSession]) -> None:
    global _active
    with _active_lock:
        _active = session


def get_active() -> Optional[RunSession]:
    with _active_lock:
        return _active


def clear_active_if_match(session: RunSession) -> None:
    global _active
    with _active_lock:
        if _active is session:
            _active = None


def _pop_active() -> Optional[RunSession]:
    global _active
    with _active_lock:
        s, _active = _active, None
    return s


# ---- client ip hand-over -------------------------------------------------
# nav_bridge does not know which browser sent the command; web_server does. It calls
# set_pending_client() right before invoking the bridge and the next open_* consumes it.

_pending_client_lock = threading.Lock()
_pending_client: Optional[tuple] = None   # (ip, monotonic ts)


def set_pending_client(ip: Optional[str]) -> None:
    global _pending_client
    with _pending_client_lock:
        _pending_client = (ip, time.monotonic()) if ip else None


def _take_pending_client() -> Optional[str]:
    global _pending_client
    with _pending_client_lock:
        item, _pending_client = _pending_client, None
    if item is None:
        return None
    ip, ts = item
    return ip if (time.monotonic() - ts) <= _PENDING_CLIENT_TTL else None


# ---- high-level helpers used by nav_bridge / bridge_node ------------------

def finish_active_run(result: str, message: Optional[str] = None,
                      end_pose: Optional[dict] = None) -> None:
    """Close the active run (if any). Idempotent: a second call finds nothing to close.
    Never raises."""
    try:
        s = _pop_active()
        if s is not None:
            s.close(result, message, end_pose)
    except Exception:
        pass


def begin_goal_run(goal_pose: dict, start_pose: Optional[dict], map_id: Optional[str],
                   route_id: Optional[str]) -> Optional[RunSession]:
    """Open a single-goal run and make it the active one. A stale open run is
    closed as CANCELED first. Never raises."""
    try:
        finish_active_run("CANCELED", "Superseded by a new goal", start_pose)
        s = RunSession.open_goal(goal_pose=goal_pose, start_pose=start_pose,
                                 map_id=map_id, route_id=route_id)
        set_active(s)
        return s
    except Exception:
        return None


def begin_waypoint_run(waypoints: list, start_pose: Optional[dict], map_id: Optional[str],
                       route_id: Optional[str], mission_type: str) -> Optional[RunSession]:
    try:
        finish_active_run("CANCELED", "Superseded by a new mission", start_pose)
        s = RunSession.open_waypoint(waypoints=waypoints, start_pose=start_pose,
                                     map_id=map_id, route_id=route_id,
                                     mission_type=mission_type)
        set_active(s)
        return s
    except Exception:
        return None


def mark_estop() -> None:
    """Flag the active run (if any) as having seen an E-stop. Never raises."""
    try:
        s = get_active()
        if s is not None:
            s.mark_estop()
    except Exception:
        pass


def on_waypoint_progress(new_index: int, pose: Optional[dict]) -> None:
    try:
        s = get_active()
        if s is not None:
            s.on_waypoint_progress(int(new_index), pose)
    except Exception:
        pass