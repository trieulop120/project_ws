"""
amr_web/history/db.py
=====================
SQLite writer thread + schema + queries for operation history and navigation runs.

Design (HISTORY_RESULTS_PLAN.md §5):
- One writer thread consuming a queue.Queue. All inserts/updates are enqueued.
- Reads use short-lived separate connections.
- PRAGMA journal_mode=WAL, synchronous=NORMAL, foreign_keys=ON.
- Schema version via PRAGMA user_version with an ordered migration list.
- Timestamps: UTC epoch seconds (REAL). Local time derived in the API/UI layer.
- Yaw stored in degrees, normalized to [-180, 180).
- All floats stored at full precision; rounding is display-only.
- Soft fail: if init() cannot open the DB, all public methods become no-ops and
  is_enabled() returns False. The rest of the system keeps working.

Public API (kept small on purpose):
    init(), shutdown(), is_enabled()
    log_event(category, level, action, detail, params, result, client_ip) -> int | None
    update_event_result(event_id, result, detail)
    open_run(run_type, map_id, route_id, start_pose, goal_pose, scenario, notes,
             waypoint_goals, client_ip) -> int | None
    close_run(run_id, result, message, end_pose)
    add_waypoint(run_id, idx, name, goal_pose, status,
                 reached_at, reached_pose, pos_error_m, yaw_error_deg)
    add_trajectory(run_id, points)  # points: list[(t, x, y, yaw_deg)]
    update_run_meta(run_id, scenario=None, notes=None, measured_error_cm=None)
    mark_open_runs_interrupted()
    query_events(filters, page, page_size, sort) -> (rows, total)
    query_runs(filters, page, page_size, sort) -> (rows, total)
    get_run(run_id) -> dict | None
    meta() -> dict
"""

from __future__ import annotations

import json
import math
import os
import queue
import sqlite3
import threading
import time
import traceback
from typing import Any, Optional


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Schema version. Increment and add a new entry to _MIGRATIONS when changing.
SCHEMA_VERSION = 1

# Queue size bound. Old telemetry is more valuable than the newest when the
# DB is wedged, so we drop the *oldest* (not newest) on overflow.
_QUEUE_MAX = 4096

# Trajectory decimation cap per run. If a run produces more than this many
# sampled points, decimate before insert (Phase 3 calls into here).
_TRAJECTORY_CAP = 20_000

# Module-level state
_lock = threading.RLock()
_enabled = False
_writer_thread: Optional[threading.Thread] = None
_queue: "queue.Queue" = queue.Queue(maxsize=_QUEUE_MAX)
_db_path: Optional[str] = None
_conn_lock = threading.Lock()  # serialises the writer's connection use

# In-process id allocation. This process is the ONLY writer of the DB, so ids
# are handed out here (seeded from MAX(id) at init) instead of being read back
# from the writer thread. Consequences: log_event()/open_run() return a REAL id
# immediately and never block, and update_event_result()/add_waypoint() can
# reference that id safely (the writer queue is FIFO: insert always precedes update).
_id_lock = threading.Lock()
_next_event_id = 1
_next_run_id = 1


def _seed_ids(conn: sqlite3.Connection) -> None:
    """Seed the id counters from the existing tables. Call after migrations."""
    global _next_event_id, _next_run_id
    try:
        ev = conn.execute("SELECT COALESCE(MAX(id), 0) FROM events").fetchone()[0]
        rn = conn.execute("SELECT COALESCE(MAX(id), 0) FROM nav_runs").fetchone()[0]
    except Exception:
        ev, rn = 0, 0
    with _id_lock:
        _next_event_id = int(ev) + 1
        _next_run_id = int(rn) + 1


def _alloc_event_id() -> int:
    global _next_event_id
    with _id_lock:
        v = _next_event_id
        _next_event_id += 1
    return v


def _alloc_run_id() -> int:
    global _next_run_id
    with _id_lock:
        v = _next_run_id
        _next_run_id += 1
    return v


# ---------------------------------------------------------------------------
# Yaw helpers
# ---------------------------------------------------------------------------

def wrap_deg(a: float) -> float:
    """Normalize an angle in degrees to [-180, 180)."""
    return (float(a) + 180.0) % 360.0 - 180.0


def rad_to_deg(a: float) -> float:
    return math.degrees(a)


# ---------------------------------------------------------------------------
# Migrations
# ---------------------------------------------------------------------------

_MIGRATIONS = [
    # version 1: initial schema
    """
    CREATE TABLE events (
        id INTEGER PRIMARY KEY,
        ts REAL NOT NULL,
        category TEXT NOT NULL,
        level TEXT NOT NULL,
        action TEXT NOT NULL,
        detail TEXT,
        params TEXT,
        result TEXT,
        client_ip TEXT
    );
    CREATE INDEX idx_events_ts ON events(ts);
    CREATE INDEX idx_events_cat ON events(category, ts);

    CREATE TABLE nav_runs (
        id INTEGER PRIMARY KEY,
        started_at REAL NOT NULL,
        ended_at REAL,
        run_type TEXT NOT NULL,
        scenario TEXT,
        notes TEXT,
        map_id TEXT,
        route_id TEXT,
        start_x REAL, start_y REAL, start_yaw_deg REAL,
        goal_x  REAL, goal_y  REAL, goal_yaw_deg  REAL,
        end_x   REAL, end_y   REAL, end_yaw_deg   REAL,
        straight_dist_m REAL,
        path_length_m REAL,
        duration_s REAL,
        pos_error_m REAL,
        yaw_error_deg REAL,
        measured_error_cm REAL,
        result TEXT,
        message TEXT,
        waypoint_total INTEGER,
        client_ip TEXT
    );
    CREATE INDEX idx_runs_started ON nav_runs(started_at);

    CREATE TABLE nav_run_waypoints (
        run_id INTEGER NOT NULL REFERENCES nav_runs(id) ON DELETE CASCADE,
        idx INTEGER NOT NULL,
        name TEXT,
        goal_x REAL, goal_y REAL, goal_yaw_deg REAL,
        reached_at REAL,
        reached_x REAL, reached_y REAL, reached_yaw_deg REAL,
        pos_error_m REAL, yaw_error_deg REAL,
        status TEXT,
        PRIMARY KEY (run_id, idx)
    );

    CREATE TABLE nav_trajectory (
        run_id INTEGER NOT NULL REFERENCES nav_runs(id) ON DELETE CASCADE,
        t REAL NOT NULL,
        x REAL, y REAL, yaw_deg REAL
    );
    CREATE INDEX idx_traj_run ON nav_trajectory(run_id, t);
    """,
]


def _apply_migrations(conn: sqlite3.Connection) -> None:
    """Run any unapplied migrations, in order. Updates user_version."""
    cur = conn.execute("PRAGMA user_version")
    current = cur.fetchone()[0]
    if current >= SCHEMA_VERSION:
        return
    for i in range(current, SCHEMA_VERSION):
        sql = _MIGRATIONS[i]
        conn.executescript(sql)
    conn.execute(f"PRAGMA user_version = {SCHEMA_VERSION}")
    conn.commit()


# ---------------------------------------------------------------------------
# Connection helpers
# ---------------------------------------------------------------------------

def _open_writable() -> Optional[sqlite3.Connection]:
    """Open the writer's connection. None if the DB is unreachable."""
    if not _db_path:
        return None
    try:
        conn = sqlite3.connect(_db_path, timeout=5.0, check_same_thread=False)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        conn.execute("PRAGMA foreign_keys=ON")
        return conn
    except Exception:
        return None


def _open_writable_at(path: str) -> Optional[sqlite3.Connection]:
    """Open a writable connection at an explicit path (used by init_sync and
    tests so we can open before _db_path is set)."""
    try:
        conn = sqlite3.connect(path, timeout=5.0, check_same_thread=False)
        conn.execute("PRAGMA journal_mode=WAL")
        conn.execute("PRAGMA synchronous=NORMAL")
        conn.execute("PRAGMA foreign_keys=ON")
        return conn
    except Exception:
        return None


def _open_readonly() -> Optional[sqlite3.Connection]:
    """Open a short-lived read-only connection. None if the DB is unreachable."""
    if not _db_path:
        return None
    try:
        # uri=True with mode=ro gives a true read-only handle.
        uri = f"file:{_db_path}?mode=ro"
        conn = sqlite3.connect(uri, uri=True, timeout=2.0)
        conn.row_factory = sqlite3.Row
        return conn
    except Exception:
        return None


# ---------------------------------------------------------------------------
# Lifecycle
# ---------------------------------------------------------------------------

def init(db_path: Optional[str] = None) -> bool:
    """Initialise the history subsystem.

    Args:
        db_path: explicit path; if None, reads env AMR_LOG_DIR (default ~/amr_logs/).
    Returns:
        True if enabled, False if the feature has been disabled (DB not openable).
    """
    global _enabled, _writer_thread, _db_path

    with _lock:
        path = db_path or _default_db_path()
        # If a previous init succeeded for the same path and the writer is
        # still running, this is a no-op.
        if _enabled and _db_path == path and _writer_thread is not None \
                and _writer_thread.is_alive():
            return True
        # Otherwise reset any prior state and start fresh.
        if _writer_thread is not None:
            try:
                _enabled = False
                _queue.put_nowait(None)
                _writer_thread.join(timeout=1.0)
            except Exception:
                pass
            _writer_thread = None
        _enabled = False
        _db_path = None
        # Drain any leftover items (sentinels, jobs from a prior path) so the
        # new writer does not exit on the first item.
        try:
            while True:
                _queue.get_nowait()
        except queue.Empty:
            pass

        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
        except Exception:
            return False

        # Try to open once and apply migrations before starting the writer.
        conn = _open_writable_at(path)
        if conn is None:
            return False
        try:
            _apply_migrations(conn)
            _seed_ids(conn)
        except Exception:
            try:
                conn.close()
            except Exception:
                pass
            return False
        try:
            conn.close()
        except Exception:
            pass

        _db_path = path
        _enabled = True
        _writer_thread = threading.Thread(
            target=_writer_loop, name="amr-history-writer", daemon=True
        )
        _writer_thread.start()

    # Side effect: log a backend_started event. Enqueue is safe even before init
    # returns because the writer thread is already running.
    try:
        log_event(
            category="SYSTEM", level="info", action="backend_started",
            detail=f"db={path}",
            params={"amr_log_dir": os.environ.get("AMR_LOG_DIR", "")},
            result="success",
        )
    except Exception:
        pass

    return True


def shutdown(timeout: float = 2.0) -> None:
    """Stop the writer thread, flushing remaining work. Safe to call twice."""
    global _enabled, _writer_thread
    with _lock:
        if not _enabled:
            return
        # Mark open runs as INTERRUPTED (best effort).
        try:
            _mark_open_runs_interrupted_now()
        except Exception:
            pass
        _enabled = False
        # Sentinel to wake the writer.
        try:
            _queue.put_nowait(None)
        except queue.Full:
            pass
        if _writer_thread is not None:
            _writer_thread.join(timeout=timeout)
            _writer_thread = None


def is_enabled() -> bool:
    with _lock:
        return _enabled


def db_path() -> Optional[str]:
    return _db_path


def _default_db_path() -> str:
    log_dir = os.environ.get("AMR_LOG_DIR", os.path.expanduser("~/amr_logs"))
    return os.path.join(log_dir, "amr_history.db")


# ---------------------------------------------------------------------------
# Writer thread
# ---------------------------------------------------------------------------

def _writer_loop() -> None:
    """Single-writer thread. Pulls jobs from _queue and runs them."""
    conn = _open_writable()
    if conn is None:
        # If we lost the DB mid-run, drain the queue without writing and exit.
        while True:
            try:
                _queue.get(timeout=0.2)
            except queue.Empty:
                return
            except Exception:
                return
    try:
        while True:
            try:
                job = _queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if job is None:  # sentinel
                break
            kind, args, kwargs = job
            try:
                with _conn_lock:
                    _run_job(conn, kind, args, kwargs)
                    conn.commit()
            except Exception:
                # Never let a single bad write kill the writer. Move on.
                # (intentionally not logged: history must not spam logs)
                continue
    finally:
        try:
            conn.close()
        except Exception:
            pass


def _run_job(conn: sqlite3.Connection, kind: str, args, kwargs) -> None:
    if kind == "log_event":
        conn.execute(
            "INSERT INTO events "
            "(id, ts, category, level, action, detail, params, result, client_ip) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
            args,
        )
    elif kind == "update_event_result":
        # args = (result, msg, msg, msg, event_id). The command_response message is
        # APPENDED to the existing detail ("x=1 y=2 | Goal accepted"), never replaces it.
        conn.execute(
            "UPDATE events SET result = ?, detail = CASE "
            "  WHEN ? IS NULL THEN detail "
            "  WHEN detail IS NULL THEN ? "
            "  ELSE detail || ' | ' || ? END "
            "WHERE id = ?",
            args,
        )
    elif kind == "open_run":
        conn.execute(
            "INSERT INTO nav_runs "
            "(id, started_at, run_type, scenario, notes, map_id, route_id, "
            " start_x, start_y, start_yaw_deg, goal_x, goal_y, goal_yaw_deg, "
            " waypoint_total, client_ip) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            args,
        )
    elif kind == "close_run":
        conn.execute(
            "UPDATE nav_runs SET ended_at=?, result=?, message=?, "
            "end_x=?, end_y=?, end_yaw_deg=?, "
            "straight_dist_m=?, path_length_m=?, duration_s=?, "
            "pos_error_m=?, yaw_error_deg=? "
            "WHERE id = ?",
            args,
        )
    elif kind == "add_waypoint":
        conn.execute(
            "INSERT OR REPLACE INTO nav_run_waypoints "
            "(run_id, idx, name, goal_x, goal_y, goal_yaw_deg, "
            " reached_at, reached_x, reached_y, reached_yaw_deg, "
            " pos_error_m, yaw_error_deg, status) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            args,
        )
    elif kind == "add_trajectory":
        conn.executemany(
            "INSERT INTO nav_trajectory (run_id, t, x, y, yaw_deg) "
            "VALUES (?, ?, ?, ?, ?)",
            args,
        )
    elif kind == "update_run_meta":
        conn.execute(
            "UPDATE nav_runs SET "
            "  scenario = COALESCE(?, scenario), "
            "  notes = COALESCE(?, notes), "
            "  measured_error_cm = COALESCE(?, measured_error_cm) "
            "WHERE id = ?",
            args,
        )
    elif kind == "mark_open_runs_interrupted":
        conn.execute(
            "UPDATE nav_runs SET result = 'INTERRUPTED', "
            "ended_at = COALESCE(ended_at, ?) "
            "WHERE ended_at IS NULL",
            (time.time(),),
        )


def _enqueue(kind: str, args, kwargs: Optional[dict] = None) -> bool:
    """Put a job on the writer queue. Drops oldest on overflow. Returns False
    if the subsystem is disabled."""
    if not is_enabled():
        return False
    try:
        _queue.put_nowait((kind, args, kwargs or {}))
        return True
    except queue.Full:
        # Drop the oldest item to make room.
        try:
            _queue.get_nowait()
        except queue.Empty:
            pass
        try:
            _queue.put_nowait((kind, args, kwargs or {}))
            return True
        except queue.Full:
            return False


# ---------------------------------------------------------------------------
# Write API
# ---------------------------------------------------------------------------

def log_event(
    category: str,
    level: str,
    action: str,
    detail: Optional[str] = None,
    params: Optional[dict] = None,
    result: Optional[str] = None,
    client_ip: Optional[str] = None,
) -> Optional[int]:
    """Enqueue an event row (non-blocking). Returns the row id, or None when the
    feature is disabled. The id is allocated in-process, so it can be used
    immediately with update_event_result()."""
    if not is_enabled():
        return None
    eid = _alloc_event_id()
    payload = (
        eid,
        time.time(),
        category,
        level,
        action,
        detail,
        json.dumps(params) if params is not None else None,
        result or "requested",
        client_ip,
    )
    if not _enqueue("log_event", payload):
        return None
    return eid


def update_event_result(event_id: int, result: str, detail: Optional[str] = None) -> None:
    """Set the final result of an event and append `detail` (if any) to its detail text."""
    _enqueue("update_event_result", (result, detail, detail, detail, event_id))


def open_run(
    run_type: str,
    map_id: Optional[str],
    route_id: Optional[str],
    start_pose: Optional[dict],
    goal_pose: Optional[dict],
    scenario: Optional[str] = None,
    notes: Optional[str] = None,
    waypoint_goals: Optional[list] = None,
    client_ip: Optional[str] = None,
) -> Optional[int]:
    """Enqueue an open_run (non-blocking). Returns the new run id (allocated
    in-process) or None when the feature is disabled."""
    if not is_enabled():
        return None
    sx, sy, syaw = _pose_triple(start_pose)
    gx, gy, gyaw = _pose_triple(goal_pose)
    run_id = _alloc_run_id()
    payload = (
        run_id,
        time.time(),
        run_type,
        scenario,
        notes,
        map_id,
        route_id,
        sx, sy, syaw,
        gx, gy, gyaw,
        len(waypoint_goals) if waypoint_goals else None,
        client_ip,
    )
    if not _enqueue("open_run", payload):
        return None
    return run_id


def close_run(
    run_id: int,
    result: str,
    message: Optional[str],
    end_pose: Optional[dict],
    straight_dist_m: Optional[float] = None,
    path_length_m: Optional[float] = None,
    duration_s: Optional[float] = None,
    pos_error_m: Optional[float] = None,
    yaw_error_deg: Optional[float] = None,
) -> None:
    ex, ey, eyaw = _pose_triple(end_pose)
    _enqueue(
        "close_run",
        (
            time.time(), result, message,
            ex, ey, eyaw,
            straight_dist_m, path_length_m, duration_s,
            pos_error_m, yaw_error_deg,
            run_id,
        ),
    )


def add_waypoint(
    run_id: int,
    idx: int,
    name: Optional[str],
    goal_pose: Optional[dict],
    status: str,
    reached_at: Optional[float] = None,
    reached_pose: Optional[dict] = None,
    pos_error_m: Optional[float] = None,
    yaw_error_deg: Optional[float] = None,
) -> None:
    gx, gy, gyaw = _pose_triple(goal_pose)
    rx, ry, ryaw = _pose_triple(reached_pose)
    _enqueue(
        "add_waypoint",
        (
            run_id, idx, name,
            gx, gy, gyaw,
            reached_at, rx, ry, ryaw,
            pos_error_m, yaw_error_deg,
            status,
        ),
    )


def add_trajectory(run_id: int, points) -> None:
    """Add trajectory points. Decimates to _TRAJECTORY_CAP.

    points: iterable of (t, x, y, yaw_deg). yaw_deg is already in degrees.
    """
    points = list(points)
    if not points:
        return
    if len(points) > _TRAJECTORY_CAP:
        step = max(1, len(points) // _TRAJECTORY_CAP)
        points = points[::step]
    rows = [(run_id, t, x, y, yd) for (t, x, y, yd) in points]
    _enqueue("add_trajectory", rows)


def update_run_meta(
    run_id: int,
    scenario: Optional[str] = None,
    notes: Optional[str] = None,
    measured_error_cm: Optional[float] = None,
) -> None:
    _enqueue(
        "update_run_meta",
        (scenario, notes, measured_error_cm, run_id),
    )


def mark_open_runs_interrupted() -> None:
    """Called at backend start. Closes any run that was left open by a
    previous backend instance."""
    _enqueue("mark_open_runs_interrupted", ())


def _mark_open_runs_interrupted_now() -> None:
    """Synchronous variant for shutdown()."""
    if not _db_path:
        return
    conn = _open_writable_at(_db_path)
    if conn is None:
        return
    try:
        conn.execute(
            "UPDATE nav_runs SET result = 'INTERRUPTED', "
            "ended_at = COALESCE(ended_at, ?) "
            "WHERE ended_at IS NULL",
            (time.time(),),
        )
        conn.commit()
    finally:
        try:
            conn.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Pose helper
# ---------------------------------------------------------------------------

def _pose_triple(pose: Optional[dict]):
    """Return (x, y, yaw_deg) from a pose dict like {x, y, yaw}. yaw is
    converted from radians to degrees. None in -> (None, None, None) out."""
    if pose is None:
        return (None, None, None)
    x = pose.get("x")
    y = pose.get("y")
    yaw = pose.get("yaw")
    if isinstance(yaw, (int, float)):
        yaw = rad_to_deg(yaw)
        yaw = wrap_deg(yaw)
    return (x, y, yaw)


# ---------------------------------------------------------------------------
# Read API
# ---------------------------------------------------------------------------

def query_events(
    filters: Optional[dict] = None,
    page: int = 1,
    page_size: int = 50,
    sort: str = "desc",
) -> tuple[list, int]:
    filters = filters or {}
    page = max(1, int(page))
    page_size = max(1, min(200, int(page_size)))
    sort = "asc" if str(sort).lower() == "asc" else "desc"

    where, params = _build_event_where(filters)
    order = "ASC" if sort == "asc" else "DESC"
    offset = (page - 1) * page_size

    conn = _open_readonly()
    if conn is None:
        return [], 0
    try:
        total = conn.execute(
            f"SELECT COUNT(*) FROM events {where}", params
        ).fetchone()[0]
        rows = conn.execute(
            f"SELECT id, ts, category, level, action, detail, params, result, client_ip "
            f"FROM events {where} ORDER BY ts {order}, id {order} "
            f"LIMIT ? OFFSET ?",
            (*params, page_size, offset),
        ).fetchall()
        out = [dict(r) for r in rows]
        for r in out:
            if r.get("params"):
                try:
                    r["params"] = json.loads(r["params"])
                except Exception:
                    pass
        return out, int(total)
    finally:
        try:
            conn.close()
        except Exception:
            pass


def query_runs(
    filters: Optional[dict] = None,
    page: int = 1,
    page_size: int = 50,
    sort: str = "desc",
) -> tuple[list, int]:
    filters = filters or {}
    page = max(1, int(page))
    page_size = max(1, min(200, int(page_size)))
    sort = "asc" if str(sort).lower() == "asc" else "desc"

    where, params = _build_run_where(filters)
    order = "ASC" if sort == "asc" else "DESC"
    offset = (page - 1) * page_size

    conn = _open_readonly()
    if conn is None:
        return [], 0
    try:
        total = conn.execute(
            f"SELECT COUNT(*) FROM nav_runs {where}", params
        ).fetchone()[0]
        rows = conn.execute(
            f"SELECT * FROM nav_runs {where} ORDER BY started_at {order}, id {order} "
            f"LIMIT ? OFFSET ?",
            (*params, page_size, offset),
        ).fetchall()
        return [dict(r) for r in rows], int(total)
    finally:
        try:
            conn.close()
        except Exception:
            pass


def get_run(run_id: int) -> Optional[dict]:
    """Return a dict with summary, waypoints, and decimated trajectory."""
    conn = _open_readonly()
    if conn is None:
        return None
    try:
        run = conn.execute(
            "SELECT * FROM nav_runs WHERE id = ?", (run_id,)
        ).fetchone()
        if run is None:
            return None
        out = dict(run)
        out["waypoints"] = [
            dict(r) for r in conn.execute(
                "SELECT * FROM nav_run_waypoints WHERE run_id = ? ORDER BY idx",
                (run_id,),
            ).fetchall()
        ]
        # Decimate trajectory to <= 500 points for plotting.
        traj = conn.execute(
            "SELECT t, x, y, yaw_deg FROM nav_trajectory WHERE run_id = ? "
            "ORDER BY t",
            (run_id,),
        ).fetchall()
        traj = [dict(r) for r in traj]
        if len(traj) > 500:
            step = max(1, len(traj) // 500)
            traj = traj[::step]
        out["trajectory"] = traj
        return out
    finally:
        try:
            conn.close()
        except Exception:
            pass


def meta() -> dict:
    """Distinct categories, levels, results, map_ids, run_types; first/last ts."""
    conn = _open_readonly() if is_enabled() else None
    if conn is None:
        return {
            "enabled": False,
            "categories": [], "levels": [], "results": [],
            "map_ids": [], "run_types": [], "clients": [],
            "first_ts": None, "last_ts": None,
            "runs_first_ts": None, "runs_last_ts": None,
        }
    try:
        def distinct(col, table):
            return [r[0] for r in conn.execute(
                f"SELECT DISTINCT {col} FROM {table} "
                f"WHERE {col} IS NOT NULL AND {col} != '' "
                f"ORDER BY {col}"
            ).fetchall()]
        cats = distinct("category", "events")
        levels = distinct("level", "events")
        ev_results = distinct("result", "events")
        run_results = distinct("result", "nav_runs")
        map_ids = distinct("map_id", "nav_runs")
        run_types = distinct("run_type", "nav_runs")
        # Combine result vocabularies.
        results = sorted(set(ev_results) | set(run_results))
        first = conn.execute("SELECT MIN(ts) FROM events").fetchone()[0]
        last = conn.execute("SELECT MAX(ts) FROM events").fetchone()[0]
        rfirst = conn.execute("SELECT MIN(started_at) FROM nav_runs").fetchone()[0]
        rlast = conn.execute("SELECT MAX(started_at) FROM nav_runs").fetchone()[0]
        clients = sorted(set(distinct("client_ip", "events")) | set(distinct("client_ip", "nav_runs")))
        return {
            "enabled": True,
            "categories": cats,
            "levels": levels,
            "results": results,
            "map_ids": map_ids,
            "run_types": run_types,
            "clients": clients,
            "first_ts": first,
            "last_ts": last,
            "runs_first_ts": rfirst,
            "runs_last_ts": rlast,
        }
    finally:
        try:
            conn.close()
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Bulk read helpers (Excel export). No paging; hard caps protect memory.
# ---------------------------------------------------------------------------

_EXPORT_MAX_EVENTS = 200_000
_EXPORT_MAX_RUNS = 20_000
_EXPORT_MAX_TRAJ_PER_RUN = 2_000


def _fetch_all(sql: str, params) -> list:
    conn = _open_readonly()
    if conn is None:
        return []
    try:
        return [dict(r) for r in conn.execute(sql, params).fetchall()]
    finally:
        try:
            conn.close()
        except Exception:
            pass


def fetch_events_all(filters: Optional[dict] = None, sort: str = "asc",
                     limit: int = _EXPORT_MAX_EVENTS) -> list:
    where, params = _build_event_where(filters or {})
    order = "ASC" if str(sort).lower() == "asc" else "DESC"
    return _fetch_all(
        f"SELECT id, ts, category, level, action, detail, params, result, client_ip "
        f"FROM events {where} ORDER BY ts {order}, id {order} LIMIT ?",
        (*params, int(limit)),
    )


def fetch_runs_all(filters: Optional[dict] = None, sort: str = "asc",
                   limit: int = _EXPORT_MAX_RUNS) -> list:
    where, params = _build_run_where(filters or {})
    order = "ASC" if str(sort).lower() == "asc" else "DESC"
    return _fetch_all(
        f"SELECT * FROM nav_runs {where} ORDER BY started_at {order}, id {order} LIMIT ?",
        (*params, int(limit)),
    )


def fetch_waypoints_all(filters: Optional[dict] = None) -> list:
    """Waypoint rows of every run matching the run filters (sub-select: no IN-list limit)."""
    where, params = _build_run_where(filters or {})
    return _fetch_all(
        f"SELECT * FROM nav_run_waypoints WHERE run_id IN "
        f"(SELECT id FROM nav_runs {where}) ORDER BY run_id, idx",
        tuple(params),
    )


def fetch_trajectory_all(filters: Optional[dict] = None,
                         max_points_per_run: int = _EXPORT_MAX_TRAJ_PER_RUN) -> list:
    """Trajectory rows of every run matching the run filters, decimated per run."""
    where, params = _build_run_where(filters or {})
    rows = _fetch_all(
        f"SELECT run_id, t, x, y, yaw_deg FROM nav_trajectory WHERE run_id IN "
        f"(SELECT id FROM nav_runs {where}) ORDER BY run_id, t",
        tuple(params),
    )
    out: list = []
    i, n = 0, len(rows)
    while i < n:
        j = i
        rid = rows[i]["run_id"]
        while j < n and rows[j]["run_id"] == rid:
            j += 1
        grp = rows[i:j]
        if len(grp) > max_points_per_run:
            step = -(-len(grp) // max_points_per_run)      # ceil
            keep = grp[::step]
            if keep[-1] is not grp[-1]:
                keep.append(grp[-1])                        # always keep the end point
            grp = keep
        out.extend(grp)
        i = j
    return out


# ---------------------------------------------------------------------------
# Filter builders
# ---------------------------------------------------------------------------

def _build_event_where(f: dict) -> tuple[str, list]:
    clauses = []
    params: list = []
    if f.get("from"):
        clauses.append("ts >= ?")
        params.append(float(f["from"]))
    if f.get("to"):
        clauses.append("ts <= ?")
        params.append(float(f["to"]))
    if f.get("category"):
        clauses.append("category = ?")
        params.append(str(f["category"]))
    if f.get("level"):
        clauses.append("level = ?")
        params.append(str(f["level"]))
    if f.get("result"):
        clauses.append("result = ?")
        params.append(str(f["result"]))
    if f.get("client"):
        clauses.append("client_ip = ?")
        params.append(str(f["client"]))
    if f.get("q"):
        clauses.append("(action LIKE ? OR detail LIKE ?)")
        q = f"%{f['q']}%"
        params.extend([q, q])
    where = "WHERE " + " AND ".join(clauses) if clauses else ""
    return where, params


def _build_run_where(f: dict) -> tuple[str, list]:
    clauses = []
    params: list = []
    if f.get("run_id"):
        clauses.append("id = ?")
        params.append(int(f["run_id"]))
    if f.get("from"):
        clauses.append("started_at >= ?")
        params.append(float(f["from"]))
    if f.get("to"):
        clauses.append("started_at <= ?")
        params.append(float(f["to"]))
    if f.get("type"):
        clauses.append("run_type = ?")
        params.append(str(f["type"]))
    if f.get("result"):
        clauses.append("result = ?")
        params.append(str(f["result"]))
    if f.get("map_id"):
        clauses.append("map_id = ?")
        params.append(str(f["map_id"]))
    if f.get("scenario"):
        clauses.append("scenario LIKE ?")
        params.append(f"%{f['scenario']}%")
    if f.get("client"):
        clauses.append("client_ip = ?")
        params.append(str(f["client"]))
    if f.get("q"):
        clauses.append("(scenario LIKE ? OR notes LIKE ? OR map_id LIKE ? OR route_id LIKE ?)")
        q = f"%{f['q']}%"
        params.extend([q, q, q, q])
    where = "WHERE " + " AND ".join(clauses) if clauses else ""
    return where, params


# ---------------------------------------------------------------------------
# Test seam: synchronous init for unit tests
# ---------------------------------------------------------------------------

def init_sync(db_path: Optional[str] = None) -> bool:
    """Like init() but the writer thread is bypassed for unit tests. Reads
    and writes go through the same connection. Used only by tests."""
    global _enabled, _db_path
    with _lock:
        # Reset any prior state (test isolation).
        _enabled = False
        _db_path = None
        path = db_path or _default_db_path()
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
        except Exception:
            return False
        conn = _open_writable_at(path)
        if conn is None:
            return False
        try:
            _apply_migrations(conn)
            _seed_ids(conn)
        except Exception:
            try:
                conn.close()
            except Exception:
                pass
            return False
        try:
            conn.close()
        except Exception:
            pass
        _db_path = path
        _enabled = True
        return True


def _direct_conn() -> Optional[sqlite3.Connection]:
    """For tests only. Returns a fresh connection that bypasses the writer."""
    if not _db_path:
        return None
    conn = _open_writable_at(_db_path)
    if conn is None:
        return None
    conn.row_factory = sqlite3.Row
    return conn