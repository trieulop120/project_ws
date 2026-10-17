"""
amr_web/history/tests/test_db.py
================================
Phase 1 acceptance tests for db.py. Run as:

    cd ~/project_ws
    python3 src/amr_web/amr_web/history/tests/test_db.py

Plain python3, no pytest dependency. Exit code 0 on success, non-zero on failure.
"""

from __future__ import annotations

import math
import os
import shutil
import sys
import tempfile
import time
import traceback

# Make the package importable when running this file directly.
HERE = os.path.dirname(os.path.abspath(__file__))
PKG_PARENT = os.path.abspath(os.path.join(HERE, "../../.."))
sys.path.insert(0, PKG_PARENT)

from amr_web.history import db  # noqa: E402


_FAILURES: list = []


def expect(cond, msg):
    if not cond:
        _FAILURES.append(msg)
        print(f"  FAIL: {msg}")
    else:
        print(f"  ok:   {msg}")


def fresh_db() -> str:
    """Fresh temp dir + path. Resets module state so tests are independent."""
    db.shutdown()
    tmp = tempfile.mkdtemp(prefix="amr_history_test_")
    return os.path.join(tmp, "amr_history.db")


# ---------------------------------------------------------------------------
# Test cases
# ---------------------------------------------------------------------------

def test_yaw_wrap():
    print("[test_yaw_wrap]")
    expect(db.wrap_deg(0) == 0, "0 -> 0")
    expect(db.wrap_deg(180) == -180, "180 -> -180")
    expect(db.wrap_deg(360) == 0, "360 -> 0")
    expect(db.wrap_deg(-180) == -180, "-180 -> -180")
    # 540 = 360 + 180, so 180 - 360 = -180. wrap(540) = -180, not 0.
    expect(db.wrap_deg(540) == -180, "540 -> -180")
    expect(db.wrap_deg(720) == 0, "720 -> 0")
    # 181 -> -179
    expect(db.wrap_deg(181) == -179, "181 -> -179")
    # The example from the plan: 359 vs 1 -> -2 (because we wrap end - start)
    a = 1.0
    b = 359.0
    expect(abs(db.wrap_deg(b - a) - (-2.0)) < 1e-9, "wrap(359 - 1) == -2")
    a = -179.0
    b = 179.0
    expect(abs(db.wrap_deg(b - a) - (-2.0)) < 1e-9, "wrap(179 - -179) == -2")
    expect(abs(db.wrap_deg(a - b) - 2.0) < 1e-9, "wrap(-179 - 179) == 2")


def test_schema_creation_and_init():
    print("[test_schema_creation_and_init]")
    path = fresh_db()
    try:
        ok = db.init_sync(db_path=path)
        expect(ok, "init_sync returns True")
        expect(db.is_enabled(), "is_enabled True")
        expect(db.db_path() == path, "db_path matches")
        # Confirm tables exist
        conn = db._direct_conn()
        expect(conn is not None, "_direct_conn returns connection")
        rows = conn.execute(
            "SELECT name FROM sqlite_master WHERE type='table' ORDER BY name"
        ).fetchall()
        names = [r["name"] for r in rows]
        for t in ("events", "nav_runs", "nav_run_waypoints", "nav_trajectory"):
            expect(t in names, f"table {t} exists")
        # user_version
        v = conn.execute("PRAGMA user_version").fetchone()[0]
        expect(v == db.SCHEMA_VERSION, f"user_version == {db.SCHEMA_VERSION}")
        conn.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_init_failure_disables_feature():
    print("[test_init_failure_disables_feature]")
    # We can't easily make a directory unwritable in this environment (we may
    # be root, or the FS may not honour mode bits). Instead test the second
    # soft-fail path: open a non-existent path inside a non-existent parent
    # directory. os.makedirs(..., exist_ok=False) is what init() uses, and we
    # have to ensure it raises. We use a path with a NUL byte which sqlite
    # cannot open.
    fresh_db()  # reset state
    bad_path = "/nonexistent_dir_with_nul_\x00.db"
    ok = db.init_sync(db_path=bad_path)
    expect(not ok, "init_sync returns False on invalid path")
    eid = db.log_event(category="SYSTEM", level="info", action="x")
    expect(eid is None, "log_event returns None when disabled")
    rid = db.open_run(run_type="goal", map_id=None, route_id=None,
                      start_pose=None, goal_pose=None)
    expect(rid is None, "open_run returns None when disabled")
    events, total = db.query_events()
    expect(total == 0, "query_events returns empty when disabled")
    # We must not leave the module in a broken state for subsequent tests.
    db.shutdown()


def test_event_insert_and_query():
    print("[test_event_insert_and_query]")
    path = fresh_db()
    try:
        # Ensure no prior module state from a previous test
        db.shutdown()
        ok = db.init_sync(db_path=path)
        expect(ok, f"init_sync returns True (path={path})")
        expect(db.is_enabled(), "is_enabled True")
        # Bypass the writer and write directly for sync semantics in tests.
        conn = db._direct_conn()
        expect(conn is not None, f"_direct_conn returns connection (path={path})")
        now = time.time()
        rows = [
            ("NAVIGATION", "info", "nav_goal", "x=1.0 y=2.0", '{"x":1.0}',
             "success", "127.0.0.1", now - 30),
            ("SLAM", "info", "start_slam", None, None, "success",
             "127.0.0.1", now - 20),
            ("SLAM", "warn", "stop_slam", "timed out", None, "failed",
             "127.0.0.1", now - 10),
            ("SAFETY", "warn", "emergency_stop", None, None, "success",
             "10.0.0.5", now - 5),
            ("SYSTEM", "info", "backend_started", f"db={path}", None,
             "success", None, now),
        ]
        for cat, lvl, act, det, params, res, ip, ts in rows:
            conn.execute(
                "INSERT INTO events (ts, category, level, action, detail, "
                "params, result, client_ip) VALUES (?, ?, ?, ?, ?, ?, ?, ?)",
                (ts, cat, lvl, act, det, params, res, ip),
            )
        conn.commit()
        conn.close()
        rows, total = db.query_events()
        expect(total == 5, f"query_events total = 5, got {total}")
        expect(len(rows) == 5, f"query_events rows = 5, got {len(rows)}")
        # Newest first
        expect(rows[0]["action"] == "backend_started", "newest first by default")
        # Filter by category
        slam, total = db.query_events(filters={"category": "SLAM"})
        expect(total == 2, f"SLAM events = 2, got {total}")
        # Filter by level
        warns, total = db.query_events(filters={"level": "warn"})
        expect(total == 2, f"warn events = 2, got {total}")
        # Filter by client
        rows, total = db.query_events(filters={"client": "10.0.0.5"})
        expect(total == 1, "filter by client returns 1")
        # Text search
        rows, total = db.query_events(filters={"q": "stop"})
        expect(total >= 1, "text search 'stop' matches >= 1")
        # Pagination
        rows, total = db.query_events(page=1, page_size=2)
        expect(len(rows) == 2 and total == 5, "page_size=2 returns 2 of 5")
        rows, total = db.query_events(page=3, page_size=2)
        expect(len(rows) == 1, "last page returns 1 of 5")
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_run_lifecycle():
    print("[test_run_lifecycle]")
    path = fresh_db()
    try:
        db.init_sync(db_path=path)
        conn = db._direct_conn()
        start = db.rad_to_deg(0.0)
        # Manually insert a run (we can't go through open_run() in init_sync mode
        # because the reply queue waits on the writer thread which is not running).
        cur = conn.execute(
            "INSERT INTO nav_runs (started_at, run_type, scenario, notes, "
            "map_id, route_id, start_x, start_y, start_yaw_deg, "
            "goal_x, goal_y, goal_yaw_deg, waypoint_total, client_ip) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (time.time(), "goal", "test1", "first run",
             "map_01", None,
             0.0, 0.0, db.wrap_deg(0.0),
             1.5, 2.5, db.wrap_deg(math.radians(90)),
             None, "127.0.0.1"),
        )
        run_id = cur.lastrowid
        # Waypoint
        conn.execute(
            "INSERT INTO nav_run_waypoints "
            "(run_id, idx, name, goal_x, goal_y, goal_yaw_deg, "
            " reached_at, reached_x, reached_y, reached_yaw_deg, "
            " pos_error_m, yaw_error_deg, status) "
            "VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)",
            (run_id, 0, "P_1",
             1.5, 2.5, db.wrap_deg(90.0),
             time.time(), 1.45, 2.55, db.wrap_deg(88.0),
             math.hypot(0.05, 0.05), db.wrap_deg(88.0 - 90.0),
             "REACHED"),
        )
        # Trajectory
        for i in range(100):
            t = i * 0.1
            x = 0.015 * i
            y = 0.025 * i
            conn.execute(
                "INSERT INTO nav_trajectory (run_id, t, x, y, yaw_deg) "
                "VALUES (?, ?, ?, ?, ?)",
                (run_id, t, x, y, 0.0),
            )
        # Close
        conn.execute(
            "UPDATE nav_runs SET ended_at=?, result=?, message=?, "
            "end_x=?, end_y=?, end_yaw_deg=?, "
            "straight_dist_m=?, path_length_m=?, duration_s=?, "
            "pos_error_m=?, yaw_error_deg=? WHERE id = ?",
            (time.time(), "SUCCEEDED", "ok",
             1.5, 2.5, db.wrap_deg(math.radians(90)),
             math.hypot(1.5, 2.5), 3.456, 10.0,
             0.0, 0.0, run_id),
        )
        conn.commit()
        conn.close()
        # get_run
        run = db.get_run(run_id)
        expect(run is not None, "get_run returns the run")
        expect(run["result"] == "SUCCEEDED", "result == SUCCEEDED")
        expect(len(run["waypoints"]) == 1, "1 waypoint row")
        expect(len(run["trajectory"]) == 100, "100 trajectory points (no decimation)")
        # query_runs
        rows, total = db.query_runs()
        expect(total == 1, "query_runs total = 1")
        # Filter
        rows, total = db.query_runs(filters={"map_id": "map_01"})
        expect(total == 1, "filter map_id returns 1")
        rows, total = db.query_runs(filters={"map_id": "other"})
        expect(total == 0, "filter map_id mismatch returns 0")
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_meta():
    print("[test_meta]")
    path = fresh_db()
    try:
        db.init_sync(db_path=path)
        conn = db._direct_conn()
        now = time.time()
        conn.executemany(
            "INSERT INTO events (ts, category, level, action, result) "
            "VALUES (?, ?, ?, ?, ?)",
            [
                (now - 10, "NAVIGATION", "info", "nav_goal", "success"),
                (now - 5, "SLAM", "info", "start_slam", "success"),
                (now, "SYSTEM", "info", "backend_started", "success"),
            ],
        )
        conn.executemany(
            "INSERT INTO nav_runs (started_at, run_type, map_id, result) "
            "VALUES (?, ?, ?, ?)",
            [
                (now - 8, "goal", "map_01", "SUCCEEDED"),
                (now - 4, "waypoint", "map_02", "FAILED"),
            ],
        )
        conn.commit()
        conn.close()
        m = db.meta()
        expect(m["enabled"] is True, "meta.enabled = True")
        expect("NAVIGATION" in m["categories"], "categories include NAVIGATION")
        expect("SLAM" in m["categories"], "categories include SLAM")
        expect("SYSTEM" in m["categories"], "categories include SYSTEM")
        expect("info" in m["levels"], "levels include info")
        expect("SUCCEEDED" in m["results"], "results include SUCCEEDED")
        expect("FAILED" in m["results"], "results include FAILED")
        expect(set(m["map_ids"]) == {"map_01", "map_02"},
               f"map_ids = {{map_01, map_02}}, got {set(m['map_ids'])}")
        expect(set(m["run_types"]) == {"goal", "waypoint"},
               "run_types = {goal, waypoint}")
        expect(m["first_ts"] is not None, "first_ts is set")
        expect(m["last_ts"] >= m["first_ts"], "last_ts >= first_ts")
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_yaw_storage_uses_degrees():
    print("[test_yaw_storage_uses_degrees]")
    path = fresh_db()
    try:
        db.init_sync(db_path=path)
        # open_run() needs the writer thread; use it in async mode.
        db.shutdown()
        # Re-init in async mode for this single test.
        ok = db.init(db_path=path)
        expect(ok, "init() (async) returns True")
        rid = db.open_run(
            run_type="goal",
            map_id="map_01",
            route_id=None,
            start_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
            goal_pose={"x": 1.0, "y": 2.0, "yaw": math.pi / 2},  # 90 deg
        )
        expect(rid is not None and rid > 0, f"open_run returns id, got {rid}")
        # Wait for writer to commit.
        time.sleep(0.2)
        db.shutdown()
        # Now read sync
        conn = db._direct_conn()
        row = conn.execute(
            "SELECT start_x, start_y, start_yaw_deg, goal_x, goal_y, goal_yaw_deg "
            "FROM nav_runs WHERE id = ?", (rid,)
        ).fetchone()
        expect(row["goal_yaw_deg"] == 90.0, f"goal_yaw_deg = 90, got {row['goal_yaw_deg']}")
        # Wrap: 270 deg stored as -90
        conn.execute("DELETE FROM nav_runs")
        conn.commit()
        conn.close()
        # Restart writer for second insert
        db.init(db_path=path)
        rid2 = db.open_run(
            run_type="goal", map_id="m", route_id=None,
            start_pose={"x": 0, "y": 0, "yaw": 0},
            goal_pose={"x": 0, "y": 0, "yaw": math.radians(270)},
        )
        time.sleep(0.2)
        db.shutdown()
        conn = db._direct_conn()
        row = conn.execute(
            "SELECT goal_yaw_deg FROM nav_runs WHERE id = ?", (rid2,)
        ).fetchone()
        expect(abs(row["goal_yaw_deg"] - (-90.0)) < 1e-9,
               f"yaw wrapped: 270 -> -90, got {row['goal_yaw_deg']}")
        conn.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_writer_thread_flush_on_shutdown():
    print("[test_writer_thread_flush_on_shutdown]")
    path = fresh_db()
    try:
        ok = db.init(db_path=path)
        expect(ok, "init returns True")
        # Enqueue many events
        for i in range(50):
            db.log_event(
                category="SYSTEM", level="info", action="tick",
                detail=f"i={i}",
            )
        # Immediately shutdown; should flush the queue
        db.shutdown(timeout=3.0)
        expect(not db.is_enabled(), "is_enabled False after shutdown")
        # Read with a fresh connection
        import sqlite3
        conn = sqlite3.connect(path)
        conn.row_factory = sqlite3.Row
        total = conn.execute("SELECT COUNT(*) FROM events").fetchone()[0]
        # 1 backend_started + 50 ticks
        expect(total == 51, f"events after shutdown = 51, got {total}")
        conn.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_mark_open_runs_interrupted():
    print("[test_mark_open_runs_interrupted]")
    path = fresh_db()
    try:
        # Insert an open run directly
        ok = db.init(db_path=path)
        expect(ok, "init returns True")
        # Use a direct connection (writer not strictly needed for this test)
        db.shutdown()
        db.init_sync(db_path=path)
        conn = db._direct_conn()
        conn.execute(
            "INSERT INTO nav_runs (started_at, run_type) VALUES (?, ?)",
            (time.time() - 100, "goal"),
        )
        conn.execute(
            "INSERT INTO nav_runs (started_at, run_type, result, ended_at) "
            "VALUES (?, ?, ?, ?)",
            (time.time() - 50, "goal", "SUCCEEDED", time.time() - 40),
        )
        conn.commit()
        conn.close()
        # Now call mark_open_runs_interrupted
        db.mark_open_runs_interrupted()
        # In init_sync mode the writer isn't running; do it directly
        db._mark_open_runs_interrupted_now()
        # Verify
        conn = db._direct_conn()
        rows = conn.execute(
            "SELECT id, result, ended_at FROM nav_runs ORDER BY id"
        ).fetchall()
        expect(rows[0]["result"] == "INTERRUPTED", "open run marked INTERRUPTED")
        expect(rows[0]["ended_at"] is not None, "ended_at populated")
        expect(rows[1]["result"] == "SUCCEEDED", "closed run untouched")
        conn.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main():
    tests = [
        test_yaw_wrap,
        test_schema_creation_and_init,
        test_init_failure_disables_feature,
        test_event_insert_and_query,
        test_run_lifecycle,
        test_meta,
        test_yaw_storage_uses_degrees,
        test_writer_thread_flush_on_shutdown,
        test_mark_open_runs_interrupted,
    ]
    for t in tests:
        try:
            t()
        except Exception:
            traceback.print_exc()
            _FAILURES.append(f"{t.__name__} raised")
    print()
    if _FAILURES:
        print(f"FAILED: {len(_FAILURES)} check(s):")
        for f in _FAILURES:
            print(f"  - {f}")
        return 1
    print("All tests passed.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
