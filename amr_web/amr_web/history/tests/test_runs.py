"""
Tests for amr_web/history/runs.py (Phase 3).  Run:  python3 test_runs.py
Every assertion reads the real SQLite rows (regression: path_length/straight_dist used
to be always NULL because the trajectory buffer was cleared before it was read).
"""
from __future__ import annotations

import math
import os
import shutil
import sqlite3
import sys
import tempfile
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.abspath(os.path.join(HERE, "../../..")))

from amr_web.history import db, runs  # noqa: E402

_FAILURES = []


def expect(cond, msg):
    if not cond:
        _FAILURES.append(msg)
        print(f"  FAIL: {msg}")
    else:
        print(f"  ok:   {msg}")


def near(a, b, tol):
    return a is not None and abs(a - b) <= tol


def fresh():
    db.shutdown()
    runs.set_active(None)
    runs.set_pending_client(None)
    tmp = tempfile.mkdtemp(prefix="amr_runs_test_")
    return os.path.join(tmp, "runs.db")


def read(path, sql, *a):
    conn = sqlite3.connect(path)
    conn.row_factory = sqlite3.Row
    try:
        return conn.execute(sql, a).fetchall()
    finally:
        conn.close()


def cleanup(path):
    try:
        shutil.rmtree(os.path.dirname(path))
    except Exception:
        pass


def test_goal_run_metrics():
    print("[test_goal_run_metrics]")
    path = fresh()
    try:
        db.init(db_path=path)
        t0 = time.time()
        t_open = time.time()
        s = runs.begin_goal_run({"x": 1.0, "y": 0.0, "yaw": 0.0},
                                {"x": 0.0, "y": 0.0, "yaw": 0.0}, "map_01", "route_a")
        expect(time.time() - t_open < 0.05, "begin_goal_run does not block (<50 ms)")
        expect(isinstance(s.run_id, int), "run id is a real int")
        # 1 m straight line, 0.05 m steps, with sub-threshold jitter (0.004 m) between samples
        n = 20
        for i in range(n + 1):
            jitter = 0.004 if i % 2 else 0.0
            runs.get_active().sample_pose(t0 + i * 0.2, i * 0.05, jitter, 0.0)
        runs.finish_active_run("SUCCEEDED", "Goal reached", {"x": 0.99, "y": 0.01, "yaw": 0.1})
        db.shutdown(timeout=3.0)

        r = read(path, "SELECT * FROM nav_runs")[0]
        expect(r["result"] == "SUCCEEDED" and r["ended_at"] is not None, "closed: result + ended_at set")
        expect(near(r["straight_dist_m"], 1.0, 1e-9), f"straight_dist = |start->goal| = 1.0 (got {r['straight_dist_m']})")
        expect(near(r["path_length_m"], 1.0, 0.03), f"path_length ~1.0 despite jitter (got {r['path_length_m']})")
        expect(near(r["pos_error_m"], math.hypot(0.01, 0.01), 1e-6), f"pos_error (got {r['pos_error_m']})")
        expect(near(r["yaw_error_deg"], math.degrees(0.1), 1e-6), f"yaw_error deg (got {r['yaw_error_deg']})")
        expect(r["duration_s"] is not None and r["duration_s"] >= 0, "duration_s set")
        expect(near(r["end_yaw_deg"], math.degrees(0.1), 1e-6), "end yaw stored in degrees")
        expect(r["client_ip"] is None, "no client ip when none was handed over")
        tr = read(path, "SELECT t FROM nav_trajectory WHERE run_id=? ORDER BY rowid", r["id"])
        expect(len(tr) == n + 2, f"trajectory = samples + final pose ({len(tr)})")
        expect(tr[0]["t"] < 0.5 and tr[-1]["t"] >= tr[0]["t"], "trajectory t is relative to run start")
    finally:
        cleanup(path)


def test_yaw_error_wraps():
    print("[test_yaw_error_wraps]")
    path = fresh()
    try:
        db.init(db_path=path)
        runs.begin_goal_run({"x": 0, "y": 0, "yaw": math.radians(179)}, {"x": 0, "y": 0, "yaw": 0.0}, None, None)
        runs.finish_active_run("SUCCEEDED", "ok", {"x": 0, "y": 0, "yaw": math.radians(-179)})
        db.shutdown(timeout=3.0)
        r = read(path, "SELECT yaw_error_deg FROM nav_runs")[0]
        expect(near(r["yaw_error_deg"], 2.0, 1e-6), f"-179 vs 179 -> +2 (got {r['yaw_error_deg']})")
    finally:
        cleanup(path)


def test_waypoint_progress():
    print("[test_waypoint_progress]")
    path = fresh()
    try:
        db.init(db_path=path)
        wps = [{"x": 1.0, "y": 0.0, "yaw": 0.0, "name": "A"},
               {"x": 2.0, "y": 0.0, "yaw": 0.0, "name": "B"},
               {"x": 3.0, "y": 0.0, "yaw": 0.0, "name": "C"}]
        runs.begin_waypoint_run(wps, {"x": 0.0, "y": 0.0, "yaw": 0.0}, "map_01", "r", "waypoints")
        runs.on_waypoint_progress(0, {"x": 0.1, "y": 0, "yaw": 0})       # still driving to #0: nothing reached
        runs.on_waypoint_progress(1, {"x": 1.02, "y": 0.01, "yaw": 0.0})  # #0 reached
        runs.on_waypoint_progress(1, {"x": 1.5, "y": 0, "yaw": 0})       # repeated feedback: no duplicate
        runs.on_waypoint_progress(2, {"x": 2.01, "y": 0.0, "yaw": 0.0})   # #1 reached
        runs.finish_active_run("SUCCEEDED", "All 3 waypoints reached", {"x": 3.0, "y": 0.0, "yaw": 0.0})
        db.shutdown(timeout=3.0)
        run = read(path, "SELECT * FROM nav_runs")[0]
        wp = read(path, "SELECT * FROM nav_run_waypoints WHERE run_id=? ORDER BY idx", run["id"])
        expect([w["idx"] for w in wp] == [0, 1, 2], f"3 waypoint rows (idx {[w['idx'] for w in wp]})")
        expect(all(w["status"] == "REACHED" for w in wp), "all REACHED")
        expect(wp[0]["name"] == "A" and near(wp[0]["pos_error_m"], math.hypot(0.02, 0.01), 1e-6),
               "waypoint 0: name + position error")
        expect(near(wp[2]["pos_error_m"], 0.0, 1e-9), "last waypoint recorded at SUCCEEDED with end pose")
        expect(run["run_type"] == "waypoints" and run["waypoint_total"] == 3, "run_type + waypoint_total")
        expect(near(run["goal_x"], 3.0, 1e-9), "run goal = last waypoint")
        expect(near(run["straight_dist_m"], 3.0, 1e-9), "straight = start -> last waypoint")
    finally:
        cleanup(path)


def test_cancel_and_supersede_and_idempotent():
    print("[test_cancel_and_supersede_and_idempotent]")
    path = fresh()
    try:
        db.init(db_path=path)
        runs.begin_goal_run({"x": 5, "y": 0, "yaw": 0}, {"x": 0, "y": 0, "yaw": 0}, None, None)
        runs.begin_goal_run({"x": 1, "y": 0, "yaw": 0}, {"x": 0, "y": 0, "yaw": 0}, None, None)   # supersedes #1
        runs.finish_active_run("CANCELED", "Navigation canceled", {"x": 0.3, "y": 0, "yaw": 0})
        runs.finish_active_run("FAILED", "late duplicate nav_result", {"x": 9, "y": 9, "yaw": 0})    # must be a no-op
        db.shutdown(timeout=3.0)
        rows = read(path, "SELECT * FROM nav_runs ORDER BY id")
        expect(len(rows) == 2, "two runs")
        expect(rows[0]["result"] == "CANCELED" and "Superseded" in (rows[0]["message"] or ""),
               "first run closed as CANCELED (superseded)")
        expect(rows[1]["result"] == "CANCELED" and rows[1]["message"] == "Navigation canceled",
               "second run closed by cancel; the late duplicate did not overwrite it")
        expect(near(rows[1]["end_x"], 0.3, 1e-9), "end pose from the first (real) close is kept")
    finally:
        cleanup(path)


def test_client_ip_handover():
    print("[test_client_ip_handover]")
    path = fresh()
    try:
        db.init(db_path=path)
        runs.set_pending_client("10.0.0.7")
        runs.begin_goal_run({"x": 1, "y": 0, "yaw": 0}, {"x": 0, "y": 0, "yaw": 0}, None, None)
        runs.finish_active_run("SUCCEEDED", "ok", {"x": 1, "y": 0, "yaw": 0})
        runs.begin_goal_run({"x": 1, "y": 0, "yaw": 0}, {"x": 0, "y": 0, "yaw": 0}, None, None)  # nothing pending now
        runs.finish_active_run("SUCCEEDED", "ok", {"x": 1, "y": 0, "yaw": 0})
        db.shutdown(timeout=3.0)
        rows = read(path, "SELECT client_ip FROM nav_runs ORDER BY id")
        expect(rows[0]["client_ip"] == "10.0.0.7", "first run carries the client ip")
        expect(rows[1]["client_ip"] is None, "pending ip is consumed once")
    finally:
        cleanup(path)


def test_disabled_is_noop():
    print("[test_disabled_is_noop]")
    db.shutdown()
    runs.set_active(None)
    s = runs.begin_goal_run({"x": 1, "y": 0, "yaw": 0}, {"x": 0, "y": 0, "yaw": 0}, None, None)
    expect(s is not None and s.run_id is None, "disabled -> session exists but has no run id")
    s.sample_pose(time.time(), 0, 0, 0)
    runs.on_waypoint_progress(3, None)
    runs.finish_active_run("SUCCEEDED", "x", None)
    runs.finish_active_run("SUCCEEDED", "x", None)
    expect(True, "no exception when disabled / called twice / pose is None")


def test_estop_marks_run():
    print("[test_estop_marks_run]")
    path = fresh()
    try:
        db.init(db_path=path)
        runs.begin_goal_run({"x": 5, "y": 0, "yaw": 0}, {"x": 0, "y": 0, "yaw": 0}, None, None)
        runs.mark_estop()
        runs.finish_active_run("CANCELED", "Navigation canceled", {"x": 1, "y": 0, "yaw": 0})
        runs.begin_goal_run({"x": 2, "y": 0, "yaw": 0}, {"x": 0, "y": 0, "yaw": 0}, None, None)
        runs.mark_estop()      # robot resumed and still arrived
        runs.finish_active_run("SUCCEEDED", "Goal reached", {"x": 2, "y": 0, "yaw": 0})
        runs.mark_estop()      # no active run: must not raise
        db.shutdown(timeout=3.0)
        rows = read(path, "SELECT * FROM nav_runs ORDER BY id")
        expect(rows[0]["result"] == "ESTOP" and rows[0]["message"].startswith("[E-stop pressed]"),
               "E-stop + CANCELED -> result ESTOP")
        expect(rows[1]["result"] == "SUCCEEDED" and "[E-stop pressed]" in rows[1]["message"],
               "E-stop but still SUCCEEDED -> result kept, message notes the E-stop")
    finally:
        cleanup(path)


if __name__ == "__main__":
    for fn in (test_goal_run_metrics, test_yaw_error_wraps, test_waypoint_progress,
               test_cancel_and_supersede_and_idempotent, test_client_ip_handover, test_estop_marks_run,
               test_disabled_is_noop):
        try:
            fn()
        except Exception:
            import traceback
            traceback.print_exc()
            _FAILURES.append(f"{fn.__name__} raised")
    print()
    if _FAILURES:
        print(f"{len(_FAILURES)} FAILURE(S):")
        for f in _FAILURES:
            print("  -", f)
        sys.exit(1)
    print("All tests passed.")