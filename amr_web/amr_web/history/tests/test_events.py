"""
amr_web/history/tests/test_events.py
=====================================
Phase 2 acceptance tests for events.py + system_hooks.py.

Run as:
    cd ~/project_ws
    python3 src/amr_web/amr_web/history/tests/test_events.py

Covers:
- record_command logs an event with the right category/action/detail.
- update_command_result updates the latest pending (we don't have real ids
  in async mode, but the in-memory pending entry is consumed).
- record_system_event is a no-op when DB is disabled.
- record_ws_connect / record_ws_disconnect emit CONNECTION events.
- record_device_transition only fires on actual edge, not first observation.
- record_nav_state_change fires on transition only.
- record_battery_low / record_battery_recovered follow the 18% / 25% re-arm.
- record_teleop_tick aggregates into a single session per client.
- end_teleop_session flushes on disconnect.
- Failure in any helper never raises.
"""

from __future__ import annotations

import os
import sys
import time
import tempfile
import shutil
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
PKG_PARENT = os.path.abspath(os.path.join(HERE, "../../.."))
sys.path.insert(0, PKG_PARENT)

from amr_web.history import db, events, system_hooks


_FAILURES: list = []


def expect(cond, msg):
    if not cond:
        _FAILURES.append(msg)
        print(f"  FAIL: {msg}")
    else:
        print(f"  ok:   {msg}")


def fresh_db() -> str:
    """Reset module state, fresh temp dir, init in async mode."""
    db.shutdown()
    # Reset events module state
    with events._teleop_lock:
        events._teleop_sessions.clear()
    system_hooks.reset()
    tmp = tempfile.mkdtemp(prefix="amr_events_test_")
    return os.path.join(tmp, "events.db")


# ---------------------------------------------------------------------------

def test_record_command_categories():
    print("[test_record_command_categories]")
    path = fresh_db()
    try:
        ok = db.init(db_path=path)
        expect(ok, "init returns True")
        cases = [
            ("start_slam", "SLAM"),
            ("stop_slam", "SLAM"),
            ("save_map", "SLAM"),
            ("start_navigation", "NAVIGATION"),
            ("nav_goal", "NAVIGATION"),
            ("waypoint_mission", "MISSION"),
            ("set_op_mode", "MODE"),
            ("emergency_stop", "SAFETY"),
            ("lift", "LIFT"),
            ("teleop", "SYSTEM"),
        ]
        ids = []
        for cmd_name, _ in cases:
            eid = events.record_command({"command": cmd_name}, client_ip="1.2.3.4")
            ids.append(eid)
            expect(eid is not None, f"record_command({cmd_name}) returns id")
        db.shutdown(timeout=3.0)
        # Verify
        import sqlite3
        c = sqlite3.connect(path)
        c.row_factory = sqlite3.Row
        rows = c.execute(
            "SELECT action, category FROM events WHERE action != 'backend_started' "
            "ORDER BY id"
        ).fetchall()
        actions = [r["action"] for r in rows]
        cats = [r["category"] for r in rows]
        for cmd_name, want_cat in cases:
            expect(cmd_name in actions, f"event for {cmd_name} present")
        # Spot-check categories
        idx = {a: i for i, a in enumerate(actions)}
        for cmd_name, want_cat in cases:
            i = idx.get(cmd_name)
            if i is not None:
                expect(cats[i] == want_cat,
                       f"{cmd_name} -> category {want_cat} (got {cats[i]})")
        c.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_command_detail_and_params():
    print("[test_command_detail_and_params]")
    path = fresh_db()
    try:
        db.init(db_path=path)
        events.record_command(
            {"command": "nav_goal",
             "pose": {"x": 1.5, "y": -2.0, "yaw": 1.5707963},
             "route_id": "r1"},
            client_ip="1.1.1.1")
        events.record_command(
            {"command": "save_map", "map_name": "warehouse_a"},
            client_ip="1.1.1.1")
        events.record_command(
            {"command": "waypoint_mission",
             "waypoints": [{"x":0}, {"x":1}, {"x":2}, {"x":3}]},
            client_ip="1.1.1.1")
        events.record_command(
            {"command": "lift", "action": "GOTO", "target_mm": 1500},
            client_ip="1.1.1.1")
        db.shutdown(timeout=3.0)
        import sqlite3
        c = sqlite3.connect(path)
        c.row_factory = sqlite3.Row
        rows = c.execute(
            "SELECT action, detail, params FROM events "
            "WHERE action != 'backend_started' ORDER BY id"
        ).fetchall()
        by_action = {r["action"]: dict(r) for r in rows}
        ng = by_action.get("nav_goal") or {}
        expect("x=1.500" in ng.get("detail", ""), "nav_goal detail has x=1.500")
        expect("y=-2.000" in ng.get("detail", ""), "nav_goal detail has y=-2.000")
        sm = by_action.get("save_map") or {}
        expect("warehouse_a" in sm.get("detail", ""),
               "save_map detail has map name")
        wm_params = (by_action.get("waypoint_mission") or {}).get("params")
        if wm_params:
            import json
            try:
                wm = json.loads(wm_params)
                expect(wm.get("waypoint_count") == 4, "waypoint_count == 4")
            except Exception:
                expect(False, "waypoint_mission params is valid JSON")
        lift_detail = (by_action.get("lift") or {}).get("detail", "")
        expect("GOTO" in lift_detail, "lift detail has action=GOTO")
        expect("1500" in lift_detail, "lift detail has target_mm=1500")
        c.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_update_command_result_updates_row():
    """The row written by record_command must really get its final result (regression:
    a fake id used to make this a silent no-op, leaving every command 'requested')."""
    print("[test_update_command_result_updates_row]")
    path = fresh_db()
    try:
        import sqlite3
        db.init(db_path=path)
        eid1 = events.record_command(
            {"command": "nav_goal", "pose": {"x": 1.0, "y": 2.0, "yaw": 0.5}}, client_ip="1.1.1.1")
        eid2 = events.record_command({"command": "start_slam"}, client_ip="1.1.1.1")
        eid3 = events.record_command({"command": "stop_slam"}, client_ip="1.1.1.1")
        expect(isinstance(eid1, int) and isinstance(eid2, int) and eid1 != eid2,
               "record_command returns distinct real int ids")
        events.update_command_result(eid1, success=True, message="Goal accepted")
        events.update_command_result(eid2, success=False, message="boom")
        # eid3 is never updated -> stays 'requested'
        events.update_command_result(999999, success=False, message="x")   # unknown id: no-op
        db.shutdown(timeout=3.0)

        conn = sqlite3.connect(path)
        conn.row_factory = sqlite3.Row
        rows = {r["id"]: r for r in conn.execute("SELECT * FROM events")}
        conn.close()
        expect(rows[eid1]["result"] == "success", "nav_goal row result == success")
        expect("x=1.000" in rows[eid1]["detail"] and "Goal accepted" in rows[eid1]["detail"],
               "detail keeps the command text AND gets the response message appended: %r" % rows[eid1]["detail"])
        expect(rows[eid2]["result"] == "failed" and rows[eid2]["detail"] == "boom",
               "start_slam row result == failed, message becomes detail (it had none)")
        expect(rows[eid3]["result"] == "requested", "never-updated row stays 'requested'")
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_system_event_when_disabled_is_noop():
    print("[test_system_event_when_disabled_is_noop]")
    fresh_db()  # resets and does NOT init
    expect(not db.is_enabled(), "DB disabled after fresh_db")
    # These must not raise.
    events.record_ws_connect("1.2.3.4")
    events.record_ws_disconnect("1.2.3.4", duration_s=5.0)
    events.record_device_transition("lidar", True)
    events.record_nav_state_change(True)
    events.record_slam_state_change(True)
    events.record_battery_low(10.0)
    events.record_battery_recovered(50.0)
    events.record_teleop_tick("1.2.3.4", 0.5, 0.0)
    events.end_teleop_session("1.2.3.4")
    events.record_system_event("TEST", "noop", level="info")
    expect(True, "all disabled-mode calls are no-ops (no exception)")


def test_system_hooks_first_observation_no_event():
    print("[test_system_hooks_first_observation_no_event]")
    path = fresh_db()
    try:
        db.init(db_path=path)
        system_hooks.reset()
        # First observation: must not fire an event.
        system_hooks.tick(system_status={
            "devices": {"lidar": {"connected": True}, "esp32": {"connected": False}},
        })
        # Drain a moment for the writer.
        time.sleep(0.3)
        # Confirm no device events.
        db.shutdown(timeout=3.0)
        import sqlite3
        c = sqlite3.connect(path)
        c.row_factory = sqlite3.Row
        rows = c.execute(
            "SELECT action FROM events WHERE category = 'SYSTEM' "
            "AND action LIKE 'device_%'"
        ).fetchall()
        expect(len(rows) == 0, "no device_* events on first observation")
        c.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_system_hooks_device_transition():
    print("[test_system_hooks_device_transition]")
    path = fresh_db()
    try:
        db.init(db_path=path)
        system_hooks.reset()
        # 1st tick: baseline
        system_hooks.tick(system_status={
            "devices": {"lidar": {"connected": True}},
        })
        # 2nd tick: lidar flips to False
        system_hooks.tick(system_status={
            "devices": {"lidar": {"connected": False}},
        })
        # 3rd tick: lidar flips back to True
        system_hooks.tick(system_status={
            "devices": {"lidar": {"connected": True}},
        })
        time.sleep(0.3)
        db.shutdown(timeout=3.0)
        import sqlite3
        c = sqlite3.connect(path)
        c.row_factory = sqlite3.Row
        rows = c.execute(
            "SELECT action, detail FROM events WHERE action LIKE 'device_%' "
            "ORDER BY id"
        ).fetchall()
        expect(len(rows) == 2, f"two device events, got {len(rows)}")
        if len(rows) >= 1:
            expect(rows[0]["action"] == "device_disconnected", "first = disconnected")
            expect("lidar" in (rows[0]["detail"] or ""), "detail mentions lidar")
        if len(rows) >= 2:
            expect(rows[1]["action"] == "device_connected", "second = connected")
        c.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_system_hooks_battery_episode():
    print("[test_system_hooks_battery_episode]")
    path = fresh_db()
    try:
        db.init(db_path=path)
        system_hooks.reset()
        # Normal -> normal: no events
        system_hooks.tick(battery={"percentage": 80.0})
        time.sleep(0.2)
        # Drop to 15% -> one battery_low
        system_hooks.tick(battery={"percentage": 15.0})
        time.sleep(0.2)
        # Still low (16%) -> no new event (already armed)
        system_hooks.tick(battery={"percentage": 16.0})
        time.sleep(0.2)
        # Recover to 30% -> one battery_recovered
        system_hooks.tick(battery={"percentage": 30.0})
        time.sleep(0.2)
        # Drop again to 12% -> one more battery_low (re-armed)
        system_hooks.tick(battery={"percentage": 12.0})
        time.sleep(0.2)
        db.shutdown(timeout=3.0)
        import sqlite3
        c = sqlite3.connect(path)
        c.row_factory = sqlite3.Row
        rows = c.execute(
            "SELECT action FROM events WHERE action IN "
            "('battery_low', 'battery_recovered') ORDER BY id"
        ).fetchall()
        actions = [r["action"] for r in rows]
        expect(actions == ["battery_low", "battery_recovered", "battery_low"],
               f"sequence = low, recovered, low; got {actions}")
        c.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_system_hooks_nav_state_transition():
    print("[test_system_hooks_nav_state_transition]")
    path = fresh_db()
    try:
        db.init(db_path=path)
        system_hooks.reset()
        # First observation: None -> False, fires nav2_stopped.
        system_hooks.tick(nav_state={"running": False})
        time.sleep(0.2)
        system_hooks.tick(nav_state={"running": False})  # same -> no event
        time.sleep(0.2)
        system_hooks.tick(nav_state={"running": True})   # False -> True
        time.sleep(0.2)
        system_hooks.tick(nav_state={"running": True})   # same -> no event
        time.sleep(0.2)
        system_hooks.tick(nav_state={"running": False})  # True -> False
        time.sleep(0.2)
        db.shutdown(timeout=3.0)
        import sqlite3
        c = sqlite3.connect(path)
        c.row_factory = sqlite3.Row
        rows = c.execute(
            "SELECT action FROM events WHERE action IN "
            "('nav2_running', 'nav2_stopped') ORDER BY id"
        ).fetchall()
        actions = [r["action"] for r in rows]
        # Expected: stopped (None->False), running (False->True), stopped (True->False).
        expect(actions == ["nav2_stopped", "nav2_running", "nav2_stopped"],
               f"sequence = stopped,running,stopped; got {actions}")
        c.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_teleop_aggregation():
    print("[test_teleop_aggregation]")
    path = fresh_db()
    try:
        db.init(db_path=path)
        with events._teleop_lock:
            events._teleop_sessions.clear()
        ip = "5.6.7.8"
        # 5 ticks with non-zero velocity
        for i in range(5):
            events.record_teleop_tick(ip, 0.5 + 0.1 * i, 0.0)
            time.sleep(0.05)
        # 1 tick with zero -> opens a window
        events.record_teleop_tick(ip, 0.0, 0.0)
        # Wait > 1s then another zero tick -> flush
        time.sleep(1.2)
        events.record_teleop_tick(ip, 0.0, 0.0)
        time.sleep(0.3)
        # Now start a second session, keep it open for a measurable
        # duration, then end via disconnect.
        events.record_teleop_tick(ip, 0.2, 0.1)
        time.sleep(0.2)
        events.record_teleop_tick(ip, 0.3, 0.05)
        time.sleep(0.2)
        events.record_teleop_tick(ip, 0.0, 0.0)
        events.end_teleop_session(ip)
        time.sleep(0.3)
        db.shutdown(timeout=3.0)
        import sqlite3
        c = sqlite3.connect(path)
        c.row_factory = sqlite3.Row
        rows = c.execute(
            "SELECT params FROM events WHERE action = 'teleop_session' ORDER BY id"
        ).fetchall()
        expect(len(rows) == 2, f"two teleop_session events, got {len(rows)}")
        if len(rows) >= 1:
            import json
            try:
                p = json.loads(rows[0]["params"])
                expect(p.get("max_linear", 0) >= 0.5,
                       f"first session max_linear >= 0.5, got {p.get('max_linear')}")
                expect(p.get("duration_s", 0) >= 0.1,
                       f"first session duration_s >= 0.1, got {p.get('duration_s')}")
            except Exception:
                expect(False, "teleop_session params is valid JSON")
        c.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_nav_result_hook():
    print("[test_nav_result_hook]")
    # Direct test of the helper that nav_bridge calls.
    path = fresh_db()
    try:
        db.init(db_path=path)
        events.record_nav_result("SUCCEEDED", "Goal reached")
        events.record_nav_result("FAILED", "Stuck")
        events.record_nav_result("CANCELED", "User cancel")
        time.sleep(0.3)
        db.shutdown(timeout=3.0)
        import sqlite3
        c = sqlite3.connect(path)
        c.row_factory = sqlite3.Row
        rows = c.execute(
            "SELECT level, result FROM events WHERE action = 'nav_result' "
            "ORDER BY id"
        ).fetchall()
        expect(len(rows) == 3, f"3 nav_result events, got {len(rows)}")
        if len(rows) >= 2:
            expect(rows[1]["level"] == "error", "FAILED level = error")
        if len(rows) >= 3:
            expect(rows[2]["result"] == "CANCELED", "third = CANCELED")
        c.close()
    finally:
        try:
            shutil.rmtree(os.path.dirname(path))
        except Exception:
            pass


def test_soft_fail_never_raises():
    print("[test_soft_fail_never_raises]")
    # Pass garbage through the public API; nothing should raise.
    events.record_command(None)
    events.record_command({})
    events.record_command({"command": ""})
    events.record_command({"command": "lift", "action": "GOTO"})  # no target_mm
    events.update_command_result(None, True)
    events.update_command_result(123, success="yes", message=None)
    expect(True, "no exception on any garbage input")


# ---------------------------------------------------------------------------
# Driver
# ---------------------------------------------------------------------------

def main():
    tests = [
        test_record_command_categories,
        test_command_detail_and_params,
        test_update_command_result_updates_row,
        test_system_event_when_disabled_is_noop,
        test_system_hooks_first_observation_no_event,
        test_system_hooks_device_transition,
        test_system_hooks_battery_episode,
        test_system_hooks_nav_state_transition,
        test_teleop_aggregation,
        test_nav_result_hook,
        test_soft_fail_never_raises,
    ]
    for t in tests:
        try:
            t()
        except Exception:
            import traceback
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