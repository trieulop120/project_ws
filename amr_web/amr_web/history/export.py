"""
amr_web/history/export.py
=========================
Excel (.xlsx) reports for the History page (HISTORY_RESULTS_PLAN.md, Phase 5).

    build_events_xlsx(filters)                       -> bytes   (sheet: Events)
    build_runs_xlsx(filters, include_trajectory)     -> bytes   (sheets: Runs, Waypoints,
                                                                 Trajectory, Summary)

`filters` are the same dicts the REST API passes to db.query_events / db.query_runs, so an
export always matches what the page is showing (without paging).

Formatting rules (thesis-friendly):
- every x / y / yaw / distance / error column uses the number format 0.000, the cell keeps
  the full-precision value (only the display is rounded);
- dates and times are separate real Excel date/time cells in the server timezone
  (env AMR_TZ, default = system timezone);
- Summary: error statistics use SUCCEEDED runs only (for FAILED/CANCELED runs the
  "error" is just the distance that was left, not an accuracy figure).
"""

from __future__ import annotations

import io
import math
import os
import statistics
from datetime import datetime
from typing import Callable, Optional

from amr_web.history import db

try:                                  # openpyxl is an optional runtime dependency
    from openpyxl import Workbook
    from openpyxl.chart import Reference, ScatterChart, Series
    from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
    from openpyxl.utils import get_column_letter
    from openpyxl.worksheet.properties import PageSetupProperties
    _OPENPYXL_OK = True
except Exception:                     # pragma: no cover - depends on the machine
    _OPENPYXL_OK = False


FMT3 = "0.000"
FMT2 = "0.00"
FMT1 = "0.0"
FMT_DATE = "yyyy-mm-dd"
FMT_TIME = "hh:mm:ss"

_HEADER_FILL = None
_THIN = None


def openpyxl_available() -> bool:
    return _OPENPYXL_OK


# ---------------------------------------------------------------------------
# Time helpers
# ---------------------------------------------------------------------------

def _tz():
    name = os.environ.get("AMR_TZ")
    if name:
        try:
            from zoneinfo import ZoneInfo
            return ZoneInfo(name)
        except Exception:
            pass
    return datetime.now().astimezone().tzinfo


def tz_name() -> str:
    tz = _tz()
    return os.environ.get("AMR_TZ") or str(datetime.now(tz).tzname() or tz)


def _dt(ts) -> Optional[datetime]:
    """UTC epoch seconds -> naive datetime in the server timezone (Excel has no tz)."""
    if ts is None:
        return None
    try:
        return datetime.fromtimestamp(float(ts), _tz()).replace(tzinfo=None, microsecond=0)
    except Exception:
        return None


def _date(ts):
    d = _dt(ts)
    return d.date() if d else None


def _time(ts):
    d = _dt(ts)
    return d.time() if d else None


def _fmt_ts(ts) -> str:
    d = _dt(ts)
    return d.strftime("%Y-%m-%d %H:%M:%S") if d else "-"


# ---------------------------------------------------------------------------
# Generic sheet writer
# ---------------------------------------------------------------------------

class Col:
    __slots__ = ("header", "get", "fmt", "width")

    def __init__(self, header: str, get: Callable, fmt: Optional[str] = None, width: float = 12):
        self.header, self.get, self.fmt, self.width = header, get, fmt, width


def _style_header(ws, ncols: int, row: int = 1) -> None:
    fill = PatternFill("solid", fgColor="1F4E5F")
    thin = Side(style="thin", color="808080")
    for c in range(1, ncols + 1):
        cell = ws.cell(row=row, column=c)
        cell.font = Font(bold=True, color="FFFFFF")
        cell.fill = fill
        cell.alignment = Alignment(horizontal="center", vertical="center", wrap_text=True)
        cell.border = Border(bottom=thin)
    ws.row_dimensions[row].height = 32


def _print_setup(ws) -> None:
    """Landscape, one page wide (any number of pages tall), header row repeated."""
    ws.page_setup.orientation = "landscape"
    ws.page_setup.fitToWidth = 1
    ws.page_setup.fitToHeight = 0
    ws.sheet_properties.pageSetUpPr = PageSetupProperties(fitToPage=True)
    ws.print_title_rows = "1:1"


def _write_table(ws, cols: list, rows: list, start_row: int = 1) -> int:
    """Write header + rows. Returns the last written row number."""
    for ci, col in enumerate(cols, start=1):
        ws.cell(row=start_row, column=ci, value=col.header)
        ws.column_dimensions[get_column_letter(ci)].width = col.width
    _style_header(ws, len(cols), start_row)
    r = start_row
    for row in rows:
        r += 1
        for ci, col in enumerate(cols, start=1):
            try:
                v = col.get(row)
            except Exception:
                v = None
            if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
                v = None
            cell = ws.cell(row=r, column=ci, value=v)
            if col.fmt and v is not None:
                cell.number_format = col.fmt
    ws.freeze_panes = ws.cell(row=start_row + 1, column=1)
    _print_setup(ws)
    if cols:
        ws.auto_filter.ref = f"A{start_row}:{get_column_letter(len(cols))}{max(r, start_row + 1)}"
    return r


def _key(k: str, fmt: Optional[str] = None, header: Optional[str] = None, width: float = 12) -> Col:
    return Col(header or k, lambda row, k=k: row.get(k), fmt, width)


# ---------------------------------------------------------------------------
# Events
# ---------------------------------------------------------------------------

def build_events_xlsx(filters: Optional[dict] = None) -> bytes:
    if not _OPENPYXL_OK:
        raise RuntimeError("openpyxl is not installed")
    rows = db.fetch_events_all(filters or {}, sort="asc")
    wb = Workbook()
    ws = wb.active
    ws.title = "Events"
    cols = [
        Col("ID", lambda r: r["id"], None, 8),
        Col("Date", lambda r: _date(r["ts"]), FMT_DATE, 12),
        Col("Time", lambda r: _time(r["ts"]), FMT_TIME, 10),
        _key("category", header="Category", width=14),
        _key("level", header="Level", width=8),
        _key("action", header="Action", width=22),
        _key("detail", header="Detail", width=60),
        _key("result", header="Result", width=12),
        _key("client_ip", header="Client", width=16),
    ]
    _write_table(ws, cols, rows)
    buf = io.BytesIO()
    wb.save(buf)
    return buf.getvalue()


# ---------------------------------------------------------------------------
# Runs
# ---------------------------------------------------------------------------

def _run_columns() -> list:
    return [
        Col("Run ID", lambda r: r["id"], None, 8),
        Col("Date", lambda r: _date(r["started_at"]), FMT_DATE, 12),
        Col("Start time", lambda r: _time(r["started_at"]), FMT_TIME, 10),
        Col("End time", lambda r: _time(r["ended_at"]), FMT_TIME, 10),
        _key("run_type", header="Type", width=10),
        _key("scenario", header="Scenario", width=24),
        _key("map_id", header="Map", width=14),
        _key("route_id", header="Route", width=14),
        _key("start_x", FMT3, "Start X (m)", 11),
        _key("start_y", FMT3, "Start Y (m)", 11),
        _key("start_yaw_deg", FMT3, "Start Yaw (deg)", 12),
        _key("goal_x", FMT3, "Goal X (m)", 11),
        _key("goal_y", FMT3, "Goal Y (m)", 11),
        _key("goal_yaw_deg", FMT3, "Goal Yaw (deg)", 12),
        _key("end_x", FMT3, "End X (m)", 11),
        _key("end_y", FMT3, "End Y (m)", 11),
        _key("end_yaw_deg", FMT3, "End Yaw (deg)", 12),
        _key("straight_dist_m", FMT3, "Straight distance (m)", 13),
        _key("path_length_m", FMT3, "Path length (m)", 12),
        _key("duration_s", FMT2, "Duration (s)", 11),
        _key("pos_error_m", FMT3, "Position error (m)", 13),
        _key("yaw_error_deg", FMT3, "Yaw error (deg)", 12),
        _key("measured_error_cm", FMT1, "Measured error (cm)", 13),
        _key("result", header="Result", width=12),
        _key("message", header="Message", width=34),
        _key("notes", header="Notes", width=34),
        _key("client_ip", header="Client", width=16),
    ]


def _waypoint_columns() -> list:
    return [
        Col("Run ID", lambda r: r["run_id"], None, 8),
        Col("Waypoint #", lambda r: (r["idx"] + 1) if r.get("idx") is not None else None, None, 11),
        _key("name", header="Name", width=16),
        _key("goal_x", FMT3, "Goal X (m)", 11),
        _key("goal_y", FMT3, "Goal Y (m)", 11),
        _key("goal_yaw_deg", FMT3, "Goal Yaw (deg)", 12),
        Col("Reached date", lambda r: _date(r.get("reached_at")), FMT_DATE, 12),
        Col("Reached time", lambda r: _time(r.get("reached_at")), FMT_TIME, 11),
        _key("reached_x", FMT3, "Reached X (m)", 12),
        _key("reached_y", FMT3, "Reached Y (m)", 12),
        _key("reached_yaw_deg", FMT3, "Reached Yaw (deg)", 14),
        _key("pos_error_m", FMT3, "Position error (m)", 13),
        _key("yaw_error_deg", FMT3, "Yaw error (deg)", 12),
        _key("status", header="Status", width=10),
    ]


def _trajectory_columns() -> list:
    return [
        Col("Run ID", lambda r: r["run_id"], None, 8),
        _key("t", FMT2, "t (s)", 10),
        _key("x", FMT3, "X (m)", 11),
        _key("y", FMT3, "Y (m)", 11),
        _key("yaw_deg", FMT3, "Yaw (deg)", 11),
    ]


def _stats(values: list) -> dict:
    v = [x for x in values if isinstance(x, (int, float)) and not math.isnan(x)]
    n = len(v)
    return {
        "n": n,
        "mean": statistics.fmean(v) if n else None,
        "std": statistics.stdev(v) if n >= 2 else None,
        "min": min(v) if n else None,
        "max": max(v) if n else None,
    }


def _summary_rows(runs: list) -> list:
    """One dict per group: ALL + every scenario."""
    groups = [("ALL RUNS", runs)]
    by_scn: dict = {}
    for r in runs:
        by_scn.setdefault((r.get("scenario") or "").strip() or "(no scenario)", []).append(r)
    groups += sorted(by_scn.items(), key=lambda kv: kv[0].lower())

    out = []
    for name, rs in groups:
        ok = [r for r in rs if r.get("result") == "SUCCEEDED"]
        pe = _stats([r.get("pos_error_m") for r in ok])
        ye_signed = [r.get("yaw_error_deg") for r in ok]
        ye = _stats(ye_signed)
        ye_abs = _stats([abs(x) for x in ye_signed if isinstance(x, (int, float))])
        du = _stats([r.get("duration_s") for r in ok])
        pl = _stats([r.get("path_length_m") for r in ok])
        me = _stats([r.get("measured_error_cm") for r in rs])
        out.append({
            "Group": name,
            "Runs": len(rs),
            "Succeeded": len(ok),
            "Success rate (%)": (100.0 * len(ok) / len(rs)) if rs else None,
            "Pos err mean (m)": pe["mean"], "Pos err std (m)": pe["std"],
            "Pos err min (m)": pe["min"], "Pos err max (m)": pe["max"],
            "Yaw err mean (deg)": ye["mean"], "Yaw err std (deg)": ye["std"],
            "Yaw |err| mean (deg)": ye_abs["mean"], "Yaw |err| max (deg)": ye_abs["max"],
            "Duration mean (s)": du["mean"], "Duration std (s)": du["std"],
            "Path length mean (m)": pl["mean"],
            "Measured err N": me["n"], "Measured err mean (cm)": me["mean"],
        })
    return out


_SUMMARY_FMT = {
    "Success rate (%)": FMT1,
    "Measured err mean (cm)": FMT1,
    "Duration mean (s)": FMT2, "Duration std (s)": FMT2,
}


def _write_summary(ws, runs: list, filters: dict) -> None:
    ws["A1"] = "Navigation results - summary"
    ws["A1"].font = Font(bold=True, size=14)
    shown = {k: v for k, v in (filters or {}).items() if v not in (None, "")}
    rng_from = _fmt_ts(shown.pop("from")) if "from" in shown else "(start)"
    rng_to = _fmt_ts(shown.pop("to")) if "to" in shown else "(now)"
    meta_rows = [
        ("Exported at", _fmt_ts(datetime.now().timestamp())),
        ("Timezone", tz_name()),
        ("Range", f"{rng_from}  ->  {rng_to}"),
        ("Other filters", ", ".join(f"{k}={v}" for k, v in shown.items()) or "-"),
        ("Runs in file", len(runs)),
        ("Note", "Error statistics use SUCCEEDED runs only. Position/yaw errors come from the "
                 "robot's own localisation (AMCL); see 'Measured error (cm)' for tape-measured values."),
    ]
    r = 2
    for k, v in meta_rows:
        ws.cell(row=r, column=1, value=k).font = Font(bold=True)
        ws.cell(row=r, column=2, value=v)
        r += 1
    rows = _summary_rows(runs)
    if not rows:
        return
    headers = list(rows[0].keys())
    hdr_row = r + 1
    for ci, h in enumerate(headers, start=1):
        ws.cell(row=hdr_row, column=ci, value=h)
        ws.column_dimensions[get_column_letter(ci)].width = 26 if ci == 1 else 14
    _style_header(ws, len(headers), hdr_row)
    for ri, row in enumerate(rows, start=hdr_row + 1):
        for ci, h in enumerate(headers, start=1):
            v = row[h]
            cell = ws.cell(row=ri, column=ci, value=v)
            if isinstance(v, float):
                cell.number_format = _SUMMARY_FMT.get(h, FMT3)
        if ri == hdr_row + 1:
            for ci in range(1, len(headers) + 1):
                ws.cell(row=ri, column=ci).font = Font(bold=True)
    ws.freeze_panes = ws.cell(row=hdr_row + 1, column=2)
    ws.page_setup.orientation = "landscape"
    ws.page_setup.fitToWidth = 1
    ws.page_setup.fitToHeight = 0
    ws.sheet_properties.pageSetUpPr = PageSetupProperties(fitToPage=True)


def _add_trajectory_chart(ws, n_points: int) -> None:
    """XY plot of a single run (only used when the export is for exactly one run)."""
    if n_points < 2:
        return
    ch = ScatterChart()
    ch.title = "Trajectory (map frame)"
    ch.style = 13
    ch.x_axis.title = "X (m)"
    ch.y_axis.title = "Y (m)"
    ch.legend = None
    ch.height, ch.width = 11, 14
    xs = Reference(ws, min_col=3, min_row=2, max_row=n_points + 1)
    ys = Reference(ws, min_col=4, min_row=2, max_row=n_points + 1)
    s = Series(ys, xs, title="path")
    s.marker.symbol = "none"
    ch.series.append(s)
    ws.add_chart(ch, "G2")


def build_runs_xlsx(filters: Optional[dict] = None, include_trajectory: bool = True) -> bytes:
    if not _OPENPYXL_OK:
        raise RuntimeError("openpyxl is not installed")
    filters = dict(filters or {})
    runs = db.fetch_runs_all(filters, sort="asc")
    wb = Workbook()

    ws = wb.active
    ws.title = "Runs"
    _write_table(ws, _run_columns(), runs)

    ws_w = wb.create_sheet("Waypoints")
    _write_table(ws_w, _waypoint_columns(), db.fetch_waypoints_all(filters))

    if include_trajectory:
        ws_t = wb.create_sheet("Trajectory")
        traj = db.fetch_trajectory_all(filters)
        _write_table(ws_t, _trajectory_columns(), traj)
        if filters.get("run_id"):
            _add_trajectory_chart(ws_t, len(traj))

    _write_summary(wb.create_sheet("Summary"), runs, filters)

    buf = io.BytesIO()
    wb.save(buf)
    return buf.getvalue()