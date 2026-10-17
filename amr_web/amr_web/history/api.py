"""
amr_web/history/api.py
======================
REST API for the History page (HISTORY_RESULTS_PLAN.md, Phase 4). Included in the
existing FastAPI app with ONE line (web_server.create_app).

    GET   /api/history/meta
    GET   /api/history/events
    GET   /api/history/runs
    GET   /api/history/runs/{run_id}
    PATCH /api/history/runs/{run_id}          body: {scenario?, notes?, measured_error_cm?}
    GET   /api/history/export/events.xlsx
    GET   /api/history/export/runs.xlsx

Conventions (same as the existing /api/* endpoints): JSON `{"status": "ok", ...}` or
`{"status": "error", "message": ...}`. Download endpoints use real HTTP status codes.
Handlers are plain `def` so FastAPI runs them in its thread pool: SQLite reads and Excel
generation never block the asyncio loop that serves the WebSocket.

Filters (all optional)  `from`, `to`  = UTC epoch seconds (the browser converts its local
date/time inputs), `page`, `page_size` (max 200), `sort` = asc|desc.
"""

from __future__ import annotations

import math
from datetime import datetime
from typing import Optional

from fastapi import APIRouter, Body, Query
from fastapi.responses import JSONResponse, Response

from amr_web.history import db, export

router = APIRouter(prefix="/api/history", tags=["history"])

_XLSX = "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet"


def _clean(**kw) -> dict:
    """Drop empty filter values."""
    return {k: v for k, v in kw.items() if v not in (None, "")}


def _page(items: list, total: int, page: int, page_size: int) -> dict:
    return {"status": "ok", "enabled": True, "total": total,
            "page": page, "page_size": page_size, "items": items}


def _disabled_page(page: int, page_size: int) -> dict:
    return {"status": "ok", "enabled": False, "total": 0,
            "page": page, "page_size": page_size, "items": []}


def _stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M")


# ---------------------------------------------------------------- meta
@router.get("/meta")
def get_meta():
    m = db.meta()
    m["status"] = "ok"
    m["timezone"] = export.tz_name()
    m["excel_available"] = export.openpyxl_available()
    return m


# -------------------------------------------------------------- events
@router.get("/events")
def get_events(
    from_: Optional[float] = Query(None, alias="from"),
    to: Optional[float] = None,
    category: Optional[str] = None,
    level: Optional[str] = None,
    result: Optional[str] = None,
    client: Optional[str] = None,
    q: Optional[str] = None,
    page: int = Query(1, ge=1),
    page_size: int = Query(50, ge=1, le=200),
    sort: str = Query("desc", pattern="^(asc|desc)$"),
):
    if not db.is_enabled():
        return _disabled_page(page, page_size)
    f = _clean(**{"from": from_}, to=to, category=category, level=level,
               result=result, client=client, q=q)
    items, total = db.query_events(f, page, page_size, sort)
    return _page(items, total, page, page_size)


# ---------------------------------------------------------------- runs
def _run_filters(from_, to, type_, result, map_id, scenario, client, q) -> dict:
    return _clean(**{"from": from_}, to=to, type=type_, result=result,
                  map_id=map_id, scenario=scenario, client=client, q=q)


@router.get("/runs")
def get_runs(
    from_: Optional[float] = Query(None, alias="from"),
    to: Optional[float] = None,
    type: Optional[str] = None,
    result: Optional[str] = None,
    map_id: Optional[str] = None,
    scenario: Optional[str] = None,
    client: Optional[str] = None,
    q: Optional[str] = None,
    page: int = Query(1, ge=1),
    page_size: int = Query(50, ge=1, le=200),
    sort: str = Query("desc", pattern="^(asc|desc)$"),
):
    if not db.is_enabled():
        return _disabled_page(page, page_size)
    f = _run_filters(from_, to, type, result, map_id, scenario, client, q)
    items, total = db.query_runs(f, page, page_size, sort)
    return _page(items, total, page, page_size)


@router.get("/runs/{run_id}")
def get_run(run_id: int):
    if not db.is_enabled():
        return {"status": "error", "enabled": False, "message": "History is disabled"}
    run = db.get_run(run_id)
    if run is None:
        return {"status": "error", "message": f"Run {run_id} not found"}
    return {"status": "ok", "run": run}


@router.patch("/runs/{run_id}")
def patch_run(run_id: int, payload: dict = Body(...)):
    """Edit the human fields of a run. Empty string clears scenario/notes."""
    if not db.is_enabled():
        return {"status": "error", "enabled": False, "message": "History is disabled"}
    if not isinstance(payload, dict):
        return {"status": "error", "message": "Body must be a JSON object"}

    scenario = payload.get("scenario")
    notes = payload.get("notes")
    measured = payload.get("measured_error_cm")

    if scenario is not None:
        scenario = str(scenario).strip()[:200]
    if notes is not None:
        notes = str(notes).strip()[:2000]
    if measured in ("", None):
        measured = None
    else:
        try:
            measured = float(measured)
        except (TypeError, ValueError):
            return {"status": "error", "message": "measured_error_cm must be a number"}
        if not math.isfinite(measured) or measured < 0 or measured > 10000:
            return {"status": "error", "message": "measured_error_cm must be between 0 and 10000"}

    if scenario is None and notes is None and measured is None:
        return {"status": "error", "message": "Nothing to update"}

    _, total = db.query_runs({"run_id": run_id}, 1, 1)
    if total == 0:
        return {"status": "error", "message": f"Run {run_id} not found"}

    db.update_run_meta(run_id, scenario=scenario, notes=notes, measured_error_cm=measured)
    return {"status": "ok", "run_id": run_id,
            "scenario": scenario, "notes": notes, "measured_error_cm": measured}


# -------------------------------------------------------------- export
def _excel_unavailable() -> JSONResponse:
    return JSONResponse(
        status_code=503,
        content={"status": "error",
                 "message": "Excel export needs the 'openpyxl' package "
                            "(sudo apt install python3-openpyxl)"},
    )


def _xlsx_response(data: bytes, name: str) -> Response:
    return Response(
        content=data, media_type=_XLSX,
        headers={"Content-Disposition": f'attachment; filename="{name}"',
                 "Cache-Control": "no-store"},
    )


@router.get("/export/events.xlsx")
def export_events(
    from_: Optional[float] = Query(None, alias="from"),
    to: Optional[float] = None,
    category: Optional[str] = None,
    level: Optional[str] = None,
    result: Optional[str] = None,
    client: Optional[str] = None,
    q: Optional[str] = None,
):
    if not export.openpyxl_available():
        return _excel_unavailable()
    if not db.is_enabled():
        return JSONResponse(status_code=503, content={"status": "error", "message": "History is disabled"})
    f = _clean(**{"from": from_}, to=to, category=category, level=level,
               result=result, client=client, q=q)
    try:
        data = export.build_events_xlsx(f)
    except Exception as e:
        return JSONResponse(status_code=500, content={"status": "error", "message": f"Export failed: {e}"})
    return _xlsx_response(data, f"amr_operation_history_{_stamp()}.xlsx")


@router.get("/export/runs.xlsx")
def export_runs(
    from_: Optional[float] = Query(None, alias="from"),
    to: Optional[float] = None,
    type: Optional[str] = None,
    result: Optional[str] = None,
    map_id: Optional[str] = None,
    scenario: Optional[str] = None,
    client: Optional[str] = None,
    q: Optional[str] = None,
    run_id: Optional[int] = None,
    include_trajectory: bool = True,
):
    if not export.openpyxl_available():
        return _excel_unavailable()
    if not db.is_enabled():
        return JSONResponse(status_code=503, content={"status": "error", "message": "History is disabled"})
    f = _run_filters(from_, to, type, result, map_id, scenario, client, q)
    if run_id:
        f["run_id"] = run_id
    try:
        data = export.build_runs_xlsx(f, include_trajectory=include_trajectory)
    except Exception as e:
        return JSONResponse(status_code=500, content={"status": "error", "message": f"Export failed: {e}"})
    name = (f"amr_run_{run_id}_{_stamp()}.xlsx" if run_id
            else f"amr_navigation_results_{_stamp()}.xlsx")
    return _xlsx_response(data, name)