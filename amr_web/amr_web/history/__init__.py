"""amr_web.history

Persistent operation history and navigation results (Phase 1-7 of HISTORY_RESULTS_PLAN.md).

Additive module: never modifies existing amr_web state. The feature is designed to
fail soft: if the DB cannot be opened, every call is a no-op and the robot runs
exactly as before.

Public entry points re-exported here for convenience:
    from amr_web.history import db, events, runs, system_hooks
"""
