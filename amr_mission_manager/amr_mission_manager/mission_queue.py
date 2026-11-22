"""Pure validation and transition helpers for bounded pallet mission queues."""
import json


SLOT_CLASS = {"A1": "medium", "A2": "medium", "B1": "tall", "B2": "tall"}


def normalize_mission_queue(raw, legacy_task, max_tasks=8):
    """Return normalized task dictionaries or raise ValueError.

    Each task has ``cargo_class``, ``quantity`` and ``drop_slot``. An empty
    queue string preserves the legacy single-task parameters.
    """
    if raw:
        try:
            queue = json.loads(raw) if isinstance(raw, str) else raw
        except (TypeError, ValueError) as exc:
            raise ValueError(f"target_queue_json is invalid JSON: {exc}") from exc
    else:
        queue = [legacy_task]
    if not isinstance(queue, list) or not queue or len(queue) > max_tasks:
        raise ValueError(f"mission queue must contain 1 to {max_tasks} tasks")

    normalized = []
    reserved_slots = set()
    for index, task in enumerate(queue, 1):
        if not isinstance(task, dict) or set(task) - {"cargo_class", "quantity", "drop_slot"}:
            raise ValueError(f"queue task {index} must contain only cargo_class, quantity, drop_slot")
        cargo = str(task.get("cargo_class") or "any").strip().lower()
        quantity = task.get("quantity", 1)
        slot_raw = str(task.get("drop_slot") or "auto").strip()
        slot = "auto" if slot_raw.lower() == "auto" else slot_raw.upper()
        if cargo not in ("any", "medium", "tall"):
            raise ValueError(f"queue task {index}: invalid cargo_class {cargo!r}")
        if isinstance(quantity, bool) or not isinstance(quantity, int) or not 1 <= quantity <= 10:
            raise ValueError(f"queue task {index}: quantity must be an integer from 1 to 10")
        if slot not in ("auto", *SLOT_CLASS):
            raise ValueError(f"queue task {index}: invalid drop_slot {slot!r}")
        if slot != "auto":
            if quantity != 1:
                raise ValueError(f"queue task {index}: fixed slot {slot} only accepts one pallet")
            expected = SLOT_CLASS[slot]
            if cargo not in ("any", expected):
                raise ValueError(f"queue task {index}: slot {slot} is incompatible with {cargo}")
            if slot in reserved_slots:
                raise ValueError(f"queue task {index}: slot {slot} is requested more than once")
            reserved_slots.add(slot)
            cargo = expected
        normalized.append({"cargo_class": cargo, "quantity": quantity, "drop_slot": slot})
    return normalized


def task_after_placement(queue, task_index, task_placed):
    """Return (transition, next_index) after a confirmed pallet placement."""
    if task_index < 0 or task_index >= len(queue):
        raise ValueError("task_index is outside the mission queue")
    if task_placed < queue[task_index]["quantity"]:
        return "repeat", task_index
    if task_index + 1 < len(queue):
        return "next", task_index + 1
    return "complete", task_index


def mission_start_blockers(*, stopping, discarding, tick_inflight,
                           unresolved_services, manager_busy,
                           terminal_idle_confirmed):
    """Explain lifecycle ownership blockers without treating stale busy as truth."""
    blockers = []
    if stopping or discarding:
        blockers.append("previous mission is still stopping")
    if tick_inflight:
        blockers.append("previous mission state transition is still in flight")
    if unresolved_services:
        blockers.append(f"{unresolved_services} service outcome(s) are still unresolved")
    if manager_busy is True and not terminal_idle_confirmed:
        blockers.append("Mission Manager reports an active navigation run")
    return blockers
