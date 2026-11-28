"""Map-scoped projection of existing mission telemetry for the 3D viewer."""
import math
import time
from threading import Lock

_CAMERA_PAIR_S = 0.35  # The detector's cargo String has no stamp or target ID.
_TARGET_FRESH_S = 3
_PICK_LOCK_STATES = {'READY', 'NAVIGATING', 'SETTLING', 'ALIGNING', 'INSERTING', 'LIFTING'}
_CARGO_TYPE_BY_CLASS = {'medium': 'A', 'tall': 'B'}


class WarehouseView:
    def __init__(self):
        self._lock = Lock()
        self._select_map(None)

    def _select_map(self, map_id):
        self._map = map_id
        self._mission = None
        self._at = 0
        self._target = None
        self._target_at = 0
        self._pose_stamp = None
        self._camera_cargo = None
        self._cargo_for_pose = False
        self._carried = None
        self._drops = {}
        self._departed = {}
        self._release = None
        self._released_at = None
        self._pickup_key = None
        self._release_key = None
        self._source_empty = False

    def _slot_identity(self, slot):
        key = self._drops.get(slot, {}).get('id', f'shelf_{slot}')
        if any(other != slot and drop['id'] == key for other, drop in self._drops.items()):
            return f'shelf_{slot}_replacement'
        return key

    @staticmethod
    def _states(status):
        shelf = status.get('shelf')
        if isinstance(shelf, dict) and shelf.get('fresh') is True and isinstance(shelf.get('states'), dict):
            return shelf['states']
        return {}

    def _floor_detecting(self, now):
        mission = self._mission or {}
        return (now - self._at <= _TARGET_FRESH_S and not self._carried and
                mission.get('state') == 'DETECTING' and mission.get('mode') == 'pick' and
                bool(mission.get('session_id')))

    def _reset_target(self):
        self._target = None
        self._target_at = 0
        self._pose_stamp = None
        self._camera_cargo = None
        self._cargo_for_pose = False

    def camera_cargo(self, map_id, data):
        """Use the existing detector estimate as a preview, scoped to a recent map pose."""
        height = data.get('height_m')
        cargo_class = data.get('class')
        if (not isinstance(cargo_class, str) or cargo_class not in _CARGO_TYPE_BY_CLASS or
                type(height) not in (float, int) or not math.isfinite(height) or not 0 < height <= 50):
            return
        with self._lock:
            now = time.monotonic()
            if not map_id or map_id != self._map or not self._floor_detecting(now):
                return
            if self._target and self._target.get('slot'):
                return
            sample = {'height_m': height, 'cargo_class': cargo_class,
                      'cargo_type': _CARGO_TYPE_BY_CLASS[cargo_class]}
            if self._target and not self._cargo_for_pose and now - self._target_at <= _CAMERA_PAIR_S:
                self._target = {**self._target, **sample}
                self._cargo_for_pose = True
            else:
                # DDS callbacks from the two topics can arrive in either order.
                self._camera_cargo = (sample, now)

    def pose(self, map_id, pose, retrieval=None, stamp=None):
        if not map_id:
            return
        with self._lock:
            if map_id != self._map:
                self._select_map(map_id)
            mission = self._mission or {}
            now = time.monotonic()
            if self._carried or now - self._at > _TARGET_FRESH_S:
                return
            if retrieval:
                run, slot = retrieval.get('run_id'), retrieval.get('slot')
                if (not run or not slot or retrieval.get('stale') or
                        retrieval.get('state') not in ('DETECTING', 'PICKING', 'RECOVERY')):
                    return
                key = self._slot_identity(slot)
            else:
                if not self._floor_detecting(now):
                    return
                key = f'pick_{mission["session_id"]}_{mission.get("operation_id")}'
                run = slot = None
                if stamp is not None and self._pose_stamp is not None and stamp <= self._pose_stamp:
                    return
                self._pose_stamp = stamp
            existing = self._target or {}
            same_target = existing.get('id') == key and existing.get('run') == run
            if same_target and not run:
                # A newly acquired camera target can replace a lost one during
                # Detect. Don't give it the previous target's dimensions.
                prev = existing['pose']
                same_target = math.dist((pose['x'], pose['y']), (prev['x'], prev['y'])) <= .18
            self._target = {**(existing if same_target else {}), 'id': key, 'pose': pose,
                            'slot': slot, 'run': run, 'locked': bool(run)}
            self._target_at = now
            self._cargo_for_pose = False
            pending, self._camera_cargo = self._camera_cargo, None
            if not run and pending and now - pending[1] <= _CAMERA_PAIR_S:
                self._target.update(pending[0])
                self._cargo_for_pose = True

    @staticmethod
    def _cargo(status):
        height = status.get('cargo_h_cm')
        return {'cargo_type': status.get('cargo_type'), 'cargo_class': status.get('cargo_class'),
                'height_m': height / 100 if type(height) in (int, float) and
                math.isfinite(height) and 0 < height <= 5000 else None}

    def mission(self, map_id, status, retrieval):
        with self._lock:
            if map_id != self._map:
                self._select_map(map_id)
            if not map_id:
                return
            previous = self._mission or {}
            # DDS status uses increasing operation IDs within a pallet-node session.
            if (previous.get('session_id') == status.get('session_id') and
                    isinstance(previous.get('operation_id'), int) and isinstance(status.get('operation_id'), int)
                    and status['operation_id'] < previous['operation_id']):
                return
            self._mission, self._at = status, time.monotonic()
            key = (status.get('session_id'), status.get('operation_id'))
            states = self._states(status)
            for slot in list(self._departed):
                if states.get(slot) == 'empty':
                    del self._departed[slot]
            # An empty reading can precede sensor settling after DROP. Forget
            # the association only after an observed full-to-empty change.
            for slot, drop in list(self._drops.items()):
                placing = (self._release_key == key and self._release and
                           self._release.get('slot') == slot and status.get('state') != 'DONE')
                if states.get(slot) == 'empty' and drop.get('seen_full') and not placing:
                    del self._drops[slot]
                elif states.get(slot) == 'full' and not drop.get('seen_full'):
                    self._drops[slot] = {**drop, 'seen_full': True}
            phase = status.get('state')
            if self._release and self._release['destination'] == 'D_2' and (
                    phase == 'DETECTING' or
                    (retrieval and retrieval.get('run_id') != self._release['run']) or
                    (key[0] and self._release_key[0] and key[0] != self._release_key[0])):
                self._release = None
            previous_key = (previous.get('session_id'), previous.get('operation_id'))
            if ((phase == 'DETECTING' and (previous.get('state') != phase or key != previous_key)) or
                    (self._target and not self._target.get('run') and
                     (phase == 'IDLE' or status.get('mode') == 'drop' or key[0] != previous_key[0]))):
                self._reset_target()

            cargo = self._cargo(status)
            if self._target and not self._target.get('run') and status.get('mode') == 'pick':
                if phase in _PICK_LOCK_STATES:
                    self._target['locked'] = True
                if phase != 'DETECTING':
                    # Mission cargo_h_cm is the finalized camera estimate.
                    self._target.update({field:value for field, value in cargo.items() if value is not None})

            confirmed_pick = status.get('cargo') is True and status.get('mode') == 'pick' and (
                status.get('state') == 'CARRYING' or
                (status.get('state') == 'BACKING' and status.get('us_pick_confirmed') is True))
            if confirmed_pick and not self._carried and key[0] and key not in (self._pickup_key, self._release_key):
                owned = self._target
                if owned and owned.get('run') and (not retrieval or retrieval.get('stale') or
                        retrieval.get('run_id') != owned['run'] or retrieval.get('slot') != owned['slot']):
                    owned = None
                if not owned and retrieval and retrieval.get('state') in ('PICKING', 'NAVIGATING_TO_D2'):
                    run, slot = retrieval.get('run_id'), retrieval.get('slot')
                    if run and slot:
                        owned = {'id': self._slot_identity(slot), 'slot': slot, 'run': run}
                if not owned:
                    owned = {'id': f'pick_{key[0]}_{key[1]}', 'slot': None, 'run': None}
                cargo = {field:value if value is not None else owned.get(field) for field, value in cargo.items()}
                if owned.get('run') and cargo['height_m'] is None:
                    # Retrieval infers the allowed class from the slot. That
                    # constraint does not identify the product on the pallet.
                    evidence = self._drops.get(owned['slot'], {})
                    cargo = {field: evidence.get(field) for field in cargo}
                self._carried = {**owned, **cargo}
                self._reset_target()
                self._release = None
                self._pickup_key = key
                source = owned.get('slot')
                self._source_empty = states.get(source) == 'empty'
                if source:
                    self._drops.pop(source, None)
                    if not self._source_empty:
                        self._departed[source] = owned['id']

            if status.get('cargo') is False and status.get('cargo_why') == 'operator_confirmed_empty':
                self._carried = None
            if not self._carried:
                return
            slot = self._carried.get('slot')
            if states.get(slot) == 'empty':
                self._source_empty = True
            drop = status.get('drop') or {}
            at_d2 = (retrieval and not retrieval.get('stale') and self._carried.get('run') and
                     retrieval.get('run_id') == self._carried['run'] and
                     retrieval.get('state') == 'PLACING_AT_D2')
            event = status.get('visual_release')
            confirmed = (event.get('destination') if isinstance(event, dict) and
                         event.get('operation_id') == key[1] else None)
            empty = status.get('cargo') is False
            if self._carried.get('run') and (confirmed == 'D_2' or
                    (at_d2 and empty and phase in ('DONE', 'BACKING'))):
                # Retrieval BACKING with cargo=False follows the release.
                self._release = {'id': self._carried['id'], 'destination': 'D_2', 'pose': None,
                                 'run': self._carried['run']}
                age = event.get('age_s') if confirmed == 'D_2' else None
                self._released_at = self._at - (age if type(age) in (float, int) and
                                                 math.isfinite(age) and age > 0 else 0)
            elif (status.get('mode') == 'drop' and isinstance(drop, dict) and drop.get('slot') and
                  (confirmed == 'rack_slot' or phase == 'BACKING' or (phase == 'DONE' and empty))):
                destination = drop['slot']
                self._drops[destination] = {**self._carried, 'slot': destination,
                                            'seen_full': states.get(destination) == 'full'}
                self._departed.pop(destination, None)
                self._release = {'id': self._carried['id'], 'destination': 'rack_slot',
                                 'slot': destination, 'pose': None, 'run': self._carried.get('run')}
            elif phase == 'DONE' and empty and status.get('mode') == 'pick' and not self._carried.get('run'):
                self._release = {'id': self._carried['id'], 'destination': 'unlocalized', 'pose': None}
            else:
                return
            # A release confirms ownership ended, even when pose is absent.
            self._release_key = key
            self._carried = None

    def snapshot(self, map_id, frame_id):
        with self._lock:
            if map_id != self._map:
                self._select_map(map_id)
            if not map_id:
                return None
            mission = self._mission or {}
            now = time.monotonic()
            fresh = now - self._at <= _TARGET_FRESH_S
            states = self._states(mission) if fresh else {}
            target = self._target
            if target and not target.get('locked') and (not fresh or now - self._target_at > _TARGET_FRESH_S):
                target = None
            drop = mission.get('drop') if mission.get('mode') == 'drop' and self._carried else None
            transfer = None
            motion = mission.get('visual_lift')
            if fresh and isinstance(motion, dict) and all(
                    type(motion.get(k)) in (float, int) and math.isfinite(motion[k])
                    for k in ('start_m', 'goal_m', 'height_m')):
                if (motion.get('action') == 'pick_backoff' and mission.get('us_pick_confirmed') is True
                        and target and target.get('locked') and not self._carried
                        and motion['goal_m'] > motion['start_m']):
                    transfer = {**motion, 'id': target['id']}
                elif (motion.get('action') == 'drop_release' and drop and drop.get('slot')
                      and motion['goal_m'] < motion['start_m']):
                    transfer = {**motion, 'id': self._carried['id'], 'slot': drop['slot']}
            return {'map_id': map_id, 'frame_id': frame_id, 'slots': states,
                    'lift_transfer': transfer,
                    'target': dict(target) if target else None,
                    'carried': self._carried, 'source_empty': self._source_empty,
                    'drops': dict(self._drops), 'departed': dict(self._departed),
                    'pending_slot': drop.get('slot') if isinstance(drop, dict) else None,
                    'release': self._release,
                    'release_age_s': (now - self._released_at if self._release and
                                      self._release['destination'] == 'D_2' else None)}
