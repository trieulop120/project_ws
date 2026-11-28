"""Keep confirmed MANUAL plans in the LLM node's current session."""
import math
import re
import time
from copy import deepcopy

try:
    from .robot_queries import normalize
except ImportError:
    from robot_queries import normalize


def resume_requested(text):
    return bool(re.fullmatch(r'(?:(?:cho xe )?chay tiep(?: di)?|resume(?: (?:plan|recovery|mission))?|tiep tuc(?: (?:ke hoach|lenh|cong viec|mission))?)[.!?]*', normalize(text).strip()))


def resume_scope(text):
    text = normalize(text)
    if re.search(r'\b(?:plan|ke hoach|lenh)\b', text):
        return 'manual'
    return 'mission' if re.search(r'\bmission\b', text) else 'owner'


class ManualRecovery:
    def _manual_resume_blocked(self):
        cp = getattr(self, '_manual_checkpoint', None)
        if cp is None or cp.get('paused_at') is None:
            return 'No paused MANUAL plan'
        now = time.monotonic()
        if now - cp['paused_at'] > 120.:
            return 'MANUAL checkpoint expired; inspect robot and issue a new plan'
        if self._op_mode != 'MANUAL':
            return 'MANUAL mode changed since the plan stopped'
        if self._odom_pose is None or now - self._odom_t > .5:
            return 'Fresh odometry required to resume the MANUAL plan'
        if self._odom_still_since is None or now - self._odom_still_since < 1.:
            return 'Robot must remain stationary before Resume'
        if self._lift_cm is None or now - self._lift_t > 1.:
            return 'Fresh lift joint_states required before Resume'
        if (self._lift_still_since is None
                or now - self._lift_still_since < float(self.get_parameter('lift_stable_s').value)):
            return 'Lift must remain stationary before Resume'
        pose, previous = self._odom_pose, cp.get('paused_pose')
        if previous is None or not all(math.isfinite(v) for v in (*pose, *previous)):
            return 'Missing finite odometry at STOP or Resume'
        yaw = abs(math.atan2(math.sin(pose[2] - previous[2]), math.cos(pose[2] - previous[2])))
        if math.hypot(pose[0] - previous[0], pose[1] - previous[1]) > .03 or yaw > math.radians(3):
            return 'Robot moved or odometry changed after the MANUAL plan stopped'
        _, nav_state = self._nav_latest()
        if nav_state in (1, 2, 3):
            return 'Previous Nav2 goal has not ended'
        if self._manager_busy is not False or self._manager_status_t < self._stop_requested_at:
            return 'Waiting for mission_manager to release pending navigation'
        for key in ('_manager_stop_future', '_pallet_stop_future'):
            future = getattr(self, key, None)
            if future is not None and not future.done():
                return 'Waiting for all STOP service requests to finish'
        if (getattr(self, '_pallet_stop_future', None) is not None
                and self._pallet_state_t < self._stop_requested_wall):
            return 'Waiting for pallet status after STOP'
        if ((self._pallet_status or {}).get('recovery') or {}).get('stop_pending'):
            return 'Waiting for pallet STOP requests to finish'
        if time.time() - self._sup_state_t > self.get_parameter('supervisor_status_max_age_s').value:
            return 'Supervisor telemetry is stale'
        if self._sup_state not in ('IDLE', 'DONE', 'FAILED', 'STOPPED'):
            return 'Supervisor owns recovery; resume its mission before a MANUAL plan'
        if getattr(self, '_retrieval_state', None) not in (None, 'IDLE', 'DONE', 'FAILED', 'CANCELED', 'STOPPED'):
            return 'Retrieval owns recovery; resume its coordinator first'
        if (self._pallet_status or {}).get('session_id') != cp.get('pallet_session'):
            return 'Pallet node restarted; inspect before a new plan'
        action = cp['steps'][cp['index']]['action']
        if action in ('pick_pallet', 'drop_pallet', 'lock_detection'):
            if time.time() - self._pallet_state_t > 3.:
                return 'Pallet telemetry is stale'
            recovery = (self._pallet_status or {}).get('recovery') or {}
            if recovery.get('checkpoint') and not recovery.get('available'):
                return recovery.get('reason') or 'Pallet operation cannot resume'
        return ''

    def _begin_manual_checkpoint(self, steps, resume_checkpoint=None):
        if resume_checkpoint is not None:
            if self._manual_checkpoint is not resume_checkpoint:
                raise RuntimeError('MANUAL checkpoint changed while awaiting confirmation')
            reason = self._manual_resume_blocked()
            if reason:
                raise RuntimeError(reason)
            return resume_checkpoint
        # A cycle is owned by its coordinator, not by this manual-plan journal.
        if any(s['action'] in ('start_auto_cycle', 'start_single_cycle', 'retrieve_pallet',
                              'resume_cycle', 'stop_cycle') for s in steps):
            return None
        cp = {'steps': deepcopy(steps), 'index': 0, 'started': False,
              'progress': {}, 'paused_at': None, 'paused_pose': None,
              'pallet_session': (getattr(self, '_pallet_status', None) or {}).get('session_id')}
        self._manual_checkpoint = cp
        return cp

    def _freeze_manual_checkpoint(self, cp):
        self._resume_step = False
        if cp is None:
            return
        if cp['index'] >= len(cp['steps']):
            self._manual_checkpoint = None
            return
        cp['paused_at'] = time.monotonic()
        cp['paused_pose'] = getattr(self, '_odom_pose', None)
        # Require new measured stop stability after the interrupted handler ends.
        self._odom_still_since = None
        self._lift_still_since = None
