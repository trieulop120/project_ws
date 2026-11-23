"""Interrupt real plan/relative-move/lift handlers; no ROS or robot access."""
import math
import json
import threading
from copy import deepcopy
import time
from types import MethodType, SimpleNamespace as NS
import unittest
from unittest.mock import patch

from test_robot_queries import method
from manual_recovery import ManualRecovery, resume_requested, resume_scope
from llm_logic import Aborted, is_stop, validate_plan
from robot_queries import classify_query, answer_query


def twist():
    return NS(linear=NS(x=0.), angular=NS(z=0.))


RUN = method('llm_command_node.py', '_run_plan', {'Aborted': Aborted})
MOVE = method('llm_command_node.py', '_h_relative_move', dict(time=time, math=math, Twist=twist))
LIFT = method('llm_command_node.py', '_command_lift', dict(time=time, String=lambda **kw: NS(**kw)))


class FakeManual(ManualRecovery):
    def __init__(self, clock):
        self.clock = clock
        self._manual_checkpoint, self._resume_step = None, False
        self._manager_busy, self._manager_status_t, self._stop_requested_at = False, clock[0], 0.
        self._odom_pose, self._odom_t = (0., 0., 0.), clock[0]
        self._odom_still_since = clock[0] - 2.
        self._lift_cm, self._lift_t, self._lift_still_since = 0., clock[0], clock[0] - 2.
        self._sup_state, self._sup_state_t, self._op_mode = 'IDLE', time.time(), 'MANUAL'
        self._pallet_status, self._pallet_state_t = {}, time.time()
        self._location = 'home'
        self.stop_at, self.vx = .2, 0.
        self.events, self.lift_commands = [], []
        self._h_relative_move = MethodType(MOVE, self)
        for name in ('set_mode', 'quick_goal', 'start_auto_cycle', 'start_single_cycle',
                     'retrieve_pallet', 'stop_cycle', 'cancel_navigation', 'resume_cycle',
                     'go_mission', 'pick_pallet', 'detect_pallet', 'lock_detection',
                     'drop_pallet', 'lift_to', 'lift_home'):
            setattr(self, '_h_' + name, lambda args, name=name: self.events.append(name) or True)
        self.pub_cmd = NS(publish=self.publish)
        self.pub_lift = NS(publish=lambda msg: self.lift_commands.append(msg.data))

    def get_parameter(self, key):
        return NS(value={'dry_run': False, 'odom_max_age_s': .5, 'relative_move_timeout_s': 90.,
                         'supervisor_status_max_age_s': 5., 'lift_wait_s': 180., 'lift_stable_s': .5, 'lift_stable_tol_cm': .3}[key])
    def _is_en(self, _=None): return False
    def _status(self, *_args, **_kwargs): pass
    def _require_cycle_idle(self): pass
    def _ensure_manual_idle(self): pass
    def _nav_latest(self): return None, None
    def _check_abort(self):
        if self.stop_at is not None and abs(self._odom_pose[0]) >= self.stop_at:
            raise Aborted()
    def publish(self, msg): self.vx = msg.linear.x
    def _sleep(self, seconds):
        self.clock[0] += seconds
        self._odom_pose = (self._odom_pose[0] + self.vx * seconds, self._odom_pose[1], self._odom_pose[2])
        self._odom_t = self._lift_t = self.clock[0]
    def settle(self):
        self.clock[0] += 2.
        self._odom_t = self._lift_t = self.clock[0]
        self._odom_still_since = self._lift_still_since = self.clock[0] - 1.5


class ManualRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.clock = [100.]
        timer = patch('time.monotonic', side_effect=lambda: self.clock[0])
        timer.start(); self.addCleanup(timer.stop)
        self.n = FakeManual(self.clock)

    def test_reverse_resumes_remaining_30cm_and_skips_completed_lift(self):
        plan = [{'action': 'lift_home', 'args': {}},
                {'action': 'relative_move', 'args': {'direction': 'backward', 'distance_m': .5, 'speed_mps': .1}},
                {'action': 'quick_goal', 'args': {'target': 'HOME'}}]
        with self.assertRaises(Aborted): RUN(self.n, plan)
        cp = self.n._manual_checkpoint
        self.assertEqual(cp['index'], 1)
        self.assertEqual(self.n.events, ['lift_home'])
        self.assertEqual(self.n.vx, 0.)
        self.n.stop_at = None
        self.n.settle()
        RUN(self.n, cp['steps'][cp['index']:], resume_checkpoint=cp)
        self.assertIsNone(self.n._manual_checkpoint)
        self.assertAlmostEqual(self.n._odom_pose[0], -.5, delta=.016)
        self.assertEqual(self.n.events, ['lift_home', 'quick_goal'])

    def test_motion_drift_fails_and_zeroes_velocity_without_running_next_step(self):
        self.n._odom_pose = (0., .1, 0.)
        cp = {'steps': [{'action': 'relative_move', 'args': {}}], 'index': 0,
              'progress': {'origin': (0., 0., 0.)}}
        self.n._manual_checkpoint, self.n._resume_step = cp, True
        with self.assertRaisesRegex(RuntimeError, 'drift'):
            MOVE(self.n, {'direction': 'backward', 'distance_m': .5, 'speed_mps': .1})
        self.assertEqual(self.n.vx, 0.)

    def test_resume_requires_fresh_stable_sensors_and_unchanged_odometry(self):
        cp = self.n._begin_manual_checkpoint([{'action': 'lift_home', 'args': {}}])
        self.n._freeze_manual_checkpoint(cp)
        self.assertIn('stationary', self.n._manual_resume_blocked())
        self.n.settle()
        self.assertEqual(self.n._manual_resume_blocked(), '')
        self.n._odom_pose = (.1, 0., 0.)
        self.assertIn('odometry changed', self.n._manual_resume_blocked())
        self.n._odom_pose = (0., 0., 0.)
        self.n._lift_t -= 10.
        self.assertIn('joint_states', self.n._manual_resume_blocked())

    def test_lift_at_target_waits_for_new_joint_sample_without_sending_home_again(self):
        n = self.n
        n._manual_checkpoint = n._begin_manual_checkpoint([{'action': 'lift_home', 'args': {}}])
        calls = []
        sleep_original = n._sleep
        def advance(seconds):
            calls.append(seconds)
            sleep_original(seconds)
        n._sleep = advance
        self.assertTrue(LIFT(n, 'HOME', 0.))
        self.assertTrue(calls)
        self.assertEqual(n.lift_commands, [])

    def test_lift_failure_stops_motor_and_keeps_original_target(self):
        n = self.n
        n._manual_checkpoint = n._begin_manual_checkpoint([{'action': 'lift_to', 'args': {'height_cm': 25}}])
        n._check_abort = lambda: (_ for _ in ()).throw(Aborted())
        with self.assertRaises(Aborted): LIFT(n, 'H25', 25.)
        self.assertEqual(n.lift_commands, ['H25', 'M0'])
        self.assertEqual(n._manual_checkpoint['progress']['target_cm'], 25.)

    def test_resume_words_are_exact_and_not_mixed_motion_requests(self):
        for text in ('resume', 'tiếp tục', 'tiếp tục kế hoạch', 'Resume plan!'):
            self.assertTrue(resume_requested(text))
        for text in ('resume rồi lấy pallet', 'tiếp tục và lùi 50cm'):
            self.assertFalse(resume_requested(text))

    def test_llm_resume_reaches_confirmation_while_supervisor_is_in_recovery(self):
        starts = []
        fn = method('llm_command_node.py', '_on_command', dict(json=json,
                    threading=NS(Thread=lambda **kw: NS(start=lambda: starts.append(kw))),
                    is_stop=is_stop, classify_query=classify_query, answer_query=answer_query,
                    resume_requested=resume_requested))
        n = NS(_busy=threading.Lock(), _sup_state='RECOVERY', _current_lang='vi',
               _is_en=lambda *_: False, _is_cycle_active=lambda: True,
               _abort=threading.Event(), _confirm_evt=threading.Event(), _worker=lambda *_: None,
               _interrupt_block_reason=lambda *_: (_ for _ in ()).throw(AssertionError('Resume must not be treated as interruption')))
        fn(n, NS(data='resume'))
        self.assertEqual(len(starts), 1)
        n._busy.release()

    def test_exact_resume_uses_saved_steps_without_api_and_still_requires_confirmation(self):
        fn = method('llm_command_node.py', '_worker', dict(time=time, json=json, deepcopy=deepcopy,
                    resume_requested=resume_requested, resume_scope=resume_scope, validate_plan=validate_plan,
                    answer_query=answer_query, Aborted=Aborted, String=lambda **kw: NS(**kw)))
        for manual in (False, True):
            plans, statuses = [], []
            busy = threading.Lock(); busy.acquire()
            cp = {'steps': [{'action': 'lift_home', 'args': {}}, {'action': 'relative_move', 'args': {
                  'direction': 'backward', 'distance_m': .5}}], 'index': 1} if manual else None
            n = NS(_is_en=lambda *_: False, _clarify_ctx=None, _manual_checkpoint=cp,
                   _sup_state='IDLE' if manual else 'RECOVERY', _retrieval_state=None,
                   _manual_resume_blocked=lambda: '', _location='home', _busy=busy,
                   _switching_command=False, _confirm_evt=threading.Event(),
                   _plan=lambda *_: (_ for _ in ()).throw(AssertionError('Must not call an API for exact Resume')),
                   _status=lambda state, **kw: statuses.append(state),
                   _action_summary=lambda *_: ['Resume remaining steps'],
                   _wait_confirm=lambda: False, pub_plan=NS(publish=lambda msg: plans.append(json.loads(msg.data))),
                   get_parameter=lambda key: NS(value=8 if key == 'max_steps' else key == 'require_confirm'),
                   get_logger=lambda: NS(info=lambda *_: None, error=lambda *_: None, warning=lambda *_: None))
            fn(n, 'resume', 'vi')
            self.assertIn('waiting_confirm', statuses)
            self.assertNotIn('running', statuses)
            self.assertEqual(plans[0]['steps'][0]['action'], 'relative_move' if manual else 'resume_cycle')
            self.assertIs(n._manual_checkpoint, cp)
            self.assertFalse(busy.locked())

    def test_timed_out_service_does_not_trigger_emergency_stop(self):
        callbacks, stops = [], []
        event = NS(wait=lambda _: False, set=lambda: None)
        fn = method('llm_command_node.py', '_call', dict(threading=NS(Event=lambda: event), Aborted=Aborted))
        client = NS(srv_name='lift', wait_for_service=lambda **_: True,
                    call_async=lambda _: NS(add_done_callback=callbacks.append))
        n = NS(_check_abort=lambda: None, _stop_requested_at=0.,
               get_logger=lambda: NS(warn=lambda *_: None))
        def stop(reason):
            stops.append(reason); n._stop_requested_at += 1.
        n._emergency_stop = stop
        with self.assertRaises(TimeoutError): fn(n, client, NS(), timeout=.01)
        callbacks[0](NS(result=lambda: NS(success=True)))
        self.assertEqual(stops, [])


if __name__ == '__main__': unittest.main()
