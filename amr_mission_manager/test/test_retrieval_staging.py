"""Exercise source state machines with action/telemetry fakes, without ROS I/O."""
import ast
import __future__
from concurrent.futures import Future
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock


ROOT = Path(__file__).resolve().parents[1] / 'amr_mission_manager'


def pose_message():
    return NS(header=NS(), pose=NS(position=NS(), orientation=NS()))


def source_classes(clock):
    namespace = dict(math=math, time=clock, Node=object, PoseStamped=pose_message,
                     NavigateToPose=NS(Goal=lambda: NS()),
                     GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6),
                     Bool=lambda **kw: NS(**kw), String=lambda **kw: NS(**kw))
    for filename in ('pallet_recovery.py', 'pallet_mission_node.py'):
        tree = ast.parse((ROOT / filename).read_text())
        nodes = [n for n in tree.body if isinstance(n, (ast.Assign, ast.FunctionDef, ast.ClassDef))]
        exec(compile(ast.Module(body=nodes, type_ignores=[]), filename, 'exec',
                     flags=__future__.annotations.compiler_flag), namespace)
    return namespace


class Handle:
    def __init__(self, accepted=True):
        self.accepted = accepted
        self.result = Future()
        self.cancel = Mock(return_value=NS(goals_canceling=['goal']))

    def get_result_async(self):
        return self.result

    def cancel_goal_async(self):
        return self.cancel()


class RetrievalStagingTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.clock = NS(monotonic=lambda: self.now, sleep=lambda _: None)
        self.ns = source_classes(self.clock)
        self.node = n = self.ns['PalletMission'].__new__(self.ns['PalletMission'])
        n.p = dict(self.ns['DEFAULTS'])
        n.stage_m = .9  # a different PICK setting must not change retrieval's .65 m
        n._state, n._mode = 'READY', 'retrieval_align'
        n._checkpoint = None
        n._operation_id = 0
        n._det_proc = None
        n._resume_operation_id = None
        n._retrieval_mode = False
        n._retrieval_align_only = False
        n._retrieval_slot = 'A1'
        n._retrieval_pose = (1.2, 0., .1, 0.)
        n._retrieval_pose_t = self.now
        n._shelf = NS(_slot_by_id={slot: NS(id=slot, product_type='A' if slot[0] == 'A' else 'B')
                                  for slot in ('A1', 'A2', 'B1', 'B2')})
        n._shelf_cfg = {'slot_level': {'A1': 1, 'A2': 2, 'B1': 1, 'B2': 1},
                        'levels': {1: {'insert_lift_cm': 25., 'release_lift_cm': 20.},
                                   2: {'insert_lift_cm': 60., 'release_lift_cm': 55.}}}
        n._cargo = False
        n._goal_handle = None
        n._nav_pending = None
        n._nav_results = []
        n._nav_epoch = 0
        n._drop_nav = None
        n._retrieval_pick_nav = None
        n._retrieval_pick_nav_elapsed = 0.
        n._retrieval_nav_t = self.now
        n._retrieval_nav_active = False
        n._tf_miss = 0
        n._tf_wait_since = None
        n._range_wait_since = None
        n._waiting_sensor = ''
        n._sensor_return_state = None
        n._ctl = self.ns['ApproachController'](n.p, 1.2, align_only=True)
        n._drop_odom_t = self.now
        n._drop_odom_speed = (0., 0.)
        self.robot = (0., 0., 0.)
        n._robot_pose = Mock(side_effect=lambda frame: self.robot)
        n._map_to_ins = Mock(side_effect=lambda x, y, yaw: (x, y, yaw))
        n._lift_height_now = Mock(return_value=0.)
        n.get_logger = Mock(return_value=Mock())
        n.get_clock = Mock(return_value=NS(now=lambda: NS(to_msg=lambda: 'stamp')))
        n._bt_path = Mock(return_value='/navigation/return_home.xml')
        n._publish_status = Mock()
        n._cancel_all_nav = Mock()
        n._halt = Mock()
        n._lift_stop = Mock()
        n._begin_lift = Mock()
        n._reap_detector = Mock()
        n.pub_detector_enable = NS(publish=Mock())
        n.pub_cmd = NS(get_subscription_count=lambda: 1)
        n.pub_esp32 = NS(get_subscription_count=lambda: 1)
        self.handle = Handle()
        self.sent = []

        def send(goal):
            self.sent.append(goal)
            future = Future()
            future.set_result(self.handle)
            return future

        n._nav = NS(server_is_ready=lambda: True, send_goal_async=Mock(side_effect=send))

    def start(self):
        response = self.node._srv_retrieval_pick(None, NS())
        self.assertTrue(response.success, response.message)
        self.assertEqual(self.node._state, 'NAVIGATING')

    def terminal(self, status=4):
        self.robot = self.node._pre
        self.handle.result.set_result(NS(status=status))

    def tick(self, dt=0., fresh=True):
        self.now += dt
        if fresh:
            self.node._drop_odom_t = self.now
        self.node._tick()

    def align(self):
        self.start()
        self.terminal()
        self.tick()
        self.tick(1.01)
        self.assertEqual(self.node._state, 'ALIGNING')

    def test_all_slots_rotated_camera_goal_and_lowered_forks(self):
        for slot in ('A1', 'A2', 'B1', 'B2'):
            for yaw in (0., math.pi/2, -math.pi/3):
                self.setUp()
                n = self.node
                n._retrieval_slot = slot
                n._retrieval_pose = (2., 3., .65, yaw)
                self.start()
                expected = (2.-.65*math.cos(yaw), 3.-.65*math.sin(yaw), yaw)
                self.assertEqual(n._pre, expected)
                goal = self.sent[-1]
                self.assertEqual(goal.pose.header.frame_id, 'map')
                self.assertAlmostEqual(goal.pose.pose.position.x, expected[0])
                self.assertAlmostEqual(goal.pose.pose.position.y, expected[1])
                self.assertEqual(goal.behavior_tree, '/navigation/return_home.xml')
                n._begin_lift.assert_not_called()
                self.assertTrue(n._ctl.align_only)  # no new approach controller yet

    def test_terminal_and_measured_stop_then_existing_alignment_and_lift(self):
        self.align()
        n = self.node
        self.assertEqual(n._ctl.stage_m, .65)
        self.assertFalse(n._ctl.align_only)
        self.assertEqual(n._line, (1.2, 0., 0.))
        n._begin_lift.assert_not_called()
        n._ctl.phase = n._ctl.CONVERGE
        n._ctl.t_phase = self.now
        n._us_t = self.now
        n._us_count = 0
        n._control(self.now)
        n._begin_lift.assert_called_once_with(20., nxt='retrieval_insert',
                                             label='Alignment complete; raise forks to 20.0 cm')
        n._lift_next = 'retrieval_insert'
        n._on_lift_reached(20.)
        self.assertTrue(n._ctl.insertion_only)
        self.assertEqual(n._ctl.phase, n._ctl.CONVERGE)

    def test_handoff_cancel_ack_does_not_start_alignment(self):
        self.start()
        self.robot = self.node._pre
        self.tick()
        self.tick(4.01)
        self.handle.cancel.assert_called_once()
        self.assertEqual(self.node._state, 'NAVIGATING')
        self.node._begin_lift.assert_not_called()
        self.terminal(status=5)
        self.tick()
        self.tick(1.01)
        self.assertEqual(self.node._state, 'ALIGNING')

    def test_nav_goal_must_end_and_status_release_ownership(self):
        self.start()
        self.node._retrieval_nav_active = True
        self.terminal()
        self.tick()
        self.tick(2.)
        self.assertEqual(self.node._state, 'SETTLING')
        self.node._retrieval_nav_active = False
        self.tick()
        self.tick(1.01)
        self.assertEqual(self.node._state, 'ALIGNING')

    def test_moving_stale_or_discontinuous_odom_never_hands_off(self):
        for fault in ('moving', 'stale', 'interrupted'):
            with self.subTest(fault=fault):
                self.setUp()
                self.start()
                self.terminal()
                self.tick()
                if fault == 'moving':
                    self.node._drop_odom_speed = (.03, 0.)
                    self.tick(1.1)
                elif fault == 'stale':
                    self.tick(1.1, fresh=False)
                else:
                    self.node._drop_odom_speed = (0., .05)
                    self.tick(.6)
                    self.node._drop_odom_speed = (0., 0.)
                    self.tick(.6)
                self.assertEqual(self.node._state, 'SETTLING')
                self.node._begin_lift.assert_not_called()

    def test_rejected_external_cancel_abort_and_bad_geometry(self):
        for fault in ('rejected', 'external_cancel', 'far_abort', 'near_abort', 'clearance', 'yaw', 'missing_tf'):
            with self.subTest(fault=fault):
                self.setUp()
                if fault == 'rejected':
                    self.handle.accepted = False
                    self.node._srv_retrieval_pick(None, NS())
                else:
                    self.start()
                    self.robot = self.node._pre
                    status = 4
                    if fault == 'external_cancel':
                        status = 5
                    elif fault == 'far_abort':
                        self.robot = (0., 0., 0.)  # original 1.20 m stance is not staging
                        status = 6
                    elif fault == 'near_abort':
                        status = 6
                    elif fault == 'clearance':
                        self.robot = (.61, 0., 0.)  # only .59 m from face, inside rotation clearance
                    elif fault == 'yaw':
                        self.robot = (*self.robot[:2], math.radians(26))
                    elif fault == 'missing_tf':
                        self.node._robot_pose = Mock(return_value=None)
                    self.handle.result.set_result(NS(status=status))
                expected = 'SETTLING' if fault == 'near_abort' else 'STOPPED' if fault == 'external_cancel' else 'FAILED'
                self.assertEqual(self.node._state, expected)
                self.node._begin_lift.assert_not_called()

    def test_timeout_cancel_timeout_and_lift_fault_stop(self):
        for fault in ('response', 'navigation', 'cancel', 'stop', 'lift'):
            with self.subTest(fault=fault):
                self.setUp()
                if fault == 'response':
                    self.node._nav.send_goal_async = Mock(return_value=Future())
                self.start()
                if fault in ('response', 'navigation'):
                    self.tick(181.)
                elif fault == 'cancel':
                    self.robot = self.node._pre
                    self.tick()
                    self.tick(4.01)
                    self.tick(10.01)
                elif fault == 'stop':
                    self.terminal()
                    self.node._drop_odom_speed = (.04, 0.)
                    self.tick(10.01)
                else:
                    self.node._lift_height_now.return_value = None
                    self.tick()
                self.assertEqual(self.node._state, 'FAILED')
                self.assertEqual(self.node._fail_phase, 'RETRIEVAL_NAV')
                self.assertIsNotNone(self.node._checkpoint)

    def test_map_odom_drift_rejected_without_overwriting_locked_pose(self):
        self.start()
        stored = self.node._pallet_ins
        self.terminal()
        self.tick()
        self.node._map_to_ins.side_effect = lambda x, y, yaw: (x+.06, y, yaw)
        self.tick(1.01)
        self.assertEqual(self.node._state, 'FAILED')
        self.assertEqual(self.node._pallet_ins, stored)

    def test_stop_resume_retains_target_flags_and_active_time(self):
        self.start()
        self.now += 12.
        self.node._emergency_stop()
        self.assertEqual(self.node._checkpoint['state'], 'NAVIGATING')
        self.assertEqual(self.node._retrieval_pick_nav_elapsed, 12.)
        self.assertTrue(self.node._retrieval_mode)
        self.handle.result.set_result(NS(status=5))
        self.node._recovery_blocked = Mock(return_value='')
        self.node._us_count = 0
        self.handle = Handle()
        result = self.node._srv_resume(None, NS())
        self.assertTrue(result.success, result.message)
        self.assertEqual(self.node._state, 'NAVIGATING')
        self.assertEqual(self.node._retrieval_pick_nav_elapsed, 12.)
        self.assertEqual(self.node._retrieval_pick_lift_cm, 20.)
        self.assertEqual(self.sent[0].pose.pose.position.x, self.sent[1].pose.pose.position.x)
        self.node._begin_lift.assert_not_called()

    def test_late_goal_after_stop_is_canceled_and_does_not_restart(self):
        pending = Future()
        self.node._nav.send_goal_async = Mock(return_value=pending)
        self.start()
        self.node._emergency_stop()
        pending.set_result(self.handle)
        self.handle.cancel.assert_called_once()
        self.handle.result.set_result(NS(status=5))
        self.assertEqual(self.node._state, 'STOPPED')
        self.assertIsNone(self.node._goal_handle)

    def test_invalid_stage_lift_and_navigation_ownership_refused(self):
        for fault in ('stage', 'nan', 'lift', 'active', 'pending', 'server'):
            with self.subTest(fault=fault):
                self.setUp()
                n = self.node
                if fault == 'stage': n.p['retrieval_stage_m'] = .5
                if fault == 'nan': n.p['retrieval_nav_timeout_s'] = float('nan')
                if fault == 'lift': n._lift_height_now.return_value = 20.
                if fault == 'active': n._retrieval_nav_active = True
                if fault == 'pending': n._nav_results = [Future()]
                if fault == 'server': n._nav.server_is_ready = lambda: False
                response = n._srv_retrieval_pick(None, NS())
                self.assertFalse(response.success)
                self.assertEqual(self.sent, [])
                self.assertFalse(n._retrieval_mode)
                self.assertEqual(n._mode, 'retrieval_align')
                n._begin_lift.assert_not_called()

    def test_send_failure_retains_an_accepted_operation_for_coordinator_resume(self):
        self.node._nav.send_goal_async = Mock(side_effect=RuntimeError('send failed'))
        response = self.node._srv_retrieval_pick(None, NS())
        self.assertTrue(response.success)
        self.assertEqual(self.node._state, 'FAILED')
        self.assertEqual(self.node._checkpoint['state'], 'NAVIGATING')

    def test_resume_waits_for_old_terminal_result_and_cannot_restart_after_lift(self):
        self.start()
        self.node._emergency_stop()
        self.node._recovery_blocked = Mock(return_value='')
        result = self.node._srv_resume(None, NS())
        self.assertFalse(result.success)
        self.assertIsNotNone(self.node._checkpoint)
        self.assertEqual(len(self.sent), 1)
        self.handle.result.set_result(NS(status=5))
        self.node._retrieval_pick_lift_cm = None
        result = self.node._srv_resume(None, NS())
        self.assertFalse(result.success)
        self.assertEqual(len(self.sent), 1)

    def test_normal_pick_and_drop_keep_their_existing_tick_paths(self):
        n = self.node
        n._retrieval_mode = False
        n._state = 'NAVIGATING'
        n._handoff_check = Mock()
        n._retrieval_pick_staging_tick = Mock()
        n._mode = 'pick'
        n._tick()
        n._handoff_check.assert_called_once()
        n._drop_staging_tick = Mock()
        n._mode = 'drop'
        n._tick()
        n._drop_staging_tick.assert_called_once()
        n._retrieval_pick_staging_tick.assert_not_called()

    def test_coordinator_keeps_selected_target_and_waits_for_staging_plus_pick(self):
        tree = ast.parse((ROOT / 'pallet_retrieval_node.py').read_text())
        method = next(m for c in tree.body if isinstance(c, ast.ClassDef)
                      for m in c.body if isinstance(m, ast.FunctionDef) and m.name == '_run_step')
        namespace = dict(time=self.clock, String=lambda **kw: NS(**kw))
        exec(compile(ast.Module(body=[method], type_ignores=[]), 'coordinator', 'exec'), namespace)
        saved = object()
        coordinator = NS(_saved_target=saved, _selected_slot_pub=NS(publish=Mock()),
                         _selected_pose_pub=NS(publish=Mock()), _status=Mock(),
                         _pm_step=Mock(), _pick_cli=object(),
                         get_parameter=lambda _: NS(value=490.))
        namespace['_run_step'](coordinator, 5, 'A2')
        coordinator._selected_pose_pub.publish.assert_called_once_with(saved)
        coordinator._pm_step.assert_called_once_with(coordinator._pick_cli, {'CARRYING'}, 490.)


if __name__ == '__main__':
    unittest.main()
