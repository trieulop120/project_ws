"""Regression tests for bounded PICK/DROP alignment recovery without ROS I/O."""
import ast
import __future__
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock


ROOT = Path(__file__).resolve().parents[1] / 'amr_mission_manager'


def setattr_many(obj, state, msg):
    obj._state, obj._msg = state, msg


class Clock:
    now = 100.0

    @classmethod
    def monotonic(cls):
        return cls.now


def source_classes():
    namespace = dict(math=math, time=Clock, Node=object,
                     PoseStamped=lambda: NS(header=NS(), pose=NS(position=NS(), orientation=NS())),
                     NavigateToPose=NS(Goal=lambda: NS()),
                     GoalStatus=NS(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6),
                     Bool=lambda **kw: NS(**kw), String=lambda **kw: NS(**kw))
    for filename in ('pallet_recovery.py', 'pallet_mission_node.py'):
        tree = ast.parse((ROOT / filename).read_text())
        nodes = [n for n in tree.body if isinstance(n, (ast.Assign, ast.FunctionDef, ast.ClassDef))]
        exec(compile(ast.Module(body=nodes, type_ignores=[]), filename, 'exec',
                     flags=__future__.annotations.compiler_flag), namespace)
    return namespace


class AlignmentRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.ns = source_classes()
        cls.p = dict(cls.ns['DEFAULTS'])

    def test_pick_gate_retry_reuses_retreat_align_cycle(self):
        ctl = self.ns['ApproachController'](dict(self.p), 0.65)
        ctl.phase = ctl.GATE
        ctl.t_phase = 0.0
        ctl.sidesteps = 1
        a_gate = -(self.p['fork_front_m'] + self.p['gate_margin_m'])
        ctl.step(1.0, (a_gate, 0.05, 0.0), (0., 0., 0.), 0, False)
        self.assertEqual(ctl.phase, ctl.RETREAT)
        self.assertEqual(ctl.gate_retries, 1)
        self.assertEqual(ctl.sidesteps, 1)
        self.assertEqual(ctl.alignment_retries, 1)

    def test_drop_pre_lift_retry_is_bounded_and_requires_improvement(self):
        ctl = self.ns['ApproachController'](dict(self.p), 0.65)
        ctl.phase = ctl.ALIGN
        ctl.sidesteps = 2
        mission = self.ns['PalletMission'].__new__(self.ns['PalletMission'])
        mission._mode, mission._drop_lifted = 'drop', False
        mission._ctl = ctl
        mission._halt, mission._set_state = Mock(), Mock()
        mission._phase_shown = None
        mission.get_logger = Mock(return_value=Mock())
        ctl.err = (0.04, 0.04, -0.70)
        ctl._progress_phase = ctl.ALIGN
        ctl._progress_start_metric, ctl._progress_metric_now = 1.0, 0.8
        ok = mission._try_alignment_recovery(1.0, (0., 0., 0.), 'Xoay ALIGN failed')
        self.assertTrue(ok)
        self.assertEqual(ctl.phase, ctl.RETREAT)
        self.assertEqual(ctl.sidesteps, 2)  # retry cannot reset consumed lateral moves
        mission._set_state.assert_called_once()

        ctl.phase = ctl.ALIGN
        ok, _ = ctl.begin_drop_alignment_retry(2.0, (0., 0., 0.), (0.039, 0.039, -0.70))
        self.assertFalse(ok)  # no measurable improvement
        ok, _ = ctl.begin_drop_alignment_retry(2.0, (0., 0., 0.), (0.025, 0.025, -0.70))
        self.assertTrue(ok)
        self.assertEqual(ctl.alignment_retries, 2)
        ctl.phase = ctl.ALIGN
        ok, why = ctl.begin_drop_alignment_retry(3.0, (0., 0., 0.), (0.01, 0.01, -0.70))
        self.assertFalse(ok)
        self.assertIn('budget exhausted', why)

    def test_alignment_timeout_without_measured_progress_does_not_retry(self):
        mission = self.ns['PalletMission'].__new__(self.ns['PalletMission'])
        mission._mode, mission._drop_lifted = 'pick', False
        mission._ctl = self.ns['ApproachController'](dict(self.p), 0.65)
        mission._ctl.phase = mission._ctl.ALIGN
        mission._ctl._progress_phase = mission._ctl.ALIGN
        mission._ctl._progress_start_metric = 1.0
        mission._ctl._progress_metric_now = 1.0
        mission.get_logger = Mock(return_value=Mock())
        mission._halt, mission._set_state = Mock(), Mock()
        ok = mission._try_alignment_recovery(
            10., (0., 0., 0.), 'Canh goc qua thoi gian')
        self.assertFalse(ok)
        ok = mission._try_alignment_recovery(
            10., (0., 0., 0.), 'Di ngang khong het lech')
        self.assertFalse(ok)
        mission._halt.assert_not_called()
        mission._set_state.assert_not_called()

    def test_sidestep_physical_distance_budget_includes_prior_retries(self):
        ctl = self.ns['ApproachController'](dict(self.p), 0.65)
        ctl.phase = ctl.SIDE_DRIVE
        ctl._ref = (0., 0.)
        ctl._side_dir = 1.0
        ctl._side_e0 = 0.2
        ctl.sidestep_distance_m = 0.59
        # This position is past the remaining 1 cm of the two-manoeuvre budget.
        _vx, _wz, _done, fail = ctl.step(
            1., (0.02, 0., 0.), (0., 0., 0.), 0, False)
        self.assertIn('tong quang duong', fail.lower())
        self.assertGreaterEqual(ctl.sidestep_distance_m, 0.61)

    def test_short_sensor_loss_holds_zero_then_fail_stops_at_bound(self):
        mission = self.ns['PalletMission'].__new__(self.ns['PalletMission'])
        mission._state, mission._msg = 'INSERTING', 'LOCK'
        mission._waiting_sensor, mission._sensor_return_state = '', None
        mission._range_wait_since = mission._tf_wait_since = None
        mission._cmd = Mock()
        mission._finish = Mock()
        mission._set_state = lambda state, msg: setattr_many(mission, state, msg)
        mission.get_logger = Mock(return_value=Mock())

        mission._wait_for_sensor('ultrasonic', 10., 'range stale', 2.)
        self.assertEqual(mission._state, 'WAITING_FOR_SENSOR')
        mission._wait_for_sensor('ultrasonic', 11.5, 'range stale', 2.)
        mission._finish.assert_not_called()
        mission._wait_for_sensor('ultrasonic', 12.1, 'range stale', 2.)
        mission._finish.assert_called_once()
        self.assertEqual(mission._cmd.call_count, 3)

    def test_missing_robot_tf_holds_zero_until_five_second_bound_then_fails(self):
        mission = self.ns['PalletMission'].__new__(self.ns['PalletMission'])
        mission.p = dict(self.p)
        mission._mode, mission._retrieval_mode = 'drop', False
        mission._state, mission._msg = 'INSERTING', 'GATE'
        mission._ctl = self.ns['ApproachController'](dict(self.p), 0.65)
        mission._ctl.phase = mission._ctl.GATE
        mission._tf_miss = 0
        mission._waiting_sensor, mission._sensor_return_state = '', None
        mission._tf_wait_since = mission._range_wait_since = None
        mission._robot_pose = Mock(return_value=None)
        mission._cmd, mission._finish = Mock(), Mock()
        mission._set_state = lambda state, msg: setattr_many(mission, state, msg)
        mission._approach_fail_phase = lambda: mission._ctl.phase
        mission.get_logger = Mock(return_value=Mock())

        for tick in range(99):
            mission._control(tick / 20.0)
        self.assertEqual(mission._state, 'WAITING_FOR_SENSOR')
        mission._finish.assert_not_called()
        mission._control(5.0)
        mission._finish.assert_called_once()
        self.assertEqual(mission._cmd.call_count, 100)

    def test_supervisor_never_starts_duplicate_drop_with_child_checkpoint(self):
        tree = ast.parse((ROOT / 'mission_supervisor_node.py').read_text())
        method = next(node for cls in tree.body if isinstance(cls, ast.ClassDef)
                      and cls.name == 'MissionSupervisor'
                      for node in cls.body if isinstance(node, ast.FunctionDef)
                      and node.name == '_can_auto_retry')
        namespace = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), 'mission_supervisor', 'exec'),
             namespace)
        supervisor = NS(_pm_recovery={'checkpoint': {'state': 'INSERTING'}},
                        _pm_insertion_only=True, _pm_fail_phase='GATE',
                        _cancel_requested=False, _pm_operation_id=18,
                        SAFE_RETRY_PHASES=('DETECT_DRIFT',),
                        p={'approach_retries': 2},
                        get_logger=Mock(return_value=Mock()))
        self.assertFalse(namespace['_can_auto_retry'](supervisor, 0))

    def test_controller_fails_closed_at_each_alignment_phase_boundary(self):
        scenarios = (
            ('ENTER', 1., (0., 0., 0.), {}, 'Mui cang'),
            ('RETREAT', 26., (-0.30, 0., 0.), {'_ref': (0., 0.)}, 'Lui qua xa/qua thoi gian'),
            ('ALIGN', 61., (-0.70, 0., 0.), {}, 'Canh goc qua thoi gian'),
            ('SIDE_TURN', 61., (-0.70, 0., 0.), {'_side_h': 0.}, 'Xoay de di ngang qua thoi gian'),
            ('SIDE_DRIVE', 21., (-0.70, 0.10, 0.),
             {'_ref': (-0.70, 0.), '_side_dir': 1., '_side_e0': .02}, 'Di ngang khong het lech'),
            ('CONVERGE', 61., (-0.70, 0., 0.), {}, 'Tien toi cong qua thoi gian'),
            ('GATE', 1., (-(self.p['fork_front_m'] + self.p['gate_margin_m']), .05, 0.),
             {'insertion_only': True}, 'Khong qua cong sau lift'),
            ('LOCK', 1., (-.30, .05, 0.), {'_a_lock0': -.30}, 'Lech trong luc vao'),
        )
        for phase, now, pose, attrs, expected in scenarios:
            with self.subTest(phase=phase):
                ctl = self.ns['ApproachController'](dict(self.p), .65)
                ctl.phase, ctl.t_start, ctl.t_phase = phase, 0., 0.
                for key, value in attrs.items():
                    setattr(ctl, key, value)
                _vx, _wz, _done, fail = ctl.step(now, pose, (0., 0., 0.), 0, False)
                self.assertIsNotNone(fail)
                self.assertIn(expected.lower(), fail.lower())

    def drop_resume_fixture(self, lateral_error):
        ns = self.ns
        n = ns['PalletMission'].__new__(ns['PalletMission'])
        now = Clock.now
        n.p = dict(self.p)
        n._state, n._mode = 'FAILED', 'drop'
        n._drop = {'slot': 'B2', 'insert_cm': 25.0}
        n._drop_lifted = True
        n._cargo = True
        n._route_path_loaded = 'route.yaml'
        n._checkpoint = {'id': 7, 'state': 'INSERTING', 'phase': 'GATE',
                         'operation_id': 21, 'time': now - 1.,
                         'pose': (-(self.p['fork_front_m'] + self.p['drop_gate_margin_m']),
                                  lateral_error, 0.),
                         'lift_cm': 25.0, 'cargo': True, 'route': 'route.yaml'}
        n._resume_max_pause_s = 120.0
        n._manager_busy, n._manager_status_t = False, now
        n._retrieval_nav_active, n._retrieval_nav_t = False, now
        n._nav_cancel_future = None
        n._nav_pending = None
        n._nav_results = []
        n._stop_request_futures = []
        n._drop_odom_t, n._drop_odom_speed = now, (0., 0.)
        n._resume_still_since, n._lift_still_since = now - 2., now - 2.
        n._lift_height_now = Mock(return_value=25.)
        n._lift_t0, n._bo_t0, n._bo_timeout = now - 3., now - 3., 30.
        n.pub_cmd = NS(get_subscription_count=lambda: 1)
        n.pub_esp32 = NS(get_subscription_count=lambda: 1)
        n._robot_pose = Mock(return_value=n._checkpoint['pose'])
        n._line = (0., 0., 0.)
        n._ctl = ns['ApproachController'](dict(self.p), 0.65)
        n._ctl.phase = n._ctl.GATE
        n._ctl.insertion_only = True
        n._ctl.t_start, n._ctl.t_phase = now - 5., now - 1.
        n._ctl.err = (lateral_error, 0., -0.52)
        n._us_t = now
        n._us_count = 3
        n._resume_operation_id = None
        n._last_resumed_checkpoint_id = 0
        n._phase_shown = None
        n.get_logger = Mock(return_value=Mock())
        n._set_state = Mock()
        return n

    def test_drop_b2_raised_25cm_resume_rechecks_gate_before_lock(self):
        # The support has no clearance sensor, so Resume is accepted only when
        # the *current* pose already meets GATE; it does not back out blindly.
        n = self.drop_resume_fixture(0.02)
        result = NS()
        n._srv_resume(None, result)
        self.assertTrue(result.success, result.message)
        self.assertIsNone(n._checkpoint)
        n._set_state.assert_called_with(
            'INSERTING', 'Resume rechecked current DROP GATE within tolerance; continuing LOCK')
        self.assertTrue(n._ctl.insertion_only)

    def test_drop_b2_raised_25cm_refuses_resume_without_clearance_evidence(self):
        n = self.drop_resume_fixture(0.05)
        checkpoint = n._checkpoint
        reason = n._recovery_blocked()
        self.assertIn('no sensor confirms pallet/fork clearance', reason)
        result = NS()
        n._srv_resume(None, result)
        self.assertFalse(result.success)
        self.assertIs(n._checkpoint, checkpoint)
        n._set_state.assert_not_called()


if __name__ == '__main__':
    unittest.main()
