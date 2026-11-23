"""Pure gate policy checks; no ROS nodes or robot commands."""
import ast
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_mission_manager.aruco_route_gate import ArucoRouteGate


class ArucoRouteGateTest(unittest.TestCase):
    def setUp(self):
        self.gate = ArucoRouteGate()

    def retrieval(self, state='NAVIGATING_TO_D1', leg='d1', run='retrieval-1', **kwargs):
        return self.gate.retrieval_transition(
            {'state': state, 'nav_leg': leg, 'run_id': run}, **kwargs)

    def test_placement_policy_unchanged(self):
        g = self.gate
        for previous, current, cargo, expected in [
            ('IDLE', 'GOTO_PICK', False, False),
            ('GOTO_PICK', 'PICKING', False, False),
            ('PICKING', 'GOTO_D', False, False),
            ('PICKING', 'GOTO_D', True, True),
            ('GOTO_D', 'RECOVERY', True, False),
            ('RECOVERY', 'GOTO_D', True, True),
        ]:
            self.assertEqual(g.transition(previous, current, cargo), expected)
        g.arrived_d1()
        self.assertFalse(g.transition('GOTO_D', 'GOTO_D', True))
        for state in ('DROPPING', 'RETURN_HOME', 'DONE', 'FAILED', 'STOPPED', 'IDLE'):
            self.assertFalse(g.transition('GOTO_D', state, True))
        self.assertFalse(g.transition('WAITING_SLOT', 'GOTO_D', True))
        self.assertTrue(g.transition('PICKING', 'GOTO_D', True))
        self.assertFalse(g.transition('GOTO_D', None, True))
        self.assertFalse(g.transition('GOTO_D', 'GOTO_D', True))

    def test_retrieval_d1_leg_and_arrival(self):
        self.assertFalse(self.retrieval('HOMING_LIFT', None, nav_goal_active=True))
        self.assertFalse(self.retrieval(nav_goal_active=False))
        self.assertTrue(self.retrieval(nav_goal_active=True))
        self.assertTrue(self.retrieval(nav_goal_active=True))  # heartbeat during travel
        self.assertFalse(self.retrieval(leg='slot', nav_goal_active=True))  # D_1 helper completed
        self.assertFalse(self.retrieval(nav_goal_active=True))  # same run cannot rearm
        self.assertFalse(self.gate.retrieval_transition(None, nav_goal_active=True))
        self.assertFalse(self.retrieval(nav_goal_active=True))  # missing heartbeat cannot erase arrival
        for state in ('ALIGNING_TO_SLOT', 'DETECTING', 'PICKING', 'NAVIGATING_TO_D2',
                      'PLACING_AT_D2', 'RETURNING_HOME', 'DONE'):
            self.assertFalse(self.retrieval(state, None, nav_goal_active=True), state)

    def test_resume_before_and_after_arrival(self):
        self.assertTrue(self.retrieval(nav_goal_active=True))
        self.assertFalse(self.retrieval('RECOVERY', None, nav_goal_active=True))
        self.assertFalse(self.retrieval(nav_goal_active=False))
        self.assertTrue(self.retrieval(nav_goal_active=True))
        self.assertFalse(self.retrieval(arrived_d1=True, nav_goal_active=True))
        self.assertFalse(self.retrieval('RECOVERY', None, nav_goal_active=True))
        self.assertFalse(self.retrieval(nav_goal_active=True))
        self.assertTrue(self.retrieval(run='retrieval-2', nav_goal_active=True))

    def test_stop_cancel_failure_and_missing_data_fail_closed(self):
        for state in ('STOPPING', 'STOPPED', 'CANCELING', 'CANCELED', 'FAILED', 'RECOVERY'):
            self.assertTrue(self.retrieval(run=state, nav_goal_active=True))
            self.assertFalse(self.retrieval(state, None, run=state, nav_goal_active=True))
        self.assertTrue(self.retrieval(nav_goal_active=True))
        self.assertFalse(self.gate.retrieval_transition(None, nav_goal_active=True))
        self.assertFalse(self.gate.enabled)
        self.assertFalse(self.retrieval(nav_goal_active=True, fresh=False))
        for status in ({}, {'state': 'NAVIGATING_TO_D1', 'nav_leg': 'd1'},
                       {'state': 'NAVIGATING_TO_D1', 'nav_leg': 'd1', 'run_id': None}):
            self.assertFalse(self.gate.retrieval_transition(status, nav_goal_active=True))
        self.assertFalse(self.retrieval(leg=None, nav_goal_active=True))
        self.assertFalse(self.retrieval(nav_goal_active=1))

    def test_sources_cannot_inherit_each_others_grant(self):
        self.assertTrue(self.gate.transition('PICKING', 'GOTO_D', True))
        self.assertFalse(self.retrieval('HOMING_LIFT', None))
        self.assertFalse(self.gate.transition('GOTO_D', 'GOTO_D', True))
        self.assertTrue(self.retrieval(nav_goal_active=True))
        self.assertFalse(self.gate.transition('IDLE', 'GOTO_PICK', False))
        self.assertFalse(self.gate.enabled)
        self.gate.reset()
        self.assertFalse(self.gate.enabled)

    def test_combined_policy_and_freshness(self):
        combine = ArucoRouteGate.combine
        retrieval = {'state': 'NAVIGATING_TO_D1', 'nav_leg': 'd1', 'run_id': 'r'}
        self.assertTrue(combine('GOTO_D', True, True, None, False, False))
        self.assertFalse(combine('GOTO_D', True, False, None, False, False))
        self.assertFalse(combine('GOTO_D', True, True, retrieval, True, True))
        self.assertFalse(combine('IDLE', False, True, retrieval, True, False))
        self.assertTrue(combine('IDLE', False, False, retrieval, True, True))
        for state in ('RECOVERY', 'STOPPING'):
            self.assertFalse(combine(state, True, True, retrieval, True, True))
        for state in ('DONE', 'FAILED', 'STOPPED'):
            self.assertTrue(combine(state, False, False, retrieval, True, True))
        self.assertFalse(combine('IDLE', False, False,
                                 {**retrieval, 'state': 'NAVIGATING_TO_D1', 'discarded': True},
                                 False, True))
        self.assertFalse(ArucoRouteGate().enabled)  # restart default


class RosWiringTest(unittest.TestCase):
    def test_single_publisher_and_required_subscription(self):
        root = Path(__file__).resolve().parents[2]
        supervisor = root / 'amr_mission_manager/amr_mission_manager/mission_supervisor_node.py'
        retrieval = root / 'amr_mission_manager/amr_mission_manager/pallet_retrieval_node.py'
        localizer = root / 'aruco_b1_package/aruco_b1_package/aruco_b1_localizer_node.py'
        sources = [path.read_text() for path in (supervisor, retrieval, localizer)]
        self.assertEqual(sum(text.count("create_publisher(Bool, '/aruco_b1/enable'")
                             for text in sources), 1)
        self.assertIn("'/pallet_retrieval/status'", sources[0])
        self.assertIn("payload['nav_goal_active']", sources[1])
        tree = ast.parse(sources[0])
        self.assertTrue(any(isinstance(node, ast.ImportFrom) and node.module == 'std_msgs.msg'
                            and any(alias.name == 'Bool' for alias in node.names)
                            for node in ast.walk(tree)))
        self.assertIn("p('mission_enable_timeout_s', 3.0)", sources[2])
        self.assertIn("p('auto_update_amcl', False)", sources[2])
        self.assertIn('self.destroy_subscription(self.image_sub)', sources[2])


if __name__ == '__main__':
    unittest.main()
