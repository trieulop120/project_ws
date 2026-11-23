"""Regression for the stationary retrieval TF consistency hold."""
import math
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_mission_manager.pallet_mission_node import PalletMission, SETTLING


class RetrievalTfHoldTest(unittest.TestCase):
    def test_large_shift_holds_without_failing_then_continues(self):
        events = []
        context = {'started': 95.0, 'terminal': True, 'stopped_since': 98.0,
                   'cancel_at': None, 'handoff': False}
        node = SimpleNamespace(
            _retrieval_pick_nav=context, _retrieval_pick_nav_elapsed=0.0,
            _settle_t0=98.0, _drop_odom_t=100.0, _drop_odom_speed=(0.0, 0.0),
            _nav_pending=None, _state=SETTLING, _msg='',
            _result={'x': 0.0, 'y': 0.0, 'yaw': 0.0}, _pallet_ins=(0.0, 0.0, 0.0),
            p={'retrieval_nav_timeout_s': 180.0, 'lift_home_tol_cm': 1.0,
               'retrieval_stop_timeout_s': 20.0, 'drop_odom_stale_s': 0.5,
               'drop_stop_linear_mps': 0.02, 'drop_stop_angular_rps': 0.04,
               'settle_s': 1.0, 'retrieval_tf_jump_m': 0.10,
               'retrieval_tf_jump_deg': 10.0},
            _lift_height_now=lambda: 0.0,
            _retrieval_navigation_idle=lambda: True,
            _retrieval_staging_reached=lambda: True,
            _halt=lambda: events.append('halt'),
            _fail_retrieval_pick_navigation=lambda *_: events.append('fail'),
            _begin_alignment=lambda: events.append('align'))
        node._map_to_ins = lambda *_: (0.11, 0.0, 0.0)
        node._set_state = lambda state, message: (setattr(node, '_state', state), setattr(node, '_msg', message))
        PalletMission._retrieval_pick_staging_tick(node, 100.0)
        self.assertEqual(events, ['halt', 'halt'])
        self.assertEqual(node._state, SETTLING)
        self.assertIs(node._retrieval_pick_nav, context)
        self.assertEqual(context['tf_hold_since'], 100.0)
        node._drop_odom_t = 101.0
        node._map_to_ins = lambda *_: (0.09, 0.0, math.radians(11.0))
        PalletMission._retrieval_pick_staging_tick(node, 101.0)
        self.assertNotIn('fail', events)
        self.assertNotIn('align', events)
        node._drop_odom_t = 102.0
        node._map_to_ins = lambda *_: (0.09, 0.0, math.radians(9.0))
        PalletMission._retrieval_pick_staging_tick(node, 102.0)
        self.assertEqual(events[-1], 'align')
        self.assertNotIn('fail', events)


if __name__ == '__main__':
    unittest.main()
