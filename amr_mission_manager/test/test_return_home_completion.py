"""RETURN_HOME must wait for the owned Nav2 terminal result and measured stop."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest


SOURCE = Path(__file__).resolve().parents[1] / 'amr_mission_manager' / 'mission_supervisor_node.py'


def return_home_method():
    tree = ast.parse(SOURCE.read_text())
    method = next(n for cls in tree.body if isinstance(cls, ast.ClassDef)
                  and cls.name == 'MissionSupervisor' for n in cls.body
                  if isinstance(n, ast.FunctionDef) and n.name == '_do_return_home')
    scope = {'math': __import__('math'), 'FAILED': 'FAILED', 'DONE': 'DONE'}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(SOURCE), 'exec'), scope)
    return scope['_do_return_home']


class ReturnHomeTests(unittest.TestCase):
    def test_near_home_while_nav2_executes_keeps_cancel_until_terminal_idle(self):
        states = []
        n = NS(_triggered=True, _t_state=0., _near_t0=None,
               p={'goto_timeout_s': 180., 'arrive_tol_m': .15, 'arrive_settle_s': 1.,
                  'target_quantity': 1},
               _dest_pose=lambda: (0., 0., 0.), _base_pose=lambda: (0., 0., 0.),
               _mission_nav_status={'run_id': 'nav-1', 'mission': 'return_home',
                                    'state': 'EXECUTING', 'busy': True},
               _mission_nav_run_id='nav-1', _manager_status_run_id='nav-1',
               _manager_busy=True, _manager_status_t=10., _nav_status_t=10.,
               _nav_goal_active=True, _navigation_idle=lambda _: not n._nav_goal_active,
               _robot_stopped=lambda _: True, _pm_cargo=False,
               _terminal_after_home='DONE', _cycle=1,
               _set_state=lambda state, msg: states.append(state))
        run = return_home_method()
        run(n, 10.)
        run(n, 11.5)
        self.assertIsNone(n._near_t0)
        self.assertEqual(states, [])
        n._mission_nav_status.update(state='SUCCEEDED', busy=False)
        n._manager_busy = False
        n._nav_goal_active = False
        n._manager_status_t = n._nav_status_t = 12.
        run(n, 12.)
        self.assertEqual(states, [])
        run(n, 13.1)
        self.assertEqual(states, ['DONE'])


if __name__ == '__main__':
    unittest.main()
