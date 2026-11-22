"""Retrieval detector/recovery regressions without ROS or robot motion."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace as NS
import threading
import time
import unittest
from unittest.mock import Mock


SOURCE = Path(__file__).resolve().parents[1] / 'amr_mission_manager' / 'pallet_retrieval_node.py'
TREE = ast.parse(SOURCE.read_text())


def source_function(name, namespace):
    node = next((n for n in TREE.body if isinstance(n, ast.FunctionDef) and n.name == name), None)
    if node is None:
        node = next(n for c in TREE.body if isinstance(c, ast.ClassDef)
                    for n in c.body if isinstance(n, ast.FunctionDef) and n.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(SOURCE), 'exec'), namespace)
    return namespace[name]


class Clock:
    now = 1.0
    on_sleep = None

    @classmethod
    def monotonic(cls):
        return cls.now

    @classmethod
    def sleep(cls, dt):
        cls.now += dt
        if cls.on_sleep:
            cls.on_sleep()


class RetrievalDetectorTests(unittest.TestCase):
    def setUp(self):
        Clock.now, Clock.on_sleep = 1., None
        self.namespace = {'math': math, 'time': Clock,
                          'PoseStamped': lambda: NS(header=NS(), pose=NS(position=NS(), orientation=NS()))}
        self.namespace['detector_counts'] = source_function('detector_counts', self.namespace)
        self.namespace['yaw_from_quaternion'] = source_function('yaw_from_quaternion', self.namespace)

    def test_null_missing_and_wrong_type_counters_are_not_inference_frames(self):
        parse = self.namespace['detector_counts']
        for payload, field in (({'detected_pallets': None, 'localized_pallets': 0}, 'detected_pallets'),
                               ({'detected_pallets': 1, 'localized_pallets': None}, 'localized_pallets'),
                               ({'detected_pallets': 1}, 'localized_pallets'),
                               ({'detected_pallets': True, 'localized_pallets': 0}, 'detected_pallets'),
                               ({'detected_pallets': 1, 'localized_pallets': 2}, 'localized_pallets')):
            with self.subTest(payload=payload):
                self.assertEqual(parse(payload), (None, field))
        self.assertEqual(parse({'detected_pallets': 1, 'localized_pallets': 1}), ((1, 1), None))

    def fixture(self, payload):
        node = NS(_lock=threading.RLock(), _candidates=None, _candidate_t=0.,
                  _pm_status=None, _pm_status_t=0., _supervisor=None, _supervisor_t=0.,
                  _detection_status=payload, _detection_status_t=1.,
                  _raise_if_cancelled=lambda: None, get_logger=Mock(return_value=Mock()),
                  get_parameter=lambda name: NS(value={'candidate_timeout_s': .4,
                      'detector_startup_timeout_s': .3, 'stable_frames': 1}[name]),
                  _lookup_shelf_front=lambda: (0., 0., 0.),
                  _model=NS(retrieval_pose_match=lambda *_: (True, '')),
                  get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None)))
        return node

    def test_slow_detector_and_null_status_wait_then_continue_current_detect_step(self):
        stable = source_function('_stable_target', self.namespace)
        pose = NS(position=NS(x=1., y=0., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))
        for field in ('detected_pallets', 'localized_pallets'):
            with self.subTest(field=field):
                Clock.now = 1.
                node = self.fixture({'detected_pallets': 1, 'localized_pallets': 1, field: None})
                def publish():
                    if Clock.now >= 1.2:
                        node._detection_status = {'detected_pallets': 1, 'localized_pallets': 1}
                        node._detection_status_t = Clock.now
                        node._candidates = NS(poses=[pose])
                        node._candidate_t = Clock.now
                Clock.on_sleep = publish
                self.assertIsNotNone(stable(node, 'B2', 1.))
        Clock.on_sleep = None

    def test_invalid_detector_status_expires_startup_without_accepting_candidate(self):
        stable = source_function('_stable_target', self.namespace)
        node = self.fixture({'detected_pallets': None, 'localized_pallets': 1})
        node._candidates = NS(poses=[NS()])
        node._candidate_t = 1.
        with self.assertRaisesRegex(RuntimeError, 'no inference frames'):
            stable(node, 'B2', 1.)

    def test_resume_retains_run_slot_stage_and_target(self):
        starts = []
        namespace = {'threading': NS(Thread=lambda **kwargs: NS(start=lambda: starts.append(kwargs)))}
        resume = source_function('_srv_resume', namespace)
        target = object()
        node = NS(_resume_blocked=lambda: '', _run_lock=threading.Lock(),
                  _lock=threading.RLock(), _resume_revision=0, _resume_in_progress=False,
                  _cancel_event=threading.Event(), _resume_slot='B2', _resume_stage=4,
                  _run_id='run-7', _saved_target=target, _run=lambda *_: None,
                  _last_status={'state': 'RECOVERY'}, _pm_status={'operation_id': 12},
                  get_logger=Mock(return_value=Mock()))
        result = resume(node, None, NS())
        self.assertTrue(result.success)
        self.assertEqual(starts[0]['args'], ('B2', True))
        self.assertEqual((node._run_id, node._resume_stage, node._saved_target), ('run-7', 4, target))
        node._run_lock.release()

    def test_web_does_not_issue_unintended_retrieval_when_mission_is_busy(self):
        web_source = Path(__file__).resolve().parents[2] / 'amr_web' / 'amr_web' / 'bridge_node.py'
        tree = ast.parse(web_source.read_text())
        method = next(n for cls in tree.body if isinstance(cls, ast.ClassDef) and cls.name == 'WebBridge'
                      for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_pallet_retrieval_start')
        namespace = {}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(web_source), 'exec'), namespace)
        send = Mock()
        node = NS(_web_stop_generation=0,
                  get_nav_state=lambda: {'nav2_ready': True, 'process_owned': True},
                  _nav_bridge=NS(_op_mode='AUTO'), mission_cycle_conflict=lambda: True,
                  _call_service_wait=send)
        accepted, reason = namespace['_pallet_retrieval_start'](node, 'B2')
        self.assertFalse(accepted)
        self.assertEqual(reason, 'Mission busy')
        send.assert_not_called()

    def test_web_ignores_old_run_heartbeat_after_new_retrieval_start(self):
        web_source = Path(__file__).resolve().parents[2] / 'amr_web' / 'amr_web' / 'bridge_node.py'
        tree = ast.parse(web_source.read_text())
        method = next(n for cls in tree.body if isinstance(cls, ast.ClassDef) and cls.name == 'WebBridge'
                      for n in cls.body if isinstance(n, ast.FunctionDef)
                      and n.name == '_on_pallet_retrieval_status')
        namespace = {'json': json, 'String': object, 'time': time}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(web_source), 'exec'), namespace)
        current = {'run_id': 'old', 'state': 'DONE'}
        node = NS(_retrieval_start_pending=True, _retrieval_start_baseline_run_id='old',
                  _pallet_retrieval_status=current)
        namespace['_on_pallet_retrieval_status'](
            node, NS(data=json.dumps({'run_id': 'old', 'state': 'DONE'})))
        self.assertIs(node._pallet_retrieval_status, current)
        self.assertTrue(node._retrieval_start_pending)

    def test_web_accepts_new_llm_run_then_rejects_old_heartbeat(self):
        web_source = Path(__file__).resolve().parents[2] / 'amr_web' / 'amr_web' / 'bridge_node.py'
        tree = ast.parse(web_source.read_text())
        method = next(n for cls in tree.body if isinstance(cls, ast.ClassDef) and cls.name == 'WebBridge'
                      for n in cls.body if isinstance(n, ast.FunctionDef)
                      and n.name == '_on_pallet_retrieval_status')
        namespace = {'json': json, 'String': object, 'time': time}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(web_source), 'exec'), namespace)
        node = NS(_retrieval_start_pending=False, _retrieval_accepted_run_id='old',
                  _retrieval_retired_run_ids=set(),
                  _RETRIEVAL_ACTIVE={'HOMING_LIFT', 'NAVIGATING_TO_D1', 'ALIGNING_TO_SLOT'},
                  _pallet_retrieval_status={'run_id': 'old', 'state': 'DONE'},
                  get_mission_supervisor_status=lambda: {'stale': True},
                  _append_recent_mission=Mock())
        callback = namespace['_on_pallet_retrieval_status']
        callback(node, NS(data=json.dumps({'run_id': 'llm-new', 'state': 'NAVIGATING_TO_D1',
                                           'slot': 'B2'})))
        self.assertEqual(node._pallet_retrieval_status['run_id'], 'llm-new')
        self.assertEqual(node._retrieval_accepted_run_id, 'llm-new')
        callback(node, NS(data=json.dumps({'run_id': 'old', 'state': 'DONE', 'slot': 'B2'})))
        self.assertEqual(node._pallet_retrieval_status['run_id'], 'llm-new')


if __name__ == '__main__':
    unittest.main()
