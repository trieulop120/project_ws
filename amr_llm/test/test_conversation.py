"""Offline web bridge -> command callback -> status broadcast. No ROS node is started."""
import json
import os
import sys
import threading
import time
import unittest
from collections import OrderedDict
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import requests
from std_msgs.msg import String

# Allow running against the installed package by setting PYTHONPATH instead.
if not os.environ.get('AMR_TEST_INSTALLED'):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from amr_llm.llm_bridge import LLMBridge
from amr_llm.llm_command_node import LLMCommandNode
from amr_llm.robot_queries import classify_query


class Conversation(unittest.TestCase):
    def setUp(self):
        n = self.node = object.__new__(LLMCommandNode)
        self.events = []
        self.params = dict(dry_run=False, supervisor_status_max_age_s=5, backends=['groq'],
                           max_steps=8, require_confirm=True, allow_unmeasured_lift=False,
                           pallet_state_key='state', battery_percentage_scale=100,
                           supervisor_busy_states=['GOTO_PICK', 'RETURN_HOME'])
        n.get_parameter = lambda name: SimpleNamespace(value=self.params[name])
        n.get_logger = Mock(return_value=Mock())
        n._lock = threading.Lock()
        n._subjects = OrderedDict()
        n._busy = threading.Lock()
        n._abort, n._confirm_evt = threading.Event(), threading.Event()
        n._confirm_val, n._pending_interrupt = False, None
        n._current_lang, n._op_mode, n._location = 'en', 'AUTO', 'home'
        n._sup_state_t = n._pallet_state_t = n._retrieval_state_t = n._lift_t = n._battery_t = 0
        n._sup_status_signature, n._sup_status_revision = None, 0
        n._sup_state = n._retrieval_state = n._pallet_state = n._current_action = None
        n._sup_status = n._pallet_status = n._retrieval_status = None
        n._lift_cm = n._battery_percent = n._manual_checkpoint = n._clarify_ctx = None
        n._cycle_run_id, n._cycle_operation_id = 'r1', 'op1'
        n._switching_command = False
        n.pub_plan, n.pub_cmd, n._call, n._run_plan = Mock(), Mock(), Mock(), Mock()
        n._call_openai = Mock(side_effect=requests.ConnectionError('private host detail'))
        n._wait_confirm = Mock(return_value=False)
        b = self.bridge = object.__new__(LLMBridge)
        b._web_node = n
        b._precheck = Mock(return_value=None)
        b._session_reset = False
        b._broadcast = lambda payload: self.events.append(payload['data'])
        b.last_status = {'state': 'waiting_confirm', 'steps': [{'action': 'quick_goal'}]}
        b._pub_cmd = Mock()
        b._pub_cmd.publish.side_effect = n._on_command
        b._pub_confirm = Mock()
        n.pub_status = Mock()
        n.pub_status.publish.side_effect = b._on_status
        self.addCleanup(self.assert_no_motion)
        self.sup(state='IDLE')

    def assert_no_motion(self):
        self.node.pub_cmd.publish.assert_not_called()
        self.node._call.assert_not_called()
        self.node._run_plan.assert_not_called()
        self.bridge._pub_confirm.publish.assert_not_called()

    def sup(self, **status):
        self.node._on_supervisor_status(String(data=json.dumps(status)))

    def ask(self, text, session='tab-A', lang='vi', **extra):
        response = self.bridge.handle(dict(command='llm_command', text=text, language=lang,
                                           conversation_id=session, **extra))
        self.assertEqual(response['status'], 'ok')
        return self.events[-1]

    def test_busy_and_pending(self):
        n = self.node
        n._busy.acquire()
        n._pending_interrupt = {'text': 'original'}
        n._abort.set()
        n._confirm_evt.set()
        checkpoint = self.bridge.last_status
        self.bridge._precheck.return_value = 'Nav2 is offline'
        n._on_battery(SimpleNamespace(percentage=72))
        self.assertEqual(self.ask('pin sao rồi')['msg'], 'Còn 72%.')
        n._battery_t -= 6
        self.assertEqual(self.ask('pin sao rồi?')['msg'], 'Mình chưa có số đo pin mới.')
        self.assertEqual(self.ask('pin sao rồi?', speech_confidence=.2)['state'], 'answer')
        self.assertTrue(all(e['observational'] for e in self.events))
        self.assertIs(self.bridge.last_status, checkpoint)
        self.assertEqual(n._pending_interrupt, {'text': 'original'})
        self.assertTrue(n._abort.is_set() and n._confirm_evt.is_set() and n._busy.locked())
        self.assertFalse(n._confirm_val)
        self.assertEqual(n._current_lang, 'en')
        self.bridge._precheck.assert_not_called()
        n.pub_plan.publish.assert_not_called()
        n._call_openai.assert_not_called()

    def test_slots_refresh_ambiguity_and_session(self):
        self.sup(state='IDLE', shelf_ok=True, slots={'A1':'empty','A2':'full'})
        self.assertEqual(self.ask('A1 còn trống không?')['msg'], 'A1 đang trống.')
        self.assertEqual(self.ask('Còn A2?')['msg'], 'A2 có hàng.')
        self.assertEqual(self.ask('Ô đó để hàng gì?', 'tab-B')['msg'], 'Bạn muốn hỏi ô nào?')
        self.assertIn('mặt hàng cụ thể', self.ask('Ô đó để hàng gì?')['msg'])
        self.sup(state='IDLE', shelf_ok=True, slots={'A2':'empty'})
        self.assertIn('A2 đang trống', self.ask('Ô đó để hàng gì?')['msg'])
        self.node._sup_state_t -= 10
        self.assertIn('chưa có số đo mới cho ô A2', self.ask('Ô đó để hàng gì?')['msg'])
        self.ask('A1 và B2 còn trống không?')
        self.assertEqual(self.ask('Ô đó để hàng gì?')['msg'], 'Bạn muốn hỏi ô nào?')
        self.assertIn('loại cao', self.ask('B2')['msg'])
        self.node._on_command(String(data='{"internal_abort": true}'))
        self.assertEqual(self.ask('Ô đó để hàng gì?')['msg'], 'Bạn muốn hỏi ô nào?')

    def test_fault_followup_and_resume(self):
        self.sup(state='RETURN_HOME')
        self.assertEqual(self.ask('Xe sao rồi?')['msg'], 'Xe đang về HOME.')
        recovery = dict(state='RECOVERY', active_mission='return_home', resume_state='RETURN_HOME',
                        msg='Nav2 ABORTED for return_home (mode=FREE): Navigation result status=6')
        self.sup(**recovery, resume_available=False, resume_reason='Waiting for Nav2 idle status')
        for q in ['Sao xe dừng lúc về HOME?', 'Có phải do vật cản không?']:
            msg = self.ask(q)['msg']
            self.assertIn('Xe chưa về tới HOME', msg)
            self.assertIn('chưa có thông tin', msg)
            self.assertNotIn('vật cản', msg)
        self.assertIn('chưa có thêm thông tin', self.ask('Vì sao vậy?')['msg'])
        self.assertEqual(self.ask('Giờ chạy tiếp được chưa?')['msg'], 'Chưa. Nav2 chưa báo đã kết thúc điều hướng.')
        self.sup(**recovery, resume_available=True)
        self.assertIn('có thể chạy tiếp khi bạn xác nhận', self.ask('Giờ chạy tiếp được chưa?')['msg'])
        self.node._sup_state_t -= 6
        self.assertIn('chưa có dữ liệu mới', self.ask('Tiếp tục được chưa?')['msg'])
        self.node._call_openai.assert_not_called()
        self.node.pub_plan.publish.assert_not_called()
        self.node._sup_state_t = time.time()
        self.ask('Cho xe chạy tiếp đi')
        with self.node._busy:
            pass
        self.assertIn('waiting_confirm', [e['state'] for e in self.events])
        plan = json.loads(self.node.pub_plan.publish.call_args.args[0].data)
        self.assertEqual(plan['steps'], [{'action':'resume_cycle','args':{}}])
        self.node._call_openai.assert_not_called()

    def test_english_and_retrieval_owner(self):
        self.sup(state='RECOVERY', resume_available=True)
        self.node._on_retrieval_status(String(data=json.dumps(dict(
            state='RECOVERY', resume_available=False, resume_reason='Camera is not ready'))))
        self.assertIn('camera is not ready', self.ask('Can we resume now?', lang='en')['msg'])
        self.node._retrieval_state_t -= 5
        self.assertIn('no current', self.ask('Can we resume?', lang='en')['msg'])
        self.assertIn('no current', self.ask('What about A1?', lang='en')['msg'])
        for q in ['Can you retrieve A1?', 'Pin sao rồi, rồi cho xe chạy tiếp đi', 'Cho xe chạy tiếp đi']:
            self.assertIsNone(classify_query(q))

    def test_slow_planner_and_clarification_ownership(self):
        started, finish = threading.Event(), threading.Event()
        def slow_model(*args):
            started.set()
            if not finish.wait(3):
                raise AssertionError('The read-only path waited for the planner')
            return '{"clarify":"Which slot?","steps":[]}'
        n = self.node
        n._call_openai.side_effect = slow_model
        try:
            self.ask('retrieve pallet', session='planner', lang='en')
            self.assertTrue(started.wait(1))
            self.assertEqual(self.ask('Ô đó để hàng gì?')['msg'], 'Bạn muốn hỏi ô nào?')
            self.assertTrue(n._busy.locked())
        finally:
            finish.set()
        with n._busy:
            pass
        self.assertEqual(self.bridge.last_status['state'], 'clarify')
        self.assertIn('ô A2', self.ask('A2')['msg'])
        self.assertEqual(self.bridge.last_status['state'], 'clarify')
        n._call_openai.assert_called_once()
        self.ask('A1', session='planner', lang='en')
        with n._busy:
            pass
        self.assertIn('retrieve pallet', n._call_openai.call_args.args[-1])

    def test_reported_fault_and_stale_measurements(self):
        self.sup(state='RECOVERY', msg='Ve HOME qua timeout; da yeu cau huy goal Nav2')
        self.assertIn('thời gian cho phép', self.ask('Mission lỗi gì?')['msg'])
        self.sup(state='PICKING', handling_status='AUTO_RECOVERING',
                 pm_status_fresh=True, pm_err={'yaw_deg':4.6})
        self.assertIn('4,6°', self.ask('Mission lỗi gì?')['msg'])
        self.sup(state='PICKING', handling_status='AUTO_RECOVERING',
                 pm_status_fresh=False, pm_err={'yaw_deg':4.6})
        self.assertNotIn('4,6°', self.ask('Mission lỗi gì?')['msg'])
        self.assertNotIn('HOME', self.ask('Sao xe dừng lúc về HOME?')['msg'])

    def test_backend_and_messages(self):
        n = self.node
        # The model can select the status tool. It never supplies telemetry prose.
        n._on_battery(SimpleNamespace(percentage=72))
        def model_reply(*args):
            n._on_battery(SimpleNamespace(percentage=68))
            return json.dumps({'steps':[{'action':'get_robot_status','args':{'topic':'battery'}}],
                               'say':'✅ 99% and blocked by an obstacle'})
        n._call_openai.side_effect = model_reply
        self.ask('give me a readout', lang='en')  # unmatched wording uses planner
        with n._busy:
            pass
        n._call_openai.assert_called_once()
        self.assertEqual(self.events[-1]['msg'], 'Battery: 68%.')
        self.assertEqual(self.events[-1]['conversation_id'], 'tab-A')
        n.pub_plan.publish.assert_not_called()
        for error in (requests.Timeout(), requests.ConnectionError('private host')):
            n._call_openai.side_effect = error
            self.ask('lấy một pallet')
            with n._busy:
                pass
            self.assertEqual(self.events[-1]['error_kind'], 'planner_connection')
            self.assertNotIn('groq', self.events[-1]['msg'])
            self.assertEqual(self.ask('Pin sao rồi?')['msg'], 'Còn 68%.')
        n._call_openai.side_effect = lambda *args: '{"clarify":"❓ Ô A1 hay A2?","steps":[]}'
        self.ask('lấy pallet')
        with n._busy:
            pass
        self.assertEqual(self.events[-1]['msg'], 'Ô A1 hay A2?')
        n._status('answer', msg='⚠️ A1, 4,6°, 72% ✅', observational=True)
        self.assertEqual(self.events[-1]['msg'], 'A1, 4,6°, 72%')
        n._status('failed', msg='Nav2 ABORTED for return_home (mode=FREE): Navigation result status=6', language='vi')
        self.assertIn('Xe chưa về tới HOME', self.events[-1]['msg'])
        self.assertNotIn('status=6', self.events[-1]['msg'])
        n.get_logger().error.assert_called()

if __name__ == '__main__':
    unittest.main()
