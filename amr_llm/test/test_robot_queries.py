"""Telemetry answers and command routing without ROS, network or motion."""
import ast
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS
import unittest

BASE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(BASE / 'amr_llm'))
from robot_queries import answer_query, build_context, classify_query
from llm_logic import is_stop, validate_plan
from manual_recovery import ManualRecovery, resume_requested
from types import MethodType


def method(file, name, namespace):
    path = BASE / 'amr_llm' / file
    tree = ast.parse(path.read_text())
    node = next(m for c in tree.body if isinstance(c, ast.ClassDef)
                for m in c.body if isinstance(m, ast.FunctionDef) and m.name == name)
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), namespace)
    if name == '_run_plan':
        fn = namespace[name]
        def run(node, *args, **kwargs):
            node._begin_manual_checkpoint = MethodType(ManualRecovery._begin_manual_checkpoint, node)
            node._freeze_manual_checkpoint = MethodType(ManualRecovery._freeze_manual_checkpoint, node)
            return fn(node, *args, **kwargs)
        return run
    return namespace[name]


def context(**extra):
    values = dict(supervisor={'state': 'PICKING', 'shelf_ok': True,
                             'slots': {'A1': 'full', 'A2': 'empty', 'B1': 'full', 'B2': 'full'},
                             'cycle': {'placed': 2, 'target_quantity': 3},
                             'progress_percent': 40, 'pm_status_fresh': True, 'cargo': False},
                  supervisor_age=.2)
    values.update(extra)
    return build_context(**values)


class QueryTests(unittest.TestCase):
    def test_single_cycle_commands_validate_but_mixed_cycles_do_not(self):
        for action, args in (
                ('start_auto_cycle', {}), ('start_single_cycle', {}),
                ('retrieve_pallet', {'slot': 'A1'}), ('stop_cycle', {}),
                ('resume_cycle', {})):
            with self.subTest(action=action):
                clean = validate_plan({'steps': [{'action': action, 'args': args}]}, 'home')
                self.assertEqual(clean[0]['action'], action)
        with self.assertRaisesRegex(ValueError, 'Cycle chỉ được ghép'):
            validate_plan({'steps': [{'action': 'start_auto_cycle', 'args': {}},
                                     {'action': 'start_single_cycle', 'args': {}}]}, 'home')

    def test_ordered_multiple_single_cycle_plans_validate(self):
        plans = [
            [{'action': 'start_single_cycle', 'args': {'cargo_class': 'medium', 'quantity': 2}}],
            [{'action': 'start_single_cycle', 'args': {'cargo_class': 'any', 'quantity': 2}}],
            [{'action': 'start_single_cycle', 'args': {'cargo_class': 'medium', 'quantity': 1, 'drop_slot': 'A2'}},
             {'action': 'start_single_cycle', 'args': {'cargo_class': 'tall', 'quantity': 1, 'drop_slot': 'B2'}}],
        ]
        for steps in plans:
            with self.subTest(steps=steps):
                clean = validate_plan({'steps': steps}, 'home')
                self.assertEqual(len(clean), len(steps))
                self.assertTrue(all(step['action'] == 'start_single_cycle' for step in clean))
        self.assertEqual(validate_plan({'steps': plans[2]}, 'home')[1]['args']['drop_slot'], 'B2')

    def test_multiple_cycle_steps_start_one_supervisor_queue(self):
        run_plan = method('llm_command_node.py', '_run_plan', {'Aborted': RuntimeError})
        calls = []
        node = NS(_is_en=lambda _language: False,
                  get_parameter=lambda _name: NS(value=False),
                  _begin_manual_checkpoint=lambda *_args: None,
                  _freeze_manual_checkpoint=lambda _cp: None,
                  _check_abort=lambda: None,
                  _status=lambda *_args, **_kwargs: None,
                  _start_cycle=lambda mode, **kwargs: calls.append((mode, kwargs)) or True,
                  _wait_cycle_complete=lambda *args, **kwargs: calls.append(('wait', kwargs)) or True,
                  _success_message='', _failure_message='', _cycle_run_id='run-42',
                  _cycle_operation_id=3)
        for action in ('set_mode', 'quick_goal', 'start_auto_cycle', 'start_single_cycle',
                       'retrieve_pallet', 'stop_cycle', 'cancel_navigation', 'resume_cycle',
                       'go_mission', 'pick_pallet', 'detect_pallet', 'lock_detection',
                       'drop_pallet', 'lift_to', 'lift_home', 'relative_move'):
            setattr(node, '_h_' + action, lambda _args: True)
        run_plan(node, [
            {'action': 'start_single_cycle', 'args': {'cargo_class': 'medium', 'quantity': 1, 'drop_slot': 'A2'}},
            {'action': 'start_single_cycle', 'args': {'cargo_class': 'tall', 'quantity': 1, 'drop_slot': 'B2'}},
        ], 'vi')
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[0][0], 'single')
        self.assertEqual(calls[0][1]['queue'][0]['drop_slot'], 'A2')
        self.assertEqual(calls[0][1]['queue'][1]['drop_slot'], 'B2')
        self.assertEqual(calls[1], ('wait', {'expected_run_id': 'run-42'}))

    def test_project_questions_without_telemetry(self):
        examples = {
            'Bạn là ai?': 'identity', 'Bạn tên gì': 'identity',
            'Tên của bạn là gì?': 'identity',
            'Giới thiệu về bản thân': 'identity', 'Who are you?': 'identity',
            'What is your name?': 'identity',
            'Ai tạo ra bạn?': 'creators', 'Ai thiết kế xe robot này?': 'creators',
            'Who built this robot?': 'creators', 'Bạn được lập trình bởi ai?': 'creators',
            'Giảng viên hướng dẫn đồ án là ai?': 'advisor',
            'Who supervised the project?': 'advisor',
            'Giới thiệu về đồ án xe robot AMR': 'project',
            'AMR là gì?': 'project', 'What is this robot for?': 'project',
            'Đồ án sử dụng công nghệ gì?': 'technology',
            'Bạn là mô hình gì?': 'llm_model', 'Bạn dùng model nào?': 'llm_model',
            'What model are you using?': 'llm_model',
            'Bạn hoạt động như thế nào?': 'llm_workflow',
            'How do you work?': 'llm_workflow',
        }
        c = build_context()
        for text, topic in examples.items():
            with self.subTest(text=text):
                self.assertIn(topic, classify_query(text) or [])
                self.assertTrue(answer_query(classify_query(text), c, text))
                step = {'action': 'get_robot_status', 'args': {'topic': topic}}
                self.assertEqual(validate_plan({'steps': [step]}, 'unknown'), [step])
        creators = answer_query(['creators'], c)
        for name in ('Trương Ngọc Triều', 'Phạm Lê Thiên Vũ', 'TS. Dương Minh Thiện'):
            self.assertIn(name, creators)
        self.assertIn('AI assistant', answer_query(['identity'], c, language='en'))

    def test_profile_mixed_with_motion_requires_planning(self):
        for text in ('Bạn là ai rồi lùi 50 cm', 'Ai tạo ra bạn và hãy nâng càng 10 cm',
                     'Bạn dùng model gì rồi lùi 50cm', 'How do you work? Then move backward',
                     'Giới thiệu đồ án; đi tới HOME', 'Who built you? Then move backward 0.5m'):
            self.assertIsNone(classify_query(text), text)

    def test_model_answer_distinguishes_configuration_from_response(self):
        c = context()
        self.assertIn('chưa xác định được tên mô hình', answer_query(['llm_model'], c))
        c['llm'] = {'configured': [{'backend': 'groq', 'model': 'example-model'}],
                    'last_success': None}
        reply = answer_query(['llm_model'], c)
        self.assertIn('được cấu hình sử dụng mô hình example-model', reply)
        self.assertNotIn('JSON', reply)
        self.assertNotIn('Backend', reply)
        c['llm']['last_success'] = {'backend': 'other', 'model': 'fallback-model'}
        reply = answer_query(['llm_model'], c, language='en')
        self.assertIn('using fallback-model', reply)
        self.assertNotIn('example-model', reply)
        self.assertNotIn('llm_model', classify_query('Model YOLO nhận diện pallet là gì?'))

    def test_natural_vietnamese_and_english_questions(self):
        examples = {'kệ có mấy sản phẩm rồi': 'inventory', 'A1 có hàng không': 'inventory',
                    'How many products are on the shelf?': 'inventory',
                    'xe đang mang hàng gì': 'cargo', 'mission đang làm gì': 'mission',
                    'đang auto không': 'mode', 'resume được không': 'mission', 'đã đặt bao nhiêu hộp': 'mission',
                    'pin còn bao nhiêu': 'battery', 'robot có thể làm gì': 'capabilities',
                    'Tôi muốn biết tồn kho trên kệ': 'inventory', 'robot đang ở đâu': 'location'}
        for text, topic in examples.items():
            with self.subTest(text=text): self.assertIn(topic, classify_query(text) or [])

    def test_movement_or_mixed_requests_do_not_bypass_confirmation(self):
        for text in ('bật auto', 'start mission', 'lấy pallet A1', 'hạ lift rồi lùi 50cm về home',
                     'kệ có bao nhiêu hàng rồi lấy A1', 'bao nhiêu ô trống và nâng càng 10cm', 'kệ có bao nhiêu pallet và lùi 50cm',
                     'How many pallets on the shelf; retrieve A1', 'Tôi muốn lấy 2 pallet'):
            self.assertIsNone(classify_query(text), text)

    def test_multi_topic_questions_keep_profile_and_telemetry(self):
        cases = {
            'Bạn là ai và pin còn bao nhiêu?': ('identity', 'battery'),
            'Giới thiệu đồ án và tình trạng mission hiện tại?': ('project', 'mission'),
            'Who are you and is Nav2 active?': ('identity', 'navigation'),
            'Ai tạo ra bạn và càng đang cao bao nhiêu?': ('creators', 'lift'),
        }
        c = context(battery=72, battery_age=.1)
        for text, expected in cases.items():
            with self.subTest(text=text):
                topics = classify_query(text)
                self.assertTrue(set(expected).issubset(topics))
                answer = answer_query(topics, c, text)
                for topic in expected:
                    self.assertIn(answer_query([topic], c, text), answer)

    def test_modal_and_punctuated_motion_still_requires_planning(self):
        for text in ('Bạn có thể lấy pallet A1 không?', 'Bạn lấy pallet A1 được không?',
                     'Can you retrieve A1?',
                     'Bạn là ai? Đi tới HOME', 'Pin còn bao nhiêu và bạn hãy lùi 50cm',
                     'What is the status? Could you move backward?',
                     'Nó còn trống không rồi lấy A1', 'Who are you and please go HOME'):
            self.assertIsNone(classify_query(text), text)

    def test_unknown_and_followup_questions_request_information_only(self):
        for text in ('Nó còn trống không?', 'Is it empty?', 'Bạn thấy hộp màu gì?',
                     'What color is the box?', 'Thời tiết hôm nay thế nào?',
                     'Tell me a joke', 'Kể chuyện vui đi', 'Explain quantum physics'):
            with self.subTest(text=text):
                topics = classify_query(text)
                self.assertEqual(topics, ['clarification'])
                self.assertTrue(answer_query(topics, context(), text))
        self.assertIn('ô hoặc vật nào', answer_query(['clarification'], context(), 'Nó còn trống không?'))
        self.assertIn('inventory', classify_query('Còn A2'))
        self.assertIn('A2: trống', answer_query(classify_query('Còn A2'), context(), 'Còn A2'))

    def test_navigation_reports_goal_and_limits_not_lifecycle_readiness(self):
        for active in (True, False, None, 'false'):
            c = context(supervisor={'nav_status_fresh': True, 'nav_goal_active': active})
            answer = answer_query(classify_query('Nav2 đang hoạt động không?'), c)
            self.assertIn('node lifecycle Nav2', answer)
            if active is True:
                self.assertIn('đang có goal', answer)
            elif active is False:
                self.assertIn('không có goal', answer)
            else:
                self.assertIn('chưa xác định', answer)
        for options in ({'supervisor_age': 10}, {'supervisor': {'nav_status_fresh': False, 'nav_goal_active': True}}):
            self.assertIn('thiếu/cũ', answer_query(['navigation'], context(**options)))

    def test_motion_uses_fresh_odometry_not_mission_or_goal(self):
        for stopped in (True, False, None, 'false'):
            c = context(supervisor={'state': 'GOTO_PICK', 'nav_status_fresh': True,
                                   'nav_goal_active': True, 'odom_fresh': True,
                                   'robot_stopped': stopped})
            answer = answer_query(classify_query('Robot đang đứng yên hay di chuyển?'), c)
            if stopped is True:
                self.assertIn('Xe đang đứng yên', answer)
            elif stopped is False:
                self.assertIn('Xe đang di chuyển', answer)
            else:
                self.assertIn('Chưa xác định', answer)
        for options in ({'supervisor_age': 10}, {'supervisor': {'odom_fresh': False, 'robot_stopped': False}}):
            self.assertIn('Chưa xác định', answer_query(['motion'], context(**options)))
        self.assertIn('motion', classify_query('Is the robot moving?'))
        answer = answer_query(['overview', 'identity'], context())
        self.assertIn('Tôi là trợ lý AI', answer)
        self.assertIn('odometry', answer)

    def test_time_questions_explain_missing_estimates(self):
        c = context(battery=72, battery_age=.1)
        for text in ('Còn bao lâu xong mission?', 'Khi nào mission xong?',
                     'How long until the mission is complete?'):
            with self.subTest(text=text):
                topics = classify_query(text)
                self.assertIn('eta', topics)
                answer = answer_query(topics, c, text)
                self.assertIn('Mission: PICKING', answer)
                self.assertIn('Chưa thể ước lượng', answer)
        for text in ('Có thể chạy thêm bao lâu với lượng pin hiện tại?',
                     'How long will the battery last?'):
            topics = classify_query(text)
            self.assertIn('battery_runtime', topics)
            self.assertNotIn('eta', topics)
            answer = answer_query(topics, c, text)
            self.assertIn('Pin còn 72%.', answer)
            self.assertIn('lịch sử xả pin', answer)
        self.assertIn('cannot estimate', answer_query(['eta'], c, language='en'))
        self.assertIn('odometry', answer_query(['motion'], c, language='en'))

    def test_health_why_does_not_invent_a_root_cause(self):
        text = 'Tại sao xe chưa sẵn sàng?'
        answer = answer_query(classify_query(text), context(), text)
        self.assertIn('chưa đủ xác định nguyên nhân', answer)

    def test_inventory_counts_and_slot_filter(self):
        c = context()
        self.assertEqual(c['shelf']['full'], 3)
        self.assertEqual(c['shelf']['empty'], 1)
        self.assertTrue(c['shelf']['exact'])
        answer = answer_query(['inventory'], c, 'kệ có mấy sản phẩm')
        self.assertIn('3/4', answer)
        self.assertIn('không phải số sản phẩm bên trong hộp', answer)
        self.assertIn('A2: trống', answer)
        answer = answer_query(['inventory'], c, 'A1 có hàng không')
        self.assertIn('A1: có hàng', answer)
        self.assertNotIn('B1:', answer)
        self.assertIn('1/2', answer_query(['inventory'], c, 'bao nhiêu ô medium'))

    def test_stale_or_invalid_shelf_is_unknown_not_empty(self):
        for options in ({'supervisor_age': 10}, {'supervisor': {'shelf_ok': False}},
                        {'supervisor': {'shelf_ok': True, 'slots': {'A1': 'full', 'B1': 1}}}):
            c = context(**options)
            self.assertFalse(c['shelf']['exact'])
            self.assertIn('Chưa thể chốt tổng số', answer_query(['inventory'], c))
        self.assertEqual(context(supervisor_age=10)['shelf']['empty'], 0)

    def test_cargo_and_lift_do_not_default_to_empty(self):
        self.assertIsNone(context(supervisor={'cargo': False, 'pm_status_fresh': False})['cargo'])
        c = context(pallet={'cargo': True, 'cargo_type': 'B', 'lift_cm': 25}, pallet_age=.1)
        self.assertTrue(c['cargo'])
        self.assertEqual(c['lift_cm'], 25)
        self.assertIn('tall', answer_query(['cargo'], c))
        c = context(supervisor_age=10, pallet={'cargo': True}, pallet_age=10)
        self.assertIsNone(c['cargo'])
        self.assertIn('thiếu/cũ', answer_query(['cargo'], c))

    def test_active_retrieval_beats_idle_supervisor(self):
        c = context(retrieval={'state': 'HOMING_LIFT', 'slot': 'B1'}, retrieval_age=.2)
        self.assertEqual(c['active_owner'], 'retrieval')
        self.assertIn('HOMING_LIFT', answer_query(['mission'], c))
        self.assertIn('checkpoint', answer_query(['mission'], c))
        c = context(retrieval={'state': 'DONE'}, retrieval_age=.2)
        self.assertEqual(c['active_owner'], 'supervisor')
        self.assertIn('PICKING', answer_query(['mission'], c))

    def test_recovery_is_advisory_not_an_automatic_resume(self):
        c = context(supervisor={'state': 'RECOVERY', 'resume_available': False, 'msg': 'gate failed'})
        self.assertIn('Không có checkpoint', answer_query(['mission'], c))
        self.assertIn('gate failed', answer_query(['mission'], c))

    def test_battery_staleness_and_finite_numbers(self):
        self.assertEqual('Pin còn 70%.', answer_query(['battery'], context(battery=70, battery_age=.2)))
        self.assertIsNone(context(battery=70, battery_age=10)['battery_percent'])
        self.assertIsNone(context(lift=float('nan'), lift_age=.1)['lift_cm'])

    def test_battery_question_answers_only_the_requested_information(self):
        c = context(battery=100, battery_age=.1)
        for text, language, expected in (
                ('Còn bao nhiêu pin', 'vi', 'Pin còn 100%.'),
                ('Pin còn bao nhiêu phần trăm?', 'vi', 'Pin còn 100%.'),
                ('What is the battery level?', 'en', 'Battery: 100%.')):
            self.assertEqual(classify_query(text), ['battery'])
            self.assertEqual(answer_query(classify_query(text), c, text, language), expected)
        stale = context(battery=100, battery_age=10)
        self.assertEqual(answer_query(['battery'], stale), 'Chưa có dữ liệu pin còn mới.')
        mixed = 'Pin và thiết bị sẵn sàng chưa?'
        topics = classify_query(mixed)
        self.assertIn('battery', topics)
        self.assertIn('health', topics)
        answer = answer_query(topics, c, mixed)
        self.assertIn('Pin còn 100%.', answer)
        self.assertIn('Thiết bị:', answer)
        self.assertNotIn('Pin', answer_query(['health'], c))
        self.assertIn('Pin còn 100%.', answer_query(['overview'], c))

    def test_status_tool_is_isolated_from_motion(self):
        query = {'action': 'get_robot_status', 'args': {'topic': 'inventory'}}
        self.assertEqual(validate_plan({'steps': [query]}, 'home'), [query])
        for topic in ('battery', 'navigation', 'motion', 'eta', 'battery_runtime', 'clarification'):
            step = {'action': 'get_robot_status', 'args': {'topic': topic}}
            self.assertEqual(validate_plan({'steps': [step]}, 'unknown'), [step])
        with self.assertRaises(ValueError):
            validate_plan({'steps': [query, {'action': 'lift_home', 'args': {}}]}, 'home')

    def test_manual_sequence_and_location_after_relative_move(self):
        steps = [{'action': 'lift_home', 'args': {}},
                 {'action': 'relative_move', 'args': {'direction': 'backward', 'distance_m': .5}},
                 {'action': 'quick_goal', 'args': {'target': 'HOME'}}]
        clean = validate_plan({'steps': steps}, 'd1')
        self.assertEqual([s['action'] for s in clean], ['lift_home', 'relative_move', 'quick_goal'])
        self.assertEqual(clean[1]['args']['speed_mps'], .1)
        with self.assertRaises(ValueError):
            validate_plan({'steps': steps[:2] + [{'action': 'go_mission', 'args': {'name': 'd1_to_p1'}}]}, 'd1')
        with self.assertRaises(ValueError):
            validate_plan({'steps': [{'action': 'relative_move', 'args': {'direction': 'backward'}}]}, 'd1')


class IntegrationTests(unittest.TestCase):
    def test_model_reporting_tracks_backend_fallback_without_network(self):
        backends = {'first': {'kind': 'openai', 'model': 'model-one'},
                    'second': {'kind': 'openai', 'model': 'model-two'}}
        fn = method('llm_command_node.py', '_plan', dict(time=time, BACKENDS=backends,
                    build_system_prompt=lambda *_: 'prompt', extract_json=json.loads))
        def call(name, *_):
            if name == 'first':
                raise RuntimeError('backend unavailable')
            return '{"steps": []}'
        node = NS(_mission_context=context, _location='unknown',
                  get_parameter=lambda _: NS(value=['first', 'second']),
                  _call_openai=call, _backend_failure=lambda _: ('error', 'unavailable'),
                  get_logger=lambda: NS(warning=lambda _: None))
        self.assertEqual(fn(node, 'plan'), {'steps': []})
        self.assertEqual(node._last_planner_model, {'backend': 'second', 'model': 'model-two'})

    def test_query_while_busy_preserves_action_and_confirmation(self):
        fn = method('llm_command_node.py', '_on_command',
                    dict(json=json, threading=threading, time=time, is_stop=is_stop,
                         classify_query=classify_query, answer_query=answer_query, resume_requested=resume_requested))
        events = []
        busy = threading.Lock(); busy.acquire()
        confirm = threading.Event()
        abort = threading.Event(); abort.set()
        pending = {'text': 'move'}
        node = NS(_busy=busy, _confirm_evt=confirm, _abort=abort, _current_lang='vi',
                  _pending_interrupt=pending, _current_action='drop_pallet',
                  _is_en=lambda lang=None: lang == 'en', _mission_context=context,
                  _status=lambda state, **kw: events.append((state, kw)))
        fn(node, NS(data=json.dumps({'text': 'How many products are on the shelf?', 'language': 'en'})))
        fn(node, NS(data=json.dumps({'text': 'Bạn là ai?', 'language': 'vi'})))
        fn(node, NS(data=json.dumps({'text': 'Ai tạo ra bạn?', 'language': 'vi'})))
        fn(node, NS(data=json.dumps({'text': 'Bạn hoạt động như thế nào?', 'language': 'vi'})))
        fn(node, NS(data=json.dumps({'text': 'Bạn dùng model gì?', 'language': 'vi'})))
        for text in ('Nó còn trống không?', 'Is it empty?', 'Bạn thấy hộp màu gì?',
                     'Bạn là ai và pin còn bao nhiêu?', 'Nav2 có hoạt động không?',
                     'Xe có đang di chuyển không?', 'Còn bao lâu xong mission?'):
            fn(node, NS(data=json.dumps({'text': text, 'language': 'vi'})))
        self.assertTrue(all(state == 'answer' for state, _ in events))
        self.assertIn('Trương Ngọc Triều', events[2][1]['msg'])
        self.assertEqual(events[0][0], 'answer')
        self.assertTrue(events[0][1]['observational'])
        self.assertEqual(node._current_lang, 'vi')
        self.assertIs(node._pending_interrupt, pending)
        self.assertFalse(confirm.is_set())
        self.assertTrue(abort.is_set())
        self.assertTrue(busy.locked())
        self.assertEqual(node._current_action, 'drop_pallet')

    def test_bridge_answers_without_nav_and_preserves_last_execution(self):
        publish = []
        handle = method('llm_bridge.py', 'handle', dict(json=json, math=__import__('math'),
                        classify_query=classify_query, MAX_TEXT_LEN=300,
                        String=lambda **kw: NS(**kw)))
        node = NS(_precheck=lambda: 'Nav2 stopped', _session_reset=False,
                  _pub_cmd=NS(publish=lambda msg: publish.append(msg)),
                  clear_status=lambda *_args: None,
                  _resp=lambda name, status='ok', message='': {'status': status})
        reply = handle(node, {'command': 'llm_command', 'text': 'kệ có mấy sản phẩm'})
        self.assertEqual(reply['status'], 'ok')
        self.assertEqual(len(publish), 1)
        reply = handle(node, {'command': 'llm_command', 'text': 'Ai tạo ra bạn?'})
        self.assertEqual(reply['status'], 'ok')
        self.assertEqual(len(publish), 2)
        for text in ('Nó còn trống không?', 'Nav2 hoạt động không?', 'Is the robot moving?'):
            reply = handle(node, {'command': 'llm_command', 'text': text})
            self.assertEqual(reply['status'], 'ok')
        self.assertEqual(len(publish), 5)
        reply = handle(node, {'command': 'llm_command', 'text': 'start mission'})
        self.assertEqual(reply['status'], 'error')
        self.assertEqual(len(publish), 5)
        on_status = method('llm_bridge.py', '_on_status', dict(json=json))
        old = {'state': 'waiting_confirm'}
        node.last_status = old; node._broadcast = lambda _: None
        on_status(node, NS(data=json.dumps({'state': 'answer', 'observational': True})))
        self.assertIs(node.last_status, old)

    def test_lift_clock_and_project_battery_units(self):
        fn = method('llm_command_node.py', '_mission_context', dict(time=time, build_context=build_context,
                    BACKENDS={'groq': {'model': 'test-model', 'key_env': 'SECRET_NOT_INCLUDED'}}))
        node = NS(_sup_status=None, _sup_state_t=0, _pallet_status=None, _pallet_state_t=0,
                  _retrieval_status=None, _retrieval_state_t=0, _lift_cm=15,
                  _lift_t=time.monotonic(), _battery_percent=70, _battery_t=time.time(),
                  _op_mode='MANUAL', _location='unknown',
                  get_parameter=lambda name: NS(value=['groq'] if name == 'backends' else 5))
        result = fn(node)
        self.assertEqual(result['lift_cm'], 15)
        self.assertEqual(result['battery_percent'], 70)
        self.assertEqual(result['llm']['configured'], [{'backend': 'groq', 'model': 'test-model'}])
        battery = method('llm_command_node.py', '_on_battery', dict(time=time, math=__import__('math')))
        node.get_parameter=lambda _:NS(value=100)
        battery(node, NS(percentage=70))
        self.assertEqual(node._battery_percent, 70)
        node.get_parameter=lambda _:NS(value=1)
        battery(node, NS(percentage=.7))
        self.assertEqual(node._battery_percent, 70)

    def test_completed_manual_steps_run_home_last(self):
        fn = method('llm_command_node.py', '_run_plan', {'Aborted': RuntimeError})
        actions = []
        node = NS(_is_en=lambda _:False, get_parameter=lambda _:NS(value=False),
                  _check_abort=lambda:None, _status=lambda *_a,**_k:None)
        for name in ('set_mode','quick_goal','start_auto_cycle','start_single_cycle','retrieve_pallet',
                     'stop_cycle','cancel_navigation','resume_cycle','go_mission','pick_pallet',
                     'detect_pallet','lock_detection','drop_pallet','lift_to','lift_home','relative_move'):
            setattr(node, '_h_'+name, lambda args: True)
        node._h_lift_home=lambda _: actions.append('lower completed') or True
        node._h_relative_move=lambda _: actions.append('reverse completed') or True
        node._h_quick_goal=lambda _: actions.append('home') or True
        fn(node, [{'action':'lift_home','args':{}}, {'action':'relative_move','args':{}},
                  {'action':'quick_goal','args':{'target':'HOME'}}], 'vi')
        self.assertEqual(actions, ['lower completed', 'reverse completed', 'home'])

    def test_manual_plan_stops_after_failed_step(self):
        fn = method('llm_command_node.py', '_run_plan', {'Aborted': RuntimeError})
        actions = []
        node = NS(_is_en=lambda _:False, get_parameter=lambda _:NS(value=False),
                  _check_abort=lambda:None, _status=lambda *_a,**_k:None)
        for name in ('set_mode','quick_goal','start_auto_cycle','start_single_cycle','retrieve_pallet',
                     'stop_cycle','cancel_navigation','resume_cycle','go_mission','pick_pallet',
                     'detect_pallet','lock_detection','drop_pallet','lift_to','lift_home','relative_move'):
            setattr(node, '_h_'+name, lambda args: True)
        node._h_lift_home=lambda _: actions.append('lower') or True
        node._h_relative_move=lambda _: actions.append('reverse') or False
        node._h_quick_goal=lambda _: actions.append('home') or True
        fn(node, [{'action':'lift_home','args':{}}, {'action':'relative_move','args':{}},
                  {'action':'quick_goal','args':{'target':'HOME'}}], 'vi')
        self.assertEqual(actions, ['lower', 'reverse'])

if __name__ == '__main__':
    unittest.main()
