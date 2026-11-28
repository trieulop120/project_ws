#!/usr/bin/env python3
"""
llm_command_node - ra lệnh cho AMR bằng tiếng Việt / tiếng Anh.

  /llm/command (String) -> LLM (Gemini/Groq/Ollama) -> kế hoạch JSON
      -> kiểm tra whitelist + vị trí -> /llm/plan -> (xác nhận /llm/confirm)
      -> gọi service/action sẵn có từng bước -> /llm/status
"""
from collections import OrderedDict
import hashlib
import json
import math
from copy import deepcopy
import os
import threading
import time

import requests
import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import BatteryState, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, SetBool, Trigger


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))

try:  # chay trong package (ros2 launch) - import tuong doi
    from .llm_logic import (
        Aborted,
        BACKENDS,
        GOAL_SUCCEEDED,
        MISSIONS,
        LIFT_LEVELS,
        LIFT_LIMIT_CM,
        TERMINAL,
        TOOLS,
        build_system_prompt,
        extract_json,
        is_stop,
        spoken_text,
        mission_from_to,
        validate_plan,
    )
except ImportError:  # chay theo tung file (python3 llm_command_node.py)
    try:
        from llm_logic import (
            Aborted,
            BACKENDS,
            GOAL_SUCCEEDED,
            MISSIONS,
            LIFT_LEVELS,
            LIFT_LIMIT_CM,
            TERMINAL,
            TOOLS,
            build_system_prompt,
            extract_json,
            is_stop,
            spoken_text,
            mission_from_to,
            validate_plan,
        )
    except ImportError:
        from amr_llm.llm_logic import (
            Aborted,
            BACKENDS,
            GOAL_SUCCEEDED,
            MISSIONS,
            LIFT_LEVELS,
            LIFT_LIMIT_CM,
            TERMINAL,
            TOOLS,
            build_system_prompt,
            extract_json,
            is_stop,
            spoken_text,
            mission_from_to,
            validate_plan,
        )


try:
    from .robot_queries import (answer_query, build_context, classify_query,
                                nav_failure, reported_reason, resolve_reference)
except ImportError:
    from amr_llm.robot_queries import (answer_query, build_context, classify_query,
                                nav_failure, reported_reason, resolve_reference)


try:
    from .manual_recovery import ManualRecovery, resume_requested, resume_scope
except ImportError:
    from amr_llm.manual_recovery import ManualRecovery, resume_requested, resume_scope


class PlannerUnavailable(RuntimeError):
    def __init__(self, detail, connection_failed):
        super().__init__(detail)
        self.connection_failed = connection_failed


class LLMCommandNode(ManualRecovery, Node):
    def __init__(self):
        super().__init__("llm_command_node")
        d = self.declare_parameter
        d("backends", ["gemini", "groq"])
        d("llm_timeout_s", 25.0)
        d("dry_run", False)
        d("require_confirm", True)
        d("confirm_timeout_s", 30.0)
        d("start_location", "home")
        d("max_steps", 8)
        d("mission_timeout_s", 180.0)
        d("cycle_completion_timeout_s", 1200.0)
        d("nav_start_timeout_s", 15.0)
        d("srv_detect", "pallet_mission/detect")
        d("srv_go_to_pallet", "pallet_mission/go_to_pallet")
        d("srv_pallet_stop", "pallet_mission/stop")
        d("srv_drop", "pallet_mission/drop")
        d("detect_first", True)
        d("detect_seconds", 4.0)
        d("detect_sample_timeout_s", 15.0)
        d("pallet_timeout_s", 120.0)
        d("pallet_status_topic", "/pallet_mission/status")
        d("pallet_state_key", "state")
        d("pallet_done", ["DONE", "CARRYING"])
        d("pallet_fail", ["FAILED", "STOPPED"])
        d("pallet_idle", ["IDLE"])
        d("set_after_pick_hold", True)
        d("allow_unmeasured_lift", False)
        d("supervisor_status_topic", "/mission_supervisor/status")
        d("supervisor_busy_states",
          ["GOTO_PICK", "PICKING", "GOTO_D", "DROPPING", "WAITING_SLOT",
           "RETURN_HOME", "RECOVERY"])
        d("supervisor_status_max_age_s", 5.0)
        d("lift_topic", "/esp32_cmd")
        d("lift_wait_s", 180.0)
        d("lift_stable_tol_cm", 0.3)
        d("lift_stable_s", 0.5)
        d("srv_retrieval_start", "pallet_retrieval/start")
        d("srv_retrieval_params", "pallet_retrieval/set_parameters")
        d("retrieval_status_topic", "/pallet_retrieval/status")
        d("retrieval_timeout_s", 600.0)
        d("odom_topic", "/odometry/filtered")
        d("odom_max_age_s", 0.5)
        d("relative_move_timeout_s", 90.0)
        d("lift_joint_name", "encoder_lift_joint")
        d("joint_states_topic", "/joint_states")
        d("battery_topic", "/battery_state")
        d("battery_percentage_scale", 100.0)  # ESP32 bridge currently publishes 0..100

        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = threading.Lock()
        self._abort = threading.Event()
        self._confirm_evt = threading.Event()
        self._confirm_val = False
        self._success_message = ""
        self._failure_message = ""
        self._current_lang = "vi"
        configured_location = str(self.get_parameter("start_location").value).lower()
        self._location = configured_location if configured_location in ("home", "p1", "d2", "d1") else "unknown"
        self._op_mode = "unknown"
        self._nav_list = []
        self._pallet_state = None
        self._pallet_state_t = 0.0
        self._pallet_status = None
        self._retrieval_state = None
        self._retrieval_state_t = 0.0
        self._retrieval_status = None
        self._lift_active = False
        self._motion_active = False
        self._manager_busy, self._manager_status_t = None, 0.0
        self._stop_requested_at = 0.0
        self._manual_checkpoint = None
        self._resume_step = False
        self._odom_still_since = None
        self._lift_still_since = None
        self._lift_still_anchor = None
        self._odom_pose = None
        self._odom_t = 0.0
        self._lift_cm = None
        self._lift_t = 0.0
        self._battery_percent = None
        self._battery_t = 0.0
        self._subjects = OrderedDict()
        self._clarify_ctx = None
        self._sup_state = None
        self._sup_state_t = 0.0
        self._sup_status = None
        self._sup_status_revision = 0
        self._sup_status_signature = None
        self._cycle_run_id = None
        self._cycle_operation_id = None
        self._current_action = None
        self._current_action_args = None
        self._current_command_text = ""
        self._pending_interrupt = None
        self._switching_command = False

        g = self._cb
        self.cli_mission = {
            m: self.create_client(Trigger, f"mission_manager/trigger_{m}_ack",
                                  callback_group=g) for m in MISSIONS}
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.cli_detect = self.create_client(SetBool, p("srv_detect"), callback_group=g)
        self.cli_go = self.create_client(Trigger, p("srv_go_to_pallet"), callback_group=g)
        self.cli_pstop = self.create_client(Trigger, p("srv_pallet_stop"), callback_group=g)
        self.cli_manager_stop = self.create_client(Trigger, "/mission_manager/stop", callback_group=g)
        self.cli_presume = self.create_client(Trigger, "pallet_mission/resume", callback_group=g)
        self.cli_drop = self.create_client(Trigger, p("srv_drop"), callback_group=g)
        self.cli_set_param = self.create_client(
            SetParameters, "pallet_mission/set_parameters", callback_group=g)
        self.cli_retrieval_start = self.create_client(
            Trigger, p("srv_retrieval_start"), callback_group=g)
        self.cli_retrieval_resume = self.create_client(Trigger, "pallet_retrieval/resume", callback_group=g)
        self.cli_retrieval_cancel = self.create_client(Trigger, "pallet_retrieval/cancel", callback_group=g)
        self.cli_retrieval_params = self.create_client(
            SetParameters, p("srv_retrieval_params"), callback_group=g)
        self.cli_cancel = self.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal", callback_group=g)
        self.cli_mstop = self.create_client(Trigger, "/mission/stop", callback_group=g)
        self.cli_cycle_start = self.create_client(Trigger, "/mission/start", callback_group=g)
        self.cli_cycle_params = self.create_client(
            SetParameters, "/mission_supervisor/set_parameters", callback_group=g)
        self.cli_set_mode = self.create_client(SetBool, "/llm/set_op_mode", callback_group=g)
        self.cli_quick_goal = {
            target: self.create_client(Trigger, f"/llm/quick_goal/{target.lower()}", callback_group=g)
            for target in ("HOME", "P_1", "D_2", "D_1")}
        self.cli_node_check = {
            target: self.create_client(Trigger, f"/llm/check_node/{target.lower()}", callback_group=g)
            for target in ("HOME", "P_1", "D_2", "D_1")}

        self.pub_plan = self.create_publisher(String, "/llm/plan", 10)
        self.pub_status = self.create_publisher(String, "/llm/status", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel_llm", 10)
        self.pub_lift = self.create_publisher(String, p("lift_topic"), 10)

        self.create_subscription(String, "/mission/status", self._on_manager_status, 10, callback_group=g)
        self.create_subscription(String, "/llm/command", self._on_command, 10, callback_group=g)
        self.create_subscription(Bool, "/llm/confirm", self._on_confirm, 10, callback_group=g)
        self.create_subscription(String, "/llm/set_location", self._on_set_loc, 10, callback_group=g)
        self.create_subscription(String, "/llm/op_mode", self._on_op_mode, 10, callback_group=g)
        self.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status",
                                 self._on_nav_status, 10, callback_group=g)
        self.create_subscription(Odometry, p("odom_topic"), self._on_odom, qos_profile_sensor_data,
                                 callback_group=g)
        self.create_subscription(JointState, p("joint_states_topic"), self._on_joint_states,
                                 qos_profile_sensor_data, callback_group=g)
        self.create_subscription(String, p("pallet_status_topic"),
                                 self._on_pallet_status, 10, callback_group=g)
        self.create_subscription(String, p("retrieval_status_topic"),
                                 self._on_retrieval_status, 10, callback_group=g)
        self.create_subscription(String, p("supervisor_status_topic"),
                                 self._on_supervisor_status, 10, callback_group=g)
        self.create_subscription(BatteryState, p("battery_topic"), self._on_battery,
                                 qos_profile_sensor_data, callback_group=g)
        self.create_service(Trigger, "llm/stop", self._srv_stop, callback_group=g)

        self._status("idle", msg="ready" if self._is_en() else "sẵn sàng")
        if p("dry_run"):
            self.get_logger().warning("*** DRY RUN ***")
        self.get_logger().info("llm_command_node sẵn sàng")

    def _is_en(self, language=None):
        return (language or self._current_lang) == "en"

    def _status(self, state, **kw):
        lang = kw.pop("language", self._current_lang)
        cp = getattr(self, '_manual_checkpoint', None)
        if cp and cp['index'] < len(cp['steps']) and not kw.get('observational'):
            kw['manual_recovery'] = {'step': cp['index'] + 1, 'total': len(cp['steps']),
                                     'action': cp['steps'][cp['index']]['action'],
                                     'paused': cp['paused_at'] is not None,
                                     'reason': self._manual_resume_blocked() if cp['paused_at'] is not None else ''}
        if self._cycle_run_id and not kw.get('observational'):
            kw.setdefault('run_id', self._cycle_run_id)
            kw.setdefault('operation_id', self._cycle_operation_id)
        if state == 'clarify' and not kw.get('low_confidence'):
            kw['clarify_until'] = time.time() + 90
        if state in ('failed', 'error'):
            detail = str(kw.get('msg') or '')
            self.get_logger().error(f'{state}: {detail}')
            kw['error_id'] = hashlib.sha256(detail.encode()).hexdigest()[:16]
            en = lang == 'en'
            kind = kw.setdefault('error_kind', 'mission')
            if kind == 'planner_connection':
                message = ('The planning service did not respond. No command was sent to the robot.' if en else
                           'Dịch vụ lập kế hoạch chưa phản hồi. Lệnh này chưa được gửi tới xe.')
            elif kind == 'planner':
                message = ('The assistant could not prepare a plan. No command was sent.' if en else
                           'Trợ lý chưa lấy được kế hoạch. Lệnh này chưa được gửi tới xe.')
            elif kind == 'plan_validation':
                message = (reported_reason(detail, en) or
                           ('I could not prepare this command for confirmation. Please check the request.' if en else
                            'Mình chưa lập được kế hoạch để xác nhận. Bạn kiểm tra lại yêu cầu nhé.'))
            else:
                message = nav_failure(detail, False, en) or reported_reason(detail, en)
                if not message:
                    step = kw.get('step')
                    message = ((f'Step {step} did not complete.' if en else f'Bước {step} chưa hoàn tất.') if step else
                               ('The mission did not report completion.' if en else 'Xe chưa báo nhiệm vụ hoàn tất.'))
            kw['msg'] = message
        for field in ('msg', 'say'):
            if field in kw:
                kw[field] = spoken_text(kw[field])
        self.pub_status.publish(String(data=json.dumps(
            {"state": state, "location": self._location, "op_mode": self._op_mode,
             "dry_run": bool(self.get_parameter("dry_run").value),
             "mission_state": self._sup_state or "unknown",
             "language": lang, **kw}, ensure_ascii=False)))

    def _check_abort(self):
        if self._abort.is_set():
            raise Aborted()

    def _sleep(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self._check_abort()
            time.sleep(0.1)

    def _call(self, client, req, timeout=5.0):
        self._check_abort()
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f'Service unavailable: {client.srv_name}')
        ev, box = threading.Event(), {'abandoned': False}
        stop_version = self._stop_requested_at
        def done(future):
            try:
                box['r'] = future.result()
            except Exception as exc:
                box['e'] = exc
            if box['abandoned'] or stop_version != self._stop_requested_at:
                self.get_logger().warn(
                    f'Late service response for {client.srv_name}; reconcile operation status before retry')
            ev.set()
        client.call_async(req).add_done_callback(done)
        if not ev.wait(timeout):
            box['abandoned'] = True
            raise TimeoutError(f'Service timed out: {client.srv_name}; result uncertain, do not replay')
        self._check_abort()
        if stop_version != self._stop_requested_at:
            raise Aborted()
        if 'e' in box:
            raise box['e']
        return box['r']

    def _on_confirm(self, msg):
        val = bool(msg.data)
        with self._lock:
            pending = self._pending_interrupt
            if pending is not None:
                self._pending_interrupt = None
        if pending is not None:
            lang = pending.get("language", self._current_lang)
            if val:
                block_reason = self._interrupt_block_reason(lang)
                if block_reason:
                    self._status("non_interruptible", msg=block_reason, language=lang)
                    return
                threading.Thread(
                    target=self._switch_to_new_command,
                    args=(pending["text"], lang, pending.get("conversation", "")),
                    daemon=True,
                ).start()
            else:
                en = self._is_en(lang)
                cont_msg = ("Continuing the current action." if en
                            else "Tiếp tục thực hiện hành động hiện tại.")
                resume_state = "running" if (self._busy.locked() or self._is_cycle_active()) else "idle"
                self._status(resume_state, msg=cont_msg, language=lang)
            return
        self._confirm_val = val
        self._confirm_evt.set()

    def _is_cycle_active(self):
        st, t = self._sup_state, self._sup_state_t
        if st is None or t <= 0.0:
            return False
        if time.time() - t > self.get_parameter("supervisor_status_max_age_s").value:
            return False
        busy = {s.upper() for s in self.get_parameter("supervisor_busy_states").value}
        return st in busy

    def _interrupt_block_reason(self, language=None):
        en = self._is_en(language)
        sup_cargo = self._fresh_supervisor_status().get("cargo") is True
        pal_cargo = self._fresh_pallet_status().get("cargo") is True
        pallet_fresh = self._pallet_state_t > 0.0 and (time.time() - self._pallet_state_t <= 3.0)
        pal_state = self._pallet_state if pallet_fresh else None
        sup_state = self._sup_state if self._is_cycle_active() else None
        act = self._current_action

        if sup_cargo or pal_cargo or pal_state == "CARRYING":
            return ("Cannot cancel mid-way because the robot is currently carrying a pallet. Continuing the current action."
                    if en else "Không thể hủy giữa chừng vì robot đang mang pallet. Hệ thống sẽ tiếp tục hoàn thành hành động hiện tại.")
        if self._lift_active or act in ("lift_to", "lift_home") or pal_state in ("LIFTING", "LOWERING"):
            return ("Cannot cancel mid-way while the fork lift is moving/lifting a pallet. Continuing the current action."
                    if en else "Không thể hủy giữa chừng khi càng nâng đang nâng/hạ pallet. Hệ thống sẽ tiếp tục hoàn thành hành động hiện tại.")
        if (act in ("pick_pallet", "drop_pallet")
                or sup_state in ("PICKING", "DROPPING", "GOTO_D")
                or pal_state in ("SETTLING", "ALIGNING", "INSERTING", "AUTO_RECOVERING",
                                 "WAITING_FOR_SENSOR", "BACKING", "HOMING")):
            return ("Cannot cancel mid-way while picking or placing a pallet on the rack. Continuing the current action."
                    if en else "Không thể hủy giữa chừng khi robot đang lấy hoặc đặt pallet lên kệ. Hệ thống sẽ tiếp tục hoàn thành hành động hiện tại.")
        if self._motion_active or act == "relative_move":
            return ("Cannot cancel mid-way during precision short-distance alignment. Continuing the current action."
                    if en else "Không thể hủy giữa chừng khi robot đang căn chỉnh khoảng cách ngắn. Hệ thống sẽ tiếp tục hoàn thành hành động hiện tại.")
        if sup_state == "RECOVERY":
            return ("Cannot cancel mid-way while the robot is executing error recovery. Continuing the current action."
                    if en else "Không thể hủy giữa chừng khi robot đang tự phục hồi lỗi. Hệ thống sẽ tiếp tục hoàn thành hành động hiện tại.")
        return None

    def _describe_active_task(self, language=None):
        en = self._is_en(language)
        handling = (self._sup_status or {}).get('handling_status')
        if handling == 'AUTO_RECOVERING':
            return ('automatically recovering pallet alignment' if en else
                    'đang tự phục hồi căn chỉnh pallet')
        if handling == 'WAITING_FOR_SENSOR':
            waiting = (self._sup_status or {}).get('pm_waiting_for') or 'sensor'
            return (f'robot stopped while waiting for fresh {waiting} data' if en else
                    f'robot đang dừng chờ dữ liệu {waiting} mới')
        if handling == 'NEED_OPERATOR':
            return ('pallet operation needs operator inspection and is not complete' if en else
                    'thao tác pallet cần người vận hành kiểm tra và chưa hoàn tất')
        if handling == 'FAILED':
            return ('pallet operation failed and was not completed' if en else
                    'thao tác pallet thất bại, chưa hoàn tất')
        act = self._current_action
        args = self._current_action_args or {}
        if act == "go_mission" and "name" in args:
            src, dst = mission_from_to(args["name"])
            loc = {"home": "HOME", "p1": "P_1", "d2": "D_2", "d1": "D_1"}
            route = f"{loc.get(src, 'current')} → {loc.get(dst, dst)}"
            return f"navigating route {route}" if en else f"di chuyển tuyến {route}"
        if act == "quick_goal" and "target" in args:
            return f"navigating to {args['target']}" if en else f"di chuyển tới {args['target']}"
        if act in ('lift_home', 'lift_to'):
            return 'adjusting the forks' if en else 'điều chỉnh độ cao càng'
        if act == 'relative_move':
            return 'carrying out a short move' if en else 'thực hiện chặng di chuyển ngắn'
        if act == "detect_pallet":
            return "detecting pallet" if en else "nhận diện pallet"
        if self._is_cycle_active():
            return (f"running pallet cycle ({self._sup_state})" if en
                    else f"chạy chu trình pallet ({self._sup_state})")
        if self._current_command_text:
            return (f"processing '{self._current_command_text}'" if en
                    else f"xử lý lệnh '{self._current_command_text}'")
        return "executing the previous command" if en else "thực hiện câu lệnh trước đó"

    def _switch_to_new_command(self, text, language, conversation=""):
        self._switching_command = True
        try:
            if self._busy.locked():
                self._abort.set()
                self._confirm_evt.set()
                nav_id, nav_state = self._nav_latest()
                if nav_id is not None and nav_state in (1, 2, 3) and self.cli_cancel.service_is_ready():
                    self.cli_cancel.call_async(CancelGoal.Request())
                    self.pub_cmd.publish(Twist())
            if self._is_cycle_active() and self.cli_mstop.service_is_ready():
                self.cli_mstop.call_async(Trigger.Request())
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline and self._is_cycle_active():
                    time.sleep(0.1)
            if not self._busy.acquire(timeout=5.0):
                en = self._is_en(language)
                self._status("busy", msg=("Could not stop previous command in time." if en
                                          else "Chưa dừng kịp lệnh trước đó."), language=language)
                return
        finally:
            self._switching_command = False

        self._abort.clear()
        self._confirm_evt.clear()
        self._confirm_val = False
        self._worker(text, language, conversation)

    def _on_set_loc(self, msg):
        candidate = msg.data.strip().lower()
        self._location = candidate if candidate in ("home", "p1", "d2", "d1") else "unknown"
        msg_txt = (f"location set = {self._location}" if self._is_en()
                   else f"đặt vị trí = {self._location}")
        self._status("idle", msg=msg_txt)

    def _on_op_mode(self, msg):
        mode = msg.data.strip().upper()
        if mode in ("AUTO", "MANUAL"):
            self._op_mode = mode

    def _on_nav_status(self, msg):
        with self._lock:
            self._nav_list = list(msg.status_list)

    def _on_manager_status(self, msg):
        try:
            data = json.loads(msg.data)
            self._manager_busy, self._manager_status_t = data.get('busy'), time.monotonic()
        except (ValueError, TypeError):
            pass

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        pose = (float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation))
        speed = (math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y), abs(msg.twist.twist.angular.z))
        now = time.monotonic()
        if not all(math.isfinite(v) for v in (*pose, *speed)):
            self._odom_pose = None
            self._odom_still_since = None
            return
        if now - self._odom_t > .5 or speed[0] > .02 or speed[1] > .04:
            self._odom_still_since = None
        elif self._odom_still_since is None:
            self._odom_still_since = now
        self._odom_pose, self._odom_t = pose, now

    def _on_joint_states(self, msg):
        try:
            i = msg.name.index(self.get_parameter("lift_joint_name").value)
            cm = float(msg.position[i]) * 100.0
            now = time.monotonic()
            if not math.isfinite(cm):
                self._lift_cm, self._lift_still_since = None, None
                return
            if (now - self._lift_t > 1. or self._lift_still_anchor is None
                    or abs(cm - self._lift_still_anchor) > float(self.get_parameter('lift_stable_tol_cm').value)):
                self._lift_still_since, self._lift_still_anchor = now, cm
            elif self._lift_still_since is None:
                self._lift_still_since = now
            self._lift_cm, self._lift_t = cm, now
        except (ValueError, IndexError, TypeError):
            pass

    def _on_pallet_status(self, msg):
        try:
            data = json.loads(msg.data)
            st = data.get(self.get_parameter("pallet_state_key").value)
            self._pallet_state = str(st).upper() if st is not None else None
            self._pallet_state_t = time.time()
            self._pallet_status = data
        except Exception:  # noqa: BLE001
            pass

    def _on_retrieval_status(self, msg):
        try:
            data = json.loads(msg.data)
            st = data.get("state")
            self._retrieval_state = str(st).upper() if st is not None else None
            self._retrieval_state_t = time.time()
            self._retrieval_status = data
        except Exception:  # noqa: BLE001
            pass

    def _on_supervisor_status(self, msg):
        try:
            data = json.loads(msg.data)
            st = data.get(self.get_parameter("pallet_state_key").value)
            cycle = data.get("cycle") if isinstance(data.get("cycle"), dict) else {}
            signature = (st, data.get("msg"), data.get("active_mission"),
                         data.get("handling_status"), data.get("pm_handling_status"),
                         data.get("pm_waiting_for"),
                         data.get("active_dest"), cycle.get("number"),
                         data.get("resume_state"), data.get("run_id"),
                         data.get("operation_id"), cycle.get("task_index"),
                         cycle.get("task_placed"))
            if signature != self._sup_status_signature:
                self._sup_status_revision += 1
                self._sup_status_signature = signature
            self._sup_status = data
            self._sup_state = str(st).upper() if st is not None else None
            self._sup_state_t = time.time()
        except Exception:  # noqa: BLE001
            pass

    def _nav_latest(self):
        with self._lock:
            if not self._nav_list:
                return None, None
            s = max(self._nav_list,
                    key=lambda x: (x.goal_info.stamp.sec, x.goal_info.stamp.nanosec))
            return tuple(int(b) for b in s.goal_info.goal_id.uuid), s.status

    def _supervisor_refusal(self):
        st, t = self._sup_state, self._sup_state_t
        if st is None or t <= 0.0:
            return None
        if time.time() - t > self.get_parameter("supervisor_status_max_age_s").value:
            return None
        busy = {s.upper() for s in self.get_parameter("supervisor_busy_states").value}
        if st in busy:
            return (f"Mission supervisor is busy ({st})." if self._is_en()
                    else f"Mission supervisor đang bận ({st}).")
        return None

    def _on_command(self, msg):
        language = "auto"
        conversation = ""
        speech_confidence = None
        raw = msg.data.strip()
        try:
            envelope = json.loads(raw)
            if isinstance(envelope, dict) and "text" in envelope:
                raw = str(envelope.get("text", ""))
                language = str(envelope.get("language", "auto")).lower()
                if isinstance(envelope.get("conversation_id"), str):
                    conversation = envelope["conversation_id"][:80]
                confidence = envelope.get("speech_confidence")
                if isinstance(confidence, (int, float)) and 0.0 <= float(confidence) <= 1.0:
                    speech_confidence = float(confidence)
            elif isinstance(envelope, dict) and envelope.get("internal_abort") is True:
                with self._lock:
                    self._subjects.clear()
                if self._busy.locked():
                    self._abort.set()
                    self._confirm_evt.set()
                    msg_txt = "LLM plan cancelled" if self._is_en() else "Kế hoạch LLM đã hủy"
                    self._status("cancelled", msg=msg_txt)
                return
        except (ValueError, TypeError):
            pass
        if language not in ("en", "vi"):
            language = "auto"
        en = self._is_en(language)
        text = raw.strip()
        if not text:
            return
        clarify = self._clarify_ctx
        plan_reply = clarify and clarify[3] == conversation and time.time() - clarify[2] < 90
        topics = classify_query(text, plan_reply=bool(plan_reply))
        if is_stop(text) and topics not in (['fault'], ['motion']):
            with self._lock:
                self._pending_interrupt = None
                self._subjects.pop(conversation, None)
            self._emergency_stop("stop command" if en else "lệnh dừng")
            return
        if speech_confidence is not None and speech_confidence < 0.55:
            prompt = ("I didn't catch that. Could you say it again?"
                      if en else "Mình nghe chưa rõ. Bạn nhắc lại được không?")
            self._status("answer" if topics else "clarify", msg=prompt, low_confidence=True,
                         observational=bool(topics), conversation_id=conversation,
                         language=language if language != "auto" else self._current_lang)
            return
        if topics:
            # Observations never acquire/release the action lock, clear abort,
            # consume a pending confirmation, or alter the running task language.
            reply_language = language if language in ("vi", "en") else self._current_lang
            self._answer(topics, text, reply_language, conversation)
            return
        with self._lock:
            self._subjects.pop(conversation, None)
        if language in ("vi", "en"):
            self._current_lang = language
        coordinator_resume = (resume_requested(text) and self._sup_state == "RECOVERY")
        if self._busy.locked() or (self._is_cycle_active() and not coordinator_resume):
            block_reason = self._interrupt_block_reason(self._current_lang)
            if block_reason:
                with self._lock:
                    self._pending_interrupt = None
                self._status("non_interruptible", msg=block_reason, language=self._current_lang)
                return
            with self._lock:
                self._pending_interrupt = {"text": text, "language": self._current_lang, "conversation": conversation}
            task_desc = self._describe_active_task(self._current_lang)
            prompt = (
                f"Robot is currently {task_desc}. Confirm cancelling the current action to run '{text}', or continue the current action?"
                if en else
                f"Robot đang {task_desc}. Bạn có xác nhận hủy hành động đang làm để chạy lệnh mới '{text}', hay tiếp tục hành động hiện tại?"
            )
            self._status("confirm_interrupt", msg=prompt, new_command=text,
                         confirm_required=True, language=self._current_lang)
            return
        if not self._busy.acquire(blocking=False):
            busy_msg = ("Executing another command. Say 'stop' to cancel."
                        if en else "Đang thực hiện lệnh khác. Nói 'dừng' để hủy.")
            self._status("busy", msg=busy_msg, language=self._current_lang)
            return

        self._abort.clear()
        self._confirm_evt.clear()
        self._confirm_val = False
        threading.Thread(target=self._worker, args=(text, self._current_lang, conversation), daemon=True).start()

    def _srv_stop(self, _req, resp):
        self._emergency_stop("service llm/stop")
        resp.success = True
        resp.message = "stopped" if self._is_en() else "đã dừng"
        return resp

    def _emergency_stop(self, why):
        self.get_logger().warn(f"DỪNG KHẨN: {why}")
        self._abort.set()
        self._stop_requested_at = time.monotonic()
        self._manager_stop_future = None
        self._pallet_stop_future = None
        self._stop_requested_wall = time.time()
        if self.cli_manager_stop.service_is_ready():
            self._manager_stop_future = self.cli_manager_stop.call_async(Trigger.Request())
        self._clarify_ctx = None
        state = self._sup_state
        age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
        busy_states = {s.upper() for s in self.get_parameter("supervisor_busy_states").value}
        cycle_active = state in busy_states and age <= self.get_parameter("supervisor_status_max_age_s").value

        if self._retrieval_state not in (None, "IDLE", "DONE", "FAILED", "CANCELED", "STOPPED", "RECOVERY"):
            if self.cli_retrieval_cancel.service_is_ready():
                self.cli_retrieval_cancel.call_async(Trigger.Request())
        elif cycle_active:
            if self.cli_mstop.service_is_ready():
                self.cli_mstop.call_async(Trigger.Request())
            else:
                self.get_logger().error("Cycle đang chạy nhưng /mission/stop chưa sẵn sàng")
        else:
            nav_id, nav_state = self._nav_latest()
            nav_active = nav_id is not None and nav_state in (1, 2, 3)
            pallet_active = self._pallet_state not in (None, "IDLE", "DONE", "CARRYING", "FAILED", "STOPPED")
            if pallet_active and self.cli_pstop.service_is_ready():
                self._pallet_stop_future = self.cli_pstop.call_async(Trigger.Request())
            if nav_active and self.cli_cancel.service_is_ready():
                self.cli_cancel.call_async(CancelGoal.Request())
            if nav_active or pallet_active:
                self.pub_cmd.publish(Twist())

            if age > self.get_parameter("supervisor_status_max_age_s").value and self.cli_mstop.service_is_ready():
                self.cli_mstop.call_async(Trigger.Request())
        if self._lift_active:
            self.pub_lift.publish(String(data="M0"))
            self._lift_active = False
        if self._motion_active:
            self.pub_cmd.publish(Twist())
            self._motion_active = False
        self._confirm_evt.set()
        self._status("stopped", msg=f"{why} (mission={state or 'unknown'})")

    def _answer(self, topics, text, language, conversation):
        with self._lock:
            topics, slots = resolve_reference(topics, text, self._subjects.pop(conversation, None))
            self._subjects[conversation] = (tuple(topics), slots)
            if len(self._subjects) > 128:
                self._subjects.popitem(last=False)
        context = self._mission_context()
        action = self._current_action
        if 'mission' in topics and action in ('relative_move', 'lift_home', 'lift_to', 'go_mission', 'quick_goal'):
            if not self.get_parameter('dry_run').value:
                task = self._describe_active_task(language)
                context['local_action'] = (f'The robot is {task}.' if language == 'en' else f'Xe đang {task}.')
        if 'resume' in topics and self._manual_checkpoint and self._manual_checkpoint.get('paused_at') is not None:
            context['manual_resume_reason'] = self._manual_resume_blocked()
        self._status("answer", msg=answer_query(topics, context, text, language, slots),
                     observational=True, conversation_id=conversation, topics=topics, language=language,
                     facts=context)

    def _worker(self, text, language="auto", conversation=""):
        en = self._is_en(language)
        self._cycle_run_id = None
        self._cycle_operation_id = None
        self._current_command_text = text
        try:
            self._status("planning", msg=text, language=language)
            ctx, self._clarify_ctx = self._clarify_ctx, None
            if ctx and ctx[3] == conversation and time.time() - ctx[2] < 90:
                if en:
                    request = (f"Original request: {ctx[0]}\nRobot asked: {ctx[1]}\n"
                               f"User replied: {text}\n"
                               "Combine into a complete request.")
                else:
                    request = (f"Yêu cầu ban đầu: {ctx[0]}\nRobot đã hỏi lại: {ctx[1]}\n"
                               f"Người dùng trả lời: {text}\n"
                               "Hãy kết hợp thành một yêu cầu hoàn chỉnh.")
                base = f"{ctx[0]} ({text})"
            else:
                request, base = text, text
            t0 = time.time()
            coordinator_owns_resume = self._sup_state == "RECOVERY" or self._retrieval_state == "RECOVERY"
            scope = resume_scope(text)
            resume_cp = (self._manual_checkpoint if resume_requested(text) and scope != "mission"
                         and (scope == "manual" or not coordinator_owns_resume) else None)
            if resume_cp is not None:
                reason = self._manual_resume_blocked()
                if reason:
                    raise RuntimeError(reason)
                plan = {"steps": deepcopy(resume_cp["steps"][resume_cp["index"]:]),
                        "say": "Resume remaining confirmed steps" if en else "Tiếp tục các bước còn lại của kế hoạch đã xác nhận"}
            elif resume_requested(text):
                plan = {"steps": [{"action": "resume_cycle", "args": {}}],
                        "say": "Resume the interrupted mission" if en else "Tiếp tục mission đang bị gián đoạn"}
            else:
                plan = self._plan(request, language)
            self.get_logger().info(f"LLM trả lời sau {time.time() - t0:.2f}s: {plan}")
            if plan.get("clarify"):
                self._clarify_ctx = (base, spoken_text(plan["clarify"]), time.time(), conversation)
                self._status("clarify", msg=plan["clarify"], language=language, conversation_id=conversation)
                return
            p = self.get_parameter
            allow_unmeasured = (p("allow_unmeasured_lift").value or p("dry_run").value)
            steps = validate_plan(plan, self._location, p("max_steps").value,
                                  allow_unmeasured_lift=allow_unmeasured)
            if not steps:
                default_noop = "No action taken." if en else "Không có hành động nào."
                self._status("noop", msg=plan.get("say") or default_noop, language=language)
                return
            if steps[0]["action"] == "get_robot_status":
                topic = steps[0]["args"]["topic"]
                self._answer([topic], text, language, conversation)
                return
            summary = self._action_summary(steps, language, self._location)
            self._confirm_evt.clear()
            self._confirm_val = False
            self.pub_plan.publish(String(data=json.dumps(
                {"say": spoken_text(plan.get("say")), "steps": steps,
                 "action_summary": summary, "language": language}, ensure_ascii=False)))
            if not p("require_confirm").value:
                self.get_logger().warning("require_confirm=false bị bỏ qua")
            self._status("waiting_confirm", msg="review_required", say=plan.get("say", ""),
                         steps=steps, action_summary=summary, confirm_required=True,
                         language=language)
            if not self._wait_confirm():
                if not self._switching_command:
                    cancel_msg = "Not confirmed" if en else "Chưa được xác nhận"
                    self._status("cancelled", msg=cancel_msg, language=language)
                return

            refreshed = validate_plan(
                {"steps": steps}, self._location, p("max_steps").value,
                allow_unmeasured_lift=allow_unmeasured)
            if refreshed != steps:
                stale_msg = ("Robot state changed; please resend the command."
                             if en else "Trạng thái robot thay đổi; hãy gửi lại lệnh.")
                self._status("stale_plan", msg=stale_msg, language=language)
                return
            try:
                self._run_plan(steps, language, resume_checkpoint=resume_cp)
            except Aborted:
                raise
            except Exception as exc:
                self._status("failed", msg=str(exc), language=language)
        except Aborted:
            if not self._switching_command:
                self._status("stopped", msg="stopped" if en else "đã dừng", language=language)
        except PlannerUnavailable as exc:
            self._status("error", msg=str(exc), language=language,
                         error_kind="planner_connection" if exc.connection_failed else "planner")
        except ValueError as exc:
            # The planner may return a mixed or incomplete action despite its prompt.
            detail = str(exc)
            if detail.startswith('Câu hỏi trạng thái phải tách'):
                question = ('Would you like the status or a control plan first?' if en else
                            'Bạn muốn kiểm tra trạng thái hay lập lệnh điều khiển trước?')
            elif detail.startswith(('retrieve_pallet.slot=', 'start_single_cycle.drop_slot=')):
                question = ('Which slot do you want?' if en else 'Bạn muốn chọn ô nào?')
            else:
                question = ''
            if question:
                self.get_logger().warning(detail)
                self._clarify_ctx = (base, question, time.time(), conversation)
                self._status('clarify', msg=question, language=language, conversation_id=conversation)
            else:
                self._status("error", msg=detail, error_kind="plan_validation", language=language)
        except Exception as exc:
            self._status("error", msg=str(exc), error_kind="plan_validation", language=language)
        finally:
            self._current_action = None
            self._current_action_args = None
            self._current_command_text = ""
            self._busy.release()

    def _wait_confirm(self):
        self._check_abort()
        timeout = self.get_parameter("confirm_timeout_s").value
        self._confirm_evt.wait(timeout)
        self._check_abort()
        return self._confirm_val

    def _run_plan(self, steps, language="auto", resume_checkpoint=None):
        en = self._is_en(language)
        queued_cycles = bool(steps) and all(
            st.get("action") == "start_single_cycle" for st in steps)
        handlers = {"set_mode": self._h_set_mode,
                    "quick_goal": self._h_quick_goal,
                    "start_auto_cycle": self._h_start_auto_cycle,
                    "start_single_cycle": self._h_start_single_cycle,
                    "retrieve_pallet": self._h_retrieve_pallet,
                    "stop_cycle": self._h_stop_cycle,
                    "cancel_navigation": self._h_cancel_navigation,
                    "resume_cycle": self._h_resume_cycle,
                    "go_mission": self._h_go_mission,
                    "pick_pallet": self._h_pick_pallet,
                    "detect_pallet": self._h_detect_pallet,
                    "lock_detection": self._h_lock_detection,
                    "drop_pallet": self._h_drop_pallet,
                    "lift_to": self._h_lift_to,
                    "lift_home": self._h_lift_home,
                    "relative_move": self._h_relative_move}
        dry = self.get_parameter("dry_run").value
        self._success_message = ""
        self._failure_message = ""
        cp = self._begin_manual_checkpoint(steps, resume_checkpoint) if not dry and not queued_cycles else None
        base_index = cp["index"] if cp else 0
        resume_started = bool(cp and cp["started"])
        try:
            if queued_cycles and not dry:
                tasks = [{"cargo_class": st["args"].get("cargo_class", "any"),
                          "quantity": st["args"].get("quantity", 1),
                          "drop_slot": st["args"].get("drop_slot", "auto")}
                         for st in steps]
                self._current_action = "start_single_cycle"
                self._current_action_args = {"queue": tasks}
                self._status("running", step=1, total=len(steps),
                             msg=("Starting ordered pallet queue" if en else
                                  "Đang khởi động hàng đợi pallet theo thứ tự"), language=language)
                if not self._start_cycle("single", queue=tasks):
                    self._status("failed", msg=self._failure_message or "Mission start failed",
                                 language=language)
                    return
                if not self._wait_cycle_complete(1, len(steps), language,
                                                 expected_run_id=self._cycle_run_id):
                    self._status("failed", msg=self._failure_message or "Mission did not complete",
                                 language=language)
                    return
                self._status("done", msg=(self._success_message or
                             ("All pallet tasks completed" if en else
                              "Đã hoàn thành toàn bộ lượt pallet")), language=language)
                return
            for i, st in enumerate(steps, 1):
                self._check_abort()
                if cp:
                    cp["index"] = base_index + i - 1
                    cp["started"] = True
                self._resume_step = resume_checkpoint is not None and resume_started and i == 1
                self._current_action = st["action"]
                self._current_action_args = st["args"]
                self._status("running", step=i, total=len(steps),
                             msg=self._action_summary([st], language, self._location)[0], language=language)
                if dry:
                    self.get_logger().info(f"[dry_run] {st}")
                    self._sleep(0.5)
                    if st["action"] == "go_mission":
                        self._location = mission_from_to(st["args"]["name"])[1]
                    elif st["action"] == "relative_move":
                        self._location = "unknown"
                    continue
                if not handlers[st["action"]](st["args"]):
                    default_fail = (f"Step {i} failed: {st['action']} {st['args']}" if en
                                    else f"Bước {i} thất bại: {st['action']} {st['args']}")
                    self._status("failed", step=i, total=len(steps),
                                 msg=self._failure_message or default_fail, language=language)
                    return
                if cp:
                    cp["index"] += 1
                    cp["started"] = False
                    cp["progress"] = {}
        finally:
            self._freeze_manual_checkpoint(cp)
            self._current_action = None
            self._current_action_args = None
        if dry:
            msg = "Preview complete" if en else "Xem trước hoàn tất"
            self._status("simulated", msg=msg, language=language)
        elif (not queued_cycles and
              any(s["action"] in ("start_auto_cycle", "start_single_cycle", "resume_cycle") for s in steps)):
            msg = "Cycle started" if en else "Đã khởi động chu trình"
            self._status("started", msg=msg, language=language)
        else:
            self._status("done", msg=self._success_message or ("Completed" if en else "Hoàn thành"),
                         language=language)

    def _wait_cycle_complete(self, step, total, language, expected_run_id=None):
        """Wait for this sequential single-cycle run to return home and publish DONE."""
        en = language == "en"
        deadline = time.monotonic() + float(
            self.get_parameter("cycle_completion_timeout_s").value)
        active_states = {"GOTO_PICK", "PICKING", "GOTO_D", "DROPPING", "WAITING_SLOT",
                         "RETURN_HOME", "STOPPING"}
        saw_active = False
        while time.monotonic() < deadline:
            self._check_abort()
            age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
            if age <= self.get_parameter("supervisor_status_max_age_s").value:
                state = self._sup_state
                snapshot = self._sup_status or {}
                if expected_run_id is None or snapshot.get("run_id") != expected_run_id:
                    time.sleep(0.2)
                    continue
                self._cycle_operation_id = snapshot.get("operation_id")
                if state in active_states:
                    saw_active = True
                    # The configured deadline is a warning while the same run
                    # still reports active work or automatic recovery.
                    deadline = max(deadline, time.monotonic() + 5.0)
                elif state == "DONE" and saw_active:
                    nav = snapshot.get("nav_mission") or {}
                    if (snapshot.get("cargo") is False
                            and snapshot.get("nav_status_fresh") is True
                            and snapshot.get("nav_goal_active") is False
                            and nav.get("state") == "SUCCEEDED"):
                        return True
                elif state in ("FAILED", "RECOVERY", "STOPPED"):
                    self._failure_message = ((self._sup_status or {}).get("msg") or
                                             (f"Cycle {step}/{total} entered {state}" if en else
                                              f"Chu trình {step}/{total} chuyển sang {state}"))
                    return False
                elif state == "IDLE" and saw_active:
                    self._failure_message = ("Cycle ended in IDLE before DONE" if en else
                                             "Chu trình về IDLE trước khi xác nhận DONE")
                    return False
            self._status("running", step=step, total=total,
                         msg=(f"Waiting for cycle {step}/{total} to complete" if en else
                              f"Đang chờ chu trình {step}/{total} hoàn tất"),
                         language=language)
            time.sleep(0.2)
        self._failure_message = (f"Timed out waiting for cycle {step}/{total} to complete" if en else
                                 f"Quá thời gian chờ chu trình {step}/{total} hoàn tất")
        return False

    @staticmethod
    def _action_summary(steps, language, initial_location):
        en = language == "en"
        loc = {"home": "HOME", "p1": "P_1", "d2": "D_2", "d1": "D_1"}
        current = initial_location
        summary = []
        for step in steps:
            action, args = step["action"], step["args"]
            if action == "set_mode":
                summary.append(f"Set mode to {args['mode']}" if en else f"Chuyển mode sang {args['mode']}")
            elif action == "quick_goal":
                summary.append(f"Navigate to {args['target']}" if en else f"Đi tới quick goal {args['target']}")
                current = {"HOME": "home", "P_1": "p1", "D_2": "d2", "D_1": "d1"}[args["target"]]
            elif action == "go_mission":
                source, dest = mission_from_to(args["name"])
                route = f"{loc.get(source, 'anywhere' if en else 'vị trí hiện tại')} {'to' if en else 'đến'} {loc[dest]}"
                summary.append(f"Run route {route}" if en else f"Chạy tuyến {route}")
                current = dest
            elif action == "pick_pallet":
                target = loc.get(current, "unknown location")
                summary.append(f"Pick pallet at {target}" if en else f"Nhận pallet tại {target}")
            elif action == "detect_pallet":
                summary.append("Detect pallet without picking" if en
                               else "Nhận diện pallet, không lấy")
            elif action == "lock_detection":
                summary.append("Stop detection and lock pallet pose" if en
                               else "Dừng detect và chốt pose pallet")
            elif action == "drop_pallet":
                summary.append("Place pallet at D_1" if en else "Đặt pallet tại D_1")
            elif action == "lift_to":
                cm = (float(args["height_cm"]) if "height_cm" in args
                      else LIFT_LEVELS[args["level"]])
                summary.append(f"Move lift to {cm:g} cm" if en else f"Nâng/hạ càng tới {cm:g} cm")
            elif action == "lift_home":
                summary.append("Lower lift to HOME" if en else "Hạ càng về HOME")
            elif action == "relative_move":
                direction = "forward" if args["direction"] == "forward" else "backward"
                summary.append((f"Move {direction} {args['distance_m']:g} m at {args['speed_mps']:g} m/s"
                                if en else
                                f"Đi {'tiến' if direction == 'forward' else 'lùi'} {args['distance_m']:g} m với tốc độ {args['speed_mps']:g} m/s"))
            elif action == "start_auto_cycle":
                summary.append("Start automatic pallet cycle" if en else "Chạy chu trình pallet Auto")
            elif action == "retrieve_pallet":
                summary.append((f"Retrieve pallet from slot {args['slot']} to D_2, then return HOME"
                                if en else
                                f"Lấy pallet từ ô {args['slot']} về D_2 rồi trở về HOME"))
            elif action == "start_single_cycle":
                cargo = args.get("cargo_class", "any")
                qty = args.get("quantity", 1)
                slot = args.get("drop_slot", "auto")
                cargo_label = {"any": "any cargo", "medium": "medium boxes",
                               "tall": "tall boxes"}.get(cargo, cargo)
                destination = f" in slot {slot}" if slot != "auto" else " on a compatible shelf"
                summary.append((f"Pick and place {qty} {cargo_label}{destination}, then return HOME"
                                if en else
                                f"Lấy và đặt {qty} sản phẩm loại {cargo_label} "
                                f"{'vào ô ' + slot if slot != 'auto' else 'lên ô kệ phù hợp'}, rồi về HOME"))
            elif action == "stop_cycle":
                summary.append("Stop pallet cycle" if en else "Dừng chu trình pallet")
            elif action == "resume_cycle":
                summary.append("Resume pallet cycle" if en else "Tiếp tục chu trình pallet")
            elif action == "cancel_navigation":
                summary.append("Cancel active Nav2 goal" if en else "Hủy goal Nav2 đang chạy")
        return summary

    def _plan(self, text, language="auto"):
        status = self._mission_context()
        system = build_system_prompt(self._location, status, language)
        failures = []
        for name in self.get_parameter("backends").value:
            cfg = BACKENDS.get(name)
            started = time.monotonic()
            try:
                if cfg is None:
                    raise RuntimeError("unknown backend")
                if cfg["kind"] == "gemini":
                    raw = self._call_gemini(cfg, system, text)
                else:
                    raw = self._call_openai(name, cfg, system, text)
                plan = extract_json(raw)
                self._last_planner_model = {'backend': name, 'model': cfg['model']}
                return plan
            except Exception as e:  # noqa: BLE001
                elapsed = time.monotonic() - started
                model = cfg.get("model", "unknown") if cfg else "unknown"
                kind, detail = self._backend_failure(e)
                self.get_logger().warning(f"LLM backend={name} model={model} result={kind} elapsed={elapsed:.2f}s detail={detail}")
                failures.append((name, kind, detail))
        connection_failed = bool(failures) and all(
            kind in ('connection_error', 'timeout') for _, kind, _ in failures)
        raise PlannerUnavailable(self._user_backend_failure(failures, language), connection_failed)

    @staticmethod
    def _backend_failure(exc):
        if isinstance(exc, requests.exceptions.Timeout):
            return "timeout", "request timed out"
        if isinstance(exc, requests.exceptions.ConnectionError):
            return "connection_error", "connection failed"
        if isinstance(exc, requests.exceptions.HTTPError):
            response = exc.response
            code = response.status_code if response is not None else 0
            category = "http_4xx" if 400 <= code < 500 else ("http_5xx" if 500 <= code < 600 else "http_error")
            return category, f"HTTP {code}"
        message = str(exc)
        if "Thiếu biến môi trường" in message or "Missing environment variable" in message:
            return "missing_key", message
        if ("invalid" in message.lower() or "json" in message.lower() or isinstance(exc, json.JSONDecodeError)):
            return "invalid_response", "response was not valid planner JSON"
        return "error", type(exc).__name__

    @staticmethod
    def _user_backend_failure(failures, language="vi"):
        en = language == "en"
        if not failures:
            return "No LLM backend configured." if en else "Không có LLM backend nào được cấu hình."
        if en:
            labels = {"timeout": "timed out", "connection_error": "connection failed",
                      "missing_key": "missing API key", "http_4xx": "API rejected request",
                      "http_5xx": "API server error", "http_error": "API HTTP error",
                      "invalid_response": "returned invalid plan", "error": "encountered error"}
            causes = "; ".join(f"{name}: {labels.get(kind, 'error')}" + (f" {detail}" if kind.startswith("http_") else "") for name, kind, detail in failures)
            return f"Failed to get plan from LLM ({causes})."
        labels = {"timeout": "hết thời gian chờ", "connection_error": "không kết nối được",
                  "missing_key": "thiếu API key", "http_4xx": "API từ chối yêu cầu",
                  "http_5xx": "dịch vụ API đang lỗi", "http_error": "API trả lỗi HTTP",
                  "invalid_response": "trả về kế hoạch không hợp lệ", "error": "gặp lỗi"}
        causes = "; ".join(f"{name}: {labels.get(kind, 'gặp lỗi')}" + (f" {detail}" if kind.startswith("http_") else "") for name, kind, detail in failures)
        return f"Không lấy được kế hoạch từ LLM ({causes})."

    def _post_backend(self, name, cfg, url, body, headers=None):
        timeout = (5.0, float(self.get_parameter("llm_timeout_s").value))
        for attempt in range(2):
            started = time.monotonic()
            try:
                response = requests.post(url, json=body, headers=headers or {}, timeout=timeout)
                elapsed = time.monotonic() - started
                if response.status_code >= 400:
                    response.raise_for_status()
                return response
            except (requests.exceptions.Timeout, requests.exceptions.ConnectionError):
                if attempt == 1:
                    raise
                time.sleep(0.5)

    def _call_gemini(self, cfg, system, text):
        key = os.environ.get(cfg["key_env"], "")
        if not key:
            raise RuntimeError(f"Thiếu biến môi trường {cfg['key_env']}")
        url = f"https://generativelanguage.googleapis.com/v1beta/models/{cfg['model']}:generateContent"
        body = {
            "system_instruction": {"parts": [{"text": system}]},
            "contents": [{"role": "user", "parts": [{"text": text}]}],
            "generationConfig": {"temperature": 0, "maxOutputTokens": 2048,
                                 "thinkingConfig": {"thinkingLevel": "MINIMAL"},
                                 "responseMimeType": "application/json"},
        }
        r = self._post_backend("gemini", cfg, url, body, headers={"x-goog-api-key": key})
        return r.json()["candidates"][0]["content"]["parts"][0]["text"]

    def _call_openai(self, name, cfg, system, text):
        headers = {}
        if cfg["key_env"]:
            key = os.environ.get(cfg["key_env"], "")
            if not key:
                raise RuntimeError(f"Thiếu biến môi trường {cfg['key_env']}")
            headers["Authorization"] = f"Bearer {key}"
        body = {"model": cfg["model"], "temperature": 0,
                "response_format": {"type": "json_object"},
                "messages": [{"role": "system", "content": system},
                             {"role": "user", "content": text}]}
        r = self._post_backend(name, cfg, cfg["base_url"] + "/chat/completions", body, headers=headers)
        return r.json()["choices"][0]["message"]["content"]

    def _on_battery(self, msg):
        value = float(msg.percentage)
        scale = float(self.get_parameter("battery_percentage_scale").value)
        self._battery_percent = (value * 100.0 / scale if math.isfinite(value)
                                 and math.isfinite(scale) and scale > 0 and 0 <= value <= scale else None)
        self._battery_t = time.time()

    def _mission_context(self):
        now = time.time()
        age = lambda stamp: now - stamp if stamp else None
        context = build_context(
            supervisor=self._sup_status, supervisor_age=age(self._sup_state_t),
            pallet=self._pallet_status, pallet_age=age(self._pallet_state_t),
            retrieval=self._retrieval_status, retrieval_age=age(self._retrieval_state_t),
            lift=self._lift_cm, lift_age=(time.monotonic() - self._lift_t if self._lift_t else None),
            battery=self._battery_percent, battery_age=age(self._battery_t),
            mode=self._op_mode, location=self._location,
            max_age=float(self.get_parameter("supervisor_status_max_age_s").value))
        context['llm'] = {
            'configured': [{'backend': name, 'model': BACKENDS.get(name, {}).get('model', 'unknown')}
                           for name in self.get_parameter('backends').value],
            'last_success': getattr(self, '_last_planner_model', None),
        }
        return context

    def _fresh_supervisor_status(self):
        age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
        max_age = self.get_parameter("supervisor_status_max_age_s").value
        if not isinstance(self._sup_status, dict) or age > max_age:
            return {"state": self._sup_state or "IDLE", "cargo": False, "can_start": True}
        data = dict(self._sup_status)
        if not isinstance(data.get("cargo"), bool):
            data["cargo"] = False
        return data

    def _fresh_pallet_status(self):
        if (not isinstance(self._pallet_status, dict) or not self._pallet_state_t
                or time.time() - self._pallet_state_t > 3.0):
            return {"state": self._pallet_state or "IDLE", "cargo": False}
        data = dict(self._pallet_status)
        if not isinstance(data.get("cargo"), bool):
            data["cargo"] = False
        return data

    def _require_cycle_idle(self, allow_cargo=False):
        if self._retrieval_state not in (None, "IDLE", "DONE", "FAILED", "CANCELED", "STOPPED"):
            raise RuntimeError("Retrieval owns the robot; Resume or cancel its recovery first")
        if (time.time() - self._sup_state_t > self.get_parameter("supervisor_status_max_age_s").value
                or not isinstance((self._sup_status or {}).get("cargo"), bool)):
            raise RuntimeError("Fresh supervisor and cargo status required")
        status = self._fresh_supervisor_status()
        st, t = self._sup_state, self._sup_state_t
        busy = {s.upper() for s in self.get_parameter("supervisor_busy_states").value}
        age = time.time() - t if t else float("inf")
        max_age = self.get_parameter("supervisor_status_max_age_s").value
        if st in busy and age <= max_age:
            msg = (f"Mission supervisor is busy ({st})" if self._is_en()
                   else f"Mission supervisor đang bận ({st})")
            raise RuntimeError(msg)
        
        carrying = status.get("cargo") is True
        if carrying and not allow_cargo:
            msg = ("Robot is carrying a pallet; only moving to D_1 or appropriate cycle actions are allowed"
                   if self._is_en() else "Robot đang mang pallet; chỉ cho phép đi tới D_1 và thao tác cycle phù hợp")
            raise RuntimeError(msg)
        return True

    def _require_stationary(self, allow_cargo=False):
        self._require_cycle_idle(allow_cargo=allow_cargo)

    def _check_node_pose(self, target):
        """Mềm hóa kiểm tra vị trí thành cảnh báo log, không throw error làm kẹt lệnh."""
        try:
            response = self._call(self.cli_node_check[target], Trigger.Request(), timeout=5.0)
            if not response.success:
                self.get_logger().warn(f"Check node pose {target}: {response.message}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Lỗi khi check node pose {target}: {exc}")

    def _h_set_mode(self, args):
        if self.get_parameter("dry_run").value:
            return True
        mode = args["mode"]
        resp = self._call(self.cli_set_mode, SetBool.Request(data=(mode == "AUTO")), timeout=8.0)
        if not resp.success:
            self.get_logger().error(f"Đổi mode thất bại: {resp.message}")
            return False
        self._op_mode = mode
        return True

    def _h_quick_goal(self, args):
        self._require_cycle_idle()
        if self._op_mode != "MANUAL":
            self.get_logger().info("Chuyển MANUAL mode cho Quick Goal...")
            self._h_set_mode({"mode": "MANUAL"})
            time.sleep(0.3)
        target = args["target"]
        base_id, _ = self._nav_latest()
        response = self._call(self.cli_quick_goal[target], Trigger.Request(), timeout=8.0)
        if not response.success:
            self.get_logger().error(f"Quick goal {target} thất bại: {response.message}")
            return False
        ok = self._wait_nav_goal(base_id)
        if ok:
            self._location = {"HOME": "home", "P_1": "p1", "D_2": "d2", "D_1": "d1"}[target]
        else:
            self._location = "unknown"
        return ok

    def _start_cycle(self, run_mode, cargo_class="any", quantity=1, drop_slot="auto", queue=None):
        self._failure_message = ""
        en = self._is_en()

        def fail(message):
            self._failure_message = str(message)
            self.get_logger().error(f"start_{run_mode}_cycle: {self._failure_message}")
            return False

        if self.get_parameter("dry_run").value:
            return True

        try:
            status = self._fresh_supervisor_status()

            # RÀNG BUỘC CỐT LÕI: Chỉ chặn khi xe BÁO ĐANG MANG PALLET THẬT (cargo is True)
            if status.get("cargo") is True:
                return fail("Cannot start while carrying a pallet (cargo=True)" if en
                            else "Không thể start khi xe đang mang pallet (cargo=True)")

            if self._op_mode != "AUTO":
                self.get_logger().info("Chuyển mode sang AUTO trước khi chạy cycle...")
                self._h_set_mode({"mode": "AUTO"})
                time.sleep(0.3)

            req = SetParameters.Request()
            req.parameters = [
                Parameter("run_mode", Parameter.Type.STRING, run_mode).to_parameter_msg(),
                Parameter("target_cargo_class", Parameter.Type.STRING,
                          "" if cargo_class == "any" else cargo_class).to_parameter_msg(),
                Parameter("target_quantity", Parameter.Type.INTEGER,
                          int(quantity)).to_parameter_msg(),
                Parameter("target_drop_slot", Parameter.Type.STRING,
                          "" if drop_slot == "auto" else drop_slot).to_parameter_msg(),
                Parameter("target_queue_json", Parameter.Type.STRING,
                          json.dumps(queue, ensure_ascii=False) if queue else "").to_parameter_msg(),
            ]
            param_result = self._call(self.cli_cycle_params, req, timeout=5.0)
            if not param_result or not param_result.results or not all(
                    item.successful for item in param_result.results):
                return fail("Supervisor rejected mission request parameters" if en else
                            "Supervisor từ chối tham số mission")

            # Status is published asynchronously. Snapshot its revision before
            # /mission/start so a previous run's terminal FAILED/RECOVERY state
            # cannot be mistaken for failure of the newly accepted run.
            start_status_revision = self._sup_status_revision
            previous_run_id = (self._sup_status or {}).get("run_id")
            result = self._call(self.cli_cycle_start, Trigger.Request(), timeout=5.0)
            if not result.success:
                return fail(f"Supervisor rejected /mission/start: {result.message}" if en
                            else f"Supervisor từ chối /mission/start: {result.message}")

            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                self._check_abort()
                age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
                state = self._sup_state
                is_new_status = self._sup_status_revision > start_status_revision
                if (is_new_status and
                        age <= self.get_parameter("supervisor_status_max_age_s").value):
                    current_run_id = (self._sup_status or {}).get("run_id")
                    if not current_run_id or current_run_id == previous_run_id:
                        time.sleep(0.1)
                        continue
                    self._cycle_run_id = current_run_id
                    self._cycle_operation_id = (self._sup_status or {}).get("operation_id")
                    if state in ("RECOVERY", "FAILED"):
                        return fail(f"Cycle entered {state}" if en else f"Cycle chuyển sang {state}")
                    if state not in (None, "IDLE", "DONE", "FAILED", "STOPPED", "RECOVERY"):
                        self._failure_message = ""
                        return True
                time.sleep(0.1)

            return fail("/mission/start was accepted but no active status with a new run_id was confirmed" if en
                        else "/mission/start đã nhận nhưng chưa xác nhận được trạng thái hoạt động với run_id mới")

        except Aborted:
            raise
        except Exception as exc:  # noqa: BLE001
            return fail(f"Error starting cycle: {type(exc).__name__}: {exc}" if en
                        else f"Lỗi khi khởi động cycle: {type(exc).__name__}: {exc}")

    def _h_start_auto_cycle(self, _args):
        return self._start_cycle("auto")

    def _h_start_single_cycle(self, args):
        return self._start_cycle("single", args.get("cargo_class", "any"),
                                 args.get("quantity", 1), args.get("drop_slot", "auto"))

    def _h_retrieve_pallet(self, args):
        """Start the shelf-retrieval coordinator in AUTO mode and wait for it."""
        slot = str(args.get("slot", "")).strip().upper()
        if slot not in ("A1", "A2", "B1", "B2"):
            raise RuntimeError(f"Invalid retrieval slot: {slot}")
        if self.get_parameter("dry_run").value:
            return True
        previous = self._retrieval_status or {}
        previous_run = previous.get('run_id')
        if (previous.get('state') not in (None, 'IDLE', 'DONE', 'FAILED', 'CANCELED', 'STOPPED')
                and time.time() - self._retrieval_state_t <= 3.):
            raise RuntimeError('Retrieval is already active or paused; use its Resume control')
        if self._op_mode != "AUTO":
            self._h_set_mode({"mode": "AUTO"})
            self._sleep(0.3)
        req = SetParameters.Request()
        req.parameters = [Parameter(
            "target_slot", Parameter.Type.STRING, slot).to_parameter_msg()]
        result = self._call(self.cli_retrieval_params, req, timeout=5.0)
        if not result or not result.results or not all(item.successful for item in result.results):
            raise RuntimeError(f"Could not select retrieval slot {slot}")
        self._retrieval_state = None
        self._retrieval_state_t = 0.0
        request_at = time.time()
        self.get_logger().info(
            f'[RETRIEVAL_COMMAND] source=LLM action=start slot={slot} '
            f'previous_run_id={previous_run} previous_state={previous.get("state")}')
        try:
            response = self._call(self.cli_retrieval_start, Trigger.Request(), timeout=5.0)
        except TimeoutError:
            # The start request may have been accepted. The run ID, not a
            # repeated Trigger request, resolves this uncertain outcome.
            response = None
        self.get_logger().info(
            f'[RETRIEVAL_COMMAND] source=LLM action=start result='
            f'{"unknown" if response is None else response.success} '
            f'slot={slot} previous_run_id={previous_run} '
            f'current_run_id={(self._retrieval_status or {}).get("run_id")} '
            f'current_state={self._retrieval_state}')
        if response is not None and not response.success:
            raise RuntimeError(response.message or f"Retrieval rejected for {slot}")
        deadline = time.monotonic() + float(self.get_parameter("retrieval_timeout_s").value)
        terminal = {"DONE", "FAILED", "CANCELED", "STOPPED", "RECOVERY"}
        while True:
            self._check_abort()
            state = self._retrieval_state
            status = self._retrieval_status or {}
            if (self._retrieval_state_t < request_at or not status.get('run_id')
                    or status.get('run_id') == previous_run):
                self._sleep(0.2)
                continue
            if state == "DONE":
                return True
            if state in terminal:
                raise RuntimeError((self._retrieval_status or {}).get(
                    "message") or f"Retrieval ended in {state}")
            if time.monotonic() >= deadline:
                self.get_logger().warn(
                    f'Retrieval {status.get("run_id")} exceeded warning deadline; '
                    f'current state={state or "unknown"}; retaining the active run')
                deadline = time.monotonic() + float(self.get_parameter("retrieval_timeout_s").value)
            self._sleep(0.2)

    def _h_stop_cycle(self, _args):
        if self.get_parameter("dry_run").value:
            return True
        result = self._call(self.cli_mstop, Trigger.Request(), timeout=5.0)
        return bool(result.success)

    def _h_cancel_navigation(self, _args):
        if self.get_parameter("dry_run").value:
            return True
        self._require_cycle_idle(allow_cargo=True)
        result = self._call(self.cli_cancel, CancelGoal.Request(), timeout=5.0)
        self.pub_cmd.publish(Twist())
        return True

    def _h_resume_cycle(self, _args):
        if (self._retrieval_state == "RECOVERY" and time.time() - self._retrieval_state_t <= 3.):
            before = self._retrieval_status or {}
            self.get_logger().info(
                f'[RETRIEVAL_COMMAND] source=LLM action=resume slot={before.get("slot")} '
                f'run_id={before.get("run_id")} previous_state={before.get("state")}')
            result = self._call(self.cli_retrieval_resume, Trigger.Request(), timeout=5.)
            self.get_logger().info(
                f'[RETRIEVAL_COMMAND] source=LLM action=resume result={result.success} '
                f'run_id={(self._retrieval_status or {}).get("run_id")} '
                f'current_state={self._retrieval_state}')
            if not result.success:
                self._failure_message = result.message
            return bool(result.success)
        if self.get_parameter("dry_run").value:
            return True
        if self._op_mode != "AUTO":
            if not self._h_set_mode({"mode": "AUTO"}):
                return False
            time.sleep(0.3)
        client = self.create_client(Trigger, "/mission/resume", callback_group=self._cb)
        result = self._call(client, Trigger.Request(), timeout=5.0)
        if not result.success:
            self._failure_message = result.message
        return bool(result.success)

    def _h_go_mission(self, a):
        name = a["name"]
        source, destination = mission_from_to(name)
        if self._op_mode != "MANUAL":
            self._h_set_mode({"mode": "MANUAL"})
            time.sleep(0.3)
        cargo = self._fresh_supervisor_status().get("cargo") is True
        self._require_cycle_idle(allow_cargo=cargo and destination == "d1")
        if cargo and destination != "d1":
            msg = ("Robot is carrying a pallet; it can only move to D_1 to drop it"
                   if self._is_en() else "Xe đang mang pallet; chỉ được đi tới D_1 để giao pallet")
            raise RuntimeError(msg)
        if source is not None and not self._resume_step:
            self._check_node_pose({"home": "HOME", "p1": "P_1", "d2": "D_2", "d1": "D_1"}[source])
        base_id, _ = self._nav_latest()
        client = self.cli_mission[name]
        if self._resume_step:
            client = self.create_client(Trigger, f"mission_manager/trigger_{name}_free_ack", callback_group=self._cb)
        response = self._call(client, Trigger.Request(), timeout=15.0)
        if not response.success:
            self.get_logger().error(f"Mission {name} không gửi được goal: {response.message}")
            return False
        ok = self._wait_nav_goal(base_id)
        self._location = mission_from_to(name)[1] if ok else "unknown"
        return ok

    def _wait_nav_goal(self, base_id):
        t0 = time.time()
        start_to = self.get_parameter("nav_start_timeout_s").value
        total_to = self.get_parameter("mission_timeout_s").value
        goal = None
        while time.time() - t0 < total_to:
            self._check_abort()
            gid, status = self._nav_latest()
            if goal is None:
                if gid is not None and gid != base_id:
                    goal = gid
                elif time.time() - t0 > start_to:
                    self.get_logger().error("Không thấy goal Nav2 mới sau khi trigger")
                    return False
            if goal is not None and gid == goal and status in TERMINAL:
                return status == GOAL_SUCCEEDED
            time.sleep(0.2)
        return False

    def _set_pallet_param(self, name, value):
        try:
            if not self.cli_set_param.wait_for_service(timeout_sec=2.0):
                return False
            req = SetParameters.Request()
            req.parameters = [Parameter(name, Parameter.Type.STRING, value).to_parameter_msg()]
            resp = self._call(self.cli_set_param, req, timeout=5.0)
            return all(x.successful for x in resp.results)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"Không set duoc {name}={value!r}: {e}")
            return False

    def _h_detect_pallet(self, _args):
        self._require_cycle_idle()
        self._ensure_manual_idle()
        pallet = self._fresh_pallet_status()
        if pallet.get("cargo") is True:
            raise RuntimeError("Robot đang mang pallet; không thể bắt đầu detect pallet mới")

        # Drop any cached sample count from a previous detection session.
        self._pallet_status = None
        self._pallet_state = None
        self._pallet_state_t = 0.0
        response = self._call(self.cli_detect, SetBool.Request(data=True))
        if not response.success:
            raise RuntimeError(f"Không khởi động được detect: {response.message}")

        timeout = float(self.get_parameter("detect_sample_timeout_s").value)
        first_sample_deadline = time.monotonic() + timeout
        warmup = max(3.0, float(self.get_parameter("detect_seconds").value))
        deadline = first_sample_deadline + warmup
        warmup_deadline = None
        sample_count = 0
        while time.monotonic() < deadline:
            self._check_abort()
            status = self._pallet_status
            fresh = self._pallet_state_t > 0 and time.time() - self._pallet_state_t <= 3.0
            samples = status.get("samples") if isinstance(status, dict) else None
            if fresh and isinstance(samples, int) and samples > 0:
                sample_count = samples
                if warmup_deadline is None:
                    warmup_deadline = time.monotonic() + warmup
                if time.monotonic() < warmup_deadline or samples < 2:
                    self._sleep(0.2)
                    continue
                live = status.get("live") if isinstance(status.get("live"), dict) else {}
                if all(k in live for k in ("x", "y", "yaw_deg")):
                    self._success_message = (
                        f"Pallet detected; map pose x={live['x']}, y={live['y']} m, "
                        f"yaw={live['yaw_deg']}° (not stabilized; not picked)" if self._is_en()
                        else f"Đã nhận diện pallet; pose map x={live['x']}, y={live['y']} m, "
                             f"góc={live['yaw_deg']}° (chưa chốt pose ổn định, chưa lấy hàng)")
                else:
                    self._success_message = (
                        "Pallet sample received; detection remains active (not picked)"
                        if self._is_en() else
                        "Đã nhận được mẫu pallet; detector vẫn bật để tiếp tục cập nhật "
                        "(chưa lấy hàng)")
                return True
            if warmup_deadline is None and time.monotonic() >= first_sample_deadline:
                break
            self._sleep(0.2)

        self._failure_message = (
            f"Detect is active but only {sample_count} pallet sample(s) arrived "
            f"within {timeout:g}s. The detector is still running; check camera/pose "
            "and retry."
            if self._is_en() else
            f"Detect đã bật nhưng chỉ nhận được {sample_count} mẫu pallet trong {timeout:g}s. "
            "Detector vẫn đang chạy; kiểm tra camera/pose rồi thử detect lại.")
        return False

    def _h_lock_detection(self, _args):
        self._require_cycle_idle()
        self._ensure_manual_idle()
        if self._resume_step:
            recovery = (self._pallet_status or {}).get('recovery') or {}
            cp = recovery.get('checkpoint') or {}
            if cp.get('state') == 'READY':
                result = self._call(self.cli_presume, Trigger.Request())
                return bool(result.success)
            if self._pallet_state == 'READY':
                return True
            if cp.get('state') == 'DETECTING':
                result = self._call(self.cli_presume, Trigger.Request())
                if not result.success or not self._h_detect_pallet({}):
                    return False
        response = self._call(self.cli_detect, SetBool.Request(data=False))
        if not response.success:
            self._failure_message = response.message or (
                "Không thể chốt mẫu detect" if not self._is_en()
                else "Could not lock pallet detection samples")
            return False
        self._pallet_state = "READY"
        self._pallet_state_t = time.time()
        self._pallet_status = dict(self._pallet_status or {})
        self._pallet_status.update({"state": "READY", "cargo": False})
        self._success_message = (
            f"Pallet pose locked: {response.message}" if self._is_en()
            else f"Đã chốt pose pallet: {response.message}")
        return True

    def _h_pick_pallet(self, _a):
        self._require_cycle_idle()
        if self._resume_step:
            return self._resume_pallet_operation()
        en = self._is_en()
        if self._op_mode != "MANUAL":
            self._h_set_mode({"mode": "MANUAL"})
            time.sleep(0.3)
        pallet = self._fresh_pallet_status()
        if pallet.get("cargo") is True or self._fresh_supervisor_status().get("cargo") is True:
            raise RuntimeError("Cannot pick pallet while already carrying cargo" if en
                               else "Không nhận pallet khi xe đang mang tải")
        p = self.get_parameter
        if p("set_after_pick_hold").value:
            self._set_pallet_param("after_pick", "hold")
        pallet_state_fresh = (self._pallet_state_t > 0 and
                              time.time() - self._pallet_state_t <= 3.0)
        pose_already_locked = pallet_state_fresh and self._pallet_state == "READY"
        if p("detect_first").value and not pose_already_locked:
            detect_on = self._call(self.cli_detect, SetBool.Request(data=True))
            if not detect_on.success:
                raise RuntimeError(f"Pallet detect failed to start: {detect_on.message}" if en
                                   else f"Pallet detect không khởi động: {detect_on.message}")
            self._sleep(p("detect_seconds").value)
            detect_off = self._call(self.cli_detect, SetBool.Request(data=False))
            if not detect_off.success:
                raise RuntimeError(f"Pallet detect failed to lock pose: {detect_off.message}" if en
                                   else f"Pallet detect không chốt được: {detect_off.message}")

        operation_id = (self._pallet_status or {}).get('operation_id', -1)
        self._pallet_state = None
        r = self._call(self.cli_go, Trigger.Request())
        if not r.success:
            self.get_logger().error(f"go_to_pallet từ chối: {r.message}")
            return False
        return self._wait_pallet_done(after_operation_id=operation_id)

    def _h_drop_pallet(self, _a):
        self._require_cycle_idle(allow_cargo=True)
        if self._resume_step:
            return self._resume_pallet_operation()
        if self._op_mode != "MANUAL":
            self._h_set_mode({"mode": "MANUAL"})
            time.sleep(0.3)
        operation_id = (self._pallet_status or {}).get('operation_id', -1)
        self._pallet_state = None
        r = self._call(self.cli_drop, Trigger.Request())
        if not r.success:
            self.get_logger().error(f"drop (pallet_mission/drop) từ chối: {r.message}")
            return False
        return self._wait_pallet_done(after_operation_id=operation_id)

    def _resume_pallet_operation(self):
        if time.time() - self._pallet_state_t > 3.:
            raise RuntimeError('Pallet status stale; cannot resume')
        data = self._pallet_status or {}
        recovery = data.get('recovery') or {}
        if not recovery.get('checkpoint'):
            expected = 'DONE' if self._current_action == 'drop_pallet' else 'CARRYING'
            if data.get('state') == expected:
                return True
            raise RuntimeError('No interrupted pallet checkpoint; inspect before a new action')
        if not recovery.get('available'):
            raise RuntimeError(recovery.get('reason') or 'Pallet Resume blocked')
        started = time.time()
        result = self._call(self.cli_presume, Trigger.Request())
        if not result.success:
            raise RuntimeError(result.message)
        checkpoint_state = recovery['checkpoint']['state']
        if self._current_action == 'pick_pallet' and checkpoint_state == 'READY':
            result = self._call(self.cli_go, Trigger.Request())
            if not result.success:
                raise RuntimeError(result.message)
        elif self._current_action == 'pick_pallet' and checkpoint_state == 'DETECTING':
            self._resume_step = False
            try:
                return self._h_pick_pallet({})
            finally:
                self._resume_step = True
        return self._wait_pallet_done(after_t=started, after_resume_id=recovery['checkpoint']['id'])

    def _wait_pallet_done(self, after_t=None, after_operation_id=None, after_resume_id=0):
        p = self.get_parameter
        done = {s.upper() for s in p("pallet_done").value}
        fail = {s.upper() for s in p("pallet_fail").value}
        idle = {s.upper() for s in p("pallet_idle").value}
        deadline, seen_active = time.time() + p("pallet_timeout_s").value, after_t is not None
        while time.time() < deadline:
            self._check_abort()
            data = self._pallet_status or {}
            resumed_id = data.get('last_resumed_checkpoint_id')
            operation_id = data.get('operation_id')
            if (time.time() - self._pallet_state_t > 3.
                    or (after_t is not None and self._pallet_state_t <= after_t)
                    or (after_resume_id > 0 and (not isinstance(resumed_id, int)
                        or isinstance(resumed_id, bool) or resumed_id < after_resume_id))
                    or (after_operation_id is not None and (not isinstance(operation_id, int)
                        or isinstance(operation_id, bool) or operation_id <= after_operation_id))):
                self._sleep(.1)
                continue
            st = self._pallet_state
            if st and st not in done | fail | idle:
                seen_active = True
                deadline = max(deadline, time.time() + 5.0)
            if st in fail:
                return False
            if st in done and (seen_active or after_operation_id is not None):
                return True
            time.sleep(0.2)
        return False

    def _h_lift_to(self, a):
        self._require_cycle_idle(allow_cargo=True)
        status = self._fresh_supervisor_status()
        cm = (float(a["height_cm"]) if "height_cm" in a
              else LIFT_LEVELS[a["level"]])
        # Manual LLM lift commands are allowed while carrying cargo. The
        # operator may intentionally lower or raise the forks; the hardware
        # controller still enforces its own motion limits.
        self._ensure_manual_idle()
        command = "HOME" if cm <= 0.0 else f"H{cm:g}"
        return self._command_lift(command, cm)

    def _h_lift_home(self, _args):
        status = self._fresh_supervisor_status()
        age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
        if age > self.get_parameter("supervisor_status_max_age_s").value or not isinstance(
                status.get("cargo"), bool):
            raise RuntimeError("Không có trạng thái tải pallet mới; từ chối hạ lift về HOME")
        # Explicit operator command: allow HOME even while cargo is present.
        self._require_cycle_idle(allow_cargo=True)
        self._ensure_manual_idle()
        return self._command_lift("HOME", 0.0)

    def _ensure_manual_idle(self):
        age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
        if age > self.get_parameter("supervisor_status_max_age_s").value:
            raise RuntimeError("Không có trạng thái supervisor mới; từ chối thao tác MANUAL")
        nav_id, nav_state = self._nav_latest()
        if nav_id is not None and nav_state in (1, 2, 3):
            raise RuntimeError("Đang có goal Nav2; hãy kết thúc hoặc hủy goal trước")
        if self._op_mode != "MANUAL":
            if not self._h_set_mode({"mode": "MANUAL"}):
                raise RuntimeError("Không chuyển được robot sang MANUAL")
            self._sleep(0.3)

    def _command_lift(self, command, target_cm):
        cp = self._manual_checkpoint
        progress = cp['progress'] if cp else {}
        if self._resume_step and progress.get('target_cm', target_cm) != target_cm:
            raise RuntimeError('Lift target changed after STOP')
        progress['target_cm'] = target_cm
        timeout = float(self.get_parameter("lift_wait_s").value)
        elapsed = progress.get('elapsed_s', 0.)
        started = time.monotonic()
        after_sample = self._lift_t
        self._lift_active = True
        completed = False
        try:
            if (self._lift_cm is None or started - self._lift_t > 1.
                    or abs(self._lift_cm - target_cm) > 1.5):
                self.pub_lift.publish(String(data=command))
            while time.monotonic() - started + elapsed < timeout:
                self._check_abort()
                if (self._lift_cm is not None and self._lift_t > after_sample
                        and time.monotonic() - self._lift_t <= 1.0
                        and abs(self._lift_cm - target_cm) <= 1.5
                        and self._lift_still_since is not None
                        and time.monotonic() - self._lift_still_since >= float(self.get_parameter('lift_stable_s').value)):
                    completed = True
                    return True
                self._sleep(0.1)
            raise TimeoutError(f'Lift has not confirmed stable arrival at {target_cm:g} cm')
        finally:
            progress['elapsed_s'] = elapsed + time.monotonic() - started
            if not completed:
                self.pub_lift.publish(String(data="M0"))
            self._lift_active = False

    def _h_relative_move(self, args):
        self._require_cycle_idle()
        self._ensure_manual_idle()
        max_age = float(self.get_parameter("odom_max_age_s").value)
        if self._odom_pose is None or time.monotonic() - self._odom_t > max_age:
            raise RuntimeError("Odometry không có hoặc đã cũ; từ chối chạy theo mét")
        nav_id, nav_state = self._nav_latest()
        if nav_id is not None and nav_state in (1, 2, 3):
            raise RuntimeError("Đang có goal Nav2; từ chối chồng lệnh vận tốc")
        cp = self._manual_checkpoint
        progress = cp['progress'] if cp else {}
        if not self._resume_step or 'origin' not in progress:
            progress['origin'] = self._odom_pose
        x0, y0, yaw0 = progress['origin']
        sign = 1.0 if args["direction"] == "forward" else -1.0
        distance, speed = float(args["distance_m"]), float(args["speed_mps"])
        timeout = float(self.get_parameter("relative_move_timeout_s").value)
        elapsed = progress.get('elapsed_s', 0.)
        started = time.monotonic()
        self._motion_active = True
        try:
            while time.monotonic() - started + elapsed < timeout:
                self._check_abort()
                if self._odom_pose is None or time.monotonic() - self._odom_t > max_age:
                    raise RuntimeError("Odometry mất/cũ khi robot đang di chuyển")
                x, y, yaw = self._odom_pose
                lateral = -(x - x0) * math.sin(yaw0) + (y - y0) * math.cos(yaw0)
                yaw_error = abs(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))
                if abs(lateral) > .03 or yaw_error > math.radians(4.):
                    raise RuntimeError('Short-distance motion drift exceeded its saved direction')
                travelled = ((x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)) * sign
                remaining = distance - travelled
                if remaining <= 0.015:
                    self._location = "unknown"
                    return True
                twist = Twist()
                twist.linear.x = sign * min(speed, max(0.035, remaining * 0.8))
                self.pub_cmd.publish(twist)
                self._sleep(0.05)
            raise TimeoutError(f"Không hoàn tất quãng đường {distance:g} m trong {timeout:g}s")
        finally:
            progress['elapsed_s'] = elapsed + time.monotonic() - started
            self.pub_cmd.publish(Twist())
            self._motion_active = False


def main(args=None):
    rclpy.init(args=args)
    node = LLMCommandNode()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        ex.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
