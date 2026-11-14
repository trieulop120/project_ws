#!/usr/bin/env python3
"""
llm_command_node - ra lệnh cho AMR bằng tiếng Việt / tiếng Anh.

  /llm/command (String) -> LLM (Gemini/Groq/Ollama) -> kế hoạch JSON
      -> kiểm tra whitelist + vị trí -> /llm/plan -> (xác nhận /llm/confirm)
      -> gọi service/action sẵn có từng bước -> /llm/status
"""
import json
import math
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
from rcl_interfaces.srv import SetParameters
from sensor_msgs.msg import JointState
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
            mission_from_to,
            validate_plan,
        )


class LLMCommandNode(Node):
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
        d("lift_wait_s", 20.0)
        d("odom_topic", "/odometry/filtered")
        d("odom_max_age_s", 0.5)
        d("relative_move_timeout_s", 90.0)
        d("lift_joint_name", "encoder_lift_joint")
        d("joint_states_topic", "/joint_states")

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
        self._location = configured_location if configured_location in ("home", "p1", "p2", "d1") else "unknown"
        self._op_mode = "unknown"
        self._nav_list = []
        self._pallet_state = None
        self._pallet_state_t = 0.0
        self._pallet_status = None
        self._lift_active = False
        self._motion_active = False
        self._odom_pose = None
        self._odom_t = 0.0
        self._lift_cm = None
        self._lift_t = 0.0
        self._clarify_ctx = None
        self._sup_state = None
        self._sup_state_t = 0.0
        self._sup_status = None
        self._sup_status_revision = 0
        self._sup_status_signature = None

        g = self._cb
        self.cli_mission = {
            m: self.create_client(Trigger, f"mission_manager/trigger_{m}_ack",
                                  callback_group=g) for m in MISSIONS}
        p = lambda k: self.get_parameter(k).value  # noqa: E731
        self.cli_detect = self.create_client(SetBool, p("srv_detect"), callback_group=g)
        self.cli_go = self.create_client(Trigger, p("srv_go_to_pallet"), callback_group=g)
        self.cli_pstop = self.create_client(Trigger, p("srv_pallet_stop"), callback_group=g)
        self.cli_drop = self.create_client(Trigger, p("srv_drop"), callback_group=g)
        self.cli_set_param = self.create_client(
            SetParameters, "pallet_mission/set_parameters", callback_group=g)
        self.cli_cancel = self.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal", callback_group=g)
        self.cli_mstop = self.create_client(Trigger, "/mission/stop", callback_group=g)
        self.cli_cycle_start = self.create_client(Trigger, "/mission/start", callback_group=g)
        self.cli_cycle_params = self.create_client(
            SetParameters, "/mission_supervisor/set_parameters", callback_group=g)
        self.cli_set_mode = self.create_client(SetBool, "/llm/set_op_mode", callback_group=g)
        self.cli_quick_goal = {
            target: self.create_client(Trigger, f"/llm/quick_goal/{target.lower()}", callback_group=g)
            for target in ("HOME", "P_1", "P_2", "D_1")}
        self.cli_node_check = {
            target: self.create_client(Trigger, f"/llm/check_node/{target.lower()}", callback_group=g)
            for target in ("HOME", "P_1", "P_2", "D_1")}

        self.pub_plan = self.create_publisher(String, "/llm/plan", 10)
        self.pub_status = self.create_publisher(String, "/llm/status", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel_llm", 10)
        self.pub_lift = self.create_publisher(String, p("lift_topic"), 10)

        self.create_subscription(String, "/llm/command", self._on_command, 10, callback_group=g)
        self.create_subscription(Bool, "/llm/confirm", self._on_confirm, 10, callback_group=g)
        self.create_subscription(String, "/llm/set_location", self._on_set_loc, 10, callback_group=g)
        self.create_subscription(String, "/llm/op_mode", self._on_op_mode, 10, callback_group=g)
        self.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status",
                                 self._on_nav_status, 10, callback_group=g)
        self.create_subscription(Odometry, p("odom_topic"), self._on_odom, 10,
                                 callback_group=g)
        self.create_subscription(JointState, p("joint_states_topic"), self._on_joint_states,
                                 10, callback_group=g)
        self.create_subscription(String, p("pallet_status_topic"),
                                 self._on_pallet_status, 10, callback_group=g)
        self.create_subscription(String, p("supervisor_status_topic"),
                                 self._on_supervisor_status, 10, callback_group=g)
        self.create_service(Trigger, "llm/stop", self._srv_stop, callback_group=g)

        self._status("idle", msg="ready" if self._is_en() else "sẵn sàng")
        if p("dry_run"):
            self.get_logger().warning("*** DRY RUN ***")
        self.get_logger().info("llm_command_node sẵn sàng")

    def _is_en(self, language=None):
        return (language or self._current_lang) == "en"

    def _status(self, state, **kw):
        lang = kw.pop("language", self._current_lang)
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
        if not client.wait_for_service(timeout_sec=2.0):
            msg = (f"Service not available: {client.srv_name}" if self._is_en()
                   else f"Service không sẵn sàng: {client.srv_name}")
            raise RuntimeError(msg)
        ev, box = threading.Event(), {}

        def _done(fut):
            try:
                box["r"] = fut.result()
            except Exception as e:  # noqa: BLE001
                box["e"] = e
            ev.set()

        client.call_async(req).add_done_callback(_done)
        if not ev.wait(timeout):
            msg = (f"Service timed out: {client.srv_name}" if self._is_en()
                   else f"Service quá thời gian: {client.srv_name}")
            raise TimeoutError(msg)
        if "e" in box:
            raise box["e"]
        return box["r"]

    def _on_confirm(self, msg):
        self._confirm_val = bool(msg.data)
        self._confirm_evt.set()

    def _on_set_loc(self, msg):
        candidate = msg.data.strip().lower()
        self._location = candidate if candidate in ("home", "p1", "p2", "d1") else "unknown"
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

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        self._odom_pose = (float(p.x), float(p.y), yaw_from_quat(msg.pose.pose.orientation))
        self._odom_t = time.monotonic()

    def _on_joint_states(self, msg):
        try:
            i = msg.name.index(self.get_parameter("lift_joint_name").value)
            self._lift_cm = float(msg.position[i]) * 100.0
            self._lift_t = time.monotonic()
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

    def _on_supervisor_status(self, msg):
        try:
            data = json.loads(msg.data)
            st = data.get(self.get_parameter("pallet_state_key").value)
            cycle = data.get("cycle") if isinstance(data.get("cycle"), dict) else {}
            signature = (st, data.get("msg"), data.get("active_mission"),
                         data.get("active_dest"), cycle.get("number"),
                         data.get("resume_state"))
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
        speech_confidence = None
        raw = msg.data.strip()
        try:
            envelope = json.loads(raw)
            if isinstance(envelope, dict) and "text" in envelope:
                raw = str(envelope.get("text", ""))
                language = str(envelope.get("language", "auto")).lower()
                confidence = envelope.get("speech_confidence")
                if isinstance(confidence, (int, float)) and 0.0 <= float(confidence) <= 1.0:
                    speech_confidence = float(confidence)
            elif isinstance(envelope, dict) and envelope.get("internal_abort") is True:
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
        else:
            self._current_lang = language
        en = self._is_en(language)
        text = raw.strip()
        if not text:
            return
        if is_stop(text):
            self._emergency_stop("stop command" if en else "lệnh dừng")
            return
        if speech_confidence is not None and speech_confidence < 0.55:
            prompt = ("I didn't hear that clearly. Please repeat the command."
                      if en else "Mình nghe chưa rõ. Bạn vui lòng nhắc lại lệnh.")
            self._status("clarify", msg=prompt, low_confidence=True, language=self._current_lang)
            return
        if not self._busy.acquire(blocking=False):
            busy_msg = ("Executing another command. Say 'stop' to cancel."
                        if en else "Đang thực hiện lệnh khác. Nói 'dừng' để hủy.")
            self._status("busy", msg=busy_msg, language=self._current_lang)
            return

        self._abort.clear()
        self._confirm_evt.clear()
        self._confirm_val = False
        threading.Thread(target=self._worker, args=(text, self._current_lang), daemon=True).start()

    def _srv_stop(self, _req, resp):
        self._emergency_stop("service llm/stop")
        resp.success = True
        resp.message = "stopped" if self._is_en() else "đã dừng"
        return resp

    def _emergency_stop(self, why):
        self.get_logger().warn(f"DỪNG KHẨN: {why}")
        self._abort.set()
        self._clarify_ctx = None
        state = self._sup_state
        age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
        busy_states = {s.upper() for s in self.get_parameter("supervisor_busy_states").value}
        cycle_active = state in busy_states and age <= self.get_parameter("supervisor_status_max_age_s").value

        if cycle_active:
            if self.cli_mstop.service_is_ready():
                self.cli_mstop.call_async(Trigger.Request())
            else:
                self.get_logger().error("Cycle đang chạy nhưng /mission/stop chưa sẵn sàng")
        else:
            nav_id, nav_state = self._nav_latest()
            nav_active = nav_id is not None and nav_state in (1, 2, 6)
            pallet_active = self._pallet_state not in (None, "IDLE", "DONE", "CARRYING", "FAILED", "STOPPED")
            if pallet_active and self.cli_pstop.service_is_ready():
                self.cli_pstop.call_async(Trigger.Request())
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

    def _worker(self, text, language="auto"):
        en = self._is_en(language)
        try:
            self._status("planning", msg=text, language=language)
            ctx, self._clarify_ctx = self._clarify_ctx, None
            if ctx and time.time() - ctx[2] < 90:
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
            plan = self._plan(request, language)
            self.get_logger().info(f"LLM trả lời sau {time.time() - t0:.2f}s: {plan}")
            if plan.get("clarify"):
                self._clarify_ctx = (base, plan["clarify"], time.time())
                self._status("clarify", msg=plan["clarify"], language=language)
                return
            p = self.get_parameter
            allow_unmeasured = (p("allow_unmeasured_lift").value or p("dry_run").value)
            steps = validate_plan(plan, self._location, p("max_steps").value,
                                  allow_unmeasured_lift=allow_unmeasured)
            if not steps:
                default_noop = "No action taken." if en else "Không có hành động nào."
                self._status("noop", msg=plan.get("say") or default_noop, language=language)
                return
            summary = self._action_summary(steps, language, self._location)
            self._confirm_evt.clear()
            self._confirm_val = False
            self.pub_plan.publish(String(data=json.dumps(
                {"say": plan.get("say", ""), "steps": steps,
                 "action_summary": summary, "language": language}, ensure_ascii=False)))
            if not p("require_confirm").value:
                self.get_logger().warning("require_confirm=false bị bỏ qua")
            self._status("waiting_confirm", msg="review_required", say=plan.get("say", ""),
                         steps=steps, action_summary=summary, confirm_required=True,
                         language=language)
            if not self._wait_confirm():
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
            self._run_plan(steps, language)
        except Aborted:
            self._status("stopped", msg="stopped" if en else "đã dừng", language=language)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Lỗi: {e}")
            self._status("error", msg=str(e), language=language)
        finally:
            self._busy.release()

    def _wait_confirm(self):
        self._check_abort()
        timeout = self.get_parameter("confirm_timeout_s").value
        self._confirm_evt.wait(timeout)
        self._check_abort()
        return self._confirm_val

    def _run_plan(self, steps, language="auto"):
        en = self._is_en(language)
        handlers = {"set_mode": self._h_set_mode,
                    "quick_goal": self._h_quick_goal,
                    "start_auto_cycle": self._h_start_auto_cycle,
                    "start_single_cycle": self._h_start_single_cycle,
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
        for i, st in enumerate(steps, 1):
            self._check_abort()
            self._status("running", step=i, total=len(steps),
                         msg=f"{st['action']} {st['args']}", language=language)
            if dry:
                self.get_logger().info(f"[dry_run] {st}")
                self._sleep(0.5)
                if st["action"] == "go_mission":
                    self._location = mission_from_to(st["args"]["name"])[1]
                continue
            if not handlers[st["action"]](st["args"]):
                default_fail = (f"Step {i} failed: {st['action']} {st['args']}" if en
                                else f"Bước {i} thất bại: {st['action']} {st['args']}")
                self._status("failed", step=i, total=len(steps),
                             msg=self._failure_message or default_fail, language=language)
                return
        if dry:
            msg = "Preview complete" if en else "Xem trước hoàn tất"
            self._status("simulated", msg=msg, language=language)
        elif any(s["action"] in ("start_auto_cycle", "start_single_cycle") for s in steps):
            msg = "Cycle started" if en else "Đã khởi động chu trình"
            self._status("started", msg=msg, language=language)
        else:
            self._status("done", msg=self._success_message or ("Completed" if en else "Hoàn thành"),
                         language=language)

    @staticmethod
    def _action_summary(steps, language, initial_location):
        en = language == "en"
        loc = {"home": "HOME", "p1": "P_1", "p2": "P_2", "d1": "D_1"}
        current = initial_location
        summary = []
        for step in steps:
            action, args = step["action"], step["args"]
            if action == "set_mode":
                summary.append(f"Set mode to {args['mode']}" if en else f"Chuyển mode sang {args['mode']}")
            elif action == "quick_goal":
                summary.append(f"Navigate to {args['target']}" if en else f"Đi tới quick goal {args['target']}")
                current = {"HOME": "home", "P_1": "p1", "P_2": "p2", "D_1": "d1"}[args["target"]]
            elif action == "go_mission":
                source, dest = mission_from_to(args["name"])
                route = f"{loc.get(source, 'anywhere')} → {loc[dest]}"
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
                return extract_json(raw)
            except Exception as e:  # noqa: BLE001
                elapsed = time.monotonic() - started
                model = cfg.get("model", "unknown") if cfg else "unknown"
                kind, detail = self._backend_failure(e)
                self.get_logger().warning(f"LLM backend={name} model={model} result={kind} elapsed={elapsed:.2f}s detail={detail}")
                failures.append((name, kind, detail))
        raise RuntimeError(self._user_backend_failure(failures, language))

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

    def _mission_context(self):
        age = time.time() - self._sup_state_t if self._sup_state_t else None
        nav_id, nav_state = self._nav_latest()
        raw = self._sup_status if isinstance(self._sup_status, dict) else {}
        cycle = raw.get("cycle") if isinstance(raw.get("cycle"), dict) else {}
        dest = raw.get("active_dest") if raw.get("active_dest") in ("NODE_HOME", "P_1", "P_2", "D_1") else None
        mission = raw.get("active_mission") if raw.get("active_mission") in MISSIONS else None
        run_mode = cycle.get("run_mode") if cycle.get("run_mode") in ("auto", "single") else "unknown"
        number = cycle.get("number") if isinstance(cycle.get("number"), int) else None
        
        # Mặc định cargo = False khi None để LLM nhận đúng xe rỗng
        cargo_raw = raw.get("cargo")
        cargo = cargo_raw if isinstance(cargo_raw, bool) else False

        lift = raw.get("pm_lift_cm")
        if not isinstance(lift, (int, float)) or not math.isfinite(float(lift)):
            lift = None
        else:
            lift = round(float(lift), 1) if -1.0 <= float(lift) <= LIFT_LIMIT_CM + 1.0 else None
        
        raw_pallet_state = self._pallet_state
        pallet_states = {"IDLE", "DETECTING", "READY", "NAVIGATING", "SETTLING",
                         "ALIGNING", "INSERTING", "LIFTING", "BACKING", "HOMING",
                         "LOWERING", "CARRYING", "DONE", "FAILED", "STOPPED"}
        pallet_state = raw_pallet_state if raw_pallet_state in pallet_states else "unknown"
        return {
            "supervisor_state": self._sup_state or "IDLE",
            "supervisor_status_age_s": round(age, 1) if age is not None else None,
            "active_mission": mission,
            "active_destination": dest,
            "cycle": {"run_mode": run_mode, "number": number},
            "cargo": cargo,
            "pallet_state": pallet_state,
            "pallet_status_age_s": (round(time.time() - self._pallet_state_t, 1) if self._pallet_state_t else None),
            "robot_stopped": raw.get("robot_stopped") if isinstance(raw.get("robot_stopped"), bool) else True,
            "nav_status_fresh": raw.get("nav_status_fresh") is True,
            "supervisor_nav_goal_active": raw.get("nav_goal_active") is True,
            "lift_cm": lift,
            "operating_mode": self._op_mode,
            "nav_goal_active": nav_id is not None and nav_state in (1, 2, 6),
            "nav_goal_status": nav_state,
        }

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
            self._location = {"HOME": "home", "P_1": "p1", "P_2": "p2", "D_1": "d1"}[target]
        else:
            self._location = "unknown"
        return ok

    def _start_cycle(self, run_mode, cargo_class="any", quantity=1, drop_slot="auto"):
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
            result = self._call(self.cli_cycle_start, Trigger.Request(), timeout=5.0)
            if not result.success:
                return fail(f"Supervisor rejected /mission/start: {result.message}" if en
                            else f"Supervisor từ chối /mission/start: {result.message}")

            deadline = time.monotonic() + 10.0
            goto_seen_at = None
            while time.monotonic() < deadline:
                self._check_abort()
                age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
                state = self._sup_state
                is_new_status = self._sup_status_revision > start_status_revision
                if (is_new_status and
                        age <= self.get_parameter("supervisor_status_max_age_s").value):
                    if state in ("RECOVERY", "FAILED"):
                        return fail(f"Cycle entered {state}" if en else f"Cycle chuyển sang {state}")
                    if state == "GOTO_PICK":
                        goto_seen_at = goto_seen_at or time.monotonic()
                        if time.monotonic() - goto_seen_at >= 0.8:
                            self._failure_message = ""
                            return True
                    elif (state not in (None, "IDLE") and (self._sup_status or {}).get("nav_goal_active") is True):
                        self._failure_message = ""
                        return True
                    else:
                        goto_seen_at = None
                time.sleep(0.1)

            return True

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
        if self.get_parameter("dry_run").value:
            return True
        if self._op_mode != "AUTO":
            if not self._h_set_mode({"mode": "AUTO"}):
                return False
            time.sleep(0.3)
        client = self.create_client(Trigger, "/mission/resume", callback_group=self._cb)
        result = self._call(client, Trigger.Request(), timeout=5.0)
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
        if source is not None:
            self._check_node_pose({"home": "HOME", "p1": "P_1", "p2": "P_2", "d1": "D_1"}[source])
        base_id, _ = self._nav_latest()
        response = self._call(self.cli_mission[name], Trigger.Request(), timeout=15.0)
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

        self._pallet_state = None
        r = self._call(self.cli_go, Trigger.Request())
        if not r.success:
            self.get_logger().error(f"go_to_pallet từ chối: {r.message}")
            return False
        return self._wait_pallet_done()

    def _h_drop_pallet(self, _a):
        if self._op_mode != "MANUAL":
            self._h_set_mode({"mode": "MANUAL"})
            time.sleep(0.3)
        self._pallet_state = None
        r = self._call(self.cli_drop, Trigger.Request())
        if not r.success:
            self.get_logger().error(f"drop (pallet_mission/drop) từ chối: {r.message}")
            return False
        return self._wait_pallet_done()

    def _wait_pallet_done(self):
        p = self.get_parameter
        done = {s.upper() for s in p("pallet_done").value}
        fail = {s.upper() for s in p("pallet_fail").value}
        idle = {s.upper() for s in p("pallet_idle").value}
        t0, seen_active = time.time(), False
        while time.time() - t0 < p("pallet_timeout_s").value:
            self._check_abort()
            st = self._pallet_state
            if st and st not in done | fail | idle:
                seen_active = True
            if st in fail:
                return False
            if st in done and (seen_active or time.time() - t0 > 5.0):
                return True
            time.sleep(0.2)
        return False

    def _h_lift_to(self, a):
        self._require_cycle_idle(allow_cargo=True)
        status = self._fresh_supervisor_status()
        cm = (float(a["height_cm"]) if "height_cm" in a
              else LIFT_LEVELS[a["level"]])
        if status.get("cargo") is True:
            if self._lift_cm is None or time.monotonic() - self._lift_t > 1.0:
                raise RuntimeError("Không có số đo lift mới; từ chối thay đổi độ cao khi đang mang pallet")
            if cm < self._lift_cm - 1.5:
                raise RuntimeError("Robot đang mang pallet; không hạ lift thấp hơn vị trí hiện tại")
        self._ensure_manual_idle()
        command = "HOME" if cm <= 0.0 else f"H{cm:g}"
        return self._command_lift(command, cm)

    def _h_lift_home(self, _args):
        status = self._fresh_supervisor_status()
        age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
        if age > self.get_parameter("supervisor_status_max_age_s").value or not isinstance(
                status.get("cargo"), bool):
            raise RuntimeError("Không có trạng thái tải pallet mới; từ chối hạ lift về HOME")
        self._require_cycle_idle()
        if status["cargo"]:
            raise RuntimeError("Robot đang mang pallet; không hạ lift về HOME vì có thể làm rơi hàng")
        self._ensure_manual_idle()
        return self._command_lift("HOME", 0.0)

    def _ensure_manual_idle(self):
        age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
        if age > self.get_parameter("supervisor_status_max_age_s").value:
            raise RuntimeError("Không có trạng thái supervisor mới; từ chối thao tác MANUAL")
        nav_id, nav_state = self._nav_latest()
        if nav_id is not None and nav_state in (1, 2, 6):
            raise RuntimeError("Đang có goal Nav2; hãy kết thúc hoặc hủy goal trước")
        if self._op_mode != "MANUAL":
            if not self._h_set_mode({"mode": "MANUAL"}):
                raise RuntimeError("Không chuyển được robot sang MANUAL")
            self._sleep(0.3)

    def _command_lift(self, command, target_cm):
        timeout = float(self.get_parameter("lift_wait_s").value)
        self._lift_active = True
        self.pub_lift.publish(String(data=command))
        deadline = time.monotonic() + timeout
        completed = False
        try:
            while time.monotonic() < deadline:
                self._check_abort()
                if (self._lift_cm is not None and
                        time.monotonic() - self._lift_t <= 1.0 and
                        abs(self._lift_cm - target_cm) <= 1.5):
                    completed = True
                    return True
                self._sleep(0.1)
            raise TimeoutError("Không nhận được xác nhận joint_states lift đã tới "
                               f"{target_cm:g} cm trong {timeout:g}s")
        finally:
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
        if nav_id is not None and nav_state in (1, 2, 6):
            raise RuntimeError("Đang có goal Nav2; từ chối chồng lệnh vận tốc")
        x0, y0, yaw0 = self._odom_pose
        sign = 1.0 if args["direction"] == "forward" else -1.0
        distance, speed = float(args["distance_m"]), float(args["speed_mps"])
        timeout = float(self.get_parameter("relative_move_timeout_s").value)
        deadline = time.monotonic() + timeout
        self._motion_active = True
        try:
            while time.monotonic() < deadline:
                self._check_abort()
                if self._odom_pose is None or time.monotonic() - self._odom_t > max_age:
                    raise RuntimeError("Odometry mất/cũ khi robot đang di chuyển")
                x, y, _ = self._odom_pose
                travelled = ((x - x0) * math.cos(yaw0) + (y - y0) * math.sin(yaw0)) * sign
                remaining = distance - travelled
                if remaining <= 0.015:
                    return True
                twist = Twist()
                twist.linear.x = sign * min(speed, max(0.035, remaining * 0.8))
                self.pub_cmd.publish(twist)
                self._sleep(0.05)
            raise TimeoutError(f"Không hoàn tất quãng đường {distance:g} m trong {timeout:g}s")
        finally:
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
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
