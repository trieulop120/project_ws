#!/usr/bin/env python3
"""
llm_command_node - ra lệnh cho AMR bằng tiếng Việt.

  /llm/command (String) -> LLM (Gemini/Groq/Ollama) -> kế hoạch JSON
      -> kiểm tra whitelist + vị trí -> /llm/plan -> (xác nhận /llm/confirm)
      -> gọi service/action sẵn có từng bước -> /llm/status

Nguyên tắc an toàn:
  * LLM chỉ được CHỌN hành động trong bảng TOOLS, không lái robot trực tiếp.
  * Lệnh "dừng" KHÔNG đi qua LLM (xử lý ngay, không có độ trễ mạng).
  * Kế hoạch phải qua validate_plan() mới được chạy.
  * Đọc mission_supervisor + pallet + Nav2 status; chỉ từ chối các thao tác
    xung đột. Stop/đổi Manual dùng luồng của manager đang sở hữu mission.

Logic thuần (bảng cấu hình + validate) nằm ở module llm_logic.py để test
được không cần ROS.
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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, SetBool, Trigger

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

# ============================================================================
# 3. NODE ROS 2
# ============================================================================
class LLMCommandNode(Node):
    def __init__(self):
        super().__init__("llm_command_node")
        d = self.declare_parameter
        d("backends", ["gemini", "groq"])      # thứ tự thử, lỗi thì sang cái sau
        d("llm_timeout_s", 25.0)
        d("dry_run", False)                    # True: chỉ duyệt kế hoạch, không điều khiển
        d("require_confirm", True)
        d("confirm_timeout_s", 30.0)
        d("start_location", "home")
        d("max_steps", 8)
        d("mission_timeout_s", 180.0)
        d("nav_start_timeout_s", 15.0)
        # --- pick pallet (giả định luồng GUI hiện tại: detect ON -> OFF -> go) ---
        d("srv_detect", "pallet_mission/detect")
        d("srv_go_to_pallet", "pallet_mission/go_to_pallet")
        d("srv_pallet_stop", "pallet_mission/stop")
        d("srv_drop", "pallet_mission/drop")
        d("detect_first", True)
        d("detect_seconds", 4.0)
        d("pallet_timeout_s", 120.0)
        d("pallet_status_topic", "/pallet_mission/status")
        d("pallet_state_key", "state")
        d("pallet_done", ["DONE", "CARRYING"])  # 'home'->DONE, 'hold'->CARRYING
        d("pallet_fail", ["FAILED", "STOPPED"])
        d("pallet_idle", ["IDLE"])
        # Q2: trưoc pick, set after_pick='hold' de pallet_mission dung o CARRYING
        # (can cho drop_pallet). Tham so cua pallet_mission, khong sua file.
        d("set_after_pick_hold", True)
        # Chặn mức nâng rack_1/rack_2 (chưa đo thực tế) khi dry_run=false
        d("allow_unmeasured_lift", False)
        # --- chống 2 bộ điều khiển: mission_supervisor nếu đang bận ---
        d("supervisor_status_topic", "/mission_supervisor/status")
        d("supervisor_busy_states",
          ["GOTO_PICK", "PICKING", "GOTO_D", "DROPPING", "WAITING_SLOT",
           "RETURN_HOME", "RECOVERY"])
        d("supervisor_status_max_age_s", 5.0)
        # --- nâng hạ (GIỮ CHỖ: thay bằng interface mới của nhóm) ---
        d("lift_topic", "/esp32_cmd")          # raw "H<cm>" theo pallet_mission
        d("lift_wait_s", 6.0)

        self._cb = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = threading.Lock()
        self._abort = threading.Event()
        self._confirm_evt = threading.Event()
        self._confirm_val = False
        self._success_message = ""
        self._failure_message = ""
        configured_location = str(self.get_parameter("start_location").value).lower()
        self._location = configured_location if configured_location in ("home", "p1", "p2", "d1") else "unknown"
        self._op_mode = "unknown"
        self._nav_list = []
        self._pallet_state = None
        self._pallet_state_t = 0.0
        self._pallet_status = None
        self._lift_active = False
        self._clarify_ctx = None   # (yêu cầu gốc, câu hỏi lại, thời điểm)
        self._sup_state = None     # state mới nhất của mission_supervisor
        self._sup_state_t = 0.0    # mốc thời gian nhận status
        self._sup_status = None    # payload đầy đủ để preflight start_auto_cycle

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
        # /mission/stop = Trigger cua mission_supervisor (neu no dang chay)
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
        self.create_subscription(String, p("pallet_status_topic"),
                                 self._on_pallet_status, 10, callback_group=g)
        self.create_subscription(String, p("supervisor_status_topic"),
                                 self._on_supervisor_status, 10, callback_group=g)
        self.create_service(Trigger, "llm/stop", self._srv_stop, callback_group=g)

        self._status("idle", msg="sẵn sàng")
        if p("dry_run"):
            self.get_logger().warning("*** DRY RUN: robot không chạy; kế hoạch chỉ được mô phỏng ***")
        self.get_logger().info(
            f"llm_command_node sẵn sàng (dry_run={p('dry_run')}, backends={p('backends')})")

    # ------------------------------------------------------------ tiện ích
    def _status(self, state, **kw):
        self.pub_status.publish(String(data=json.dumps(
            {"state": state, "location": self._location, "op_mode": self._op_mode,
             "dry_run": bool(self.get_parameter("dry_run").value),
             "mission_state": self._sup_state or "unknown", **kw}, ensure_ascii=False)))

    def _check_abort(self):
        if self._abort.is_set():
            raise Aborted()

    def _sleep(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            self._check_abort()
            time.sleep(0.1)

    def _call(self, client, req, timeout=5.0):
        """Gọi service từ thread phụ (executor vẫn quay ở thread chính)."""
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError(f"Service không sẵn sàng: {client.srv_name}")
        ev, box = threading.Event(), {}

        def _done(fut):
            try:
                box["r"] = fut.result()
            except Exception as e:  # noqa: BLE001
                box["e"] = e
            ev.set()

        client.call_async(req).add_done_callback(_done)
        if not ev.wait(timeout):
            raise TimeoutError(f"Service quá thời gian: {client.srv_name}")
        if "e" in box:
            raise box["e"]
        return box["r"]

    # ------------------------------------------------------------ subscriber
    def _on_confirm(self, msg):
        self._confirm_val = bool(msg.data)
        self._confirm_evt.set()

    def _on_set_loc(self, msg):
        candidate = msg.data.strip().lower()
        self._location = candidate if candidate in ("home", "p1", "p2", "d1") else "unknown"
        self._status("idle", msg=f"đặt vị trí = {self._location}")

    def _on_op_mode(self, msg):
        mode = msg.data.strip().upper()
        if mode in ("AUTO", "MANUAL"):
            self._op_mode = mode

    def _on_nav_status(self, msg):
        with self._lock:
            self._nav_list = list(msg.status_list)

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
        """Trả message nếu mission_supervisor đang bận, None nếu chạy được.

        Topic không fresh (supervisor không chạy) -> bỏ qua êm, không lỗi."""
        st, t = self._sup_state, self._sup_state_t
        if st is None or t <= 0.0:
            return None
        if time.time() - t > self.get_parameter("supervisor_status_max_age_s").value:
            return None
        busy = {s.upper() for s in
                self.get_parameter("supervisor_busy_states").value}
        if st in busy:
            return (f"Mission supervisor đang bận ({st}). "
                    f"Dừng nó (ros2 service call /mission/stop std_srvs/srv/Trigger) "
                    f"rồi thử lại.")
        return None

    # ------------------------------------------------------------ lệnh vào
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
                    self._status("cancelled", msg="Kế hoạch LLM đã hủy")
                return
        except (ValueError, TypeError):
            pass
        if language not in ("en", "vi"):
            language = "auto"
        text = raw.strip()
        if not text:
            return
        if is_stop(text):
            self._emergency_stop("lệnh dừng")
            return
        if speech_confidence is not None and speech_confidence < 0.55:
            prompt = ("I didn't hear that clearly. Please repeat the command."
                      if language == "en" else "Mình nghe chưa rõ. Bạn vui lòng nhắc lại lệnh.")
            self._status("clarify", msg=prompt, low_confidence=True)
            return
        if not self._busy.acquire(blocking=False):
            self._status("busy", msg="Đang thực hiện lệnh khác. Nói 'dừng' để hủy.")
            return
        # Reset before the worker starts so a STOP arriving immediately after
        # this point cannot be erased by the worker's startup sequence.
        self._abort.clear()
        self._confirm_evt.clear()
        self._confirm_val = False
        threading.Thread(target=self._worker, args=(text, language), daemon=True).start()

    def _srv_stop(self, _req, resp):
        self._emergency_stop("service llm/stop")
        resp.success, resp.message = True, "đã dừng"
        return resp

    def _emergency_stop(self, why):
        self.get_logger().warn(f"DỪNG KHẨN: {why}")
        self._abort.set()
        self._clarify_ctx = None
        state = self._sup_state
        age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
        busy_states = {s.upper() for s in self.get_parameter("supervisor_busy_states").value}
        cycle_active = state in busy_states and age <= self.get_parameter(
            "supervisor_status_max_age_s").value
        # Let the active state machine stop its own work. The supervisor stop
        # service cancels its Nav2 goal and pallet action while preserving the
        # RECOVERY checkpoint when cargo is aboard.
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
            # If status is stale, best effort stop each available manager.
            if age > self.get_parameter("supervisor_status_max_age_s").value and self.cli_mstop.service_is_ready():
                self.cli_mstop.call_async(Trigger.Request())
        if self._lift_active:
            # M0 is the lift driver's explicit stop command; never send a
            # zero position because that topic interprets values as targets.
            self.pub_lift.publish(String(data="M0"))
            self._lift_active = False
        self._confirm_evt.set()
        self._status("stopped", msg=f"{why} (mission={state or 'unknown'})")

    # ------------------------------------------------------------ luồng chính
    def _worker(self, text, language="auto"):
        try:
            self._status("planning", msg=text)
            # Nhớ ngữ cảnh khi robot vừa hỏi lại, để câu trả lời ngắn ("P1") vẫn hiểu được
            ctx, self._clarify_ctx = self._clarify_ctx, None
            if ctx and time.time() - ctx[2] < 90:
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
                self._status("clarify", msg=plan["clarify"])
                return
            p = self.get_parameter
            # dry_run cho phép xem trước kế hoạch rack_* (chỉ in, không chạy)
            allow_unmeasured = (p("allow_unmeasured_lift").value
                                or p("dry_run").value)
            steps = validate_plan(plan, self._location, p("max_steps").value,
                                  allow_unmeasured_lift=allow_unmeasured)
            if not steps:
                self._status("noop", msg=plan.get("say", "Không có hành động nào."))
                return
            summary = self._action_summary(steps, language, self._location)
            # Clear any stale confirmation before publishing this plan. STOP
            # sets the event and abort flag; _wait_confirm must preserve it.
            self._confirm_evt.clear()
            self._confirm_val = False
            self.pub_plan.publish(String(data=json.dumps(
                {"say": plan.get("say", ""), "steps": steps,
                 "action_summary": summary}, ensure_ascii=False)))
            if not p("require_confirm").value:
                self.get_logger().warning(
                    "require_confirm=false bị bỏ qua: mọi lệnh có thể tác động robot luôn cần xác nhận")
            self._status("waiting_confirm", msg="review_required", say=plan.get("say", ""),
                         steps=steps, action_summary=summary, confirm_required=True)
            if not self._wait_confirm():
                self._status("cancelled", msg="Chưa được xác nhận")
                return
            # A plan can wait for confirmation while the robot state changes.
            # Revalidate route assumptions now; each handler also checks its
            # live state immediately before sending a command.
            refreshed = validate_plan(
                {"steps": steps}, self._location, p("max_steps").value,
                allow_unmeasured_lift=allow_unmeasured)
            if refreshed != steps:
                self._status("stale_plan", msg="Trạng thái robot thay đổi; hãy gửi lại lệnh.")
                return
            self._run_plan(steps, language)
        except Aborted:
            self._status("stopped", msg="đã dừng")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Lỗi: {e}")
            self._status("error", msg=str(e))
        finally:
            self._busy.release()

    def _wait_confirm(self):
        self._check_abort()
        timeout = self.get_parameter("confirm_timeout_s").value
        self._confirm_evt.wait(timeout)
        self._check_abort()
        return self._confirm_val

    def _run_plan(self, steps, language="auto"):
        handlers = {"set_mode": self._h_set_mode,
                    "quick_goal": self._h_quick_goal,
                    "start_auto_cycle": self._h_start_auto_cycle,
                    "start_single_cycle": self._h_start_single_cycle,
                    "stop_cycle": self._h_stop_cycle,
                    "cancel_navigation": self._h_cancel_navigation,
                    "resume_cycle": self._h_resume_cycle,
                    "go_mission": self._h_go_mission,
                    "pick_pallet": self._h_pick_pallet,
                    "drop_pallet": self._h_drop_pallet,
                    "lift_to": self._h_lift_to}
        dry = self.get_parameter("dry_run").value
        self._success_message = ""
        self._failure_message = ""
        for i, st in enumerate(steps, 1):
            self._check_abort()
            self._status("running", step=i, total=len(steps),
                         msg=f"{st['action']} {st['args']}")
            if dry:
                self.get_logger().info(f"[dry_run] {st}")
                self._sleep(0.5)
                if st["action"] == "go_mission":
                    self._location = mission_from_to(st["args"]["name"])[1]
                continue
            if not handlers[st["action"]](st["args"]):
                self._status("failed", step=i, total=len(steps),
                             msg=self._failure_message or
                             f"Bước {i} thất bại: {st['action']} {st['args']}")
                return
        if dry:
            msg = ("Xem trước hoàn tất; robot không nhận lệnh điều khiển" if language != "en"
                   else "Preview complete; no robot commands were sent")
            self._status("simulated", msg=msg)
        elif any(s["action"] in ("start_auto_cycle", "start_single_cycle") for s in steps):
            msg = "Đã khởi động chu trình" if language != "en" else "Cycle started"
            self._status("started", msg=msg)
        else:
            self._status("done", msg=self._success_message or
                         ("Hoàn thành" if language != "en" else "Completed"))

    @staticmethod
    def _action_summary(steps, language, initial_location):
        """Deterministic review text derived only from validated action args."""
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
            elif action == "drop_pallet":
                summary.append("Place pallet at D_1 (compatible slot auto-selected at execution)"
                               if en else "Đặt pallet tại D_1 (mission manager tự chọn ô phù hợp khi chạy)")
            elif action == "lift_to":
                cm = LIFT_LEVELS[args["level"]]
                summary.append(f"Move lift to {cm:g} cm" if en else f"Nâng/hạ càng tới {cm:g} cm")
            elif action == "start_auto_cycle":
                summary.append("Start automatic pallet cycle" if en else "Chạy chu trình pallet Auto")
            elif action == "start_single_cycle":
                summary.append("Run one pallet cycle" if en else "Chạy một chu trình pallet")
            elif action == "stop_cycle":
                summary.append("Stop pallet cycle" if en else "Dừng chu trình pallet")
            elif action == "resume_cycle":
                summary.append("Resume pallet cycle from recovery" if en else "Tiếp tục cycle từ RECOVERY")
            elif action == "cancel_navigation":
                summary.append("Cancel active Nav2 goal" if en else "Hủy goal Nav2 đang chạy")
        return summary

    # ------------------------------------------------------------ LLM
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
                self.get_logger().warning(
                    f"LLM backend={name} model={model} result={kind} "
                    f"elapsed={elapsed:.2f}s detail={detail}")
                failures.append((name, kind, detail))
        raise RuntimeError(self._user_backend_failure(failures))

    @staticmethod
    def _backend_failure(exc):
        if isinstance(exc, requests.exceptions.Timeout):
            return "timeout", "request timed out"
        if isinstance(exc, requests.exceptions.ConnectionError):
            return "connection_error", "connection failed"
        if isinstance(exc, requests.exceptions.HTTPError):
            response = exc.response
            code = response.status_code if response is not None else 0
            category = "http_4xx" if 400 <= code < 500 else (
                "http_5xx" if 500 <= code < 600 else "http_error")
            return category, f"HTTP {code}"
        message = str(exc)
        if "Thiếu biến môi trường" in message:
            return "missing_key", message
        if ("invalid" in message.lower() or "json" in message.lower()
                or isinstance(exc, json.JSONDecodeError)):
            return "invalid_response", "response was not valid planner JSON"
        return "error", type(exc).__name__

    @staticmethod
    def _user_backend_failure(failures):
        if not failures:
            return "Không có LLM backend nào được cấu hình."
        labels = {
            "timeout": "hết thời gian chờ",
            "connection_error": "không kết nối được",
            "missing_key": "thiếu API key",
            "http_4xx": "API từ chối yêu cầu (kiểm tra key/quyền/model)",
            "http_5xx": "dịch vụ API đang lỗi",
            "http_error": "API trả lỗi HTTP",
            "invalid_response": "trả về kế hoạch không hợp lệ",
            "error": "gặp lỗi",
        }
        causes = "; ".join(
            f"{name}: {labels.get(kind, 'gặp lỗi')}"
            + (f" {detail}" if kind.startswith("http_") else "")
            for name, kind, detail in failures)
        return f"Không lấy được kế hoạch từ LLM ({causes})."

    def _post_backend(self, name, cfg, url, body, headers=None):
        timeout = (5.0, float(self.get_parameter("llm_timeout_s").value))
        for attempt in range(2):
            started = time.monotonic()
            try:
                response = requests.post(
                    url, json=body, headers=headers or {}, timeout=timeout)
                elapsed = time.monotonic() - started
                if response.status_code >= 400:
                    category = "4xx" if response.status_code < 500 else "5xx"
                    self.get_logger().error(
                        f"LLM backend={name} model={cfg['model']} HTTP="
                        f"{response.status_code} ({category}) elapsed={elapsed:.2f}s")
                    response.raise_for_status()
                self.get_logger().info(
                    f"LLM backend={name} model={cfg['model']} HTTP="
                    f"{response.status_code} elapsed={elapsed:.2f}s")
                return response
            except (requests.exceptions.Timeout,
                    requests.exceptions.ConnectionError) as exc:
                elapsed = time.monotonic() - started
                category = "timeout" if isinstance(exc, requests.exceptions.Timeout) else "connection_error"
                self.get_logger().warning(
                    f"LLM backend={name} model={cfg['model']} {category} "
                    f"attempt={attempt + 1}/2 elapsed={elapsed:.2f}s")
                if attempt == 1:
                    raise
                time.sleep(0.5)

    def _call_gemini(self, cfg, system, text):
        key = os.environ.get(cfg["key_env"], "")
        if not key:
            raise RuntimeError(f"Thiếu biến môi trường {cfg['key_env']}")
        if not key.isascii():
            raise RuntimeError("GEMINI_API_KEY chứa ký tự không phải ASCII, "
                               "có vẻ chưa dán khóa thật")
        url = ("https://generativelanguage.googleapis.com/v1beta/models/"
               f"{cfg['model']}:generateContent")
        body = {
            "system_instruction": {"parts": [{"text": system}]},
            "contents": [{"role": "user", "parts": [{"text": text}]}],
            "generationConfig": {"temperature": 0, "maxOutputTokens": 2048,
                                 "thinkingConfig": {"thinkingLevel": "MINIMAL"},
                                 "responseMimeType": "application/json"},
        }
        r = self._post_backend(
            "gemini", cfg, url, body, headers={"x-goog-api-key": key})
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
        r = self._post_backend(
            name, cfg, cfg["base_url"] + "/chat/completions", body,
            headers=headers)
        return r.json()["choices"][0]["message"]["content"]

    # ------------------------------------------------------------ handler từng hành động
    def _mission_context(self):
        age = time.time() - self._sup_state_t if self._sup_state_t else None
        nav_id, nav_state = self._nav_latest()
        raw = self._sup_status if isinstance(self._sup_status, dict) else {}
        cycle = raw.get("cycle") if isinstance(raw.get("cycle"), dict) else {}
        dest = raw.get("active_dest")
        dest = dest if dest in ("NODE_HOME", "P_1", "P_2", "D_1") else None
        mission = raw.get("active_mission")
        mission = mission if mission in MISSIONS else None
        run_mode = cycle.get("run_mode")
        run_mode = run_mode if run_mode in ("auto", "single") else "unknown"
        number = cycle.get("number")
        number = number if isinstance(number, int) and 0 <= number < 1000000 else None
        cargo = raw.get("cargo") if type(raw.get("cargo")) is bool else None
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
            "supervisor_state": self._sup_state if self._sup_state in {
                "IDLE", "GOTO_PICK", "PICKING", "GOTO_D", "DROPPING",
                "WAITING_SLOT", "RETURN_HOME", "RECOVERY", "DONE", "FAILED", "STOPPED"
            } else "unknown",
            "supervisor_status_age_s": round(age, 1) if age is not None else None,
            "active_mission": mission,
            "active_destination": dest,
            "cycle": {"run_mode": run_mode, "number": number},
            "cargo": cargo,
            "pallet_state": pallet_state,
            "pallet_status_age_s": (round(time.time() - self._pallet_state_t, 1)
                                    if self._pallet_state_t else None),
            "robot_stopped": raw.get("robot_stopped") if type(raw.get("robot_stopped")) is bool else None,
            "nav_status_fresh": raw.get("nav_status_fresh") is True,
            "supervisor_nav_goal_active": raw.get("nav_goal_active") if type(raw.get("nav_goal_active")) is bool else None,
            "lift_cm": lift,
            "operating_mode": self._op_mode,
            "nav_goal_active": nav_id is not None and nav_state in (1, 2, 6),
            "nav_goal_status": nav_state,
        }

    def _fresh_supervisor_status(self):
        age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
        max_age = self.get_parameter("supervisor_status_max_age_s").value
        if not isinstance(self._sup_status, dict) or age > max_age:
            raise RuntimeError("Mission supervisor status không có hoặc đã cũ")
        return self._sup_status

    def _fresh_pallet_status(self):
        if (not isinstance(self._pallet_status, dict) or not self._pallet_state_t
                or time.time() - self._pallet_state_t > 3.0):
            raise RuntimeError("Pallet status không có hoặc đã cũ")
        data = self._pallet_status
        if type(data.get("cargo")) is not bool:
            raise RuntimeError("Pallet manager chưa xác nhận trạng thái tải")
        return data

    def _require_cycle_idle(self, allow_cargo=False):
        status = self._fresh_supervisor_status()
        st, t = self._sup_state, self._sup_state_t
        busy = {s.upper() for s in self.get_parameter("supervisor_busy_states").value}
        age = time.time() - t if t else float("inf")
        max_age = self.get_parameter("supervisor_status_max_age_s").value
        if st in busy and age <= max_age:
            raise RuntimeError(f"Mission supervisor đang bận ({st}); hãy dừng hoặc resume đúng cycle trước")
        if st not in {"IDLE", "DONE", "FAILED", "STOPPED"}:
            raise RuntimeError(f"Mission supervisor ở trạng thái chưa cho phép thao tác ({st or 'unknown'})")
        pallet = self._fresh_pallet_status()
        if type(status.get("cargo")) is not bool or status.get("cargo") != pallet.get("cargo"):
            raise RuntimeError("Mission supervisor và pallet manager không thống nhất trạng thái tải")
        carrying = status.get("cargo") is True
        if carrying and not allow_cargo:
            raise RuntimeError("Robot đang mang pallet; chỉ cho phép đi tới D_1 và thao tác cycle phù hợp")
        pallet_active = (self._pallet_state_t > 0.0
                         and time.time() - self._pallet_state_t <= 3.0
                         and self._pallet_state not in
                         (None, "IDLE", "DONE", "CARRYING", "FAILED", "STOPPED"))
        if pallet_active:
            raise RuntimeError(f"Pallet mission đang thực thi ({self._pallet_state})")
        return True

    def _require_stationary(self, allow_cargo=False):
        self._require_cycle_idle(allow_cargo=allow_cargo)
        status = self._fresh_supervisor_status()
        if status.get("robot_stopped") is not True:
            raise RuntimeError("Odometry chưa xác nhận xe đang đứng yên")
        if status.get("nav_status_fresh") is not True or status.get("nav_goal_active") is not False:
            raise RuntimeError("Nav2 status chưa xác nhận goal đã kết thúc")

    def _check_node_pose(self, target):
        response = self._call(self.cli_node_check[target], Trigger.Request(), timeout=5.0)
        if not response.success:
            raise RuntimeError(response.message or f"Vị trí thực tế không hợp lệ tại {target}")

    def _h_set_mode(self, args):
        if self.get_parameter("dry_run").value:
            self.get_logger().info(f"[dry_run] set_mode {args['mode']}")
            return True
        mode = args["mode"]
        prior_state = self._sup_state
        prior_age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
        resp = self._call(self.cli_set_mode, SetBool.Request(data=(mode == "AUTO")), timeout=8.0)
        if not resp.success:
            self.get_logger().error(f"Đổi mode thất bại: {resp.message}")
            return False
        self._op_mode = mode
        if mode == "MANUAL":
            busy_states = {s.upper() for s in self.get_parameter("supervisor_busy_states").value}
            if prior_state in busy_states and prior_age <= self.get_parameter("supervisor_status_max_age_s").value:
                # A canceled in-flight mission may have stopped between named
                # route nodes. Named mission starts are no longer trustworthy.
                self._location = "unknown"
            # Wait until the supervisor has acknowledged STOPPED/RECOVERY.
            # RECOVERY is intentionally not treated as idle by quick-goal checks.
            deadline = time.time() + 3.0
            while time.time() < deadline:
                self._check_abort()
                age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
                if age <= self.get_parameter("supervisor_status_max_age_s").value and self._sup_state not in {
                        s.upper() for s in self.get_parameter("supervisor_busy_states").value}:
                    break
                if self._sup_state == "RECOVERY" and age <= 2.0:
                    break
                time.sleep(0.1)
        return True

    def _h_quick_goal(self, args):
        self._require_cycle_idle()
        if self._op_mode != "MANUAL":
            raise RuntimeError("Quick goal chỉ chạy ở MANUAL mode")
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

    def _start_cycle(self, run_mode):
        if self.get_parameter("dry_run").value:
            self.get_logger().info(f"[dry_run] start_cycle {run_mode}")
            return True
        status = self._sup_status
        age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
        if not status or age > self.get_parameter("supervisor_status_max_age_s").value:
            self.get_logger().error("Không có mission status mới từ supervisor")
            return False
        if status.get("can_start") is not True or status.get("cargo") is not False:
            self.get_logger().error(f"Supervisor chưa sẵn sàng ({self._sup_state})")
            return False
        if (status.get("robot_stopped") is not True
                or status.get("nav_status_fresh") is not True
                or status.get("nav_goal_active") is not False):
            self.get_logger().error("Chưa xác nhận robot đứng yên và Nav2 không có goal")
            return False
        try:
            pallet = self._fresh_pallet_status()
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return False
        if (pallet.get("cargo") is not False
                or pallet.get("state") not in ("IDLE", "DONE", "FAILED", "STOPPED")):
            self.get_logger().error("Pallet manager chưa xác nhận xe rỗng và sẵn sàng")
            return False
        nav_id, nav_state = self._nav_latest()
        if nav_id is not None and nav_state in (1, 2, 6):
            self.get_logger().error("Nav2 đang chạy goal; kết thúc goal trước khi chạy cycle")
            return False
        if not self._h_set_mode({"mode": "AUTO"}):
            return False
        # Mode change may block while stopping a prior controller; re-read
        # all start preconditions immediately before the supervisor call.
        status = self._fresh_supervisor_status()
        pallet = self._fresh_pallet_status()
        if (status.get("can_start") is not True or status.get("cargo") is not False
                or pallet.get("cargo") is not False
                or status.get("robot_stopped") is not True
                or status.get("nav_status_fresh") is not True
                or status.get("nav_goal_active") is not False):
            self.get_logger().error("Trạng thái thay đổi trong lúc chuyển AUTO; hủy khởi động cycle")
            return False
        req = SetParameters.Request()
        req.parameters = [Parameter("run_mode", Parameter.Type.STRING, run_mode).to_parameter_msg()]
        response = self._call(self.cli_cycle_params, req, timeout=5.0)
        if not response.results or not response.results[0].successful:
            return False
        result = self._call(self.cli_cycle_start, Trigger.Request(), timeout=5.0)
        if not result.success:
            self._failure_message = f"Supervisor từ chối start: {result.message}"
            self.get_logger().error(self._failure_message)
            return False
        deadline = time.monotonic() + 10.0
        goto_seen_at = None
        while time.monotonic() < deadline:
            self._check_abort()
            age = time.time() - self._sup_state_t if self._sup_state_t else float("inf")
            state = self._sup_state
            if age <= self.get_parameter("supervisor_status_max_age_s").value:
                if state in ("RECOVERY", "FAILED"):
                    self._failure_message = (
                        f"Cycle vào {state}: {(self._sup_status or {}).get('msg', '')}")
                    self.get_logger().error(self._failure_message)
                    return False
                if state == "GOTO_PICK":
                    goto_seen_at = goto_seen_at or time.monotonic()
                    if time.monotonic() - goto_seen_at >= 0.8:
                        return True
                elif (state not in (None, "IDLE")
                      and (self._sup_status or {}).get("nav_goal_active") is True):
                    return True
                else:
                    goto_seen_at = None
            time.sleep(0.1)
        self._failure_message = (
            f"Supervisor không xác nhận cycle khởi động trong 10 giây "
            f"(state={self._sup_state or 'unknown'}; "
            f"msg={(self._sup_status or {}).get('msg', '')})")
        self.get_logger().error(self._failure_message)
        return False

    def _h_start_auto_cycle(self, _args):
        """Start the same auto cycle exposed by the Web dashboard."""
        return self._start_cycle("auto")

    def _h_start_single_cycle(self, _args):
        return self._start_cycle("single")

    def _h_stop_cycle(self, _args):
        if self.get_parameter("dry_run").value:
            return True
        status = self._fresh_supervisor_status()
        if self._sup_state not in {s.upper() for s in self.get_parameter("supervisor_busy_states").value}:
            self.get_logger().error("Không có cycle đang chạy để dừng")
            return False
        result = self._call(self.cli_mstop, Trigger.Request(), timeout=5.0)
        return bool(result.success)

    def _h_cancel_navigation(self, _args):
        if self.get_parameter("dry_run").value:
            return True
        self._require_cycle_idle(allow_cargo=True)
        goal_id, goal_state = self._nav_latest()
        if goal_id is None or goal_state not in (1, 2, 6):
            self._success_message = "Không có goal Nav2"
            self._status("running", msg="Không có goal Nav2 để hủy")
            self.get_logger().info("Không có goal Nav2 đang chạy")
            return True
        result = self._call(self.cli_cancel, CancelGoal.Request(), timeout=5.0)
        if not result.goals_canceling:
            self.get_logger().error("Nav2 không chấp nhận yêu cầu hủy goal")
            return False
        self.pub_cmd.publish(Twist())
        return True

    def _h_resume_cycle(self, _args):
        if self.get_parameter("dry_run").value:
            return True
        status = self._fresh_supervisor_status()
        if self._sup_state != "RECOVERY" or status.get("resume_available") is not True:
            self.get_logger().error("Chỉ resume được khi mission ở RECOVERY")
            return False
        client = self.create_client(Trigger, "/mission/resume", callback_group=self._cb)
        result = self._call(client, Trigger.Request(), timeout=5.0)
        return bool(result.success)

    def _h_go_mission(self, a):
        name = a["name"]
        source, destination = mission_from_to(name)
        if self._op_mode != "MANUAL":
            raise RuntimeError("Mission thủ công cần chuyển Navigation sang MANUAL trước")
        cargo = self._fresh_supervisor_status().get("cargo") is True
        self._require_cycle_idle(allow_cargo=cargo and destination == "d1")
        if cargo and destination != "d1":
            raise RuntimeError("Xe đang mang pallet; chỉ được đi tới D_1 để giao pallet")
        self._require_stationary(allow_cargo=cargo and destination == "d1")
        if source is not None:
            self._check_node_pose({"home": "HOME", "p1": "P_1", "p2": "P_2", "d1": "D_1"}[source])
        base_id, _ = self._nav_latest()
        # Nếu service trả về chỉ sau khi mission xong, tăng timeout ở dòng này.
        response = self._call(self.cli_mission[name], Trigger.Request(), timeout=15.0)
        if not response.success:
            self.get_logger().error(f"Mission {name} không gửi được goal: {response.message}")
            return False
        ok = self._wait_nav_goal(base_id)
        self._location = mission_from_to(name)[1] if ok else "unknown"
        return ok

    def _wait_nav_goal(self, base_id):
        """Chờ goal Nav2 MỚI xuất hiện rồi chờ nó kết thúc."""
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
        """SetParameters tới /pallet_mission/set_parameters (mẫu của
        mission_supervisor_node). Trả True/False, không throw."""
        try:
            if not self.cli_set_param.wait_for_service(timeout_sec=2.0):
                return False
            req = SetParameters.Request()
            req.parameters = [
                Parameter(name, Parameter.Type.STRING, value).to_parameter_msg()]
            resp = self._call(self.cli_set_param, req, timeout=5.0)
            return all(x.successful for x in resp.results)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"Không set duoc {name}={value!r}: {e}")
            return False

    def _h_pick_pallet(self, _a):
        if self._op_mode != "MANUAL":
            raise RuntimeError("Nhận pallet thủ công cần MANUAL mode")
        if self._location not in ("p1", "p2"):
            raise RuntimeError("Chỉ nhận pallet khi vị trí dự kiến là P_1 hoặc P_2")
        self._require_stationary()
        pallet = self._fresh_pallet_status()
        if pallet.get("cargo") is not False or self._fresh_supervisor_status().get("cargo") is not False:
            raise RuntimeError("Không nhận pallet khi xe đang mang tải hoặc trạng thái tải chưa rõ")
        if self._pallet_state not in ("IDLE", "DONE", "FAILED", "STOPPED"):
            raise RuntimeError(f"Pallet mission không sẵn sàng ({self._pallet_state})")
        self._check_node_pose("P_1" if self._location == "p1" else "P_2")
        # Giả định theo báo cáo: detect ON (gom mẫu) -> detect OFF (chốt pose)
        # -> go_to_pallet. Nếu nhóm đã gộp thành 1 service, đặt detect_first:=false.
        p = self.get_parameter
        # Q2: drop_pallet sau này cần xe dừng ở CARRYING (giữ pallet)
        if p("set_after_pick_hold").value:
            if not self._set_pallet_param("after_pick", "hold"):
                self.get_logger().warn(
                    "Không set được after_pick='hold' - drop_pallet có thể bị "
                    "pallet_mission từ chối (pallet tự hạ ở HOME)")
        if p("detect_first").value:
            detect_on = self._call(self.cli_detect, SetBool.Request(data=True))
            if not detect_on.success:
                raise RuntimeError(f"Pallet detect không khởi động: {detect_on.message}")
            self._sleep(p("detect_seconds").value)
            detect_off = self._call(self.cli_detect, SetBool.Request(data=False))
            if not detect_off.success:
                raise RuntimeError(f"Pallet detect không chốt được: {detect_off.message}")
        # Detection takes several seconds. Recheck pose, cargo, and stationary
        # state at the point where the pallet approach action is actually sent.
        self._require_stationary()
        pallet = self._fresh_pallet_status()
        if pallet.get("cargo") is not False or self._fresh_supervisor_status().get("cargo") is not False:
            raise RuntimeError("Trạng thái tải đổi trong lúc detect; hủy thao tác nhận pallet")
        if self._pallet_state != "READY":
            raise RuntimeError(f"Pallet detect chưa ở trạng thái READY ({self._pallet_state}); hủy thao tác")
        self._check_node_pose("P_1" if self._location == "p1" else "P_2")
        self._pallet_state = None
        r = self._call(self.cli_go, Trigger.Request())
        if not r.success:
            self.get_logger().error(f"go_to_pallet từ chối: {r.message}")
            return False
        return self._wait_pallet_done()

    def _h_drop_pallet(self, _a):
        """Drop cargo at D_1 after mission-state and cargo checks."""
        if self._op_mode != "MANUAL":
            raise RuntimeError("Giao pallet thủ công cần MANUAL mode")
        self._require_stationary(allow_cargo=True)
        if self._location != "d1":
            raise RuntimeError("Chỉ được drop pallet khi robot ở D_1")
        pallet = self._fresh_pallet_status()
        if (self._fresh_supervisor_status().get("cargo") is not True
                or pallet.get("cargo") is not True or self._pallet_state != "CARRYING"):
            raise RuntimeError("Robot không ở trạng thái đang mang pallet")
        self._check_node_pose("D_1")
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
        if self._op_mode != "MANUAL":
            raise RuntimeError("Điều khiển càng thủ công cần MANUAL mode")
        sup = self._fresh_supervisor_status()
        pallet = self._fresh_pallet_status()
        carrying = sup.get("cargo") is True or pallet.get("cargo") is True
        self._require_stationary(allow_cargo=carrying and a.get("level") == "carry")
        if sup.get("cargo") != pallet.get("cargo"):
            raise RuntimeError("Supervisor và pallet manager không thống nhất trạng thái tải")
        if carrying and a.get("level") != "carry":
            raise RuntimeError("Xe đang mang pallet; chỉ chấp nhận mức carry")
        # GIỮ CHỖ: thay bằng service/topic nâng theo mức mới của nhóm,
        # và chờ phản hồi vị trí thật thay vì chờ theo thời gian.
        cm = LIFT_LEVELS[a["level"]]
        current_cm = pallet.get("lift_cm")
        if not isinstance(current_cm, (int, float)) or not math.isfinite(float(current_cm)):
            raise RuntimeError("Không có số đo lift hiện tại mới; từ chối lệnh nâng/hạ")
        if not (0.0 <= float(current_cm) <= LIFT_LIMIT_CM
                and 0.0 <= float(cm) <= LIFT_LIMIT_CM):
            raise RuntimeError(f"Lift target/current vượt giới hạn 0–{LIFT_LIMIT_CM:g} cm")
        if abs(float(current_cm) - float(cm)) <= 1.0:
            self._success_message = f"Lift đã ở mức {cm:g} cm"
            self._status("running", msg=f"Lift đã ở mức {cm:g} cm")
            return True
        self._lift_active = True
        self.pub_lift.publish(String(data=f"H{cm:g}"))
        deadline = time.time() + self.get_parameter("lift_wait_s").value
        try:
            while time.time() < deadline:
                self._check_abort()
                age = time.time() - self._pallet_state_t if self._pallet_state_t else float("inf")
                measured = (self._pallet_status or {}).get("lift_cm")
                if (age <= 1.5 and isinstance(measured, (int, float))
                        and math.isfinite(float(measured))
                        and abs(float(measured) - float(cm)) <= 1.0):
                    self._lift_active = False
                    return True
                time.sleep(0.1)
        except Aborted:
            self.pub_lift.publish(String(data="M0"))
            self._lift_active = False
            raise
        self.pub_lift.publish(String(data="M0"))
        self._lift_active = False
        raise RuntimeError(f"Lift chưa tới {cm:g} cm trong thời gian cho phép; đã gửi M0")


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
