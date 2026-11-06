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
  * Nếu mission_supervisor đang bận (GOTO_PICK/PICKING/GOTO_D/DROPPING/
    RECOVERY) -> từ chối lệnh, tránh 2 bộ điều khiển cùng một lúc.

Logic thuần (bảng cấu hình + validate) nằm ở module llm_logic.py để test
được không cần ROS.
"""
import json
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
        d("llm_timeout_s", 8.0)
        d("dry_run", False)                    # True: chỉ in kế hoạch, không điều khiển
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
          ["GOTO_PICK", "PICKING", "GOTO_D", "DROPPING", "RECOVERY"])
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
        self._location = self.get_parameter("start_location").value
        self._nav_list = []
        self._pallet_state = None
        self._clarify_ctx = None   # (yêu cầu gốc, câu hỏi lại, thời điểm)
        self._sup_state = None     # state mới nhất của mission_supervisor
        self._sup_state_t = 0.0    # mốc thời gian nhận status

        g = self._cb
        self.cli_mission = {
            m: self.create_client(Empty, f"mission_manager/trigger_{m}",
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

        self.pub_plan = self.create_publisher(String, "/llm/plan", 10)
        self.pub_status = self.create_publisher(String, "/llm/status", 10)
        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_lift = self.create_publisher(String, p("lift_topic"), 10)

        self.create_subscription(String, "/llm/command", self._on_command, 10, callback_group=g)
        self.create_subscription(Bool, "/llm/confirm", self._on_confirm, 10, callback_group=g)
        self.create_subscription(String, "/llm/set_location", self._on_set_loc, 10, callback_group=g)
        self.create_subscription(GoalStatusArray, "navigate_to_pose/_action/status",
                                 self._on_nav_status, 10, callback_group=g)
        self.create_subscription(String, p("pallet_status_topic"),
                                 self._on_pallet_status, 10, callback_group=g)
        self.create_subscription(String, p("supervisor_status_topic"),
                                 self._on_supervisor_status, 10, callback_group=g)
        self.create_service(Trigger, "llm/stop", self._srv_stop, callback_group=g)

        self._status("idle", msg="sẵn sàng")
        self.get_logger().info(
            f"llm_command_node sẵn sàng (dry_run={p('dry_run')}, backends={p('backends')})")

    # ------------------------------------------------------------ tiện ích
    def _status(self, state, **kw):
        self.pub_status.publish(String(data=json.dumps(
            {"state": state, "location": self._location, **kw}, ensure_ascii=False)))

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
        self._location = msg.data.strip().lower()
        self._status("idle", msg=f"đặt vị trí = {self._location}")

    def _on_nav_status(self, msg):
        with self._lock:
            self._nav_list = list(msg.status_list)

    def _on_pallet_status(self, msg):
        try:
            data = json.loads(msg.data)
            st = data.get(self.get_parameter("pallet_state_key").value)
            self._pallet_state = str(st).upper() if st is not None else None
        except Exception:  # noqa: BLE001
            pass

    def _on_supervisor_status(self, msg):
        try:
            data = json.loads(msg.data)
            st = data.get(self.get_parameter("pallet_state_key").value)
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
        text = msg.data.strip()
        if not text:
            return
        if is_stop(text):
            self._emergency_stop("lệnh dừng")
            return
        if not self._busy.acquire(blocking=False):
            self._status("busy", msg="Đang thực hiện lệnh khác. Nói 'dừng' để hủy.")
            return
        ref = self._supervisor_refusal()
        if ref:
            self._busy.release()
            self._status("busy", msg=ref)
            return
        threading.Thread(target=self._worker, args=(text,), daemon=True).start()

    def _srv_stop(self, _req, resp):
        self._emergency_stop("service llm/stop")
        resp.success, resp.message = True, "đã dừng"
        return resp

    def _emergency_stop(self, why):
        self.get_logger().warn(f"DỪNG KHẨN: {why}")
        self._abort.set()
        self._clarify_ctx = None
        self.pub_cmd.publish(Twist())              # v = 0
        # KHÔNG gửi 0 cho trục nâng: linear.z được hiểu là vị trí (cm).
        if self.cli_cancel.service_is_ready():
            self.cli_cancel.call_async(CancelGoal.Request())   # hủy mọi goal Nav2
        if self.cli_pstop.service_is_ready():
            self.cli_pstop.call_async(Trigger.Request())
        # Dừng chu trình mission_supervisor (chỉ khi nó đang chạy)
        if self.cli_mstop.service_is_ready():
            self.cli_mstop.call_async(Trigger.Request())
        self._confirm_evt.set()
        self._status("stopped", msg=why)

    # ------------------------------------------------------------ luồng chính
    def _worker(self, text):
        try:
            self._abort.clear()
            self._confirm_evt.clear()
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
            plan = self._plan(request)
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
            self.pub_plan.publish(String(data=json.dumps(
                {"say": plan.get("say", ""), "steps": steps}, ensure_ascii=False)))
            if p("require_confirm").value:
                self._status("waiting_confirm", msg=plan.get("say", ""), steps=steps)
                if not self._wait_confirm():
                    self._status("cancelled", msg="Chưa được xác nhận")
                    return
            ref = self._supervisor_refusal()
            if ref:
                self._status("busy", msg=ref)
                return
            self._run_plan(steps)
        except Aborted:
            self._status("stopped", msg="đã dừng")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"Lỗi: {e}")
            self._status("error", msg=str(e))
        finally:
            self._busy.release()

    def _wait_confirm(self):
        self._confirm_evt.clear()
        self._confirm_val = False
        timeout = self.get_parameter("confirm_timeout_s").value
        self._confirm_evt.wait(timeout)
        self._check_abort()
        return self._confirm_val

    def _run_plan(self, steps):
        handlers = {"go_mission": self._h_go_mission,
                    "pick_pallet": self._h_pick_pallet,
                    "drop_pallet": self._h_drop_pallet,
                    "lift_to": self._h_lift_to}
        dry = self.get_parameter("dry_run").value
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
                             msg=f"Bước {i} thất bại: {st['action']} {st['args']}")
                return
        self._status("done", msg="Hoàn thành")

    # ------------------------------------------------------------ LLM
    def _plan(self, text):
        system = build_system_prompt(self._location)
        last = None
        for name in self.get_parameter("backends").value:
            try:
                cfg = BACKENDS[name]
                if cfg["kind"] == "gemini":
                    raw = self._call_gemini(cfg, system, text)
                else:
                    raw = self._call_openai(cfg, system, text)
                return extract_json(raw)
            except Exception as e:  # noqa: BLE001
                last = e
                self.get_logger().warn(f"Backend '{name}' lỗi: {e}")
        raise RuntimeError(f"Mọi LLM backend đều lỗi: {last}")

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
                                 "responseMimeType": "application/json"},
        }
        r = requests.post(url, json=body,
                          timeout=self.get_parameter("llm_timeout_s").value,
                          headers={"x-goog-api-key": key})
        r.raise_for_status()
        return r.json()["candidates"][0]["content"]["parts"][0]["text"]

    def _call_openai(self, cfg, system, text):
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
        r = requests.post(cfg["base_url"] + "/chat/completions", json=body,
                          headers=headers,
                          timeout=self.get_parameter("llm_timeout_s").value)
        r.raise_for_status()
        return r.json()["choices"][0]["message"]["content"]

    # ------------------------------------------------------------ handler từng hành động
    def _h_go_mission(self, a):
        name = a["name"]
        base_id, _ = self._nav_latest()
        # Nếu service trả về chỉ sau khi mission xong, tăng timeout ở dòng này.
        self._call(self.cli_mission[name], Empty.Request(), timeout=5.0)
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
            self._call(self.cli_detect, SetBool.Request(data=True))
            self._sleep(p("detect_seconds").value)
            self._call(self.cli_detect, SetBool.Request(data=False))
        self._pallet_state = None
        r = self._call(self.cli_go, Trigger.Request())
        if not r.success:
            self.get_logger().error(f"go_to_pallet từ chối: {r.message}")
            return False
        return self._wait_pallet_done()

    def _h_drop_pallet(self, _a):
        """Đặt pallet đang mang lên kệ (pallet_mission tự chọn ô theo
        shelf.yaml + /shelf/occupancy). Cần: robot ở D_1, đang CARRYING,
        cargo=True - chính là trạng thái sau pick với after_pick='hold'."""
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
        # GIỮ CHỖ: thay bằng service/topic nâng theo mức mới của nhóm,
        # và chờ phản hồi vị trí thật thay vì chờ theo thời gian.
        cm = LIFT_LEVELS[a["level"]]
        self.pub_lift.publish(String(data=f"H{cm:g}"))
        self._sleep(self.get_parameter("lift_wait_s").value)
        return True


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
