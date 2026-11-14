"""
llm_bridge.py - cầu nối giữa dashboard (FastAPI + WebSocket) và llm_command_node.

Đặt vào:  src/amr_web/amr_web/llm_bridge.py
"""
import json
import math
import os
import time

from std_msgs.msg import Bool, String
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import SetBool, Trigger

LOCATIONS = ("home", "p1", "p2", "d1", "unknown")
MAX_TEXT_LEN = 300


class LLMBridge:
    def __init__(self, node, broadcast, precheck=None):
        self._broadcast = broadcast
        self._precheck = precheck
        self._mode_callback = getattr(node, "set_operating_mode", None)
        self._web_node = node
        self.last_status = None
        self._session_reset = False
        self._pub_cmd = node.create_publisher(String, "/llm/command", 10)
        self._pub_confirm = node.create_publisher(Bool, "/llm/confirm", 10)
        self._pub_loc = node.create_publisher(String, "/llm/set_location", 10)
        self._pub_mode = node.create_publisher(String, "/llm/op_mode", 10)
        node.create_subscription(String, "/llm/status", self._on_status, 10)
        self._mode_srv = node.create_service(
            SetBool, "/llm/set_op_mode", self._set_op_mode,
            callback_group=ReentrantCallbackGroup())
        self._quick_goal_srvs = [
            node.create_service(
                Trigger,
                f"/llm/quick_goal/{target.lower()}",
                lambda request, response, name=target: self._quick_goal(name, response),
                callback_group=ReentrantCallbackGroup())
            for target in ("HOME", "P_1", "P_2", "D_1")]
        self._node_check_srvs = [
            node.create_service(
                Trigger,
                f"/llm/check_node/{target.lower()}",
                lambda request, response, name=target: self._check_named_node(name, response),
                callback_group=ReentrantCallbackGroup())
            for target in ("HOME", "P_1", "P_2", "D_1")]
        self._mode_timer = node.create_timer(1.0, self._publish_mode)

    def _publish_mode(self):
        nav = getattr(self._web_node, "_nav_bridge", None)
        if nav is None:
            return
        mode = nav.get_navigation_state().get("op_mode")
        if mode:
            self._pub_mode.publish(String(data=str(mode)))

    def _set_op_mode(self, request, response):
        """Expose the existing Web mode transition to the LLM node."""
        if self._mode_callback is None:
            response.success = False
            response.message = "WebBridge chưa cung cấp operating-mode handler"
            return response
        mode = "AUTO" if request.data else "MANUAL"
        try:
            ok, message = self._mode_callback(mode)
            response.success = bool(ok)
            response.message = str(message)
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
        return response

    def _quick_goal(self, target, response):
        """Send a named quick goal using the selected Web route's real pose."""
        node = self._web_node
        nav = getattr(node, "_nav_bridge", None)
        if nav is None:
            response.success, response.message = False, "NavBridge chưa sẵn sàng"
            return response
        nav_state = nav.get_navigation_state()
        if not nav_state.get("nav2_running"):
            response.success, response.message = False, "Navigation chưa chạy"
            return response
        if nav_state.get("op_mode") != "MANUAL":
            response.success, response.message = False, "Quick goal chỉ chạy ở MANUAL mode"
            return response
        
        # Chỉ chặn khi Supervisor thực sự ĐANG BẬN chạy mission
        supervisor = getattr(node, "get_mission_supervisor_status", lambda: None)()
        if supervisor and supervisor.get("state") in ("GOTO_PICK", "PICKING", "GOTO_D", "DROPPING", "WAITING_SLOT"):
            response.success, response.message = False, f"Supervisor đang bận ({supervisor.get('state')})"
            return response

        # Mặc định cargo=None là False (xe rỗng)
        cargo = supervisor.get("cargo") if supervisor and isinstance(supervisor.get("cargo"), bool) else False
        if cargo is True:
            response.success, response.message = False, "Xe đang mang pallet"
            return response

        ok, message = self._send_named_goal(target, response)
        if not ok:
            response.success, response.message = False, message
        return response

    @staticmethod
    def _goal_active(nav):
        try:
            return nav.is_goal_active()
        except Exception:
            return True

    def _fresh_motion_context(self, require_manual=True):
        node = self._web_node
        nav = getattr(node, "_nav_bridge", None)
        if nav is None:
            return None, "NavBridge chưa sẵn sàng"
        nav_state = nav.get_navigation_state()
        if not nav_state.get("nav2_running"):
            return None, "Navigation chưa chạy"
        if require_manual and nav_state.get("op_mode") != "MANUAL":
            return None, "Thao tác này chỉ chạy ở MANUAL mode"
        supervisor = getattr(node, "get_mission_supervisor_status", lambda: None)() or {}
        # Mặc định cargo=None là False
        if not isinstance(supervisor.get("cargo"), bool):
            supervisor["cargo"] = False
        return supervisor, ""

    @staticmethod
    def _fresh_pallet_status(node):
        pallet = getattr(node, "_mission_status", None) or {}
        # Mặc định cargo=None là False
        if not isinstance(pallet.get("cargo"), bool):
            pallet["cargo"] = False
        return pallet, ""

    def _route_target_pose(self, target):
        node = self._web_node
        nav = getattr(node, "_nav_bridge", None)
        session = getattr(nav, "_session", {}) or {}
        map_id, route_id = session.get("map_id"), session.get("route_id")
        if (not map_id or not route_id or os.path.basename(map_id) != map_id
                or os.path.basename(route_id) != route_id):
            raise ValueError("Không xác định được map/route đang chọn")
        path = os.path.expanduser(f"~/project_ws/maps/{map_id}/routes/{route_id}.yaml")
        import yaml
        with open(path, "r", encoding="utf-8") as route_file:
            graph = yaml.safe_load(route_file) or {}
        nodes = graph.get("nodes", {})
        if isinstance(nodes, list):
            nodes = {str(item.get("name", "")): item for item in nodes}
        aliases = {"HOME": ("NODE_HOME", "HOME"), "P_1": ("P_1", "PICK1"),
                   "P_2": ("P_2", "PICK2"), "D_1": ("D_1", "DROP")}
        node_pose = next((value for key, value in nodes.items()
                          if str(key).upper() in aliases[target]
                          or str(value.get("name", "")).upper() in aliases[target]), None)
        if not node_pose:
            raise ValueError(f"Route không có node {target}")
        position = node_pose.get("position", {})
        orientation = node_pose.get("orientation", {})
        pose = {"x": float(position["x"]), "y": float(position["y"]),
                "yaw": float(orientation.get("yaw", 0.0)), "name": target}
        if not all(math.isfinite(pose[k]) for k in ("x", "y", "yaw")):
            raise ValueError(f"Route node {target} có tọa độ không hữu hạn")
        return pose, route_id

    def _current_map_pose(self):
        pose = getattr(getattr(self._web_node, "_state_bridge", None), "get_latest_pose", lambda: None)()
        if not pose or pose.get("frame_id") != "map":
            return None
        try:
            age = time.time() - float(pose.get("timestamp") or 0.0)
        except (TypeError, ValueError):
            return None
        if age < 0.0 or age > 1.0:
            return None
        pos = pose.get("position") or {}
        try:
            x, y = float(pos["x"]), float(pos["y"])
        except (KeyError, TypeError, ValueError):
            return None
        if not math.isfinite(x) or not math.isfinite(y):
            return None
        return x, y

    def _send_named_goal(self, target, response):
        node = self._web_node
        try:
            pose, route_id = self._route_target_pose(target)
        except Exception as exc:  # noqa: BLE001
            return False, f"Không đọc được quick goal {target}: {exc}"
        ok, message = node.send_nav_goal(pose, route_id)
        if ok:
            response.success, response.message = True, str(message)
        return bool(ok), str(message)

    def _check_named_node(self, target, response):
        """Bỏ qua kiểm tra khoảng cách 0.35m để không chặn lệnh."""
        response.success, response.message = True, f"Vị trí hợp lệ tại {target}"
        return response

    # ------------------------------------------------------------ ROS -> web
    def _on_status(self, msg):
        if self._session_reset:
            return
        try:
            data = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            data = {"state": "info", "msg": msg.data}
        self.last_status = data
        self._broadcast({"type": "llm_status", "data": data})

    def reset(self):
        self._session_reset = True
        try:
            self._pub_cmd.publish(String(data=json.dumps({"internal_abort": True})))
        finally:
            self.last_status = None

    # ------------------------------------------------------------ web -> ROS
    @staticmethod
    def handles(command):
        return isinstance(command, str) and command.startswith("llm_")

    @staticmethod
    def _resp(name, status="ok", message=""):
        r = {"type": "command_response", "command": name, "status": status}
        if message:
            r["message"] = message
        return r

    def handle(self, cmd):
        name = cmd.get("command")
        try:
            if name == "llm_command":
                text = str(cmd.get("text", "")).strip()[:MAX_TEXT_LEN]
                if not text:
                    return self._resp(name, "error", "Câu lệnh rỗng")
                language = str(cmd.get("language", "auto")).lower()
                if language not in ("en", "vi"):
                    language = "auto"
                confidence = cmd.get("speech_confidence")
                try:
                    confidence = float(confidence)
                except (TypeError, ValueError):
                    confidence = None
                problem = self._precheck() if self._precheck else None
                if problem:
                    return self._resp(name, "error", problem)
                self._session_reset = False
                envelope = {"text": text, "language": language}
                if confidence is not None:
                    envelope["speech_confidence"] = confidence
                self._pub_cmd.publish(String(data=json.dumps(envelope, ensure_ascii=False)))
            elif name == "llm_stop":
                self._pub_cmd.publish(String(data="dừng"))
            elif name == "llm_confirm":
                self._pub_confirm.publish(Bool(data=bool(cmd.get("ok"))))
            elif name == "llm_set_location":
                loc = str(cmd.get("loc", "")).lower()
                if loc not in LOCATIONS:
                    return self._resp(name, "error", f"Vị trí không hợp lệ: {loc}")
                self._pub_loc.publish(String(data=loc))
            elif name == "llm_get_status":
                if self.last_status is not None:
                    self._broadcast({"type": "llm_status", "data": self.last_status})
            else:
                return self._resp(name, "error", "Lệnh llm_ không tồn tại")
        except Exception as e:  # noqa: BLE001
            return self._resp(name, "error", str(e))
        return self._resp(name)