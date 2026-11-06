"""
llm_bridge.py - cầu nối giữa dashboard (FastAPI + WebSocket) và llm_command_node.

Đặt vào:  src/amr_web/amr_web/llm_bridge.py

Luồng:
  Trình duyệt --sendCommand({command:'llm_command', text})--> web_server.handle_command
      --> LLMBridge.handle()  --> publish /llm/command  --> llm_command_node
  llm_command_node --> /llm/status --> LLMBridge --> broadcast {type:'llm_status', data}
      --> ros_connection.js --> window.handleLLMStatus() (file llm_voice.js)

Các lệnh WebSocket được hỗ trợ (đều bắt đầu bằng "llm_"):
  llm_command {text}   gửi câu lệnh tiếng Việt
  llm_confirm {ok}     xác nhận / hủy kế hoạch
  llm_stop             dừng (llm_command_node tự hủy goal Nav2, v=0, dừng pallet_mission)
  llm_set_location {loc}  chỉnh lại vị trí robot mà LLM đang nhớ
  llm_get_status       yêu cầu phát lại trạng thái gần nhất (khi F5/mở trang)
"""
import json

from std_msgs.msg import Bool, String

LOCATIONS = ("home", "p1", "p2", "d1", "unknown")
MAX_TEXT_LEN = 300


class LLMBridge:
    def __init__(self, node, broadcast, precheck=None):
        """
        node:      node ROS 2 đang chạy (ví dụ bridge_node) - dùng để tạo pub/sub
        broadcast: hàm đồng bộ broadcast(dict) gửi tới mọi WebSocket client.
                   Nó được gọi từ thread của rclpy, nên phải an toàn đa luồng
                   (giống cách bridge_node đang đẩy nav_state / camera cho web).
        precheck:  (tùy chọn) hàm không tham số, trả về chuỗi lỗi nếu KHÔNG được phép
                   chạy lệnh LLM lúc này (vd: chưa bật Nav2, đang MANUAL), hoặc None nếu ổn.
                   Đây là lớp kiểm tra phía server, độc lập với kiểm tra ở trình duyệt.
        """
        self._broadcast = broadcast
        self._precheck = precheck
        self.last_status = None
        self._pub_cmd = node.create_publisher(String, "/llm/command", 10)
        self._pub_confirm = node.create_publisher(Bool, "/llm/confirm", 10)
        self._pub_loc = node.create_publisher(String, "/llm/set_location", 10)
        node.create_subscription(String, "/llm/status", self._on_status, 10)

    # ------------------------------------------------------------ ROS -> web
    def _on_status(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception:  # noqa: BLE001
            data = {"state": "info", "msg": msg.data}
        self.last_status = data
        self._broadcast({"type": "llm_status", "data": data})

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
        """Nhận dict lệnh từ WebSocket, trả về dict command_response."""
        name = cmd.get("command")
        try:
            if name == "llm_command":
                text = str(cmd.get("text", "")).strip()[:MAX_TEXT_LEN]
                if not text:
                    return self._resp(name, "error", "Câu lệnh rỗng")
                problem = self._precheck() if self._precheck else None
                if problem:
                    return self._resp(name, "error", problem)
                self._pub_cmd.publish(String(data=text))
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