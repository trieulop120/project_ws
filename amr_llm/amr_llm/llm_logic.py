#!/usr/bin/env python3
"""llm_logic - logic thuan cua LLM command (KHONG import ROS).

Tach ra de test duoc bang `python3 test/test_llm_logic.py` khong can rclpy.
Node ROS (llm_command_node.py) import cac hang so + ham nay.

Giao dien (doc-only) da xac minh tren repo:
  * MISSIONS - khop MISSION_BT_MAP cua mission_manager_node (7 mieu)
  * pallet_mission: state key "state"; done = DONE/CARRYING (after_pick
    'home' -> DONE; 'hold' -> CARRYING); fail = FAILED/STOPPED (state ket
    thuc that cua pallet_mission, khong co ERROR/ABORTED)
  * pallet_mission/drop (Trigger) - can cargo=True + state CARRYING
  * mission_supervisor: /mission_supervisor/status (JSON, khoa "state"),
    RECOVERY = tranh thai ban (che do LLM nut tay)
"""
import json
import re

# 7 ten mieu - ghep MISSION_BT_MAP (mission_manager_node.py)
MISSIONS = ["home_to_p1", "home_to_p2", "p1_to_d1", "p2_to_d1",
            "d1_to_p1", "d1_to_p2", "return_home"]

# Do cao luon (m) - chot theo protocol /esp32_cmd "H{cm}".
LIFT_LEVELS = {"low": 2.0, "carry": 12.0, "rack_1": 30.0, "rack_2": 55.0}
# rack_* chi so GIU CHO (chua do that) - khong duoc nang tu do;
# "dat len ke" dung tool drop_pallet (pallet_mission tu chon tang 25/20
# hoac 60/55 cm theo shelf.yaml).
UNMEASURED_LIFT_LEVELS = ("rack_1", "rack_2")

WORLD = """\
- HOME: diem xuat phat/do xe.
- P_1, P_2: cac diem lay pallet (pallet dat tren san).
- D_1: diem tra pallet, tai ke.
- Ke co 2 tang (rack_1 = tang 1, rack_2 = tang 2); "dat len ke" dung
  hanh dong drop_pallet - KHONG dung lift_to de di den do cao tang ke."""

TOOLS = {
    "go_mission": {
        "desc": "Chay robot theo tuyen co san giua 2 diem. Ten = <diem di>_to_<diem den>.",
        "params": {"name": MISSIONS},
    },
    "pick_pallet": {
        "desc": "Nhan dien pallet phia truoc, can huong va tien cang vao pallet. "
                "Chi goi khi robot dang dung o diem lay (P_1 hoac P_2).",
        "params": {},
    },
    "drop_pallet": {
        "desc": "Dat pallet dang giu len ke (robot phai dang o D_1, xe dang mang "
                "pallet sau pick_pallet). Tu dong chon o/ke hop le va dat vao; "
                "KHONG dat do cao can theo hang.",
        "params": {},
    },
    "lift_to": {
        "desc": "Nang/ha cang toi muc cho truoc. low = sat san (de luan vao pallet), "
                "carry = nang len de cho. rack_1/rack_2 CHUA do that, duong cong "
                "thuc te se chan; de dat len ke dung drop_pallet.",
        "params": {"level": list(LIFT_LEVELS)},
    },
}

# Lenh dung duoc xu ly truoc LLM (khong do tre mang)
STOP_WORDS = ["dừng", "dung lai", "stop", "khẩn cấp", "hủy", "huỷ", "thôi"]

# Backend LLM. Kiểm tra tên model còn hiệu lực trên trang của nhà cung cấp.
BACKENDS = {
    "gemini": {"kind": "gemini", "model": "gemini-3.5-flash-lite",
               "key_env": "GEMINI_API_KEY"},
    "groq": {"kind": "openai", "base_url": "https://api.groq.com/openai/v1",
             "model": "llama-3.3-70b-versatile", "key_env": "GROQ_API_KEY"},
    "ollama": {"kind": "openai", "base_url": "http://localhost:11434/v1",
               "model": "qwen2.5:7b", "key_env": ""},
}

PROMPT = """\
Bạn là bộ lập kế hoạch cho robot xe nâng (AMR) trong kho. Người dùng nói tiếng Việt.
Chỉ trả về MỘT đối tượng JSON hợp lệ, không giải thích thêm, không markdown.

BỐI CẢNH:
<<WORLD>>

Vị trí hiện tại của robot: <<LOCATION>>

HÀNH ĐỘNG ĐƯỢC PHÉP (tuyệt đối không dùng hành động khác):
<<TOOLS>>

QUY TẮC:
- Mỗi go_mission phải bắt đầu từ đúng vị trí hiện tại của robot, rồi cập nhật vị trí sau khi chạy.
- Trước khi pick_pallet, càng nên ở mức low. Sau khi nhận pallet, nâng lên carry trước khi đi.
- Đưa pallet lên kệ: đến D_1 (go_mission) RỒI gọi drop_pallet làm bước cuối -
  KHÔNG dùng lift_to rack_1/rack_2 (mức chưa đo, sẽ bị chặn).
- Nếu câu lệnh thiếu thông tin hoặc mơ hồ (không rõ P_1/P_2, tầng nào...), để steps rỗng
  và hỏi lại ở trường "clarify".
- Nếu yêu cầu không làm được bằng các hành động trên, steps rỗng và giải thích ở "say".

ĐỊNH DẠNG:
{"say": "câu xác nhận ngắn", "clarify": null hoặc "câu hỏi lại",
 "steps": [{"action": "...", "args": {...}}]}

VÍ DỤ (robot đang ở home):
Người dùng: đi lấy pallet ở P1 rồi đưa lên kệ
{"say":"Tôi sẽ lấy pallet ở P1 và đưa lên kệ.","clarify":null,"steps":[
{"action":"lift_to","args":{"level":"low"}},
{"action":"go_mission","args":{"name":"home_to_p1"}},
{"action":"pick_pallet","args":{}},
{"action":"lift_to","args":{"level":"carry"}},
{"action":"go_mission","args":{"name":"p1_to_d1"}},
{"action":"drop_pallet","args":{}}]}
Người dùng: mang pallet đi
{"say":"","clarify":"Bạn muốn lấy pallet ở P_1 hay P_2?","steps":[]}
Người dùng: về nhà đi
{"say":"Tôi sẽ quay về HOME.","clarify":null,"steps":[
{"action":"go_mission","args":{"name":"return_home"}}]}
"""

# State ket thuc (terminal) cua goal Nav2 (action_msgs/GoalStatus) -
# gia tri nguyen ban de module nay khong can ROS.
GOAL_SUCCEEDED = 4
GOAL_CANCELED = 5
GOAL_ABORTED = 6
TERMINAL = (GOAL_SUCCEEDED, GOAL_CANCELED, GOAL_ABORTED)


def build_system_prompt(location):
    lines = []
    for name, spec in TOOLS.items():
        params = ", ".join(f"{k} ∈ {v}" for k, v in spec["params"].items()) \
            or "không có tham số"
        lines.append(f"- {name}: {spec['desc']} ({params})")
    return (PROMPT.replace("<<WORLD>>", WORLD)
                  .replace("<<LOCATION>>", location)
                  .replace("<<TOOLS>>", "\n".join(lines)))


def extract_json(raw):
    """Giai dict JSON tu raw text cua LLM (che markdown fence)."""
    raw = raw.strip()
    raw = re.sub(r"^```(?:json)?|```$", "", raw, flags=re.M).strip()
    return json.loads(raw)


def mission_from_to(name):
    """Phan ten mieu -> (diem xuat phat hoac None, diem den).

    return_home: diem den HOME tu bat ky vi tri nao.
    Nem ValueError neu ten khong co trong MISSIONS."""
    if name not in MISSIONS:
        raise ValueError(f"Tên mission không hợp lệ: {name!r}")
    if name == "return_home":
        return None, "home"
    a, b = name.split("_to_")
    return a, b


def validate_plan(plan, location, max_steps=8,
                 allow_unmeasured_lift=False):
    """Trả về danh sách bước đã làm sạch, hoặc ném ValueError.

    Kiem tra: ten hanh dong trong TOOLS; tham so khop bang; gia tri hop le;
    go_mission phai bat dau tu vi tri hien tai; so buoc <= max_steps;
    va (nu allow_unmeasured_lift=False) chan lift_to rack_1/rack_2 vi
    do cao chua do that.
    """
    steps = plan.get("steps") or []
    if not isinstance(steps, list):
        raise ValueError("steps phải là danh sách")
    if len(steps) > max_steps:
        raise ValueError(f"Kế hoạch quá dài ({len(steps)} > {max_steps} bước)")
    clean, loc = [], location
    for s in steps:
        action = s.get("action")
        spec = TOOLS.get(action)
        if spec is None:
            raise ValueError(f"Hành động không được phép: {action}")
        args = s.get("args") or {}
        if set(args) - set(spec["params"]):
            raise ValueError(f"Tham số lạ ở {action}: {args}")
        for key, allowed in spec["params"].items():
            if args.get(key) not in allowed:
                raise ValueError(f"{action}.{key}={args.get(key)!r} không hợp lệ")
        if action == "lift_to" and args.get("level") in UNMEASURED_LIFT_LEVELS \
                and not allow_unmeasured_lift:
            raise ValueError(
                f"lift_to '{args['level']}' bị chặn: mức chưa đo thực tế. "
                f"Hãy dùng drop_pallet để đặt pallet lên kệ.")
        if action == "go_mission":
            frm, to = mission_from_to(args["name"])
            if frm is not None and frm != loc:
                raise ValueError(
                    f"Robot đang ở '{loc}', không thể chạy {args['name']}")
            loc = to
        clean.append({"action": action,
                      "args": {k: args[k] for k in spec["params"]}})
    return clean


def is_stop(text):
    """Benh nhan lenh dung ngan go (khong di qua LLM)."""
    t = text.lower()
    return len(t) <= 30 and any(w in t for w in STOP_WORDS)


class Aborted(Exception):
    """Dung khan cap - worker throw khi _abort set."""


