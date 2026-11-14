#!/usr/bin/env python3
"""llm_logic - logic thuần của LLM command (KHÔNG import ROS).
"""
import json
import math
import os
import re

MISSIONS = ["home_to_p1", "home_to_p2", "p1_to_d1", "p2_to_d1",
            "p1_to_p2", "d1_to_p1", "d1_to_p2", "return_home"]

LIFT_LEVELS = {"low": 2.0, "carry": 12.0, "rack_1": 30.0, "rack_2": 55.0}
UNMEASURED_LIFT_LEVELS = ("rack_1", "rack_2")
LIFT_LIMIT_CM = 80.0

MANUAL_ONLY = {"quick_goal", "go_mission", "pick_pallet", "drop_pallet",
               "detect_pallet", "lock_detection", "lift_to", "lift_home",
               "relative_move"}
CYCLE_ACTIONS = {"start_auto_cycle", "start_single_cycle", "stop_cycle", "resume_cycle"}

WORLD = """\
- HOME: điểm xuất phát / đỗ xe.
- P_1, P_2: các điểm lấy pallet (pallet đặt trên sàn).
- D_1: điểm trả pallet, tại kệ.
- Kệ có 2 tầng; "đặt lên kệ" dùng drop_pallet, KHÔNG dùng lift_to rack_1/rack_2."""

GROUPS = {
    "A": "NHÓM A - CHẾ ĐỘ NAVIGATION (2 nút AUTO / MANUAL trên web). Chỉ đổi chế độ giao diện, KHÔNG chạy mission:",
    "B": "NHÓM B - MISSION / CHU TRÌNH PALLET TỰ ĐỘNG (nút START AUTO MISSION / CANCEL MISSION / RESUME RECOVERY):",
    "C": "NHÓM C - THAO TÁC MANUAL (Quick Goals, lấy/đặt pallet, lift và di chuyển tương đối):",
    "D": "NHÓM D - HỦY GOAL ĐIỀU HƯỚNG:",
}

TOOLS = {
    "set_mode": {
        "group": "A",
        "desc": "CHỈ đổi chế độ Navigation (AUTO / MANUAL) trên giao diện. "
                "BẮT BUỘC dùng tool này khi câu lệnh chỉ muốn đổi chế độ như: 'bật auto', 'bật chế độ auto', "
                "'mode auto', 'chế độ auto', 'chuyển auto', 'auto', 'manual', 'chuyển manual', 'bật tay', "
                "'turn on auto', 'auto mode', 'switch to manual'. "
                "TUYỆT ĐỐI DÙNG TOOL NÀY NẾU CÂU LỆNH KHÔNG CÓ TỪ 'mission' HOẶC 'chu trình' HOẶC 'cycle'.",
        "params": {"mode": ["AUTO", "MANUAL"]},
    },
    "start_auto_cycle": {
        "group": "B",
        "desc": "START AUTO MISSION = Kích hoạt chu trình làm việc pallet tự động (lặp lại). "
                "BẮT BUỘC câu lệnh PHẢI CHỨA TỪ 'mission' HOẶC 'chu trình' "
                "(ví dụ: 'start mission', 'start auto mission', 'chạy mission', 'bật mission', 'chạy chu trình auto'). "
                "CẤM DÙNG tool này khi người dùng chỉ nói 'bật auto', 'bật chế độ auto', 'mode auto', 'auto mode' mà KHÔNG có từ 'mission'/'chu trình'. "
                "Phải là bước duy nhất trong kế hoạch.",
        "params": {},
    },
    "start_single_cycle": {
        "group": "B",
        "desc": "Chạy mission pallet có giới hạn số lượng rồi về HOME. Mặc định lấy 1 pallet bất kỳ. "
                "cargo_class=medium nghĩa là chỉ lấy hộp medium; tall nghĩa là chỉ lấy hộp cao; any là loại nào cũng được. "
                "quantity là số pallet phải đặt thành công lên kệ trước khi mission hoàn tất. "
                "Dùng cho 'chạy một chu trình', 'lấy 2 box cao', 'lấy 3 hộp medium rồi đặt lên kệ', 'one cycle'. "
                "Phải là bước duy nhất trong kế hoạch.",
        "params": {"cargo_class": ["any", "medium", "tall"],
                    "drop_slot": ["auto", "A1", "A2", "B1", "B2"],
                    "quantity": {"type": "integer", "min": 1, "max": 10}},
    },
    "stop_cycle": {
        "group": "B",
        "desc": "CANCEL MISSION: dừng chu trình pallet đang chạy. Câu 'cancel mission / hủy / dừng' "
                "thường đã được hệ thống xử lý trước khi tới bạn; nếu vẫn nhận được thì dùng tool này. "
                "Phải là bước duy nhất trong kế hoạch.",
        "params": {},
    },
    "resume_cycle": {
        "group": "B",
        "desc": "RESUME RECOVERY: tiếp tục chu trình từ checkpoint sau khi bị hủy/lỗi. "
                "Người dùng nói: 'resume', 'tiếp tục', 'resume recovery'. "
                "Phải là bước duy nhất trong kế hoạch.",
        "params": {},
    },
    "quick_goal": {
        "group": "C",
        "desc": "Đi thẳng tới MỘT điểm (4 nút Quick Goals: Return Home, Go Pick 1, Go Pick 2, Go Drop), "
                "KHÔNG lấy/đặt pallet. Người dùng nói: 'go pick 1', 'go pick 2', 'go drop', "
                "'return home', 'về home', 'về nhà', 'đi tới P1', 'đi tới D1', 'go to P1'. "
                "Dùng được từ bất kỳ vị trí nào.",
        "params": {"target": ["HOME", "P_1", "P_2", "D_1"]},
    },
    "go_mission": {
        "group": "C",
        "desc": "CHỈ dùng làm bước trong chuỗi lấy/đặt pallet (đi tới điểm lấy rồi pick_pallet; "
                "đi tới D_1 rồi drop_pallet). Tên = <điểm đi>_to_<điểm đến>, điểm đi PHẢI là vị trí "
                "hiện tại của robot. KHÔNG dùng cho lệnh đi tới một điểm đơn thuần (dùng quick_goal). "
                "KHÔNG liên quan tới 'start mission'.",
        "params": {"name": MISSIONS},
    },
    "pick_pallet": {
        "group": "C",
        "desc": "Nhận diện pallet phía trước, căn hướng và tiến càng vào pallet. "
                "Chỉ gọi khi robot đang đứng ở điểm lấy (P_1 hoặc P_2).",
        "params": {},
    },
    "detect_pallet": {
        "group": "C",
        "desc": "Bật nhận diện pallet tại vị trí hiện tại, chờ pose và thu thập mẫu trong vài giây để bước lock_detection có thể chốt. Không căn xe, không tiến càng và không lấy pallet. Detection tiếp tục chạy để người vận hành xem hoặc dùng lệnh pick sau.",
        "params": {},
    },
    "lock_detection": {
        "group": "C",
        "desc": "Dừng detector và chốt pose ổn định từ các mẫu đã thu thập. Chỉ hoạt động khi pallet_mission đang DETECTING. Không di chuyển hoặc lấy pallet; sau khi chốt thành công, pick_pallet có thể dùng pose này mà không detect lại.",
        "params": {},
    },
    "drop_pallet": {
        "group": "C",
        "desc": "Đặt pallet đang giữ lên kệ (robot phải ở D_1 và đang mang pallet). "
                "Tự chọn ô/kệ hợp lệ; KHÔNG chọn độ cao theo hàng.",
        "params": {},
    },
    "lift_to": {
        "group": "C",
        "desc": "Nâng/hạ càng tới chiều cao người dùng yêu cầu. Dùng height_cm cho số cm cụ thể (0–80 cm), ví dụ 15 cm -> height_cm=15. Dùng level=low/carry khi người dùng nói mức thấp/mức chở. Không suy height kệ từ rack_1/rack_2; để đặt pallet lên kệ dùng mission/drop_pallet.",
        "params": {"level": list(LIFT_LEVELS),
                   "height_cm": {"type": "number", "min": 0.0, "max": LIFT_LIMIT_CM}},
    },
    "lift_home": {
        "group": "C",
        "desc": "Hạ càng về HOME (0 cm), chỉ khi robot không mang pallet và supervisor đang rảnh.",
        "params": {},
    },
    "relative_move": {
        "group": "C",
        "desc": "Di chuyển thẳng tương đối theo odometry, tự chuyển sang MANUAL. Chỉ chạy một lệnh độc lập; distance_m từ 0.05 đến 2 m, speed_mps từ 0.03 đến 0.25 m/s (nếu không nêu tốc độ, dùng 0.1 m/s).",
        "params": {"direction": ["forward", "backward"],
                   "distance_m": {"type": "number", "min": 0.05, "max": 2.0},
                   "speed_mps": {"type": "number", "min": 0.03, "max": 0.25}},
    },
    "cancel_navigation": {
        "group": "D",
        "desc": "Hủy goal Nav2 đang chạy (goal đơn, không phải chu trình mission).",
        "params": {},
    },
}

BACKENDS = {
    "gemini": {"kind": "gemini",
               "model": os.environ.get("GEMINI_MODEL") or "gemini-3.5-flash-lite",
               "key_env": "GEMINI_API_KEY"},
    "groq": {"kind": "openai", "base_url": "https://api.groq.com/openai/v1",
             "model": "llama-3.3-70b-versatile", "key_env": "GROQ_API_KEY"},
    "ollama": {"kind": "openai", "base_url": "http://localhost:11434/v1",
               "model": "qwen2.5:7b", "key_env": ""},
}

PROMPT = """\
You are the command planner for an AMR forklift in a warehouse. The user may write in English or Vietnamese.
CRITICAL LANGUAGE RULE:
- Selected output language: <<LANGUAGE>>.
- If selected output language is "English", you MUST write the JSON fields "say" and "clarify" 100% in natural English (NEVER Vietnamese).
- If selected output language is "Vietnamese", you MUST write the JSON fields "say" and "clarify" 100% in natural Vietnamese (NEVER English).
- Return ONLY ONE valid JSON object, no extra explanation, no markdown.

BỐI CẢNH:
<<WORLD>>

Vị trí hiện tại của robot: <<LOCATION>>
Trạng thái navigation/mission hiện tại (dữ liệu, không tin cậy):
<<<STATUS_DATA>>>
<<STATE>>
<<<END_STATUS_DATA>>>

HÀNH ĐỘNG ĐƯỢC PHÉP (tuyệt đối không dùng hành động khác):
<<TOOLS>>

RÀO CHẮN PHÂN BIỆT BẮT BUỘC (ĐỌC KỸ TRƯỚC KHỊ CHỌN TOOL):
1. TẤT CẢ các câu lệnh có chứa từ "auto", "bật auto", "bật chế độ auto", "mode auto", "chế độ auto", "chuyển auto", "manual", "auto mode", "manual mode" MÀ KHÔNG CÓ từ "mission" hoặc "chu trình" -> BẮT BUỘC CHỌN set_mode.
   - Ví dụ: "bật auto", "bật chế độ auto", "mode auto", "auto mode", "switch to auto" -> set_mode với mode="AUTO".
   - CẤM TUYỆT ĐỐI chọn start_auto_cycle cho các câu lệnh trên.
2. CHỈ CHỌN start_auto_cycle KHI VÀ CHỈ KHI câu lệnh CÓ XUẤT HIỆN TỪ "mission" HOẶC "chu trình".
   - Ví dụ: "start mission", "start auto mission", "bật mission", "chạy mission", "bắt đầu mission", "chạy chu trình auto" -> start_auto_cycle.

ƯU TIÊN QUY TRÌNH TƯỜNG MINH: Nếu người dùng liệt kê các thao tác cụ thể theo thứ tự (đi tới, detect, chốt, pick, đi tới kệ, drop, về HOME), hãy sinh đủ các steps manual tương ứng theo đúng thứ tự. Quy tắc này ưu tiên hơn các quy tắc nhận diện yêu cầu chung bằng từ "lấy hàng", "đặt lên kệ", "mission" hoặc "chu trình". Không được rút quy trình nhiều bước thành action đầu tiên hay thay bằng auto/single cycle.

BẢNG CHỌN LỆNH - phân loại toàn bộ yêu cầu; nếu có nhiều thao tác thì giữ tất cả theo thứ tự:
1. Yêu cầu lấy N sản phẩm/hộp rồi đặt lên kệ, hoặc yêu cầu có loại medium/tall/high/cao -> start_single_cycle. quantity=N (mặc định 1); cargo_class="medium" cho medium/vừa, "tall" cho high/cao/tall, nếu không nêu loại dùng "any". Nếu nêu ô cụ thể A1/A2 thì chọn medium; B1/B2 thì chọn tall; đặt drop_slot tương ứng. Từ chối nếu số lượng >1 với ô cố định. Mission chỉ báo hoàn tất sau khi đủ số lượng đã đặt thành công lên kệ.
2. Câu chỉ nói "one cycle / một chu trình / một lần / single" mà không yêu cầu nhiều sản phẩm -> start_single_cycle với quantity=1, cargo_class="any".
3. Câu có từ "mission" hoặc "chu trình" (start mission, start auto mission, bật mission, chạy mission, bắt đầu mission, chạy chu trình auto) -> start_auto_cycle.
4. Câu có "resume / tiếp tục" -> resume_cycle.
5. Câu chỉ nói về chế độ/mode, KHÔNG CÓ từ "mission" hay "chu trình" (bật auto, bật chế độ auto, mode auto, chế độ auto, start auto, auto, chuyển sang auto, manual, chuyển manual, bật tay, auto mode, manual mode) -> set_mode.
6. Câu mơ hồ như "start", "chạy", "run" (không rõ đổi chế độ hay chạy mission) -> steps rỗng và hỏi lại ở "clarify" theo đúng <<LANGUAGE>>:
   - Nếu English: "Do you want to Start Mission or switch Navigation to AUTO?"
   - Nếu Vietnamese: "Bạn muốn Start Mission hay chuyển Navigation sang AUTO?"
PHÂN RÃ CÂU LỆNH NHIỀU BƯỚC (BẮT BUỘC):
- Không dừng ở ý định đầu tiên. Tạo một step cho MỖI thao tác người dùng yêu cầu, giữ nguyên thứ tự; giữ các thao tác độc lập thành steps riêng.
- Khi người dùng nêu quy trình manual rõ ràng như đi tới Pick 1, detect, dừng/chốt detect, lấy pallet, đi tới kệ, đặt pallet, về HOME: dùng lần lượt quick_goal(P_1), detect_pallet, lock_detection, pick_pallet, go_mission(p1_to_d1), drop_pallet, go_mission(return_home). Không thay quy trình rõ ràng này bằng start_single_cycle.
- Với các câu lệnh ở nhiều lượt: nếu vị trí hiện tại đã là P_1/P_2 và người dùng chỉ nói lấy/pick, gọi pick_pallet trực tiếp để dùng pose READY đã chốt; không điều hướng hoặc detect lại. Nếu vị trí là P_1/P_2 và người dùng nói đem pallet tới kệ, đặt xuống rồi về HOME, tạo đủ go_mission(p1_to_d1/p2_to_d1), drop_pallet, go_mission(return_home). Nếu đã ở D_1 và đang mang pallet, chỉ cần drop_pallet rồi return_home theo yêu cầu.
- Nếu người dùng chỉ nói lấy pallet và đặt lên kệ mà không yêu cầu các thao tác riêng, có thể dùng start_single_cycle.

7. Chỉ nhận diện/detect pallet, KHÔNG lấy -> detect_pallet. Nếu yêu cầu vừa đi tới Pick 1/2 vừa detect thì dùng quick_goal tới P_1/P_2 rồi detect_pallet; tuyệt đối không dùng pick_pallet nếu người dùng không bảo lấy/nâng pallet.
8. Dừng detect/chốt mẫu -> lock_detection; nếu người dùng đồng thời yêu cầu các bước tiếp theo, thêm tiếp các action tương ứng.
9. Đi tới MỘT điểm đơn thuần (go pick 1, go pick 2, go drop, return home, về home, về nhà, đi tới P1/P2/D1, go to P1/P2/D1) -> quick_goal.
10. Thao tác pallet tay khi người dùng nói rõ muốn lấy/pick hoặc đặt/drop -> chỉ thêm go_mission nếu robot chưa ở điểm cần thiết; tại P_1/P_2 dùng pick_pallet trực tiếp, tại D_1 dùng drop_pallet trực tiếp. Giữ nguyên các bước đi/chốt/return được nêu. Detect đơn thuần không phải pick.
11. Hạ càng về HOME/0 -> lift_home. Nếu người dùng nói nâng/hạ mà không nêu mức hoặc chiều cao thì hỏi lại; không tự đoán độ cao. Yêu cầu chiều cao số cụ thể như 15 cm -> lift_to height_cm=15. Mọi height_cm phải trong [0, 80]; ngoài giới hạn thì hỏi lại. Nếu người dùng nêu rõ preset low/carry thì dùng level tương ứng; carry=12 cm chỉ là preset phần mềm hiện có, chưa được xác nhận hiệu chuẩn phần cứng.
12. Đi tiến/lùi một khoảng theo mét (ví dụ "tiến 1m tốc độ 0.1m/s") -> relative_move, direction forward/backward và dùng đúng distance_m, speed_mps người dùng nêu; nếu không nêu tốc độ thì dùng 0.1 m/s. Nếu yêu cầu tốc độ ngoài 0.03–0.25 m/s thì hỏi lại. Đây là lệnh MANUAL độc lập, không tự ghép với nhiệm vụ khác.
13. Hủy goal điều hướng đang chạy (không phải mission, cancel navigation) -> cancel_navigation.
14. Hủy / dừng mission (stop mission, cancel mission) -> stop_cycle.

ĐỊNH DẠNG:
{"say": "short confirmation in <<LANGUAGE>>", "clarify": null or "clarification question in <<LANGUAGE>>",
 "steps": [{"action": "...", "args": {...}}]}

VÍ DỤ SONG NGỮ (BẮT BUỘC TUÂN THỦ NGÔN NGỮ <<LANGUAGE>>):
- Khi <<LANGUAGE>> = Vietnamese:
  User: lấy 2 box high rồi đặt lên kệ
  {"say":"Tôi sẽ lấy và đặt 2 hộp cao lên kệ, sau đó về HOME.","clarify":null,"steps":[{"action":"start_single_cycle","args":{"cargo_class":"tall","quantity":2}}]}
  User: lấy một hộp medium đặt ở A2
  {"say":"Tôi sẽ lấy một hộp medium và đặt vào ô A2.","clarify":null,"steps":[{"action":"start_single_cycle","args":{"cargo_class":"medium","quantity":1,"drop_slot":"A2"}}]}
  User: tiến 1 mét với tốc độ 0.1 mét mỗi giây
  {"say":"Tôi sẽ chuyển sang MANUAL và đi tiến 1 mét với tốc độ 0.1 mét mỗi giây.","clarify":null,"steps":[{"action":"relative_move","args":{"direction":"forward","distance_m":1.0,"speed_mps":0.1}}]}
  User: hạ lift về HOME
  {"say":"Tôi sẽ hạ lift về HOME.","clarify":null,"steps":[{"action":"lift_home","args":{}}]}
  User: đi tới Pick 1 và detect pallet
  {"say":"Tôi sẽ tới P_1 và bật nhận diện pallet, không thực hiện thao tác lấy.","clarify":null,"steps":[{"action":"quick_goal","args":{"target":"P_1"}},{"action":"detect_pallet","args":{}}]}
  User: đi tới Pick 1, detect, chốt mẫu, lấy pallet, đem tới kệ đặt xuống rồi về Home
  {"say":"Tôi sẽ tới P_1, nhận diện và chốt pose pallet, lấy hàng, đem tới kệ đặt xuống rồi về HOME.","clarify":null,"steps":[{"action":"quick_goal","args":{"target":"P_1"}},{"action":"detect_pallet","args":{}},{"action":"lock_detection","args":{}},{"action":"pick_pallet","args":{}},{"action":"go_mission","args":{"name":"p1_to_d1"}},{"action":"drop_pallet","args":{}},{"action":"go_mission","args":{"name":"return_home"}}]}
  User: nâng càng lên 15cm
  {"say":"Tôi sẽ nâng càng lên 15 cm.","clarify":null,"steps":[{"action":"lift_to","args":{"height_cm":15}}]}
  User: bật chế độ auto
  {"say":"Tôi sẽ chuyển Navigation sang AUTO.","clarify":null,"steps":[{"action":"set_mode","args":{"mode":"AUTO"}}]}
  User: start mission
  {"say":"Tôi sẽ bắt đầu chu trình tự động.","clarify":null,"steps":[{"action":"start_auto_cycle","args":{}}]}
  User: đi tới P1
  {"say":"Tôi sẽ di chuyển tới P_1.","clarify":null,"steps":[{"action":"quick_goal","args":{"target":"P_1"}}]}

- Khi <<LANGUAGE>> = English:
  User: pick two tall boxes and put them on the shelf
  {"say":"I will pick and place two tall boxes, then return HOME.","clarify":null,"steps":[{"action":"start_single_cycle","args":{"cargo_class":"tall","quantity":2}}]}
  User: pick one medium box and put it in A2
  {"say":"I will pick one medium box and place it in slot A2.","clarify":null,"steps":[{"action":"start_single_cycle","args":{"cargo_class":"medium","quantity":1,"drop_slot":"A2"}}]}
  User: switch to auto mode
  {"say":"I will switch Navigation to AUTO mode.","clarify":null,"steps":[{"action":"set_mode","args":{"mode":"AUTO"}}]}
  User: start mission
  {"say":"I will start the automatic pallet mission cycle.","clarify":null,"steps":[{"action":"start_auto_cycle","args":{}}]}
  User: go to P1
  {"say":"I will navigate to P_1.","clarify":null,"steps":[{"action":"quick_goal","args":{"target":"P_1"}}]}
"""

GOAL_SUCCEEDED = 4
GOAL_CANCELED = 5
GOAL_ABORTED = 6
TERMINAL = (GOAL_SUCCEEDED, GOAL_CANCELED, GOAL_ABORTED)


def build_tools_text():
    lines, last_group = [], None
    for name, spec in TOOLS.items():
        group = spec.get("group")
        if group != last_group and group in GROUPS:
            if lines:
                lines.append("")
            lines.append(GROUPS[group])
            last_group = group
        params = ", ".join(
            f"{k} ∈ {v}" if isinstance(v, list) else
            f"{k} là {v.get('type')} trong [{v.get('min')}, {v.get('max')}]"
            for k, v in spec["params"].items()) or "không có tham số"
        lines.append(f"- {name}: {spec['desc']} ({params})")
    return "\n".join(lines)


def build_system_prompt(location, state=None, language="auto"):
    output_language = {"en": "English", "vi": "Vietnamese"}.get(
        language, "the same language as the user's command")
    return (PROMPT.replace("<<WORLD>>", WORLD)
                  .replace("<<LOCATION>>", location)
                  .replace("<<STATE>>", json.dumps(state or {}, ensure_ascii=False))
                  .replace("<<LANGUAGE>>", output_language)
                  .replace("<<TOOLS>>", build_tools_text()))


def extract_json(raw):
    raw = raw.strip()
    raw = re.sub(r"^```(?:json)?|```$", "", raw, flags=re.M).strip()
    return json.loads(raw)


def mission_from_to(name):
    if name not in MISSIONS:
        raise ValueError(f"Tên mission không hợp lệ: {name!r}")
    if name == "return_home":
        return None, "home"
    a, b = name.split("_to_")
    return a, b


def validate_plan(plan, location, max_steps=8, allow_unmeasured_lift=False):
    if not isinstance(plan, dict):
        raise ValueError("Kế hoạch phải là object JSON")
    steps = plan.get("steps") or []
    if not isinstance(steps, list):
        raise ValueError("steps phải là danh sách")
    if len(steps) > max_steps:
        raise ValueError(f"Kế hoạch quá dài ({len(steps)} > {max_steps} bước)")
    clean, loc = [], location
    if any(isinstance(s, dict) and s.get("action") in CYCLE_ACTIONS for s in steps) and len(steps) != 1:
        raise ValueError("Lệnh cycle phải là hành động duy nhất trong kế hoạch")
    mode_after_steps = None
    for s in steps:
        if not isinstance(s, dict):
            raise ValueError("Mỗi bước phải là object")
        action = s.get("action")
        spec = TOOLS.get(action)
        if spec is None:
            raise ValueError(f"Hành động không được phép: {action}")
        args = s.get("args") or {}
        if not isinstance(args, dict):
            raise ValueError(f"{action}.args phải là object")
        if set(args) - set(spec["params"]):
            raise ValueError(f"Tham số lạ ở {action}: {args}")
        if action == "start_single_cycle":
            args.setdefault("cargo_class", "any")
            args.setdefault("quantity", 1)
            args.setdefault("drop_slot", "auto")
        if action == "relative_move":
            args.setdefault("speed_mps", 0.1)
        for key, constraint in spec["params"].items():
            if isinstance(constraint, list):
                if (action == "lift_to" and key == "level" and
                        "height_cm" in args and key not in args):
                    continue
                if key not in args or args[key] not in constraint:
                    raise ValueError(f"{action}.{key}={args[key]!r} không hợp lệ")
            elif key in args:
                value = args[key]
                if (constraint.get("type") == "integer" and
                        (isinstance(value, bool) or not isinstance(value, int) or
                         not constraint["min"] <= value <= constraint["max"])):
                    raise ValueError(
                        f"{action}.{key} phải là số nguyên từ {constraint['min']} "
                        f"đến {constraint['max']}")
                if (constraint.get("type") == "number" and
                        (isinstance(value, bool) or not isinstance(value, (int, float)) or
                         not math.isfinite(float(value)) or
                         not constraint["min"] <= float(value) <= constraint["max"])):
                    raise ValueError(
                        f"{action}.{key} phải là số từ {constraint['min']} "
                        f"đến {constraint['max']}")
        if action in {"relative_move", "lift_home"} and len(steps) != 1:
            raise ValueError(f"{action} phải là hành động duy nhất trong kế hoạch")
        if action == "lift_to":
            if ("level" in args) == ("height_cm" in args):
                raise ValueError("lift_to cần đúng một trong level hoặc height_cm")
        if action == "start_single_cycle":
            slot, cargo = args["drop_slot"], args["cargo_class"]
            slot_class = "medium" if slot in ("A1", "A2") else "tall" if slot in ("B1", "B2") else None
            if slot_class and cargo not in ("any", slot_class):
                raise ValueError(f"drop_slot {slot} không tương thích cargo_class {cargo}")
            if slot != "auto" and args["quantity"] != 1:
                raise ValueError("Ô kệ chỉ định chỉ chứa được một pallet; quantity phải bằng 1")
            if slot_class and cargo == "any":
                args["cargo_class"] = slot_class
        if action == "set_mode":
            mode_after_steps = args["mode"]
        if action in MANUAL_ONLY and mode_after_steps == "AUTO":
            raise ValueError(f"{action} chỉ chạy ở MANUAL nhưng kế hoạch vừa chuyển sang AUTO")
        if action == "lift_to" and args.get("level") in UNMEASURED_LIFT_LEVELS and not allow_unmeasured_lift:
            raise ValueError(f"lift_to '{args['level']}' bị chặn: mức chưa đo thực tế.")
        if action == "go_mission":
            frm, to = mission_from_to(args["name"])
            if frm is not None and frm != loc:
                raise ValueError(f"Robot đang ở '{loc}', không thể chạy {args['name']}")
            loc = to
        clean.append({"action": action, "args": {
            k: args[k] for k in spec["params"] if k in args}})
    return clean


def is_stop(text):
    t = " ".join(text.lower().split()).strip(" .,!?:;")
    if re.match(r"^(?:dừng|dung|stop|hủy|huỷ|cancel)\s+(?:detect|detection|nhận diện|nhan dien)\b", t):
        return False
    return bool(re.match(
        r"^(?:dừng(?:\s+lại)?|dung(?:\s+lai)?|stop(?:\s+the\s+robot)?|"
        r"emergency\s+stop|khẩn\s+cấp|hủy|huỷ|cancel|thôi)(?=$|[\s,;.!?:])",
        t))


class Aborted(Exception):
    pass
