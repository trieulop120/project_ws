#!/usr/bin/env python3
"""llm_logic - logic thuần của LLM command (KHÔNG import ROS).
"""
import json
import math
import os
import re

try:
    from .robot_queries import TOPICS
except ImportError:
    from robot_queries import TOPICS

MISSIONS = ["home_to_p1", "home_to_d2", "p1_to_d1", "d2_to_d1",
            "d1_to_p1", "d1_to_d2", "p1_to_d2", "return_home"]

LIFT_LEVELS = {"low": 2.0, "carry": 12.0, "rack_1": 30.0, "rack_2": 55.0}
UNMEASURED_LIFT_LEVELS = ("rack_1", "rack_2")
LIFT_LIMIT_CM = 80.0

MANUAL_ONLY = {"quick_goal", "go_mission", "pick_pallet", "drop_pallet",
               "detect_pallet", "lock_detection", "lift_to", "lift_home",
               "relative_move"}
CYCLE_ACTIONS = {"start_auto_cycle", "start_single_cycle", "retrieve_pallet", "stop_cycle", "resume_cycle"}

WORLD = """\
- HOME: điểm xuất phát / đỗ xe.
- P_1: điểm lấy pallet (pallet đặt trên sàn) - điểm lấy duy nhất.
- D_1: điểm trả pallet, tại kệ (lưu kho).
- D_2: điểm thả pallet lấy ra từ kệ (xuất kho) - sau này cơ chế tự chọn loại.
- Kệ có 3 cột, phân loại theo chiều cao hàng:
  + Cột A (dành cho pallet/hộp thấp = medium): có 2 tầng gồm A1 (kệ tầng 1 / tầng dưới) và A2 (kệ tầng 2 / tầng trên).
  + Cột B (dành cho pallet/hộp cao = tall): gồm B1 và B2 (1 tầng).
- "Đặt lên kệ" dùng start_single_cycle hoặc drop_pallet, KHÔNG dùng lift_to rack_1/rack_2."""

GROUPS = {
    "A": "NHÓM A - CHẾ ĐỘ NAVIGATION (2 nút AUTO / MANUAL trên web). Chỉ đổi chế độ giao diện, KHÔNG chạy mission:",
    "B": "NHÓM B - MISSION / CHU TRÌNH PALLET TỰ ĐỘNG (nút START AUTO MISSION / CANCEL MISSION / RESUME RECOVERY):",
    "C": "NHÓM C - THAO TÁC MANUAL (Quick Goals, lấy/đặt pallet, lift và di chuyển tương đối):",
    "E": "NHÓM E - HỎI THÔNG TIN, KHÔNG ĐIỀU KHIỂN XE:",
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
                "quantity là số pallet phải đặt thành công lên kệ trước khi mission hoàn tất (1–10). "
                "Dùng cho 'chạy một chu trình', 'lấy 2 box cao', 'lấy 3 hộp medium rồi đặt lên kệ', 'one cycle'. "
                "Có thể kết hợp nhiều start_single_cycle khác loại/ô trong một kế hoạch; Supervisor "
                "xử lý queue có thứ tự trong cùng một run, chỉ về HOME sau lượt cuối. Không ghép với auto/retrieval/manual.",
        "params": {"cargo_class": ["any", "medium", "tall"],
                    "drop_slot": ["auto", "A1", "A2", "B1", "B2"],
                    "quantity": {"type": "integer", "min": 1, "max": 10}},
    },
    "retrieve_pallet": {
        "group": "B",
        "desc": "Lấy pallet ra khỏi một ô kệ đã chọn và đưa tới D_2, sau đó về HOME. "
                "Chạy retrieval coordinator ở AUTO mode; không dùng cho yêu cầu lấy pallet sàn rồi đặt lên kệ.",
        "params": {"slot": ["A1", "A2", "B1", "B2"]},
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
        "desc": "Đi thẳng tới MỘT điểm (4 nút Quick Goals: Return Home, Go Pick, Go Drop, Go D_2), "
                "KHÔNG lấy/đặt pallet. Người dùng nói: 'go pick', 'go drop', "
                "'return home', 'về home', 'về nhà', 'đi tới P1', 'đi tới D1', 'go to P1', 'go to D2'. "
                "Dùng được từ bất kỳ vị trí nào.",
        "params": {"target": ["HOME", "P_1", "D_1", "D_2"]},
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
                "Chỉ gọi khi robot đang đứng ở điểm lấy pallet (P_1).",
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
        "desc": "Nâng/hạ càng tới chiều cao người dùng yêu cầu, kể cả khi đang mang pallet nếu người dùng yêu cầu rõ. Dùng height_cm cho số cm cụ thể (0–80 cm), ví dụ 15 cm -> height_cm=15. Dùng level=low/carry khi người dùng nói mức thấp/mức chở. Không suy height kệ từ rack_1/rack_2; để đặt pallet lên kệ dùng mission/drop_pallet.",
        "params": {"level": list(LIFT_LEVELS),
                   "height_cm": {"type": "number", "min": 0.0, "max": LIFT_LIMIT_CM}},
    },
    "lift_home": {
        "group": "C",
        "desc": "Hạ càng về HOME (0 cm). Cho phép cả khi robot đang mang pallet nếu người dùng yêu cầu rõ; supervisor/Navigation vẫn phải ở trạng thái phù hợp.",
        "params": {},
    },
    "relative_move": {
        "group": "C",
        "desc": "Di chuyển thẳng tương đối theo odometry, tự chuyển sang MANUAL. Có thể là một bước trong chuỗi MANUAL tuần tự; distance_m từ 0.05 đến 2 m, speed_mps từ 0.03 đến 0.25 m/s (nếu không nêu tốc độ, dùng 0.1 m/s).",
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

TOOLS["get_robot_status"] = {
    "group": "E",
    "desc": "Trả lời thông tin về bản thân trợ lý (identity), model LLM đang cấu hình (llm_model), quy trình xử lý lệnh (llm_workflow), đồ án AMR (project), sinh viên thực hiện (creators), giảng viên hướng dẫn (advisor), công nghệ (technology); hoặc đọc telemetry về tồn kho, mission, cargo, lift, thiết bị, khả năng xe. navigation báo trạng thái goal Nav2; motion báo đứng yên/di chuyển theo odometry; eta và battery_runtime giải thích giới hạn ước lượng thời gian. clarification hỏi rõ thông tin còn thiếu. Không thực thi hành động; là bước duy nhất trong kế hoạch.",
    "params": {"topic": list(TOPICS)},
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

ƯU TIÊN CÂU HỎI THÔNG TIN:
- Câu hỏi về bản thân trợ lý, tên, model LLM, cách xử lý lệnh, tác giả đồ án, giảng viên hướng dẫn, mục tiêu hoặc công nghệ xe -> get_robot_status với topic identity/llm_model/llm_workflow/project/creators/advisor/technology tương ứng. Hệ thống có câu trả lời trực tiếp; không tự bịa tên, trường, thành tích, tên model nền hoặc người tạo model nền. Chỉ hỏi về đồ án không phải yêu cầu chạy xe.
- Nếu hỏi số hàng/ô trống, vị trí ô, cargo, tiến độ, lỗi, resume có sẵn hay khả năng xe: chỉ dùng get_robot_status. Câu hỏi nhắc AUTO/mission không phải yêu cầu đổi mode/chạy mission.
- Không bịa tồn kho, cargo hoặc trạng thái từ dữ liệu thiếu/cũ. Unknown không phải empty. Số ô có hàng không cho biết số sản phẩm trong từng hộp; loại A/B là loại ô cấu hình.
- Trả lời ngắn gọn, đúng các ý người dùng hỏi. Hỏi mức pin dùng battery, chỉ báo phần trăm pin; không kèm tình trạng camera/kệ/route. Hỏi thiết bị dùng health; chỉ báo tổng quan khi người dùng yêu cầu tổng quan.
- Hỏi Nav2/goal dùng navigation; hỏi xe đứng yên/di chuyển dùng motion. Goal đang hoạt động không chứng minh xe đang di chuyển hoặc toàn bộ Nav2 lifecycle đã sẵn sàng. Chỉ kết luận chuyển động khi odometry còn mới.
- Hỏi còn bao lâu xong mission dùng eta; hỏi pin chạy thêm bao lâu dùng battery_runtime. Hệ thống chưa có ước lượng thời gian đáng tin cậy; không tự suy từ phần trăm tiến độ hoặc pin. Câu hỏi thiếu đối tượng dùng clarification.
- Hỏi thông tin không cần xác nhận và không ngắt mission đang chạy. Không ghép get_robot_status với thao tác cơ khí; yêu cầu vừa hỏi vừa điều khiển thì clarify để tách yêu cầu, không âm thầm bỏ một phần.
- Resume retrieval hoặc chuỗi MANUAL chỉ dùng checkpoint hợp lệ do hệ thống báo; không tự lập lại thao tác đã xong. Không hứa chạy song song hai mission vật lý.

RÀO CHẮN PHÂN BIỆT BẮT BUỘC (ĐỌC KỸ TRƯỚC KHỊ CHỌN TOOL):
1. TẤT CẢ các câu lệnh có chứa từ "auto", "bật auto", "bật chế độ auto", "mode auto", "chế độ auto", "chuyển auto", "manual", "auto mode", "manual mode" MÀ KHÔNG CÓ từ "mission" hoặc "chu trình" -> BẮT BUỘC CHỌN set_mode.
   - Ví dụ: "bật auto", "bật chế độ auto", "mode auto", "auto mode", "switch to auto" -> set_mode với mode="AUTO".
   - CẤM TUYỆT ĐỐI chọn start_auto_cycle cho các câu lệnh trên.
2. CHỈ CHỌN start_auto_cycle KHI VÀ CHỈ KHI câu lệnh CÓ XUẤT HIỆN TỪ "mission" HOẶC "chu trình".
   - Ví dụ: "start mission", "start auto mission", "bật mission", "chạy mission", "bắt đầu mission", "chạy chu trình auto" -> start_auto_cycle.

ƯU TIÊN QUY TRÌNH TƯỜNG MINH: Nếu người dùng liệt kê các thao tác cụ thể theo thứ tự (đi tới, detect, chốt, pick, đi tới kệ, drop, về HOME), hãy sinh đủ các steps manual tương ứng theo đúng thứ tự. Quy tắc này ưu tiên hơn các quy tắc nhận diện yêu cầu chung bằng từ "lấy hàng", "đặt lên kệ", "mission" hoặc "chu trình". Không được rút quy trình nhiều bước thành action đầu tiên hay thay bằng auto/single cycle.

BẢNG CHỌN LỆNH - phân loại toàn bộ yêu cầu; nếu có nhiều thao tác thì giữ tất cả theo thứ tự:
1. Yêu cầu lấy pallet ra khỏi ô kệ, xuất kho, lấy hàng ở A1/A2/B1/B2 hoặc đưa về D_2 -> retrieve_pallet với slot tương ứng. Đây là retrieval ở AUTO, không phải lấy pallet sàn rồi đặt lên kệ.
2. Yêu cầu lấy N sản phẩm/hộp/pallet cùng loại rồi đặt lên kệ -> start_single_cycle, quantity=N (1–10; mặc định 1). cargo_class="medium" cho thấp/low/medium/vừa, "tall" cho high/cao/tall, nếu không nêu loại dùng "any". A1/A2 dành cho medium; B1/B2 dành cho tall. Ô cố định chỉ chứa một pallet nên quantity=1. Nếu yêu cầu nhiều loại/ô khác nhau, tạo một start_single_cycle cho MỖI lượt theo đúng thứ tự, ví dụ medium/A2 rồi tall/B2 là HAI bước quantity=1. Supervisor nhận các bước thành một queue trong cùng run; sau mỗi DROP đã xác nhận, cargo=false và Nav2 idle mới đi D_1→P_1 cho lượt kế. Chỉ lượt cuối mới RETURN_HOME; lỗi/Stop thì dừng queue.
   - Ví dụ “lấy 3 medium đặt lên kệ” -> một bước quantity=3. “medium lên A2 rồi tall lên B2” -> hai bước quantity=1: medium/A2, sau đó tall/B2.
2. Câu chỉ nói "one cycle / một chu trình / một lần / single" mà không yêu cầu nhiều sản phẩm -> start_single_cycle với quantity=1, cargo_class="any".
3. Câu có từ "mission" hoặc "chu trình" (start mission, start auto mission, bật mission, chạy mission, bắt đầu mission, chạy chu trình auto) -> start_auto_cycle.
4. Câu có "resume / tiếp tục" -> resume_cycle.
5. Câu chỉ nói về chế độ/mode, KHÔNG CÓ từ "mission" hay "chu trình" (bật auto, bật chế độ auto, mode auto, chế độ auto, start auto, auto, chuyển sang auto, manual, chuyển manual, bật tay, auto mode, manual mode) -> set_mode.
6. Câu mơ hồ như "start", "chạy", "run" (không rõ đổi chế độ hay chạy mission) -> steps rỗng và hỏi lại ở "clarify" theo đúng <<LANGUAGE>>:
   - Nếu English: "Do you want to Start Mission or switch Navigation to AUTO?"
   - Nếu Vietnamese: "Bạn muốn Start Mission hay chuyển Navigation sang AUTO?"
PHÂN RÃ CÂU LỆNH NHIỀU BƯỚC (BẮT BUỘC):
- Không dừng ở ý định đầu tiên. Tạo một step cho MỖI thao tác người dùng yêu cầu, giữ nguyên thứ tự; giữ các thao tác độc lập thành steps riêng. Nhiều start_single_cycle được gom thành một queue tuần tự của Supervisor, không khởi động nhiều mission riêng.
- Khi người dùng nêu quy trình manual rõ ràng như đi tới Pick, detect, dừng/chốt detect, lấy pallet, đi tới kệ, đặt pallet, về HOME: dùng lần lượt quick_goal(P_1), detect_pallet, lock_detection, pick_pallet, go_mission(p1_to_d1), drop_pallet, go_mission(return_home). Không thay quy trình rõ ràng này bằng start_single_cycle.
- Với các câu lệnh ở nhiều lượt: nếu vị trí hiện tại đã là P_1 và người dùng chỉ nói lấy/pick, gọi pick_pallet trực tiếp để dùng pose READY đã chốt; không điều hướng hoặc detect lại. Nếu vị trí là P_1 và người dùng nói đem pallet tới kệ, đặt xuống rồi về HOME, tạo đủ go_mission(p1_to_d1), drop_pallet, go_mission(return_home). Nếu đã ở D_1 và đang mang pallet, chỉ cần drop_pallet rồi return_home theo yêu cầu.
- Nếu người dùng chỉ nói lấy pallet và đặt lên kệ mà không yêu cầu các thao tác riêng, có thể dùng start_single_cycle.

7. Chỉ nhận diện/detect pallet, KHÔNG lấy -> detect_pallet. Nếu yêu cầu vừa đi tới Pick vừa detect thì dùng quick_goal tới P_1 rồi detect_pallet; tuyệt đối không dùng pick_pallet nếu người dùng không bảo lấy/nâng pallet.
8. Dừng detect/chốt mẫu -> lock_detection; nếu người dùng đồng thời yêu cầu các bước tiếp theo, thêm tiếp các action tương ứng.
9. Đi tới MỘT điểm đơn thuần (go pick, go drop, go D_2, return home, về home, về nhà, đi tới P1/D1/D2, go to P1/D1/D2) -> quick_goal.
10. Thao tác pallet tay khi người dùng nói rõ muốn lấy/pick hoặc đặt/drop -> chỉ thêm go_mission nếu robot chưa ở điểm cần thiết; tại P_1 dùng pick_pallet trực tiếp, tại D_1 dùng drop_pallet trực tiếp. Giữ nguyên các bước đi/chốt/return được nêu. Detect đơn thuần không phải pick.
11. Hạ càng về HOME/0 -> lift_home. Nếu người dùng nói nâng/hạ mà không nêu mức hoặc chiều cao thì hỏi lại; không tự đoán độ cao. Yêu cầu chiều cao số cụ thể như 15 cm -> lift_to height_cm=15. Mọi height_cm phải trong [0, 80]; ngoài giới hạn thì hỏi lại. Nếu người dùng nêu rõ preset low/carry thì dùng level tương ứng; carry=12 cm chỉ là preset phần mềm hiện có, chưa được xác nhận hiệu chuẩn phần cứng.
12. Đi tiến/lùi một khoảng theo mét (ví dụ "tiến 1m tốc độ 0.1m/s") -> relative_move, direction forward/backward và dùng đúng distance_m, speed_mps người dùng nêu; nếu không nêu tốc độ thì dùng 0.1 m/s. Nếu yêu cầu tốc độ ngoài 0.03–0.25 m/s thì hỏi lại. Có thể ghép theo thứ tự với lift_home/lift_to và quick_goal HOME; đổi 50 cm thành distance_m=0.5. Mỗi bước phải hoàn tất mới chạy bước tiếp theo. Sau relative_move, vị trí node được coi là unknown; dùng quick_goal cho điểm tiếp theo, không đoán tên leg graph.
13. Hủy goal điều hướng đang chạy (không phải mission, cancel navigation) -> cancel_navigation.
14. Hủy / dừng mission (stop mission, cancel mission) -> stop_cycle.

ĐỊNH DẠNG:
{"say": "short confirmation in <<LANGUAGE>>", "clarify": null or "clarification question in <<LANGUAGE>>",
 "steps": [{"action": "...", "args": {...}}]}

VÍ DỤ SONG NGỮ (BẮT BUỘC TUÂN THỦ NGÔN NGỮ <<LANGUAGE>>):
- Khi <<LANGUAGE>> = Vietnamese:
  User: lấy pallet ở A2 đưa về D_2
  {"say":"Tôi sẽ chuyển sang AUTO, lấy pallet ở ô A2 đưa tới D_2 rồi về HOME.","clarify":null,"steps":[{"action":"retrieve_pallet","args":{"slot":"A2"}}]}
  User: lấy 2 box high rồi đặt lên kệ
  {"say":"Tôi sẽ lấy và đặt 2 hộp cao lên kệ, sau đó về HOME.","clarify":null,"steps":[{"action":"start_single_cycle","args":{"cargo_class":"tall","quantity":2}}]}
  User: lấy một hộp medium đặt ở A2
  {"say":"Tôi sẽ lấy một hộp medium và đặt vào ô A2.","clarify":null,"steps":[{"action":"start_single_cycle","args":{"cargo_class":"medium","quantity":1,"drop_slot":"A2"}}]}
  User: lấy pallet thấp và đưa lên kệ tầng 2 dù kệ tầng 1 trống
  {"say":"Tôi sẽ lấy một pallet thấp (medium) và đặt lên kệ tầng 2 (ô A2).","clarify":null,"steps":[{"action":"start_single_cycle","args":{"cargo_class":"medium","quantity":1,"drop_slot":"A2"}}]}
  User: tiến 1 mét với tốc độ 0.1 mét mỗi giây
  {"say":"Tôi sẽ chuyển sang MANUAL và đi tiến 1 mét với tốc độ 0.1 mét mỗi giây.","clarify":null,"steps":[{"action":"relative_move","args":{"direction":"forward","distance_m":1.0,"speed_mps":0.1}}]}
  User: hạ lift về HOME
  {"say":"Tôi sẽ hạ lift về HOME.","clarify":null,"steps":[{"action":"lift_home","args":{}}]}
  User: đi tới Pick 1 và detect pallet
  {"say":"Tôi sẽ tới P_1 và bật nhận diện pallet, không thực hiện thao tác lấy.","clarify":null,"steps":[{"action":"quick_goal","args":{"target":"P_1"}},{"action":"detect_pallet","args":{}}]}
  User: đi tới Pick 1, detect, chốt mẫu, lấy pallet, đem tới kệ đặt xuống rồi về Home
  {"say":"Tôi sẽ tới P_1, nhận diện và chốt pose pallet, lấy hàng, đem tới kệ đặt xuống rồi về HOME.","clarify":null,"steps":[{"action":"quick_goal","args":{"target":"P_1"}},{"action":"detect_pallet","args":{}},{"action":"lock_detection","args":{}},{"action":"pick_pallet","args":{}},{"action":"go_mission","args":{"name":"p1_to_d1"}},{"action":"drop_pallet","args":{}},{"action":"go_mission","args":{"name":"return_home"}}]}
  User: hạ càng về HOME, lùi 50cm sau đó về home
  {"say":"Tôi sẽ hạ càng về 0 cm, lùi 0,5 m rồi đi HOME, thực hiện lần lượt.","clarify":null,"steps":[{"action":"lift_home","args":{}},{"action":"relative_move","args":{"direction":"backward","distance_m":0.5,"speed_mps":0.1}},{"action":"quick_goal","args":{"target":"HOME"}}]}
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
    cycle_steps = [s for s in steps if isinstance(s, dict) and s.get("action") in CYCLE_ACTIONS]
    # Every cycle command is valid alone; only start_single_cycle can form an
    # ordered multi-pallet queue. Other combinations must stay isolated.
    is_single_cycle_command = len(steps) == 1 and len(cycle_steps) == 1
    if cycle_steps and not is_single_cycle_command and (len(cycle_steps) != len(steps) or
                        any(s.get("action") != "start_single_cycle" for s in cycle_steps)):
        raise ValueError("Cycle chỉ được ghép với các start_single_cycle tuần tự")
    if any(isinstance(s, dict) and s.get("action") == "get_robot_status" for s in steps) and len(steps) != 1:
        raise ValueError("Câu hỏi trạng thái phải tách khỏi kế hoạch điều khiển")
    mode_after_steps = None
    fixed_cycle_slots = set()
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
                    raise ValueError(f"{action}.{key}={args.get(key)!r} không hợp lệ")
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
        if action == "relative_move" and "distance_m" not in args:
            raise ValueError("relative_move cần distance_m")
        if action == "relative_move":
            loc = "unknown"
        elif action == "quick_goal":
            loc = {"HOME": "home", "P_1": "p1", "D_1": "d1", "D_2": "d2"}[args["target"]]
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
            if slot != "auto" and slot in fixed_cycle_slots:
                raise ValueError(f"Ô {slot} bị yêu cầu nhiều lần trong cùng kế hoạch")
            if slot != "auto":
                fixed_cycle_slots.add(slot)
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
