"""Read-only robot questions and telemetry context; no ROS or action calls."""
import math
import re
import unicodedata

SLOTS = ('A1', 'A2', 'B1', 'B2')
PROFILE_ANSWERS = {
    'llm_workflow': {
        'vi': 'Tôi nhận câu chữ từ web hoặc văn bản do tính năng giọng nói chuyển thành. Câu hỏi quen thuộc được trả lời trực tiếp từ thông tin đồ án và dữ liệu xe. Với lệnh điều khiển, backend LLM phân tích yêu cầu thành kế hoạch JSON; phần mềm kiểm tra action cho phép, tham số và điều kiện robot, rồi trình kế hoạch để bạn xác nhận. Sau khi xác nhận, ROS thực hiện từng bước, theo dõi kết quả và dừng chuỗi khi lỗi hoặc bị hủy. Bộ điều khiển ROS và phần cứng thực hiện chuyển động; model nhận diện pallet là thành phần thị giác riêng.',
        'en': 'I receive text from the web or transcribed speech. Familiar questions are answered directly from project information and robot telemetry. For control requests, an LLM backend produces a JSON plan; software validates supported actions, parameters and robot conditions, then asks you to confirm. After confirmation, ROS executes each step, monitors results and stops the sequence on failure or cancellation. ROS controllers and hardware perform motion; pallet detection uses a separate vision model.',
    },
    'identity': {
        'vi': 'Tôi là trợ lý AI của đồ án robot AMR, hỗ trợ hỏi thông tin về xe, lập kế hoạch và gửi các lệnh điều khiển được hệ thống cho phép. Tôi là phần mềm trợ lý tích hợp trên xe; các thao tác điều khiển cần được xác nhận trước khi thực hiện.',
        'en': 'I am the AI assistant for this AMR robot project. I help report robot information, plan tasks and send commands supported by the system. I am software integrated with the robot; control actions require confirmation before execution.',
    },
    'project': {
        'vi': 'Đây là đồ án xe robot AMR (Autonomous Mobile Robot, robot di động tự hành) phục vụ vận chuyển pallet trong mô hình kho. Xe kết hợp điều hướng tự động, nhận diện pallet bằng camera RGB-D, cơ cấu nâng hạ càng và giao diện web có trợ lý AI. Các luồng làm việc gồm lấy pallet ở P_1, đặt lên kệ A1/A2/B1/B2, lấy pallet từ kệ đưa tới D_2 và trở về HOME.',
        'en': 'This AMR (Autonomous Mobile Robot) project handles pallet transport in a warehouse model. It combines autonomous navigation, RGB-D pallet detection, a fork lift mechanism and a web interface with an AI assistant. Workflows include picking at P_1, placing in A1/A2/B1/B2, retrieving pallets from the rack to D_2 and returning HOME.',
    },
    'creators': {
        'vi': 'Đồ án xe robot AMR được thực hiện bởi hai sinh viên Trương Ngọc Triều và Phạm Lê Thiên Vũ, dưới sự hướng dẫn của TS. Dương Minh Thiện. Đây là nhóm thực hiện và giảng viên hướng dẫn của đồ án tích hợp trợ lý trên xe.',
        'en': 'The AMR robot project was developed by students Trương Ngọc Triều and Phạm Lê Thiên Vũ, supervised by Dr. Dương Minh Thiện. These are the project team and supervisor responsible for the robot project integrating this assistant.',
    },
    'advisor': {
        'vi': 'Giảng viên hướng dẫn đồ án xe robot AMR là TS. Dương Minh Thiện. Hai sinh viên thực hiện là Trương Ngọc Triều và Phạm Lê Thiên Vũ.',
        'en': 'The AMR robot project is supervised by Dr. Dương Minh Thiện. The student developers are Trương Ngọc Triều and Phạm Lê Thiên Vũ.',
    },
    'technology': {
        'vi': 'Phần mềm xe sử dụng ROS 2, Nav2 cho điều hướng, SLAM/định vị, camera RGB-D để nhận diện và ước lượng pose pallet, cùng ESP32 cho phần cứng điều khiển. Giao diện web kết nối với ROS và trợ lý AI để xem trạng thái, lập kế hoạch và xác nhận lệnh. Đây là mô tả kiến trúc; trạng thái hoạt động hiện tại cần xem telemetry của xe.',
        'en': 'The robot uses ROS 2, Nav2 navigation, SLAM/localization, an RGB-D camera for pallet detection and pose estimation, and ESP32 hardware control. The web interface connects to ROS and the AI assistant for status, planning and command confirmation. This describes the architecture; current availability requires live robot telemetry.',
    },
}
TOPICS = ('inventory', 'mission', 'cargo', 'lift', 'battery', 'health', 'navigation', 'motion',
          'fault', 'resume', 'slot_contents', 'eta', 'battery_runtime', 'clarification', 'capabilities', 'location',
          'mode', 'overview', 'llm_model', *PROFILE_ANSWERS)
ACTIVE_RETRIEVAL = {'HOMING_LIFT', 'NAVIGATING_TO_D1', 'ALIGNING_TO_SLOT',
                    'AUTO_RECOVERING', 'WAITING_FOR_SENSOR',
                    'LOADING_DETECTOR', 'DETECTING', 'PICKING', 'NAVIGATING_TO_D2',
                    'PLACING_AT_D2', 'RETURNING_HOME', 'CANCELING', 'RECOVERY'}


def normalize(text):
    return ''.join(c for c in unicodedata.normalize('NFD', str(text).lower().replace('đ', 'd'))
                   if unicodedata.category(c) != 'Mn')


SLOT = r'\b[ab][12]\b'
SLOT_REF = r'\b(?:o (?:do|nay|ay)|(?:that|this) slot|no|it)\b'
WHY_AGAIN = r'(?:(?:the |vay )?(?:tai sao|vi sao|sao|why)(?: (?:vay|the|lai the|is that))?|nguyen nhan (?:la )?gi)'
RESUME_CHECK = (r'(?:(?:(?:the |vay )?(?:bay gio |gio )?(?:xe )?(?:co the )?'
                r'(?:resume|chay tiep|tiep tuc)(?: mission)? (?:duoc (?:chua|khong)|chua)(?: vay| a| nhi)?)'
                r'|(?:can (?:we|it|the robot)|is it possible to) (?:resume|continue)(?: now)?)')


def mentioned_slots(text):
    return tuple(dict.fromkeys(slot.upper() for slot in re.findall(SLOT, normalize(text))))


def classify_query(text, *, plan_reply=False):
    """Recognize informational requests before the motion interrupt path.

    Mixed questions + movement requests go through normal plan validation.
    Unrecognized questions ask for clarification without interrupting a task.
    """
    t = normalize(text).strip()
    phrase = t.rstrip(' ?.!,')
    if re.fullmatch(RESUME_CHECK, phrase):
        return ['resume']
    if re.fullmatch(r'(?:con (?:bay )?gio(?: thi sao)?|gio thi sao|what about now|and now)', phrase):
        return ['refresh']
    if phrase in ('pin', 'battery', 'pin hien tai', 'battery report', 'battery report please'):
        return ['battery']
    if re.fullmatch(WHY_AGAIN, phrase):
        return ['followup']
    if re.fullmatch(SLOT, phrase):
        return None if plan_reply else ['inventory']
    if re.fullmatch(r'dung (?:vi sao|tai sao|vi ly do gi|luc nao|chua|roi (?:a|ha))', phrase):
        return ['motion'] if re.search(r'chua|roi', phrase) else ['fault']
    # Vietnamese "dùng model" (use a model) is not "dừng" (stop).
    motion = r'(?:lay|dat|di|ve|lui|tien|quay|chuyen|chay|bat|tat|nang|ha|huy|dung(?!\s+(?:model|mo hinh|cong nghe|llm|backend|yolo)\b)|tiep tuc|resume|continue|start|stop|move|drive|reverse|turn|go|pick|retrieve|place|switch|lift|lower|cancel)\b'
    if re.search(r'\b(?:toi muon(?! biet| hoi)|toi can(?! biet)|i want(?! to know)|i need(?! to know))\b', t):
        return None
    if re.search(r'(?:^|[;,.?!\n]|\b(?:roi|sau do|then|va|and)\b)\s*'
                 r'(?:(?:hay|please|vui long|cho xe|cho robot|ban|ban co the|can you|could you|would you)\s+)*'
                 + motion, t):
        return None
    # Static project questions also work without telemetry or an LLM API.
    # Keep them after the motion guards so mixed requests still need planning.
    profile_patterns = {
        'llm_model': r'\b(?:mo hinh|model|backend|gemini|groq|ollama|chatgpt|llama|qwen)\b',
        'llm_workflow': r'\b(?:(?:ban|tro ly|llm|ai) (?:hoat dong|xu ly|hieu lenh|lam viec)|cach (?:ban|tro ly|llm) (?:hoat dong|xu ly|lam viec)|quy trinh (?:xu ly|llm)|how (?:do you|does (?:the assistant|the llm)) work)\b',
        'identity': r'\b(?:(?:ban|cau|may|em|tro ly|robot|xe) (?:la ai|ten (?:la )?gi)|ten (?:cua )?(?:ban|tro ly|robot|xe) (?:la )?gi|who (?:are you|is the assistant)|what is your name|introduce yourself|gioi thieu (?:ve )?(?:ban than|chinh ban))\b',
        'creators': r'\b(?:ai (?:da )?(?:tao ra|lam ra|che tao|phat trien|thiet ke|thuc hien|lap trinh)|nguoi (?:tao ra|che tao|phat trien|thuc hien)|(?:tao ra|phat trien|lap trinh) boi ai|(?:nhom|sinh vien) (?:nao|thuc hien|phat trien)|truong ngoc trieu|pham le thien vu|who (?:created|made|built|developed|designed)|creators?|developers?|project team)\b',
        'advisor': r'\b(?:giao vien huong dan|giang vien huong dan|thay huong dan|nguoi huong dan|duong minh thien|supervisor|advisor|who (?:supervises|supervised))\b',
        'project': r'\b(?:do an|de tai|gioi thieu (?:ve )?(?:xe|robot)|amr la gi|amr nghia la gi|muc (?:dich|tieu)|ung dung|project|what (?:is an amr|does amr mean|is this robot for))\b',
        'technology': r'\b(?:cong nghe|kien truc|cau tao|hoat dong nhu the nao|nguyen ly|technology|technologies|architecture|how (?:does|do) .*work)\b',
    }
    profile_topics = [topic for topic, pattern in profile_patterns.items() if re.search(pattern, t)]
    if 'llm_model' in profile_topics and re.search(r'\b(?:pallet|yolo|openvino|segmentation|segment|detector)\b', t) and not re.search(r'\b(?:llm|tro ly)\b', t):
        profile_topics.remove('llm_model')
        if 'technology' not in profile_topics:
            profile_topics.append('technology')
    if 'llm_workflow' in profile_topics and 'technology' in profile_topics:
        profile_topics.remove('technology')
    if profile_topics:
        # A creator/advisor/architecture question already identifies the project;
        # include its overview only when explicitly requested as well.
        if len(profile_topics) > 1 and 'project' in profile_topics and not re.search(r'\b(?:gioi thieu|tong quan|overview)\b', t):
            profile_topics.remove('project')
    question = bool(re.search(r'\b(?:sao|bao nhieu|bao gio|bao lau|may|gi|nao|khong|chua|when|tai sao|vi sao|the nao|lam gi|o dau|how|what|which|why|where|can|is|are)\b|\?', t))
    report = bool(re.search(r'\b(?:ton kho|trang thai|tien do|kiem tra|tom tat|che do hien tai|bao cao|thong ke|cho biet|cho toi biet|giai thich|ke chuyen|inventory|status|progress|capabilities|tell me|explain)\b', t))
    slot_followup = bool(re.search(r'\b(?:con|the con|what about|how about)\s+[ab][12]\b', t))
    if not (question or report or profile_topics or slot_followup):
        return None
    topics = list(profile_topics)
    patterns = {
        'inventory': r'\b(?:ke(?!\s+chuyen\b)|ton kho|o trong|o nao|shelf|shelves|inventory|stock|slots?|a[12]|b[12])\b',
        'mission': r'\b(?:mission|chu trinh|tien do|da dat|dang lam|lam gi|loi|recovery|resume|tiep tuc|that bai|xong|progress|doing|error|failed|complete)\b',
        'cargo': r'\b(?:dang mang|dang cho|co mang|tren cang|cargo|carrying|load)\b',
        'lift': r'\b(?:do cao|cao bao nhieu|lift|fork|cang cao|cang dang)\b',
        'battery': r'\b(?:pin|battery)\b',
        'health': r'\b(?:camera|cam bien|sensor|thiet bi|ket noi|connection|health|san sang|ready)\b',
        'navigation': r'\b(?:nav2|navigation|dieu huong|goal)\b',
        'motion': r'\b(?:di chuyen|dung yen|dang chay|van toc|toc do|moving|stationary|stopped|speed)\b',
        'mode': r'\b(?:che do|mode|auto|manual)\b',
        'location': r'\b(?:o dau|vi tri|where|location)\b',
        'capabilities': r'\b(?:lam duoc|co the lam|chuc nang|kha nang|ho tro|capabilities|help|can you do|can the robot do)\b',
    }
    for topic, pattern in patterns.items():
        if re.search(pattern, t):
            topics.append(topic)
    if (re.search(r'\b(?:loi|that bai|failed|error|aborted)\b', t)
            or (re.search(r'\b(?:sao|why|nguyen nhan|co phai|was it)\b', t)
                and re.search(r'\b(?:dung|ngung|ket|stop(?:ped)?|vat can|lech goc|mat dinh vi|obstacle)\b', t))):
        topics = [x for x in topics if x not in ('mission', 'navigation', 'motion')] + ['fault']
    if re.search(r'\b(?:resume|chay tiep|tiep tuc)\b', t) and (question or report):
        topics = [x for x in topics if x != 'mission'] + ['resume']
    if re.search(r'\b(?:hang gi|loai (?:hang|pallet) nao|what.*(?:stored|cargo|hold)|what.s in)\b', t) and (
            mentioned_slots(t) or re.search(SLOT_REF, t)):
        topics = [x for x in topics if x != 'inventory'] + ['slot_contents']
    elif not topics and re.search(SLOT_REF, t) and re.search(r'\b(?:trong|hang|empty|occupied)\b', t):
        topics.append('inventory')
    if re.search(r'\b(?:bao lau|bao gio|khi nao|how long|when|eta|time left|remaining time)\b', t):
        if re.search(r'\b(?:pin|battery)\b', t):
            topics.append('battery_runtime')
        else:
            topics.extend(['mission', 'eta'])
    if not topics and re.search(r'\b(?:robot|xe|trang thai|status|bao cao)\b', t):
        topics = ['mission'] if re.search(r'\b(?:xe|robot)\b', t) else ['overview']
    return list(dict.fromkeys(topics)) or ['clarification']


def resolve_reference(topics, text, previous):
    """Only carry the subject. Measurements and admission flags belong to this turn."""
    t = normalize(text).strip(' ?.!,')
    prior_topics, prior_slots = previous or ((), ())
    slots = mentioned_slots(t)
    if topics == ['refresh']:
        return (list(prior_topics), prior_slots) if prior_topics else (['clarification'], ())
    if topics == ['followup']:
        if len(prior_topics) == 1 and prior_topics[0] in ('fault', 'mission', 'resume', 'health'):
            topics = ['fault' if prior_topics[0] == 'mission' else prior_topics[0]]
        elif not prior_topics:
            topics = ['cause_clarification']
        else:
            topics = ['clarification']
    if topics == ['inventory'] and len(slots) == 1 and prior_topics == ('slot_contents',) and re.fullmatch(
            r'(?:(?:con|the con|what about|how about) )?' + SLOT + r'(?: thi sao)?', t):
        topics = ['slot_contents']
    if not slots and re.search(SLOT_REF, t) and any(k in topics for k in ('slot_contents', 'inventory')):
        if len(prior_slots) == 1:
            slots = prior_slots
    return topics, slots


def number(value, low=None, high=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        return None
    if (low is not None and value < low) or (high is not None and value > high):
        return None
    return value


def build_context(*, supervisor=None, supervisor_age=None, pallet=None, pallet_age=None,
                  retrieval=None, retrieval_age=None, lift=None, lift_age=None,
                  battery=None, battery_age=None, mode='unknown', location='unknown', max_age=5.0):
    sup = supervisor if isinstance(supervisor, dict) else {}
    pm = pallet if isinstance(pallet, dict) else {}
    ret = retrieval if isinstance(retrieval, dict) else {}
    fresh = lambda age, limit: number(age, 0, limit) is not None
    flag = lambda value: value if isinstance(value, bool) else None
    sf, pf, rf = fresh(supervisor_age, max_age), fresh(pallet_age, 3), fresh(retrieval_age, 3)
    shelf_ok = sf and sup.get('shelf_ok') is True
    raw_slots = sup.get('slots') if isinstance(sup.get('slots'), dict) else {}
    slots = {s: raw_slots.get(s) if shelf_ok and raw_slots.get(s) in ('empty', 'full') else 'unknown'
             for s in SLOTS}
    counts = {state: sum(v == state for v in slots.values()) for state in ('full', 'empty', 'unknown')}
    cargo = pm.get('cargo') if pf else sup.get('cargo') if sf and sup.get('pm_status_fresh') is True else None
    cargo = cargo if isinstance(cargo, bool) else None
    cargo_type = pm.get('cargo_type') if pf else sup.get('cargo_type') if sf else None
    cargo_type = cargo_type if cargo is True and cargo_type in ('A', 'B') else None
    height = number(lift, 0, 80) if fresh(lift_age, 3) else None
    if height is None and pf:
        height = number(pm.get('lift_cm'), 0, 80)
    cycle = sup.get('cycle') if sf and isinstance(sup.get('cycle'), dict) else {}
    owner = 'retrieval' if rf and ret.get('state') in ACTIVE_RETRIEVAL else 'supervisor'
    return {
        'supervisor_state': sup.get('state', 'unknown') if sf else 'unknown',
        'handling_status': sup.get('handling_status', 'unknown') if sf else 'unknown',
        'pm_handling_status': sup.get('pm_handling_status', 'unknown') if sf else 'unknown',
        'pm_waiting_for': sup.get('pm_waiting_for') if sf and sup.get('pm_status_fresh') is True else None,
        'pm_phase': sup.get('pm_phase') if sf else None,
        'pm_err': sup.get('pm_err') if sf and sup.get('pm_status_fresh') is True and isinstance(sup.get('pm_err'), dict) else None,
        'pm_retries': sup.get('pm_retries') if sf and isinstance(sup.get('pm_retries'), dict) else None,
        'pm_message': str(sup.get('pm_msg') or '')[:300] if sf else '',
        'supervisor_status_age_s': supervisor_age,
        'supervisor_fresh': sf,
        'active_mission': sup.get('active_mission') if sf else None,
        'active_destination': sup.get('active_dest') if sf else None,
        'cycle': {k: cycle.get(k) for k in ('run_mode', 'placed', 'target_quantity', 'target_cargo_class', 'target_drop_slot')},
        'progress_percent': number(sup.get('progress_percent'), 0, 100) if sf else None,
        'mission_message': str(sup.get('msg') or '')[:600] if sf else '',
        'resume_available': flag(sup.get('resume_available')) if sf else None,
        'resume_reason': str(sup.get('resume_reason') or '')[:300] if sf else '',
        'resume_state': sup.get('resume_state') if sf else None,
        'retries': sup.get('retries') if sf else None,
        'shelf': {'slots': slots, **counts, 'capacity': len(SLOTS), 'exact': counts['unknown'] == 0,
                  'note': 'Occupancy counts slots/pallet positions, not individual products inside a box.'},
        'cargo': cargo, 'cargo_type': cargo_type,
        'pallet_state': pm.get('state', 'unknown') if pf else 'unknown',
        'pallet_status_age_s': pallet_age,
        'pallet_message': str(pm.get('msg') or '')[:600] if pf else '',
        'pallet_fail_phase': pm.get('fail_phase') if pf else None,
        'lift_cm': height,
        'battery_percent': number(battery, 0, 100) if fresh(battery_age, 5) else None,
        'operating_mode': mode, 'location_hint': location,
        'robot_stopped': flag(sup.get('robot_stopped')) if sf and sup.get('odom_fresh') is True else None,
        'nav_status_fresh': sf and sup.get('nav_status_fresh') is True,
        'supervisor_nav_goal_active': flag(sup.get('nav_goal_active')) if sf and sup.get('nav_status_fresh') is True else None,
        'health': {'camera': flag(sup.get('camera_ok')) if sf else None,
                   'shelf': shelf_ok if sf else None,
                   'route': flag((sup.get('route_graph') or {}).get('loaded')) if sf and isinstance(sup.get('route_graph'), dict) else None},
        'retrieval': {'state': ret.get('state', 'unknown') if rf else 'unknown',
                      'slot': ret.get('slot') if rf else None,
                      'message': str(ret.get('message') or '')[:600] if rf else '',
                      'recovery_required': rf and ret.get('recovery_required') is True,
                      'resume_available': flag(ret.get('resume_available')) if rf else None,
                      'resume_reason': str(ret.get('resume_reason') or '')[:300] if rf else '',
                      'recovery_from': ret.get('recovery_from') if rf else None,
                      'stale_active': not rf and ret.get('state') in ACTIVE_RETRIEVAL,
                      'fresh': rf, 'status_age_s': retrieval_age},
        'active_owner': owner,
    }


def answer_query(topics, context, text='', language='vi', slots=None):
    """Answer from sanitized live facts; never emit action steps."""
    en = language == 'en'
    t = normalize(text)
    topics = list(topics)
    if 'overview' in topics:
        topics = list(dict.fromkeys([topic for topic in topics if topic != 'overview'] +
                                   ['mission', 'inventory', 'cargo', 'lift', 'health', 'battery',
                                    'navigation', 'motion', 'mode', 'location']))
    slots = mentioned_slots(text) if slots is None else slots
    replies = []
    for topic in topics:
        if topic in PROFILE_ANSWERS:
            replies.append(PROFILE_ANSWERS[topic]['en' if en else 'vi'])
    if 'cause_clarification' in topics:
        if context['supervisor_state'] in ('RECOVERY', 'FAILED', 'STOPPED') or context['retrieval']['state'] == 'RECOVERY':
            replies.append(_mission_reply(context, t, True, en))
        else:
            replies.append('Which problem do you mean?' if en else 'Bạn muốn hỏi nguyên nhân chuyện gì?')
    if 'clarification' in topics:
        replies.append('What would you like to check?' if en else 'Bạn muốn kiểm tra phần nào?')
    if 'llm_model' in topics:
        llm = context.get('llm') or {}
        configured = llm.get('configured') or []
        last = llm.get('last_success')
        model = last or (configured[0] if configured else None)
        if model:
            if last:
                line = (f"I am the AMR robot's AI assistant, using {model['model']} to understand requests and plan tasks." if en else
                        f"Tôi là trợ lý AI của xe robot AMR, sử dụng mô hình {model['model']} để hiểu yêu cầu và lập kế hoạch cho xe.")
            else:
                line = (f"I am the AMR robot's AI assistant, configured to use {model['model']} to understand requests and plan tasks." if en else
                        f"Tôi là trợ lý AI của xe robot AMR, được cấu hình sử dụng mô hình {model['model']} để hiểu yêu cầu và lập kế hoạch cho xe.")
        else:
            line = ("I am the AMR robot's AI assistant. The model name is currently unavailable." if en else
                    'Tôi là trợ lý AI của xe robot AMR. Hiện tôi chưa xác định được tên mô hình đang sử dụng.')
        line += (' After you confirm a control plan, the robot executes its steps in order.' if en else
                 ' Sau khi bạn xác nhận kế hoạch điều khiển, xe sẽ thực hiện từng bước theo thứ tự.')
        replies.append(line)
    if 'inventory' in topics or 'slot_contents' in topics:
        replies.append(_shelf_reply(context['shelf'], slots, t, 'slot_contents' in topics, en))
    if 'mission' in topics or 'fault' in topics:
        replies.append(_mission_reply(context, t, 'fault' in topics, en))
    if 'resume' in topics:
        replies.append(_resume_reply(context, en))
    if 'cargo' in topics:
        cargo = context['cargo']
        replies.append(('Cargo is unknown; telemetry is missing/stale.' if en else 'Mình chưa có số đo mới để biết trên càng có hàng không.') if cargo is None else
                       ('No cargo confirmed.' if en else 'Xe báo không mang pallet.') if cargo is False else
                       ('Carrying a pallet' if en else 'Xe đang mang pallet') + {'A': ' medium (A).', 'B': ' tall (B).'}.get(context['cargo_type'], '; class unknown.' if en else '; chưa rõ loại.'))
    if 'lift' in topics:
        h = context['lift_cm']
        replies.append(('Lift height unavailable/stale.' if en else 'Chưa có độ cao càng còn mới.') if h is None else
                       (f'Fork height: {h:g} cm.' if en else f'Càng đang ở độ cao {h:g} cm.'))
    if 'mode' in topics:
        replies.append(('Navigation mode: ' if en else 'Chế độ Navigation: ') + str(context['operating_mode']) + '.')
    if 'location' in topics:
        hint = context['location_hint']
        replies.append((f'Last configured/reported location label: {hint}; this is not a fresh map pose.' if en else
                        f'Nhãn vị trí đã báo/cấu hình: {hint}; đây không phải pose map mới được xác minh.'))
    if 'navigation' in topics:
        active = context['supervisor_nav_goal_active']
        if not context['nav_status_fresh'] or active is None:
            replies.append('Nav2: navigation status is missing/stale; goal activity is unknown.' if en else
                           'Mình chưa có cập nhật mới từ Nav2 để biết xe đang đi tới điểm nào.')
        else:
            replies.append(('Nav2: a navigation goal is active.' if active else 'Nav2: no active navigation goal is reported.') if en else
                           ('Nav2 đang dẫn xe tới điểm đến.' if active else 'Nav2 hiện không có điểm đến đang chạy.'))
        if re.search(r'\b(?:ready|san sang)\b', t):
            replies.append('Goal status does not establish that every Nav2 lifecycle node is ready.' if en else
                       'Trạng thái goal chưa đủ để xác nhận toàn bộ node lifecycle Nav2 sẵn sàng.')
    if 'motion' in topics:
        stopped = context['robot_stopped']
        if stopped is None:
            replies.append('Motion is unknown: fresh odometry is unavailable.' if en else
                           'Chưa có số đo mới để biết xe đang chạy hay đứng yên.')
        else:
            replies.append(('The robot is stationary according to fresh odometry.' if stopped else 'The robot is moving according to fresh odometry.') if en else
                           ('Xe đang đứng yên.' if stopped else 'Xe đang di chuyển.'))
    if 'eta' in topics:
        replies.append('I cannot estimate mission completion time: the system has no validated remaining-time estimate. Progress percentage alone is insufficient.' if en else
                       'Chưa thể ước lượng còn bao lâu hoàn thành mission: hệ thống chưa có dữ liệu thời gian còn lại đáng tin cậy. Phần trăm tiến độ chưa đủ để suy ra thời gian.')
    if 'battery_runtime' in topics:
        replies.append('I cannot estimate remaining operating time from battery percentage alone; discharge history and power consumption are unavailable.' if en else
                       'Chưa thể ước lượng xe chạy thêm bao lâu chỉ từ phần trăm pin; hiện chưa có lịch sử xả pin và mức tiêu thụ điện để tính thời lượng còn lại.')
    if 'battery' in topics:
        battery = context['battery_percent']
        replies.append(('I have no current battery reading.' if en else 'Mình chưa có số đo pin mới.')
                       if battery is None else
                       (f'Battery: {battery:g}%.' if en else (f'Còn {battery:g}%.' if len(topics) == 1 else f'Pin còn {battery:g}%.')))
    if 'health' in topics:
        labels = {'camera': 'Camera', 'shelf': 'Shelf sensor' if en else 'Cảm biến kệ',
                  'route': 'Route' if en else 'Tuyến đường'}
        keys = ['camera'] if 'camera' in t and len(topics) == 1 else labels
        for key in keys:
            value = context['health'][key]
            words = {True: 'is ready.' if en else 'đã sẵn sàng.',
                     False: 'is not ready.' if en else 'chưa sẵn sàng.',
                     None: 'has no current update.' if en else 'chưa có cập nhật mới.'}
            replies.append(f'{labels[key]} {words[value]}')
        if re.search(r'\b(?:tai sao|vi sao|why)\b', t):
            replies.append('I have no further details on the cause.' if en else 'Mình chưa có thông tin cụ thể về nguyên nhân.')
    if 'capabilities' in topics:
        replies.append('I can report inventory by slot/type, cargo, fork height, battery, mission progress and recovery. I can plan approved navigation, detection, pick/place, Auto/single cycles and retrieval from A1/A2/B1/B2. Questions can run alongside a mission; physical actions are sequenced with confirmation. Retrieval Resume requires a valid operation checkpoint.' if en else
                       'Bạn có thể hỏi mình về xe, hàng trên kệ hoặc tiến độ lấy và đặt pallet. Nếu muốn xe di chuyển hay nâng hạ càng, mình sẽ đưa các bước để bạn xác nhận.')
    return '\n'.join(replies)


def _shelf_reply(shelf, slots, text, contents, en):
    if not slots and (contents or re.search(SLOT_REF, text)):
        return 'Which slot do you mean?' if en else 'Bạn muốn hỏi ô nào?'
    if slots:
        lines = []
        for slot in slots:
            state = shelf['slots'][slot]
            if state == 'unknown':
                line = (f'I have no current reading for {slot}.' if en else
                        f'Mình chưa có số đo mới cho ô {slot}.')
            elif state == 'empty':
                line = f'{slot} is empty.' if en else f'{slot} đang trống.'
            else:
                line = f'{slot} is occupied.' if en else f'{slot} có hàng.'
            if contents:
                size = 'medium' if slot.startswith('A') else 'tall'
                line += (f' This slot is for {size} pallets.' if en else
                         f' Ô này dành cho pallet loại {"vừa" if size == "medium" else "cao"}.')
                if state != 'empty':
                    line += (' The sensor cannot identify the goods.' if en else
                             ' Cảm biến chưa cho biết mặt hàng cụ thể.')
            lines.append(line)
        return ' '.join(lines)
    selected = SLOTS
    if re.search(r'\b(?:medium|thap|vua|loai a)\b', text):
        selected = SLOTS[:2]
    elif re.search(r'\b(?:tall|high|cao|loai b)\b', text):
        selected = SLOTS[2:]
    states = shelf['slots']
    if all(states[s] == 'unknown' for s in selected):
        return 'I have no current shelf readings.' if en else 'Mình chưa có số đo mới từ kệ.'
    parts = []
    for state, label in [('full', 'occupied' if en else 'có hàng'),
                         ('empty', 'empty' if en else 'trống'),
                         ('unknown', 'unknown' if en else 'chưa rõ')]:
        group = [s for s in selected if states[s] == state]
        if group:
            parts.append(f'{len(group)} {label} ({", ".join(group)})' if en else
                         f'{len(group)} ô {label} ({", ".join(group)})')
    line = ', '.join(parts) + '.'
    if re.search(r'\b(?:san pham|products?|units?)\b', text):
        line += (' I can count occupied slots only.' if en else
                 ' Mình chỉ đếm được vị trí pallet, chưa biết số sản phẩm bên trong.')
    return line


# These are producer states, not inferred physical motion.
ACTIVITY = {
    'IDLE': ('Xe đang chờ nhiệm vụ.', 'The robot is waiting for a task.'),
    'DONE': ('Xe đã hoàn thành nhiệm vụ.', 'The mission is complete.'),
    'RETURN_HOME': ('Xe đang về HOME.', 'The robot is returning to HOME.'),
    'RETURNING_HOME': ('Xe đang về HOME.', 'The robot is returning to HOME.'),
    'GOTO_PICK': ('Xe đang tới điểm lấy hàng.', 'The robot is heading to the pickup point.'),
    'GOTO_D': ('Xe đang tới điểm giao hàng.', 'The robot is heading to the drop-off point.'),
    'PICKING': ('Xe đang lấy pallet.', 'The robot is picking up a pallet.'),
    'DROPPING': ('Xe đang đặt pallet lên kệ.', 'The robot is placing a pallet on the rack.'),
    'WAITING_SLOT': ('Xe đang chờ ô kệ trống.', 'The robot is waiting for a free slot.'),
    'HOMING_LIFT': ('Xe đang đưa càng về vị trí gốc.', 'The robot is homing the forks.'),
    'ALIGNING_TO_SLOT': ('Xe đang căn vị trí trước ô kệ.', 'The robot is aligning with the slot.'),
    'NAVIGATING_TO_D1': ('Xe đang tới kệ lấy hàng.', 'The robot is navigating to the rack.'),
    'NAVIGATING_TO_D2': ('Xe đang đưa hàng tới D_2.', 'The robot is taking the pallet to D_2.'),
    'PLACING_AT_D2': ('Xe đang đặt hàng ở D_2.', 'The robot is placing the pallet at D_2.'),
    'LOADING_DETECTOR': ('Xe đang bật nhận diện pallet.', 'The pallet detector is starting.'),
    'LIFTING': ('Xe đang nâng càng.', 'The forks are rising.'),
    'LOWERING': ('Xe đang hạ càng.', 'The forks are lowering.'),
    'ALIGNING': ('Xe đang căn vị trí với pallet.', 'The robot is aligning with the pallet.'),
    'INSERTING': ('Xe đang đưa càng vào pallet.', 'The robot is inserting the forks.'),
    'DETECTING': ('Xe đang tìm pallet.', 'The robot is looking for the pallet.'),
    'AUTO_RECOVERING': ('Xe đang tự xử lý trục trặc.', 'The controller is attempting recovery.'),
    'WAITING_FOR_SENSOR': ('Xe đang chờ số đo mới từ cảm biến.', 'The robot is waiting for a sensor reading.'),
    'NEED_OPERATOR': ('Xe đang chờ người vận hành kiểm tra.', 'The robot needs an operator inspection.'),
    'STOPPING': ('Xe đang xử lý yêu cầu dừng.', 'The stop request is in progress.'),
    'CANCELING': ('Xe đang xử lý yêu cầu hủy.', 'Cancellation is in progress.'),
}


def nav_failure(message, home, en, *, explain=False, followup=False):
    """The supervisor writes this when it consumes the current leg's result."""
    if not re.search(r'\b(?:Nav2\s+ABORTED|Navigation result status=6|return_home aborted)\b', message, re.I):
        return ''
    home = home or bool(re.search(r'\bfor return_home\b', message, re.I))
    line = ('The robot did not reach HOME. ' if en else 'Xe chưa về tới HOME. ') if home else ''
    line += 'Nav2 ended this navigation attempt.' if en else 'Nav2 đã dừng lượt điều hướng này.'
    if 'NavigateToPose server not ready' in message:
        return line + (' The navigation server was not ready.' if en else ' Dịch vụ điều hướng chưa sẵn sàng.')
    if 'Nav2 goal not accepted' in message:
        return line + (' Nav2 did not accept the goal.' if en else ' Nav2 chưa nhận điểm đến.')
    if followup:
        return ('Nav2 only reported that the attempt ended. I have no more detail about why.' if en else
                'Nav2 chỉ báo lượt điều hướng đã dừng. Mình chưa có thêm thông tin về nguyên nhân.')
    if not explain:
        return line
    return line + (' I have no further details on the cause.' if en else
                   ' Mình chưa có thông tin để biết vì sao Nav2 dừng.')


def _mission_reply(c, text, why, en):
    r = c['retrieval']
    retrieval = c['active_owner'] == 'retrieval'
    if not retrieval and (not c['supervisor_fresh'] or (r['stale_active'] and c['supervisor_state'] == 'IDLE')):
        return 'I have no current mission update.' if en else 'Mình chưa có cập nhật mới về nhiệm vụ của xe.'
    if not retrieval and c['supervisor_state'] == 'IDLE' and r['state'] in ('FAILED', 'CANCELED') and r['recovery_required']:
        return ('The last retrieval stopped. Check the forks and pallet.' if en else
                'Lượt lấy hàng từ kệ đã dừng. Bạn kiểm tra càng và pallet nhé.')
    state = r['state'] if retrieval else c['supervisor_state']
    message = r['message'] if retrieval else c['mission_message']
    handling = state if retrieval else c['handling_status']
    if state in ('FAILED', 'RECOVERY', 'STOPPED'):
        home = (r['recovery_from'] == 'RETURNING_HOME' if retrieval else
                c['resume_state'] == 'RETURN_HOME' or c['active_mission'] == 'return_home')
        followup = bool(re.fullmatch(WHY_AGAIN, text.strip(' ?.!, ')))
        nav = nav_failure(message, home, en, explain=why, followup=followup)
        if nav:
            return nav
        line = 'The mission stopped.' if en else 'Nhiệm vụ đã dừng.'
        detail = reported_reason(message, en)
        return line + ' ' + (detail or ('I have no further details on the cause.' if en else
                                       'Mình chưa có thông tin cụ thể về nguyên nhân.'))
    if not retrieval and state == 'IDLE' and c.get('local_action'):
        return c['local_action']
    if not retrieval and state in ('IDLE', 'DONE') and c['pallet_state'] in ACTIVITY:
        if c['pallet_state'] not in ('IDLE', 'DONE'):
            state = c['pallet_state']
    if not retrieval and state == 'IDLE' and c['pallet_state'] in ('FAILED', 'STOPPED'):
        detail = reported_reason(c['pallet_message'], en)
        return ('Pallet handling stopped. ' if en else 'Thao tác với pallet đã dừng. ') + (
            detail or ('The cause is unclear.' if en else 'Mình chưa rõ nguyên nhân.'))
    activity = handling if handling in ('AUTO_RECOVERING', 'NEED_OPERATOR', 'WAITING_FOR_SENSOR') else state
    line = ACTIVITY.get(activity, ('Mình chưa rõ xe đang làm gì.', 'The current activity is unclear.'))[en]
    if activity == 'WAITING_FOR_SENSOR' and c['pm_waiting_for'] and not retrieval:
        sensor = {'tf': 'TF', 'ultrasonic': 'siêu âm' if not en else 'ultrasonic'}.get(str(c['pm_waiting_for']).lower())
        if sensor:
            line = (f'Waiting for fresh {sensor} data.' if en else f'Xe đang chờ số đo mới từ {sensor}.')
    if why and activity == 'AUTO_RECOVERING' and not retrieval:
        yaw = number((c['pm_err'] or {}).get('yaw_deg'))
        if yaw is not None:
            line += (f' Measured angle error is {yaw:g}°.' if en else
                     ' Góc lệch đo được là ' + f'{yaw:g}'.replace('.', ',') + '°.')
    if why and activity not in ('AUTO_RECOVERING', 'NEED_OPERATOR', 'WAITING_FOR_SENSOR'):
        line += (' I have no reported cause for that stop.' if en else ' Mình chưa thấy nguyên nhân của lần dừng bạn hỏi.')
    if not retrieval and re.search(r'\b(?:tien do|da dat|bao nhieu|progress|placed|how many|eta)\b', text):
        placed = number(c['cycle'].get('placed'), 0)
        target = number(c['cycle'].get('target_quantity'), 1)
        if placed is not None:
            count = f'{placed:g}/{target:g}' if target else f'{placed:g}'
            line += (f' {count} pallets placed.' if en else f' Đã đặt {count} pallet.')
        if c['progress_percent'] is not None:
            line += (f" Progress is {c['progress_percent']:g}%." if en else f" Tiến độ hiện tại là {c['progress_percent']:g}%.")
    return line


def reported_reason(detail, en):
    # Only translate conditions explicitly reported by the controller.
    facts = {
        'waiting for nav2 idle status': ('Nav2 chưa báo đã kết thúc điều hướng.', 'Nav2 has not reported idle yet.'),
        'nav2 has not confirmed idle': ('Nav2 chưa báo đã kết thúc điều hướng.', 'Nav2 has not reported idle yet.'),
        'waiting for fresh odometry and a stationary robot': ('Xe đang chờ số đo xác nhận đã đứng yên.', 'Waiting for a reading confirming the robot is stationary.'),
        'waiting for fresh pallet/cargo status after stop': ('Chưa có cập nhật về hàng trên càng sau khi dừng.', 'Cargo status has not been updated since the stop.'),
        'camera is not ready': ('Camera chưa sẵn sàng.', 'The camera is not ready.'),
        'route graph missing or stale': ('Xe chưa có dữ liệu tuyến đường mới.', 'Current route data is unavailable.'),
        'route changed since recovery checkpoint': ('Tuyến đường đã thay đổi kể từ khi xe dừng.', 'The route has changed since the stop.'),
        'cargo state unknown': ('Chưa xác định được hàng trên càng.', 'Cargo status is unknown.'),
    }
    message = normalize(detail).strip().rstrip('.')
    if message in facts:
        return facts[message][en]
    known_reports = (
        ('user-requested recovery', 'Người vận hành đã yêu cầu dừng.', 'The operator requested the stop.'),
        ('ve home qua timeout', 'Xe chưa kịp về HOME trong thời gian cho phép.', 'The return to HOME timed out.'),
        ('khong doc duoc pose home/robot', 'Chưa có vị trí để xác nhận xe đã về HOME.', 'Position data could not confirm arrival at HOME.'),
        ('camera pallet khong co frame moi', 'Camera nhận diện pallet chưa gửi ảnh mới.', 'There is no recent pallet camera image.'),
        ('du lieu cam bien ke stale', 'Chưa có số đo mới từ kệ.', 'The shelf reading is out of date.'),
        ('drop failed', 'Lượt đặt pallet chưa hoàn tất. Bạn kiểm tra vị trí pallet nhé.', 'Placement did not complete. Check the pallet position.'),
        ('service timed out', 'Xe chưa phản hồi nên chưa rõ lệnh đã chạy đến đâu.', 'No response yet. The outcome of the command is unknown.'),
        ('service unavailable', 'Dịch vụ trên xe chưa sẵn sàng cho thao tác này.', 'The service for this operation is unavailable.'),
        ('service outcome uncertain', 'Chưa có phản hồi xác nhận kết quả thao tác.', 'The outcome of the operation is unconfirmed.'),
        ('fresh lift joint_states required', 'Chưa có số đo độ cao càng mới.', 'A current fork height reading is needed.'),
        ('fresh odometry required', 'Chưa có số đo chuyển động mới.', 'A current motion reading is needed.'),
        ('fresh stationary odometry required', 'Xe đang chờ số đo xác nhận đã đứng yên.', 'A reading confirming the robot is stationary is needed.'),
        ('waiting for pallet stop', 'Đang chờ xác nhận dừng thao tác pallet.', 'Waiting for the pallet controller to confirm the stop.'),
        ('retrieval checkpoint expired', 'Đã hết thời gian chạy tiếp lượt lấy hàng này. Bạn kiểm tra xe nhé.', 'The retrieval checkpoint expired. Inspect the robot.'),
        ('manual checkpoint expired', 'Đã hết thời gian chạy tiếp kế hoạch này. Bạn kiểm tra xe nhé.', 'The paused plan expired. Inspect the robot.'),
    )
    for prefix, vi, english in known_reports:
        if message.startswith(prefix):
            return english if en else vi
    return ''


def _resume_reply(c, en):
    r = c['retrieval']
    available, reason = c['resume_available'], c['resume_reason']
    if c['active_owner'] == 'retrieval':
        available, reason = r['resume_available'], r['resume_reason']
        recovery = r['state'] == 'RECOVERY'
    elif r['stale_active']:
        available, recovery = None, False
    elif 'manual_resume_reason' in c and c['supervisor_state'] != 'RECOVERY':
        reason = c['manual_resume_reason']
        available, recovery = not reason, True
    else:
        recovery = c['supervisor_state'] == 'RECOVERY'
    if available is None:
        return 'I have no current Resume decision.' if en else 'Mình chưa có dữ liệu mới để biết xe chạy tiếp được chưa.'
    if recovery and available:
        return 'Yes. Resume is available and needs your confirmation.' if en else 'Được. Xe báo có thể chạy tiếp khi bạn xác nhận.'
    if recovery:
        return ('Not yet. ' if en else 'Chưa. ') + (reported_reason(reason, en) or (
            'The controller has not cleared Resume.' if en else 'Xe chưa báo đủ điều kiện chạy tiếp.'))
    return 'There is no paused mission to resume.' if en else 'Hiện chưa có nhiệm vụ tạm dừng để chạy tiếp.'
