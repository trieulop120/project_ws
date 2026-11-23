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
          'eta', 'battery_runtime', 'clarification', 'capabilities', 'location',
          'mode', 'overview', 'llm_model', *PROFILE_ANSWERS)
ACTIVE_RETRIEVAL = {'HOMING_LIFT', 'NAVIGATING_TO_D1', 'ALIGNING_TO_SLOT',
                    'AUTO_RECOVERING', 'WAITING_FOR_SENSOR',
                    'LOADING_DETECTOR', 'DETECTING', 'PICKING', 'NAVIGATING_TO_D2',
                    'PLACING_AT_D2', 'RETURNING_HOME', 'CANCELING', 'RECOVERY'}


def normalize(text):
    return ''.join(c for c in unicodedata.normalize('NFD', str(text).lower().replace('đ', 'd'))
                   if unicodedata.category(c) != 'Mn')


def classify_query(text):
    """Recognize informational requests before the motion interrupt path.

    Mixed questions + movement requests go through normal plan validation.
    Unrecognized questions ask for clarification without interrupting a task.
    """
    t = normalize(text).strip()
    # Vietnamese "dùng model" (use a model) is not "dừng" (stop).
    motion = r'(?:lay|dat|di|ve|lui|tien|quay|chuyen|chay|bat|tat|nang|ha|huy|dung(?!\s+(?:model|mo hinh|cong nghe|llm|backend|yolo)\b)|tiep tuc|start|stop|move|drive|reverse|turn|go|pick|retrieve|place|switch|lift|lower|cancel)\b'
    if re.search(r'\b(?:toi muon(?! biet| hoi)|toi can(?! biet)|i want(?! to know)|i need(?! to know))\b', t):
        return None
    if re.search(r'(?:^|[;,.?!\n]|\b(?:roi|sau do|then|va|and)\b)\s*'
                 r'(?:(?:hay|please|vui long|ban|ban co the|can you|could you|would you)\s+)*'
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
    question = bool(re.search(r'\b(?:bao nhieu|bao gio|bao lau|may|gi|nao|khong|chua|when|tai sao|vi sao|the nao|lam gi|o dau|how|what|which|why|where|can|is|are)\b|\?', t))
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
    if re.search(r'\b(?:bao lau|bao gio|khi nao|how long|when|eta|time left|remaining time)\b', t):
        if re.search(r'\b(?:pin|battery)\b', t):
            topics.append('battery_runtime')
        else:
            topics.extend(['mission', 'eta'])
    if not topics and re.search(r'\b(?:robot|xe|trang thai|status|bao cao)\b', t):
        topics = ['overview']
    return list(dict.fromkeys(topics)) or ['clarification']


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
        'pm_waiting_for': sup.get('pm_waiting_for') if sf else None,
        'pm_phase': sup.get('pm_phase') if sf else None,
        'pm_err': sup.get('pm_err') if sf and isinstance(sup.get('pm_err'), dict) else None,
        'pm_retries': sup.get('pm_retries') if sf and isinstance(sup.get('pm_retries'), dict) else None,
        'pm_message': str(sup.get('pm_msg') or '')[:300] if sf else '',
        'supervisor_status_age_s': supervisor_age,
        'supervisor_fresh': sf,
        'active_mission': sup.get('active_mission') if sf else None,
        'active_destination': sup.get('active_dest') if sf else None,
        'cycle': {k: cycle.get(k) for k in ('run_mode', 'placed', 'target_quantity', 'target_cargo_class', 'target_drop_slot')},
        'progress_percent': number(sup.get('progress_percent'), 0, 100) if sf else None,
        'mission_message': str(sup.get('msg') or '')[:600] if sf else '',
        'resume_available': sf and sup.get('resume_available') is True,
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
                      'fresh': rf, 'status_age_s': retrieval_age},
        'active_owner': owner,
    }


def answer_query(topics, context, text='', language='vi'):
    """Answer from sanitized live facts; never emit action steps."""
    en = language == 'en'
    t = normalize(text)
    topics = list(topics)
    if 'overview' in topics:
        topics = list(dict.fromkeys([topic for topic in topics if topic != 'overview'] +
                                   ['mission', 'inventory', 'cargo', 'lift', 'health', 'battery',
                                    'navigation', 'motion', 'mode', 'location']))
    replies = []
    for topic in topics:
        if topic in PROFILE_ANSWERS:
            replies.append(PROFILE_ANSWERS[topic]['en' if en else 'vi'])
    if 'clarification' in topics:
        if re.search(r'\b(?:no|o do|o nay|cai do|it|that slot|this slot)\b', t):
            replies.append('Which slot or object do you mean? Please specify A1/A2/B1/B2 or the cargo on the forks.' if en else
                           'Bạn đang hỏi ô hoặc vật nào? Hãy nêu rõ A1/A2/B1/B2 hoặc hàng trên càng.')
        else:
            replies.append('I cannot answer that from the available robot data. Please specify the information you need; I can report mission, inventory, cargo, lift, battery and navigation status.' if en else
                           'Mình chưa thể trả lời câu này từ dữ liệu xe hiện có. Bạn hãy nêu rõ thông tin cần biết; mình có thể báo mission, tồn kho, hàng trên càng, độ cao càng, pin và điều hướng.')
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
    if 'inventory' in topics:
        shelf = context['shelf']
        selected = [s for s in SLOTS if re.search(r'\b'+s.lower()+r'\b', t)]
        if not selected:
            selected = list(SLOTS)
            if re.search(r'\b(?:medium|thap|vua|loai a)\b', t): selected = ['A1', 'A2']
            elif re.search(r'\b(?:tall|high|cao|loai b)\b', t): selected = ['B1', 'B2']
        states = {s: shelf['slots'][s] for s in selected}
        full, empty, unknown = (sum(v == state for v in states.values()) for state in ('full', 'empty', 'unknown'))
        names = {'full': 'occupied' if en else 'có hàng', 'empty': 'empty' if en else 'trống', 'unknown': 'unknown' if en else 'chưa rõ'}
        detail = ', '.join(f'{s}: {names[v]}' for s,v in states.items())
        prefix = (f'{full}/{len(states)} known occupied, {empty} empty, {unknown} unknown slots. ' if en else
                  f'{full}/{len(states)} ô được xác nhận có hàng, {empty} ô trống, {unknown} ô chưa rõ. ')
        if unknown:
            prefix += ('Cannot give an exact total with missing/stale sensor data. ' if en else
                       'Chưa thể chốt tổng số vì dữ liệu cảm biến thiếu/cũ hoặc ô chưa xác định. ')
        replies.append(prefix + detail + ('. Counts occupied pallet positions, not units inside boxes.' if en else
                                        '. Đây là số vị trí pallet có hàng, không phải số sản phẩm bên trong hộp.'))
    if 'mission' in topics:
        if context['active_owner'] == 'retrieval':
            r = context['retrieval']
            replies.append(f"Retrieval {r['slot'] or ''}: {r['state']}. {r['message']} " +
                           ('Retrieval Resume requires a valid operation checkpoint.' if en else 'Retrieval chỉ Resume khi checkpoint thao tác còn hợp lệ.'))
        elif not context['supervisor_fresh']:
            replies.append('Mission status is unavailable or stale.' if en else 'Trạng thái mission chưa có hoặc đã mất cập nhật.')
        else:
            cycle = context['cycle']
            state = context['supervisor_state']
            handling = context['handling_status']
            progress = context['progress_percent']
            placed = number(cycle.get('placed'), 0)
            target = number(cycle.get('target_quantity'), 1)
            line = f"Mission: {state}; {context['active_mission'] or '--'} → {context['active_destination'] or '--'}."
            if progress is not None: line += (f' Current cycle progress: {progress:g}%.' if en else f' Tiến độ chu kỳ hiện tại: {progress:g}%.')
            if placed is not None: line += (f' Placed: {placed:g}' if en else f' Đã đặt: {placed:g}') + (f'/{target:g}.' if target else '.')
            if context['mission_message']: line += ' ' + context['mission_message']
            if handling == 'AUTO_RECOVERING':
                line += (' The pallet controller is recovering automatically or holding zero while conditions are unsafe.' if en else
                         ' Bộ điều khiển pallet đang tự phục hồi hoặc giữ vận tốc zero khi chưa đủ điều kiện an toàn.')
                err, retries = context['pm_err'] or {}, context['pm_retries'] or {}
                detail = [context['pm_phase'],
                          f"lateral {err['lat_cm']} cm" if isinstance(err.get('lat_cm'), (int, float)) else None,
                          f"yaw {err['yaw_deg']}°" if isinstance(err.get('yaw_deg'), (int, float)) else None,
                          f"retry {retries['alignment']}" if isinstance(retries.get('alignment'), int) else None,
                          context['pm_message']]
                line += ' ' + '; '.join(str(value) for value in detail if value) + '.'
            elif handling == 'WAITING_FOR_SENSOR':
                sensor = context['pm_waiting_for'] or 'sensor'
                line += (f' Robot stopped while waiting for fresh {sensor} data.' if en else
                         f' Robot đang dừng chờ dữ liệu {sensor} mới.')
            elif handling == 'NEED_OPERATOR':
                line += (' Operator inspection is required; the pallet operation is not complete.' if en else
                         ' Cần người vận hành kiểm tra; thao tác pallet chưa hoàn tất.')
            elif handling == 'FAILED':
                line += (' The operation failed and was not reported complete.' if en else
                         ' Thao tác thất bại, chưa được xác nhận hoàn tất.')
            if state == 'RECOVERY':
                line += (' Resume checkpoint: ' if en else ' Checkpoint resume: ') + str(context['resume_state'] or '--') + '.'
                line += (' Resume requires checking robot/cargo/lift first.' if context['resume_available'] else ' No resumable checkpoint.') if en else (
                    ' Chỉ Resume sau khi kiểm tra robot/pallet/lift; đây không phải xác nhận an toàn.' if context['resume_available'] else ' Không có checkpoint để Resume.')
            replies.append(line)
        if context['pallet_state'] in ('FAILED', 'STOPPED') and context['pallet_message']:
            replies.append(f"Pallet {context['pallet_state']} ({context['pallet_fail_phase'] or '--'}): {context['pallet_message']}")
        r = context['retrieval']
        if r['fresh'] and r['recovery_required'] and context['active_owner'] != 'retrieval':
            replies.append(('Last retrieval needs inspection: ' if en else 'Retrieval trước cần kiểm tra: ') + r['message'])
    if 'cargo' in topics:
        cargo = context['cargo']
        replies.append(('Cargo is unknown; telemetry is missing/stale.' if en else 'Chưa xác định hàng trên càng vì dữ liệu thiếu/cũ.') if cargo is None else
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
                           'Nav2: dữ liệu điều hướng thiếu/cũ; chưa xác định có goal đang chạy hay không.')
        else:
            replies.append(('Nav2: a navigation goal is active.' if active else 'Nav2: no active navigation goal is reported.') if en else
                           ('Nav2: đang có goal điều hướng hoạt động.' if active else 'Nav2: hiện báo không có goal điều hướng hoạt động.'))
        replies.append('Goal status does not establish that every Nav2 lifecycle node is ready.' if en else
                       'Trạng thái goal chưa đủ để xác nhận toàn bộ node lifecycle Nav2 sẵn sàng.')
    if 'motion' in topics:
        stopped = context['robot_stopped']
        if stopped is None:
            replies.append('Motion is unknown: fresh odometry is unavailable.' if en else
                           'Chưa xác định xe đang di chuyển hay đứng yên vì thiếu odometry còn mới.')
        else:
            replies.append(('The robot is stationary according to fresh odometry.' if stopped else 'The robot is moving according to fresh odometry.') if en else
                           ('Xe đang đứng yên theo odometry còn mới.' if stopped else 'Xe đang di chuyển theo odometry còn mới.'))
    if 'eta' in topics:
        replies.append('I cannot estimate mission completion time: the system has no validated remaining-time estimate. Progress percentage alone is insufficient.' if en else
                       'Chưa thể ước lượng còn bao lâu hoàn thành mission: hệ thống chưa có dữ liệu thời gian còn lại đáng tin cậy. Phần trăm tiến độ chưa đủ để suy ra thời gian.')
    if 'battery_runtime' in topics:
        replies.append('I cannot estimate remaining operating time from battery percentage alone; discharge history and power consumption are unavailable.' if en else
                       'Chưa thể ước lượng xe chạy thêm bao lâu chỉ từ phần trăm pin; hiện chưa có lịch sử xả pin và mức tiêu thụ điện để tính thời lượng còn lại.')
    if 'battery' in topics:
        battery = context['battery_percent']
        replies.append(('Battery data is unavailable or stale.' if en else 'Chưa có dữ liệu pin còn mới.')
                       if battery is None else
                       (f'Battery: {battery:g}%.' if en else f'Pin còn {battery:g}%.'))
    if 'health' in topics:
        names = {True:'ready' if en else 'sẵn sàng', False:'not ready' if en else 'chưa sẵn sàng', None:'unknown' if en else 'chưa rõ'}
        line = ('Sensors: ' if en else 'Thiết bị: ') + ', '.join(
            f'{key}: {names.get(value, names[None])}' for key, value in context['health'].items())
        if re.search(r'\b(?:tai sao|vi sao|why)\b', t):
            line += (' These are reported checks; a specific root cause is not available from these flags alone.' if en else
                     ' Đây là các kiểm tra được báo về; riêng các cờ này chưa đủ xác định nguyên nhân cụ thể.')
        replies.append(line)
    if 'capabilities' in topics:
        replies.append('I can report inventory by slot/type, cargo, fork height, battery, mission progress and recovery. I can plan approved navigation, detection, pick/place, Auto/single cycles and retrieval from A1/A2/B1/B2. Questions can run alongside a mission; physical actions are sequenced with confirmation. Retrieval Resume requires a valid operation checkpoint.' if en else
                       'Tôi có thể báo tồn kho theo ô/loại, cargo, độ cao càng, pin, tiến độ và recovery; lập kế hoạch đi điểm, detect/chốt pallet, lấy/đặt, Auto/single cycle và xuất kho A1/A2/B1/B2. Có thể hỏi khi xe đang làm việc; thao tác cơ khí chạy theo thứ tự và cần xác nhận. Retrieval chỉ Resume khi checkpoint thao tác còn hợp lệ.')
    return '\n'.join(replies)
