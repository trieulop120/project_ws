// resources/web/js/llm_voice.js
// Trợ lý LLM điều khiển robot bằng giọng nói / chữ (song ngữ EN / VI).
// - Nhúng vào khung "AI Assistant" (.ai-chat-box) ở trang Navigation; nếu không thấy khung đó
//   thì tự tạo cửa sổ nổi (nút 🎙️ góc phải dưới).
// - Dùng lại sendCommand() / WebSocket sẵn có. Nhận trạng thái qua window.handleLLMStatus().
// - KHÔNG có nút dừng khẩn riêng: dashboard đã có nút dừng tổng.
// Nạp SAU navigation.js:  <script src="js/llm_voice.js"></script>
(function () {
    'use strict';

    // ============================================
    // Cấu hình
    // ============================================
    const REQUIRE_NAV = true;    // phải bật Navigation (Nav2) mới cho LLM chạy lệnh
    const REQUIRE_AUTO = true;   // phải ở chế độ AUTO (tránh xung đột với teleop tay)
    const LOCATIONS = ['home', 'p1', 'p2', 'd1', 'unknown'];
    const STOP_RE = /^(dừng|dung|stop|khẩn cấp|emergency)(\s|$)/i;
    const CANCEL_RE = /^(hủy|huỷ|không|thôi|cancel|no)(\s|$)/i;
    const YES_RE = /^(xác nhận|đồng ý|ok|okay|oke|chạy đi|làm đi|được|confirm|yes|go ahead)(\s|$)/i;
    const WATCHDOG_MS = 6000;

    // ============================================
    // Từ điển song ngữ - EN là mặc định
    // ============================================
    const I18N = {
        en: {
            status: {
                planning: 'Thinking…', waiting_confirm: 'Waiting for confirmation',
                running: 'Running', done: 'Done', failed: 'Failed', error: 'Error',
                clarify: 'Robot asks', stopped: 'Stopped', noop: 'Nothing to do',
                cancelled: 'Cancelled', idle: 'Ready', busy: 'Busy'
            },
            welcome: "LLM Assistant is ready. Example: 'pick pallet at P1 and place on level 2 rack'. Say 'stop' for emergency stop.",
            httpWarn: 'HTTP connection detected: Browser restricts microphone access. Please use HTTPS, localhost, or mobile voice keyboard.',
            manualWarn: 'Robot is currently in MANUAL mode. Please switch to AUTO mode first.',
            navWarn: 'Navigation (Nav2) is not running. Please start Navigation first.',
            placeholder: 'Type your command...',
            micTitle: 'Tap to speak',
            gearTitle: 'Settings',
            sendTitle: 'Send',
            langTitle: 'Language',
            optAuto: 'Auto-send when done speaking',
            optTts: 'Read responses aloud',
            optVconfirm: 'Confirm by voice',
            locLabel: 'Robot location:',
            goHome: 'Return to HOME',
            goRoute: 'Go route ',
            arrow: ' → ',
            pick: 'Detect and pick pallet',
            drop: 'Place pallet on shelf',
            lift: 'Lift height: ',
            planTitle: 'Plan:',
            btnYes: 'Confirm',
            btnNo: 'Cancel',
            okYes: '✔ confirmed',
            okNo: '✖ cancelled',
            noteNew: '(replaced by a new plan)',
            noteDone: '(handled)',
            step: 'Step',
            speakConfirm: 'Say confirm to run, or cancel.',
            wsDown: 'Not connected to server (WebSocket).',
            wsTimeout: 'No response from the LLM node. Is llm_command_node running? (ros2 launch amr_mission_manager llm_system.launch.py)',
            noMic: 'Browser does not support speech recognition (use Chrome, Edge or Safari).',
            micDenied: 'Microphone permission denied (or page is not secure - needs https).',
            micNet: 'Speech recognition needs an Internet connection.',
            micErr: 'Microphone: '
        },
        vi: {
            status: {
                planning: 'Đang suy nghĩ…', waiting_confirm: 'Chờ bạn xác nhận',
                running: 'Đang chạy', done: 'Hoàn thành', failed: 'Thất bại',
                error: 'Lỗi', clarify: 'Robot hỏi lại', stopped: 'Đã dừng',
                noop: 'Không có hành động', cancelled: 'Đã hủy',
                idle: 'Sẵn sàng', busy: 'Đang bận'
            },
            welcome: "Trợ lý LLM sẵn sàng. Ví dụ: 'đi lấy pallet ở P1 rồi đưa lên kệ tầng 2'. Nói 'dừng' để dừng khẩn.",
            httpWarn: 'Trang chạy HTTP nên trình duyệt chặn micro. Mở bằng https (Tailscale) hoặc localhost, hoặc dùng nút micro trên bàn phím điện thoại.',
            manualWarn: 'Robot đang ở chế độ MANUAL. Hãy chuyển sang AUTO trước.',
            navWarn: 'Chưa bật Navigation (Nav2). Hãy Start Navigation trước.',
            placeholder: 'Nhập lệnh của bạn...',
            micTitle: 'Chạm để nói',
            gearTitle: 'Cài đặt',
            sendTitle: 'Gửi',
            langTitle: 'Ngôn ngữ',
            optAuto: 'Tự gửi khi nói xong',
            optTts: 'Đọc phản hồi',
            optVconfirm: 'Xác nhận bằng giọng',
            locLabel: 'Vị trí robot:',
            goHome: 'Quay về HOME',
            goRoute: 'Đi tuyến ',
            arrow: ' → ',
            pick: 'Nhận diện và lấy pallet',
            drop: 'Đặt pallet lên kệ',
            lift: 'Nâng/hạ càng: ',
            planTitle: 'Kế hoạch:',
            btnYes: 'Xác nhận',
            btnNo: 'Hủy',
            okYes: '✔ đã xác nhận',
            okNo: '✖ đã hủy',
            noteNew: '(đã thay bằng kế hoạch mới)',
            noteDone: '(đã xử lý)',
            step: 'Bước',
            speakConfirm: 'Nói xác nhận để chạy, hoặc hủy.',
            wsDown: 'Chưa kết nối tới server (WebSocket).',
            wsTimeout: 'Chưa thấy phản hồi từ node LLM. llm_command_node đã chạy chưa? (ros2 launch amr_mission_manager llm_system.launch.py)',
            noMic: 'Trình duyệt không hỗ trợ nhận giọng nói (dùng Chrome, Edge hoặc Safari).',
            micDenied: 'Chưa cấp quyền micro (hoặc trang không an toàn - cần https).',
            micNet: 'Nhận giọng nói cần Internet.',
            micErr: 'Micro: '
        }
    };
    const LANGS = [{ id: 'en', label: 'EN' }, { id: 'vi', label: 'VI' }];
    const DEFAULT_LANG = 'en';

    // ============================================
    // Trạng thái
    // ============================================
    let state = '';
    let listening = false, speaking = false, rec = null;
    let planBubble = null, curSteps = [], lastStep = -1, watchdog = null;
    let lang = DEFAULT_LANG;
    let lastStatus = null;              // để dựng lại nhãn trạng thái khi đổi ngôn ngữ
    const el = {};   // tham chiếu tới các phần tử giao diện
    const SR = window.SpeechRecognition || window.webkitSpeechRecognition;

    // Bộ icon SVG công nghiệp đồng bộ với hệ thống dashboard
    const ICONS = {
        mic: `<svg viewBox="0 0 24 24" width="15" height="15" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 1a3 3 0 0 0-3 3v8a3 3 0 0 0 6 0V4a3 3 0 0 0-3-3z"/><path d="M19 10v2a7 7 0 0 1-14 0v-2"/><line x1="12" y1="19" x2="12" y2="23"/><line x1="8" y1="23" x2="16" y2="23"/></svg>`,
        micActive: `<svg viewBox="0 0 24 24" width="15" height="15" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><path d="M12 1a3 3 0 0 0-3 3v8a3 3 0 0 0 6 0V4a3 3 0 0 0-3-3z" fill="currentColor"/><path d="M19 10v2a7 7 0 0 1-14 0v-2"/><line x1="12" y1="19" x2="12" y2="23"/><line x1="8" y1="23" x2="16" y2="23"/></svg>`,
        send: `<svg viewBox="0 0 24 24" width="15" height="15" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><line x1="22" y1="2" x2="11" y2="13"/><polygon points="22 2 15 22 11 13 2 9 22 2"/></svg>`,
        gear: `<svg viewBox="0 0 24 24" width="15" height="15" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><circle cx="12" cy="12" r="3"/><path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z"/></svg>`
    };

    // ============================================
    // Tiện ích
    // ============================================
    function T() { return I18N[lang] || I18N[DEFAULT_LANG]; }

    function h(tag, cls, text) {
        const e = document.createElement(tag);
        if (cls) e.className = cls;
        if (text !== undefined) e.textContent = text;
        return e;
    }
    function send(cmd) {
        if (typeof sendCommand === 'function') return sendCommand(cmd);
        return false;
    }
    function toast(msg, type) {
        if (typeof window.showToast === 'function') window.showToast(msg, type || 'info');
    }
    function activity(type, msg) {
        if (typeof addActivity === 'function') addActivity(type, msg);
    }
    function loadPref(k, def) {
        try { const v = localStorage.getItem('llm_' + k); return v === null ? def : v === '1'; } catch (e) { return def; }
    }
    function savePref(k, v) { try { localStorage.setItem('llm_' + k, v ? '1' : '0'); } catch (e) { /* bỏ qua */ } }
    function loadStr(k, def) {
        try { const v = localStorage.getItem('llm_' + k); return v === null ? def : v; } catch (e) { return def; }
    }
    function saveStr(k, v) { try { localStorage.setItem('llm_' + k, v); } catch (e) { /* bỏ qua */ } }

    // ============================================
    // Giao diện
    // ============================================
    function injectCss() {
        if (document.getElementById('llmStyles')) return;
        const css = h('style');
        css.id = 'llmStyles';
        css.textContent = `
        .llm-msg{margin:0 0 6px;padding:6px 9px;border-radius:8px;line-height:1.35;word-break:break-word;max-width:94%;
          background:var(--bg-panel-secondary,#12181A);border:1px solid var(--border,#2A363B);color:var(--text-primary,#E2E8F0)}
        .llm-msg.user{margin-left:auto;background:rgba(85,221,187,.14);border-color:var(--accent,#55DDBB)}
        .llm-msg.sys{background:none;border:0;padding:0 2px;color:var(--text-muted,#64748B);font-size:.68rem;max-width:100%}
        .llm-msg.err{border-color:var(--error,#F16C70);color:var(--error,#F16C70)}
        .llm-msg.sys.err{border:0}
        .llm-plan ol{margin:4px 0 8px;padding-left:18px}
        .llm-row{display:flex;gap:6px;align-items:center}
        .llm-ctrl{display:flex;flex-direction:column;gap:6px;flex-shrink:0;flex-grow:0}
        .llm-head{display:flex;align-items:center;justify-content:space-between;gap:8px;min-height:1.5em}
        .llm-status{font-size:.68rem;color:var(--text-secondary,#94A3B3);flex:1;min-width:0;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
        .llm-lang{background:var(--bg-primary,#0F1416);color:var(--text-secondary,#94A3B3);border:1px solid var(--border,#2A363B);
          border-radius:4px;font:inherit;font-size:.62rem;padding:1px 4px;cursor:pointer;flex-shrink:0}
        .llm-lang:hover{border-color:var(--accent,#55DDBB);color:var(--accent,#55DDBB)}
        .llm-in{flex:1;min-width:0;background:var(--bg-primary,#0F1416);color:var(--text-primary,#E2E8F0);
          border:1px solid var(--border,#2A363B);border-radius:6px;padding:8px;font:inherit}
        .llm-btn{border:1px solid var(--border,#2A363B);background:var(--bg-panel-secondary,#12181A);color:var(--text-primary,#E2E8F0);
          border-radius:6px;padding:7px 10px;cursor:pointer;font:inherit;flex-shrink:0;display:inline-flex;align-items:center;justify-content:center;transition:all .15s ease}
        .llm-btn svg{width:15px;height:15px;transition:transform .15s ease}
        .llm-btn:hover{border-color:var(--accent,#55DDBB);color:var(--accent,#55DDBB)}
        .llm-btn:hover svg{transform:scale(1.1)}
        .llm-btn.ok{background:var(--accent,#55DDBB);color:#000;border-color:var(--accent,#55DDBB);font-weight:600}
        .llm-btn.live{background:var(--error,#F16C70);color:#fff;border-color:var(--error,#F16C70);animation:llmPulse 1.2s infinite}
        @keyframes llmPulse{50%{opacity:.6}}
        .llm-set{display:none;flex-wrap:wrap;gap:4px 12px;font-size:.68rem;color:var(--text-secondary,#94A3B3)}
        .llm-set.open{display:flex}
        .llm-set label{display:flex;align-items:center;gap:4px;cursor:pointer}
        .llm-set select{background:var(--bg-primary,#0F1416);color:var(--text-primary,#E2E8F0);border:1px solid var(--border,#2A363B);border-radius:4px;font:inherit}
        #llmFab{position:fixed;right:18px;bottom:18px;z-index:9000;width:50px;height:50px;border-radius:50%;border:1px solid var(--accent,#55DDBB);
          background:var(--bg-panel,#182023);color:var(--accent,#55DDBB);cursor:pointer;box-shadow:0 4px 14px rgba(0,0,0,.4);
          display:flex;align-items:center;justify-content:center;transition:transform .2s ease,background .2s ease}
        #llmFab:hover{transform:scale(1.05)}
        #llmFab svg{width:22px;height:22px}
        #llmFab.live{background:var(--error,#F16C70);color:#fff;border-color:var(--error,#F16C70);animation:llmPulse 1.2s infinite}
        #llmFloat{position:fixed;right:18px;bottom:82px;z-index:9000;width:min(380px,calc(100vw - 36px));height:min(460px,70vh);
          display:flex;flex-direction:column;padding:10px;background:var(--bg-panel,#182023);border:1px solid var(--border,#2A363B);
          border-radius:var(--radius-lg,8px);box-shadow:0 8px 30px rgba(0,0,0,.45);font-size:.75rem}
        #llmFloat[hidden]{display:none}
        #llmFloat .llm-chat{flex:1;min-height:0;overflow-y:auto;background:var(--bg-primary,#0F1416);border:1px solid var(--border,#2A363B);border-radius:6px;padding:10px}
        `;
        document.head.appendChild(css);
    }

    function buildControls() {
        const ctrl = h('div', 'llm-ctrl');

        // --- hàng trạng thái: nhãn + ô chọn ngôn ngữ ---
        const head = h('div', 'llm-head');
        el.status = h('div', 'llm-status');
        el.langsel = h('select', 'llm-lang');
        LANGS.forEach((l) => {
            const o = h('option', null, l.label);
            o.value = l.id;
            el.langsel.appendChild(o);
        });
        el.langsel.value = lang;
        el.langsel.title = T().langTitle;
        el.langsel.onchange = () => setLang(el.langsel.value);
        head.appendChild(el.status);
        head.appendChild(el.langsel);

        // --- hàng điều khiển: [Mic] [Ô nhập] [Send] [Settings] ---
        const row = h('div', 'llm-row');
        el.mic = h('button', 'llm-btn');
        el.mic.innerHTML = ICONS.mic;
        el.input = h('input', 'llm-in');
        el.input.type = 'text';
        el.input.autocomplete = 'off';
        el.send = h('button', 'llm-btn');
        el.send.innerHTML = ICONS.send;
        el.gear = h('button', 'llm-btn');
        el.gear.innerHTML = ICONS.gear;
        [el.mic, el.input, el.send, el.gear].forEach((x) => row.appendChild(x));

        // --- hàng cài đặt ---
        el.set = h('div', 'llm-set');
        const mk = (key, text, def) => {
            const lb = h('label');
            const cb = h('input');
            cb.type = 'checkbox';
            cb.checked = loadPref(key, def);
            cb.onchange = () => savePref(key, cb.checked);
            lb.appendChild(cb);
            lb.appendChild(document.createTextNode(text));
            el.set.appendChild(lb);
            return cb;
        };
        el.auto = mk('auto', T().optAuto, true);
        el.tts = mk('tts', T().optTts, true);
        el.vconfirm = mk('vconfirm', T().optVconfirm, true);
        const lbl = h('label', null, T().locLabel);
        el.locsel = h('select');
        LOCATIONS.forEach((l) => { const o = h('option', null, l); o.value = l; el.locsel.appendChild(o); });
        el.locsel.onchange = () => send({ command: 'llm_set_location', loc: el.locsel.value });
        lbl.appendChild(el.locsel);
        el.set.appendChild(lbl);

        ctrl.appendChild(head);
        ctrl.appendChild(row);
        ctrl.appendChild(el.set);
        return ctrl;
    }

    function mount() {
        injectCss();
        const box = document.querySelector('.ai-chat-box');
        const ctrl = buildControls();
        if (box) {
            // Nhúng vào khung AI Assistant có sẵn: .ai-chat-box là danh sách tin nhắn
            el.chat = box;
            box.innerHTML = '';
            box.insertAdjacentElement('afterend', ctrl);
        } else {
            const fab = h('button', null);
            fab.id = 'llmFab';
            fab.innerHTML = ICONS.mic;
            fab.title = 'Trợ lý giọng nói (LLM)';
            const fl = h('div');
            fl.id = 'llmFloat';
            fl.hidden = true;
            el.chat = h('div', 'llm-chat');
            fl.appendChild(el.chat);
            fl.appendChild(ctrl);
            document.body.appendChild(fl);
            document.body.appendChild(fab);
            el.fab = fab;
            fab.onclick = () => { fl.hidden = !fl.hidden; };
            el.float = fl;
        }
        el.mic.onclick = () => {
            if (listening && rec) { rec.stop(); return; }
            if (window.speechSynthesis) window.speechSynthesis.cancel();
            speaking = false;
            startListen();
        };
        el.send.onclick = () => processText(el.input.value, false);
        el.input.addEventListener('keydown', (e) => { if (e.key === 'Enter') processText(el.input.value, false); });
        el.gear.onclick = () => el.set.classList.toggle('open');
        applyLang(false);
        addMsg('sys', T().welcome);
        if (!window.isSecureContext) {
            addMsg('sys', T().httpWarn, 'err');
        }
    }

    // ============================================
    // Đổi ngôn ngữ
    // ============================================
    function setLang(id) {
        lang = (I18N[id] ? id : DEFAULT_LANG);
        saveStr('lang', lang);
        applyLang(true);
    }

    function applyLang(speakIt) {
        const t = T();
        if (el.input) el.input.placeholder = t.placeholder;
        if (el.mic) el.mic.title = t.micTitle;
        if (el.gear) el.gear.title = t.gearTitle;
        if (el.send) el.send.title = t.sendTitle;
        if (el.langsel) {
            el.langsel.value = lang;
            el.langsel.title = t.langTitle;
        }
        // nhãn các tuỳ chọn trong hàng cài đặt
        if (el.set) {
            const texts = [t.optAuto, t.optTts, t.optVconfirm];
            const boxes = el.set.querySelectorAll('label');
            boxes.forEach((lb, i) => {
                if (texts[i] === undefined) return;
                const cb = lb.querySelector('input');      // giữ lại checkbox
                lb.textContent = '';
                if (cb) lb.appendChild(cb);
                lb.appendChild(document.createTextNode(texts[i]));
            });
            // nhãn "Vị trí robot:" nằm sau 3 checkbox, giữ nguyên <select>
            const locLb = el.set.querySelector('label:last-of-type');
            if (locLb && el.locsel && el.locsel.parentNode === locLb) {
                const sel = el.locsel;
                locLb.textContent = '';
                locLb.appendChild(document.createTextNode(t.locLabel));
                locLb.appendChild(sel);
            }
        }
        renderStatus();
        if (speakIt) {
            addMsg('sys', lang === 'vi' ? 'Đã chuyển sang Tiếng Việt.' : 'Switched to English.');
        }
    }

    function renderStatus() {
        if (!el.status) return;
        const t = T();
        const s = lastStatus;
        if (!s) { el.status.textContent = t.status.idle; return; }
        el.status.textContent = (t.status[s.state] || s.state) +
            (s.state === 'running' ? ` (${s.step}/${s.total})` : '') +
            (s.location ? '  ·  ' + t.locLabel.replace(/:$/, '') + ': ' + s.location : '');
    }

    // Ô chat có thể cuộn: chỉ tự cuộn xuống đáy khi người dùng đang ở gần đáy
    // (hoặc vừa cuộn lên), nếu không thì giữ nguyên vị trí để người dùng đọc lại.
    function atBottom() {
        if (!el.chat) return true;
        return el.chat.scrollHeight - el.chat.scrollTop - el.chat.clientHeight < 40;
    }

    function scrollToBottom(force) {
        if (!el.chat) return;
        if (force === true || atBottom()) {
            el.chat.scrollTop = el.chat.scrollHeight;
        }
    }

    function addMsg(kind, text, extra) {
        const stick = atBottom();
        const m = h('div', 'llm-msg ' + kind + (extra ? ' ' + extra : ''), text);
        el.chat.appendChild(m);
        while (el.chat.children.length > 80) el.chat.removeChild(el.chat.firstChild);
        scrollToBottom(stick);
        return m;
    }

    // ============================================
    // Kế hoạch + xác nhận
    // ============================================
    function describe(s) {
        const a = (s && s.args) || {};
        if (!s) return '';
        const t = T();
        if (s.action === 'go_mission') return a.name === 'return_home' ? t.goHome : t.goRoute + String(a.name).replace('_to_', t.arrow);
        if (s.action === 'pick_pallet') return t.pick;
        if (s.action === 'drop_pallet') return t.drop;
        if (s.action === 'lift_to') return t.lift + a.level;
        return s.action;
    }

    function closePlan(note) {
        if (!planBubble) return;
        planBubble.row.remove();
        if (note) planBubble.el.appendChild(h('div', 'llm-msg sys', note));
        planBubble = null;
    }

    function showPlan(s) {
        closePlan(T().noteNew);
        curSteps = s.steps || [];
        const b = h('div', 'llm-msg llm-plan');
        b.appendChild(h('div', null, s.msg || T().planTitle));
        const ol = h('ol');
        curSteps.forEach((x) => ol.appendChild(h('li', null, describe(x))));
        b.appendChild(ol);
        const row = h('div', 'llm-row');
        const yes = h('button', 'llm-btn ok', T().btnYes);
        const no = h('button', 'llm-btn', T().btnNo);
        yes.onclick = () => { send({ command: 'llm_confirm', ok: true }); closePlan(T().okYes); };
        no.onclick = () => { send({ command: 'llm_confirm', ok: false }); closePlan(T().okNo); };
        row.appendChild(yes);
        row.appendChild(no);
        b.appendChild(row);
        el.chat.appendChild(b);
        scrollToBottom();
        planBubble = { el: b, row: row };
    }

    // ============================================
    // Điều kiện an toàn + gửi lệnh
    // ============================================
    function preflight() {
        if (REQUIRE_NAV && !window.isNavigationActive) return T().navWarn;
        if (REQUIRE_AUTO && window.currentOpMode !== 'AUTO') return T().manualWarn;
        return null;
    }

    function stopAll() {
        if (window.speechSynthesis) window.speechSynthesis.cancel();
        speaking = false;
        if (rec && listening) { try { rec.abort(); } catch (e) { /* bỏ qua */ } }
        clearTimeout(watchdog);
        // Dùng đúng đường dừng khẩn của dashboard + bảo node LLM hủy kế hoạch
        send({ command: 'nav_cancel' });
        send({ command: 'emergency_stop' });
        send({ command: 'llm_stop' });
        addMsg('user', '⛔ DỪNG KHẨN');
        activity('warn', 'LLM: dừng khẩn');
    }

    function sendLLM(text) {
        const problem = preflight();
        if (problem) { addMsg('sys', '⚠ ' + problem, 'err'); toast(problem, 'warn'); speak(problem); return; }
        addMsg('user', text);
        activity('info', 'LLM lệnh: ' + text);
        if (!send({ command: 'llm_command', text: text })) {
            addMsg('sys', T().wsDown, 'err');
            return;
        }
        clearTimeout(watchdog);
        watchdog = setTimeout(() => addMsg('sys', T().wsTimeout, 'err'), WATCHDOG_MS);
    }

    // fromVoice: true nếu là câu nói (xác nhận bằng giọng chỉ hoạt động khi bật tùy chọn)
    function processText(raw, fromVoice) {
        const text = (raw || '').trim();
        if (!text) return;
        el.input.value = '';
        const t = text.toLowerCase().replace(/[.,!?]/g, '').trim();
        const waiting = state === 'waiting_confirm';
        if (STOP_RE.test(t)) { stopAll(); return; }
        if (waiting && (!fromVoice || el.vconfirm.checked)) {
            if (YES_RE.test(t)) { addMsg('user', text); send({ command: 'llm_confirm', ok: true }); closePlan(T().okYes); return; }
            if (CANCEL_RE.test(t)) { addMsg('user', text); send({ command: 'llm_confirm', ok: false }); closePlan(T().okNo); return; }
        }
        if (state === 'running' && CANCEL_RE.test(t)) { stopAll(); return; }   // "hủy/thôi" khi đang chạy = dừng
        sendLLM(text);
    }

    // ============================================
    // Giọng nói
    // ============================================
    function speak(text, cb) {
        if (!el.tts || !el.tts.checked || !text || !window.speechSynthesis) { if (cb) cb(); return; }
        window.speechSynthesis.cancel();
        const u = new SpeechSynthesisUtterance(text);
        u.lang = lang === 'vi' ? 'vi-VN' : 'en-US';
        speaking = true;
        u.onend = u.onerror = () => { speaking = false; if (cb) cb(); };
        window.speechSynthesis.speak(u);
    }

    function setMic(on) {
        listening = on;
        el.mic.classList.toggle('live', on);
        el.mic.innerHTML = on ? ICONS.micActive : ICONS.mic;
        if (el.fab) {
            el.fab.classList.toggle('live', on);
            el.fab.innerHTML = on ? ICONS.micActive : ICONS.mic;
        }
    }

    function startListen() {
        if (!SR) { addMsg('sys', T().noMic, 'err'); return; }
        if (listening || speaking) return;
        rec = new SR();
        rec.lang = lang === 'vi' ? 'vi-VN' : 'en-US';
        rec.interimResults = true;
        rec.continuous = false;
        rec.maxAlternatives = 1;
        rec.onresult = (e) => {
            let t = '', fin = false;
            for (let i = e.resultIndex; i < e.results.length; i++) {
                t += e.results[i][0].transcript;
                if (e.results[i].isFinal) fin = true;
            }
            el.input.value = t;
            if (fin) { if (el.auto.checked) processText(t, true); }
        };
        rec.onerror = (e) => {
            const t = T();
            if (e.error === 'not-allowed') addMsg('sys', t.micDenied, 'err');
            else if (e.error === 'network') addMsg('sys', t.micNet, 'err');
            else if (e.error !== 'no-speech' && e.error !== 'aborted') addMsg('sys', t.micErr + e.error, 'err');
        };
        rec.onend = () => setMic(false);
        try { rec.start(); setMic(true); } catch (e) { /* đang nghe rồi */ }
    }

    // ============================================
    // Nhận trạng thái từ backend (ros_connection.js gọi)
    // ============================================
    window.handleLLMStatus = function (msg) {
        const s = (msg && msg.data) ? msg.data : msg;
        if (!s || !s.state || !el.chat) return;
        clearTimeout(watchdog);
        const prev = state;
        state = s.state;
        lastStatus = s;

        renderStatus();
        if (LOCATIONS.includes(s.location)) el.locsel.value = s.location;

        if (s.state !== 'waiting_confirm') closePlan(T().noteDone);

        if (s.state === 'running') {
            if (s.step !== lastStep) {
                lastStep = s.step;
                const d = describe(curSteps[s.step - 1]) || s.msg || '';
                addMsg('sys', `▶ ${T().step} ${s.step}/${s.total}: ${d}`);
            }
        } else {
            lastStep = -1;
        }
        if (prev === s.state) return;

        const t = T();
        switch (s.state) {
            case 'waiting_confirm':
                showPlan(s);
                speak((s.msg || '') + ' ' + t.speakConfirm, () => { if (el.vconfirm.checked) startListen(); });
                break;
            case 'clarify':
                addMsg('robot', '❓ ' + s.msg);
                speak(s.msg, startListen);
                break;
            case 'done':
                addMsg('robot', '✔ ' + t.status.done + '.');
                speak(t.status.done + '.');
                activity('success', 'LLM: ' + t.status.done);
                break;
            case 'stopped':
                addMsg('robot', '⛔ ' + t.status.stopped + '.');
                speak(t.status.stopped + '.');
                break;
            case 'cancelled':
                addMsg('robot', t.status.cancelled + '.');
                break;
            case 'noop':
                addMsg('robot', s.msg || t.status.noop + '.');
                speak(s.msg);
                break;
            case 'failed':
            case 'error':
                addMsg('robot', '✖ ' + (s.msg || t.status.error), 'err');
                speak(t.status.error + '. ' + (s.msg || ''));
                activity('error', 'LLM: ' + (s.msg || s.state));
                toast(s.msg || t.status.error, 'error');
                break;
            case 'busy':
                addMsg('sys', s.msg || t.status.busy + '.');
                break;
            default:
                break;
        }
    };

    // ============================================
    // Khởi tạo
    // ============================================
    lang = (function () {
        const v = loadStr('lang', DEFAULT_LANG);
        return I18N[v] ? v : DEFAULT_LANG;
    })();

    function init() { mount(); }
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
})();
