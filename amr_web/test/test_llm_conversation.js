// Run the shipped chat handlers with a local DOM, WebSocket and speech sink.
const GLib = imports.gi.GLib;
const scriptPath = ARGV[0] || 'src/amr_web/resources/web/js/llm_voice.js';
const source = new TextDecoder().decode(GLib.file_get_contents(scriptPath)[1]);
function assert(ok, message) { if (!ok) throw Error(message); }

class Element {
    constructor(tag) {
        this.tag = tag;
        this.children = [];
        this.style = {};
        this.textContent = '';
        this.className = '';
        this.classList = { toggle() {} };
    }
    appendChild(e) { e.remove(); this.children.push(e); e.parentNode = this; return e; }
    remove() {
        if (this.parentNode) this.parentNode.children.splice(this.parentNode.children.indexOf(this), 1);
        this.parentNode = null;
    }
    replaceChildren() { for (const e of this.children.slice()) e.remove(); }
    insertAdjacentElement(_, e) { this.parentNode.appendChild(e); }
    addEventListener() {}
    find(test) {
        if (test(this)) return this;
        for (const e of this.children) { const match = e.find(test); if (match) return match; }
        return null;
    }
    querySelectorAll(tag) { return this.children.filter(e => e.tag === tag); }
    querySelector(tag) {
        return tag === 'label:last-of-type' ? this.children.at(-1) : this.find(e => e.tag === tag);
    }
    get words() { return [this.textContent, ...this.children.map(e => e.words)].join(' '); }
}

for (const lang of ['vi', 'en']) {
    const body = new Element('body');
    const chat = body.appendChild(new Element('div'));
    const document = { body, head: new Element('head'), readyState: 'complete',
        querySelector: () => chat, getElementById: () => null,
        createElement: t => new Element(t),
        createTextNode(text) { const n = new Element('#text'); n.textContent = text; return n; } };
    const sent = [], spoken = [], timers = new Map();
    let timer = 0, uuid = 0;
    const crypto = { getRandomValues(a) { return a.fill(++uuid); } };
    const localStorage = { getItem: k => k === 'llm_lang' ? lang : null };
    const window = { isSecureContext: true, speechSynthesis: {
        cancel() {}, getVoices: () => [{lang, name:'Test'}],
        speak(u) { spoken.push(u.text); }
    } };
    class SpeechSynthesisUtterance { constructor(text) { this.text = text; } }
    const requestAnimationFrame = f => f();
    const sendCommand = c => { sent.push(c); return true; };
    const setTimeout = f => { timers.set(++timer, f); return timer; };
    const clearTimeout = id => timers.delete(id);
    eval(source);

    const handle = s => window.handleLLMStatus({data:s});
    const input = body.find(e => e.className === 'llm-in');
    const send = body.find(e => e.tag === 'button' && e.title === (lang === 'vi' ? 'Gửi' : 'Send'));
    const ask = text => { input.value = text; send.onclick(); };
    const last = () => chat.children.at(-1);
    const question = '  ✅ A1 còn trống không?  ';
    const friendly = lang === 'vi' ? 'Bạn muốn hỏi ô nào?' : 'Which slot do you mean?';
    const status = {state:'waiting_confirm', say:'🤖 Plan A1', steps:[{action:'quick_goal', args:{target:'HOME'}}],
        action_summary:['✅ Go to HOME']};
    handle(status);
    const plan = last();
    const confirm = plan.find(e => e.tag === 'button');
    assert(confirm && !/[✅🤖]/u.test(plan.words), 'plan prose');
    ask(question);
    const session = sent.at(-1).conversation_id;
    assert(last().textContent === question && sent.at(-1).text === question, 'user text preserved');
    for (const msg of [friendly, 'A1, 4,6°, 72%']) {
        handle({state:'answer', observational:true, msg:'⚠️ ' + msg + ' ✅', conversation_id:session});
        assert(last().textContent === msg && !last().className.includes('err'), 'neutral answer');
        assert(spoken.at(-1) === msg, 'clean speech');
    }
    handle({state:'answer', observational:true, msg:'other tab', conversation_id:'elsewhere'});
    assert(spoken.at(-1) !== 'other tab', 'private follow-up');
    assert(plan.find(e => e === confirm), 'confirmation survived');
    assert(sent.length === 1 && sent[0].command === 'llm_command', 'questions sent no control command');
    confirm.onclick();
    assert(sent.at(-1).command === 'llm_confirm' && sent.at(-1).ok, 'original confirmation works');

    for (const state of ['running','clarify','done','simulated','started','stopped','confirm_interrupt','non_interruptible']) {
        handle({state, step:1, total:1, msg:'❓ A1, 4,6°, 72%'});
        assert(!/[▶✔❓⚠✖⛔◉]/u.test(last().words + spoken.at(-1)), 'plain chat and speech: ' + state);
    }
    const errors = [
        {state:'failed', error_kind:'mission', error_id:'nav', msg:'Nav2 stopped.'},
        {state:'failed', error_kind:'mission', error_id:'camera', msg:'Camera is not ready.'},
    ];
    for (const e of errors) {
        const before = chat.children.length, utterances = spoken.length;
        for (let i = 0; i < 4; i++) handle(e);
        assert(chat.children.length === before + 1 && spoken.length === utterances + 1, 'one notice per failure');
        assert(last().textContent === e.msg && spoken.at(-1) === e.msg, 'new failure visible and spoken');
        assert(last().className.includes('err'), 'failure styled as error');
    }
    handle({state:'error', msg:'Traceback ... /private/path'});
    assert(!last().words.includes('Traceback') && !spoken.at(-1).includes('/private'), 'legacy error hidden');
    ask('go HOME');
    handle({state:'pending'});
    assert(timers.size === 1, 'bridge pending is not a node reply');
    for (const f of Array.from(timers.values())) f();
    assert(last().textContent === spoken.at(-1) && /chưa xác nhận|cannot confirm/.test(last().words), 'uncertain timeout');
    ask('go HOME');
    window.handleLLMCommandResponse({command:'llm_command', status:'error', reason_code:'telemetry'});
    assert(timers.size === 0 && !last().className.includes('err'), 'precheck handles missing data without a second timeout');
    const before = session;
    window.resetLLMSession();
    ask('what about that slot?');
    assert(sent.at(-1).conversation_id !== before, 'reset starts a new conversation');
    print(lang + ' chat, confirmation, timeout, error replay and speech checks passed');
}
