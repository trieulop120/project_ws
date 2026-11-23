// Run from workspace root: gjs src/amr_web/test/test_llm_chat_scroll.js
const GLib = imports.gi.GLib;
const [ok, bytes] = GLib.file_get_contents('src/amr_web/resources/web/js/llm_voice.js');
if (!ok) throw new Error('Cannot read llm_voice.js');
const source = new TextDecoder().decode(bytes);
const frames = [];
var requestAnimationFrame = callback => { frames.push(callback); };
var setTimeout = () => 1;
var clearTimeout = () => {};
var localStorage = {getItem: () => null, setItem: () => {}};
var window = globalThis;
window.isSecureContext = true;
var sent = [];
var sendCommand = command => { sent.push(command); return true; };

class Element {
    constructor(tag = 'div') {
        this.tag = tag;
        this.children = [];
        this.style = {};
        this.clientHeight = 80;
        this.scrollTop = 0;
        this.classList = {toggle: () => {}, add: () => {}, remove: () => {}};
    }
    appendChild(child) { this.children.push(child); child.parentNode = this; return child; }
    removeChild(child) { this.children.splice(this.children.indexOf(child), 1); child.parentNode = null; }
    get firstChild() { return this.children[0]; }
    get scrollHeight() { return this.children.reduce((n, child) => n + 24 + child.children.length * 24, 0); }
    set innerHTML(value) { this.children = []; this._html = value; }
    get innerHTML() { return this._html || ''; }
    insertAdjacentElement(_where, child) { this.control = child; }
    addEventListener(name, callback) { this['on' + name] = callback; }
    querySelectorAll(selector) { return selector === 'label' ? this.children.filter(c => c.tag === 'label') : []; }
    querySelector(selector) {
        if (selector === 'input') return this.children.find(c => c.tag === 'input') || null;
        if (selector === 'label:last-of-type') return this.children.filter(c => c.tag === 'label').at(-1) || null;
        return null;
    }
    remove() { this.parentNode?.removeChild(this); }
}

function check(value, message) { if (!value) throw new Error(message); }
function flush() { while (frames.length) frames.shift()(); }
function find(root, cls) {
    if ((root.className || '').split(' ').includes(cls)) return root;
    for (const child of root.children) { const match = find(child, cls); if (match) return match; }
    return null;
}
function run(embedded) {
    let recognition;
    window.SpeechRecognition = class {
        constructor() { recognition = this; }
        start() {}
        stop() {}
    };
    const chat = embedded ? new Element() : null;
    const body = new Element();
    var document = {
        readyState: 'complete', head: new Element(), body,
        createElement: tag => new Element(tag), createTextNode: text => ({textContent: text}),
        getElementById: () => null, querySelector: selector => selector === '.ai-chat-box' ? chat : null
    };
    eval(source);
    const float = embedded ? null : body.children.find(c => c.id === 'llmFloat');
    const box = chat || float.children[0];
    const controls = chat?.control || float.children[1];
    const input = find(controls, 'llm-in');
    const send = controls.children[1].children[2];
    flush();
    box.scrollTop = 0;
    input.value = 'send by button'; send.onclick(); flush();
    check(box.scrollTop === box.scrollHeight, 'Send did not scroll');
    box.scrollTop = 0;
    input.value = 'send by Enter'; input.onkeydown({key: 'Enter'}); flush();
    check(box.scrollTop === box.scrollHeight, 'Enter did not scroll');
    box.scrollTop = 0;
    controls.children[1].children[0].onclick();
    recognition.onresult({resultIndex: 0, results: [Object.assign([{transcript: 'send by voice', confidence: 0.9}], {isFinal: true})]});
    flush();
    check(box.scrollTop === box.scrollHeight && sent.at(-1).text === 'send by voice', 'voice send did not scroll');
    for (const state of ['answer', 'clarify', 'waiting_confirm', 'confirm_interrupt', 'error']) {
        box.scrollTop = 0;
        window.handleLLMStatus({state, msg: 'new content', observational: state === 'answer',
            steps: Array.from({length: 12}, (_, i) => ({action: 'step' + i}))});
        flush();
        check(box.scrollTop === box.scrollHeight, state + ' did not reach bottom');
        if (['waiting_confirm', 'confirm_interrupt'].includes(state)) {
            const plan = box.children.filter(child => (child.className || '').includes('llm-plan')).at(-1);
            check(plan && find(plan, 'llm-row')?.children.length === 2, state + ' buttons missing');
        }
    }
    box.scrollTop = 0;
    window.handleLLMStatus({state: 'planning'});
    flush();
    check(box.scrollTop === 0, 'background status changed scroll position');
    if (float) {
        box.scrollTop = 0;
        body.children.find(c => c.id === 'llmFab').onclick(); flush();
        check(box.scrollTop === box.scrollHeight, 'opening float did not show latest message');
    }
}
run(true);
run(false);
print('LLM chat scroll scenarios passed');
