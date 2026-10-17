// resources/web/js/history.js
// History page (data-page="system"): Operation History + Navigation Results.
// Data comes only from /api/history/*. No WebSocket, no map canvas, no external libraries.
(function () {
    'use strict';

    const API = '/api/history';
    const $ = (id) => document.getElementById(id);

    // Static option lists; /meta values are merged on top of these.
    const EV_CATEGORIES = ['SLAM', 'NAVIGATION', 'MISSION', 'MAP', 'ROUTE', 'LIFT', 'MODE', 'SAFETY', 'SYSTEM', 'CONNECTION'];
    const EV_LEVELS = ['info', 'warn', 'error'];
    const EV_RESULTS = ['requested', 'success', 'failed', 'canceled'];
    const RUN_TYPES = ['goal', 'waypoints', 'route'];
    const RUN_RESULTS = ['SUCCEEDED', 'FAILED', 'CANCELED', 'ESTOP', 'INTERRUPTED'];

    const FIELD_IDS = {
        events: { category: 'histEvCategory', level: 'histEvLevel', result: 'histEvResult', client: 'histEvClient', q: 'histEvQ' },
        runs: { type: 'histRunType', result: 'histRunResult', map_id: 'histRunMap', scenario: 'histRunScenario', client: 'histRunClient', q: 'histRunQ' }
    };

    const state = {
        tab: 'events',
        ready: false,
        enabled: true,
        meta: null,
        preset: 'all',
        range: { from: null, to: null },          // applied range (epoch seconds)
        tabs: {
            events: { page: 1, pageSize: 25, sort: 'desc', f: {}, items: [], total: 0 },
            runs: { page: 1, pageSize: 25, sort: 'desc', f: {}, items: [], total: 0 }
        },
        token: 0,
        abort: null
    };

    const modal = {
        open: false, id: null, run: null, wps: [], traj: [], trajState: 'idle',
        dirty: false, token: 0, lastFocus: null, ro: null
    };

    // ======================================================================
    // Helpers
    // ======================================================================
    function esc(v) {
        return String(v == null ? '' : v)
            .replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;')
            .replace(/"/g, '&quot;').replace(/'/g, '&#39;');
    }
    function g(o, ...keys) {
        if (!o) return undefined;
        for (const k of keys) if (o[k] !== undefined && o[k] !== null) return o[k];
        return undefined;
    }
    function num(v) {
        if (v == null || v === '' || typeof v === 'boolean') return null;
        const n = Number(v);
        return isFinite(n) ? n : null;
    }
    const DASH = '<span class="hist-null">-</span>';
    function f3(v) { const n = num(v); return n === null ? '-' : n.toFixed(3); }
    function f3h(v) { const n = num(v); return n === null ? DASH : n.toFixed(3); }
    function pad2(n) { return String(n).padStart(2, '0'); }

    function normTs(v) {
        if (v == null || v === '') return null;
        let n = Number(v);
        if (!isFinite(n)) {
            const p = Date.parse(v);
            if (!isFinite(p)) return null;
            n = p / 1000;
        }
        return n > 1e11 ? n / 1000 : n;   // tolerate milliseconds
    }
    function fmtTime(v, timeOnly) {
        const t = normTs(v);
        if (t === null) return '-';
        const d = new Date(t * 1000);
        const hms = pad2(d.getHours()) + ':' + pad2(d.getMinutes()) + ':' + pad2(d.getSeconds());
        if (timeOnly) return hms;
        return d.getFullYear() + '-' + pad2(d.getMonth() + 1) + '-' + pad2(d.getDate()) + ' ' + hms;
    }
    function fmtDur(v) {
        const s = num(v);
        if (s === null || s < 0) return '-';
        const m = Math.floor(s / 60);
        const rest = s - m * 60;
        return pad2(m) + ':' + (rest < 10 ? '0' : '') + rest.toFixed(1);
    }
    function toLocalInput(d) {
        return d.getFullYear() + '-' + pad2(d.getMonth() + 1) + '-' + pad2(d.getDate()) + 'T' + pad2(d.getHours()) + ':' + pad2(d.getMinutes());
    }
    function isMobile() { return window.matchMedia && window.matchMedia('(max-width: 768px)').matches; }
    function toast(msg, type) { if (typeof window.showToast === 'function') window.showToast(msg, type || 'success'); }

    function resultTone(result) {
        switch (String(result || '').toUpperCase()) {
            case 'SUCCEEDED': case 'SUCCESS': return 'ok';
            case 'FAILED': return 'bad';
            case 'CANCELED': case 'ESTOP': return 'warn';
            case 'INTERRUPTED': return 'idle';
            default: return '';
        }
    }
    function chip(text, tone) {
        if (text == null || text === '') return DASH;
        return '<span class="hist-chip ' + (tone || 'plain') + '">' + esc(text) + '</span>';
    }
    function levelChip(level) {
        const l = String(level || '').toLowerCase();
        return chip(level, l === 'error' ? 'bad' : l === 'warn' ? 'warn' : 'idle');
    }
    function eventResultChip(res) {
        const r = String(res || '').toLowerCase();
        const tone = r === 'success' ? 'ok' : r === 'failed' ? 'bad' : (r === 'canceled' || r === 'cancelled') ? 'warn' : r === 'requested' ? 'idle' : 'plain';
        return chip(res, tone);
    }

    function uniq(list) {
        const seen = new Set(); const out = [];
        list.forEach(v => { const s = String(v); if (s !== '' && !seen.has(s)) { seen.add(s); out.push(s); } });
        return out;
    }
    function metaList(m, keys) {
        if (!m) return [];
        for (const k of keys) {
            const v = m[k];
            if (Array.isArray(v)) {
                return v.map(x => (x && typeof x === 'object') ? (x.value ?? x.name ?? x.id ?? '') : x).filter(x => x !== '' && x != null);
            }
        }
        return [];
    }

    async function getJSON(path, params, signal, init) {
        const qs = params ? new URLSearchParams(params).toString() : '';
        const res = await fetch(API + path + (qs ? '?' + qs : ''), Object.assign({
            signal, cache: 'no-store', headers: { 'Accept': 'application/json' }
        }, init || {}));
        let body = null;
        try { body = await res.json(); } catch (e) { /* handled below */ }
        if (!res.ok || (body && body.status === 'error')) {
            throw new Error((body && body.message) || ('Request failed (HTTP ' + res.status + ')'));
        }
        if (body === null) throw new Error('Unexpected response from the server');
        return body;
    }
    function pickList(data, keys) {
        if (Array.isArray(data)) return data;
        if (!data) return [];
        for (const k of keys) if (Array.isArray(data[k])) return data[k];
        for (const k of Object.keys(data)) if (Array.isArray(data[k])) return data[k];
        return [];
    }

    // ======================================================================
    // Filters
    // ======================================================================
    function fillSelect(id, values) {
        const sel = $(id);
        if (!sel) return;
        const keep = sel.value;
        sel.innerHTML = '<option value="">Any</option>' + uniq(values).map(v => '<option value="' + esc(v) + '">' + esc(v) + '</option>').join('');
        if (keep && uniq(values).includes(keep)) sel.value = keep;
    }

    function populateSelects() {
        const m = state.meta || {};
        fillSelect('histEvCategory', EV_CATEGORIES.concat(metaList(m, ['categories', 'category'])));
        fillSelect('histEvLevel', EV_LEVELS.concat(metaList(m, ['levels', 'level'])));
        fillSelect('histEvResult', EV_RESULTS.concat(metaList(m, ['event_results'])));
        fillSelect('histRunType', RUN_TYPES.concat(metaList(m, ['run_types', 'types', 'type'])));
        const allRes = metaList(m, ['results']).filter(v => String(v) === String(v).toUpperCase());
        fillSelect('histRunResult', RUN_RESULTS.concat(metaList(m, ['run_results']), allRes));
        fillSelect('histRunMap', metaList(m, ['map_ids', 'maps', 'map_id', 'map_names']));
        // restore the applied values (options may have just been rebuilt)
        writeInputs(state.tab);

        const first = normTs(g(m, 'first_ts', 'first', 'min_ts', 'first_timestamp'));
        const last = normTs(g(m, 'last_ts', 'last', 'max_ts', 'last_timestamp'));
        const allBtn = document.querySelector('.hist-presets [data-preset="all"]');
        if (allBtn) allBtn.title = (first !== null && last !== null) ? ('Records from ' + fmtTime(first) + ' to ' + fmtTime(last)) : '';
    }

    function writeInputs(tab) {
        const ids = FIELD_IDS[tab];
        const f = state.tabs[tab].f;
        Object.keys(ids).forEach(k => { const el = $(ids[k]); if (el) el.value = f[k] || ''; });
        $('histFrom').value = state.range.from !== null ? toLocalInput(new Date(state.range.from * 1000)) : '';
        $('histTo').value = state.range.to !== null ? toLocalInput(new Date(state.range.to * 1000)) : '';
        markPreset();
    }

    // Reads the inputs into the applied state. Returns false (and toasts) when invalid.
    function readInputs(tab) {
        const fromV = $('histFrom').value, toV = $('histTo').value;
        const from = fromV ? Date.parse(fromV) / 1000 : null;
        let to = toV ? Date.parse(toV) / 1000 + 59.999 : null;   // datetime-local has minute precision
        if ((from !== null && !isFinite(from)) || (to !== null && !isFinite(to))) {
            toast('Invalid date or time', 'error');
            return false;
        }
        if (from !== null && to !== null && from > to) {
            toast('"From" must be before "To"', 'error');
            return false;
        }
        state.range.from = from;
        state.range.to = to === null ? null : Number(to.toFixed(3));
        const ids = FIELD_IDS[tab];
        const f = {};
        Object.keys(ids).forEach(k => { const v = ($(ids[k]).value || '').trim(); if (v) f[k] = v; });
        state.tabs[tab].f = f;
        state.tabs[tab].page = 1;
        return true;
    }

    function markPreset() {
        document.querySelectorAll('.hist-presets [data-preset]').forEach(b => {
            b.classList.toggle('on', b.dataset.preset === state.preset);
        });
    }

    function applyPreset(name) {
        const now = new Date();
        const sod = new Date(now.getFullYear(), now.getMonth(), now.getDate(), 0, 0);
        const eod = new Date(now.getFullYear(), now.getMonth(), now.getDate(), 23, 59);
        if (name === 'today') {
            $('histFrom').value = toLocalInput(sod);
            $('histTo').value = toLocalInput(eod);
        } else if (name === '7d') {
            const start = new Date(sod.getTime()); start.setDate(start.getDate() - 6);   // today + previous 6 days
            $('histFrom').value = toLocalInput(start);
            $('histTo').value = toLocalInput(eod);
        } else {
            $('histFrom').value = '';
            $('histTo').value = '';
        }
        state.preset = name;
        markPreset();
        applyFilters();
    }

    function activeFilterCount() {
        const t = state.tabs[state.tab];
        return Object.keys(t.f).length + ((state.range.from !== null || state.range.to !== null) ? 1 : 0);
    }
    function updateFilterBadge() {
        const n = activeFilterCount();
        const b = $('histFilterCount');
        b.textContent = String(n);
        b.hidden = n === 0;
    }

    function applyFilters() {
        if (!readInputs(state.tab)) return false;
        updateFilterBadge();
        if (isMobile()) setFiltersOpen(false);
        load();
        return true;
    }

    function clearFilters() {
        Object.keys(FIELD_IDS[state.tab]).forEach(k => { const el = $(FIELD_IDS[state.tab][k]); if (el) el.value = ''; });
        state.preset = 'all';
        $('histFrom').value = '';
        $('histTo').value = '';
        markPreset();
        applyFilters();
    }

    function setFiltersOpen(open) {
        $('histFilters').classList.toggle('open', open);
        $('histFilterToggle').setAttribute('aria-expanded', open ? 'true' : 'false');
    }

    function buildParams(tab, paging) {
        const t = state.tabs[tab];
        const p = {};
        if (state.range.from !== null) p.from = Number(state.range.from.toFixed(3));
        if (state.range.to !== null) p.to = state.range.to;
        Object.keys(t.f).forEach(k => { p[k] = t.f[k]; });
        p.sort = t.sort;
        if (paging) { p.page = t.page; p.page_size = t.pageSize; }
        return p;
    }

    // ======================================================================
    // Table rendering
    // ======================================================================
    const COLS = {
        events: [
            { h: 'Time', cls: 'mono', f: r => esc(fmtTime(r.ts)) },
            { h: 'Category', f: r => chip(r.category, 'plain') },
            { h: 'Level', f: r => levelChip(r.level) },
            { h: 'Action', cls: 'mono', f: r => r.action ? esc(r.action) : DASH },
            { h: 'Detail', cls: 'wrap', f: r => r.detail ? esc(r.detail) : DASH },
            { h: 'Result', f: r => eventResultChip(r.result) },
            { h: 'Client', cls: 'mono', f: r => { const c = g(r, 'client_ip', 'client'); return c ? esc(c) : DASH; } }
        ],
        runs: [
            { h: 'ID', cls: 'mono', f: r => esc(r.id) },
            { h: 'Started', cls: 'mono', f: r => esc(fmtTime(r.started_at)) },
            { h: 'Type', f: r => chip(r.run_type, 'plain') },
            { h: 'Scenario', cls: 'wrap', f: r => r.scenario ? esc(r.scenario) : DASH },
            { h: 'Map', cls: 'mono', f: r => r.map_id ? esc(r.map_id) : DASH },
            { h: 'Goal X (m)', cls: 'num m-hide', f: r => f3h(r.goal_x) },
            { h: 'Goal Y (m)', cls: 'num m-hide', f: r => f3h(r.goal_y) },
            { h: 'Goal yaw (deg)', cls: 'num m-hide', f: r => f3h(r.goal_yaw_deg) },
            { h: 'Pos err (m)', cls: 'num', f: r => f3h(r.pos_error_m) },
            { h: 'Yaw err (deg)', cls: 'num', f: r => f3h(r.yaw_error_deg) },
            { h: 'Measured (cm)', cls: 'num', f: r => f3h(r.measured_error_cm) },
            { h: 'Path (m)', cls: 'num m-hide', f: r => f3h(r.path_length_m) },
            { h: 'Duration (mm:ss.s)', cls: 'num', f: r => { const d = fmtDur(r.duration_s); return d === '-' ? DASH : esc(d); } },
            { h: 'Result', f: r => chip(r.result, resultTone(r.result) || 'plain') }
        ]
    };

    function rowTone(tab, r) {
        if (tab === 'runs') return resultTone(r.result);
        const level = String(r.level || '').toLowerCase();
        const res = String(r.result || '').toLowerCase();
        if (level === 'error' || res === 'failed') return 'bad';
        if (level === 'warn') return 'warn';
        if (res === 'success') return 'ok';
        return '';
    }

    function renderTable() {
        const tab = state.tab;
        const t = state.tabs[tab];
        const cols = COLS[tab];
        const table = $('histTable');
        table.className = 'hist-table' + (tab === 'runs' ? ' runs' : '');

        $('histHead').innerHTML = '<tr>' + cols.map(c =>
            '<th scope="col" class="' + (c.cls || '').replace('mono', '').replace('wrap', '').replace('m-hide', '').trim() + '">' + esc(c.h) + '</th>'
        ).join('') + '</tr>';

        $('histBody').innerHTML = t.items.map(r => {
            const tone = rowTone(tab, r);
            const attrs = tab === 'runs'
                ? ' data-id="' + esc(r.id) + '" tabindex="0" role="button" aria-label="Open run ' + esc(r.id) + '"'
                : '';
            return '<tr data-tone="' + tone + '"' + attrs + '>' + cols.map(c =>
                '<td class="' + (c.cls || '') + '" data-label="' + esc(c.h) + '">' + c.f(r) + '</td>'
            ).join('') + '</tr>';
        }).join('');
    }

    function renderPager() {
        const t = state.tabs[state.tab];
        const last = Math.max(1, Math.ceil(t.total / t.pageSize));
        const cur = Math.min(t.page, last);
        const a = t.total === 0 ? 0 : (cur - 1) * t.pageSize + 1;
        const b = Math.min(cur * t.pageSize, t.total);
        $('histRangeText').textContent = t.total === 0 ? 'No rows' : ('Showing ' + a + '-' + b + ' of ' + t.total);
        $('histPageSize').value = String(t.pageSize);
        $('histSort').textContent = t.sort === 'desc' ? 'Newest first' : 'Oldest first';

        const nav = $('histPages');
        if (last <= 1) { nav.innerHTML = ''; return; }
        const set = new Set([1, last, cur - 1, cur, cur + 1]);
        const nums = Array.from(set).filter(n => n >= 1 && n <= last).sort((x, y) => x - y);
        let html = '<button type="button" class="btn btn-secondary" data-page="' + (cur - 1) + '"' + (cur <= 1 ? ' disabled' : '') + '>Prev</button>';
        let prev = 0;
        nums.forEach(n => {
            if (n - prev > 1) html += '<span class="gap">...</span>';
            html += '<button type="button" class="btn btn-secondary num-btn' + (n === cur ? ' current' : '') + '" data-page="' + n + '"' +
                (n === cur ? ' aria-current="page"' : '') + '>' + n + '</button>';
            prev = n;
        });
        html += '<span class="hist-pinfo">Page ' + cur + ' / ' + last + '</span>';
        html += '<button type="button" class="btn btn-secondary" data-page="' + (cur + 1) + '"' + (cur >= last ? ' disabled' : '') + '>Next</button>';
        nav.innerHTML = html;
    }

    function setCardMode(opts) {
        const card = $('histPanel');
        card.classList.toggle('is-loading', !!opts.loading);
        card.classList.toggle('no-table', !!opts.noTable);
        card.classList.toggle('no-pager', !!opts.noPager);
    }

    function showState(kind, title, text, withRetry, withClear) {
        const el = $('histState');
        let html = '<strong>' + esc(title) + '</strong>' + (text ? '<p>' + esc(text) + '</p>' : '');
        if (withRetry) html += '<button type="button" class="btn btn-secondary" data-act="retry">Retry</button>';
        if (withClear) html += '<button type="button" class="btn btn-secondary" data-act="clear">Clear filters</button>';
        el.innerHTML = html;
        el.hidden = false;
        el.dataset.kind = kind;
    }
    function hideState() { $('histState').hidden = true; }

    // ======================================================================
    // Loading
    // ======================================================================
    async function loadMeta() {
        try {
            const m = await getJSON('/meta');
            state.meta = m;
            state.enabled = m.enabled !== false;
        } catch (e) {
            console.warn('[History] meta failed:', e.message);
        }
        populateSelects();
        $('histDownload').disabled = !state.enabled;
    }

    async function load() {
        const tab = state.tab;
        const t = state.tabs[tab];
        const token = ++state.token;
        if (state.abort) { try { state.abort.abort(); } catch (e) { /* ignore */ } }
        const ctl = (typeof AbortController !== 'undefined') ? new AbortController() : null;
        state.abort = ctl;

        const hasRows = t.items.length > 0;
        if (hasRows) {
            hideState();
            setCardMode({ loading: true });
        } else {
            showState('loading', 'Loading...', '');
            setCardMode({ noTable: true, noPager: true });
        }

        try {
            const data = await getJSON('/' + tab, buildParams(tab, true), ctl ? ctl.signal : undefined);
            if (token !== state.token) return;

            if (data.enabled === false) {
                state.enabled = false;
                $('histDownload').disabled = true;
                t.items = []; t.total = 0;
                showState('disabled', 'History is disabled',
                    'The log database could not be opened, so nothing is being recorded. Check that the log folder (AMR_LOG_DIR) is writable and look at the backend log, then restart the backend.');
                setCardMode({ noTable: true, noPager: true });
                return;
            }
            state.enabled = true;
            $('histDownload').disabled = false;

            t.items = pickList(data, tab === 'events' ? ['events', 'items', 'rows'] : ['runs', 'items', 'rows']);
            t.total = num(data.total) !== null ? num(data.total) : t.items.length;

            const last = Math.max(1, Math.ceil(t.total / t.pageSize));
            if (t.items.length === 0 && t.total > 0 && t.page > last) {   // page vanished (data changed)
                t.page = last;
                load();
                return;
            }

            if (t.items.length === 0) {
                const filtered = activeFilterCount() > 0;
                showState('empty',
                    tab === 'events' ? 'No operations found' : 'No navigation runs found',
                    filtered ? 'Nothing matches the current filters. Widen the time range or clear a filter.'
                        : (tab === 'events' ? 'Operations appear here as soon as someone uses the dashboard.' : 'Runs appear here after the robot finishes a goal or a mission.'),
                    false, filtered);
                setCardMode({ noTable: true });
                renderPager();
                return;
            }

            hideState();
            setCardMode({});
            renderTable();
            renderPager();
        } catch (e) {
            if (e.name === 'AbortError' || token !== state.token) return;
            showState('error', 'Could not load history', e.message || 'Unknown error', true, false);
            setCardMode({ noTable: true, noPager: true });
        }
    }

    // ======================================================================
    // Excel download (normal link: works on phones, keeps this page open)
    // ======================================================================
    function triggerDownload(url) {
        const a = document.createElement('a');
        a.href = url;
        a.download = '';
        a.rel = 'noopener';
        a.style.display = 'none';
        document.body.appendChild(a);
        a.click();
        setTimeout(() => a.remove(), 1500);
    }

    function downloadExcel() {
        if (!state.enabled) return;
        if (!applyFilters()) return;            // table and file always use the same filters
        const tab = state.tab;
        const params = buildParams(tab, false);
        triggerDownload(API + '/export/' + (tab === 'events' ? 'events' : 'runs') + '.xlsx?' + new URLSearchParams(params).toString());
        toast('Excel download started', 'success');
    }

    // ======================================================================
    // Run detail modal
    // ======================================================================
    function normTraj(arr) {
        if (!Array.isArray(arr)) return [];
        const out = [];
        arr.forEach(p => {
            let o = null;
            if (Array.isArray(p)) o = { t: num(p[0]), x: num(p[1]), y: num(p[2]), yaw: num(p[3]) };
            else if (p && typeof p === 'object') o = { t: num(p.t), x: num(p.x), y: num(p.y), yaw: num(g(p, 'yaw_deg', 'yaw')) };
            if (o && o.x !== null && o.y !== null) out.push(o);
        });
        return out;
    }

    function poseRow(label, x, y, yaw) {
        return '<tr><td>' + label + '</td><td>' + f3(x) + '</td><td>' + f3(y) + '</td><td>' + f3(yaw) + '</td></tr>';
    }

    function renderRunHead() {
        const r = modal.run;
        $('histRunTitle').textContent = 'Run #' + r.id;
        const parts = [chip(r.result, resultTone(r.result) || 'plain'), chip(r.run_type, 'plain'), '<span>Started ' + esc(fmtTime(r.started_at)) + '</span>'];
        if (r.ended_at) parts.push('<span>Ended ' + esc(fmtTime(r.ended_at)) + '</span>');
        if (r.map_id) parts.push('<span>Map ' + esc(r.map_id) + '</span>');
        if (r.route_id) parts.push('<span>Route ' + esc(r.route_id) + '</span>');
        const c = g(r, 'client_ip', 'client');
        if (c) parts.push('<span>Client ' + esc(c) + '</span>');
        let html = parts.join('');
        if (r.message) html += '<span class="msg">' + esc(r.message) + '</span>';
        $('histRunHead').innerHTML = html;
    }

    function renderRunSummary() {
        const r = modal.run;
        const dur = fmtDur(r.duration_s);
        const failed = String(r.result || '').toUpperCase() !== 'SUCCEEDED';
        let html = '<table class="hist-pose-table"><thead><tr><th>Pose</th><th>X (m)</th><th>Y (m)</th><th>Yaw (deg)</th></tr></thead><tbody>' +
            poseRow('Start', r.start_x, r.start_y, r.start_yaw_deg) +
            poseRow('Goal', r.goal_x, r.goal_y, r.goal_yaw_deg) +
            poseRow('End', r.end_x, r.end_y, r.end_yaw_deg) +
            '</tbody></table>';
        html += '<div class="hist-metrics">' +
            metric('Position error (m)', f3(r.pos_error_m)) +
            metric('Yaw error (deg)', f3(r.yaw_error_deg)) +
            metric('Measured error (cm)', f3(r.measured_error_cm)) +
            metric('Duration (mm:ss.s)', dur) +
            metric('Straight distance (m)', f3(r.straight_dist_m)) +
            metric('Path length (m)', f3(r.path_length_m)) +
            '</div>';
        html += '<div class="hist-note">' +
            (failed ? 'This run did not finish normally, so the errors above only show how far the robot was from the goal when it ended. They do not measure stopping accuracy. ' : '') +
            'Position and yaw errors come from the robot\'s own localisation. Enter the tape-measure value in Measured error for the real-world accuracy.' +
            '</div>';
        $('histRunSummary').innerHTML = html;
    }
    function metric(k, v) { return '<div class="hist-metric"><span class="k">' + esc(k) + '</span><span class="v">' + esc(v) + '</span></div>'; }

    function renderWaypoints(stateName, message) {
        const box = $('histRunWaypoints');
        if (stateName === 'loading') { box.innerHTML = '<div class="hist-inline-state">Loading waypoints...</div>'; return; }
        if (stateName === 'error') { box.innerHTML = '<div class="hist-inline-state">Could not load waypoints: ' + esc(message) + '</div>'; return; }
        if (!modal.wps.length) { box.innerHTML = ''; return; }
        const rows = modal.wps.map(w => {
            const reached = w.reached_at ? fmtTime(w.reached_at, true) : '-';
            return '<tr><td>' + esc(w.idx) + '</td><td>' + (w.name ? esc(w.name) : '-') + '</td>' +
                '<td>' + f3(w.goal_x) + '</td><td>' + f3(w.goal_y) + '</td><td>' + f3(w.goal_yaw_deg) + '</td>' +
                '<td>' + esc(reached) + '</td>' +
                '<td>' + f3(w.reached_x) + '</td><td>' + f3(w.reached_y) + '</td><td>' + f3(w.reached_yaw_deg) + '</td>' +
                '<td>' + f3(w.pos_error_m) + '</td><td>' + f3(w.yaw_error_deg) + '</td>' +
                '<td>' + (w.status ? chip(w.status, w.status === 'REACHED' ? 'ok' : w.status === 'FAILED' ? 'bad' : 'warn') : '-') + '</td></tr>';
        }).join('');
        box.innerHTML = '<h4>Waypoints</h4><div class="hist-wp-wrap"><table class="hist-wp-table"><thead><tr>' +
            '<th>Idx</th><th>Name</th><th>Goal X (m)</th><th>Goal Y (m)</th><th>Goal yaw (deg)</th><th>Reached at</th>' +
            '<th>Reached X (m)</th><th>Reached Y (m)</th><th>Reached yaw (deg)</th><th>Pos err (m)</th><th>Yaw err (deg)</th><th>Status</th>' +
            '</tr></thead><tbody>' + rows + '</tbody></table></div>';
    }

    function fillForm(r) {
        $('histEditScenario').value = r.scenario || '';
        $('histEditNotes').value = r.notes || '';
        $('histEditMeasured').value = num(r.measured_error_cm) !== null ? String(num(r.measured_error_cm)) : '';
    }
    function setSaveMsg(text, cls) {
        const el = $('histSaveMsg');
        el.textContent = text || '';
        el.className = 'hist-save-msg' + (cls ? ' ' + cls : '');
    }

    function openRun(id) {
        const row = state.tabs.runs.items.find(r => String(r.id) === String(id));
        if (!row) return;
        modal.open = true;
        modal.id = row.id;
        modal.run = Object.assign({}, row);
        modal.wps = [];
        modal.traj = [];
        modal.trajState = 'loading';
        modal.dirty = false;
        modal.lastFocus = document.activeElement;

        renderRunHead();
        renderRunSummary();
        renderWaypoints('loading');
        fillForm(modal.run);
        setSaveMsg('');
        $('histRunSave').disabled = false;

        const ov = $('histRunModal');
        ov.classList.add('active');
        ov.setAttribute('aria-hidden', 'false');
        document.body.classList.add('hist-modal-open');
        $('histRunBody').scrollTop = 0;
        setTimeout(() => $('histRunClose').focus(), 30);

        const box = document.querySelector('.hist-plot-box');
        if (window.ResizeObserver && box) {
            modal.ro = new ResizeObserver(() => drawPlot());
            modal.ro.observe(box);
        }
        requestAnimationFrame(drawPlot);
        loadRunDetail(row);
    }

    async function loadRunDetail(row) {
        const token = ++modal.token;
        try {
            const d = await getJSON('/runs/' + encodeURIComponent(row.id));
            if (token !== modal.token || !modal.open) return;
            const core = d.run || d.summary || d;
            modal.run = Object.assign({}, row, core);
            modal.run.id = row.id;
            modal.wps = pickListSafe(d.waypoints || core.waypoints);
            modal.traj = normTraj(d.trajectory || core.trajectory || d.points);
            modal.trajState = 'ok';
            renderRunHead();
            renderRunSummary();
            renderWaypoints('ok');
            if (!modal.dirty) fillForm(modal.run);
            drawPlot();
        } catch (e) {
            if (token !== modal.token || !modal.open) return;
            modal.trajState = 'error';
            renderWaypoints('error', e.message);
            drawPlot();
        }
    }
    function pickListSafe(v) { return Array.isArray(v) ? v : []; }

    function closeRun(force) {
        if (!modal.open) return;
        if (!force && modal.dirty && !window.confirm('Discard unsaved changes?')) return;
        modal.open = false;
        modal.token++;
        if (modal.ro) { modal.ro.disconnect(); modal.ro = null; }
        const ov = $('histRunModal');
        ov.classList.remove('active');
        ov.setAttribute('aria-hidden', 'true');
        document.body.classList.remove('hist-modal-open');
        if (modal.lastFocus && typeof modal.lastFocus.focus === 'function') {
            try { modal.lastFocus.focus({ preventScroll: true }); } catch (e) { /* ignore */ }
        }
    }

    async function saveRun() {
        if (!modal.open) return;
        const body = {
            scenario: $('histEditScenario').value.trim(),
            notes: $('histEditNotes').value.trim()
        };
        const mRaw = $('histEditMeasured').value.trim();
        let measuredSkipped = false;
        if (mRaw !== '') {
            const n = Number(mRaw);
            if (!isFinite(n)) { setSaveMsg('Measured error must be a number', 'err'); return; }
            body.measured_error_cm = n;
        } else if (num(modal.run.measured_error_cm) !== null) {
            measuredSkipped = true;   // the backend can overwrite this value but cannot clear it yet
        }

        const btn = $('histRunSave');
        btn.disabled = true;
        setSaveMsg('Saving...');
        try {
            const res = await getJSON('/runs/' + encodeURIComponent(modal.id), null, undefined, {
                method: 'PATCH',
                headers: { 'Content-Type': 'application/json', 'Accept': 'application/json' },
                body: JSON.stringify(body)
            });
            const patch = Object.assign({}, body);
            if (res && res.run && typeof res.run === 'object') Object.assign(patch, res.run);
            Object.assign(modal.run, patch);
            const item = state.tabs.runs.items.find(r => String(r.id) === String(modal.id));
            if (item) Object.assign(item, patch);
            modal.dirty = false;
            renderRunSummary();
            if (state.tab === 'runs') renderTable();
            setSaveMsg(measuredSkipped ? 'Saved. The old measured error was kept: it can be changed but not cleared.' : 'Saved', 'ok');
            toast('Run #' + modal.id + ' saved', 'success');
        } catch (e) {
            setSaveMsg('Save failed: ' + e.message, 'err');
            toast('Save failed: ' + e.message, 'error');
        } finally {
            btn.disabled = false;
        }
    }

    // ---------------------------------------------------------------- plot
    function cssVar(name, fb) {
        const v = getComputedStyle(document.documentElement).getPropertyValue(name).trim();
        return v || fb;
    }
    function niceStep(raw) {
        const p = Math.pow(10, Math.floor(Math.log10(raw)));
        const m = raw / p;
        return (m <= 1 ? 1 : m <= 2 ? 2 : m <= 5 ? 5 : 10) * p;
    }

    function drawPlot() {
        const cv = $('histTrajCanvas');
        if (!cv || !modal.open || !modal.run) return;
        const box = cv.parentElement;
        const w = box.clientWidth, h = box.clientHeight;
        if (!w || !h) return;
        const dpr = Math.min(window.devicePixelRatio || 1, 3);
        cv.width = Math.round(w * dpr);
        cv.height = Math.round(h * dpr);
        const c = cv.getContext('2d');
        c.setTransform(dpr, 0, 0, dpr, 0, 0);
        c.clearRect(0, 0, w, h);

        const col = {
            accent: cssVar('--accent', '#55DDBB'), text: cssVar('--text-primary', '#E2E8F0'),
            muted: cssVar('--text-muted', '#64748B'), warn: cssVar('--warning', '#F5B94C'),
            err: cssVar('--error', '#F16C70'), sec: cssVar('--text-secondary', '#94A3B3'),
            grid: cssVar('--border', '#2A363B'), bg: cssVar('--bg-panel-secondary', '#12181A')
        };
        const r = modal.run;
        const marks = {
            start: { x: num(r.start_x), y: num(r.start_y), yaw: num(r.start_yaw_deg) },
            goal: { x: num(r.goal_x), y: num(r.goal_y), yaw: num(r.goal_yaw_deg) },
            end: { x: num(r.end_x), y: num(r.end_y), yaw: num(r.end_yaw_deg) }
        };
        const wpGoals = modal.wps.map(wp => ({ x: num(wp.goal_x), y: num(wp.goal_y), idx: wp.idx })).filter(p => p.x !== null && p.y !== null);

        const pts = [];
        modal.traj.forEach(p => pts.push([p.x, p.y]));
        Object.keys(marks).forEach(k => { if (marks[k].x !== null && marks[k].y !== null) pts.push([marks[k].x, marks[k].y]); });
        wpGoals.forEach(p => pts.push([p.x, p.y]));

        const emptyEl = $('histTrajEmpty');
        if (pts.length === 0) {
            emptyEl.textContent = modal.trajState === 'loading' ? 'Loading trajectory...' :
                modal.trajState === 'error' ? 'Trajectory could not be loaded.' : 'No position data was recorded for this run.';
            emptyEl.hidden = false;
            return;
        }
        emptyEl.hidden = true;

        let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
        pts.forEach(p => { minX = Math.min(minX, p[0]); maxX = Math.max(maxX, p[0]); minY = Math.min(minY, p[1]); maxY = Math.max(maxY, p[1]); });
        const spanX = Math.max(maxX - minX, 1), spanY = Math.max(maxY - minY, 1);   // at least 1 m so tiny runs stay readable
        const ml = 34, mr = 14, mt = 14, mb = 24;
        const aw = w - ml - mr, ah = h - mt - mb;
        const scale = Math.min(aw / spanX, ah / spanY);   // equal aspect: 1 m is the same length on both axes
        const ox = ml + (aw - spanX * scale) / 2;
        const oyBottom = mt + ah - (ah - spanY * scale) / 2;
        const cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
        // keep the data centered when the span was padded to 1 m
        const offX = ((maxX - minX) < spanX) ? (spanX - (maxX - minX)) / 2 : 0;
        const offY = ((maxY - minY) < spanY) ? (spanY - (maxY - minY)) / 2 : 0;
        void cx; void cy;
        const sx = x => ox + (x - minX + offX) * scale;
        const sy = y => oyBottom - (y - minY + offY) * scale;

        // grid + labels
        const step = niceStep(Math.max(spanX, spanY) / 5);
        const decimals = step < 1 ? (step < 0.1 ? 2 : 1) : 0;
        c.font = '10px "JetBrains Mono", monospace';
        c.lineWidth = 1;
        const gx0 = Math.ceil((minX - offX) / step) * step, gx1 = maxX + offX;
        const gy0 = Math.ceil((minY - offY) / step) * step, gy1 = maxY + offY;
        c.strokeStyle = col.grid; c.fillStyle = col.muted;
        c.textAlign = 'center'; c.textBaseline = 'top';
        for (let x = gx0; x <= gx1 + 1e-9; x += step) {
            const px = Math.round(sx(x)) + .5;
            c.beginPath(); c.moveTo(px, mt); c.lineTo(px, h - mb); c.stroke();
            c.fillText(x.toFixed(decimals), px, h - mb + 5);
        }
        c.textAlign = 'right'; c.textBaseline = 'middle';
        for (let y = gy0; y <= gy1 + 1e-9; y += step) {
            const py = Math.round(sy(y)) + .5;
            c.beginPath(); c.moveTo(ml, py); c.lineTo(w - mr, py); c.stroke();
            c.fillText(y.toFixed(decimals), ml - 5, py);
        }
        c.textAlign = 'right'; c.textBaseline = 'bottom';
        c.fillText('m', w - 4, h - 2);

        // path
        if (modal.traj.length > 1) {
            c.strokeStyle = col.accent; c.globalAlpha = .85; c.lineWidth = 2; c.lineJoin = 'round';
            c.beginPath();
            modal.traj.forEach((p, i) => { if (i === 0) c.moveTo(sx(p.x), sy(p.y)); else c.lineTo(sx(p.x), sy(p.y)); });
            c.stroke();
            c.globalAlpha = 1;
        } else if (modal.trajState === 'ok') {
            c.fillStyle = col.muted; c.textAlign = 'left'; c.textBaseline = 'top';
            c.fillText('No trajectory samples recorded', ml + 6, mt + 4);
        } else if (modal.trajState === 'loading') {
            c.fillStyle = col.muted; c.textAlign = 'left'; c.textBaseline = 'top';
            c.fillText('Loading trajectory...', ml + 6, mt + 4);
        }

        const tick = (m, color) => {          // heading tick, yaw in degrees, map frame (x right, y up)
            if (m.yaw === null) return;
            const a = m.yaw * Math.PI / 180, px = sx(m.x), py = sy(m.y);
            c.strokeStyle = color; c.lineWidth = 2;
            c.beginPath(); c.moveTo(px, py); c.lineTo(px + Math.cos(a) * 16, py - Math.sin(a) * 16); c.stroke();
        };
        const diamond = (px, py, s, fill, stroke) => {
            c.beginPath(); c.moveTo(px, py - s); c.lineTo(px + s, py); c.lineTo(px, py + s); c.lineTo(px - s, py); c.closePath();
            if (fill) { c.fillStyle = fill; c.fill(); }
            if (stroke) { c.strokeStyle = stroke; c.lineWidth = 1.5; c.stroke(); }
        };

        // waypoint goals
        c.font = '10px "JetBrains Mono", monospace'; c.textAlign = 'left'; c.textBaseline = 'bottom';
        wpGoals.forEach(p => {
            diamond(sx(p.x), sy(p.y), 4, null, col.warn);
            c.fillStyle = col.warn; c.fillText(String(p.idx), sx(p.x) + 6, sy(p.y) - 4);
        });

        const tone = resultTone(r.result);
        const endColor = tone === 'ok' ? col.accent : tone === 'bad' ? col.err : tone === 'warn' ? col.warn : col.sec;

        if (marks.goal.x !== null && marks.goal.y !== null) {
            tick(marks.goal, col.warn);
            diamond(sx(marks.goal.x), sy(marks.goal.y), 7, col.warn, col.bg);
        }
        if (marks.start.x !== null && marks.start.y !== null) {
            tick(marks.start, col.text);
            c.beginPath(); c.arc(sx(marks.start.x), sy(marks.start.y), 5.5, 0, Math.PI * 2);
            c.fillStyle = col.bg; c.fill(); c.strokeStyle = col.text; c.lineWidth = 2; c.stroke();
        }
        if (marks.end.x !== null && marks.end.y !== null) {
            tick(marks.end, endColor);
            c.beginPath(); c.arc(sx(marks.end.x), sy(marks.end.y), 5.5, 0, Math.PI * 2);
            c.fillStyle = endColor; c.fill(); c.strokeStyle = col.bg; c.lineWidth = 1.5; c.stroke();
        }
    }

    // ======================================================================
    // Events / wiring
    // ======================================================================
    function switchTab(tab) {
        if (tab === state.tab && state.ready) return;
        state.tab = tab;
        document.querySelectorAll('.hist-tab').forEach(b => {
            const on = b.dataset.tab === tab;
            b.classList.toggle('active', on);
            b.setAttribute('aria-selected', on ? 'true' : 'false');
        });
        $('histPanel').setAttribute('aria-labelledby', tab === 'events' ? 'histTabEvents' : 'histTabRuns');
        document.querySelectorAll('.hist-fields').forEach(el => { el.hidden = el.dataset.for !== tab; });
        writeInputs(tab);       // drops edits that were never applied
        updateFilterBadge();
        const t = state.tabs[tab];
        t.items = [];           // never show the other tab's rows while loading
        renderPager();
        load();
    }

    function init() {
        document.querySelectorAll('.hist-tab').forEach(b => b.addEventListener('click', () => switchTab(b.dataset.tab)));
        document.querySelector('.hist-presets').addEventListener('click', e => {
            const b = e.target.closest('[data-preset]');
            if (b) applyPreset(b.dataset.preset);
        });
        ['histFrom', 'histTo'].forEach(id => $(id).addEventListener('input', () => { state.preset = null; markPreset(); }));
        $('histApply').addEventListener('click', applyFilters);
        $('histClear').addEventListener('click', clearFilters);
        $('histDownload').addEventListener('click', downloadExcel);
        $('histRefresh').addEventListener('click', () => { loadMeta(); load(); });
        $('histFilterToggle').addEventListener('click', () => setFiltersOpen(!$('histFilters').classList.contains('open')));

        // Enter in a text field applies the filters
        document.querySelectorAll('#histFilters input[type="text"]').forEach(el => {
            el.addEventListener('keydown', e => { if (e.key === 'Enter' && !e.isComposing) applyFilters(); });
        });

        // Keep letter keys typed in a <select> away from other pages' global keyboard shortcuts
        $('page-system').addEventListener('keydown', e => {
            if (e.target.tagName === 'SELECT' && e.key !== 'Escape') e.stopPropagation();
        });

        $('histPageSize').addEventListener('change', e => {
            const t = state.tabs[state.tab];
            t.pageSize = parseInt(e.target.value, 10) || 25;
            t.page = 1;
            load();
        });
        $('histSort').addEventListener('click', () => {
            const t = state.tabs[state.tab];
            t.sort = t.sort === 'desc' ? 'asc' : 'desc';
            t.page = 1;
            load();
        });
        $('histPages').addEventListener('click', e => {
            const b = e.target.closest('button[data-page]');
            if (!b || b.disabled) return;
            state.tabs[state.tab].page = parseInt(b.dataset.page, 10) || 1;
            load();
            if (isMobile()) $('histPanel').scrollIntoView({ block: 'start' });
            else $('histScroll').scrollTop = 0;
        });
        $('histState').addEventListener('click', e => {
            const b = e.target.closest('[data-act]');
            if (!b) return;
            if (b.dataset.act === 'retry') { loadMeta(); load(); }
            if (b.dataset.act === 'clear') clearFilters();
        });

        // Results rows -> detail
        const body = $('histBody');
        body.addEventListener('click', e => {
            if (state.tab !== 'runs') return;
            const tr = e.target.closest('tr[data-id]');
            if (tr) openRun(tr.dataset.id);
        });
        body.addEventListener('keydown', e => {
            if (state.tab !== 'runs' || (e.key !== 'Enter' && e.key !== ' ')) return;
            const tr = e.target.closest('tr[data-id]');
            if (tr) { e.preventDefault(); openRun(tr.dataset.id); }
        });

        // Modal
        $('histRunClose').addEventListener('click', () => closeRun(false));
        $('histRunCloseBtn').addEventListener('click', () => closeRun(false));
        $('histRunModal').addEventListener('click', e => { if (e.target === $('histRunModal')) closeRun(false); });
        $('histRunSave').addEventListener('click', saveRun);
        $('histRunDownload').addEventListener('click', () => {
            if (modal.id != null) triggerDownload(API + '/export/runs.xlsx?run_id=' + encodeURIComponent(modal.id));
        });
        ['histEditScenario', 'histEditNotes', 'histEditMeasured'].forEach(id =>
            $(id).addEventListener('input', () => { modal.dirty = true; setSaveMsg(''); }));
        $('histEditScenario').addEventListener('keydown', e => {
            if (e.key === 'Enter' && !e.isComposing) { e.preventDefault(); saveRun(); }
        });
        $('histEditMeasured').addEventListener('keydown', e => {
            if (e.key === 'Enter' && !e.isComposing) { e.preventDefault(); saveRun(); }
        });

        document.addEventListener('keydown', e => {
            if (!modal.open) return;
            if (e.key === 'Escape') { e.preventDefault(); closeRun(false); return; }
            if (e.key === 'Tab') {           // keep focus inside the dialog
                const items = Array.from($('histRunModal').querySelectorAll('button, input, textarea, select, [tabindex]:not([tabindex="-1"])'))
                    .filter(el => !el.disabled && el.offsetParent !== null);
                if (!items.length) return;
                const first = items[0], last = items[items.length - 1];
                if (e.shiftKey && document.activeElement === first) { e.preventDefault(); last.focus(); }
                else if (!e.shiftKey && document.activeElement === last) { e.preventDefault(); first.focus(); }
            }
        });
        window.addEventListener('resize', () => { if (modal.open) drawPlot(); });

        $('histRunModal').setAttribute('aria-hidden', 'true');
        markPreset();
        updateFilterBadge();
    }

    // Called by navigateToPage('system') in app.js
    window.onNavigateToHistory = function () {
        const title = $('pageTitle');
        if (title) title.textContent = 'History';
        if (!state.ready) {
            init();
            state.ready = true;
            populateSelects();
        }
        loadMeta();
        load();
    };

    // Everything above only touches DOM that exists in index.html; nothing runs until the page is opened.
})();