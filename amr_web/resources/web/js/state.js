// Global Application State
// P0-6 Fix: Use window.* for cross-module access (single source of truth)
window.mapData = null;
window.robotPose = null;
window.mapImageData = null;
let isWsConnected = false;
let isRobotConnected = false;
let activityLog = [];
let alarmLog = [];
const MAX_ACTIVITY = 50;
const MAX_ALARMS = 20;
let alarmFilter = 'all';

// Device & System State (Heartbeat Monitoring)
let deviceStates = {
    esp32: { connected: false, hz: 0 },
    lidar: { connected: false, hz: 0 },
    camera: { connected: false, hz: 0 },
    lift: { connected: false, hz: 0 }
};
let systemStatus = 'Offline';  // "Online" | "Degraded" | "Offline"

// Map View Transform State
let viewTransform = { x: 0, y: 0, scale: 1 };
let isDragging = false;
let dragStart = { x: 0, y: 0 };
let transformStart = { x: 0, y: 0, scale: 1 };
let canvas = null;
let ctx = null;

// SLAM Page State (on window for cross-module access)
window.isSlamActive = false;
window.laserData = null;  // Latest laser scan data
window.slamPendingStart = false;  // Track if start_slam command is pending

// Navigation Page State (on window for cross-module access)
window.isNavigationActive = false;

// Display ownership is independent of ROS control ownership. Only fresh active
// sources compete; a missing/stale active source must never look like Idle.
window.missionActiveStates = {
    retrieval: new Set(['HOMING_LIFT', 'NAVIGATING_TO_D1', 'ALIGNING_TO_SLOT',
        'AUTO_RECOVERING', 'WAITING_FOR_SENSOR', 'LOADING_DETECTOR', 'DETECTING', 'PICKING', 'NAVIGATING_TO_D2',
        'PLACING_AT_D2', 'RETURNING_HOME', 'CANCELING', 'RECOVERY']),
    supervisor: new Set(['GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING',
        'WAITING_SLOT', 'RETURN_HOME', 'RECOVERY', 'STOPPING']),
    pallet: new Set(['DETECTING', 'NAVIGATING', 'SETTLING', 'ALIGNING',
        'INSERTING', 'AUTO_RECOVERING', 'WAITING_FOR_SENSOR',
        'LIFTING', 'BACKING', 'HOMING', 'LOWERING']),
    navigation: new Set(['SENDING', 'ACCEPTED', 'ACTIVE', 'CANCELING', 'UNKNOWN', 'STARTING'])
};
// Age transport receipts locally; browser and robot wall clocks may differ.
window.stampMissionStatus = status => status ? {...status, received_at: Date.now()} : null;
window.missionStatusFresh = status => !!status && !status.stale &&
    window.missionTransportConnected !== false &&
    (!Number.isFinite(status.received_at) || Date.now() - status.received_at <= 3000);
// UI request bookkeeping; task truth still comes only from supervisor snapshots.
window.autoMissionRequest = null;
window.acceptMissionSupervisorStatus = status => {
    const previous = window.currentMissionSupervisorStatus;
    if (previous && status && previous.supervisor_session_id &&
            previous.supervisor_session_id === status.supervisor_session_id &&
            Number.isFinite(previous.revision) && Number.isFinite(status.revision) && status.revision < previous.revision) return;
    window.currentMissionSupervisorStatus = window.stampMissionStatus(status);
};
window.autoMissionStartPending = false;
window.retrievalStartPending = false;
// Resume is an acknowledgement within the same run, not a new mission start.
window.retrievalResumePending = null;
window.resolveRetrievalResume = status => {
    const pending = window.retrievalResumePending;
    if (!pending || !window.missionStatusFresh(status)) return;
    const newRun = pending.runId && status.run_id && status.run_id !== pending.runId;
    const acknowledged = Number.isFinite(status.resume_revision) && status.resume_revision > pending.revision;
    const changedPhase = !['RECOVERY', 'CANCELING'].includes(status.state);
    if (newRun || acknowledged || changedPhase) window.retrievalResumePending = null;
};
window.getMissionOwner = (() => {
    const previous = {};
    let lastActiveSource = null;
    let discardedSupervisorKey = null;
    let discardedRetrievalRunId = null;
    return () => {
        if (window.missionTransportConnected === false ||
                (window.missionTransportConnected === true && !window.missionSnapshotReady)) {
            return {source: 'unavailable', status: {}, active: false, stale: true};
        }
        const sources = {
            retrieval: window.currentPalletRetrievalStatus,
            supervisor: window.currentMissionSupervisorStatus,
            pallet: window.currentCargoTelemetry,
            navigation: window.latestNavState
        };
        for (const source of Object.keys(sources)) {
            if (sources[source]) previous[source] = sources[source];
            else if (window.missionActiveStates[source].has(previous[source]?.state)) {
                sources[source] = {...previous[source], stale: true};
            }
        }
        for (const source of Object.keys(sources)) {
            const status = sources[source];
            if (source === 'supervisor' && status?.web_control_pending && window.missionStatusFresh(status)) {
                return {source, status, active: true, stale: false};
            }
            if (window.missionActiveStates[source].has(status?.state) && !window.missionStatusFresh(status)) {
                // An in-flight/paused retrieval still owns the presentation
                // while its heartbeat is stale. Mark it unavailable, but never
                // switch to AUTO or imply that the robot was released.
                if (source === 'retrieval') return {source, status, active: true, stale: true};
                return {source: 'unavailable', status: {}, active: false, stale: true};
            }
            if (window.missionStatusFresh(status) &&
                    window.missionActiveStates[source].has(status.state)) {
                lastActiveSource = source;
                if (source !== 'retrieval' && sources.retrieval &&
                        ['DONE', 'FAILED', 'CANCELED', 'STOPPED'].includes(sources.retrieval.state)) {
                    window.suppressTerminalPalletRetrievalStatus = true;
                }
                return {source, status, active: true, stale: false};
            }
            // A start awaiting acknowledgement owns the display as well.
            if ((source === 'retrieval' && window.retrievalStartPending) ||
                    (source === 'supervisor' && window.autoMissionStartPending)) {
                return {source, status: {state: 'STARTING'}, active: true, stale: false};
            }
        }
        if (Object.entries(sources).some(([source, status]) =>
                window.missionActiveStates[source].has(status?.state))) {
            return {source: 'unavailable', status: {}, active: false, stale: true};
        }
        const retrieval = sources.retrieval;
        const supervisor = sources.supervisor;
        const supervisorKey = supervisor && JSON.stringify([
            supervisor.supervisor_session_id, supervisor.run_id, supervisor.revision]);
        if (retrieval?.discarded && retrieval.state === 'CANCELED' && supervisor?.state === 'DONE' &&
                retrieval.run_id && retrieval.run_id !== discardedRetrievalRunId) {
            discardedRetrievalRunId = retrieval.run_id;
            discardedSupervisorKey = retrieval.discarded_supervisor_key
                ? JSON.stringify(retrieval.discarded_supervisor_key) : supervisorKey;
        }
        if (supervisor?.state === 'DONE' && discardedSupervisorKey === supervisorKey &&
                window.missionStatusFresh(supervisor)) {
            return {source: 'navigation', status: {state: 'IDLE'}, active: false, stale: false};
        }
        if (window.currentOpMode === 'AUTO' && window.missionStatusFresh(retrieval) &&
                !retrieval.discarded && !retrieval.superseded &&
                !window.suppressTerminalPalletRetrievalStatus &&
                ['DONE', 'FAILED', 'CANCELED', 'STOPPED'].includes(retrieval.state)) {
            return {source: 'retrieval', status: retrieval, active: false, stale: false};
        }
        const pallet = sources.pallet;
        if (window.currentOpMode === 'AUTO' && supervisor && !window.missionStatusFresh(supervisor)) {
            return {source: 'unavailable', status: {}, active: false, stale: true};
        }
        if (window.currentOpMode === 'AUTO' && window.missionStatusFresh(supervisor) &&
                !(supervisor.state === 'IDLE' && supervisor.can_start === false &&
                  supervisor.stop_confirmed !== true && pallet?.recovery?.checkpoint)) {
            return {source: 'supervisor', status: supervisor, active: false, stale: false};
        }
        if (window.missionStatusFresh(pallet) && ['pick', 'drop'].includes(pallet.mode) &&
                ((lastActiveSource === 'pallet' && ['READY', 'DONE', 'FAILED', 'STOPPED', 'CARRYING'].includes(pallet.state)) ||
                 (supervisor?.state === 'IDLE' && ['READY', 'CARRYING'].includes(pallet.state)))) {
            return {source: 'pallet', status: pallet, active: false, stale: false};
        }

        if (window.missionStatusFresh(pallet) && pallet.mode &&
                ['READY', 'DONE', 'FAILED', 'STOPPED', 'CARRYING'].includes(pallet.state)) {
            return {source: 'pallet', status: pallet, active: false, stale: false};
        }
        if ((window.currentOpMode === 'AUTO' && !window.missionStatusFresh(supervisor)) ||
                (sources.navigation && !window.missionStatusFresh(sources.navigation))) {
            return {source: 'unavailable', status: {}, active: false, stale: true};
        }
        return {source: 'navigation', status: sources.navigation || {state: 'IDLE'},
            active: false, stale: false};
    };
})();
window.missionStateLabels = {
    IDLE: 'Idle', STOPPING: 'Stopping...', STARTING: 'Starting', SENDING: 'Sending goal', ACCEPTED: 'Goal accepted',
    UNKNOWN: 'State unknown', HOMING_LIFT: 'Homing',
    NAVIGATING_TO_D1: 'Navigating', ALIGNING_TO_SLOT: 'Aligning',
    LOADING_DETECTOR: 'Loading detector', DETECTING: 'Detecting', PICKING: 'Picking',
    NAVIGATING_TO_D2: 'Navigating', PLACING_AT_D2: 'Placing', RETURNING_HOME: 'Returning',
    CANCELING: 'Canceling', GOTO_PICK: 'Navigating', GOTO_D: 'Navigating',
    DROPPING: 'Placing', WAITING_SLOT: 'Waiting', RETURN_HOME: 'Returning',
    RECOVERY: 'Recovery', NAVIGATING: 'Navigating', SETTLING: 'Settling',
    ALIGNING: 'Aligning', INSERTING: 'Inserting', LIFTING: 'Lifting', BACKING: 'Backing',
    AUTO_RECOVERING: 'Auto correcting alignment', WAITING_FOR_SENSOR: 'Waiting for sensor',
    NEED_OPERATOR: 'Operator check required',
    HOMING: 'Homing', LOWERING: 'Lowering', READY: 'Ready', CARRYING: 'Holding',
    ACTIVE: 'Navigating', SUCCEEDED: 'Done', DONE: 'Done', FAILED: 'Failed',
    CANCELED: 'Canceled', STOPPED: 'Stopped'
};

// Render content from the selected owner, without changing control ownership.
// Supervisor pm_* is the authoritative AUTO snapshot, never the raw pallet
// cache left by a previous leg. Its publisher clears pm_state on service ACK.
window.getMissionPresentation = (() => {
    let context = '', observedPallet = false;
    const slots = new Set(['A1', 'A2', 'B1', 'B2']);
    const terminal = new Set(['DONE', 'FAILED', 'CANCELED', 'STOPPED']);
    const palletLabels = {DETECTING: 'Detecting', READY: 'Pallet Ready',
        NAVIGATING: 'To Staging', SETTLING: 'Settling', ALIGNING: 'Aligning',
        INSERTING: 'Inserting', AUTO_RECOVERING: 'Auto correcting alignment',
        WAITING_FOR_SENSOR: 'Waiting for sensor', LIFTING: 'Lifting', BACKING: 'Backing',
        LOWERING: 'Lowering', HOMING: 'Homing', CARRYING: 'Carrying',
        DONE: 'Done', FAILED: 'Failed', STOPPED: 'Stopped'};
    return (owner = window.getMissionOwner()) => {
        const m = owner.status;
        const state = m.state;
        const view = {label: owner.stale ? 'Unavailable' :
            (window.missionStateLabels[state] || 'Unavailable'), target: '--', detail: ''};
        if (owner.stale) return view;
        const handling = m.handling_status || m.pm_handling_status;
        if (handling === 'AUTO_RECOVERING') view.label = 'Auto correcting alignment';
        else if (handling === 'WAITING_FOR_SENSOR') view.label = 'Waiting for sensor';
        else if (handling === 'NEED_OPERATOR') view.label = 'Operator check required';
        else if (handling === 'FAILED') view.label = 'Failed';
        if (owner.source !== 'supervisor') { context = ''; observedPallet = false; }
        if (owner.source === 'retrieval') {
            const labels = {HOMING_LIFT: 'Homing', ALIGNING_TO_SLOT: 'Aligning',
                AUTO_RECOVERING: 'Auto correcting alignment', WAITING_FOR_SENSOR: 'Waiting for sensor',
                LOADING_DETECTOR: 'Loading', DETECTING: 'Detecting', PICKING: 'Picking',
                NAVIGATING_TO_D2: 'To D_2', PLACING_AT_D2: 'Placing', RETURNING_HOME: 'To Home'};
            const stageStates = ['HOMING_LIFT', 'NAVIGATING_TO_D1', 'NAVIGATING_TO_D1',
                'ALIGNING_TO_SLOT', 'DETECTING', 'PICKING', 'NAVIGATING_TO_D2',
                'PLACING_AT_D2', 'PLACING_AT_D2', 'RETURNING_HOME'];
            const paused = ['RECOVERY', 'CANCELING'].includes(state);
            const stage = Number.isInteger(m.resume_step) ? stageStates[m.resume_step] : null;
            const phase = paused ? (m.recovery_from || stage) : state;
            const leg = paused ? (m.recovery_nav_leg ||
                (m.resume_step === 1 ? 'd1' : m.resume_step === 2 ? 'slot' : null)) : m.nav_leg;
            const pending = window.retrievalResumePending;
            view.resuming = state === 'RECOVERY' && (m.resuming === true || !!pending);
            view.label = view.resuming ? (pending?.uncertain ? 'Resume pending' : 'Resuming') : labels[state] || view.label;
            if (phase === 'NAVIGATING_TO_D1') {
                if (!paused) view.label = leg === 'd1' ? 'To D_1' : leg === 'slot' ? 'To Slot' : 'Navigating';
                view.target = leg === 'd1' ? 'D_1' : leg === 'slot' && slots.has(m.slot) ? m.slot : '--';
            } else if (['NAVIGATING_TO_D2', 'PLACING_AT_D2'].includes(phase)) view.target = 'D_2';
            else if (phase === 'RETURNING_HOME') view.target = 'HOME';
            else if (!terminal.has(state) && slots.has(m.slot)) view.target = m.slot;
            if (view.resuming) {
                view.detail = m.resuming ? 'Restoring the saved retrieval operation' :
                    'Waiting for retrieval Resume acknowledgement';
            } else if (['FAILED', 'CANCELED', 'STOPPED', 'RECOVERY'].includes(state) || m.recovery_required) {
                const cp = m.operation_checkpoint;
                const operation = cp && !['DONE', 'IDLE', 'CARRYING'].includes(cp.state) ?
                    `Paused: ${palletLabels[cp.state] || cp.state}${Number.isFinite(cp.lift_target_cm) ? ` → ${cp.lift_target_cm} cm` : ''}` : '';
                view.detail = [...new Set([operation, m.message, m.resume_reason].filter(Boolean))].join(' · ');
            }
            return view;
        }
        if (owner.source === 'supervisor') {
            if (state === 'DONE') {
                view.label = 'DONE';
                view.detail = 'Mission completed';
                return view;
            }
            const phase = ['PICKING', 'DROPPING'].includes(state) ? state :
                state === 'RECOVERY' && ['PICKING', 'DROPPING'].includes(m.resume_state) ? m.resume_state : null;
            const key = JSON.stringify([window.missionDisplaySession || 0, phase,
                m.active_mission, m.active_dest, m.cycle?.number]);
            if (key !== context) { context = key; observedPallet = false; }
            const mode = phase === 'DROPPING' ? 'drop' : 'pick';
            const raw = window.currentCargoTelemetry;
            const modeMismatch = window.missionStatusFresh(raw) && raw.mode && raw.mode !== mode;
            const beforePhase = Number.isFinite(m.pm_status_age_s) && Number.isFinite(m.state_elapsed_s) &&
                m.pm_status_age_s > m.state_elapsed_s + 0.1 && state !== 'RECOVERY';
            const age = Number.isFinite(m.pm_status_age_s) ? m.pm_status_age_s +
                (Number.isFinite(m.received_at) ? Math.max(0, Date.now() - m.received_at) / 1000 : 0) : 0;
            // Match the producer's 1.5 s pm_status_fresh window as cached
            // supervisor snapshots age between WebSocket updates.
            let related = !!phase && m.pm_status_fresh === true && age <= 1.5 && !modeMismatch && !beforePhase;
            const pmState = m.pm_state;
            // Sampling cannot inherit a previous run's insertion or result.
            if (phase === 'PICKING' && m.progress_phase === 'sampling' &&
                    !['DETECTING', 'READY'].includes(pmState)) related = false;
            if (phase === 'PICKING' && m.progress_phase === 'sampling' &&
                    pmState === 'READY' && !observedPallet) related = false;
            if (mode === 'drop' && ['DETECTING', 'READY', 'CARRYING', 'HOMING'].includes(pmState)) related = false;
            if (related && window.missionActiveStates.pallet.has(pmState)) observedPallet = true;
            if (related && ['DONE', 'FAILED', 'STOPPED', 'CARRYING'].includes(pmState) &&
                    !observedPallet) related = false;
            if (['PICKING', 'DROPPING'].includes(state) && related && palletLabels[pmState]) {
                view.label = mode === 'drop' && pmState === 'LIFTING' ? 'Lift Moving' :
                    mode === 'drop' && pmState === 'DONE' ? 'Placed' : palletLabels[pmState];
            }
            if (m.web_control_pending || state === 'STOPPING' || state === 'RECOVERY' && m.stop_confirmed === false) {
                view.label = m.cancel_pending || m.web_control_action === 'mission_cycle_cancel' ? 'Canceling...' : 'Stopping...';
            }
            if (['GOTO_PICK', 'PICKING'].includes(state) || state === 'RECOVERY' && phase === 'PICKING') view.target = m.active_dest || '--';
            else if (state === 'GOTO_D' || state === 'RECOVERY' && m.resume_state === 'GOTO_D') view.target = 'D_1';
            else if (phase === 'DROPPING' && slots.has(m.drop_slot)) view.target = m.drop_slot;
            else if (state === 'RETURN_HOME' || state === 'RECOVERY' && m.resume_state === 'RETURN_HOME') view.target = 'HOME';
            if (['RECOVERY', 'FAILED', 'STOPPED'].includes(state)) {
                const cp = m.operation_checkpoint;
                const operation = cp ? `${cp.state}${Number.isFinite(cp.lift_target_cm) ? ` → ${cp.lift_target_cm} cm` : ''}` : '';
                view.detail = [m.msg, operation, m.resume_reason].filter(Boolean).join(' · ');
            }
            if (related && (pmState === 'FAILED' || ['RECOVERY', 'FAILED'].includes(state))) {
                const diagnostic = [m.pm_msg, m.pm_fail_phase && `[${m.pm_fail_phase}]`].filter(Boolean).join(' ');
                if (diagnostic && !view.detail.includes(diagnostic)) view.detail = [view.detail, diagnostic].filter(Boolean).join(' · ');
            }
            // A phase label from the child snapshot must not hide its explicit
            // recovery/wait/operator state. Stop presentation still takes priority.
            const handling = m.handling_status || m.pm_handling_status;
            if (!m.web_control_pending && state !== 'STOPPING' &&
                    !(state === 'RECOVERY' && m.stop_confirmed === false)) {
                if (handling === 'AUTO_RECOVERING') view.label = 'Auto correcting alignment';
                else if (handling === 'WAITING_FOR_SENSOR') view.label = 'Waiting for sensor';
                else if (handling === 'NEED_OPERATOR') view.label = 'Operator check required';
                else if (handling === 'FAILED') view.label = 'Failed';
            }
            if (['AUTO_RECOVERING', 'WAITING_FOR_SENSOR'].includes(handling)) {
                const err = m.pm_err || {};
                const retries = m.pm_retries || {};
                const metrics = [m.pm_phase,
                    Number.isFinite(err.lat_cm) ? `Lateral ${err.lat_cm.toFixed(1)} cm` : null,
                    Number.isFinite(err.yaw_deg) ? `Yaw ${err.yaw_deg.toFixed(1)}°` : null,
                    Number.isInteger(retries.alignment) ? `Retry ${retries.alignment}` : null,
                    m.pm_msg].filter(Boolean);
                view.detail = metrics.join(' · ');
            } else view.detail = '';
            return view;
        }
        if (owner.source === 'pallet') {
            context = ''; observedPallet = false;
            view.label = palletLabels[state] || view.label;
            // NAVIGATING is published only by the PICK/DROP staging senders.
            if (state === 'NAVIGATING' && !['pick', 'drop'].includes(m.mode)) view.label = 'Navigating';
            if (m.mode === 'drop' && state === 'LIFTING') view.label = 'Lift Moving';
            if (m.mode === 'drop' && state === 'DONE') view.label = 'Placed';
            if (!terminal.has(state)) view.target = m.mode === 'drop'
                ? (slots.has(m.slot) ? m.slot : '--') : m.mode === 'pick' ? 'Pallet' : '--';
            if (['FAILED', 'STOPPED'].includes(state)) {
                const cp = m.recovery?.checkpoint;
                const operation = cp ? `${cp.state}${Number.isFinite(cp.lift_target_cm) ? ` → ${cp.lift_target_cm} cm` : ''}` : '';
                view.detail = [m.msg, operation, m.recovery?.reason, m.fail_phase && `[${m.fail_phase}]`].filter(Boolean).join(' · ');
            }
        }
        return view;
    };
})();

// These are observed milestones, not time-based estimates. A cycle/owner/new
// retrieval run resets the accumulator; retries within it cannot move it back.
window.getMissionProgress = (() => {
    let key = '', value = 0, step = 0, lastState = '';
    const retrievalStates = ['HOMING_LIFT', 'NAVIGATING_TO_D1', 'ALIGNING_TO_SLOT',
        'LOADING_DETECTOR', 'DETECTING', 'PICKING', 'NAVIGATING_TO_D2', 'PLACING_AT_D2', 'RETURNING_HOME'];
    return (owner = window.getMissionOwner()) => {
        const m = owner.status, state = m.state;
        if (owner.stale) return {percent: null, text: '--', label: 'Mission progress'};
        const nextKey = JSON.stringify([window.missionDisplaySession || 0, owner.source,
            owner.source === 'supervisor' ? [m.run_id, m.cycle?.number] : null,
            owner.source === 'retrieval' ? m.slot : null, owner.source === 'retrieval' ? m.run_id : null]);
        const newRun = !(owner.source === 'retrieval' && m.run_id) && (owner.active && ['DONE', 'FAILED', 'STOPPED', 'CANCELED', 'IDLE'].includes(lastState) ||
            lastState === 'DONE' && state !== 'DONE' ||
            state === 'IDLE' && lastState !== 'IDLE' ||
            owner.source === 'retrieval' && state === 'HOMING_LIFT' && !['HOMING_LIFT', 'RECOVERY', 'CANCELING'].includes(lastState));
        if (nextKey !== key || newRun) { key = nextKey; value = 0; step = 0; }
        lastState = state;
        if (owner.source === 'retrieval') {
            const stageIndices = [0, 1, 1, 2, 4, 5, 6, 7, 7, 8];
            const paused = ['RECOVERY', 'CANCELING'].includes(state);
            let index = retrievalStates.indexOf(paused ? m.recovery_from : state);
            if (index < 0 && Number.isInteger(m.resume_step)) index = stageIndices[m.resume_step] ?? -1;
            if (index >= 0) step = Math.max(step, index);
            if (state === 'DONE') step = retrievalStates.length;
            value = state === 'DONE' ? 100 : Math.min(99, Math.max(value, step / retrievalStates.length * 100));
            return {percent: value, text: state === 'DONE' ? '9 / 9' :
                ['FAILED', 'CANCELED', 'STOPPED'].includes(state) ? window.missionStateLabels[state] :
                `${Math.min(step + 1, 9)} / 9`, label: state === 'RECOVERY' ?
                    (m.resuming || window.retrievalResumePending ? 'Resuming retrieval' : 'Retrieval paused') : 'Retrieval step'};
        }
        if (owner.source === 'supervisor') {
            if (state === 'DONE') value = 100;
            else if (Number.isFinite(m.progress_percent)) value = Math.max(value, Math.min(99, Math.max(0, m.progress_percent)));
            return {percent: value, text: Number.isFinite(m.progress_percent) || value > 0 ? `${Math.round(value)}%` : '--',
                label: Number.isFinite(m.cycle?.number) &&
                    ['GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING', 'WAITING_SLOT'].includes(state)
                    ? `Cycle ${m.cycle.number + 1} progress` : 'Mission progress'};
        }
        if (owner.source === 'pallet') return {percent: state === 'DONE' ? 100 : null,
            text: state === 'DONE' ? '100%' : '--', label: 'Mission progress'};
        return null; // Manual navigation retains its own goal feedback.
    };
})();

window.notifyMissionError = (() => {
    // Deduplicate by mission/run + significant state, never by changing ROS
    // error text (it can grow on every heartbeat). Detailed diagnostics remain
    // in backend logs, not in user-facing toast notifications.
    const seen = new Set();
    return (owner = window.getMissionOwner(), view = window.getMissionPresentation(owner)) => {
        if (owner.stale || view.resuming) return;
        const m = owner.status || {};
        const state = m.state;
        let notice = '', severity = 'warn';
        if (owner.source === 'supervisor') {
            if (m.recovery_cause === 'user' || !['RECOVERY', 'FAILED'].includes(state)) return;
            notice = 'Mission needs recovery';
        } else if (owner.source === 'retrieval') {
            if (state === 'RECOVERY') {
                const operatorCancel = /RETRIEVAL_CANCELED|Operator requested retrieval cancellation/i.test(m.message || '');
                notice = operatorCancel ? 'Retrieval paused' : 'Retrieval needs recovery';
            } else if (state === 'FAILED') {
                notice = 'Retrieval failed'; severity = 'error';
            } else if (state === 'CANCELED') {
                notice = 'Retrieval canceled'; severity = 'info';
            } else return;
        } else if (owner.source === 'pallet') {
            if (!['FAILED', 'STOPPED'].includes(state)) return;
            notice = state === 'FAILED' ? 'Pallet operation failed' : 'Pallet operation stopped';
            severity = state === 'FAILED' ? 'error' : 'info';
        } else return;
        const key = JSON.stringify([owner.source, m.run_id || m.session_id || '',
            m.resume_revision || 0, state]);
        if (seen.has(key)) return;
        seen.add(key);
        if (seen.size > 128) seen.delete(seen.values().next().value);
        window.showToast?.(notice, severity);
        if (typeof addActivity === 'function') addActivity(severity, notice);
    };
})();

/**
 * Cập nhật laser scan data từ backend
 * @param {Object} d - Laser scan JSON payload
 */
function updateLaser(d) {
    window.laserData = d;
    // Trigger map render to show laser scan overlay
    if (typeof window.renderMap === 'function') {
        window.renderMap();
    }
}

/**
 * Xử lý command response từ backend
 * @param {Object} d - Command response payload
 */
function handleCommandResponse(d) {
    const cmd = d.command || 'unknown';
    const status = d.status || 'unknown';
    const message = d.message || '';

    console.log('Command response:', cmd, status, message);

    if (cmd && cmd.startsWith('llm_')) {
        window.handleLLMCommandResponse?.(d);
        return;
    }

    if (cmd && cmd.startsWith('mission_cycle_')) {
        if (d.handled_by_request && window.autoMissionRequest?.command === cmd) return;
        if (typeof window.handleMissionCycleCommandResponse === 'function') {
            window.handleMissionCycleCommandResponse(d);
        }
        if (typeof window.showToast === 'function') {
            let text;
            if (!['ok', 'started'].includes(status)) text = cmd === 'mission_cycle_resume' ? 'Resume unavailable; check the robot' : 'Mission command not confirmed';
            else if (cmd === 'mission_cycle_start') text = 'Auto cycle started';
            else if (cmd === 'mission_cycle_stop') text = 'Mission paused';
            else if (cmd === 'mission_cycle_resume') text = 'Mission resumed';
            else if (cmd === 'mission_cycle_cancel') text = 'Mission canceled';
            else text = 'Mission command accepted';
            window.showToast(text, status === 'error' ? (cmd === 'mission_cycle_resume' ? 'warn' : 'error') : 'success');
        }
        return;
    }

    if (cmd === 'pallet_retrieval_start') {
        if (d.handled_by_request) return;
        if (typeof window.showToast === 'function') {
            window.showToast(message || (status === 'started'
                ? 'Pallet retrieval started' : 'Pallet retrieval failed'),
            status === 'started' ? 'success' : 'error');
        }
        return;
    }

    if (cmd === 'set_op_mode') {
        return;
    }

    if (cmd === 'get_nav_state' && d.nav_state &&
        typeof window.handleNavStateUpdate === 'function') {
        window.handleNavStateUpdate(d.nav_state);
        return;
    }

    // Phase 5 Fix: Redirect nav commands to navigation.js
    if (['start_navigation', 'stop_navigation', 'nav_goal', 'nav_cancel',
         'waypoint_mission', 'set_initial_pose', 'emergency_stop'].includes(cmd)) {
        if (typeof window.onNavCommandResponse === 'function') {
            window.onNavCommandResponse(d);
        }
        return;
    }

    if (cmd === 'start_slam') {
        window.slamPendingStart = false;

        if (status === 'started') {
            addActivity('info', 'SLAM started successfully');
            window.isSlamActive = true;
            updateSlamUI();
        } else if (status === 'failed') {
            addAlarm('error', 'SLAM start failed: ' + message);
            window.isSlamActive = false;
            updateSlamUI();
        }
    } else if (cmd === 'stop_slam') {
        addActivity('info', 'SLAM stopped');
        window.isSlamActive = false;
        // P0-6 Fix: Only clear map when SLAM actually stops (not on every slam_state=false)
        if (window.mapRenderer) {
            window.mapRenderer.hide(true);
        }
        updateSlamUI();
        updateOpModeDisplay();
    } else if (cmd === 'save_map') {
        if (status === 'success') {
            addActivity('info', 'Map saved successfully');
            if (typeof window.showToast === 'function') {
                window.showToast('Map saved successfully', 'success');
            }
        } else {
            addAlarm('error', 'Map save failed: ' + message);
            if (typeof window.showToast === 'function') {
                window.showToast('Map save failed: ' + message, 'error');
            }
        }
    } else if (cmd === 'teleop') {
        // Teleop doesn't need feedback
    } else if (cmd === 'lift') {
        // Lift doesn't need feedback
    }
}

/**
 * Cập nhật SLAM UI dựa trên trạng thái isSlamActive
 */
function updateSlamUI() {
    const slamWelcome = document.getElementById('slamWelcome');
    const slamLayout = document.getElementById('slamLayout');
    const slamStatusBadge = document.querySelector('.slam-status-badge');

    if (slamWelcome && slamLayout) {
        if (window.isSlamActive) {
            slamWelcome.style.display = 'none';
            slamLayout.style.display = 'grid';
        } else {
            slamWelcome.style.display = 'flex';
            slamLayout.style.display = 'none';
        }
    }

    if (slamStatusBadge) {
        if (window.isSlamActive) {
            slamStatusBadge.textContent = 'SLAM Active';
            slamStatusBadge.className = 'slam-status-badge active';
        } else {
            slamStatusBadge.textContent = 'SLAM Inactive';
            slamStatusBadge.className = 'slam-status-badge';
        }
    }
}

/**
 * Cập nhật Operating Mode display trên Overview card
 */
function updateOpModeDisplay() {
    const opModeEl = document.getElementById('cardOpMode');
    const subModeEl = document.getElementById('cardSubMode');

    if (window.isSlamActive) {
        if (opModeEl) {
            opModeEl.textContent = 'SLAM Mode';
            opModeEl.className = 'metric-value accent';
        }
        if (subModeEl) {
            subModeEl.className = 'mode-badge slam';
            subModeEl.innerHTML = '<span>Mapping Active</span>';
        }
    } else if (window.isNavigationActive) {
        if (opModeEl) {
            opModeEl.textContent = 'Navigation';
            opModeEl.className = 'metric-value accent';
        }
        if (subModeEl) {
            // Lấy Auto/Manual từ navigation.js hoặc backend
            const opMode = window.currentOpMode || 'MANUAL';
            subModeEl.className = 'mode-badge nav';
            subModeEl.innerHTML = `<span>${opMode}</span>`;
        }
    } else {
        // Reset về mặc định
        if (opModeEl) {
            opModeEl.textContent = 'Unknown';
            opModeEl.className = 'metric-value muted';
        }
        if (subModeEl) {
            subModeEl.className = 'mode-badge';
            subModeEl.innerHTML = '<span>--</span>';
        }
    }
}

// B06 Fix: Export updateOpModeDisplay cho ros_connection.js
window.updateOpModeDisplay = updateOpModeDisplay;

