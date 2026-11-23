// Run: gjs src/amr_web/test/test_mission_display.js
const GLib = imports.gi.GLib;
const path = GLib.build_filenamev([GLib.get_current_dir(), 'src/amr_web/resources/web/js/state.js']);
const [ok, bytes] = GLib.file_get_contents(path);
if (!ok) throw new Error('Cannot read state.js');
var window = globalThis;
eval(new TextDecoder().decode(bytes));

function check(condition, message) {
    if (!condition) throw new Error(message);
}
function supervisor(state, revision, extra = {}) {
    window.acceptMissionSupervisorStatus({state, revision, supervisor_session_id: 'session-1',
        run_id: 'run-1', progress_percent: state === 'DONE' ? 100 : 75, ...extra});
    const owner = window.getMissionOwner();
    check(owner.source === 'supervisor' && owner.status.state === state, `${state}: wrong owner`);
    return owner;
}

window.currentOpMode = 'AUTO';
window.missionTransportConnected = true;
window.missionSnapshotReady = true;
window.latestNavState = window.stampMissionStatus({state: 'IDLE', nav2_running: false});
const phases = ['IDLE', 'GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING', 'RETURN_HOME'];
phases.forEach((state, index) => {
    const owner = supervisor(state, index + 1);
    check(window.getMissionPresentation(owner).label !== 'DONE', `${state}: premature completion`);
    check(window.getMissionProgress(owner).percent < 100, `${state}: premature 100%`);
});
let owner = supervisor('DONE', 7, {active_mission: 'return_home'});
check(window.getMissionPresentation(owner).label === 'DONE', 'DONE label missing');
check(window.getMissionPresentation(owner).detail === 'Mission completed', 'completion text missing');
check(window.getMissionProgress(owner).percent === 100, 'DONE progress is not 100%');

// Nav2 idle and a later supervisor heartbeat cannot clear terminal truth.
window.latestNavState = window.stampMissionStatus({state: 'IDLE', nav2_running: false});
owner = supervisor('DONE', 8, {active_mission: 'return_home'});
check(window.getMissionOwner().source === 'supervisor', 'Nav2 idle displaced DONE');
window.acceptMissionSupervisorStatus({state: 'IDLE', revision: 6,
    supervisor_session_id: 'session-1', run_id: 'run-1'});
check(window.getMissionOwner().status.state === 'DONE', 'older heartbeat displaced DONE');

let revision = 8;
for (const state of ['RECOVERY', 'FAILED', 'STOPPED']) {
    owner = supervisor(state, ++revision);
    check(window.getMissionPresentation(owner).label !== 'DONE', `${state} shown as DONE`);
    check(window.getMissionProgress(owner).percent < 100, `${state} shown as 100%`);
}
window.currentPalletRetrievalStatus = window.stampMissionStatus({state: 'HOMING_LIFT', run_id: 'retrieval-1'});
check(window.getMissionOwner().source === 'retrieval', 'active retrieval lost ownership');
window.currentPalletRetrievalStatus = window.stampMissionStatus({state: 'DONE', run_id: 'retrieval-1'});
check(window.getMissionOwner().source === 'retrieval', 'terminal retrieval lost ownership');
window.currentPalletRetrievalStatus = null;
window.suppressTerminalPalletRetrievalStatus = true;
owner = supervisor('GOTO_PICK', ++revision, {run_id: 'run-2'});
check(window.getMissionPresentation(owner).label !== 'DONE', 'new mission retained DONE');
owner = supervisor('IDLE', ++revision, {run_id: 'run-2'});
check(window.getMissionPresentation(owner).label === 'Idle', 'reset did not clear completion');
owner = supervisor('DONE', ++revision, {run_id: 'run-2', can_start: true});
window.currentPalletRetrievalStatus = window.stampMissionStatus({
    state: 'CANCELED', run_id: 'retrieval-2', discarded: true, superseded: true});
owner = window.getMissionOwner();
check(owner.source === 'navigation' && owner.status.state === 'IDLE', 'discard showed prior AUTO DONE');
check(window.getMissionProgress(owner) === null, 'discard retained 100% progress');
window.acceptMissionSupervisorStatus({state: 'GOTO_PICK', revision: ++revision,
    supervisor_session_id: 'session-1', run_id: 'run-3'});
check(window.getMissionOwner().source === 'supervisor', 'new AUTO mission stayed dismissed');
window.acceptMissionSupervisorStatus({state: 'DONE', revision: ++revision,
    supervisor_session_id: 'session-1', run_id: 'run-3'});
check(window.getMissionOwner().source === 'supervisor', 'new AUTO completion was hidden by old discard');
print('Mission display transitions passed');
