import {createSceneManager} from '../resources/web/js/scene_manager.js';

function assert(condition, message) {
    if (!condition) throw new Error(message);
}

// GJS does not provide browser AbortController. The fake fetch deliberately
// resolves after cancellation to exercise the selection guard as well.
globalThis.AbortController ??= class {
    signal = {aborted: false};
    abort() { this.signal.aborted = true; }
};
const requests = [];
const fetchScene = (id, signal) => new Promise((resolve, reject) => {
    requests.push({id, signal, resolve, reject});
});
const grid = id => ({map_id: id, is_pgm_raw: true, frame_id: 'map'});
const a = 'Warehouse A.2', b = 'Kho B';
const layoutA = {map_id: a, objects: [{id: 'rack_1'}]};
const state = id => ({map_id: id, objects: [{id: 'pallet_1'}]});
let active = null, cargo = null, ready = 0, now = 1000;
const viewer = {
    clearMap() { active = cargo = null; },
    setMap(map, layout) { active = {map, layout}; },
    setSemanticState(message, observedAt) { cargo = {message, observedAt}; return true; },
};
const errors = [];
const manager = createSceneManager({viewer, fetchScene, onReady: () => ready++,
    onError: error => errors.push(error), now: () => now});

manager.select(a);
assert(!manager.commit(grid(a)), 'wait for the matching scene response');
manager.setState(state(a));
manager.select(b);
assert(requests[0].signal.aborted && !active && !cargo, 'switch clears old request, geometry and cargo');
requests[0].resolve(layoutA);
await Promise.resolve();
assert(ready === 0 && !active, 'late old layout is ignored');
assert(!manager.setState(state(a)), 'shared ROS frame does not admit the previous map cargo');
requests[1].resolve(null);
await Promise.resolve();
assert(!manager.commit(grid(a)) && manager.commit(grid(b)), 'only selected grid can mount');
assert(active.layout === null && !cargo, 'map without config mounts as an empty semantic scene');

manager.select(a);
manager.setState(state(a));
now += 200;
requests[2].resolve(layoutA);
await Promise.resolve();
assert(manager.commit(grid(a)) && active.layout === layoutA, 'matching map and config mount together');
assert(cargo.observedAt === 1000, 'pending state keeps its original age');
manager.select(b);
assert(!active && !cargo, 'switch disposes the current scene immediately');
requests[3].reject(new Error('scene file unavailable'));
await Promise.resolve();
assert(errors.length === 1 && manager.commit(grid(b)) && active.layout === null,
       'layout error still allows grid-only rendering');
manager.select(a);
manager.setState(state(a));
now += 3100;
requests[4].resolve(layoutA);
await Promise.resolve();
assert(manager.commit(grid(a)) && !cargo, 'expired pending cargo is discarded');
manager.refresh();
const movedRack = {...layoutA, objects: [{id: 'rack_1', x: 4, yaw: .25}]};
requests[5].resolve(movedRack);
await Promise.resolve();
assert(manager.commit(grid(a)) && active.layout === movedRack, 'changed same-route rack rebuilds the scene');
const beforeReady = ready;
manager.refresh();
requests[6].resolve(JSON.parse(JSON.stringify(movedRack)));
await Promise.resolve();
assert(ready === beforeReady, 'unchanged layout does not rebuild');
const sameGrid = active.map;
manager.select(a, 'another-route');
assert(active.map === sameGrid && manager.commit(sameGrid), 'same-map route switches retain mounted grid');
requests[7].resolve(movedRack);
await Promise.resolve();
assert(manager.commit(sameGrid), 'same grid still accepts the refreshed route layout');
print('multi-map scene manager tests passed');
