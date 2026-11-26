import GLib from 'gi://GLib';
import {createCloudTracker} from '../resources/web/js/cloud_obstacles.js';

const [ok, bytes] = GLib.file_get_contents(
    GLib.build_filenamev([GLib.path_get_dirname(import.meta.url.replace('file://', '')),
                          '../resources/web/js/point_cloud_layer.js']));
if (!ok) throw new Error('Cannot read point cloud layer');
const source = new TextDecoder().decode(bytes)
    .replace(/^import[^;]+;\s*/gm, '')
    .replace('export function createPointCloudLayer', 'function createPointCloudLayer');

function assert(condition, message) {
    if (!condition) throw new Error(message);
}

class Resource { dispose() { this.disposed = true; } }
class Mesh {
    constructor(geometry, material) {
        this.geometry = geometry;
        this.material = material;
        this.position = {set: (...values) => { this.center = values; }};
        this.scale = {set: (...values) => { this.dimensions = values; }};
    }
}
const scene = {added: [], add(object) { this.added.push(object); },
               remove(object) { this.added = this.added.filter(item => item !== object); }};
const createLayer = new Function('THREE', 'createCloudTracker', source + '\nreturn createPointCloudLayer;')(
    {BoxGeometry: Resource, MeshLambertMaterial: Resource, Mesh}, createCloudTracker);
let suppressed = false;
const layer = createLayer(scene, boxes => suppressed ? [] : boxes);
const obstacle = [[1.15, 2, 0.15], [1.15, 2, 0.29], [1.15, 2, 0.43],
                  [1.29, 2, 0.15], [1.29, 2, 0.29], [1.29, 2, 0.43],
                  [NaN, 2, 1], [1.15, 2, 0.01], [1.15, 2, 3]];
const message = (ms, points = obstacle, frame = 'map') =>
    ({frame_id: frame, stamp: {sec: 10, nanosec: 1e6 * ms}, points});

assert(!layer.update(message(0, obstacle, 'camera'), 'map', 0), 'wrong frame');
assert(layer.update(message(0), 'map', 0), 'valid map frame');
assert(!layer.update(message(0), 'map', 1), 'duplicate timestamp');
layer.tick(0);
layer.update(message(200), 'map', 200);
layer.tick(400);
assert(!scene.added.length, 'two samples without sustained tracking are insufficient');
layer.update(message(600), 'map', 600);
layer.tick(800);
assert(scene.added.length === 1, 'recurrent cluster becomes an object');
const mesh = scene.added[0];
assert(Math.abs(mesh.center[2] - 0.29) < 1e-9, 'height uses the observed Z interval');
layer.update(message(900), 'map', 900);
layer.tick(1200);
assert(mesh === scene.added[0], 'reuse mesh on observation refresh');
suppressed = true;
layer.refresh();
assert(!scene.added.length, 'semantic or wall match hides the estimated box');
suppressed = false;
layer.refresh();
assert(scene.added.length === 1, 'new priority decision can reveal the estimate');
layer.tick(2300);
assert(!scene.added.length, 'TTL is measured from last real observation');
layer.clear();
assert(layer.update(message(0, [[1, 2, 0.3]]), 'map', 2400), 'reset stamp on map change');
layer.update(message(400, [[1, 2, 0.3]]), 'map', 2800);
layer.update(message(800, [[1, 2, 0.3]]), 'map', 3200);
layer.tick(3200);
assert(!scene.added.length, 'sparse observations remain as map footprints');
layer.dispose();
assert(mesh.geometry.disposed && mesh.material.disposed, 'release shared geometry and material');

// A repeated chain can connect multiple objects but does not support the huge
// XY rectangle between the ends of a diagonal component.
const sparseTracker = createCloudTracker();
const sparse = [];
for (let i = 0; i < 16; i++) {
    for (const z of [.15,.29,.43]) {
        sparse.push([.02+i*.14, .02+i*.14, z], [.16+i*.14, .02+i*.14, z]);
    }
}
for (const time of [0,200,400,600,800]) {
    sparseTracker.update(message(time, sparse), 'map', time);
    assert(!sparseTracker.tick(time)?.length, 'sparse chain must not make a giant box');
}

// Independent interior clusters on either side of a wall stay separate.
const splitTracker = createCloudTracker({
    canConnect: (a,b) => !((a[0] < 0 && b[0] >= 0) || (a[0] >= 0 && b[0] < 0)),
});
const split = [];
for (const x of [-.12,-.10,.02,.04]) {
    for (const y of [.01,.16]) for (const z of [.15,.29,.43]) split.push([x,y,z]);
}
let splitResult = null;
for (const time of [0,200,400,600,800]) {
    splitTracker.update(message(time,split), 'map', time);
    splitResult = splitTracker.tick(time) || splitResult;
}
assert(splitResult?.length === 2 && splitResult[0].x < 0 && splitResult[1].x > 0,
       'voxel components cannot be joined through a structural wall');

const mapLayer = createLayer(scene, boxes => boxes);
for (const time of [0,200,400,600,800]) {
    mapLayer.update(message(time), 'map', time);
    mapLayer.tick(time);
}
assert(scene.added.length === 1, 'stable tracked box is mounted before map change');
mapLayer.clear();
assert(!scene.added.length, 'map change removes tracked boxes immediately, without waiting for TTL');
mapLayer.dispose();
print('point cloud obstacle tests passed');
