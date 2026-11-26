// Map-frame observations only. Dimensions describe the observed envelope in metres.
export const CLOUD_DEFAULTS = Object.freeze({
    windowMs: 4000, ttlMs: 1400, updateMs: 400, voxel: 0.14, tolerance: 0.12,
    minZ: 0.12, maxZ: 2.6, minSpanZ: 0.20, maxSpanXY: 2.5,
});

const MAX_FRAMES = 24;
const MAX_POINTS = 500;
const MAX_TRACKS = 64;
const voxelKey = (x, y, z) => `${x},${y},${z}`;

function cluster(frames, config, now) {
    const voxels = new Map();
    for (const frame of frames) {
        for (const point of frame.points) {
            const bins = point.map(value => Math.floor(value / config.voxel));
            const key = voxelKey(...bins);
            if (!voxels.has(key)) {
                voxels.set(key, {bins, min: point.slice(), max: point.slice(),
                                 seen: 0, latest: null, at: 0});
            }
            const voxel = voxels.get(key);
            if (voxel.latest !== frame) voxel.seen++;
            voxel.latest = frame;
            voxel.at = frame.at;
            for (let axis = 0; axis < 3; axis++) {
                voxel.min[axis] = Math.min(voxel.min[axis], point[axis]);
                voxel.max[axis] = Math.max(voxel.max[axis], point[axis]);
            }
        }
    }
    // Repeated voxel observations reject one-frame noise and most moving trails.
    for (const [key, voxel] of voxels) {
        if (voxel.seen < 2 || now - voxel.at >= config.ttlMs) voxels.delete(key);
    }
    const candidates = [];
    for (const [key, first] of voxels) {
        const queue = [first];
        const min = first.min.slice(), max = first.max.slice();
        let lastSeen = first.at;
        voxels.delete(key);
        for (let head = 0; head < queue.length; head++) {
            const {bins: [x, y, z], at, min: lower, max: upper} = queue[head];
            lastSeen = Math.max(lastSeen, at);
            for (let axis = 0; axis < 3; axis++) {
                min[axis] = Math.min(min[axis], lower[axis]);
                max[axis] = Math.max(max[axis], upper[axis]);
                for (const step of [-1, 1]) {
                    const next = voxelKey(x + (axis === 0 ? step : 0),
                                          y + (axis === 1 ? step : 0),
                                          z + (axis === 2 ? step : 0));
                    const neighbour = voxels.get(next);
                    const from = [(x + 0.5) * config.voxel, (y + 0.5) * config.voxel];
                    const to = [(x + (axis === 0 ? step : 0) + 0.5) * config.voxel,
                                (y + (axis === 1 ? step : 0) + 0.5) * config.voxel];
                    if (neighbour && (!config.canConnect || config.canConnect(from, to))) {
                        queue.push(neighbour);
                        voxels.delete(next);
                    }
                }
            }
        }
        const [width, depth, height] = max.map((value, axis) => value - min[axis]);
        if (queue.length < 5 || height < config.minSpanZ ||
                Math.max(width, depth) < config.voxel ||
                Math.max(width, depth) > config.maxSpanXY) continue;
        if (Math.max(width, depth) >= 0.75) {
            const columns = new Set(queue.map(({bins: [x, y]}) => `${x},${y}`));
            const footprint = (Math.floor(max[0]/config.voxel)-Math.floor(min[0]/config.voxel)+1) *
                              (Math.floor(max[1]/config.voxel)-Math.floor(min[1]/config.voxel)+1);
            if (columns.size < 0.20*footprint) continue;
        }
        candidates.push({x: (min[0] + max[0]) / 2, y: (min[1] + max[1]) / 2, z: min[2],
                         width: Math.max(config.voxel, width), depth: Math.max(config.voxel, depth),
                         height, yaw: 0, lastSeen, support: queue.length});
    }
    return candidates.sort((a, b) => b.support - a.support).slice(0, MAX_TRACKS);
}

function matchDistance(a, b, config) {
    const dx = Math.abs(a.x - b.x), dy = Math.abs(a.y - b.y);
    const limit = config.voxel + config.tolerance;
    if (Math.hypot(dx, dy) > limit || Math.abs(a.z - b.z) > limit ||
            dx > (a.width + b.width) / 2 + config.tolerance ||
            dy > (a.depth + b.depth) / 2 + config.tolerance ||
            Math.abs(a.width - b.width) > limit || Math.abs(a.depth - b.depth) > limit ||
            Math.abs(a.height - b.height) > limit) return Infinity;
    return Math.hypot(dx, dy, a.z - b.z);
}

export function createCloudTracker(options = {}) {
    const config = {...CLOUD_DEFAULTS, ...options};
    let frames = [];
    let tracks = [];
    let lastStamp = null;
    let lastRun = -Infinity;
    let dirty = false;
    let nextId = 0;

    function update(message, expectedFrame, now) {
        const {sec, nanosec} = message?.stamp || {};
        if (!expectedFrame || message?.frame_id !== expectedFrame || !Array.isArray(message.points) ||
                !Number.isSafeInteger(sec) || !Number.isInteger(nanosec) ||
                nanosec < 0 || nanosec >= 1e9) return false;
        if (lastStamp && (sec < lastStamp.sec ||
                (sec === lastStamp.sec && nanosec <= lastStamp.nanosec))) return false;
        lastStamp = {sec, nanosec};
        const points = message.points.slice(0, MAX_POINTS).filter(point =>
            Array.isArray(point) && point.length === 3 && point.every(Number.isFinite) &&
            point[2] >= config.minZ && point[2] <= config.maxZ &&
            (!config.acceptPoint || config.acceptPoint(point, message.sensor_origin)));
        frames.push({at: now, points});
        dirty = true;
        if (frames.length > MAX_FRAMES) frames.shift();
        return true;
    }

    function tick(now) {
        frames = frames.filter(frame => now - frame.at < config.windowMs);
        const remaining = tracks.filter(track => now - track.box.lastSeen < config.ttlMs);
        let changed = remaining.length !== tracks.length;
        tracks = remaining;
        if (dirty && frames.length && now - lastRun >= config.updateMs) {
            lastRun = now;
            dirty = false;
            const available = new Set(tracks);
            for (const box of cluster(frames, config, now)) {
                let best = null;
                let distance = Infinity;
                for (const track of available) {
                    const score = matchDistance(track.box, box, config);
                    if (score < distance) {
                        best = track;
                        distance = score;
                    }
                }
                if (!best) {
                    if (tracks.length < MAX_TRACKS) {
                        tracks.push({id: ++nextId, box, firstSeen: box.lastSeen, stable: false});
                    }
                    continue;
                }
                available.delete(best);
                if (box.lastSeen <= best.box.lastSeen) continue;
                best.box = box;
                best.stable ||= box.lastSeen - best.firstSeen >= config.updateMs;
                changed ||= best.stable;
            }
        }
        return changed ? tracks.filter(track => track.stable).map(({id, box}) => ({id, ...box})) : null;
    }

    function clear() {
        frames = [];
        tracks = [];
        lastStamp = null;
        lastRun = -Infinity;
        dirty = false;
    }

    return {update, tick, clear, config};
}
