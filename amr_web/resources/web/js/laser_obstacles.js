// Datums are in metres and monotonic client milliseconds. Each new scan
// replaces the last observed shape of a track instead of accumulating points.
export const LASER_MAX_STRIPS = 256;
export const LASER_UPDATE_MS = 100;
const MAX_BEAMS = 8192;
const MAX_TRACKS = 48;
const MAX_POINTS_PER_CLUSTER = 72;
const MATCH_M = 0.07;
const MIN_SPAN_M = 0.08;
const EXPIRED_MS = 1100;
const CLEAR_MARGIN_M = 0.12;

function validVector(values, length) {
    return Array.isArray(values) && values.length === length && values.every(Number.isFinite);
}

export function projectLaserScan(scan, frame) {
    const pose = scan?.scan_pose;
    const ranges = scan?.ranges;
    const {sec, nanosec} = scan?.stamp || {};
    const angleMin = scan?.angle_min_rad ?? scan?.angle_min;
    const step = scan?.angle_increment_rad ?? scan?.angle_increment;
    if (!frame || pose?.frame_id !== frame || !validVector(pose.position,3) ||
            !validVector(pose.rotation,4) || !(pose.height_m > 0) || !Number.isFinite(pose.height_m) ||
            !Number.isSafeInteger(sec) || sec < 0 || !Number.isInteger(nanosec) ||
            nanosec < 0 || nanosec >= 1e9 || !(sec || nanosec) ||
            !Number.isFinite(angleMin) || !Number.isFinite(step) || step === 0 ||
            !Array.isArray(ranges) || ranges.length < 3 || ranges.length > MAX_BEAMS ||
            !Number.isFinite(scan.range_min) || !Number.isFinite(scan.range_max) ||
            scan.range_min < 0 || scan.range_min >= scan.range_max) return null;

    const magnitude = Math.hypot(...pose.rotation);
    if (Math.abs(magnitude-1) > 0.01) return null;
    const [x,y,z,w] = pose.rotation.map(value => value/magnitude);
    // The two map-frame basis vectors of the LaserScan sensor plane.
    const u = [1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y)];
    const v = [2*(x*y-w*z), 1-2*(x*x+z*z), 2*(y*z+w*x)];
    const determinant = u[0]*v[1]-u[1]*v[0];
    if (Math.abs(determinant) < 0.2) return null;
    const [px,py,pz] = pose.position;
    const rangeAt = i => {
        const r = ranges[i];
        return Number.isFinite(r) && r >= scan.range_min && r <= scan.range_max ? r : null;
    };
    const runs = [];
    let points = [];
    let previous = null;
    for (let i = 0; i < ranges.length; i++) {
        const r = rangeAt(i);
        if (r === null) {
            if (points.length >= 3) runs.push(points);
            points = [];
            previous = null;
            continue;
        }
        // Split at range jumps as well as XY gaps. Sparse long range
        // returns can be farther apart, but never bridge gaps over 14 cm.
        const jump = Math.min(.14, .055 + Math.abs(step)*r*1.8);
        const angle = angleMin+i*step;
        const a = Math.cos(angle)*r, b = Math.sin(angle)*r;
        const point = [px+u[0]*a+v[0]*b, py+u[1]*a+v[1]*b, pz+u[2]*a+v[2]*b];
        if (previous && (Math.abs(previous.r-r) > jump ||
                Math.hypot(point[0]-previous.point[0],point[1]-previous.point[1]) > jump)) {
            if (points.length >= 3) runs.push(points);
            points = [];
        }
        points.push(point);
        previous = {point,r};
    }
    if (points.length >= 3) runs.push(points);

    const clusters = [];
    for (const run of runs) {
        const bounds = [Infinity, Infinity, -Infinity, -Infinity];
        for (const [x,y] of run) {
            bounds[0] = Math.min(bounds[0],x); bounds[1] = Math.min(bounds[1],y);
            bounds[2] = Math.max(bounds[2],x); bounds[3] = Math.max(bounds[3],y);
        }
        const span = Math.hypot(bounds[2]-bounds[0],bounds[3]-bounds[1]);
        if (span < MIN_SPAN_M) continue;
        // Evenly retain the measured extents for tracking. Rendering samples
        // the original trace via simplified segments, never a bounding box.
        const count = Math.min(run.length, MAX_POINTS_PER_CLUSTER);
        const samples = Array.from({length:count}, (_,i) => run[Math.round(i*(run.length-1)/(count-1))]);
        clusters.push({samples, lines:simplifyRun(run), bounds, height:pose.height_m, span});
    }

    function seesBeyond([x,y]) {
        const dx = x-px, dy = y-py;
        const a = (dx*v[1]-dy*v[0])/determinant;
        const b = (u[0]*dy-u[1]*dx)/determinant;
        const distance = Math.hypot(a,b);
        if (distance < scan.range_min || distance+CLEAR_MARGIN_M > scan.range_max) return false;
        const center = angleMin+step*(ranges.length-1)/2;
        const angle = Math.atan2(b,a);
        const wrapped = angle+Math.round((center-angle)/(2*Math.PI))*2*Math.PI;
        const ray = Math.round((wrapped-angleMin)/step);
        if (ray < 0 || ray >= ranges.length) return false;
        const observed = rangeAt(ray);
        return observed !== null && observed > distance+CLEAR_MARGIN_M;
    }

    return {clusters:clusters.sort((a,b) => b.span-a.span).slice(0,MAX_TRACKS), seesBeyond};
}

function distanceSquared(p,a,b) {
    const dx = b[0]-a[0], dy = b[1]-a[1];
    const t = Math.max(0,Math.min(1,((p[0]-a[0])*dx+(p[1]-a[1])*dy)/(dx*dx+dy*dy || 1)));
    return (p[0]-a[0]-dx*t)**2+(p[1]-a[1]-dy*t)**2;
}

function simplifyRun(points) {
    // Start a new piece periodically to bound the work even on adversarial
    // zigzags. Recursive subdivision remains local to each 48-beam window.
    const keep = [points[0]];
    function segment(start,end) {
        let split = -1, worst = .022**2;
        for (let i = start+1; i < end; i++) {
            const distance = distanceSquared(points[i],points[start],points[end]);
            if (distance > worst) {
                worst = distance;
                split = i;
            }
        }
        if (split >= 0) {
            segment(start,split);
            segment(split,end);
        } else {
            keep.push(points[end]);
        }
    }
    for (let first = 0; first < points.length-1; first += 48) {
        segment(first,Math.min(first+48,points.length-1));
    }
    return keep;
}

function affinity(a,b) {
    const [x0,y0,x1,y1] = a.bounds, [u0,v0,u1,v1] = b.bounds;
    if (x1+MATCH_M < u0 || u1+MATCH_M < x0 || y1+MATCH_M < v0 || v1+MATCH_M < y0) return 0;
    // Compare the shorter visible part with the other trace, allowing an
    // occluder or field of view to change the observed length.
    const [shorter,longer] = a.span <= b.span ? [a,b] : [b,a];
    let score = 0, count = 0;
    const stride = Math.max(1,Math.ceil(shorter.samples.length/16));
    const lineStride = Math.max(1,Math.ceil((longer.lines.length-1)/96));
    for (let i = 0; i < shorter.samples.length; i += stride) {
        let nearest = MATCH_M**2;
        for (let j = 1; j < longer.lines.length; j += lineStride) {
            nearest = Math.min(nearest,distanceSquared(shorter.samples[i],longer.lines[j-1],longer.lines[j]));
        }
        score += 1-Math.sqrt(nearest)/MATCH_M;
        count++;
    }
    return score/count;
}

export function createLaserTracker() {
    let tracks = [];
    let stamp = null;

    function expire(now) {
        const previous = tracks.length;
        tracks = tracks.filter(track => now-track.seenAt < (track.hits >= 2 ? EXPIRED_MS : 350));
        return tracks.length !== previous;
    }

    function update(scan, frame, now) {
        const next = scan?.stamp;
        if (!next || (stamp && (next.sec < stamp.sec ||
                next.sec === stamp.sec && next.nanosec <= stamp.nanosec))) return false;
        const projected = projectLaserScan(scan,frame);
        if (!projected) return false;
        stamp = next;
        expire(now);
        const matched = new Set();
        const fresh = [];
        for (const cluster of projected.clusters) {
            let best = null, score = .38;
            for (const track of tracks) {
                if (matched.has(track)) continue;
                const match = affinity(cluster,track.cluster);
                if (match <= score) continue;
                score = match;
                best = track;
            }
            if (best) matched.add(best);
            fresh.push({cluster, seenAt:now, hits:(best?.hits ?? 0)+1, cleared:0});
        }
        const unobserved = tracks.filter(track => {
            if (matched.has(track)) return false;
            const samples = track.cluster.samples;
            let clear = 0;
            for (const point of samples) if (projected.seesBeyond(point)) clear++;
            track.cleared = clear/samples.length >= .65 ? track.cleared+1 : 0;
            return track.cleared < 2;
        });
        // Give established obstacles priority when there is a burst of new
        // returns. The count and age bounds also apply outside the scan FOV.
        tracks = [...fresh.filter(t => t.hits >= 2), ...unobserved.filter(t => t.hits >= 2),
            ...fresh.filter(t => t.hits < 2), ...unobserved.filter(t => t.hits < 2)].slice(0,MAX_TRACKS);
        return true;
    }

    return {
        update, expire,
        confirmed: () => tracks.filter(track => track.hits >= 2).map(track => track.cluster),
        clear: () => { tracks = []; stamp = null; },
    };
}

export function clipLaserStrips(clusters, covered, step = .02) {
    const strips = [];
    function append(a,b,height) {
        if (Math.hypot(b[0]-a[0],b[1]-a[1]) >= .04) strips.push({a,b,height});
    }
    for (const {lines,height} of clusters) {
        for (let i = 1; i < lines.length; i++) {
            const a = lines[i-1], b = lines[i];
            const count = Math.max(1,Math.ceil(Math.hypot(b[0]-a[0],b[1]-a[1])/step));
            const at = t => a.map((value,axis) => value+(b[axis]-value)*t);
            let start = null;
            for (let j = 0; j < count; j++) {
                if (!covered(...at((j+.5)/count))) {
                    if (start === null) start = j;
                } else if (start !== null) {
                    append(at(start/count),at(j/count),height);
                    start = null;
                }
                if (strips.length >= LASER_MAX_STRIPS) return strips;
            }
            if (start !== null) append(at(start/count),b,height);
            if (strips.length >= LASER_MAX_STRIPS) return strips;
        }
    }
    return strips;
}

// Exact footprint/height matching for models, and a small registration
// allowance around an occupied raster cell. Clip only the covered part.
export function laserCoverage(info, wall, interior, boundary, models) {
    const {width,height,resolution,origin} = info;
    const c = Math.cos(origin.theta), s = Math.sin(origin.theta);
    const footprints = models.filter(model => model.suppressCloud !== false)
        .map(model => ({...model,c:Math.cos(model.yaw ?? 0),s:Math.sin(model.yaw ?? 0)}));
    return (x,y,z) => {
        if (!boundary.allowsPoint(x,y)) return true;
        const dx = x-origin.x, dy = y-origin.y;
        const gx = (dx*c+dy*s)/resolution, gy = (-dx*s+dy*c)/resolution;
        const pad = .025/resolution;
        for (let row = Math.max(0,Math.floor(gy-pad)); row <= Math.min(height-1,Math.floor(gy+pad)); row++) {
            for (let col = Math.max(0,Math.floor(gx-pad)); col <= Math.min(width-1,Math.floor(gx+pad)); col++) {
                const i = row*width+col;
                if (!wall[i] && !interior?.[i]) continue;
                const distance = Math.hypot(Math.max(0,col-gx,gx-col-1),Math.max(0,row-gy,gy-row-1));
                if (distance <= pad) return true;
            }
        }
        return footprints.some(model => {
            const zPad = model.exactCoverage ? 0 : .04;
            const xyPad = model.exactCoverage ? 0 : .02;
            if (z < model.z-zPad || z > model.z+model.height+zPad) return false;
            const dx = x-model.x, dy = y-model.y;
            return Math.abs(dx*model.c+dy*model.s) < model.width/2+xyPad &&
                Math.abs(-dx*model.s+dy*model.c) < model.depth/2+xyPad;
        });
    };
}
