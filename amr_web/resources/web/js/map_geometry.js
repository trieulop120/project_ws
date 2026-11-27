import {ShapeUtils, Vector2} from '../vendor/three/three.module.js';
import {floodGrid, gridBorder, pointInRing, wallContourVariants, segmentsMeet,
        selectWallBands} from './wall_contours.js';

// Pure ROS grid-to-world conversion. PGM rows start at the top; OccupancyGrid row 0 is at the bottom.
export function gridCellToSourceIndex(width, height, column, row, isPgm) {
    return (isPgm ? height - 1 - row : row) * width + column;
}

export function cellCenterWorld(info, column, row) {
    const localX = (column + 0.5) * info.resolution;
    const localY = (row + 0.5) * info.resolution;
    const yaw = info.origin.theta || 0;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    return {
        x: info.origin.x + c * localX - s * localY,
        y: info.origin.y + s * localX + c * localY,
    };
}

export function mapWorldCorners(info) {
    const yaw = info.origin.theta || 0;
    const c = Math.cos(yaw);
    const s = Math.sin(yaw);
    const w = info.width * info.resolution;
    const h = info.height * info.resolution;
    return [[0, 0], [w, 0], [w, h], [0, h]].map(([x, y]) => ({
        x: info.origin.x + c * x - s * y,
        y: info.origin.y + s * x + c * y,
    }));
}

export function normalizeMapInfo(map) {
    const raw = map?.info;
    const origin = raw?.origin;
    const width = Number(raw?.width);
    const height = Number(raw?.height);
    const resolution = Number(raw?.resolution);
    const x = Number(origin?.x);
    const y = Number(origin?.y);
    const theta = Number(origin?.theta ?? 0);
    const frameId = map?.ros_frame_id || raw?.frame_id || (map?.is_pgm_raw ? 'map' : map?.frame_id);
    if (!Number.isSafeInteger(width) || width < 1 || !Number.isSafeInteger(height) || height < 1 ||
            width * height > 100000000 || !Number.isFinite(resolution) || resolution <= 0 ||
            !Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(theta) ||
            typeof frameId !== 'string' || !frameId.trim() || typeof map?.data !== 'string') {
        throw new Error('Invalid map metadata or frame');
    }
    const mode = raw?.mode || 'trinary';
    const negate = Number(raw?.negate ?? 0);
    const occupiedThreshold = Number(raw?.occupied_thresh ?? 0.65);
    const freeThreshold = Number(raw?.free_thresh ?? 0.25);
    if (!['trinary', 'scale', 'raw'].includes(mode) || ![0, 1].includes(negate) ||
            !Number.isFinite(occupiedThreshold) || !Number.isFinite(freeThreshold) ||
            freeThreshold < 0 || occupiedThreshold > 1 || freeThreshold >= occupiedThreshold) {
        throw new Error('Invalid map occupancy metadata');
    }
    return {
        width, height, resolution, origin: {x, y, theta}, frame_id: frameId,
        mode, negate,
        occupied_thresh: occupiedThreshold,
        free_thresh: freeThreshold,
    };
}

function classify(value, isPgm, info) {
    if (!isPgm) return value === 1 ? 1 : value === 2 ? 2 : 0;
    // map_saver's trinary PGM writes unknown cells as gray 205.
    if (info.mode === 'trinary' && value === 205) return 0;
    if (info.mode === 'raw') {
        if (value === 255 || value > 100) return 0;
        if (value / 100 > info.occupied_thresh) return 2;
        if (value / 100 < info.free_thresh) return 1;
        return 0;
    }
    const occupiedProbability = info.negate ? value / 255 : (255 - value) / 255;
    if (occupiedProbability > info.occupied_thresh) return 2;
    if (occupiedProbability < info.free_thresh) return 1;
    return 0;
}

const COLORS = [
    [116, 126, 129], // unknown: flat, neutral gray
    [207, 216, 214], // free: flat, light floor
    [63, 78, 85],    // occupied interior footprint
];

export function mapPixels(categories, width, height, surveyExterior = null) {
    const pixels = new Uint8ClampedArray(width * height * 4);
    for (let row = 0; row < height; row++) {
        for (let column = 0; column < width; column++) {
            const index = row * width + column;
            const category = categories[index];
            const color = COLORS[category];
            const pixel = ((height - 1 - row) * width + column) * 4;
            pixels[pixel] = color[0];
            pixels[pixel + 1] = color[1];
            pixels[pixel + 2] = color[2];
            pixels[pixel + 3] = category === 0 ? (surveyExterior?.[index] ? 0 : 28) : 255;
        }
    }
    return pixels;
}

// Clean mode hides only small 8-connected occupied components within the supplied limits.
// Hidden cells become neutral on this display copy; the ROS map and 2D view are untouched.
export function cleanSmallOccupiedComponents(categories, width, height, maxCells = 2,
                                             maxSpanCells = Infinity) {
    const clean = categories.slice();
    const seen = new Uint8Array(categories.length);
    const queue = new Int32Array(categories.length);
    let removedCells = 0;
    let removedComponents = 0;
    for (let start = 0; start < categories.length; start++) {
        if (categories[start] !== 2 || seen[start]) continue;
        let head = 0;
        let tail = 1;
        queue[0] = start;
        seen[start] = 1;
        let minX = width;
        let maxX = 0;
        let minY = height;
        let maxY = 0;
        while (head < tail) {
            const index = queue[head++];
            const x = index % width;
            const y = Math.floor(index / width);
            minX = Math.min(minX, x);
            maxX = Math.max(maxX, x);
            minY = Math.min(minY, y);
            maxY = Math.max(maxY, y);
            for (let dy = -1; dy <= 1; dy++) {
                const nextY = y + dy;
                if (nextY < 0 || nextY >= height) continue;
                for (let dx = -1; dx <= 1; dx++) {
                    const nextX = x + dx;
                    if (nextX < 0 || nextX >= width) continue;
                    const next = nextY * width + nextX;
                    if (categories[next] === 2 && !seen[next]) {
                        seen[next] = 1;
                        queue[tail++] = next;
                    }
                }
            }
        }
        if (tail <= maxCells && maxX - minX + 1 <= maxSpanCells &&
                maxY - minY + 1 <= maxSpanCells) {
            for (let i = 0; i < tail; i++) clean[queue[i]] = 0;
            removedCells += tail;
            removedComponents++;
        }
    }
    return {categories: clean, removedCells, removedComponents};
}

// Test the rotated footprint against occupied cells in its grid-local bounds.
export function occupiedFootprintCells(categories, info, object, tolerance = 0) {
    const {width, height, resolution, origin} = info;
    const c = Math.cos(origin.theta), s = Math.sin(origin.theta);
    const dx = object.x - origin.x, dy = object.y - origin.y;
    const x = c * dx + s * dy, y = -s * dx + c * dy;
    const angle = (object.yaw || 0) - origin.theta;
    const u = Math.cos(angle), v = Math.sin(angle);
    const halfWidth = object.width / 2 + tolerance;
    const halfDepth = object.depth / 2 + tolerance;
    const boundX = Math.abs(u) * halfWidth + Math.abs(v) * halfDepth;
    const boundY = Math.abs(v) * halfWidth + Math.abs(u) * halfDepth;
    const halfCell = resolution / 2;
    const cellRadius = halfCell * (Math.abs(u) + Math.abs(v));
    const cells = [];
    const edgeEpsilon = 1e-9;
    const intersectionEpsilon = -1e-9;
    for (let row = Math.max(0, Math.floor((y - boundY - edgeEpsilon) / resolution));
            row <= Math.min(height - 1, Math.floor((y + boundY + edgeEpsilon) / resolution)); row++) {
        for (let col = Math.max(0, Math.floor((x - boundX - edgeEpsilon) / resolution));
                col <= Math.min(width - 1, Math.floor((x + boundX + edgeEpsilon) / resolution)); col++) {
            const index = row * width + col;
            if (categories[index] !== 2) continue;
            const px = (col + 0.5) * resolution - x, py = (row + 0.5) * resolution - y;
            if (Math.abs(px) <= boundX + halfCell + intersectionEpsilon &&
                    Math.abs(py) <= boundY + halfCell + intersectionEpsilon &&
                    Math.abs(px * u + py * v) <= halfWidth + cellRadius + intersectionEpsilon &&
                    Math.abs(-px * v + py * u) <= halfDepth + cellRadius + intersectionEpsilon) cells.push(index);
        }
    }
    return cells;
}

export const WALL_HEIGHT = 2.0;

export function extractPerimeter(categories, info) {
    const {width, height} = info;
    const queue = new Int32Array(categories.length);
    const perimeter = selectWallBands(categories, info, queue);
    const {wall} = perimeter;
    if (!categories.includes(1)) {
        return {...perimeter, wallEvidence:wall, outside:new Uint8Array(categories.length)};
    }
    // Preserve the distinction between measured exterior and unknown that
    // might still be reachable. Only measured wall cells occlude cloud rays.
    const known = categories.map(value => Number(value !== 0));
    const exterior = floodGrid(known,width,height,gridBorder(width,height),queue);
    function* freeCells() {
        for (let i = 0; i < categories.length; i++) if (categories[i] === 1) yield i;
    }
    const reachable = floodGrid(wall,width,height,freeCells(),queue);
    const outside = exterior.map((value,i) => Number(value && !reachable[i]));
    function markOutward(start, step, length) {
        let outward = false, inBand = false, previousFree = false;
        for (let n = 0, i = start; n < length; n++, i += step) {
            if (wall[i]) {
                // A band must have scanned free evidence on its inner side.
                if (!inBand) outward = previousFree;
                inBand = true;
                continue;
            }
            inBand = false;
            previousFree = categories[i] === 1;
            if (exterior[i] && outward) outside[i] = 1;
            else outward = false;
        }
    }
    for (let row = 0; row < height; row++) {
        markOutward(row*width,1,width);
        markOutward(row*width+width-1,-1,width);
    }
    for (let col = 0; col < width; col++) {
        markOutward(col,width,height);
        markOutward(col+(height-1)*width,-width,height);
    }
    return {...perimeter, wallEvidence:wall, outside, exteriorUnknown:exterior};
}

// Merge equal horizontal runs on consecutive ROS grid rows into rectangles.
// Each rectangle is [column, bottom-up row, width, height] in grid cells.
export function mergeOccupiedRectangles(categories, width, height) {
    const rectangles = [];
    let active = new Map();
    for (let row = 0; row < height; row++) {
        const next = new Map();
        for (let column = 0; column < width;) {
            if (categories[row * width + column] !== 2) {
                column++;
                continue;
            }
            const start = column;
            while (column < width && categories[row * width + column] === 2) column++;
            const key = `${start}:${column}`;
            const rectangle = active.get(key);
            if (rectangle) {
                rectangle[3]++;
                active.delete(key);
                next.set(key, rectangle);
            } else {
                next.set(key, [start, row, column - start, 1]);
            }
        }
        for (const rectangle of active.values()) rectangles.push(rectangle);
        active = next;
    }
    for (const rectangle of active.values()) rectangles.push(rectangle);
    return rectangles;
}

const CLIP_EPSILON = 1e-7;

function rackCutouts(racks, {origin}) {
    const c = Math.cos(origin.theta), s = Math.sin(origin.theta);
    return racks.map(rack => {
        const dx = rack.x-origin.x, dy = rack.y-origin.y;
        const x = c*dx+s*dy, y = -s*dx+c*dy;
        const angle = rack.yaw-origin.theta;
        const u = Math.cos(angle), v = Math.sin(angle);
        const hx = rack.width/2, hy = rack.depth/2;
        const corners = [[-hx,-hy], [hx,-hy], [hx,hy], [-hx,hy]]
            .map(([a,b]) => [x+u*a-v*b, y+v*a+u*b]);
        const planes = corners.map(([ax, ay], i) => {
            const [bx, by] = corners[(i+1)%4];
            const length = Math.hypot(bx-ax, by-ay);
            const nx = (by-ay)/length, ny = (ax-bx)/length;
            return [nx, ny, nx*ax+ny*ay];
        });
        return {corners, planes, bounds: xyBounds(corners)};
    });
}

function xyBounds(points) {
    const xs = points.map(p => p[0]), ys = points.map(p => p[1]);
    return [Math.min(...xs), Math.min(...ys), Math.max(...xs), Math.max(...ys)];
}

function meetsBounds(a, b) {
    return a[0] <= b[2]+CLIP_EPSILON && a[2] >= b[0]-CLIP_EPSILON &&
           a[1] <= b[3]+CLIP_EPSILON && a[3] >= b[1]-CLIP_EPSILON;
}

function planeDistance(point, [nx, ny, limit]) {
    return nx*point[0] + ny*point[1] - limit;
}

function interpolate(a, b, t) {
    return a.map((value, axis) => value + t*(b[axis]-value));
}

function subtractFootprint(points, {planes}) {
    let inside = points;
    const outside = [];
    for (const plane of planes) {
        if (!inside.length) break;
        const distances = inside.map(point => {
            const distance = planeDistance(point, plane);
            return Math.abs(distance) <= CLIP_EPSILON ? 0 : distance;
        });
        if (distances.every(value => value <= 0)) continue;
        if (distances.every(value => value >= 0)) {
            outside.push(inside);
            break;
        }
        const kept = [], remainder = [];
        for (let i = 0; i < inside.length; i++) {
            const j = (i+1)%inside.length;
            const d = distances[i], next = distances[j];
            if (d >= 0) kept.push(inside[i]);
            if (d <= 0) remainder.push(inside[i]);
            if (d*next < 0) {
                const cut = interpolate(inside[i], inside[j], d/(d-next));
                kept.push(cut);
                remainder.push(cut);
            }
        }
        outside.push(kept);
        inside = remainder;
    }
    return outside;
}

// Parameter interval for a segment inside a convex footprint.
function segmentInterval(a, b, planes) {
    let first = 0, last = 1;
    for (const plane of planes) {
        const da = planeDistance(a, plane), db = planeDistance(b, plane);
        if (da > 0 && db > 0) return null;
        if (da <= 0 && db <= 0) continue;
        const t = da/(da-db);
        if (da > 0) first = Math.max(first, t);
        else last = Math.min(last, t);
        if (last-first <= CLIP_EPSILON) return null;
    }
    return [first, last];
}

// Build one watertight-looking surface from occupied cells. Internal faces between
// adjacent rectangles are omitted, so a tall wall does not show box seams.
export function buildWallSurfaceData(categories, info, racks = [], heightMeters = null,
                                     contours = null) {
    const {width, height, resolution} = info;
    const rectangles = contours ? [] : mergeOccupiedRectangles(categories, width, height);
    const cutouts = rackCutouts(racks, info);
    const positions = [];
    const normals = [];
    const indices = [];
    let faces = 0;
    const z0 = 0.002;
    const z1 = heightMeters ?? z0 + WALL_HEIGHT;
    function polygon(points, normal) {
        if (points.length < 3) return;
        const base = positions.length / 3;
        for (const point of points) {
            positions.push(...point);
            normals.push(...normal);
        }
        for (let i = 1; i < points.length-1; i++) indices.push(base, base+i, base+i+1);
        faces++;
    }
    function clippedFace(points, normal) {
        const bounds = xyBounds(points);
        let pieces = [points];
        for (const cutout of cutouts) {
            if (meetsBounds(bounds, cutout.bounds)) {
                pieces = pieces.flatMap(piece => subtractFootprint(piece, cutout));
            }
        }
        for (const piece of pieces) polygon(piece, normal);
    }

    const quad = (a,b,c,d,normal) => clippedFace([a,b,c,d],normal);

    // Find the parts of rack edges inside the wall footprint. Merge intervals
    // across adjacent top triangles before emitting the vertical cut faces.
    const capIntervals = cutouts.map(() => [[],[],[],[]]);
    function capEdges(bounds, limits) {
        const [x0,y0,x1,y1] = bounds;
        for (const cutout of cutouts) {
            if (!meetsBounds([x0,y0,x1,y1], cutout.bounds)) continue;
            for (const [i, a] of cutout.corners.entries()) {
                const b = cutout.corners[(i+1)%4];
                const [nx, ny] = cutout.planes[i];
                // Sample on the retained side to assign a coincident edge to
                // exactly one wall rectangle, and to respect overlapping racks.
                const sample = p => [p[0] + nx*CLIP_EPSILON, p[1] + ny*CLIP_EPSILON];
                const pa = sample(a), pb = sample(b);
                const interval = segmentInterval(pa, pb, limits);
                if (!interval) continue;
                let intervals = [interval];
                for (const other of cutouts) {
                    if (other === cutout || !meetsBounds(cutout.bounds, other.bounds)) continue;
                    const overlap = segmentInterval(pa, pb, other.planes);
                    if (overlap) {
                        intervals = intervals.flatMap(([start,end]) =>
                            [[start, Math.min(end,overlap[0])], [Math.max(start,overlap[1]), end]]
                                .filter(([lo,hi]) => hi-lo > CLIP_EPSILON));
                    }
                }
                for (const [start, end] of intervals) {
                    if (contours) {
                        capIntervals[cutouts.indexOf(cutout)][i].push([start,end]);
                    } else {
                        const p = interpolate(a,b,start), q = interpolate(a,b,end);
                        polygon([[...p,z0], [...p,z1], [...q,z1], [...q,z0]], [-nx,-ny,0]);
                    }
                }
            }
        }
    }
    for (const [x, y, w, h] of rectangles) {
        const x0 = x * resolution;
        const x1 = (x + w) * resolution;
        const y0 = y * resolution;
        const y1 = (y + h) * resolution;
        quad([x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1], [0, 0, 1]);
        capEdges([x0,y0,x1,y1], [[-1,0,-x0],[1,0,x1],[0,-1,-y0],[0,1,y1]]);
    }
    if (contours) {
        for (const {outer, holes} of contours) {
            const vectors = ring => ring.map(([x,y]) => new Vector2(x,y));
            const vertices = [outer,...holes].flat();
            const triangles = ShapeUtils.triangulateShape(vectors(outer),holes.map(vectors));
            for (const triangle of triangles) {
                const points = triangle.map(index => vertices[index]);
                clippedFace(points.map(p => [...p,z1]),[0,0,1]);
                if (!cutouts.length) continue;
                const limits = points.map(([x,y],i) => {
                    const [nx,ny] = points[(i+1)%3];
                    const dx = nx-x, dy = ny-y;
                    const length = Math.hypot(dx,dy);
                    return [dy/length,-dx/length,(dy*x-dx*y)/length];
                });
                capEdges(xyBounds(points),limits);
            }
            for (const ring of [outer,...holes]) {
                for (let i = 0; i < ring.length; i++) {
                    const a = ring[i], b = ring[(i+1)%ring.length];
                    const dx = b[0]-a[0], dy = b[1]-a[1];
                    const length = Math.hypot(dx,dy);
                    quad([...a,z0],[...b,z0],[...b,z1],[...a,z1],[dy/length,-dx/length,0]);
                }
            }
        }
        capIntervals.forEach((sides,j) => sides.forEach((intervals,i) => {
            const {corners,planes} = cutouts[j];
            const a = corners[i], b = corners[(i+1)%4];
            const [nx,ny] = planes[i];
            let end = -Infinity;
            let start;
            function emit() {
                if (start === undefined) return;
                const p = interpolate(a,b,start), q = interpolate(a,b,end);
                polygon([[...p,z0],[...p,z1],[...q,z1],[...q,z0]],[-nx,-ny,0]);
            }
            for (const [low,high] of intervals.sort((a,b) => a[0]-b[0])) {
                if (low > end+CLIP_EPSILON) {
                    emit();
                    start = low;
                }
                end = Math.max(end,high);
            }
            emit();
        }));
    } else {
        const occupied = (x, y) => x >= 0 && x < width && y >= 0 && y < height &&
            categories[y * width + x] === 2;
        for (let y = 0; y < height; y++) {
            for (const north of [true, false]) {
                for (let x = 0; x < width;) {
                    const exposed = column => occupied(column, y) &&
                        !occupied(column, y + (north ? 1 : -1));
                    if (!exposed(x)) { x++; continue; }
                    const start = x;
                    while (x < width && exposed(x)) x++;
                    const x0 = start * resolution;
                    const x1 = x * resolution;
                    const edge = (y + (north ? 1 : 0)) * resolution;
                    if (north) {
                        quad([x0, edge, z0], [x0, edge, z1], [x1, edge, z1],
                             [x1, edge, z0], [0, 1, 0]);
                    } else {
                        quad([x0, edge, z0], [x1, edge, z0], [x1, edge, z1],
                             [x0, edge, z1], [0, -1, 0]);
                    }
                }
            }
        }
        for (let x = 0; x < width; x++) {
            for (const east of [true, false]) {
                for (let y = 0; y < height;) {
                    const exposed = row => occupied(x, row) &&
                        !occupied(x + (east ? 1 : -1), row);
                    if (!exposed(y)) { y++; continue; }
                    const start = y;
                    while (y < height && exposed(y)) y++;
                    const y0 = start * resolution;
                    const y1 = y * resolution;
                    const edge = (x + (east ? 1 : 0)) * resolution;
                    if (east) {
                        quad([edge, y0, z0], [edge, y1, z0], [edge, y1, z1],
                             [edge, y0, z1], [1, 0, 0]);
                    } else {
                        quad([edge, y0, z0], [edge, y0, z1], [edge, y1, z1],
                             [edge, y1, z0], [-1, 0, 0]);
                    }
                }
            }
        }
    }
    return {
        positions: new Float32Array(positions),
        normals: new Float32Array(normals),
        indices: positions.length / 3 <= 65535 ? new Uint16Array(indices) : new Uint32Array(indices),
        faces,
    };
}

export function rackWallClipping(wall, info, layout, contours = null) {
    const rackWallConflicts = [];
    const cuts = [];
    for (const rack of layout?.objects ?? []) {
        if (rack.type !== 'rack') continue;
        const cells = occupiedFootprintCells(wall, info, rack).length;
        const cutout = contours && rackCutouts([rack],info)[0];
        const meetsContour = cutout && contours.some(({outer, holes}) => {
            if (!meetsBounds(xyBounds(outer),cutout.bounds)) return false;
            if (cutout.corners.some(p => pointInRing(p,outer) && !holes.some(h => pointInRing(p,h)))) {
                return true;
            }
            return [outer,...holes].some(ring => ring.some((p,i) =>
                pointInRing(p,cutout.corners) || cutout.corners.some((a,j) =>
                    segmentsMeet(p,ring[(i+1)%ring.length],a,cutout.corners[(j+1)%4]))));
        });
        if (cells || meetsContour) {
            rackWallConflicts.push({id: rack.id, cells});
            cuts.push(rack);
        }
    }
    return {rackWallConflicts, cuts};
}

// Trace free-cell fringe from the measured survey exterior. Labels and
// display edits stay local to the Clean variant.
function trimFloorFringe(categories, width, survey) {
    if (!survey) return 0;
    const {core, exterior:surveyExterior} = survey;
    if (!core.includes(1) || !categories.some((v,i) => v === 1 && surveyExterior[i])) return 0;
    const labels = new Int32Array(categories.length);
    const queue = new Int32Array(categories.length);
    let region = 0;
    let hidden = 0;
    function neighbours(i, visit) {
        if (i % width) visit(i-1);
        if (i % width < width-1) visit(i+1);
        if (i >= width) visit(i-width);
        if (i+width < categories.length) visit(i+width);
    }

    // Positive labels identify supported free regions inside the envelope.
    // Exterior seeds can then trace a ray all the way to a measured wall,
    // including its short unsupported root just inside the envelope.
    // Group contacts at the survey opening scale: a jagged mouth along a
    // room edge can touch many non-adjacent pixels. Two separated mouths
    // support a passage even when both lead into the same room.
    const mouthSize = Math.max(1, Math.ceil(2*survey.openingRadiusCells));
    const mouthStride = Math.ceil(width/mouthSize) + 2;
    function mouthBin(i) {
        return Math.floor(i%width/mouthSize) + mouthStride*Math.floor(Math.floor(i/width)/mouthSize);
    }
    function separateEntrances(bins) {
        if (bins.size < 2) return false;
        const pending = [bins.values().next().value];
        bins.delete(pending[0]);
        while (pending.length) {
            const bin = pending.pop();
            for (let dy = -1; dy <= 1; dy++) {
                for (let dx = -1; dx <= 1; dx++) {
                    const next = bin + dy*mouthStride + dx;
                    if (bins.delete(next)) pending.push(next);
                }
            }
        }
        return bins.size > 0;
    }
    function visitComponent(seed, label) {
        let end = 1;
        const contacts = label < 0 ? new Set() : null;
        let firstAnchor = 0;
        let passage = false;
        queue[0] = seed;
        labels[seed] = label;
        function visit(i) {
            if (categories[i] !== 1 || (label > 0 && surveyExterior[i])) return;
            if (!labels[i]) {
                labels[i] = label;
                queue[end++] = i;
            } else if (label < 0 && labels[i] > 0) {
                passage ||= firstAnchor !== 0 && firstAnchor !== labels[i];
                firstAnchor = labels[i];
                if (!passage) contacts.add(mouthBin(i));
            }
        }
        for (let n = 0; n < end; n++) neighbours(queue[n], visit);
        if (label < 0 && !passage && !separateEntrances(contacts)) {
            for (let n = 0; n < end; n++) categories[queue[n]] = 0;
            hidden += end;
        }
    }
    for (let i = 0; i < categories.length; i++) {
        if (core[i] && !surveyExterior[i] && categories[i] === 1 && !labels[i]) visitComponent(i, ++region);
    }
    if (!region) return 0;
    for (let i = 0; i < categories.length; i++) {
        if (surveyExterior[i] && categories[i] === 1 && !labels[i]) visitComponent(i, -1);
    }
    return hidden;
}

export function createDisplayVariants(decoded, layout = null) {
    const {width, height, resolution} = decoded.info;
    const maxCells = Math.min(8, Math.floor(0.0032 / (resolution * resolution) + 1e-9));
    const maxSpanCells = Math.floor(0.10 / resolution + 1e-9);
    const filtered = cleanSmallOccupiedComponents(decoded.categories, width, height,
                                                  maxCells, maxSpanCells);
    const {survey, exteriorUnknown, ...perimeter} = extractPerimeter(decoded.categories, decoded.info);
    const hiddenFreeCells = trimFloorFringe(filtered.categories, width, survey);
    // The measured unknown flood keeps enclosed holes distinguishable even
    // where the survey opening trims the narrow floor around them.
    const transparent = survey?.core.includes(1) ? exteriorUnknown : null;
    if (transparent && hiddenFreeCells) {
        for (let i = 0; i < transparent.length; i++) {
            if (decoded.categories[i] === 1 && filtered.categories[i] === 0) transparent[i] = 1;
        }
    }
    const {raw:contours,clean:refinedContours} = wallContourVariants(perimeter.wall, decoded.info);
    const clipping = rackWallClipping(perimeter.wall, decoded.info, layout, contours);
    const interior = categories => categories.map((value, index) =>
        value === 2 && !perimeter.wall[index] ? 2 : 0);
    return {
        ...perimeter, ...clipping, contours, refinedContours,
        raw: {interior: interior(decoded.categories), pixels: decoded.pixels, occupiedCells: decoded.occupied.length / 2},
        clean: {
            interior: interior(filtered.categories),
            pixels: mapPixels(filtered.categories, width, height, transparent),
            occupiedCells: decoded.occupied.length / 2 - filtered.removedCells,
            hiddenFreeCells,
            removedCells: filtered.removedCells,
            removedComponents: filtered.removedComponents,
            maxFilteredCells: maxCells,
        },
    };
}

export function decodeMap(map, info = normalizeMapInfo(map)) {
    const width = info.width;
    const height = info.height;
    const isPgm = map.is_pgm_raw === true;
    const binary = atob(map.data);
    if (binary.length !== width * height) throw new Error('Map data length does not match width × height');

    const categories = new Uint8Array(width * height); // ROS grid rows are bottom-up.
    const occupied = []; // Flat [column, bottom-up row] pairs for one InstancedMesh.
    for (let gridRow = 0; gridRow < height; gridRow++) {
        for (let column = 0; column < width; column++) {
            const source = gridCellToSourceIndex(width, height, column, gridRow, isPgm);
            const category = classify(binary.charCodeAt(source), isPgm, info);
            categories[gridRow * width + column] = category;
            if (category === 2) occupied.push(column, gridRow);
        }
    }
    return {info, categories, pixels: mapPixels(categories, width, height), occupied};
}

export const SEMANTIC_TOLERANCE = 0.04;
const MIN_VISIBLE_FRAGMENT = 0.08;
const MAX_FRAGMENTS = 12;

function boxBounds(box) {
    return [box.x-box.width/2, box.y-box.depth/2, box.z,
            box.x+box.width/2, box.y+box.depth/2, box.z+box.height];
}

// XY area inside the oriented semantic footprint, measured in its local axes.
function overlapArea(box, object, tolerance) {
    const [x0, y0, , x1, y1] = boxBounds(box);
    const c = Math.cos(object.yaw), s = Math.sin(object.yaw);
    const limits = [object.width/2 + tolerance, object.depth/2 + tolerance];
    let polygon = [[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(([x, y]) => {
        const dx = x-object.x, dy = y-object.y;
        return [dx*c + dy*s, -dx*s + dy*c];
    });
    for (const axis of [0, 1]) {
        for (const sign of [-1, 1]) {
            const next = [];
            for (let i = 0; i < polygon.length; i++) {
                const a = polygon[i], b = polygon[(i + 1) % polygon.length];
                const da = limits[axis] - sign*a[axis], db = limits[axis] - sign*b[axis];
                if (da >= 0) next.push(a);
                if ((da >= 0) !== (db >= 0)) {
                    const t = da / (da - db);
                    next.push([a[0] + t*(b[0]-a[0]), a[1] + t*(b[1]-a[1])]);
                }
            }
            polygon = next;
        }
    }
    let twiceArea = 0;
    for (let i = 0; i < polygon.length; i++) {
        const [x, y] = polygon[i], [nx, ny] = polygon[(i+1) % polygon.length];
        twiceArea += x*ny - nx*y;
    }
    return Math.abs(twiceArea)/2;
}

function overlapRatio(box, object, tolerance) {
    const bottom = Math.max(box.z, object.z-tolerance);
    const top = Math.min(box.z+box.height, object.z+object.height+tolerance);
    if (top <= bottom) return 0;
    return overlapArea(box, object, tolerance) * (top-bottom) / (box.width*box.depth*box.height);
}

function coveredLoadRatio(box, parts, tolerance) {
    const slabs = [];
    for (const part of parts) {
        const bottom = Math.max(box.z, part.z-tolerance);
        const top = Math.min(box.z+box.height, part.z+part.height+tolerance);
        if (top <= bottom) continue;
        const area = overlapArea(box, part, tolerance);
        if (area > 0) slabs.push({bottom, top, area});
    }

    // Use a conservative union per Z interval. Adjacent pallet/carton layers
    // contribute together, while overlapping cartons cannot double-count air.
    const levels = [...new Set(slabs.flatMap(({bottom, top}) => [bottom, top]))].sort((a,b) => a-b);
    let volume = 0;
    for (let i = 1; i < levels.length; i++) {
        const bottom = levels[i-1], top = levels[i];
        let area = 0;
        for (const slab of slabs) {
            if (slab.bottom <= bottom && slab.top >= top) area = Math.max(area, slab.area);
        }
        volume += area * (top-bottom);
    }
    return volume / (box.width*box.depth*box.height);
}

function matchingOverlap(ratio, near) {
    return ratio >= 0.70 || (ratio >= 0.50 && near >= 0.90);
}

function subtractSemantic(box, object, ratio) {
    const c = Math.abs(Math.cos(object.yaw)), s = Math.abs(Math.sin(object.yaw));
    const footprint = {...object, width: c*object.width + s*object.depth,
                       depth: s*object.width + c*object.depth};
    const bounds = boxBounds(box);
    const occupied = boxBounds(footprint);
    const intersection = bounds.map((value, axis) => axis < 3 ?
        Math.max(value, occupied[axis]) : Math.min(value, occupied[axis]));
    const cut = intersection.slice(0, 3).map((value, axis) => intersection[axis+3]-value);
    if (cut.some(span => span <= 0)) return [box];
    // The envelope is exact at right angles. At other angles, only use it
    // when the extra volume removed is small relative to the original box.
    const cutRatio = (cut[0]*cut[1]*cut[2]) / (box.width*box.depth*box.height);
    if (cutRatio - ratio > 0.12) return [box];
    const pieces = [];
    // Remove the exact intersection by peeling non-overlapping slabs along each axis.
    for (const axis of [0, 1, 2]) {
        for (const side of [0, 1]) {
            const slab = bounds.slice();
            const edge = axis + (side ? 0 : 3);
            slab[edge] = intersection[axis + (side ? 3 : 0)];
            const [width, depth, height] = slab.slice(3).map((value, index) => value-slab[index]);
            if (Math.min(width, depth, height) >= MIN_VISIBLE_FRAGMENT) {
                pieces.push({...box, id: `${box.id}:${axis}${side}`, width, depth, height,
                    x: (slab[0]+slab[3])/2, y: (slab[1]+slab[4])/2, z: slab[2]});
            }
        }
        bounds[axis] = intersection[axis];
        bounds[axis+3] = intersection[axis+3];
    }
    return pieces;
}

function suppressSemantic(boxes, semantic, tolerance) {
    function nearby(piece, object) {
        const distance = Math.hypot(piece.x-object.x, piece.y-object.y);
        return distance <= (Math.hypot(piece.width, piece.depth) +
            Math.hypot(object.width, object.depth))/2 + tolerance;
    }

    function clip(pieces, object) {
        const next = pieces.flatMap(piece => {
            if (!nearby(piece, object)) return [piece];
            const ratio = overlapRatio(piece, object, 0);
            const near = ratio >= 0.50 && !object.exactCoverage ? overlapRatio(piece, object, tolerance) : 0;
            if (matchingOverlap(ratio, near)) return [];
            return ratio >= 0.08 ? subtractSemantic(piece, object, ratio) : [piece];
        });
        return next.length <= MAX_FRAGMENTS ? next : pieces;
    }

    return boxes.flatMap(box => {
        let pieces = [box];
        for (const object of semantic) {
            if (object.suppressCloud === false || !pieces.length) continue;
            if (!object.parts) {
                pieces = clip(pieces, object);
                continue;
            }
            pieces = pieces.filter(piece => {
                if (!nearby(piece, object) || overlapRatio(piece, object, 0) < 0.50) return true;
                const ratio = coveredLoadRatio(piece, object.parts, 0);
                const near = ratio >= 0.50 && !object.exactCoverage ? coveredLoadRatio(piece, object.parts, tolerance) : 0;
                return !matchingOverlap(ratio, near);
            });
            for (const part of object.parts) pieces = clip(pieces, part);
        }
        return pieces;
    });
}

export function resolveObstacles(boxes, perimeter, info, semantic, tolerance,
                                 semanticTolerance = SEMANTIC_TOLERANCE) {
    return suppressSemantic(boxes, semantic, semanticTolerance).filter(box =>
        occupiedFootprintCells(perimeter, info, box, tolerance).length === 0);
}

// Keep cloud observations on the warehouse side of mapped perimeter walls.
// Unknown remains usable unless the perimeter topology provides exterior
// evidence; sensor rays also identify points observed behind a wall on open maps.
export function createCloudBoundary(info, outside, wall = null) {
    const {width, height, resolution, origin} = info;
    wall ||= new Uint8Array(width*height);
    const c = Math.cos(origin.theta), s = Math.sin(origin.theta);
    function mapCell(x, y) {
        const dx = x-origin.x, dy = y-origin.y;
        const col = Math.floor((c*dx+s*dy)/resolution);
        const row = Math.floor((-s*dx+c*dy)/resolution);
        if (col < 0 || col >= width || row < 0 || row >= height) return -1;
        return row*width+col;
    }
    function allowsPoint(x, y) {
        const cell = mapCell(x, y);
        return cell >= 0 && !outside[cell] && !wall[cell];
    }
    function crossesWall(a, b) {
        const steps = Math.max(1, Math.ceil(Math.hypot(b[0]-a[0], b[1]-a[1]) /
                                             Math.max(resolution*0.35, 0.005)));
        for (let step = 0; step <= steps; step++) {
            const t = step/steps;
            const cell = mapCell(a[0]+(b[0]-a[0])*t, a[1]+(b[1]-a[1])*t);
            if (cell >= 0 && wall[cell]) return true;
        }
        return false;
    }
    function allowsObservation(point, sensorOrigin) {
        if (!allowsPoint(point[0], point[1])) return false;
        return !sensorOrigin || !crossesWall(sensorOrigin, point);
    }
    function allowsBox(box) {
        // Test the full envelope at a spacing tied to map resolution. Requiring
        // a clear majority prevents a large outside box from passing because a
        // few samples happen to land in the raster; support points are filtered
        // individually before clustering, so a genuinely split cluster keeps
        // only its observed interior side.
        if (occupiedFootprintCells(wall, info, box).length) return false;
        const nx = Math.min(65, Math.max(9, Math.ceil(box.width/(resolution*0.5))));
        const ny = Math.min(65, Math.max(9, Math.ceil(box.depth/(resolution*0.5))));
        let valid = 0;
        for (let row = 0; row < ny; row++) {
            for (let col = 0; col < nx; col++) {
                const x = box.x + ((col+0.5)/nx-0.5)*box.width;
                const y = box.y + ((row+0.5)/ny-0.5)*box.depth;
                if (allowsPoint(x,y)) valid++;
            }
        }
        return valid/(nx*ny) >= 0.5;
    }
    function canConnect(a, b) {
        return !crossesWall(a, b);
    }
    return {allowsPoint, allowsObservation, allowsBox, canConnect};
}
