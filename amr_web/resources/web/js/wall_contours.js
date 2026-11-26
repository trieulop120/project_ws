// The envelope is a display-only estimate of the surveyed footprint. Occupied
// evidence is always read back from the original raster.
const RAY_FRINGE_M = 0.12;
const ENVELOPE_NOTCH_M = 0.10;
const WALL_SEARCH_M = 0.20;
const MIN_WALL_SPAN_M = 0.24;
const MIN_WALL_LENGTH_M = 0.10;
const MAX_EDGE_REPAIR_M = 0.10;
const MAX_PINHOLE_M = 0.07;

function distanceTo(mask, width, height, target) {
    // Chamfer distance in thirds of a cell. The diagonal weight approximates
    // Euclidean distance without sweeping a large kernel at fine resolutions.
    const distance = new Uint32Array(mask.length);
    distance.fill(0x3fffffff);
    for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
            const i = row*width + col;
            if (mask[i] === target) {
                distance[i] = 0;
                continue;
            }
            if (col) distance[i] = Math.min(distance[i], distance[i-1]+3);
            if (!row) continue;
            distance[i] = Math.min(distance[i], distance[i-width]+3);
            if (col) distance[i] = Math.min(distance[i], distance[i-width-1]+4);
            if (col+1 < width) distance[i] = Math.min(distance[i], distance[i-width+1]+4);
        }
    }
    for (let row = height-1; row >= 0; row--) {
        for (let col = width-1; col >= 0; col--) {
            const i = row*width + col;
            if (col+1 < width) distance[i] = Math.min(distance[i], distance[i+1]+3);
            if (row+1 >= height) continue;
            distance[i] = Math.min(distance[i], distance[i+width]+3);
            if (col) distance[i] = Math.min(distance[i], distance[i+width-1]+4);
            if (col+1 < width) distance[i] = Math.min(distance[i], distance[i+width+1]+4);
        }
    }
    return distance;
}

export function floodGrid(mask, width, height, seeds, queue) {
    const reached = new Uint8Array(mask.length);
    let end = 0;
    function visit(i) {
        if (mask[i] || reached[i]) return;
        reached[i] = 1;
        queue[end++] = i;
    }
    for (const i of seeds) visit(i);
    for (let start = 0; start < end; start++) {
        const i = queue[start];
        const col = i%width;
        if (col) visit(i-1);
        if (col+1 < width) visit(i+1);
        if (i >= width) visit(i-width);
        if (i < mask.length-width) visit(i+width);
    }
    return reached;
}

export function* gridBorder(width, height) {
    for (let col = 0; col < width; col++) {
        yield col;
        yield (height-1)*width+col;
    }
    for (let row = 1; row+1 < height; row++) {
        yield row*width;
        yield row*width+width-1;
    }
}

function components(mask, width, height, queue, consume) {
    const seen = new Uint8Array(mask.length);
    for (let i = 0; i < mask.length; i++) {
        if (!mask[i] || seen[i]) continue;
        seen[i] = 1;
        queue[0] = i;
        let end = 1;
        for (let start = 0; start < end; start++) {
            const cell = queue[start], x = cell%width, y = Math.floor(cell/width);
            for (let row = Math.max(0,y-1); row <= Math.min(height-1,y+1); row++) {
                for (let col = Math.max(0,x-1); col <= Math.min(width-1,x+1); col++) {
                    const adjacent = row*width+col;
                    if (!mask[adjacent] || seen[adjacent]) continue;
                    seen[adjacent] = 1;
                    queue[end++] = adjacent;
                }
            }
        }
        consume(queue.subarray(0,end));
    }
}

export function selectWallBands(categories, {width, height, resolution}, queue) {
    if (!categories.includes(1)) {
        return {wall:new Uint8Array(categories.length), perimeterBands:0,
            perimeterCells:0, bridgedWallCells:0};
    }
    const known = categories.map(value => Number(value !== 0));
    const morph = (mask, metres, dilate) => {
        const radius = metres/resolution*3;
        const distance = distanceTo(mask,width,height,dilate ? 1 : 0);
        return Uint8Array.from(distance, value => Number(dilate ? value <= radius : value > radius));
    };
    function contourDistance(footprint) {
        const exterior = floodGrid(footprint,width,height,gridBorder(width,height),queue);
        const toOutside = distanceTo(exterior,width,height,1);
        const toInside = distanceTo(exterior,width,height,0);
        return toOutside.map((distance,i) => {
            const x = i%width, y = Math.floor(i/width);
            // Outside the raster is also a boundary for cropped surveys.
            const borderDistance = 3*Math.min(x+1,y+1,width-x,height-y);
            return Math.min(distance,borderDistance)+toInside[i];
        });
    }
    function compactContour(radius) {
        const opened = morph(morph(known,radius,false),radius,true);
        // An entirely narrow survey can still use its measured outer contour.
        const compact = opened.some(Boolean) ? opened : known;
        const footprint = morph(morph(compact,ENVELOPE_NOTCH_M,true),ENVELOPE_NOTCH_M,false);
        return contourDistance(footprint);
    }
    const contour = compactContour(RAY_FRINGE_M);
    // Allow for wider coherent scan streaks only when the occupied band
    // itself persists nearby. The second envelope supplies a search region,
    // never new occupied evidence.
    const coarseContour = compactContour(RAY_FRINGE_M*2);
    const observedContour = contourDistance(known);
    const nearContour = WALL_SEARCH_M/resolution*3+3;
    const band = categories.map((value,i) => {
        if (value !== 2) return 0;
        return contour[i] <= nearContour ||
            coarseContour[i] <= nearContour ||
            (observedContour[i] <= nearContour/2 && contour[i] <= 2*nearContour) ? 2 : 0;
    });

    // Repair short holes in selection along supported straight or diagonal
    // runs. Scan the original occupied raster so free doors and unknown gaps
    // with no returns across the wall thickness cannot become walls.
    const repaired = new Set();
    const minimumAnchor = Math.max(2,Math.ceil(0.06/resolution));
    for (const [dx,dy] of [[1,0],[0,1],[1,1],[-1,1]]) {
        const stepMeters = resolution*Math.hypot(dx,dy);
        const maxGap = Math.floor(MAX_EDGE_REPAIR_M/stepMeters+1e-9);
        if (!maxGap) continue;
        const index = (x,y) => y*width+x;
        const inside = (x,y) => x>=0 && x<width && y>=0 && y<height;
        const anchor = (x,y,sign) => {
            for (let n=0; n<minimumAnchor; n++, x+=sign*dx, y+=sign*dy) {
                if (!inside(x,y) || !band[index(x,y)]) return false;
            }
            return true;
        };
        for (let y=0; y<height; y++) {
            for (let x=0; x<width; x++) {
                if (!band[index(x,y)] || !anchor(x,y,-1)) continue;
                const gap = [];
                for (let n=1; n <= maxGap+1; n++) {
                    const nx=x+n*dx, ny=y+n*dy, i=index(nx,ny);
                    if (!inside(nx,ny) || categories[i] !== 2) break;
                    if (band[i]) {
                        if (anchor(nx,ny,1)) for (const cell of gap) repaired.add(cell);
                        break;
                    }
                    gap.push(i);
                }
            }
        }
    }
    for (const cell of repaired) band[cell]=2;
    const wall = new Uint8Array(categories.length);
    let perimeterBands = 0, perimeterCells = 0;
    components(band,width,height,queue,cells => {
        let minX = width, minY = height, maxX = 0, maxY = 0;
        let touchesSurveyContour = false;
        for (const cell of cells) {
            const x = cell%width, y = Math.floor(cell/width);
            minX = Math.min(minX,x); minY = Math.min(minY,y);
            maxX = Math.max(maxX,x); maxY = Math.max(maxY,y);
            touchesSurveyContour ||= contour[cell] <= nearContour ||
                (observedContour[cell] <= nearContour/2 && contour[cell] <= 2*nearContour);
        }
        if (!touchesSurveyContour || cells.length*resolution < MIN_WALL_LENGTH_M ||
                Math.hypot(maxX-minX,maxY-minY)*resolution+resolution < MIN_WALL_SPAN_M) return;
        for (const cell of cells) wall[cell] = 2;
        perimeterBands++;
        perimeterCells += cells.length;
    });
    return {wall, perimeterBands, perimeterCells,
        bridgedWallCells: [...repaired].filter(i => wall[i]).length};
}

// March at cell centres, with 8-connected walls and 4-connected background.
// At a diagonal contact both corners share a filled saddle, so the extrusion
// has a continuous top and two continuous sides instead of a point contact.
export function traceWallRings(wall, {width, height, resolution}) {
    const edges = new Map();
    const vertexWidth = 2*width+3;
    const occupied = (x,y) => x >= 0 && x < width && y >= 0 && y < height && Boolean(wall[y*width+x]);
    const cases = [
        [], [[0,3]], [[1,0]], [[1,3]], [[2,1]], [[0,1],[2,3]], [[2,0]], [[2,3]],
        [[3,2]], [[0,2]], [[1,2],[3,0]], [[1,2]], [[3,1]], [[0,1]], [[3,0]], [],
    ];
    for (let y = -1; y < height; y++) {
        for (let x = -1; x < width; x++) {
            const code = Number(occupied(x,y)) | Number(occupied(x+1,y))<<1 |
                Number(occupied(x+1,y+1))<<2 | Number(occupied(x,y+1))<<3;
            if (!cases[code].length) continue;
            const node = (2*y+2)*vertexWidth+2*x+2;
            const corners = [node+1,node+vertexWidth+2,node+2*vertexWidth+1,node+vertexWidth];
            for (const [from,to] of cases[code]) edges.set(corners[from],corners[to]);
        }
    }
    const rings = [];
    while (edges.size) {
        const first = edges.keys().next().value;
        const points = [];
        let next = first;
        do {
            points.push([(next%vertexWidth-1)*resolution/2,
                (Math.floor(next/vertexWidth)-1)*resolution/2]);
            const previous = next;
            next = edges.get(previous);
            edges.delete(previous);
        } while (next !== first);
        rings.push(points);
    }
    return rings;
}

export function signedArea(points) {
    let twiceArea = 0;
    for (let i = 0, j = points.length-1; i < points.length; j = i++) {
        twiceArea += points[j][0]*points[i][1]-points[j][1]*points[i][0];
    }
    return twiceArea/2;
}

export function pointInRing([x,y], ring) {
    let inside = false;
    for (let i = 0, j = ring.length-1; i < ring.length; j = i++) {
        const [ax,ay] = ring[j], [bx,by] = ring[i];
        if ((ay > y) !== (by > y) && x < ax+(bx-ax)*(y-ay)/(by-ay)) inside = !inside;
    }
    return inside;
}

function lineDistance([x,y],[ax,ay],[bx,by]) {
    const dx = bx-ax, dy = by-ay;
    const t = Math.max(0,Math.min(1,((x-ax)*dx+(y-ay)*dy)/(dx*dx+dy*dy || 1)));
    return Math.hypot(x-ax-t*dx,y-ay-t*dy);
}

function simplifyWallRing(points, tolerance) {
    let split = 1;
    for (let i = 2; i < points.length; i++) {
        if (lineDistance(points[i],points[0],points[0]) >
                lineDistance(points[split],points[0],points[0])) split = i;
    }
    const loop = [...points,points[0]];
    const keep = new Set([0,split]);
    const pending = [[0,split],[split,points.length]];
    while (pending.length) {
        const [a,b] = pending.pop();
        let distance = tolerance, pivot = -1;
        for (let i = a+1; i < b; i++) {
            const deviation = lineDistance(loop[i],loop[a],loop[b]);
            if (deviation <= distance) continue;
            distance = deviation;
            pivot = i;
        }
        if (pivot < 0) continue;
        keep.add(pivot);
        pending.push([a,pivot],[pivot,b]);
    }
    const result = points.filter((_,i) => keep.has(i));
    return result.length >= 3 && Math.sign(signedArea(result)) === Math.sign(signedArea(points)) ?
        result : null;
}

export function segmentsMeet(a,b,c,d) {
    const epsilon = 1e-10;
    for (let axis = 0; axis < 2; axis++) {
        if (Math.max(a[axis],b[axis])+epsilon < Math.min(c[axis],d[axis]) ||
                Math.max(c[axis],d[axis])+epsilon < Math.min(a[axis],b[axis])) return false;
    }
    const side = (p,q,r) => (q[0]-p[0])*(r[1]-p[1])-(q[1]-p[1])*(r[0]-p[0]);
    const opposite = (u,v) => Math.min(u,v) <= epsilon && Math.max(u,v) >= -epsilon;
    return opposite(side(a,b,c),side(a,b,d)) && opposite(side(c,d,a),side(c,d,b));
}

function ringBounds(points) {
    const bounds = [Infinity,Infinity,-Infinity,-Infinity];
    for (const [x,y] of points) {
        bounds[0] = Math.min(bounds[0],x);
        bounds[1] = Math.min(bounds[1],y);
        bounds[2] = Math.max(bounds[2],x);
        bounds[3] = Math.max(bounds[3],y);
    }
    return bounds;
}

function ringsMeet(a,b,bounds) {
    if (a !== b) {
        const [ax,ay,bx,by] = bounds.get(a), [cx,cy,dx,dy] = bounds.get(b);
        if (bx < cx || dx < ax || by < cy || dy < ay) return false;
    }
    for (let i = 0; i < a.length; i++) {
        for (let j = 0; j < b.length; j++) {
            if (a === b && (j <= i+1 || i === 0 && j === a.length-1)) continue;
            if (segmentsMeet(a[i],a[(i+1)%a.length],b[j],b[(j+1)%b.length])) return true;
        }
    }
    return false;
}

export function prepareWallContours(wall, info) {
    const groups = [], holes = [];
    for (const ring of traceWallRings(wall,info)) {
        if (signedArea(ring) > 0) groups.push({outer:ring, holes:[]});
        else holes.push(ring);
    }
    for (const hole of holes) {
        // Retain an actual passage. Omit only tiny enclosed pinholes in the
        // triangulation of an already measured, connected wall band.
        const [x0,y0,x1,y1] = ringBounds(hole);
        if (Math.max(x1-x0,y1-y0) <= MAX_PINHOLE_M) continue;
        let owner = null;
        for (const group of groups) {
            if (pointInRing(hole[0],group.outer) &&
                    (!owner || signedArea(group.outer) < signedArea(owner.outer))) owner = group;
        }
        if (owner) owner.holes.push(hole);
    }
    const tolerance = Math.min(0.035, info.resolution*1.4);
    const originals = groups.flatMap(({outer, holes}) => [outer,...holes]);
    const simplified = originals.map(ring => {
        // A full-size tolerance can collapse a one-cell diagonal band.
        // Retry with a smaller tolerance before falling back to the raster.
        for (const scale of [1,0.5,0.25]) {
            const candidate = simplifyWallRing(ring,tolerance*scale);
            if (candidate && !ringsMeet(candidate,candidate)) return candidate;
        }
        return ring;
    });
    // Protect holes and open passages when a chord would cross a neighbour.
    const bounds = new Map([...originals,...simplified].map(ring => [ring,ringBounds(ring)]));
    const collided = new Set();
    for (let i = 0; i < simplified.length; i++) {
        for (let j = i+1; j < simplified.length; j++) {
            if (simplified[i] === originals[i] && simplified[j] === originals[j]) continue;
            if (ringsMeet(simplified[i],simplified[j],bounds) ||
                    ringsMeet(simplified[i],originals[j],bounds) || ringsMeet(originals[i],simplified[j],bounds) ||
                    pointInRing(originals[i][0],originals[j]) !== pointInRing(simplified[i][0],simplified[j]) ||
                    pointInRing(originals[j][0],originals[i]) !== pointInRing(simplified[j][0],simplified[i])) {
                collided.add(i);
                collided.add(j);
            }
        }
    }
    for (const i of collided) simplified[i] = originals[i];
    let offset = 0;
    return groups.map(({holes}) => ({outer:simplified[offset++],
        holes:holes.map(() => simplified[offset++])}));
}
