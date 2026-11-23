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
    [218, 227, 222], // free: flat, light floor
    [91, 108, 112],  // occupied floor under the walls
];

export function mapPixels(categories, width, height) {
    const pixels = new Uint8ClampedArray(width * height * 4);
    for (let row = 0; row < height; row++) {
        for (let column = 0; column < width; column++) {
            const category = categories[row * width + column];
            const color = COLORS[category];
            const pixel = ((height - 1 - row) * width + column) * 4;
            pixels[pixel] = color[0];
            pixels[pixel + 1] = color[1];
            pixels[pixel + 2] = color[2];
            pixels[pixel + 3] = category === 0 ? 28 : 255;
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

// Build one watertight-looking surface from occupied cells. Internal faces between
// adjacent rectangles are omitted, so a tall wall does not show box seams.
export function buildWallSurfaceData(categories, width, height, resolution, wallHeight,
                                     rectangles = mergeOccupiedRectangles(categories, width, height)) {
    const positions = [];
    const normals = [];
    const indices = [];
    let quads = 0;
    const z0 = 0.002;
    const z1 = z0 + wallHeight;
    function quad(a, b, c, d, normal) {
        const base = positions.length / 3;
        for (const point of [a, b, c, d]) {
            positions.push(...point);
            normals.push(...normal);
        }
        indices.push(base, base + 1, base + 2, base, base + 2, base + 3);
        quads++;
    }
    for (const [x, y, w, h] of rectangles) {
        const x0 = x * resolution;
        const x1 = (x + w) * resolution;
        const y0 = y * resolution;
        const y1 = (y + h) * resolution;
        quad([x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1], [0, 0, 1]);
    }
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
    return {
        positions: new Float32Array(positions),
        normals: new Float32Array(normals),
        indices: positions.length / 3 <= 65535 ? new Uint16Array(indices) : new Uint32Array(indices),
        quads,
    };
}

export function createDisplayVariants(decoded) {
    const {width, height, resolution} = decoded.info;
    // Physical limits prevent Clean from hiding large objects on coarse-resolution maps.
    const maxCells = Math.min(8, Math.floor(0.0032 / (resolution * resolution) + 1e-9));
    const maxSpanCells = Math.floor(0.10 / resolution + 1e-9);
    const filtered = cleanSmallOccupiedComponents(decoded.categories, width, height,
                                                  maxCells, maxSpanCells);
    return {
        raw: {
            categories: decoded.categories,
            pixels: decoded.pixels,
            rectangles: mergeOccupiedRectangles(decoded.categories, width, height),
            occupiedCells: decoded.occupied.length / 2,
        },
        clean: {
            categories: filtered.categories,
            pixels: mapPixels(filtered.categories, width, height),
            rectangles: mergeOccupiedRectangles(filtered.categories, width, height),
            occupiedCells: decoded.occupied.length / 2 - filtered.removedCells,
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
