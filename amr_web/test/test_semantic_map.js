import {prepareWallContours, signedArea, traceWallRings} from '../resources/web/js/wall_contours.js';
import {buildWallSurfaceData, cellCenterWorld, createCloudBoundary, createDisplayVariants, extractPerimeter,
        mapPixels, occupiedFootprintCells, rackWallClipping, resolveObstacles, WALL_HEIGHT} from '../resources/web/js/map_geometry.js';
import {normalizeSemanticLayout, normalizeSemanticState} from '../resources/web/js/semantic_layout.js';

function assert(condition, message) {
    if (!condition) throw new Error(message);
}

const palette=mapPixels(new Uint8Array([0,1,2]),3,1);
assert(palette[4]===207&&palette[5]===216&&palette[6]===214,
       'free floor palette is #CFD8D6');
assert(palette[8]===63&&palette[9]===78&&palette[10]===85,
       'interior occupied palette is #3F4E55');

const info = {width: 80, height: 60, resolution: 0.05,
    origin: {x: 4, y: -2, theta: Math.PI/2}, frame_id: 'map'};
const at = (x, y) => y*info.width + x;
const cells = new Uint8Array(info.width*info.height).fill(1);
cells.fill(0, 0, at(0, 5));
for (let x = 5; x < 73; x++) {
    if (x !== 44 && x !== 45) {
        cells[at(x, 5)] = 2;
        cells[at(x, 6)] = 2;
    }
    cells[at(x, 22)] = 2; // Long interior occupied line.
}
// The raw band is still occupied here but has no clear local exterior sample.
cells[at(21, 4)] = 1;
// Unknown pocket entirely enclosed by scanned free cells.
for (let y = 33; y < 44; y++) {
    for (let x = 22; x < 42; x++) cells[at(x, y)] = y === 33 ? 2 : 0;
}
const before = cells.slice();
const decoded = {info, categories: cells, pixels: null,
    occupied: new Array(cells.reduce((sum, value) => sum + (value === 2 ? 2 : 0), 0))};
const perimeter = createDisplayVariants(decoded);
assert(perimeter.wall[at(12, 5)] && perimeter.wall[at(12, 6)], 'perimeter includes occupied thickness');
assert(perimeter.wall[at(21, 5)] && perimeter.wall[at(21, 6)],
       'an occupied band stays continuous even where local outside evidence is interrupted');
assert(!perimeter.wall[at(44, 5)] && !perimeter.wall[at(45, 6)], 'free opening remains open');
assert(!perimeter.wall[at(24, 22)], 'long interior occupied remains flat');
assert(!perimeter.wall[at(29, 33)], 'enclosed unknown pocket is not exterior');
assert(perimeter.wall.every((value, index) => !value || cells[index] === 2),
       'unknown and free never form wall geometry');
assert(cells.every((value, index) => value === before[index]), 'occupancy remains unchanged');

const freeToUnknown = new Uint8Array(cells.length).fill(1);
freeToUnknown.fill(0, 0, at(0, 5));
assert(extractPerimeter(freeToUnknown, info).perimeterCells === 0, 'a free/unknown interface is flat');
assert(extractPerimeter(new Uint8Array(cells.length), info).perimeterCells === 0, 'unknown-only is flat');

const diagonal = new Uint8Array(cells.length);
for (let y = 0; y < info.height; y++) {
    for (let x = 0; x < info.width; x++) {
        diagonal[at(x, y)] = x < y ? 0 : x === y ? 2 : 1;
    }
}
const diagonalWall = extractPerimeter(diagonal, info);
assert(diagonalWall.wall[at(10, 10)] && diagonalWall.wall[at(45, 45)],
       'diagonal occupied boundary with free/exterior sides is continuous');

// Free ray streaks on the outward side break a per-column classifier, but
// do not change the outer contour of the surveyed room.
const fringeInfo = {...info, width:100, height:80, resolution:.02};
const fringe = new Uint8Array(fringeInfo.width*fringeInfo.height);
const setFringe = (x,y,value) => { fringe[y*fringeInfo.width+x] = value; };
for (let y=20; y<=70; y++) {
    for (let x=10; x<=89; x++) {
        const door = x>=42 && x<=57 && y<=21;
        setFringe(x,y, !door && (x===10 || x===89 || y<=21 || y===70) ? 2 : 1);
    }
}
for (let x=12; x<89; x+=3) {
    for (let y=4; y<20; y++) setFringe(x,y,1);
}
for (let x=25; x<74; x++) setFringe(x,40,2);
setFringe(18,17,2);
const fringeWall = createDisplayVariants({info:fringeInfo,categories:fringe,
    occupied:new Array(fringe.reduce((sum,value) => sum+Number(value===2)*2,0))});
const fringeAt = (x,y) => fringeWall.wall[y*fringeInfo.width+x];
assert(fringeAt(10,20) && fringeAt(10,30) && fringeAt(20,20) && fringeAt(85,20),
       'solid contour keeps a right angle and long bands with external free streaks');
assert(!fringeAt(50,20) && !fringeAt(50,40) && !fringeAt(18,17),
       'free doorway, interior obstruction and loose exterior noise are excluded');
assert(fringeWall.wall.every((value,i) => !value || fringe[i]===2),
       'cleanup never promotes unknown or free cells');
const stairs = new Uint8Array(24*24);
for (let i=3; i<21; i++) stairs[i*24+i]=2;
const stairsInfo = {...info,width:24,height:24,resolution:.02};
const stairsContour = prepareWallContours(stairs,stairsInfo);
const stairsMesh = buildWallSurfaceData(stairs,stairsInfo,[],null,stairsContour);
assert(traceWallRings(stairs,stairsInfo).length === 1 && stairsContour.length === 1,
       'diagonal contacts create one continuous contour');
assert(stairsMesh.indices.length/3 < 36 &&
       stairsMesh.normals.some(v => v > .6 && v < .8),
       'a diagonal wall has long inclined faces instead of one extrusion per cell');
assert(signedArea(stairsContour[0].outer) > 0, 'wall ring keeps its orientation');

const rackCenter = cellCenterWorld(info, 30, 22);
const wallCenter = cellCenterWorld(info, 60, 5);
const layout = normalizeSemanticLayout({map_id: 'warehouse_a', frame_id: 'map', objects: [
    {id: 'rack_1', type: 'rack', ...rackCenter, yaw: 0.3,
     width: 0.2, depth: 0.2, height: 1.6,
     slots: [{id: 'slot_1', ...rackCenter, z: 0.6, yaw: 0.3}]},
    {id: 'rack_cut', type: 'rack', ...wallCenter, yaw: 0.48,
     width: 0.25, depth: 0.23, height: 1.6},
    {id: 'pallet_1', type: 'pallet', width: 0.8, depth: 0.6, height: 0.15,
     boxes: [{width: 0.3, depth: 0.2, height: 0.4}]},
], overrides: [{type: 'protect_wall', region: {...rackCenter, width: 0.2, depth: 0.2}}]},
'warehouse_a', 'map');
const composed = createDisplayVariants(decoded, layout);
assert(composed.rackWallConflicts.map(value => value.id).join() === 'rack_cut',
       'only the new perimeter can generate a rack conflict, even with legacy overrides');
assert(composed.wall[at(60, 5)] && cells[at(60, 5)] === 2,
       'rack clipping is applied to the mesh and does not rewrite the perimeter or source grid');
assert(!occupiedFootprintCells(composed.wall, info, layout.objects[0]).length,
       'rack on interior occupancy has no map walls through it');

function topArea(surface) {
    let result = 0;
    const {positions, normals, indices} = surface;
    for (let i = 0; i < indices.length; i += 3) {
        const [a,b,c] = Array.from(indices.subarray(i,i+3), j => j*3);
        if (normals[a+2] !== 1) continue;
        result += Math.abs((positions[b]-positions[a])*(positions[c+1]-positions[a+1]) -
                           (positions[c]-positions[a])*(positions[b+1]-positions[a+1]))/2;
        assert(Math.abs(positions[a+2]-WALL_HEIGHT-0.002) < 1e-6, 'wall height');
    }
    return result;
}
const full = buildWallSurfaceData(composed.wall, info, [], null, composed.contours);
const clipped = buildWallSurfaceData(composed.wall, info, composed.cuts, null, composed.contours);
const movedRack = {...layout.objects[1], ...rackCenter};
assert(!rackWallClipping(composed.wall, info, {objects:[movedRack]}, composed.contours).cuts.length,
       'moving a rack away from the contour clears its wall cut');
// Check the clipped triangles in rack-local coordinates. A small tolerance
// excludes the expected contact on the exact cut boundary.
const cutRack = layout.objects[1];
const mapYaw = info.origin.theta;
for (let i = 0; i < clipped.indices.length; i += 3) {
    const triangle = Array.from(clipped.indices.subarray(i,i+3), index => {
        const x = clipped.positions[index*3], y = clipped.positions[index*3+1];
        const dx = info.origin.x + Math.cos(mapYaw)*x - Math.sin(mapYaw)*y - cutRack.x;
        const dy = info.origin.y + Math.sin(mapYaw)*x + Math.cos(mapYaw)*y - cutRack.y;
        return [Math.cos(cutRack.yaw)*dx + Math.sin(cutRack.yaw)*dy,
                -Math.sin(cutRack.yaw)*dx + Math.cos(cutRack.yaw)*dy];
    });
    for (let u = 0; u <= 8; u++) {
        for (let v = 0; v <= 8-u; v++) {
            const weights = [u/8, v/8, (8-u-v)/8];
            const x = triangle.reduce((sum,p,index) => sum+p[0]*weights[index], 0);
            const y = triangle.reduce((sum,p,index) => sum+p[1]*weights[index], 0);
            assert(Math.abs(x) >= cutRack.width/2-1e-6 || Math.abs(y) >= cutRack.depth/2-1e-6,
                   'no retained triangles penetrate the rotated rack footprint');
        }
    }
}
const originalArea = topArea(full);
const clippedArea = topArea(clipped);
assert(clippedArea < originalArea && clippedArea > originalArea - 0.25*0.23,
       'rotated rack cuts only the intersecting portion of the perimeter');
assert(Array.from(clipped.normals).some(value => Math.abs(value) > 0.01 && Math.abs(value) < 0.99),
       'clipped perimeter has vertical cut faces');

const exactInfo = {...info, width: 1, height: 1, resolution: 1, origin: {x: 4, y: -2, theta: Math.PI/2}};
const cellRack = {...cellCenterWorld(exactInfo, 0, 0), yaw: Math.PI/2, width: 0.4, depth: 0.4};
const exact = buildWallSurfaceData(new Uint8Array([2]), exactInfo, [cellRack]);
assert(Math.abs(topArea(exact) - 0.84) < 1e-6, 'cutting part of a grid cell preserves its remaining area');
assert(exact.faces === 12, 'partial cut creates four caps, four outer sides and four top polygons');

const rackState = normalizeSemanticState({map_id: 'warehouse_a', objects: [
    {id: 'pallet_1', type: 'pallet', state: 'rack', rack_id: 'rack_1', slot_id: 'slot_1'},
]}, layout);
assert(rackState[0].pose.z === 0.6 && rackState[0].boxes[0].x === 0, 'slot and box poses still resolve');
const pictured = normalizeSemanticState({map_id:'warehouse_a', objects:[{
    id:'pallet_1',type:'pallet',state:'destination',pose:{x:4,y:-2},
    width:0.8,depth:0.6,height:0.15,boxes:[{id:'box', z:-0.04,width:.21,depth:.2,height:.3}],
}]}, layout)[0];
assert(pictured.width===.3 && pictured.depth===.4 && pictured.height===.05 &&
       pictured.boxes[0].z===0 && pictured.boxes[0].height===.3,
       'visual pallet uses the surveyed size, and carton starts at or above the deck');
assert(normalizeSemanticState({map_id: 'other_map', objects: []}, layout) === null,
       'semantic state from another map is rejected');
for (const [parent, expected] of [[{type: 'robot'}, 'carried'], [{type: 'map'}, 'destination']]) {
    const moved = normalizeSemanticState({map_id: 'warehouse_a', objects: [
        {id: 'pallet_1', type: 'pallet', parent, pose: {x: 0.1, y: -0.1}},
    ]}, layout);
    assert(moved[0].state === expected && moved[0].boxes.length === 1, 'pallet reparenting retains box');
}

const estimate = (id, pose) => ({id, ...pose, z: 0.12, yaw: 0, width: 0.14, depth: 0.14, height: 0.4});
const interior = estimate(1, cellCenterWorld(info, 24, 22));
const againstWall = estimate(2, cellCenterWorld(info, 10, 5));
const overRack = estimate(3, rackCenter);
const visible = resolveObstacles([interior, againstWall, overRack], composed.wall, info, layout.objects, .12);
assert(visible.length === 1 && visible[0] === interior,
       'interior texture neither hides nor raises a cloud box, perimeter and semantic rack suppress theirs');
const cloudWall = perimeter.wallEvidence.map((value, index) =>
    value || perimeter.wall[index] ? 2 : 0);
const boundary = createCloudBoundary(info, perimeter.outside, cloudWall);
const boxAt = (x, y, depth = .1) => ({...cellCenterWorld(info, x,y), width: .1, depth});
assert(!boundary.allowsBox(boxAt(30, 2)), 'box beyond the outer wall is rejected on an open map');
assert(boundary.allowsBox(boxAt(24, 21)), 'scanned interior is valid');
assert(boundary.allowsBox({...boxAt(30, 7),width:.02,depth:.02}),
       'cloud box next to the wall remains valid when its extent stays inside');
assert(!boundary.allowsBox({...boxAt(30, 5), depth: .3}),
       'box extent intersecting the perimeter wall is rejected');
const wallBehind = cellCenterWorld(info, 30, 2), wallSensor = cellCenterWorld(info, 30, 15);
assert(!boundary.allowsObservation([wallBehind.x,wallBehind.y],[wallSensor.x,wallSensor.y]),
       'a point observed behind a wall is rejected even when the map has an opening elsewhere');
const wallRayStart = cellCenterWorld(info, 30, 7), wallRayEnd = cellCenterWorld(info, 30, 4);
assert(!boundary.canConnect([wallRayStart.x,wallRayStart.y],[wallRayEnd.x,wallRayEnd.y]),
       'voxel clustering cannot join points across the wall');
assert(boundary.allowsBox(boxAt(28, 37)), 'unknown pocket is not automatically exterior');
assert(!boundary.allowsBox({...boxAt(24, 12), width: 2}),
       'footprint extent can reject a box whose centre is in scanned space');
assert(perimeter.raw.interior[at(24,22)] === 2 && !perimeter.raw.interior[at(24,5)] &&
       !perimeter.raw.interior[at(28,37)], 'strip mask uses only occupied interior cells');
const strip = buildWallSurfaceData(perimeter.clean.interior, info, [], .12981);
assert(Math.abs(Math.max(...Array.from(strip.positions).filter((_, i) => i%3===2))-.12981) < 1e-6,
       'strip top follows supplied lidar TF height');
const closedInfo = {width:40,height:40,resolution:.05,
    origin:{x:-3,y:5,theta:.37},frame_id:'map'};
const closed = new Uint8Array(closedInfo.width*closedInfo.height);
for (let y=5;y<35;y++) for (let x=5;x<35;x++) {
    const edge=x===5||x===34||y===5||y===34;
    closed[y*closedInfo.width+x]=edge?2:1;
}
const closedPerimeter = extractPerimeter(closed,closedInfo);
const closedWall = closedPerimeter.wallEvidence.map((value,index) =>
    value||closedPerimeter.wall[index]?2:0);
const closedBoundary = createCloudBoundary(closedInfo,closedPerimeter.outside,closedWall);
const closedInterior = cellCenterWorld(closedInfo,20,20);
const closedExterior = cellCenterWorld(closedInfo,2,20);
assert(closedPerimeter.perimeterCells>0 && closedBoundary.allowsPoint(closedInterior.x,closedInterior.y),
       'closed map retains its scanned interior');
assert(!closedBoundary.allowsPoint(closedExterior.x,closedExterior.y),
       'closed map rejects exterior unknown reached from the raster edge');
assert(!closedBoundary.canConnect([closedInterior.x,closedInterior.y],
                                 [closedExterior.x,closedExterior.y]),
       'closed map perimeter blocks cluster connectivity');

// An open map can have no topological exterior mask, while a direct sensor
// ray still proves that an occupied structural band occludes the point.
const openInfo={width:24,height:20,resolution:.1,origin:{x:2,y:-1,theta:0}};
const openWall=new Uint8Array(openInfo.width*openInfo.height);
for(let x=3;x<18;x++) if(x!==17) openWall[10*openInfo.width+x]=2;
const openBoundary=createCloudBoundary(openInfo,new Uint8Array(openWall.length),openWall);
const sensor=cellCenterWorld(openInfo,8,14), behind=cellCenterWorld(openInfo,8,6);
assert(openBoundary.allowsPoint(behind.x,behind.y) &&
       !openBoundary.allowsObservation([behind.x,behind.y],[sensor.x,sensor.y]),
       'open map rejects a point behind occupied wall evidence without marking all unknown exterior');
print('perimeter and semantic map tests passed');

