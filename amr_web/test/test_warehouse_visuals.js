import {fitRackPallet, palletCloudVolume, pickupPreview, slotOpening, warehousePallets} from '../resources/web/js/warehouse_visuals.js';
import {createCloudBoundary, resolveObstacles} from '../resources/web/js/map_geometry.js';
import {laserCoverage} from '../resources/web/js/laser_obstacles.js';

function assert(value, message) {
    if (!value) throw new Error(message);
}

const rack = {id: 'rack', type: 'rack', slots: [
    {id: 'a', shelf_slot_id: 'A', x: 2, y: 1, z: .2, yaw: 0},
    {id: 'b', shelf_slot_id: 'B', x: 2, y: 2, z: .55, yaw: 0},
]};
const layout = {objects: [rack], cargo_catalog: [
    {cargo_type: 'medium', width: .21, depth: .18, height: .2},
    {cargo_type: 'tall', width: .28, depth: .24, height: .47},
]};
let pallets = warehousePallets({slots: {A: 'full', B: 'empty'}}, layout);
assert(pallets.length === 1 && pallets[0].id === 'shelf_A', 'occupied only');
assert(pallets[0].height === .05 && pallets[0].width === .30 && pallets[0].depth === .40, 'pallet measurements');
assert(pallets[0].boxes[0].suppressCloud === false, 'unverified placeholder does not suppress cloud');
const selected = {id: 'shelf_A', slot: 'A', pose: {x: 2, y: 1, z: .225, yaw: Math.PI/2}};
pallets = warehousePallets({slots: {A: 'full'}, target: selected}, layout);
assert(pallets.length === 1 && pallets[0].id === 'shelf_A' && pallets[0].pose.y === 1 &&
       pallets[0].pose.z === .2 && pallets[0].selectedFace === selected.pose,
       'selected face marks the same pallet without lifting it off the slot');
const carried = {...selected, cargo_type: 'tall'};
pallets = warehousePallets({slots: {A: 'full'}, carried}, layout);
assert(pallets.length === 1 && pallets[0].state === 'carried' && pallets[0].boxes[0].height === .47,
       'pickup uses the same ID and a dimensioned cargo type');
pallets = warehousePallets({slots: {B: 'full'}, drops: {B: {...carried, cargo_type: 'medium'}}}, layout);
assert(pallets.length === 1 && pallets[0].id === 'shelf_A' && pallets[0].pose.z === .55 &&
       pallets[0].boxes[0].height === .20, 'confirmed destination slot and catalog sizes');

const pending = warehousePallets({carried, pending_slot:'B', slots:{B:'full'}},layout);
assert(pending.length===1 && pending[0].state==='carried','early occupancy cannot imply release');
for (const occupancy of ['empty', 'full', 'unknown']) {
    const placed = warehousePallets({slots:{B:occupancy},drops:{B:carried}},layout);
    assert(placed.length===1 && placed[0].id===carried.id && placed[0].state==='rack',
        'confirmed ownership survives asynchronous occupancy');
}

const yaw = .8;
const c = Math.cos(yaw), s = Math.sin(yaw);
const slot = (id,x,z,cartonHeight) => ({id,shelf_slot_id:id,x:3+c*x,y:2+s*x,z,yaw:yaw-Math.PI/2,
    cartonHeight});
const actual = {type:'rack',id:'shelf_front',x:3,y:2,z:0,yaw,width:1.875,depth:.35,height:.9,
    partitions:[-.3125,.3125],
    supports:[{id:'A1',x:.625,z:.2,width:.625},{id:'A2',x:.625,z:.55,width:.625},
        {id:'B1',x:0,z:.2,width:.625},{id:'B2',x:-.625,z:.2,width:.625}],
    slots:[slot('A1',.625,.2,.15),slot('A2',.625,.55,.15),
        slot('B1',0,.2,.4),slot('B2',-.625,.2,.4)]};
const real = {objects:[actual],cargo_catalog:[
    {cargo_type:'large',width:.4,depth:.61,height:.41,sizeSource:'measured'},
    {cargo_type:'measured',width:.26,depth:.20,height:.15,sizeSource:'configured',yaw:.1},
]};
function forShelf(snapshot) { return warehousePallets(snapshot,real); }
function near(a,b) { return Math.abs(a-b)<1e-8; }
const [low,upper,tall] = forShelf({slots:{A1:'full',A2:'full',B1:'full',B2:'empty'}});
assert(low.boxes[0].height===.15 && low.boxes[0].heightSource==='slot_nominal' &&
       upper.boxes[0].height===.15 && tall.boxes[0].height===.4 && tall.boxes[0].heightSource==='slot_nominal',
    'fixed display heights for occupied A and B slots');
assert(near(slotOpening(actual,actual.slots[0]).top,.515) &&
       near(slotOpening(actual,actual.slots[2]).top,.9), 'upper board only limits its own column');
for (const item of [low,upper,tall]) assert(!fitRackPallet(item,real).issues.length,'nominal carton fits rotated model');
const [observed] = forShelf({slots:{A1:'full'},drops:{A1:{id:'known',cargo_type:'measured',height_m:.249}}});
assert(observed.boxes[0].height===.15 && observed.boxes[0].heightSource==='slot_nominal' &&
    observed.boxes[0].observedHeight===.249 && observed.boxes[0].yaw===.1 &&
    fitRackPallet(observed,real).pallet.boxes[0].height===.15,
    'shelf presentation uses the slot height and retains the unmodified camera measurement');
const [oversize] = forShelf({slots:{A1:'full'},drops:{A1:{id:'bad',cargo_type:'large'}}});
const conflict = fitRackPallet(oversize,real);
assert(conflict.pallet.boxes.length===0 && conflict.issues[0].configuredHeight===.41 &&
       near(conflict.issues[0].availableHeight,.265) &&
       conflict.issues[0].axes.includes('slot_width') && conflict.issues[0].axes.includes('slot_height'),
       'oversize measured geometry raises a report without scaling or intersecting rack');
const [tallCamera] = forShelf({slots:{A1:'full'},drops:{A1:{id:'bad-camera',height_m:.40}}});
assert(fitRackPallet(tallCamera,real).pallet.boxes[0].height===.15 &&
    tallCamera.boxes[0].observedHeight===.40, 'sensor noise does not change the rack presentation');
assert(!forShelf({slots:{A1:'full'},departed:{A1:'shelf_A1'}}).length,
    'a lingering source sensor does not rematerialize a released pallet');
const [chosen] = forShelf({slots:{B1:'full'},drops:{B1:{id:'one',height_m:.39}},
    target:{id:'one',slot:'B1',pose:{x:3,y:2,z:.32,yaw}}});
assert(chosen.id==='one' && chosen.pose.z===.2 && chosen.boxes[0].height===.40 && chosen.boxes[0].observedHeight===.39,
    'retrieval reuses the measured cargo of the known shelf load');
const [inTransit] = forShelf({slots:{B1:'full'},carried:{id:'one',slot:'B1',height_m:.39}});
assert(inTransit.id==='one' && inTransit.state==='carried' && inTransit.boxes[0].height===.39,
    'measured height and identity survive pickup');

const map = {width: 20, height: 20, resolution: .1, origin: {x: 0, y: 0, theta: 0}};
const wall = new Uint8Array(400);
const known = {x: 1, y: 1, z: .3, width: .40, depth: .40, height: .45, yaw: 0};
const cloud = {id: 1, ...known};
assert(!resolveObstacles([cloud], wall, map, [known], .12).length, 'duplicate is suppressed');
const nextTo = {...cloud, x: 1.39};
assert(resolveObstacles([nextTo], wall, map, [known], .12).length === 1,
       'light contact does not hide an independent object');
const connected = {...cloud, x: 1.28, width: .65};
const [remainder] = resolveObstacles([connected], wall, map, [known], .12);
assert(remainder && remainder.x-remainder.width/2 >= 1.2-1e-9, 'partial intersection is clipped');
assert(resolveObstacles([cloud], wall, map, [{...known, x: 1.6, carried: true}], .12).length,
       'moving cargo does not suppress the old location');
assert(resolveObstacles([cloud], wall, map, [{...known, suppressCloud: false}], .12).length,
       'placeholder footprint has no suppression');
const rotatedRack = {x: 1, y: 1, z: 0, width: 1.875, depth: .35, height: .9, yaw: .015};
const rackCloud = {id: 3, x: 1, y: 1, z: .15, width: 1.8, depth: .34, height: .70, yaw: 0};
assert(!resolveObstacles([rackCloud], wall, map, [rotatedRack], .12).length,
       'rotated rack suppresses its duplicate cloud volume');
const separate = {...rackCloud, x: 1.1, y: 1.48, width: .42, depth: .4};
const [beside] = resolveObstacles([separate], wall, map, [rotatedRack], .12);
assert(beside && beside.id === separate.id, 'independent object near the rack is not erased');
const merged = {...separate, y: 1.31, depth: .62};
const [clipped] = resolveObstacles([merged], wall, map, [rotatedRack], .12);
assert(clipped && clipped.depth < merged.depth && clipped.y-clipped.depth/2 > 1.17,
       'near-aligned rack clips a partial overlap while retaining the independent remainder');
const high = {...rackCloud, z: 1.03};
assert(resolveObstacles([high], wall, map, [rotatedRack], .12).length === 1,
       'cloud above the rack has no vertical overlap');
// A complete load contributes both the pallet and carton without counting the
// unoccupied corners of their joint envelope.
const palletPart = {x:1,y:1,z:0.2,yaw:Math.PI/2,width:.4,depth:.3,height:.05};
const cartonPart = {x:1,y:1,z:0.25,yaw:Math.PI/2,width:.26,depth:.20,height:.16};
const volume = palletCloudVolume([palletPart,cartonPart]);
assert(Math.abs(volume.height-.21)<1e-9 && volume.yaw===palletPart.yaw,
       'load envelope follows pallet yaw and includes the carton height');
const loadCloud = {id:8,x:1,y:1,z:.22,yaw:0,width:.21,depth:.30,height:.19};
assert(!resolveObstacles([loadCloud],wall,map,[volume],.12).length, 'confirmed load is suppressed');
const emptyEnvelope = {...loadCloud,x:1.12,y:1.17,z:.35,width:.1,depth:.1,height:.1};
assert(resolveObstacles([emptyEnvelope],wall,map,[volume],.12).length===1,
       'air beyond carton inside the load envelope does not erase a nearby object');
const estimatedLoad = palletCloudVolume([palletPart,{...cartonPart,suppressCloud:false}]);
assert(estimatedLoad===palletPart && resolveObstacles([loadCloud],wall,map,[estimatedLoad],.12).length,
       'unverified cargo does not enlarge the suppression volume');

// A floor selection is already a confirmed detector target. The same
// displayed pose drives geometry and sensor overlap for PICKUP.
const floor = {id:'pick_session_1',pose:{x:.8,y:1,z:.025,yaw:0},height_m:.249};
const initial = pickupPreview(null,floor);
const jitter = {...floor,pose:{...floor.pose,x:.82,yaw:.03}};
const tracked = pickupPreview(initial,jitter);
assert(tracked.pose.x > .8 && tracked.pose.x < .82 && pickupPreview(tracked,jitter) === tracked,
       'valid pose changes blend and repeated telemetry does not cause extra motion');
const locked = pickupPreview(tracked,{...jitter,locked:true});
const late = pickupPreview(locked,{...jitter,pose:{...jitter.pose,x:1.1}});
assert(late.pose===tracked.pose && late.locked,'locked face ignores later camera movement');
const [floorPallet] = warehousePallets({target:floor},{objects:[]},initial);
assert(near(floorPallet.pose.x,.95) && floorPallet.pose.z===0 && floorPallet.boxes[0].height===.249,
       'pose is the bottom centre, height belongs to the camera carton');
const [held] = warehousePallets({carried:floor},{objects:[]});
assert(held.id===floorPallet.id && held.boxes[0].id===floorPallet.boxes[0].id &&
       held.boxes[0].height===floorPallet.boxes[0].height, 'carrying preserves the selected geometry identity');
const solidCarton = {...floorPallet.boxes[0],x:1,y:1,z:.05};
const floorVolume = palletCloudVolume([{...floorPallet,...floorPallet.pose},solidCarton]);
const matching = {id:91,x:1,y:1,z:.09,width:.22,depth:.19,height:.17};
const nearby = {...matching,id:92,x:1.26,width:.20};
const result = resolveObstacles([matching,nearby],wall,map,[floorVolume],.12);
assert(result.length===1 && result[0].id===92, 'suppress camera duplicate while preserving independent object');
const covered = laserCoverage(map,wall,wall,createCloudBoundary(map,wall),[solidCarton]);
assert(covered(1.124,1,.23) && !covered(1.13,1,.23) && !covered(1,1,.32),
       'floor pickup laser masking stays inside the rendered carton');
print('warehouse pallet and cloud tests passed');
