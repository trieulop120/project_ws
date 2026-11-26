export const PALLET_SIZE = Object.freeze({width: 0.40, depth: 0.30, height: 0.05}); // Visualization only, +X along forks.
export const PALLET_DECK_FRACTION = 0.24;
const zero = {x: 0, y: 0, z: 0, yaw: 0};
const poseAxes = ['x', 'y', 'z', 'yaw'];

export function pickupPreview(previous, target) {
    const raw = target?.pose;
    if (!raw || target.slot || !poseAxes.every(axis => Number.isFinite(raw[axis]))) return null;
    const next = {id:target.id, raw, pose:raw, locked:target.locked === true};
    if (!previous || previous.id !== next.id) return next;
    const latest = previous.pose;
    if (next.locked || previous.locked) return {...next, pose:latest, locked:true};
    if (poseAxes.every(axis => previous.raw[axis] === raw[axis])) return previous;
    // Do not drag the old visual across the floor when the camera reacquires
    // a different target. Small changes blend in map coordinates.
    const distance = Math.hypot(raw.x-previous.raw.x,raw.y-previous.raw.y);
    if (distance > .18) return next;
    const angle = raw.yaw-latest.yaw;
    const dyaw = Math.atan2(Math.sin(angle),Math.cos(angle));
    const alpha = .4;
    return {...next, pose:{x:latest.x+alpha*(raw.x-latest.x), y:latest.y+alpha*(raw.y-latest.y),
        z:latest.z+alpha*(raw.z-latest.z), yaw:latest.yaw+alpha*dyaw}};
}

function makePallet(id, parent, pose, evidence, catalog, nominalHeight = null) {
    const spec = catalog.find(entry => entry.cargo_type === evidence?.cargo_type);
    const observed = evidence?.height_m;
    const observedHeight = Number.isFinite(observed) && observed > 0 ? observed : null;
    const footprintSource = spec?.sizeSource ?? (spec ? 'configured' : 'placeholder');
    const shelfHeight = parent.type === 'rack_slot' ? nominalHeight : null;
    const heightSource = shelfHeight !== null ? 'slot_nominal' : observedHeight ? 'camera_estimate' :
        spec ? footprintSource : nominalHeight !== null ? 'source_slot_nominal' : 'placeholder';
    const height = shelfHeight ?? observedHeight ?? spec?.height ?? nominalHeight ?? .15;
    const approximate = footprintSource === 'placeholder' || heightSource === 'camera_estimate' ||
        heightSource === 'slot_nominal' || heightSource === 'source_slot_nominal';
    // A selected floor load represents an observed obstacle. Use only its
    // rendered volume for duplicate suppression, even with estimated XY.
    const exactCoverage = !evidence?.slot && parent.type !== 'rack_slot';
    const box = {...zero, id: `${id}_cargo`, width:spec?.width ?? .25, depth:spec?.depth ?? .22,
        height, yaw:spec?.yaw ?? 0, color:spec?.color, approximate,
        exactCoverage,
        suppressCloud:exactCoverage ||
            (footprintSource !== 'placeholder' && (!approximate || heightSource === 'camera_estimate')),
        footprintSource, heightSource, observedHeight, configuredHeight:spec?.height ?? null};
    return {id, parent, pose, offset: zero,
            state: {robot:'carried', rack_slot:'rack', map:'destination'}[parent.type],
            ...PALLET_SIZE, exactCoverage, cargoType: evidence?.cargo_type, cargoClass: evidence?.cargo_class,
            boxes: [box]};
}

export function warehousePallets(snapshot, layout, preview = null) {
    if (!snapshot || !layout) return [];
    const catalog = layout.cargo_catalog ?? [];
    const pallets = [];
    const {slots, target, carried, drops, departed, source_empty: sourceEmpty} = snapshot;
    const reserved = new Set(Object.values(drops ?? {}).map(object => object.id));
    if (carried) reserved.add(carried.id);
    const linked = [];
    for (const rack of layout.objects.filter(object => object.type === 'rack')) {
        for (const slot of rack.slots) {
            const id = slot.shelf_slot_id;
            if (!id) continue;
            const parent = {type:'rack_slot', rack_id:rack.id, slot_id:slot.id};
            linked.push({id, rack, slot});
            const ownership = drops?.[id]?.id !== carried?.id ? drops?.[id] : null;
            if (target?.slot === id && !carried) {
                // Detection identifies the slot. Keep the represented pallet on
                // its known support and retain the measured face for diagnostics.
                const pallet = makePallet(target.id,parent,slot,{...ownership,...target},
                    catalog,slot.cartonHeight);
                pallet.selectedFace = target.pose;
                pallet.selected = true;
                pallets.push(pallet);
                continue;
            }
            if (slots?.[id] !== 'full' || departed?.[id] || (!sourceEmpty && carried?.slot === id)) continue;
            const defaultId = `shelf_${id}`;
            const slotPalletId = reserved.has(defaultId) ? `${defaultId}_replacement` : defaultId;
            pallets.push(makePallet(ownership?.id ?? slotPalletId,parent,slot,ownership,catalog,
                slot.cartonHeight));
        }
    }
    if (target && !target.slot && !carried && target.pose &&
            poseAxes.every(axis => Number.isFinite(target.pose[axis]))) {
        // An unlinked face near an occupied slot has no independent identity.
        // Leave its existing shelf representation alone.
        const {x, y, z, yaw} = preview?.id === target.id ? preview.pose : target.pose;
        const pose = {x: x + Math.cos(yaw)*PALLET_SIZE.width/2,
                      y: y + Math.sin(yaw)*PALLET_SIZE.width/2,
                      z: z-PALLET_SIZE.height/2, yaw};
        const duplicatesSlot = linked.some(({id,rack,slot}) => {
            if (slots?.[id] !== 'full') return false;
            const room = slotOpening(rack,slot);
            if (!room) return false;
            const p = inRack(rack,pose);
            return p.x > room.minX && p.x < room.maxX && p.y > room.minY && p.y < room.maxY &&
                Math.abs(p.z-room.bottom) <= PALLET_SIZE.height;
        });
        if (!duplicatesSlot) {
            const pallet = makePallet(target.id, {type:'map'},pose,target,catalog);
            pallet.selectedFace = target.pose;
            pallet.selected = true;
            pallet.locked = target.locked === true;
            pallets.push(pallet);
        }
    }
    if (carried) {
        const origin = linked.find(({id}) => id === carried.slot);
        pallets.push(makePallet(carried.id,{type:'robot'},null,carried,catalog,origin?.slot.cartonHeight));
    }
    return pallets;
}

export function rackConstruction(rack) {
    const post = Math.min(.055,rack.width/12,rack.depth/12);
    const board = Math.min(.035,rack.height/15);
    const supports = rack.supports ?? [1/3,2/3,1].map(fraction =>
        ({width:rack.width, x:0, z:rack.height*fraction}));
    return {post,board,supports};
}

function inRack(rack, pose) {
    const c = Math.cos(rack.yaw), s = Math.sin(rack.yaw);
    const dx = pose.x-rack.x, dy = pose.y-rack.y;
    return {x:c*dx+s*dy, y:-s*dx+c*dy, z:pose.z-rack.z, yaw:pose.yaw-rack.yaw};
}

const EPS = 1e-5;

// This is clearance in the rendered construction. Board and post thickness
// come from the same renderer inputs, and are not surveyed slot measurements.
export function slotOpening(rack,slot) {
    const {post,board,supports} = rackConstruction(rack);
    const p = inRack(rack,slot);
    const surface = supports.find(s => s.id === slot.id) ??
        supports.find(s => Math.abs(s.z-p.z) <= EPS && Math.abs(s.x-p.x) < s.width/2);
    if (!surface || Math.abs(surface.z-p.z) > EPS || Math.abs(p.x-surface.x) >= surface.width/2) return null;
    let minX = Math.max(-rack.width/2+post,surface.x-surface.width/2);
    let maxX = Math.min(rack.width/2-post,surface.x+surface.width/2);
    for (const partition of rack.partitions ?? []) {
        if (partition < p.x) minX = Math.max(minX,partition+board/2);
        else maxX = Math.min(maxX,partition-board/2);
    }
    let top = rack.height;
    for (const support of supports) {
        if (support.z <= surface.z+EPS || support.x+support.width/2 <= minX+EPS ||
                support.x-support.width/2 >= maxX-EPS) continue;
        top = Math.min(top,support.z-board);
    }
    if (maxX <= minX || top <= surface.z) return null;
    return {minX,maxX,minY:-rack.depth/2,maxY:rack.depth/2,bottom:surface.z,top,
        source:'rendered_rack'};
}

function halfExtents(width,depth,yaw) {
    const c = Math.abs(Math.cos(yaw)), s = Math.abs(Math.sin(yaw));
    return [(c*width+s*depth)/2,(s*width+c*depth)/2];
}

function cartonInRack(rack, pallet, box) {
    const {pose} = pallet;
    const c = Math.cos(pose.yaw), s = Math.sin(pose.yaw);
    return inRack(rack,{x:pose.x+c*box.x-s*box.y,y:pose.y+s*box.x+c*box.y,
        z:pose.z+pallet.height+box.z,yaw:pose.yaw+box.yaw});
}

export function fitRackPallet(pallet, layout) {
    if (pallet.parent?.type !== 'rack_slot') return {pallet,issues:[]};
    const {rack_id:rackId,slot_id:slotId} = pallet.parent;
    const rack = layout?.objects.find(item => item.type === 'rack' && item.id === rackId);
    const slot = rack?.slots.find(item => item.id === slotId);
    const room = slot && slotOpening(rack,slot);
    const issues = [];
    const issue = (code,details={}) => issues.push({code,palletId:pallet.id,rackId,slotId,...details});
    if (!room) {
        issue('support_unavailable');
        return {pallet:null,issues};
    }
    const p = inRack(rack,pallet.pose);
    const [hx,hy] = halfExtents(pallet.width,pallet.depth,p.yaw);
    // A centred pallet may overhang the shallow rack along the forks by the
    // difference between the surveyed shelf depth and pallet length.
    const overhang = Math.max(0,pallet.width-rack.depth)/2;
    if (Math.abs(p.z-room.bottom) > EPS || p.z+pallet.height > room.top+EPS ||
            p.x-hx < room.minX-EPS || p.x+hx > room.maxX+EPS ||
            p.y-hy < room.minY-overhang-EPS || p.y+hy > room.maxY+overhang+EPS) {
        issue('pallet_outside_support');
        return {pallet:null,issues};
    }
    const boxes = [];
    for (const box of pallet.boxes) {
        const center = cartonInRack(rack,pallet,box);
        const [boxX,boxY] = halfExtents(box.width,box.depth,center.yaw);
        const xRoom = Math.min(center.x-room.minX,room.maxX-center.x);
        const yRoom = Math.min(center.y-room.minY,room.maxY-center.y);
        const heightRoom = room.top-center.z;
        const footprintScale = box.footprintSource === 'placeholder' ?
            Math.min(1,xRoom/boxX,yRoom/boxY) : 1;
        const height = box.heightSource === 'placeholder' ? Math.min(box.height,heightRoom) : box.height;
        const axes = [];
        if (footprintScale <= 0 || boxX*footprintScale > xRoom+EPS) axes.push('slot_width');
        if (footprintScale <= 0 || boxY*footprintScale > yRoom+EPS) axes.push('slot_depth');
        if (height <= 0 || Math.max(height,box.configuredHeight ?? 0) > heightRoom+EPS ||
                center.z < room.bottom-EPS) axes.push('slot_height');
        if (axes.length) {
            issue('carton_outside_clearance',{cartonId:box.id,axes,
                height:box.height,configuredHeight:box.configuredHeight ?? null,heightSource:box.heightSource,observedHeight:box.observedHeight ?? null,
                availableHeight:heightRoom,availableWidth:2*xRoom,availableDepth:2*yRoom,
                clearanceSource:room.source});
            continue;
        }
        boxes.push({...box,height,width:box.width*footprintScale,depth:box.depth*footprintScale});
    }
    return {pallet:{...pallet,boxes},issues};
}

// An oriented broad-phase envelope for the confirmed pallet and its contents.
// The matcher uses the parts to avoid suppressing empty space in the envelope.
export function palletCloudVolume(footprints) {
    const pallet = footprints[0];
    const parts = footprints.filter(part => part.suppressCloud !== false);
    if (parts.length < 2) return pallet;
    const c = Math.cos(pallet.yaw), s = Math.sin(pallet.yaw);
    let minX = -pallet.width/2, maxX = pallet.width/2;
    let minY = -pallet.depth/2, maxY = pallet.depth/2;
    let maxZ = pallet.z + pallet.height;
    for (const part of parts.slice(1)) {
        const dx = part.x-pallet.x, dy = part.y-pallet.y;
        const x = c*dx+s*dy, y = -s*dx+c*dy;
        const angle = part.yaw-pallet.yaw;
        const u = Math.abs(Math.cos(angle)), v = Math.abs(Math.sin(angle));
        const halfX = (u*part.width+v*part.depth)/2;
        const halfY = (v*part.width+u*part.depth)/2;
        minX = Math.min(minX,x-halfX); maxX = Math.max(maxX,x+halfX);
        minY = Math.min(minY,y-halfY); maxY = Math.max(maxY,y+halfY);
        maxZ = Math.max(maxZ,part.z+part.height);
    }
    const x = (minX+maxX)/2, y = (minY+maxY)/2;
    return {...pallet, x:pallet.x+c*x-s*y, y:pallet.y+s*x+c*y,
            width:maxX-minX, depth:maxY-minY, height:maxZ-pallet.z, parts};
}
