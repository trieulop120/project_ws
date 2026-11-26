import {PALLET_SIZE} from './warehouse_visuals.js';

// Scene coordinates use metres and radians. Child offsets are resolved from their parent.
const validNumber = value => typeof value === 'number' && Number.isFinite(value);
const validId = value => typeof value === 'string' && /^[A-Za-z0-9_-]{1,80}$/.test(value);
const positive = value => validNumber(value) && value > 0 && value <= 50;

function pose(value) {
    if (!value || ![value.x, value.y, value.z ?? 0, value.yaw ?? 0].every(validNumber)) {
        throw new Error('Invalid semantic pose');
    }
    return {x: value.x, y: value.y, z: value.z ?? 0, yaw: value.yaw ?? 0};
}

function offset(value) {
    return pose({x: 0, y: 0, ...value});
}

function dimensions({width, depth, height}) {
    if (![width, depth, height].every(positive)) throw new Error('Invalid semantic dimensions');
    return {width, depth, height};
}

export function composePose(parent, child, baseZ = 0) {
    const c = Math.cos(parent.yaw), s = Math.sin(parent.yaw);
    return {x: parent.x + c * child.x - s * child.y,
            y: parent.y + s * child.x + c * child.y,
            z: parent.z + baseZ + child.z, yaw: parent.yaw + child.yaw};
}

function scopedId(id, seen) {
    if (!validId(id) || seen.has(id)) throw new Error('Invalid or duplicate semantic ID');
    seen.add(id);
    return id;
}

function boxes(value, parent) {
    if (value === undefined) return [];
    if (!Array.isArray(value) || value.length > 32) throw new Error('Invalid semantic boxes');
    const seen = new Set();
    return value.map((box, index) => {
        const local = offset(box.pose ?? box);
        if (parent.type === 'pallet') local.z = Math.max(0, local.z);
        const approximate = box.approximate === true;
        const sizeSource = approximate ? 'placeholder' :
            box.dimensions_source === 'measured' ? 'measured' : 'configured';
        return {id: scopedId(box.id ?? `box_${index + 1}`, seen), type: 'box', parent,
                ...local, ...dimensions(box), approximate, footprintSource:sizeSource,
                heightSource:sizeSource, suppressCloud:!approximate};
    });
}

function palletParent(entry) {
    if (entry.parent !== undefined) {
        const parent = entry.parent;
        if (parent?.type === 'rack_slot' && validId(parent.rack_id) && validId(parent.slot_id)) {
            return {type: 'rack_slot', rack_id: parent.rack_id, slot_id: parent.slot_id};
        }
        if (parent?.type === 'robot' || parent?.type === 'map') return {type: parent.type};
        throw new Error('Invalid pallet parent');
    }
    if (entry.state === 'rack') return {type: 'rack_slot', rack_id: entry.rack_id, slot_id: entry.slot_id};
    if (entry.state === 'carried') return {type: 'robot'};
    if (entry.state === 'destination') return {type: 'map'};
    throw new Error('Missing pallet parent');
}

function placePallet(entry, template, racks) {
    const parent = palletParent(entry);
    const state = {rack_slot: 'rack', robot: 'carried', map: 'destination'}[parent.type];
    if (entry.state !== undefined && entry.state !== state) throw new Error('Pallet state conflicts with parent');
    let location = null;
    let relative = null;
    if (parent.type === 'rack_slot') {
        const slot = racks.get(parent.rack_id)?.slots.find(item => item.id === parent.slot_id);
        if (!slot) return null;
        location = composePose(slot, offset(entry.pose));
    } else if (parent.type === 'map') {
        location = pose(entry.pose);
    } else {
        relative = offset(entry.pose);
    }
    return {id: entry.id, type: 'pallet', state, parent, pose: location, offset: relative,
        ...PALLET_SIZE,
        boxes: entry.boxes === undefined ? template?.boxes ?? [] :
            boxes(entry.boxes, {type: 'pallet', id: entry.id})};
}

export function normalizeSemanticLayout(value, mapId, frameId) {
    if (!value || value.map_id !== mapId ||
            (value.frame_id !== undefined && value.frame_id !== frameId) ||
            (value.schema_version !== undefined && value.schema_version !== 1) ||
            !Array.isArray(value.objects) || value.objects.length > 200) {
        throw new Error('Semantic layout does not match selected map/frame or schema');
    }
    const ids = new Set();
    const shelfSlotIds = new Set();
    const objects = value.objects.map(object => {
        const id = scopedId(object?.id, ids);
        if (object.type === 'rack') {
            const size = dimensions(object);
            const rackPose = pose(object.pose ?? object);
            const slots = object.slots ?? [];
            if (!Array.isArray(slots) || slots.length > 100) throw new Error('Invalid rack slots');
            const slotIds = new Set();
            const supports = object.supports;
            const partitions = object.partitions ?? [];
            if ((supports !== undefined && (!Array.isArray(supports) || supports.length > 100 ||
                    supports.some(item => !item || !validNumber(item.x) || !validNumber(item.z) ||
                        item.z <= 0 || item.z > size.height || !positive(item.width) ||
                        Math.abs(item.x) + item.width/2 > size.width/2 + 1e-6))) ||
                    !Array.isArray(partitions) || partitions.length > 100 ||
                    partitions.some(x => !validNumber(x) || Math.abs(x) >= size.width/2)) {
                throw new Error('Invalid rack supports or partitions');
            }
            return {...rackPose, id, type: 'rack', ...size, supports, partitions, slots: slots.map(slot => {
                const slotId = scopedId(slot?.id, slotIds);
                if (slot.relative_to !== undefined && !['rack', 'map'].includes(slot.relative_to)) {
                    throw new Error('Invalid slot coordinate frame');
                }
                const slotPose = pose(slot.pose ?? slot);
                const cartonHeight = slot.cargo_height_m ?? null;
                if (cartonHeight !== null && !positive(cartonHeight)) {
                    throw new Error('Invalid slot carton height');
                }
                return {id: slotId, ...(slot.relative_to === 'rack' ? composePose(rackPose, slotPose) : slotPose),
                    shelf_slot_id: slot.shelf_slot_id == null ? undefined :
                        scopedId(slot.shelf_slot_id, shelfSlotIds),
                    cartonHeight,
                    boxes: boxes(slot.boxes, {type: 'rack_slot', rack_id: id, slot_id: slotId})};
            })};
        }
        if (object.type === 'pallet') {
            return {id, type: 'pallet', ...PALLET_SIZE, boxes: boxes(object.boxes, {type: 'pallet', id})};
        }
        if (object.type === 'box' && (object.parent?.type ?? 'map') === 'map') {
            return {id, type: 'box', parent: {type: 'map'}, ...pose(object.pose ?? object), ...dimensions(object)};
        }
        throw new Error(`Unsupported semantic object type or parent: ${object.type}`);
    });
    const racks = new Map(objects.filter(object => object.type === 'rack').map(rack => [rack.id, rack]));
    const placedPallets = [];
    for (const [index, object] of value.objects.entries()) {
        if (object.type !== 'pallet' || (object.parent === undefined && object.state === undefined)) continue;
        const pallet = placePallet({...object, boxes: undefined}, objects[index], racks);
        if (!pallet) throw new Error('Configured pallet refers to an unknown slot');
        placedPallets.push(pallet);
    }

    const rawOverrides = value.overrides ?? [];
    if (!Array.isArray(rawOverrides) || rawOverrides.length > 200) throw new Error('Invalid semantic overrides');
    function region(entry) {
        if (![entry.width, entry.depth].every(positive)) throw new Error('Invalid override region');
        return {...pose(entry.pose ?? entry), width: entry.width, depth: entry.depth};
    }
    const overrides = rawOverrides.map(entry => {
        if (entry?.type === 'protect_wall') return {type: entry.type, region: region(entry.region ?? {})};
        if (entry?.type === 'rack_wall_candidate' && racks.has(entry.rack_id)) {
            return {type: entry.type, rack_id: entry.rack_id,
                    region: entry.region === undefined ? null : region(entry.region)};
        }
        throw new Error('Invalid semantic override or rack reference');
    });
    const catalog = value.cargo_catalog ?? [];
    if (!Array.isArray(catalog) || catalog.length > 100) throw new Error('Invalid cargo catalog');
    const cargoIds = new Set();
    const cargo_catalog = catalog.map(item => {
        const cargo_type = scopedId(item?.cargo_type, cargoIds);
        const model = item.model ?? 'box';
        const sizeSource = item.dimensions_source ?? 'configured';
        const yaw = item.yaw ?? 0;
        if (model !== 'box' || (item.color !== undefined &&
                !/^#[0-9a-fA-F]{6}$/.test(item.color)) ||
                !['configured','measured'].includes(sizeSource) || !validNumber(yaw)) {
            throw new Error('Invalid cargo visual');
        }
        return {cargo_type, model, color: item.color, sizeSource, yaw, ...dimensions(item)};
    });
    return {schema_version: 1, map_id: mapId, frame_id: frameId, objects, placedPallets,
            overrides, cargo_catalog};
}

export function normalizeSemanticState(value, layout) {
    if (!layout || !value || value.map_id !== layout.map_id ||
            (value.frame_id !== undefined && value.frame_id !== layout.frame_id) ||
            !Array.isArray(value.objects) || value.objects.length > 100) return null;
    const byId = new Map(layout.objects.map(object => [object.id, object]));
    const racks = new Map(layout.objects.filter(object => object.type === 'rack')
                                       .map(object => [object.id, object]));
    const seen = new Set();
    const objects = [];
    try {
        for (const entry of value.objects) {
            if (entry?.type !== 'pallet') throw new Error('Invalid runtime pallet');
            scopedId(entry.id, seen);
            const template = byId.get(entry.id);
            if (template && template.type !== 'pallet') throw new Error('Pallet ID conflicts with scene object');
            const pallet = placePallet(entry, template, racks);
            if (pallet) objects.push(pallet);
        }
    } catch (_) {
        return null;
    }
    return objects;
}
