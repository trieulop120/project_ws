import * as THREE from 'three';
import {composePose, normalizeSemanticState} from './semantic_layout.js?v=1.5.0';
import {fitRackPallet, palletCloudVolume, pickupPreview, rackConstruction, warehousePallets} from './warehouse_visuals.js?v=1.1.0';
import {cartonGeometryKey, createCargoModels} from './cargo_models.js?v=1.2.0';

// Geometry belongs to this layer; map, cloud and robot resources remain separate.
export function createSemanticScene(scene, onChange) {
    const staticRoot = new THREE.Group();
    const cargoRoot = new THREE.Group();
    scene.add(staticRoot, cargoRoot);
    const rackMaterial = new THREE.MeshStandardMaterial({color: 0xc3a77a, metalness: 0.18, roughness: 0.70});
    const models = createCargoModels();
    let layout = null;
    let warehouse = null;
    let warehouseKey = null;
    let warehouseAt = 0;
    let preview = null;
    const slotSurfaces = new Map();
    const slotMaterials = {
        full: new THREE.MeshStandardMaterial({color: 0xb99b6b, roughness: 0.8}),
        empty: rackMaterial,
        unknown: new THREE.MeshStandardMaterial({color: 0x918574, roughness: 0.8}),
    };
    let attachment = null;
    let state = null;
    let stateMessage = null;
    let stateKey = null;
    let runtimeOwned = false;
    let lastStateAt = 0;
    let staticFootprints = [];
    let cargoFootprints = [];
    let cargoVolumes = [];
    let fitIssues = [];
    let fitKey = '';
    const carriedPosition = new THREE.Vector3();
    const carriedRotation = new THREE.Quaternion();
    const carriedEuler = new THREE.Euler();

    function box(parent, width, depth, height, x, y, z, material) {
        const mesh = new THREE.Mesh(new THREE.BoxGeometry(width, depth, height), material);
        mesh.position.set(x, y, z);
        parent.add(mesh);
        return mesh;
    }

    function clearChildren(root) {
        for (const child of [...root.children]) {
            root.remove(child);
            child.traverse(object => { if (object.geometry) models.disposeGeometry(object.geometry); });
        }
    }

    function rackMesh(rack) {
        const group = new THREE.Group();
        group.name = rack.id;
        group.position.set(rack.x, rack.y, rack.z);
        group.rotation.z = rack.yaw;
        const {post,board:shelfHeight,supports} = rackConstruction(rack);
        for (const x of [-rack.width / 2 + post / 2, rack.width / 2 - post / 2]) {
            for (const y of [-rack.depth / 2 + post / 2, rack.depth / 2 - post / 2]) {
                box(group, post, post, rack.height, x, y, rack.height / 2, rackMaterial);
            }
        }
        for (const support of supports) {
            const surface = box(group, support.width, rack.depth, shelfHeight, support.x, 0,
                                support.z - shelfHeight/2, rackMaterial);
            const slot = rack.slots.find(item => item.id === support.id);
            if (slot?.shelf_slot_id) {
                surface.name = `slot_${slot.id}_surface`;
                slotSurfaces.set(slot.shelf_slot_id, surface);
            }
        }
        for (const x of rack.partitions) {
            box(group, shelfHeight, rack.depth, rack.height, x, 0, rack.height/2, rackMaterial);
        }
        staticRoot.add(group);
    }

    function addBox(parent, item, geometrySize = item) {
        const holder = new THREE.Group();
        holder.name = item.id;
        place(holder, item);
        holder.add(models.createCarton(geometrySize));
        describeCarton(holder,item);
        parent.add(holder);
        return holder;
    }

    function describeCarton(holder,item) {
        holder.userData.approximate = item.approximate === true;
        holder.userData.dimensions = {width:item.width,depth:item.depth,height:item.height,
            footprintSource:item.footprintSource,heightSource:item.heightSource,
            cameraHeight:item.observedHeight ?? null,configuredHeight:item.configuredHeight ?? null};
    }

    function updateCartons(group, records, pallet) {
        const active = new Set();
        for (const item of pallet.boxes) {
            active.add(item.id);
            // Live floor loads can receive a new height on each scan. Keep
            // their pooled geometry and apply the height along local +Z.
            const geometrySize = item.exactCoverage ? {...item,height:.25} : item;
            const key = cartonGeometryKey(geometrySize);
            let record = records.get(item.id);
            const pose = {...item, z: pallet.height + item.z};
            if (!record) {
                record = {holder: addBox(group, pose, geometrySize), key};
                records.set(item.id, record);
            } else {
                if (record.key !== key) {
                    models.updateCarton(record.holder.children[0],geometrySize);
                    record.key = key;
                }
                place(record.holder, pose);
                describeCarton(record.holder,item);
            }
            record.holder.children[0].scale.z = item.height/geometrySize.height;
        }
        for (const [id, {holder}] of records) {
            if (active.has(id)) continue;
            holder.removeFromParent();
            clearChildren(holder);
            records.delete(id);
        }
    }

    function visiblePallets() {
        const configured = state ?? (runtimeOwned ? [] : layout?.placedPallets ?? []);
        const live = warehousePallets(warehouse, layout, preview);
        const linked = new Set(layout?.objects.filter(object => object.type === 'rack')
            .flatMap(rack => rack.slots.filter(slot => slot.shelf_slot_id)
                .map(slot => JSON.stringify([rack.id, slot.id]))) ?? []);
        return [...live, ...configured.filter(pallet =>
            !live.some(object => object.id === pallet.id || (object.state === 'carried' && pallet.state === 'carried')) &&
            !linked.has(JSON.stringify([pallet.parent?.rack_id, pallet.parent?.slot_id])))];
    }

    function place(mesh, pose) {
        mesh.position.set(pose.x, pose.y, pose.z);
        mesh.rotation.z = pose.yaw;
    }

    const palletMeshes = new Map();
    let carriedPallets = [];
    function renderCargo() {
        for (const [id, surface] of slotSurfaces) {
            const state = warehouse?.slots?.[id];
            surface.userData.occupancy = state === 'full' || state === 'empty' ? state : 'unknown';
            surface.material = slotMaterials[surface.userData.occupancy];
        }
        cargoFootprints = [];
        cargoVolumes = [];
        carriedPallets = [];
        const active = new Set();
        fitIssues = [];
        for (const candidate of visiblePallets()) {
            const {pallet,issues} = fitRackPallet(candidate,layout);
            fitIssues.push(...issues);
            if (!pallet) continue;
            const carried = pallet.state === 'carried';
            const root = carried ? attachment : cargoRoot;
            active.add(pallet.id);
            let existing = palletMeshes.get(pallet.id);
            // A slow GLB load can delay attachment, without leaving the
            // confirmed load on the ground or discarding its existing mesh.
            if (!root) {
                existing?.mesh.removeFromParent();
                continue;
            }
            if (!existing) {
                const mesh = new THREE.Group();
                mesh.name = pallet.id;
                mesh.add(models.createPallet(pallet));
                existing = {mesh, boxes: new Map()};
                palletMeshes.set(pallet.id, existing);
            }
            const {mesh, boxes} = existing;
            mesh.userData.cargoType = pallet.cargoType;
            mesh.userData.cargoClass = pallet.cargoClass;
            mesh.userData.selected = pallet.selected === true;
            mesh.userData.locked = pallet.locked === true;
            mesh.userData.selectedFace = pallet.selectedFace ?? null;
            mesh.userData.fitIssues = issues;
            updateCartons(mesh, boxes, pallet);
            if (mesh.parent !== root) root.add(mesh);
            place(mesh, carried ? pallet.offset : pallet.pose);
            if (carried) {
                carriedPallets.push(pallet);
            } else {
                const parts = palletFootprints(pallet, pallet.pose);
                cargoFootprints.push(...parts);
                cargoVolumes.push(palletCloudVolume(parts));
            }
        }
        for (const [id, {mesh}] of palletMeshes) {
            if (active.has(id)) continue;
            mesh.removeFromParent();
            clearChildren(mesh);
            palletMeshes.delete(id);
        }
        const nextFitKey = JSON.stringify(fitIssues);
        if (fitIssues.length && nextFitKey !== fitKey) {
            console.warn('Warehouse fit conflicts', {mapId:layout?.map_id, issues:fitIssues});
        }
        fitKey = nextFitKey;
        onChange();
    }

    function palletFootprints(pallet, pose, carried = false) {
        return [{...pallet, ...pose, carried},
            ...pallet.boxes.map(item => ({...item, ...composePose(pose, item, pallet.height), carried}))];
    }

    function carriedFootprints(robotVisible) {
        if (!robotVisible || !attachment || !carriedPallets.length) return [];
        attachment.getWorldPosition(carriedPosition);
        attachment.getWorldQuaternion(carriedRotation);
        const {x, y, z} = carriedPosition;
        const yaw = carriedEuler.setFromQuaternion(carriedRotation, 'ZYX').z;
        return carriedPallets.map(pallet =>
            palletFootprints(pallet, composePose({x, y, z, yaw}, pallet.offset), true));
    }

    function getFootprints(robotVisible) {
        return [...staticFootprints, ...cargoFootprints, ...carriedFootprints(robotVisible).flat()];
    }

    function getCloudFootprints(robotVisible) {
        return [...staticFootprints, ...cargoVolumes, ...carriedFootprints(robotVisible).map(palletCloudVolume)];
    }

    function setLayout(next) {
        const sameScope = next && next.map_id === layout?.map_id && next.frame_id === layout.frame_id;
        clearChildren(staticRoot);
        slotSurfaces.clear();
        layout = next;
        if (sameScope) {
            state = stateMessage ? normalizeSemanticState(stateMessage,next) : null;
            stateKey = state ? JSON.stringify(state) : null;
        } else {
            warehouse = warehouseKey = null;
            warehouseAt = 0;
            preview = null;
            runtimeOwned = false;
            state = stateMessage = stateKey = null;
            lastStateAt = 0;
        }
        staticFootprints = [];
        for (const rack of layout?.objects || []) {
            if (rack.type === 'box') {
                addBox(staticRoot, rack);
                staticFootprints.push(rack);
            }
            if (rack.type !== 'rack') continue;
            rackMesh(rack);
            staticFootprints.push(rack);
            for (const slot of rack.slots) {
                for (const item of slot.boxes) {
                    const placed = {...item, ...composePose(slot, item)};
                    addBox(staticRoot, placed);
                    staticFootprints.push(placed);
                }
            }
        }
        renderCargo();
    }

    function setState(message, now = performance.now()) {
        if (!message) {
            stateMessage = null;
            if (!state) return false;
            state = null;
            stateKey = null;
            lastStateAt = 0;
            renderCargo();
            return false;
        }
        const normalized = normalizeSemanticState(message, layout);
        if (!normalized) return false;
        const key = JSON.stringify(normalized);
        runtimeOwned = true;
        stateMessage = message;
        lastStateAt = now;
        if (key === stateKey) return true;
        state = normalized;
        stateKey = key;
        renderCargo();
        return true;
    }

    function setRobotAttachment(next) {
        if (attachment === next) return;
        attachment = next;
        renderCargo();
    }

    function setWarehouse(message, now = performance.now()) {
        if (message && (message.map_id !== layout?.map_id || message.frame_id !== layout?.frame_id)) return false;
        warehouseAt = message ? now : 0;
        const key = message ? JSON.stringify(message) : null;
        if (warehouseKey === key) return true;
        warehouse = message;
        warehouseKey = key;
        preview = pickupPreview(preview,message?.target);
        renderCargo();
        return true;
    }

    function tick(now) {
        if (state && now - lastStateAt > 3000) setState(null);
        if (warehouse && now - warehouseAt > 3000) setWarehouse(null);
    }

    function dispose() {
        setLayout(null);
        scene.remove(staticRoot, cargoRoot);
        rackMaterial.dispose();
        models.dispose();
        slotMaterials.full.dispose();
        slotMaterials.unknown.dispose();
    }

    return {setLayout, setState, setWarehouse, setRobotAttachment, getFootprints, getCloudFootprints,
            getFitIssues: () => fitIssues,
            hasCarried: () => carriedPallets.length > 0, tick, dispose};
}
