import * as THREE from 'three';
import {composePose, normalizeSemanticState} from './semantic_layout.js?v=1.5.0';
import {fitRackPallet, palletCloudVolume, pickupPreview, rackConstruction, warehousePallets} from './warehouse_visuals.js?v=1.2.0';
import {cartonGeometryKey, createCargoModels} from './cargo_models.js?v=1.2.0';

// Geometry belongs to this layer; map, cloud and robot resources remain separate.
export function createSemanticScene(scene, onChange, hasMapPose = () => false) {
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
    let floorRelease = null;
    let releasedLoad = null;
    // Retain just the release window so a delayed event can select the pose
    // before backoff, even if the browser has already rendered a newer TF.
    let forkTrack = null;
    const releaseWindowMs = 3000;
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
        if (floorRelease) live.push(floorRelease.pallet);
        const linked = new Set(layout?.objects.filter(object => object.type === 'rack')
            .flatMap(rack => rack.slots.filter(slot => slot.shelf_slot_id)
                .map(slot => JSON.stringify([rack.id, slot.id]))) ?? []);
        return [...live, ...configured.filter(pallet =>
            !(releasedLoad?.id === pallet.id && (pallet.state === 'carried' ||
                releasedLoad.destination === 'D_2')) &&
            !live.some(object => object.id === pallet.id || (object.state === 'carried' && pallet.state === 'carried')) &&
            !linked.has(JSON.stringify([pallet.parent?.rack_id, pallet.parent?.slot_id])))];
    }

    function place(mesh, pose) {
        mesh.position.set(pose.x, pose.y, pose.z);
        mesh.rotation.z = pose.yaw;
    }

    const palletMeshes = new Map();
    let carriedPallets = [];
    let transfer = null;
    let shownLift = null;

    function forkPose() {
        attachment.getWorldPosition(carriedPosition);
        attachment.getWorldQuaternion(carriedRotation);
        const {x, y, z} = carriedPosition;
        const yaw = carriedEuler.setFromQuaternion(carriedRotation, 'ZYX').z;
        return {x, y, z, yaw};
    }

    function updateTransfer() {
        if (!transfer) return;
        const {pallet, mesh, motion, support} = transfer;
        const picking = motion.action === 'pick_backoff';
        mesh.visible = hasMapPose() || picking;
        if (!hasMapPose()) {
            transfer.pose = support;
        } else {
            const fork = composePose(forkPose(), pallet.offset);
            const h = shownLift ?? motion.height_m;
            const progress = THREE.MathUtils.clamp((h-motion.start_m)/(motion.goal_m-motion.start_m), 0, 1);
            const weight = picking ? progress : 1-progress;
            const angle = fork.yaw-support.yaw;
            // The deck rises on contact with the tines and rests on the shelf
            // once the tines descend below it. Both are in map coordinates.
            transfer.pose = {
                x: THREE.MathUtils.lerp(support.x, fork.x, weight),
                y: THREE.MathUtils.lerp(support.y, fork.y, weight),
                z: Math.max(support.z, fork.z),
                yaw: support.yaw + weight*Math.atan2(Math.sin(angle), Math.cos(angle)),
            };
        }
        place(mesh, transfer.pose);
    }
    function renderCargo() {
        for (const [id, surface] of slotSurfaces) {
            const state = warehouse?.slots?.[id];
            surface.userData.occupancy = state === 'full' || state === 'empty' ? state : 'unknown';
            surface.material = slotMaterials[surface.userData.occupancy];
        }
        cargoFootprints = [];
        cargoVolumes = [];
        carriedPallets = [];
        transfer = null;
        const active = new Set();
        fitIssues = [];
        for (const candidate of visiblePallets()) {
            const {pallet,issues} = fitRackPallet(candidate,layout);
            fitIssues.push(...issues);
            if (!pallet) continue;
            const carried = pallet.state === 'carried';
            const motion = warehouse?.lift_transfer;
            let support = null;
            if (attachment && motion && motion.id === pallet.id) {
                if (motion.action === 'pick_backoff' && pallet.selected) support = pallet.pose;
                if (motion.action === 'drop_release' && carried) {
                    support = layout.objects.filter(obj => obj.type === 'rack')
                        .flatMap(rack => rack.slots).find(slot => slot.shelf_slot_id === motion.slot);
                }
            }
            const root = carried && !support ? attachment : cargoRoot;
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
            mesh.userData.poseEstimated = pallet.poseEstimated === true;
            updateCartons(mesh, boxes, pallet);
            if (mesh.parent !== root) root.add(mesh);
            place(mesh, carried ? pallet.offset : pallet.pose);
            mesh.visible = true;
            if (support) {
                transfer = {pallet, mesh, support, motion};
                updateTransfer();
            } else if (carried) {
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
        const parts = [];
        if (transfer?.mesh.visible && (robotVisible || !hasMapPose() && transfer.pallet.selected)) {
            parts.push(palletFootprints(transfer.pallet, transfer.pose, true));
        }
        if (robotVisible && attachment && carriedPallets.length) {
            const fork = forkPose();
            parts.push(...carriedPallets.map(pallet =>
                palletFootprints(pallet, composePose(fork, pallet.offset), true)));
        }
        return parts;
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
            clearFloorRelease();
            preview = forkTrack = releasedLoad = null;
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
        const measured = floorRelease?.pallet.poseEstimated && normalized.find(pallet =>
            pallet.id === floorRelease.pallet.id && pallet.state === 'destination');
        if (measured) {
            floorRelease.pallet = {...floorRelease.pallet, pose:{...measured.pose,z:0}, poseEstimated:false};
        }
        if (key === stateKey && !measured) return true;
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

    function rememberForkPose(now) {
        const carried = warehouse?.carried;
        if (!carried?.run || !attachment || !hasMapPose()) return;
        const pallet = carriedPallets.find(item => item.id === carried.id);
        if (!pallet) return;
        const {x,y,z,yaw} = forkPose();
        if (![x,y,z,yaw].every(Number.isFinite)) return;
        if (!forkTrack || forkTrack.run !== carried.run || forkTrack.pallet.id !== pallet.id) {
            forkTrack = {run:carried.run, pallet, samples:[]};
        }
        forkTrack.pallet = pallet;
        const samples = forkTrack.samples;
        samples.push({at:now, pose:{x,y,z,yaw}});
        // Keep one predecessor at the edge of the window.
        while (samples.length > 1 && now - samples[1].at > releaseWindowMs) samples.shift();
    }

    function clearFloorRelease() {
        if (floorRelease) clearTimeout(floorRelease.timer);
        floorRelease = null;
    }

    function expireFloorRelease() {
        clearFloorRelease();
        renderCargo();
    }

    function reconcileRelease(message, now) {
        const release = message?.release;
        if (floorRelease && message && (release?.run !== floorRelease.run ||
                release?.id !== floorRelease.pallet.id || message.carried || message.target)) {
            clearFloorRelease();
        }
        const previousLoad = warehouse?.carried;
        let pallet = forkTrack?.pallet;
        if (!forkTrack || forkTrack.run !== release?.run) {
            pallet = previousLoad?.run === release?.run && carriedPallets.find(item => item.id === release?.id);
        }
        if (release?.destination === 'D_2' && !message.carried && pallet?.id === release.id) {
            rememberForkPose(performance.now());
            const at = now - 1000 * Math.max(0, message.release_age_s ?? 0);
            const deadline = at + releaseWindowMs;
            const reportedPose = release.pose ?? (lastStateAt >= at && state?.find(item =>
                item.id === pallet.id && item.state === 'destination')?.pose);
            const measured = reportedPose && ['x','y','yaw'].every(k => Number.isFinite(reportedPose[k]));
            const samples = forkTrack?.samples ?? [];
            // At zero event age the current attachment is the release transform.
            const sample = message.release_age_s > 0 ?
                samples.findLast(item => item.at <= at) : samples[samples.length-1];
            const pose = measured ? reportedPose : sample?.pose;
            if (pose && deadline > performance.now()) {
                floorRelease = {run:release.run, deadline, pallet:{...pallet, state:'destination',
                    parent:{type:'map'}, pose:{...pose,z:0}, poseEstimated:!measured,
                    exactCoverage:true, boxes:pallet.boxes.map(box => ({...box,suppressCloud:true}))}};
                floorRelease.timer = setTimeout(expireFloorRelease, deadline - performance.now());
            }
        }
        if (message && (message.carried?.id !== forkTrack?.pallet.id ||
                message.carried?.run !== forkTrack?.run)) forkTrack = null;
    }

    function setWarehouse(message, now = performance.now()) {
        if (message && (message.map_id !== layout?.map_id || message.frame_id !== layout?.frame_id)) return false;
        const carried = message?.carried;
        if (carried && releasedLoad?.id === carried.id && releasedLoad.run === (carried.run ?? null)) {
            return false;
        }
        warehouseAt = message ? now : 0;
        const key = message ? JSON.stringify({...message,release_age_s:undefined}) : null;
        if (warehouseKey === key) return true;
        if (message?.release) releasedLoad = {...message.release,run:message.release.run ?? null};
        else if (carried || message?.target) releasedLoad = null;
        reconcileRelease(message, now);
        warehouse = message;
        warehouseKey = key;
        preview = pickupPreview(preview,message?.target);
        renderCargo();
        rememberForkPose(performance.now());
        return true;
    }

    function tick(now, liftMeters = null) {
        shownLift = liftMeters;
        updateTransfer();
        if (floorRelease && now >= floorRelease.deadline) expireFloorRelease();
        if (!warehouse && forkTrack && now - forkTrack.samples.at(-1).at >= releaseWindowMs) forkTrack = null;
        rememberForkPose(now);
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
            hasCarried: () => carriedPallets.length > 0 || !!transfer, tick, dispose};
}
