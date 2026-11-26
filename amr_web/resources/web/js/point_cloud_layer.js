import * as THREE from 'three';
import {createCloudTracker} from './cloud_obstacles.js?v=1.2.0';

export function createPointCloudLayer(scene, resolve, options) {
    const tracker = createCloudTracker(options);
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const material = new THREE.MeshLambertMaterial({color: 0x5b6b73});
    const meshes = new Map();
    let observed = [];
    let visibleBoxes = [];

    function refresh() {
        const boxes = resolve(observed, tracker.config.tolerance);
        visibleBoxes = boxes;
        const active = new Set(boxes.map(box => box.id));
        for (const [id, mesh] of meshes) {
            if (active.has(id)) continue;
            scene.remove(mesh);
            meshes.delete(id);
        }
        for (const box of boxes) {
            let mesh = meshes.get(box.id);
            if (!mesh) {
                mesh = new THREE.Mesh(geometry, material);
                mesh.name = 'Estimated obstacle';
                meshes.set(box.id, mesh);
                scene.add(mesh);
            }
            mesh.position.set(box.x, box.y, box.z + box.height / 2);
            mesh.scale.set(box.width, box.depth, box.height);
        }
    }

    function update(message, expectedFrame, now = performance.now()) {
        return tracker.update(message, expectedFrame, now);
    }

    function tick(now) {
        const next = tracker.tick(now);
        if (!next) return;
        observed = next;
        refresh();
    }

    function clear() {
        tracker.clear();
        observed = [];
        refresh();
    }

    function dispose() {
        clear();
        geometry.dispose();
        material.dispose();
    }

    return {update, tick, refresh, clear, dispose, getVisibleBoxes: () => visibleBoxes};
}
