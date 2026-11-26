import * as THREE from 'three';
import {clipLaserStrips, createLaserTracker, LASER_MAX_STRIPS, LASER_UPDATE_MS} from './laser_obstacles.js';

export function createLaserObstacleLayer(scene, coverage) {
    const tracker = createLaserTracker();
    const geometry = new THREE.BoxGeometry(1,1,1);
    const material = new THREE.MeshLambertMaterial({color:0x3f4e55});
    const mesh = new THREE.InstancedMesh(geometry,material,LASER_MAX_STRIPS);
    mesh.name = 'LaserScan obstacles';
    mesh.count = 0;
    mesh.visible = false;
    mesh.frustumCulled = false;
    mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
    scene.add(mesh);
    const transform = new THREE.Object3D();
    let pending = null;
    let updatedAt = -Infinity;
    let dirty = false;

    return {
        update(scan, frame, now) {
            if (scan?.scan_pose?.frame_id === frame) pending = {scan,frame,now};
        },
        invalidate() { dirty = true; },
        tick(now) {
            if ((!pending && !dirty && !mesh.count) || now-updatedAt < LASER_UPDATE_MS) return;
            updatedAt = now;
            if (pending) {
                const next = pending;
                pending = null;
                if (now-next.now < 250) dirty = tracker.update(next.scan,next.frame,next.now) || dirty;
            }
            dirty = tracker.expire(now) || dirty;
            if (!dirty) return;
            dirty = false;
            const mask = coverage();
            const strips = mask ? clipLaserStrips(tracker.confirmed(),mask.covered,mask.step) : [];
            strips.forEach(({a,b,height},i) => {
                const dx = b[0]-a[0], dy = b[1]-a[1];
                transform.position.set((a[0]+b[0])/2,(a[1]+b[1])/2,.002+height/2);
                transform.rotation.z = Math.atan2(dy,dx);
                transform.scale.set(Math.hypot(dx,dy),.035,height);
                transform.updateMatrix();
                mesh.setMatrixAt(i,transform.matrix);
            });
            mesh.count = strips.length;
            mesh.visible = mesh.count > 0;
            mesh.instanceMatrix.needsUpdate = true;
        },
        clear() {
            tracker.clear();
            pending = null;
            dirty = false;
            mesh.count = 0;
            mesh.visible = false;
            updatedAt = -Infinity;
        },
        dispose() {
            mesh.removeFromParent();
            mesh.dispose();
            geometry.dispose();
            material.dispose();
        },
    };
}
