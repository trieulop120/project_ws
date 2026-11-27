import * as THREE from 'three';
import { GLTFLoader } from '../vendor/three/loaders/GLTFLoader.js';
import {PALLET_DECK_FRACTION, PALLET_SIZE} from './warehouse_visuals.js?v=1.2.0';

const MODEL_URL = '/models/amr_description.glb';
let templatePromise = null;

function forkTopSurface(fork) {
    const planes = new Map();
    const a = new THREE.Vector3(), b = new THREE.Vector3(), c = new THREE.Vector3();
    fork.traverse(mesh => {
        const positions = mesh.geometry?.getAttribute('position');
        if (!positions) return;
        const indices = mesh.geometry.getIndex();
        const vertex = (point,index) => point.fromBufferAttribute(positions,indices?.getX(index) ?? index)
            .applyMatrix4(mesh.matrixWorld);
        for (let i=0; i+2<(indices?.count ?? positions.count); i+=3) {
            vertex(a,i); vertex(b,i+1); vertex(c,i+2);
            if (Math.max(a.z,b.z,c.z)-Math.min(a.z,b.z,c.z) > .0001) continue;
            const area = ((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x))/2;
            if (area <= 0) continue;
            const key = Math.round((a.z+b.z+c.z)/.0003);
            const plane = planes.get(key) ?? {area:0, height:0,
                minX:Infinity,maxX:-Infinity,minY:Infinity,maxY:-Infinity};
            plane.area += area;
            plane.height += area*(a.z+b.z+c.z)/3;
            plane.minX = Math.min(plane.minX,a.x,b.x,c.x);
            plane.maxX = Math.max(plane.maxX,a.x,b.x,c.x);
            plane.minY = Math.min(plane.minY,a.y,b.y,c.y);
            plane.maxY = Math.max(plane.maxY,a.y,b.y,c.y);
            planes.set(key,plane);
        }
    });
    let top = null;
    // The load surface is the dominant flat upward surface. The upper
    // end of the vertical bracket contributes only a small horizontal cap.
    for (const plane of planes.values()) if (!top || plane.area > top.area) top = plane;
    return top && {...top, z:top.height/top.area};
}

function fallbackRobot(reason) {
    // Dimensions and wheel centers come from amr_description STL/URDF at lift=0.
    const root = new THREE.Group();
    const bodyMaterial = new THREE.MeshStandardMaterial({color: 0x6e9da7, roughness: 0.8});
    const darkMaterial = new THREE.MeshStandardMaterial({color: 0x202b30, roughness: 0.95});
    const frontMaterial = new THREE.MeshStandardMaterial({color: 0x42d4b4, roughness: 0.7});
    function box(width, depth, height, x, y, z, material) {
        const mesh = new THREE.Mesh(new THREE.BoxGeometry(width, depth, height), material);
        mesh.position.set(x, y, z);
        root.add(mesh);
    }
    box(0.64, 0.48, 0.22, 0.007, 0, 0.20, bodyMaterial);
    box(0.035, 0.36, 1.0, -0.13, 0, 0.66, bodyMaterial);
    box(0.025, 0.35, 0.035, 0.33, 0, 0.23, frontMaterial); // +X front.
    for (const y of [-0.2451, 0.2451]) {
        const wheel = new THREE.Mesh(new THREE.CylinderGeometry(0.05, 0.05, 0.03, 20),
                                     darkMaterial);
        wheel.rotation.x = Math.PI / 2;
        wheel.position.set(0, y, 0.05);
        root.add(wheel);
    }
    const cargoAttachment = new THREE.Group();
    cargoAttachment.position.set(0.33, 0, 0.25); // fallback deck top, not a TF offset
    root.add(cargoAttachment);
    return {
        object: root,
        kind: 'fallback',
        reason: reason?.message || String(reason),
        liftNode: null,
        cargoAttachment,
        cargoBaseZ: cargoAttachment.position.z,
        dispose() {
            root.traverse(object => object.geometry?.dispose());
            for (const material of [bodyMaterial, darkMaterial, frontMaterial]) material.dispose();
        },
    };
}

export async function createRobotModel() {
    try {
        if (!templatePromise) {
            const loader = new GLTFLoader();
            templatePromise = loader.loadAsync(MODEL_URL).then(gltf => {
                if (!gltf.scene.getObjectByName('base_footprint') ||
                        !gltf.scene.getObjectByName('base_link') ||
                        !gltf.scene.getObjectByName('encoder_lift_link')) {
                    throw new Error('GLB is missing required URDF links');
                }
                return gltf.scene;
            }).catch(error => {
                // Failed network/parse requests must not poison the tab cache forever.
                templatePromise = null;
                throw error;
            });
        }
        const model = (await templatePromise).clone(true);
        const basis = new THREE.Group();
        basis.rotation.x = Math.PI / 2; // glTF Y-up -> ROS Z-up; +X stays forward.
        basis.add(model);
        const liftNode = model.getObjectByName('encoder_lift_link');
        let cargoAttachment = null;
        const leftFork = model.getObjectByName('left_fork_link');
        const rightFork = model.getObjectByName('right_fork_link');
        if (leftFork && rightFork) {
            basis.updateMatrixWorld(true);
            const left = forkTopSurface(leftFork), right = forkTopSurface(rightFork);
            const rear = Math.max(left?.minX ?? Infinity,right?.minX ?? Infinity);
            if (left && right && Math.abs(left.z-right.z) <= .002 &&
                    rear < Math.min(left.maxX,right.maxX)) {
                // Place the rear of the pallet at the start of both tines.
                // Their top surfaces meet the underside of its deck boards.
                const underDeck = PALLET_SIZE.height*(1-PALLET_DECK_FRACTION);
                const bottomCenter = new THREE.Vector3(rear+PALLET_SIZE.width/2,
                    (left.minY+left.maxY+right.minY+right.maxY)/4,
                    Math.max(left.z,right.z)-underDeck);
                cargoAttachment = new THREE.Group();
                cargoAttachment.position.copy(liftNode.worldToLocal(bottomCenter));
                cargoAttachment.quaternion.copy(liftNode.getWorldQuaternion(new THREE.Quaternion()).invert());
                liftNode.add(cargoAttachment);
            }
        }
        return {
            object: basis,
            kind: 'glb',
            reason: '',
            liftNode,
            cargoAttachment,
            liftBaseY: liftNode.position.y,
            dispose() { /* Cached GLB geometry/materials remain reusable in this tab. */ },
        };
    } catch (error) {
        return fallbackRobot(error);
    }
}
