import * as THREE from 'three';
import { GLTFLoader } from '../vendor/three/loaders/GLTFLoader.js';

const MODEL_URL = '/models/amr_description.glb';
let templatePromise = null;

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
    return {
        object: root,
        kind: 'fallback',
        reason: reason?.message || String(reason),
        liftNode: null,
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
        return {
            object: basis,
            kind: 'glb',
            reason: '',
            liftNode,
            liftBaseY: liftNode.position.y,
            dispose() { /* Cached GLB geometry/materials remain reusable in this tab. */ },
        };
    } catch (error) {
        return fallbackRobot(error);
    }
}
