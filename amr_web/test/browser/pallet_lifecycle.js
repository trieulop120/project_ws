// Run as an ES module in the web viewer's Three.js import map.
import * as THREE from 'three';
import {createSemanticScene} from '/js/semantic_scene.js';

export async function testPalletLifecycle() {
    const assert = (condition, message) => { if (!condition) throw new Error(message); };
    const scene = new THREE.Scene();
    const forks = new THREE.Group();
    scene.add(forks);
    const layer = createSemanticScene(scene, () => {}, () => true);
    const scope = {map_id:'test',frame_id:'map'};
    const carried = {id:'test_load',run:'retrieval-1',slot:'A1',height_m:.24};
    const release = {id:carried.id,run:carried.run,destination:'D_2'};
    const load = () => scene.getObjectByName(carried.id);
    const checkFootprints = count => {
        assert(layer.getFootprints(true).length === count * 2,'pallet and carton footprints');
        assert(layer.getCloudFootprints(true).length === count,'cloud volume');
    };
    try {
        layer.setLayout({...scope,objects:[]});
        layer.setRobotAttachment(forks);
        forks.position.set(2,3,.1);
        forks.rotation.z = .7;
        layer.setWarehouse({...scope,carried});
        const original = load();
        const vertices = original.children[0].geometry.getAttribute('position');
        // Each procedural box contributes 24 vertices. Verify the individual
        // parts in pallet coordinates before the fork/map yaw is applied.
        const extents = [];
        for (let start=0; start<vertices.count; start+=24) {
            const bounds = new THREE.Box3();
            const point = new THREE.Vector3();
            for (let i=start; i<start+24; i++) bounds.expandByPoint(point.fromBufferAttribute(vertices,i));
            extents.push(bounds);
        }
        const near = (a,b) => Math.abs(a-b)<1e-6;
        const span = (box,axis) => box.max[axis]-box.min[axis];
        const runners = extents.filter(box => near(span(box,'x'),.30));
        const boards = extents.filter(box => near(span(box,'y'),.40));
        assert(runners.length===3 && boards.length===5 && extents.length===17,'runners, boards and support blocks');
        for (const y of [-.085,.085]) {
            const channels = extents.filter(box => box.min.z<.037 && box.min.y<y && y<box.max.y);
            assert(channels.length===0,'fork entry obstructed along local X');
        }
        const [footprint, carton] = layer.getFootprints(true);
        assert(near(footprint.width,.30) && near(footprint.depth,.40) && near(footprint.height,.05) &&
            near(footprint.yaw,.7),'suppression footprint must use pallet-local X/Y at fork yaw');
        assert(near(carton.x,footprint.x) && near(carton.y,footprint.y) &&
            near(carton.z,footprint.z+.05),'carton centred on the deck');
        // Simulate receipt after the robot has already started backing away.
        const at = performance.now();
        layer.tick(at);
        await new Promise(resolve => setTimeout(resolve, 40));
        forks.position.x = 1;
        const delivered = performance.now();
        layer.tick(delivered);
        layer.setWarehouse({...scope,release,release_age_s:(delivered-at)/1000},delivered);
        assert(load() === original && original.parent !== forks && original.userData.poseEstimated,
            'release must transfer the same mesh');
        assert(original.position.x === 2 && original.position.y === 3 && original.position.z === 0 &&
            Math.abs(original.rotation.z-.7)<1e-6,'release used a post-backoff transform');
        checkFootprints(1);
        layer.setWarehouse(null);
        layer.setWarehouse({...scope,release,release_age_s:2},at + 2000);
        forks.position.set(1,2,.5);
        assert(!layer.setWarehouse({...scope,carried},at + 2200),'accepted stale carrying heartbeat');
        layer.tick(at + 2999);
        assert(load() === original && original.position.x === 2,'disconnect/reconnect shortened the deadline');
        layer.tick(at + 3000);
        checkFootprints(0);
        layer.setWarehouse({...scope,release,release_age_s:4},at + 4000);
        assert(!load(),'heartbeat rearmed an expired release');

        carried.run = release.run = 'second-retrieval';
        layer.setWarehouse({...scope,carried});
        layer.setWarehouse({...scope,release:{...release,pose:{x:4,y:5,yaw:.2}}});
        assert(load()?.position.x === 4 && !load().userData.poseEstimated,'measured release pose');
        layer.setWarehouse({...scope,carried:{...carried,id:'new-load',run:'retrieval-2'}});
        assert(!load() && layer.hasCarried(),'new mission retained previous release');
        layer.setLayout({...scope,map_id:'other',objects:[]});
        checkFootprints(0);
    } finally {
        layer.dispose();
    }
}
