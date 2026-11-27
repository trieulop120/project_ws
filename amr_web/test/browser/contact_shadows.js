// Run in Firefox with the viewer's import map. Read actual fragment output
// across an unknown cell, a rotated map edge and an opaque marker outline.
import * as THREE from 'three';
import {createContactShadows} from '/js/contact_shadows.js';
import {createRouteMarkerLayer} from '/js/route_marker_layer.js';

export function testContactShadows(renderer) {
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0xffffff);
    const robot = new THREE.Group();
    scene.add(robot);
    const shadows = createContactShadows(scene, robot);
    const markers = createRouteMarkerLayer(scene);
    const floor = new THREE.DataTexture(new Uint8Array([
        255,255,255,255, 100,100,100,28,
        100,100,100,0, 255,255,255,255,
    ]), 2, 2);
    floor.needsUpdate = true;
    const origin = {x: 4, y: -3, theta: Math.PI/2};
    shadows.setFloor({origin, width: 2, height: 2, resolution: 1}, floor);
    const rack = {type:'rack', x:3.5, y:-2.5, z:0, yaw:.3, width:5, depth:5};
    shadows.setSemantic([rack]);
    const camera = new THREE.OrthographicCamera(-.01,.01,.01,-.01,.1,5);
    const target = new THREE.WebGLRenderTarget(4,4);
    const previous = renderer.getRenderTarget();
    function sample(x,y) {
        camera.position.set(x,y,2);
        camera.lookAt(x,y,0);
        renderer.setRenderTarget(target);
        renderer.render(scene,camera);
        const pixel = new Uint8Array(4);
        renderer.readRenderTargetPixels(target,2,2,1,1,pixel);
        return pixel.slice(0,3).join(',');
    }
    const assert = (ok, message) => { if (!ok) throw new Error(message); };
    try {
        assert(sample(3.5,-2.5) !== '255,255,255', 'missing contact on surveyed floor');
        for (const [x,y] of [[3.5,-1.5],[2.5,-2.5],[4.2,-2.5]]) {
            assert(sample(x,y) === '255,255,255','shadow outside valid floor');
        }
        markers.update([{x:3.5,y:-2.5,color:'#ff7700',label:'HOME'}]);
        const outline = scene.getObjectByName('Route node markers').children.find(mesh => mesh.isInstancedMesh);
        outline.geometry.computeBoundingBox();
        const size = outline.geometry.boundingBox.getSize(new THREE.Vector3());
        assert(Math.abs(size.x-.80) < 1e-6 && Math.abs(size.y-.80) < 1e-6, 'marker must be 0.80 m square');
        const edge = sample(3.89,-2.5);
        shadows.setSemantic([]);
        assert(edge === sample(3.89,-2.5) && edge !== '255,255,255','shadow obscured route outline');
        assert(sample(3.5,-2.38) === '255,255,255','shadow remained after removal');
    } finally {
        renderer.setRenderTarget(previous);
        markers.dispose();
        shadows.dispose();
        target.dispose();
        floor.dispose();
    }
}
