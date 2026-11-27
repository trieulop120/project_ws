import * as THREE from 'three';

// Path centres are already in ROS map XY. Keep the ribbon outside mapGroup,
// whose origin and yaw apply only to the occupancy raster.
const HALF_WIDTH = .015;
const MAX_POINTS = 2000; // Shared plan_to_json limit.
const Z = .012;

export function createNavPlanLayer(scene) {
    // Per segment: two ribbon triangles, one bevel and at most one arrow.
    const positions = new THREE.BufferAttribute(new Float32Array(MAX_POINTS*12*3),3);
    positions.setUsage(THREE.DynamicDrawUsage);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position',positions);
    geometry.setDrawRange(0,0);
    const material = new THREE.MeshBasicMaterial({color:0x387bd7,side:THREE.DoubleSide});
    const mesh = new THREE.Mesh(geometry,material);
    mesh.name = 'Nav2 global plan';
    mesh.frustumCulled = false;
    mesh.visible = false;
    scene.add(mesh);
    let previous = null;

    function update(points) {
        if (!points || points.length<2 || points.length>MAX_POINTS) {
            previous = null;
            mesh.visible = false;
            geometry.setDrawRange(0,0);
            return;
        }
        if (previous === points) return;
        if (!points.every(p => Array.isArray(p) && p.length===2 && p.every(Number.isFinite))) {
            update(null);
            return;
        }
        if (points.length===previous?.length &&
                points.every(([x,y],i)=>x===previous[i][0] && y===previous[i][1])) {
            previous = points;
            return;
        }
        previous = points;
        let count=0, distance=0, nextArrow=.75;
        let lastX=0, lastY=0;
        const vertex=(x,y)=>positions.setXYZ(count++,x,y,Z);
        for (let i=1; i<points.length; i++) {
            const [x,y]=points[i-1], [bx,by]=points[i];
            const dx=bx-x, dy=by-y, length=Math.hypot(dx,dy);
            if (!length) continue;
            const nx=-dy/length*HALF_WIDTH, ny=dx/length*HALF_WIDTH;
            if (count) {
                const side = lastX*ny-lastY*nx > 0 ? -1 : 1;
                vertex(x,y); vertex(x+side*lastX,y+side*lastY); vertex(x+side*nx,y+side*ny);
            }
            vertex(x+nx,y+ny); vertex(x-nx,y-ny); vertex(bx+nx,by+ny);
            vertex(bx+nx,by+ny); vertex(x-nx,y-ny); vertex(bx-nx,by-ny);
            if (distance+length>=nextArrow) {
                const t=(nextArrow-distance)/length;
                const ax=x+t*dx, ay=y+t*dy;
                vertex(ax+.045*dx/length,ay+.045*dy/length);
                vertex(ax-.035*dx/length+2.6*nx,ay-.035*dy/length+2.6*ny);
                vertex(ax-.035*dx/length-2.6*nx,ay-.035*dy/length-2.6*ny);
                nextArrow=distance+length+1.25;
            }
            distance+=length;
            lastX=nx; lastY=ny;
        }
        geometry.setDrawRange(0,count);
        positions.needsUpdate=true;
        mesh.visible=count>0;
    }
    return {update,dispose() { mesh.removeFromParent(); geometry.dispose(); material.dispose(); }};
}
