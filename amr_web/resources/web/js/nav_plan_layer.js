import * as THREE from 'three';

// Path centres are already in ROS map XY. Keep the ribbon outside mapGroup,
// whose origin and yaw apply only to the occupancy raster.
const HALF_WIDTH = .045;
const MAX_POINTS = 2000; // Shared plan_to_json limit.
const Z = .012;

export function createNavPlanLayer(scene) {
    // Two ribbon triangles and at most one bevel per segment.
    const positions = new THREE.BufferAttribute(new Float32Array(MAX_POINTS*9*3),3);
    positions.setUsage(THREE.DynamicDrawUsage);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position',positions);
    const coords = new THREE.BufferAttribute(new Float32Array(MAX_POINTS*9*2),2);
    coords.setUsage(THREE.DynamicDrawUsage);
    geometry.setAttribute('uv',coords);
    geometry.setDrawRange(0,0);
    const reducedMotion = globalThis.matchMedia?.('(prefers-reduced-motion: reduce)');
    const material = new THREE.ShaderMaterial({
        side: THREE.DoubleSide, transparent: true, depthWrite: false,
        uniforms: {time: {value: 0}},
        vertexShader: `
            varying vec2 pathUV;
            void main() {
                pathUV = uv;
                gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.);
            }`,
        fragmentShader: `
            uniform float time;
            varying vec2 pathUV;
            void main() {
                float edge = abs(pathUV.y);
                float phase = (pathUV.x + edge * .055) / .23;
                float mark = 1. - smoothstep(.035, .12, abs(fract(phase+.5)-.5));
                float pulse = .78 + .22*cos(pathUV.x*5.-time*3.);
                vec3 green = mix(vec3(.012,.19,.055), vec3(.20,1.,.35)*pulse, mark);
                gl_FragColor = vec4(green, (.32 + .65 * mark)*(1.-smoothstep(.82,1.,edge)));
                #include <colorspace_fragment>
            }`,
    });
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
        let count=0, distance=0;
        let lastX=0, lastY=0;
        const vertex=(x,y,along,side)=>{
            positions.setXYZ(count,x,y,Z);
            coords.setXY(count++,along,side);
        };
        for (let i=1; i<points.length; i++) {
            const [x,y]=points[i-1], [bx,by]=points[i];
            const dx=bx-x, dy=by-y, length=Math.hypot(dx,dy);
            if (!length) continue;
            const nx=-dy/length*HALF_WIDTH, ny=dx/length*HALF_WIDTH;
            if (count) {
                const side = lastX*ny-lastY*nx > 0 ? -1 : 1;
                vertex(x,y,distance,0);
                vertex(x+side*lastX,y+side*lastY,distance,side);
                vertex(x+side*nx,y+side*ny,distance,side);
            }
            const end=distance+length;
            vertex(x+nx,y+ny,distance,1); vertex(x-nx,y-ny,distance,-1); vertex(bx+nx,by+ny,end,1);
            vertex(bx+nx,by+ny,end,1); vertex(x-nx,y-ny,distance,-1); vertex(bx-nx,by-ny,end,-1);
            distance=end;
            lastX=nx; lastY=ny;
        }
        geometry.setDrawRange(0,count);
        positions.needsUpdate=true;
        coords.needsUpdate=true;
        mesh.visible=count>0;
    }
    function tick(timestamp) {
        if (mesh.visible) material.uniforms.time.value = reducedMotion?.matches ? 0 : timestamp/1000;
    }
    return {update,tick,dispose() { mesh.removeFromParent(); geometry.dispose(); material.dispose(); }};
}
