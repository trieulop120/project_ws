import * as THREE from 'three';

// A shared soft footprint evaluated on a unit quad. Sample the cached floor at
// cell centres so transparent exterior and unknown cells cannot receive shadows.
export function createContactShadows(scene, robotRoot) {
    const geometry = new THREE.PlaneGeometry(1, 1);
    const uniforms = {
        floorMap: {value: null},
        worldToGrid: {value: new THREE.Matrix3()},
        gridSize: {value: new THREE.Vector2()},
    };
    const material = new THREE.ShaderMaterial({
        uniforms, transparent: true, depthWrite: false,
        vertexShader: `
            uniform mat3 worldToGrid;
            varying vec2 cell, footprint;
            void main() {
                vec4 p = vec4(position, 1.0);
                #ifdef USE_INSTANCING
                    p = instanceMatrix * p;
                #endif
                p = modelMatrix * p;
                cell = (worldToGrid * vec3(p.xy, 1.0)).xy;
                footprint = 2.0 * uv - 1.0;
                gl_Position = projectionMatrix * viewMatrix * p;
            }`,
        fragmentShader: `
            uniform sampler2D floorMap;
            uniform vec2 gridSize;
            varying vec2 cell, footprint;
            void main() {
                if (any(lessThan(cell, vec2(0.0))) || any(greaterThanEqual(cell, gridSize))) discard;
                vec2 uv = (floor(cell) + 0.5) / gridSize;
                if (texture2D(floorMap, uv, -16.0).a < 0.999) discard;
                float edge = length(max(abs(footprint) - 0.53, 0.0));
                float alpha = 0.20 * (1.0 - smoothstep(0.06, 0.47, edge));
                gl_FragColor = vec4(0.0, 0.0, 0.0, alpha);
            }`,
    });
    const robot = new THREE.Mesh(geometry, material);
    robot.name = 'Robot contact shadow';
    robot.renderOrder = -1;
    robot.visible = false;
    robotRoot.add(robot);
    const batches = new Map();
    const transform = new THREE.Object3D();

    function footprint(object, x, y, width, depth, yaw) {
        object.position.set(x, y, .003);
        object.rotation.z = yaw;
        object.scale.set(width * 1.32, depth * 1.32, 1);
    }

    function setRobotModel(model) {
        // Measure once in base_footprint coordinates, before attaching to the
        // moving robot or adding cargo. Include the offset of the GLB bounds.
        const {min, max} = new THREE.Box3().setFromObject(model);
        footprint(robot, (min.x+max.x)/2, (min.y+max.y)/2, max.x-min.x, max.y-min.y, 0);
        robot.visible = true;
    }

    function updateBatch(name, items) {
        let mesh = batches.get(name);
        if (items.length > (mesh?.instanceMatrix.count ?? 0)) {
            mesh?.removeFromParent();
            mesh?.dispose();
            mesh = new THREE.InstancedMesh(geometry, material, 2 ** Math.ceil(Math.log2(items.length)));
            mesh.name = name;
            mesh.renderOrder = -1;
            mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
            batches.set(name, mesh);
            scene.add(mesh);
        }
        if (!mesh) return;
        mesh.count = items.length;
        mesh.visible = mesh.count > 0;
        if (!mesh.visible) return;
        items.forEach(({x, y, width, depth, yaw = 0}, i) => {
            footprint(transform, x, y, width, depth, yaw);
            transform.updateMatrix();
            mesh.setMatrixAt(i, transform.matrix);
        });
        mesh.instanceMatrix.needsUpdate = true;
        mesh.computeBoundingSphere();
    }

    function setFloor(info, texture) {
        uniforms.floorMap.value = texture;
        if (!info) return;
        const {x, y, theta} = info.origin;
        const c = Math.cos(theta) / info.resolution, s = Math.sin(theta) / info.resolution;
        uniforms.worldToGrid.value.set(c, s, -c*x-s*y, -s, c, s*x-c*y, 0, 0, 1);
        uniforms.gridSize.value.set(info.width, info.height);
    }

    function setSemantic(items) {
        updateBatch('Semantic contact shadows', items.filter(item => Math.abs(item.z) <= .025 &&
            (item.type === 'rack' || item.parent?.type === 'map')));
    }

    function setObstacles(boxes) {
        // Cloud boxes begin at measured points above the floor (sensor minZ
        // is 0.12 m). Limit shadows to substantial, low-reaching volumes.
        updateBatch('Obstacle contact shadows', boxes.filter(box =>
            box.z <= .18 && Math.min(box.width, box.depth) >= .22 && box.height >= .25));
    }

    function dispose() {
        for (const mesh of batches.values()) {
            mesh.removeFromParent();
            mesh.dispose();
        }
        batches.clear();
        robot.removeFromParent();
        uniforms.floorMap.value = null;
        geometry.dispose();
        material.dispose();
    }
    return {setRobotModel, setFloor, setSemantic, setObstacles, dispose};
}
