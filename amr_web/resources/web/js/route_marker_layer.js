import * as THREE from 'three';

// Four flat strips keep the outline visible at oblique angles on WebGL.
export function createRouteMarkerLayer(scene, {side = 0.80, stroke = 0.024, z = 0.024, anisotropy = 1} = {}) {
    const root = new THREE.Group();
    root.name = 'Route node markers';
    scene.add(root);

    const vertices = [], indices = [];
    const h = side/2, k = h-stroke;
    for (const [x0,y0,x1,y1] of [[-h,-h,h,-k], [-h,k,h,h], [-h,-k,-k,k], [k,-k,h,k]]) {
        const i = vertices.length/3;
        vertices.push(x0,y0,0, x1,y0,0, x1,y1,0, x0,y1,0);
        indices.push(i,i+1,i+2,i,i+2,i+3);
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.Float32BufferAttribute(vertices, 3));
    geometry.setIndex(indices);

    const labels = new Map();
    const labelWidth = (side - 2*stroke) * 0.94;
    const labelGeometry = new THREE.PlaneGeometry(labelWidth, labelWidth / 4);
    const cameraRight = new THREE.Vector3();
    const cameraForward = new THREE.Vector3();
    const topDownThreshold = Math.cos(Math.PI / 9);
    let labelAngle = 0;
    let topDown = false;

    function createLabel(text) {
        const canvas = document.createElement('canvas');
        canvas.width = 512;
        canvas.height = 128;
        const ctx = canvas.getContext('2d');
        ctx.font = '700 134px Arial, sans-serif';
        ctx.textAlign = 'center';
        ctx.fillStyle = '#343a3d';
        const metrics = ctx.measureText(text);
        const baseline = (canvas.height + metrics.actualBoundingBoxAscent - metrics.actualBoundingBoxDescent)/2;
        ctx.fillText(text, canvas.width/2, baseline, canvas.width - 28);
        const texture = new THREE.CanvasTexture(canvas);
        texture.colorSpace = THREE.SRGBColorSpace;
        texture.minFilter = THREE.LinearMipmapLinearFilter;
        texture.anisotropy = anisotropy;
        const label = new THREE.Mesh(labelGeometry, new THREE.MeshBasicMaterial({
            map: texture, transparent: true, depthWrite: false, depthTest: !topDown,
        }));
        label.name = text;
        label.renderOrder = 1;
        label.rotation.z = labelAngle;
        labels.set(text, label);
        root.add(label);
        return label;
    }

    function faceCamera(camera) {
        if (!labels.size) return;
        cameraRight.set(1,0,0).applyQuaternion(camera.quaternion);
        if (Math.hypot(cameraRight.x, cameraRight.y) < 1e-6) return;
        const angle = Math.atan2(cameraRight.y, cameraRight.x);
        cameraForward.set(0,0,-1).applyQuaternion(camera.quaternion);
        const overhead = -cameraForward.z >= topDownThreshold;
        if (angle === labelAngle && overhead === topDown) return;
        labelAngle = angle;
        topDown = overhead;
        for (const label of labels.values()) {
            label.rotation.z = angle;
            // Keep floor labels readable beneath the robot in overhead views.
            label.material.depthTest = !topDown;
        }
    }

    let mesh = null;
    let capacity = 0;
    const transform = new THREE.Matrix4();
    const color = new THREE.Color();
    function update(markers) {
        for (const label of labels.values()) label.visible = false;
        if (markers.length > capacity) {
            mesh?.removeFromParent();
            mesh?.dispose();
            mesh?.material.dispose();
            capacity = 2 ** Math.ceil(Math.log2(markers.length));
            mesh = new THREE.InstancedMesh(geometry, new THREE.MeshBasicMaterial({}), capacity);
            mesh.instanceMatrix.setUsage(THREE.DynamicDrawUsage);
            root.add(mesh);
        }
        if (!mesh) return;
        mesh.count = markers.length;
        markers.forEach(({x, y, color: value, label: text}, index) => {
            mesh.setMatrixAt(index, transform.makeTranslation(x, y, z));
            mesh.setColorAt(index, color.set(value));
            if (text) {
                const label = labels.get(text) ?? createLabel(text);
                label.position.set(x, y, z + 0.004);
                label.visible = true;
            }
        });
        mesh.instanceMatrix.needsUpdate = true;
        if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true;
        mesh.computeBoundingSphere();
    }
    function dispose() {
        root.removeFromParent();
        mesh?.dispose();
        mesh?.material.dispose();
        geometry.dispose();
        labelGeometry.dispose();
        for (const label of labels.values()) {
            label.material.map.dispose();
            label.material.dispose();
        }
        labels.clear();
    }
    return {update, faceCamera, dispose};
}
