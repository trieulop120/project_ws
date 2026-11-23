import * as THREE from 'three';
import { OrbitControls } from '../vendor/three/OrbitControls.js';
import { buildWallSurfaceData, createDisplayVariants, decodeMap,
         mapWorldCorners, normalizeMapInfo } from './map_geometry.js?v=1.2.1';
import { createRobotModel } from './robot_model.js?v=1.0.1';

// Viewer owns only its WebGL resources. ROS data and the existing 2D canvas stay elsewhere.
export function createViewer(onError = () => {}, onModelStatus = () => {}, onFollowChange = () => {}) {
    let container = null;
    let scene = null;
    let camera = null;
    let renderer = null;
    let controls = null;
    let grid = null;
    let axes = null;
    let frame = null;
    let visible = false;
    let mapGroup = null;
    let variants = null;
    let mapInfo = null;
    let mapData = null;
    let mapKey = null;
    let wallsVisible = true;
    let axesVisible = true;
    let displayMode = 'clean';
    let mapBuildCount = 0;
    let robotRoot = null;
    let robotModel = null;
    let modelLoading = false;
    let modelGeneration = 0;
    let robotVisible = true;
    let targetPose = null;
    let displayedYaw = 0;
    let hasDisplayedPose = false;
    let liftHeightM = null;
    let cameraMode = 'free';
    let followPosition = null;
    let lastRenderTime = 0;
    let renderedFrames = 0;

    const initialPosition = new THREE.Vector3(8, -8, 8);
    const target = new THREE.Vector3(0, 0, 0);

    function renderFrame(timestamp) {
        frame = null;
        if (!visible || !renderer) return;
        frame = requestAnimationFrame(renderFrame);
        if (timestamp - lastRenderTime < 1000 / 30) return;
        const dt = Math.min(0.1, lastRenderTime ? (timestamp - lastRenderTime) / 1000 : 1 / 30);
        lastRenderTime = timestamp;
        updateRobot(dt);
        controls.update();
        renderer.render(scene, camera);
        renderedFrames++;
    }

    function updateRobot(dt) {
        if (!robotRoot || !robotModel || !targetPose || !robotVisible || !mapInfo) return;
        const factor = 1 - Math.exp(-dt / 0.12);
        if (!hasDisplayedPose) {
            robotRoot.position.set(targetPose.x, targetPose.y, 0);
            displayedYaw = targetPose.yaw;
            hasDisplayedPose = true;
        } else {
            robotRoot.position.x += (targetPose.x - robotRoot.position.x) * factor;
            robotRoot.position.y += (targetPose.y - robotRoot.position.y) * factor;
            const yawDelta = Math.atan2(Math.sin(targetPose.yaw - displayedYaw),
                                        Math.cos(targetPose.yaw - displayedYaw));
            displayedYaw += yawDelta * factor;
        }
        robotRoot.position.z = 0; // base_footprint is on the floor.
        robotRoot.rotation.z = displayedYaw;
        robotRoot.visible = true;
        if (cameraMode === 'follow' && controls && camera) {
            const x = robotRoot.position.x;
            const y = robotRoot.position.y;
            const z = 0.55;
            if (!followPosition) {
                // Place the camera behind the robot only ONCE on enabling Follow.
                // Subsequent updates translate the camera, preserving user zoom.
                const distance = 3.5;
                controls.target.set(x, y, z);
                camera.position.set(x - distance * Math.cos(displayedYaw),
                                    y - distance * Math.sin(displayedYaw), z + 2.7);
                followPosition = new THREE.Vector3(x, y, z);
            } else {
                const dx = x - followPosition.x;
                const dy = y - followPosition.y;
                const dz = z - followPosition.z;
                controls.target.x += dx;
                controls.target.y += dy;
                controls.target.z += dz;
                camera.position.x += dx;
                camera.position.y += dy;
                camera.position.z += dz;
                followPosition.set(x, y, z);
            }
        }
    }

    async function ensureRobotModel() {
        if (robotModel || modelLoading || !scene) return;
        modelLoading = true;
        const generation = modelGeneration;
        const result = await createRobotModel();
        if (generation !== modelGeneration || !scene || !robotRoot) {
            result.dispose();
            return;
        }
        modelLoading = false;
        robotModel = result;
        robotRoot.add(result.object);
        if (liftHeightM !== null) setLiftHeight(liftHeightM);
        onModelStatus({kind: result.kind, reason: result.reason});
    }

    function onContextLost(event) {
        event.preventDefault();
        onError(new Error('WebGL context lost'));
    }

    function resize() {
        if (!renderer || !container) return;
        const width = container.clientWidth;
        const height = container.clientHeight;
        if (width < 1 || height < 1) return;
        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height, false);
    }

    function init(element) {
        if (renderer) return;
        container = element;
        try {
            scene = new THREE.Scene();
            scene.background = new THREE.Color(0x17252b);
            camera = new THREE.PerspectiveCamera(55, 1, 0.05, 1000);
            camera.up.set(0, 0, 1); // ROS: X forward, Y left, Z up.
            camera.position.copy(initialPosition);
            camera.lookAt(target);

            grid = new THREE.GridHelper(20, 20, 0x4d6970, 0x2d4148);
            grid.rotation.x = Math.PI / 2; // GridHelper's XZ plane -> ROS XY plane.
            scene.add(grid);
            axes = new THREE.AxesHelper(2); // X red, Y green, Z blue.
            axes.visible = axesVisible;
            scene.add(axes);
            scene.add(new THREE.HemisphereLight(0xe8f3f1, 0x708186, 1.1));
            const sunlight = new THREE.DirectionalLight(0xffffff, 1.25);
            sunlight.position.set(-3, -4, 8);
            scene.add(sunlight);
            robotRoot = new THREE.Group();
            robotRoot.name = 'digital_twin_base_footprint';
            robotRoot.visible = false;
            scene.add(robotRoot);

            renderer = new THREE.WebGLRenderer({ antialias: true, powerPreference: 'low-power' });
            renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
            renderer.domElement.addEventListener('webglcontextlost', onContextLost);
            container.appendChild(renderer.domElement);
            controls = new OrbitControls(camera, renderer.domElement);
            controls.target.copy(target);
            controls.enableDamping = true;
            controls.mouseButtons.LEFT = THREE.MOUSE.PAN;
            controls.mouseButtons.MIDDLE = THREE.MOUSE.DOLLY;
            controls.mouseButtons.RIGHT = THREE.MOUSE.ROTATE;
            controls.touches.ONE = THREE.TOUCH.PAN;
            controls.touches.TWO = THREE.TOUCH.DOLLY_ROTATE;
            applyCameraInteraction();
            resize();
        } catch (error) {
            dispose();
            throw error;
        }
    }

    function show() {
        if (!renderer) throw new Error('3D viewer is not initialized');
        resize();
        if (visible) return;
        visible = true;
        lastRenderTime = 0;
        frame = requestAnimationFrame(renderFrame);
    }

    function hide() {
        visible = false;
        if (frame !== null) cancelAnimationFrame(frame);
        frame = null;
        lastRenderTime = 0;
    }

    function resetCamera() {
        if (!camera || !controls) return;
        // Reset/Fit are free-map actions, so they stop following the robot.
        if (cameraMode === 'follow') setCameraMode('free');
        if (mapInfo) {
            fitToMap();
            return;
        }
        camera.position.copy(initialPosition);
        camera.up.set(0, 0, 1);
        controls.target.copy(target);
        controls.update();
        if (!visible) renderer.render(scene, camera);
    }

    function fitToMap() {
        if (!mapInfo || !camera || !controls) return;
        if (cameraMode === 'follow') setCameraMode('free');
        resize();
        const corners = mapWorldCorners(mapInfo);
        const center = new THREE.Vector3(
            (corners[0].x + corners[2].x) / 2,
            (corners[0].y + corners[2].y) / 2,
            variants?.[displayMode].occupiedCells ? 1 : 0,
        );
        const radius = Math.hypot(mapInfo.width * mapInfo.resolution,
                                  mapInfo.height * mapInfo.resolution,
                                  variants?.[displayMode].occupiedCells ? 2 : 0) / 2;
        const verticalFov = THREE.MathUtils.degToRad(camera.fov);
        const horizontalFov = 2 * Math.atan(Math.tan(verticalFov / 2) * camera.aspect);
        const distance = Math.max(2, radius / Math.sin(Math.min(verticalFov, horizontalFov) / 2) * 1.08);
        const direction = new THREE.Vector3(0.7, -0.9, 1.8).normalize();
        controls.target.copy(center);
        camera.position.copy(center).addScaledVector(direction, distance);
        camera.near = Math.max(0.01, distance / 10000);
        camera.far = Math.max(1000, distance * 8);
        camera.updateProjectionMatrix();
        controls.update();
    }

    function applyCameraInteraction() {
        if (!controls) return;
        const following = cameraMode === 'follow';
        // Follow ON: wheel/pinch zoom only. Follow OFF: original pan/rotate/zoom.
        controls.enablePan = !following;
        controls.enableRotate = !following;
        controls.enableZoom = true;
        controls.enableDamping = !following;
        controls.touches.ONE = THREE.TOUCH.PAN;
        // With pan disabled, DOLLY_PAN becomes pinch-to-zoom only.
        controls.touches.TWO = following ? THREE.TOUCH.DOLLY_PAN : THREE.TOUCH.DOLLY_ROTATE;
        controls.update();
    }

    function setCameraMode(value) {
        if (value !== 'free' && value !== 'follow') throw new Error('Invalid camera mode');
        if (value === cameraMode) return;
        cameraMode = value;
        followPosition = null;
        if (camera) camera.up.set(0, 0, 1);
        applyCameraInteraction();
        if (value === 'follow' && targetPose && robotModel && mapInfo) {
            updateRobot(0);
            controls?.update();
        }
        onFollowChange(value === 'follow');
    }

    function setFollowRobot(enabled) {
        setCameraMode(enabled ? 'follow' : 'free');
    }

    function isFollowingRobot() {
        return cameraMode === 'follow';
    }

    function setRobotPose(pose) {
        if (!pose || !mapInfo) {
            targetPose = null;
            hasDisplayedPose = false;
            // Keep the camera offset (and zoom) through a temporary pose outage.
            // A map clear/change resets the tracking reference instead.
            if (!mapInfo) followPosition = null;
            if (robotRoot) robotRoot.visible = false;
            return;
        }
        if (![pose.x, pose.y, pose.yaw].every(Number.isFinite)) {
            targetPose = null;
            hasDisplayedPose = false;
            if (robotRoot) robotRoot.visible = false;
            return;
        }
        targetPose = {x: pose.x, y: pose.y, yaw: pose.yaw};
        if (!hasDisplayedPose) {
            displayedYaw = pose.yaw;
            if (robotRoot) {
                robotRoot.position.set(pose.x, pose.y, 0);
                robotRoot.rotation.z = pose.yaw;
            }
        }
    }

    function setRobotVisible(value) {
        robotVisible = value === true;
        if (robotRoot) robotRoot.visible = robotVisible && !!targetPose && !!robotModel && !!mapInfo;
    }

    function setLiftHeight(value) {
        if (!Number.isFinite(value) || value < 0 || value > 0.8) return;
        liftHeightM = value;
        if (robotModel?.liftNode) robotModel.liftNode.position.y = robotModel.liftBaseY + value;
    }

    function disposeMapGroup(group) {
        if (group) {
            group.traverse(object => {
                object.geometry?.dispose();
                const materials = Array.isArray(object.material) ? object.material : [object.material];
                for (const material of materials) {
                    material?.map?.dispose();
                    material?.dispose();
                }
            });
        }
    }

    function clearMap() {
        if (mapGroup) {
            scene.remove(mapGroup);
            disposeMapGroup(mapGroup);
        }
        mapGroup = null;
        variants = null;
        mapInfo = null;
        mapData = null;
        mapKey = null;
        setRobotPose(null);
        if (grid) grid.visible = true;
    }

    function setMap(map) {
        if (!renderer) throw new Error('3D viewer is not initialized');
        if (!map) {
            clearMap();
            return false;
        }
        const info = normalizeMapInfo(map);
        const key = JSON.stringify([map.map_id || map.frame_id, map.is_pgm_raw === true, info]);
        if (key === mapKey && map.data === mapData) return false;
        if (info.width > renderer.capabilities.maxTextureSize ||
                info.height > renderer.capabilities.maxTextureSize) {
            throw new Error('Map exceeds this browser’s maximum texture size');
        }
        const displayData = createDisplayVariants(decodeMap(map, info));
        const nextGroup = new THREE.Group();
        const nextVariants = {};
        try {
            nextGroup.position.set(info.origin.x, info.origin.y, 0);
            nextGroup.rotation.z = info.origin.theta;
            for (const mode of ['raw', 'clean']) {
                const data = displayData[mode];
                const image = document.createElement('canvas');
                image.width = info.width;
                image.height = info.height;
                image.getContext('2d').putImageData(
                    new ImageData(data.pixels, info.width, info.height), 0, 0);
                const texture = new THREE.CanvasTexture(image);
                texture.flipY = true; // Canvas top row corresponds to the map's highest Y row.
                texture.magFilter = THREE.NearestFilter;
                texture.minFilter = THREE.NearestFilter;
                texture.colorSpace = THREE.SRGBColorSpace;
                const floor = new THREE.Mesh(
                    new THREE.PlaneGeometry(info.width * info.resolution, info.height * info.resolution),
                    new THREE.MeshBasicMaterial({map: texture, side: THREE.DoubleSide,
                                                 transparent: true, depthWrite: false}),
                );
                floor.geometry.translate(info.width * info.resolution / 2,
                                         info.height * info.resolution / 2, 0);
                floor.visible = mode === displayMode;
                nextGroup.add(floor);

                // OccupancyGrid stores only XY occupancy; this 2 m height is display-only.
                const wallHeight = 2;
                let wallMesh = null;
                if (data.rectangles.length) {
                    const surface = buildWallSurfaceData(data.categories, info.width, info.height,
                                                         info.resolution, wallHeight, data.rectangles);
                    const geometry = new THREE.BufferGeometry();
                    geometry.setAttribute('position', new THREE.BufferAttribute(surface.positions, 3));
                    geometry.setAttribute('normal', new THREE.BufferAttribute(surface.normals, 3));
                    geometry.setIndex(new THREE.BufferAttribute(surface.indices, 1));
                    geometry.computeBoundingSphere();
                    wallMesh = new THREE.Mesh(
                        geometry,
                        new THREE.MeshLambertMaterial({color: 0x71989b, side: THREE.DoubleSide}),
                    );
                    wallMesh.visible = mode === displayMode && wallsVisible;
                    nextGroup.add(wallMesh);
                    data.wallQuads = surface.quads;
                }
                nextVariants[mode] = {floor, walls: wallMesh, ...data};
            }
        } catch (error) {
            disposeMapGroup(nextGroup);
            throw error;
        }

        clearMap();
        mapGroup = nextGroup;
        variants = nextVariants;
        mapInfo = info;
        mapKey = key;
        mapData = map.data;
        scene.add(nextGroup);
        grid.visible = false;
        mapBuildCount++;
        fitToMap();
        void ensureRobotModel();
        return true;
    }

    function setWallsVisible(value) {
        wallsVisible = value === true;
        if (variants) {
            for (const mode of ['raw', 'clean']) {
                if (variants[mode].walls) variants[mode].walls.visible =
                    mode === displayMode && wallsVisible;
            }
        }
    }

    function setDisplayMode(value) {
        if (value !== 'raw' && value !== 'clean') throw new Error('Invalid 3D display mode');
        displayMode = value;
        if (variants) {
            for (const mode of ['raw', 'clean']) {
                variants[mode].floor.visible = mode === displayMode;
                if (variants[mode].walls) variants[mode].walls.visible =
                    mode === displayMode && wallsVisible;
            }
        }
    }

    function setAxesVisible(value) {
        axesVisible = value === true;
        if (axes) axes.visible = axesVisible;
    }

    function getMapSummary() {
        return {
            frame_id: mapInfo?.frame_id || null,
            width: mapInfo?.width || 0,
            height: mapInfo?.height || 0,
            resolution: mapInfo?.resolution || 0,
            origin: mapInfo?.origin || null,
            occupied: variants?.[displayMode].occupiedCells || 0,
            rectangles: variants?.[displayMode].rectangles.length || 0,
            wall_quads: variants?.[displayMode].wallQuads || 0,
            raw_occupied: variants?.raw.occupiedCells || 0,
            clean_occupied: variants?.clean.occupiedCells || 0,
            filtered_cells: variants?.clean.removedCells || 0,
            filtered_components: variants?.clean.removedComponents || 0,
            clean_max_cells: variants?.clean.maxFilteredCells || 0,
            display_mode: displayMode,
            axes_visible: axesVisible,
            follow_robot: isFollowingRobot(),
            camera_target: controls ? {x: controls.target.x, y: controls.target.y, z: controls.target.z} : null,
            camera_position: camera ? {x: camera.position.x, y: camera.position.y, z: camera.position.z} : null,
            walls_visible: wallsVisible,
            build_count: mapBuildCount,
        };
    }

    function dispose() {
        hide();
        // Invalidate an in-flight asynchronous model load before tearing down scene.
        modelGeneration++;
        clearMap();
        if (robotModel) {
            robotRoot?.remove(robotModel.object);
            robotModel.dispose();
            robotModel = null;
        }
        modelLoading = false;
        robotRoot = null;
        followPosition = null;
        controls?.dispose();
        controls = null;
        if (renderer) {
            renderer.domElement.removeEventListener('webglcontextlost', onContextLost);
            renderer.dispose();
            renderer.forceContextLoss();
            renderer.domElement.remove();
        }
        renderer = null;
        grid?.geometry.dispose();
        grid?.material.dispose();
        axes?.geometry.dispose();
        axes?.material.dispose();
        grid = null;
        axes = null;
        scene = null;
        camera = null;
        container = null;
    }

    return { init, show, hide, resize, dispose, resetCamera, fitToMap,
             setMap, clearMap, setWallsVisible, setDisplayMode, setAxesVisible, getMapSummary,
             setRobotPose, setRobotVisible, setLiftHeight,
             setFollowRobot, isFollowingRobot, setCameraMode };
}
