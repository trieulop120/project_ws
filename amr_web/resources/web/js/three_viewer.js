import {createRouteMarkerLayer} from './route_marker_layer.js?v=1.3.0';
import * as THREE from 'three';
import { OrbitControls } from '../vendor/three/OrbitControls.js';
import { buildWallSurfaceData, createCloudBoundary, createDisplayVariants, decodeMap, mapWorldCorners,
         normalizeMapInfo, occupiedFootprintCells, rackWallClipping, resolveObstacles } from './map_geometry.js?v=2.5.0';
import { normalizeSemanticLayout } from './semantic_layout.js?v=1.5.0';
import { createSemanticScene } from './semantic_scene.js?v=1.7.0';
import { createPointCloudLayer } from './point_cloud_layer.js?v=2.5.0';
import {createLaserObstacleLayer} from './laser_obstacle_layer.js';
import {laserCoverage} from './laser_obstacles.js?v=1.1.0';
import { createRobotModel } from './robot_model.js?v=1.2.0';

// Viewer owns only its WebGL resources. ROS data and the existing 2D canvas stay elsewhere.
export function createViewer(onError = () => {}, onModelStatus = () => {}, onFollowChange = () => {}) {
    let container = null;
    let scene = null;
    let camera = null;
    let renderer = null;
    let controls = null;
    let grid = null;
    let semanticScene = null;
    let routeMarkers = null;
    let cloudLayer = null;
    let laserLayer = null;
    let frame = null;
    let visible = false;
    let mapGroup = null;
    let variants = null;
    let perimeter = null;
    let wallMesh = null;
    let scanMesh = null;
    let scanMask = null;
    let scanKey = null;
    let lidarHeight = null;
    let cloudBoundary = null;
    let mapInfo = null;
    let mapData = null;
    let mapKey = null;
    let mapSourceKey = null;
    let activeMapId = null;
    let wallsVisible = true;
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
    let lastCargoRefresh = 0;
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
        cloudLayer?.tick(timestamp);
        semanticScene?.tick(timestamp);
        if (semanticScene?.hasCarried() && timestamp - lastCargoRefresh >= 120) {
            lastCargoRefresh = timestamp;
            refreshScanStrip();
            cloudLayer?.refresh();
        }
        laserLayer?.tick(timestamp);
        controls.update();
        routeMarkers?.faceCamera(camera);
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
        semanticScene?.setRobotAttachment(result.kind === 'glb' ? result.cargoAttachment : null);
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
            scene.background = new THREE.Color(0x1e282d);
            camera = new THREE.PerspectiveCamera(55, 1, 0.05, 1000);
            camera.up.set(0, 0, 1); // ROS: X forward, Y left, Z up.
            camera.position.copy(initialPosition);
            camera.lookAt(target);

            grid = new THREE.GridHelper(20, 20, 0x4d6970, 0x2d4148);
            grid.rotation.x = Math.PI / 2; // GridHelper's XZ plane -> ROS XY plane.
            scene.add(grid);
            scene.add(new THREE.HemisphereLight(0xe8f3f1, 0x708186, 1.1));
            const sunlight = new THREE.DirectionalLight(0xffffff, 1.25);
            sunlight.position.set(-3, -4, 8);
            scene.add(sunlight);
            robotRoot = new THREE.Group();
            robotRoot.name = 'digital_twin_base_footprint';
            robotRoot.visible = false;
            scene.add(robotRoot);
            routeMarkers = createRouteMarkerLayer(scene);
            semanticScene = createSemanticScene(scene, () => {
                refreshScanStrip();
                cloudLayer?.refresh();
            });
            cloudLayer = createPointCloudLayer(scene, reconcileObstacles, {
                acceptPoint: (point, sensorOrigin) =>
                    cloudBoundary?.allowsObservation(point, sensorOrigin) === true,
                canConnect: (a, b) => cloudBoundary?.canConnect(a, b) === true,
            });
            laserLayer = createLaserObstacleLayer(scene, getLaserCoverage);

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
        cloudLayer?.clear();
        laserLayer?.clear();
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
        // Follow ON orbits around the robot and zooms, while the target stays on the robot.
        controls.enablePan = !following;
        controls.enableRotate = true;
        controls.enableZoom = true;
        controls.enableDamping = true;
        controls.mouseButtons.LEFT = following ? THREE.MOUSE.ROTATE : THREE.MOUSE.PAN;
        controls.mouseButtons.RIGHT = THREE.MOUSE.ROTATE;
        controls.touches.ONE = following ? THREE.TOUCH.ROTATE : THREE.TOUCH.PAN;
        controls.touches.TWO = THREE.TOUCH.DOLLY_ROTATE;
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
        else if (robotModel?.cargoAttachment && Number.isFinite(robotModel.cargoBaseZ)) {
            robotModel.cargoAttachment.position.z = robotModel.cargoBaseZ + value;
        }
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
        routeMarkers?.update([]);
        if (mapGroup) {
            scene.remove(mapGroup);
            disposeMapGroup(mapGroup);
        }
        mapGroup = null;
        variants = null;
        perimeter = null;
        wallMesh = null;
        scanMesh = null;
        scanMask = null;
        scanKey = null;
        cloudBoundary = null;
        mapInfo = null;
        mapData = null;
        mapKey = null;
        mapSourceKey = null;
        activeMapId = null;
        setRobotPose(null);
        semanticScene?.setLayout(null);
        cloudLayer?.clear();
        laserLayer?.clear();
        if (grid) grid.visible = true;
    }

    function buildSurfaceMesh(surface, color, topColor = null) {
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(surface.positions, 3));
        geometry.setAttribute('normal', new THREE.BufferAttribute(surface.normals, 3));
        geometry.setIndex(new THREE.BufferAttribute(surface.indices, 1));
        if (topColor !== null) {
            const base = new THREE.Color(color), highlight = new THREE.Color(topColor);
            const colors = new Float32Array(surface.positions.length);
            let topZ = -Infinity;
            for (let index = 2; index < surface.positions.length; index += 3) {
                topZ = Math.max(topZ, surface.positions[index]);
            }
            for (let index = 0; index < surface.positions.length; index += 3) {
                const selected = surface.normals[index+2] > 0.99 &&
                    surface.positions[index+2] >= topZ - 1e-5 ? highlight : base;
                colors[index] = selected.r;
                colors[index+1] = selected.g;
                colors[index+2] = selected.b;
            }
            geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        }
        geometry.computeBoundingSphere();
        const mesh = new THREE.Mesh(geometry,
            new THREE.MeshLambertMaterial({color: topColor === null ? color : 0xffffff,
                                           vertexColors: topColor !== null,
                                           side: THREE.DoubleSide}));
        return mesh;
    }

    function refreshScanStrip() {
        if (!variants || !mapGroup) return;
        const interior = variants[displayMode].interior;
        const omitted = new Set();
        if (lidarHeight) {
            for (const object of semanticScene.getFootprints(robotRoot?.visible)) {
                if (object.type !== 'rack' && (object.z >= lidarHeight || object.z + object.height <= 0)) continue;
                for (const index of occupiedFootprintCells(interior, mapInfo, object)) omitted.add(index);
            }
        }
        const key = JSON.stringify([displayMode, lidarHeight, [...omitted].sort((a,b) => a-b)]);
        if (scanKey === key) return;
        scanKey = key;
        scanMask = null;
        laserLayer?.invalidate();
        if (scanMesh) {
            scanMesh.removeFromParent();
            disposeMapGroup(scanMesh);
            scanMesh = null;
        }
        if (!lidarHeight) return;
        const mask = interior.slice();
        for (const index of omitted) mask[index] = 0;
        const surface = buildWallSurfaceData(mask, mapInfo, [], lidarHeight);
        if (!surface.indices.length) return;
        scanMask = mask;
        scanMesh = buildSurfaceMesh(surface, 0x3f4e55);
        scanMesh.name = 'LiDAR footprint';
        mapGroup.add(scanMesh);
    }

    function setLidarHeight(height) {
        if (lidarHeight === height) return;
        lidarHeight = height;
        refreshScanStrip();
    }

    function reconcileObstacles(boxes, tolerance) {
        laserLayer?.invalidate();
        if (!perimeter) return [];
        return resolveObstacles(boxes.filter(cloudBoundary.allowsBox), perimeter.wall, mapInfo,
            semanticScene.getCloudFootprints(robotRoot?.visible), tolerance);
    }

    function getLaserCoverage() {
        if (!mapInfo || !cloudBoundary) return null;
        const models = [...semanticScene.getFootprints(robotRoot?.visible), ...cloudLayer.getVisibleBoxes()];
        return {
            covered: laserCoverage(mapInfo,perimeter.wall,scanMask,cloudBoundary,models),
            step: Math.max(.005,Math.min(.02,mapInfo.resolution/2)),
        };
    }

    function setRouteMarkers(mapId, markers) {
        if (activeMapId === mapId) routeMarkers?.update(markers);
    }

    function reportRackConflicts(mapId, conflicts) {
        for (const conflict of conflicts) {
            console.warn('3D perimeter wall clipped at a rack footprint. Verify its coordinates.',
                         {mapId, ...conflict});
        }
    }

    function setMap(map, semanticLayout = null) {
        if (!renderer) throw new Error('3D viewer is not initialized');
        if (!map) {
            clearMap();
            return false;
        }
        const info = normalizeMapInfo(map);
        const mapId = map.map_id ?? (map.is_pgm_raw ? map.frame_id : null);
        if (!mapId) throw new Error('Missing managed map ID');
        let layout = null;
        if (semanticLayout) {
            try {
                layout = normalizeSemanticLayout(semanticLayout, mapId, info.frame_id);
            } catch (error) {
                console.warn('3D semantic layout ignored:', error);
            }
        }
        const sourceKey = JSON.stringify([mapId, map.is_pgm_raw === true, info]);
        const key = JSON.stringify([sourceKey, layout]);
        if (key === mapKey && map.data === mapData) return false;
        const sameSource = sourceKey === mapSourceKey && map.data === mapData;
        if (sameSource) {
            const clipping = rackWallClipping(perimeter.wall, info, layout, perimeter.contours);
            const cutKey = cuts => JSON.stringify(cuts.map(({x,y,width,depth,yaw}) => [x,y,width,depth,yaw]));
            if (cutKey(clipping.cuts) !== cutKey(perimeter.cuts)) {
                const surface = buildWallSurfaceData(perimeter.wall, info, clipping.cuts, null, perimeter.contours);
                const replacement = surface.indices.length ? buildSurfaceMesh(surface, 0x6f8b90, 0x9bb4b8) : null;
                wallMesh?.removeFromParent();
                disposeMapGroup(wallMesh);
                wallMesh = replacement;
                if (wallMesh) {
                    wallMesh.visible = wallsVisible;
                    mapGroup.add(wallMesh);
                }
                perimeter.wallFaces = surface.faces;
            }
            Object.assign(perimeter, clipping);
            reportRackConflicts(mapId, perimeter.rackWallConflicts);
            mapKey = key;
            semanticScene?.setLayout(layout || {map_id: mapId, frame_id: info.frame_id, objects: []});
            return true;
        }
        if (info.width > renderer.capabilities.maxTextureSize ||
                info.height > renderer.capabilities.maxTextureSize) {
            throw new Error('Map exceeds this browser’s maximum texture size');
        }
        const displayData = createDisplayVariants(decodeMap(map, info), layout);
        const {raw, clean, cuts, ...nextPerimeter} = displayData;
        reportRackConflicts(mapId, nextPerimeter.rackWallConflicts);
        nextPerimeter.cuts = cuts;
        const nextGroup = new THREE.Group();
        const nextVariants = {};
        let nextWall = null;
        try {
            nextGroup.position.set(info.origin.x, info.origin.y, 0);
            nextGroup.rotation.z = info.origin.theta;
            const surface = buildWallSurfaceData(nextPerimeter.wall, info, cuts, null, nextPerimeter.contours);
            nextPerimeter.wallFaces = surface.faces;
            if (surface.indices.length) {
                nextWall = buildSurfaceMesh(surface, 0x6f8b90, 0x9bb4b8);
                nextWall.visible = wallsVisible;
                nextGroup.add(nextWall);
            }
            for (const [mode, data] of [['raw', raw], ['clean', clean]]) {
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

                nextVariants[mode] = {...data, floor};
            }
        } catch (error) {
            disposeMapGroup(nextGroup);
            throw error;
        }

        clearMap();
        mapGroup = nextGroup;
        variants = nextVariants;
        perimeter = nextPerimeter;
        wallMesh = nextWall;
        mapInfo = info;
        const cloudWall = nextPerimeter.wallEvidence.map((value, index) =>
            value || nextPerimeter.wall[index] ? 2 : 0);
        cloudBoundary = createCloudBoundary(info, nextPerimeter.outside, cloudWall);
        mapKey = key;
        mapSourceKey = sourceKey;
        activeMapId = mapId;
        mapData = map.data;
        scene.add(nextGroup);
        semanticScene?.setLayout(layout || {map_id: mapId, frame_id: info.frame_id, objects: []});
        grid.visible = false;
        mapBuildCount++;
        if (cameraMode !== 'follow') {
            fitToMap();
        }
        void ensureRobotModel();
        return true;
    }

    function setWallsVisible(value) {
        wallsVisible = value === true;
        if (wallMesh) wallMesh.visible = wallsVisible;
    }

    function setDisplayMode(value) {
        if (value !== 'raw' && value !== 'clean') throw new Error('Invalid 3D display mode');
        displayMode = value;
        if (variants) {
            for (const mode of ['raw', 'clean']) {
                variants[mode].floor.visible = mode === displayMode;
            }
        }
        refreshScanStrip();
    }

    function setPointCloud(message) {
        if (!activeMapId || message?.map_id !== activeMapId) return false;
        return cloudLayer?.update(message, mapInfo?.frame_id) || false;
    }

    function clearPointCloud() { cloudLayer?.clear(); }

    function setLaserScan(scan) {
        if (activeMapId && scan?.map_id === activeMapId) {
            laserLayer?.update(scan,mapInfo.frame_id,performance.now());
        }
    }

    function clearLaserScan() { laserLayer?.clear(); }

    function setSemanticState(message, at) {
        return semanticScene?.setState(message, at) || false;
    }

    function setWarehouse(message, at) {
        return semanticScene?.setWarehouse(message, at) || false;
    }

    function getMapSummary() {
        return {
            map_id: activeMapId,
            frame_id: mapInfo?.frame_id || null,
            width: mapInfo?.width || 0,
            height: mapInfo?.height || 0,
            resolution: mapInfo?.resolution || 0,
            origin: mapInfo?.origin || null,
            occupied: variants?.[displayMode].occupiedCells || 0,
            perimeter_cells: perimeter?.perimeterCells || 0,
            perimeter_bands: perimeter?.perimeterBands || 0,
            wall_faces: perimeter?.wallFaces || 0,
            rack_wall_conflicts: perimeter?.rackWallConflicts ?? [],
            warehouse_fit_issues: semanticScene?.getFitIssues() ?? [],
            bridged_wall_cells: perimeter?.bridgedWallCells || 0,
            raw_occupied: variants?.raw.occupiedCells || 0,
            clean_occupied: variants?.clean.occupiedCells || 0,
            filtered_cells: variants?.clean.removedCells || 0,
            filtered_components: variants?.clean.removedComponents || 0,
            clean_max_cells: variants?.clean.maxFilteredCells || 0,
            display_mode: displayMode,
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
            semanticScene?.setRobotAttachment(null);
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
        cloudLayer?.dispose();
        laserLayer?.dispose();
        semanticScene?.dispose();
        routeMarkers?.dispose();
        routeMarkers = null;
        cloudLayer = null;
        laserLayer = null;
        semanticScene = null;
        grid = null;
        scene = null;
        camera = null;
        container = null;
    }

    return { init, show, hide, resize, dispose,
             setMap, clearMap, setRouteMarkers, setWallsVisible, setDisplayMode, getMapSummary,
             setPointCloud, clearPointCloud, setLaserScan, clearLaserScan, setSemanticState, setWarehouse,
             setRobotPose, setRobotVisible, setLiftHeight, setLidarHeight,
             setFollowRobot, isFollowingRobot, setCameraMode };
}
