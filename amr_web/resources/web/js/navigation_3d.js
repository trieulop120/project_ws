// Navigation-only switch. The Three.js module and vendor assets load on first 3D use.
(function () {
    'use strict';

    let layer, button2D, button3D, axesButton, axesLegend, fitButton, wallsButton,
        rawButton, cleanButton, status, viewport, followButton, poseReadout;
    let viewer = null;
    let mode = '2d';
    let request = 0;
    let loading = false;
    let observedMapId = null;
    let displayedMapId = null;
    let loadingMapId = null;
    let failedMapId = null;
    let failedReason = '';
    let wallsVisible = true;
    let axesVisible = true;
    let displayMode = 'clean';
    let lastPose = null;
    let lastPoseArrival = 0;
    let transportConnected = false;
    let poseMonitor = null;
    let latestLiftMeters = null;
    let modelKind = 'loading';
    const POSE_TIMEOUT_MS = 3000;

    const NO_MAP_MESSAGE = 'Vui lòng chọn bản đồ trong Navigation';

    function setStatus(message) {
        if (!status) return;
        status.textContent = message;
        status.hidden = !message;
    }

    function updateFollowButton(enabled = viewer?.isFollowingRobot() === true) {
        if (!followButton) return;
        followButton.textContent = enabled ? 'Follow Robot: ON' : 'Follow Robot: OFF';
        followButton.setAttribute('aria-pressed', String(enabled));
        followButton.classList.toggle('active', enabled);
        const hint = document.querySelector('#navThreeLayer .nav-three-hint');
        if (hint) hint.textContent = enabled
            ? 'Follow Robot: scroll or pinch to zoom'
            : 'Drag to pan · Right drag to rotate · Scroll to zoom';
    }

    function createRobotControls() {
        // The original HTML/CSS was not supplied. Inject next to the existing
        // Fit Map button, preserving the toolbar's native layout and class names.
        followButton = document.getElementById('navThreeFollow');
        if (!followButton && fitButton?.parentElement) {
            followButton = document.createElement('button');
            followButton.id = 'navThreeFollow';
            followButton.type = 'button';
            followButton.className = fitButton.className;
            followButton.title = 'ON: zoom only. OFF: pan, rotate and zoom.';
            fitButton.insertAdjacentElement('afterend', followButton);
        }
        followButton?.addEventListener('click', () => {
            if (!viewer || mode !== '3d') return;
            viewer.setFollowRobot(!viewer.isFollowingRobot());
            updateFollowButton();
        });
        updateFollowButton(false);

        poseReadout = document.getElementById('navThreeRobotTelemetry');
        if (!poseReadout && layer) {
            poseReadout = document.createElement('div');
            poseReadout.id = 'navThreeRobotTelemetry';
            poseReadout.setAttribute('role', 'status');
            poseReadout.setAttribute('aria-live', 'off');
            Object.assign(poseReadout.style, {
                position: 'absolute', right: '10px', bottom: '10px', zIndex: '6',
                pointerEvents: 'none', padding: '6px 9px', borderRadius: '6px',
                background: 'rgba(15, 28, 34, .8)', color: '#e8f3f1',
                fontSize: '12px', lineHeight: '1.4', fontFamily: 'monospace',
                whiteSpace: 'pre-line'
            });
            layer.appendChild(poseReadout);
        }
        updatePoseReadout('Waiting for pose');
    }

    function updatePoseReadout(label, pose = null) {
        if (!poseReadout) return;
        const model = modelKind === 'loading' ? 'Loading model' :
            (modelKind === 'fallback' ? 'Fallback model' : 'AMR GLB');
        poseReadout.textContent = pose
            ? `${label} | ${model}\nX ${pose.x.toFixed(3)} m  Y ${pose.y.toFixed(3)} m  Yaw ${(pose.yaw * 180 / Math.PI).toFixed(1)}°`
            : `${label} | ${model}`;
    }

    function refreshRobotPose() {
        if (!viewer || mode !== '3d') return;
        const mapFrame = viewer.getMapSummary()?.frame_id;
        if (!displayedMapId || !mapFrame) {
            viewer.setRobotPose(null);
            updatePoseReadout('Waiting for selected map');
            return;
        }
        const age = Date.now() - lastPoseArrival;
        const valid = transportConnected &&
            lastPose && lastPose.stale !== true &&
            age >= 0 && age < POSE_TIMEOUT_MS &&
            lastPose.map_id === displayedMapId &&
            lastPose.frame_id === mapFrame &&
            lastPose.base_frame_id === 'base_footprint' &&
            [lastPose.position?.x, lastPose.position?.y,
             lastPose.orientation?.yaw].every(Number.isFinite);

        if (valid) {
            const pose = {
                x: lastPose.position.x, y: lastPose.position.y,
                yaw: lastPose.orientation.yaw
            };
            viewer.setRobotPose(pose);
            updatePoseReadout('Live', pose);
        } else {
            viewer.setRobotPose(null);
            const label = !transportConnected ? 'Disconnected' :
                (!lastPose || age >= POSE_TIMEOUT_MS || lastPose.stale === true)
                    ? 'Pose unavailable/stale' : 'Waiting for map-frame pose';
            updatePoseReadout(label);
        }
    }

    function onPoseMessage(pose) {
        if (!pose || typeof pose !== 'object') return;
        lastPose = pose;
        lastPoseArrival = Date.now();
        if (mode === '3d') refreshRobotPose();
    }

    function onTransportChange(connected) {
        transportConnected = connected === true;
        if (!transportConnected) {
            lastPose = null;
            lastPoseArrival = 0;
        }
        if (mode === '3d') refreshRobotPose();
    }

    function startPoseMonitor() {
        if (poseMonitor !== null) return;
        poseMonitor = window.setInterval(refreshRobotPose, 500);
    }

    function stopPoseMonitor() {
        if (poseMonitor !== null) window.clearInterval(poseMonitor);
        poseMonitor = null;
    }

    function renderSelectedMap() {
        if (!viewer || mode !== '3d' || !canDisplay()) return;
        const mapId = window.selectedNavMap;
        const map = window.mapData;
        if (!mapId) {
            viewer.clearMap();
            displayedMapId = null;
            setStatus(NO_MAP_MESSAGE);
        } else if (failedMapId === mapId) {
            viewer.clearMap();
            displayedMapId = null;
            setStatus(`Failed to load map "${mapId}": ${failedReason}`);
        } else if (loadingMapId === mapId) {
            if (displayedMapId !== mapId) viewer.clearMap();
            setStatus(`Loading map "${mapId}"...`);
        } else if (!map?.is_pgm_raw ||
                   (map.map_id || map.frame_id) !== mapId) {
            viewer.clearMap();
            displayedMapId = null;
            setStatus(`Loading map "${mapId}"...`);
        } else {
            try {
                viewer.setMap(map);
                displayedMapId = mapId;
                viewer.setWallsVisible(wallsVisible);
                viewer.setDisplayMode(displayMode);
                viewer.setAxesVisible(axesVisible);
                setStatus('');
                if (latestLiftMeters !== null) viewer.setLiftHeight(latestLiftMeters);
                refreshRobotPose();
            } catch (error) {
                viewer.clearMap();
                displayedMapId = null;
                setStatus(`Cannot display map "${mapId}": ${error.message}`);
                console.warn('3D map data rejected:', error);
            }
        }
    }

    function canDisplay() {
        return window.isNavigationActive === true &&
            document.getElementById('page-navigation')?.classList.contains('active') &&
            !viewport?.closest('.nav-center-col')?.classList.contains('rack-mode');
    }

    function updateButtons() {
        button2D.classList.toggle('active', mode === '2d');
        button3D.classList.toggle('active', mode === '3d');
        button2D.setAttribute('aria-pressed', String(mode === '2d'));
        button3D.setAttribute('aria-pressed', String(mode === '3d'));
        button3D.disabled = loading;
        rawButton?.setAttribute('aria-pressed', String(displayMode === 'raw'));
        cleanButton?.setAttribute('aria-pressed', String(displayMode === 'clean'));
        updateFollowButton();
    }

    function show2D() {
        ++request; // Ignore a pending dynamic import after switching back.
        loading = false;
        mode = '2d';
        viewer?.hide();
        stopPoseMonitor();
        layer.hidden = true;
        updateButtons();
        window.resizeCanvas?.();
        window.renderMap?.();
    }

    function syncVisibility() {
        if (mode !== '3d' || !viewer) return;
        if (canDisplay()) {
            layer.hidden = false;
            renderSelectedMap();
            requestAnimationFrame(() => {
                if (mode === '3d' && canDisplay()) {
                    viewer.resize();
                    viewer.show();
                    startPoseMonitor();
                    refreshRobotPose();
                }
            });
        } else {
            viewer.hide();
            stopPoseMonitor();
        }
    }

    async function show3D() {
        if (mode === '3d' || loading) return;
        const thisRequest = ++request;
        loading = true;
        updateButtons();
        try {
            if (!viewer) {
                const { createViewer } = await import('/js/three_viewer.js?v=1.3.2');
                if (thisRequest !== request) return;
                viewer = createViewer(() => {
                    show2D();
                    viewer?.dispose();
                    viewer = null;
                    window.showToast?.('3D view unavailable. Returned to 2D.', 'warn');
                }, modelStatus => {
                    modelKind = modelStatus.kind;
                    if (modelStatus.kind === 'fallback') {
                        console.warn('AMR GLB unavailable; using fallback:', modelStatus.reason);
                    }
                    refreshRobotPose();
                }, updateFollowButton);
            }
            if (thisRequest !== request) return;
            window.cancelNavMapTool?.();
            layer.hidden = false;
            viewer.init(layer);
            mode = '3d';
            modelKind = 'loading';
            updateFollowButton();
            loading = false;
            updateButtons();
            syncVisibility();
        } catch (error) {
            console.warn('3D viewer initialization failed:', error);
            viewer?.dispose();
            viewer = null;
            show2D();
            window.showToast?.('3D view unavailable. Returned to 2D.', 'warn');
        }
    }

    function init() {
        viewport = document.getElementById('navMapViewport');
        layer = document.getElementById('navThreeLayer');
        button2D = document.getElementById('navView2D');
        button3D = document.getElementById('navView3D');
        axesButton = document.getElementById('navThreeAxesToggle');
        axesLegend = document.getElementById('navThreeAxesLegend');
        fitButton = document.getElementById('navThreeFit');
        wallsButton = document.getElementById('navThreeWalls');
        rawButton = document.getElementById('navThreeRaw');
        cleanButton = document.getElementById('navThreeClean');
        status = document.getElementById('navThreeStatus');
        if (!viewport || !layer || !button2D || !button3D || !axesButton || !axesLegend ||
                !fitButton || !wallsButton || !rawButton || !cleanButton || !status) return;

        createRobotControls();
        // Reuse the existing WebSocket; no additional connection/subscriber.
        transportConnected = window.navigationTransportState === 'CONNECTED';
        button2D.addEventListener('click', show2D);
        button3D.addEventListener('click', show3D);
        axesButton.addEventListener('click', () => {
            axesVisible = !axesVisible;
            axesButton.textContent = axesVisible ? 'TF Axes On' : 'TF Axes Off';
            axesButton.setAttribute('aria-pressed', String(axesVisible));
            axesLegend.hidden = !axesVisible;
            viewer?.setAxesVisible(axesVisible);
        });
        fitButton.addEventListener('click', () => viewer?.fitToMap());
        wallsButton.addEventListener('click', () => {
            wallsVisible = !wallsVisible;
            wallsButton.textContent = wallsVisible ? 'Walls On' : 'Walls Off';
            wallsButton.setAttribute('aria-pressed', String(wallsVisible));
            viewer?.setWallsVisible(wallsVisible);
        });
        for (const [button, value] of [[rawButton, 'raw'], [cleanButton, 'clean']]) {
            button.addEventListener('click', () => {
                displayMode = value;
                viewer?.setDisplayMode(displayMode);
                updateButtons();
            });
        }
        document.getElementById('navMapTools')?.addEventListener('click', () => {
            if (mode === '3d') show2D();
        }, true);

        // OrbitControls events must not reach Navigation's 2D pose/goal tools.
        for (const type of ['pointerdown', 'pointermove', 'pointerup', 'pointercancel', 'contextmenu', 'wheel']) {
            layer.addEventListener(type, event => event.stopPropagation());
        }
        if ('ResizeObserver' in window) {
            const observer = new ResizeObserver(() => {
                if (mode === '3d' && canDisplay()) viewer?.resize();
            });
            observer.observe(viewport);
        } else {
            window.addEventListener('resize', () => {
                if (mode === '3d' && canDisplay()) viewer?.resize();
            });
        }
        window.addEventListener('pagehide', () => {
            ++request;
            stopPoseMonitor();
            viewer?.dispose();
            viewer = null;
            mode = '2d';
            loading = false;
            layer.hidden = true;
        });
        window.addEventListener('pageshow', updateButtons);
        updateButtons();
    }

    window.navigation3D = {
        onPageChange: () => syncVisibility(),
        onPose: onPoseMessage,
        onTransportChange,
        onLift: lift => {
            if (!lift || !Number.isFinite(lift.height)) return;
            const meters = lift.height / 1000; // system_status lift.height is in mm
            if (meters < 0 || meters > 0.8) return;
            latestLiftMeters = meters;
            viewer?.setLiftHeight(meters);
        },
        onMapVisibilityChange: () => syncVisibility(),
        onMapSelectionChanged: mapId => {
            if (mapId === observedMapId) return;
            observedMapId = mapId;
            loadingMapId = null;
            failedMapId = null;
            viewer?.clearMap();
            displayedMapId = null;
            renderSelectedMap();
        },
        onMapLoading: mapId => {
            loadingMapId = mapId;
            failedMapId = null;
            if (displayedMapId !== mapId) viewer?.clearMap();
            renderSelectedMap();
        },
        onMapLoaded: mapId => {
            if (mapId !== window.selectedNavMap) return;
            loadingMapId = null;
            failedMapId = null;
            renderSelectedMap();
        },
        onMapLoadFailed: (mapId, reason) => {
            if (mapId !== window.selectedNavMap) return;
            loadingMapId = null;
            failedMapId = mapId;
            failedReason = reason || 'Map data unavailable';
            viewer?.clearMap();
            displayedMapId = null;
            renderSelectedMap();
        },
        isActive: () => mode === '3d',
        getMapSummary: () => viewer?.getMapSummary() || null,
    };
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
})();
