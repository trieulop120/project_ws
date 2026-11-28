// Navigation-only switch. The Three.js module and vendor assets load on first 3D use.
(function () {
    'use strict';

    let layer, button2D, button3D, wallsButton,
        rawButton, cleanButton, status, viewport, followButton, poseReadout;
    let viewer = null;
    let plan = null;
    let planState = null;
    let pathVisible = false;

    function syncPlan() {
        const state = planState;
        const matches = transportConnected && plan && state && !state.stale && !state.stopping &&
            state.process_owned && state.nav2_running && state.goal_active &&
            (state.state === 'ACTIVE' || state.state === 'ACCEPTED') &&
            state.plan_goal_id && state.plan_goal_start > 0 && !plan.plan_ambiguous &&
            plan.timestamp >= state.plan_goal_start && plan.bridge_session_id === state.bridge_session_id &&
            (!plan.plan_goal_id || plan.plan_goal_id === state.plan_goal_id) &&
            plan.map_id === state.map_id && plan.map_id === window.selectedNavMap;
        // A one-shot plan may precede its action status or the initial map.
        // Bind it once, using the ROS acceptance stamp from that action.
        if (matches && !plan.plan_goal_id) plan = {...plan,plan_goal_id:state.plan_goal_id};
        viewer?.setNavPlan(pathVisible && matches && displayedMapId === plan.map_id ? plan : null);
    }

    let mode = '2d';
    let request = 0;
    let loading = false;
    let observedMapId = null;
    let displayedMapId = null;
    let loadingMapId = null;
    let failedMapId = null;
    let failedReason = '';
    let wallsVisible = true;
    let displayMode = 'clean';
    let cloudRequested = false;
    let sceneManager = null;
    let markerFeed = null;
    let observedRouteId = null;
    let lastPose = null;
    let lastPoseArrival = 0;
    let transportConnected = false;
    let poseMonitor = null;
    let lastLayoutRefresh = 0;
    let lidarHeight = null;
    let latestLiftMeters = null;
    const POSE_TIMEOUT_MS = 3000;

    const NO_MAP_MESSAGE = 'Vui lòng chọn bản đồ trong Navigation';

    function setStatus(message) {
        if (!status) return;
        if (status.textContent !== message) status.textContent = message;
        status.hidden = !message;
    }

    function updateFollowButton(enabled = viewer?.isFollowingRobot() === true) {
        if (!followButton) return;
        followButton.textContent = enabled ? 'Follow Robot: ON' : 'Follow Robot: OFF';
        followButton.setAttribute('aria-pressed', String(enabled));
        followButton.classList.toggle('active', enabled);
    }

    function createRobotControls() {
        followButton = document.getElementById('navThreeFollow');
        followButton?.addEventListener('click', () => {
            if (!viewer || mode !== '3d') return;
            viewer.setFollowRobot(!viewer.isFollowingRobot());
            updateFollowButton();
        });
        updateFollowButton(false);

        poseReadout = document.getElementById('navThreeRobotTelemetry');
    }

    function updatePoseReadout(label) {
        if (poseReadout && poseReadout.textContent !== label) poseReadout.textContent = label;
    }

    function hasLiveMapPose(frame) {
        const age = performance.now() - lastPoseArrival;
        return transportConnected && displayedMapId && frame && lastPose &&
            lastPose.stale !== true && age >= 0 && age < POSE_TIMEOUT_MS &&
            lastPose.map_id === displayedMapId && lastPose.frame_id === frame &&
            lastPose.base_frame_id === 'base_footprint' &&
            [lastPose.position?.x, lastPose.position?.y,
             lastPose.orientation?.yaw].every(Number.isFinite);
    }

    function refreshRobotPose() {
        if (!viewer || mode !== '3d') return;
        const mapFrame = viewer.getMapSummary()?.frame_id;
        if (!displayedMapId || !mapFrame) {
            viewer.setRobotPose(null);
            updatePoseReadout(transportConnected ? 'Waiting for map' : 'Disconnected');
            syncPointCloudStream();
            return;
        }
        const age = performance.now() - lastPoseArrival;
        if (hasLiveMapPose(mapFrame)) {
            const pose = {
                x: lastPose.position.x, y: lastPose.position.y,
                yaw: lastPose.orientation.yaw
            };
            viewer.setRobotPose(pose);
            updatePoseReadout('Live');
        } else {
            viewer.setRobotPose(null);
            viewer.clearPointCloud();
            const label = !transportConnected ? 'Disconnected' :
                (lastPose && (age >= POSE_TIMEOUT_MS || lastPose.stale === true)) ? 'Stale' : 'Waiting for pose';
            updatePoseReadout(label);
        }
        syncPointCloudStream();
    }

    function onPoseMessage(pose) {
        if (!transportConnected || !pose || typeof pose !== 'object') return;
        // The bridge may rebroadcast a cached pose. Its source timestamp
        // advances even for a stationary robot receiving fresh telemetry.
        if (!Number.isFinite(pose.timestamp) || pose.timestamp !== lastPose?.timestamp ||
                pose.map_id !== lastPose?.map_id || pose.frame_id !== lastPose?.frame_id) {
            lastPoseArrival = performance.now();
        }
        lastPose = pose;
        if (mode === '3d') refreshRobotPose();
    }

    function onTransportChange(connected) {
        transportConnected = connected === true;
        if (!transportConnected) {
            plan = planState = null;
            viewer?.setNavPlan(null);
            lidarHeight = null;
            viewer?.setLidarHeight(null);
            cloudRequested = false; // The old socket and its opt-in no longer exist.
            viewer?.clearPointCloud();
            viewer?.clearLaserScan();
            sceneManager?.clearState();
            lastPose = null;
            lastPoseArrival = 0;
        }
        if (mode === '3d') refreshRobotPose();
        syncPointCloudStream();
    }

    function startPoseMonitor() {
        if (poseMonitor !== null) return;
        poseMonitor = window.setInterval(() => {
            refreshRobotPose();
            const now = performance.now();
            if (now - lastLayoutRefresh >= 5000) {
                lastLayoutRefresh = now;
                sceneManager?.refresh();
                markerFeed?.refresh();
            }
        }, 500);
    }

    function stopPoseMonitor() {
        if (poseMonitor !== null) window.clearInterval(poseMonitor);
        poseMonitor = null;
    }

    function syncPointCloudStream() {
        const wanted = Boolean(mode === '3d' && canDisplay() &&
            displayedMapId === window.selectedNavMap && hasLiveMapPose(viewer?.getMapSummary()?.frame_id));
        if (wanted === cloudRequested) return;
        if (window.sendCommand?.({command: 'point_cloud_stream', enabled: wanted})) {
            cloudRequested = wanted;
        }
        if (!wanted) viewer?.clearPointCloud();
    }

    async function fetchScene(mapId, signal, routeId) {
        const request = new AbortController();
        const cancel = () => request.abort();
        signal.addEventListener('abort', cancel, {once: true});
        const timeout = window.setTimeout(cancel, 5000);
        try {
            const route = routeId ? `?route_id=${encodeURIComponent(routeId)}` : '';
            const response = await fetch(`/api/map/scene/${encodeURIComponent(mapId)}${route}`,
                                         {signal: request.signal, cache: 'no-store'});
            if (response.status === 404) return null;
            if (!response.ok) throw new Error(`HTTP ${response.status}`);
            return await response.json();
        } finally {
            window.clearTimeout(timeout);
            signal.removeEventListener('abort', cancel);
        }
    }

    function renderSelectedMap() {
        if (!viewer || mode !== '3d' || !canDisplay()) return;
        const mapId = window.selectedNavMap;
        const map = window.mapData;
        const route = mapId ? window.selectedNavRoute ?? null : null;
        markerFeed.select(mapId || null, route);
        sceneManager.select(mapId || null, route);
        if (!mapId) {
            displayedMapId = null;
            setStatus(NO_MAP_MESSAGE);
        } else if (failedMapId === mapId) {
            sceneManager.clearGrid();
            displayedMapId = null;
            setStatus(`Failed to load map "${mapId}": ${failedReason}`);
        } else if (loadingMapId === mapId) {
            setStatus(`Loading map "${mapId}"...`);
        } else if (!map?.is_pgm_raw ||
                   (map.map_id || map.frame_id) !== mapId) {
            if (displayedMapId) sceneManager.clearGrid();
            displayedMapId = null;
            setStatus(`Loading map "${mapId}"...`);
        } else {
            try {
                const previousBuild = viewer.getMapSummary().build_count;
                if (!sceneManager.commit(map)) {
                    displayedMapId = null;
                    setStatus(`Loading map "${mapId}"...`);
                    refreshRobotPose();
                    return;
                }
                const changedMap = displayedMapId !== mapId;
                displayedMapId = mapId;
                syncPlan();
                if (changedMap || viewer.getMapSummary().build_count !== previousBuild) markerFeed.replay();
                viewer.setWallsVisible(wallsVisible);
                viewer.setDisplayMode(displayMode);
                setStatus('');
                viewer.setLidarHeight(lidarHeight);
                if (latestLiftMeters !== null) viewer.setLiftHeight(latestLiftMeters);
            } catch (error) {
                sceneManager.clearGrid();
                displayedMapId = null;
                setStatus(`Cannot display map "${mapId}": ${error.message}`);
                console.warn('3D map data rejected:', error);
            }
        }
        refreshRobotPose();
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
        syncPointCloudStream();
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
            syncPointCloudStream();
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
            if (displayedMapId) {
                sceneManager.clearGrid();
                displayedMapId = null;
            }
            syncPointCloudStream();
        }
    }

    async function show3D() {
        if (mode === '3d' || loading) return;
        const thisRequest = ++request;
        loading = true;
        updateButtons();
        try {
            if (!viewer) {
                const [{createViewer}, {createSceneManager}, {createRouteMarkerFeed}] = await Promise.all([
                    import('/js/three_viewer.js?v=3.9.0'),
                    import('/js/scene_manager.js?v=1.3.0'),
                    import('/js/route_marker_feed.js?v=1.1.0'),
                ]);
                if (thisRequest !== request) return;
                viewer = createViewer(() => {
                    show2D();
                    markerFeed?.select(null, null);
                    sceneManager?.select(null);
                    viewer?.dispose();
                    viewer = null;
                    sceneManager = null;
                    markerFeed = null;
                    window.showToast?.('3D view unavailable. Returned to 2D.', 'warn');
                }, modelStatus => {
                    if (modelStatus.kind === 'fallback') {
                        console.warn('AMR GLB unavailable; using fallback:', modelStatus.reason);
                    }
                    refreshRobotPose();
                }, updateFollowButton);
                markerFeed = createRouteMarkerFeed({
                    fetchRoutes: async (mapId, signal) => {
                        const response = await fetch(`/api/routes?map_id=${encodeURIComponent(mapId)}`,
                                                     {signal, cache: 'no-store'});
                        if (!response.ok) throw new Error(`HTTP ${response.status}`);
                        const data = await response.json();
                        if (data.status !== 'ok' || !Array.isArray(data.routes)) throw new Error('Invalid route catalog');
                        return data.routes;
                    },
                    publish: (mapId, markers) => viewer?.setRouteMarkers(mapId, markers),
                    colorForNode: window.routeNodeColor,
                    onError: error => console.warn('3D route markers unavailable:', error),
                });
                sceneManager = createSceneManager({viewer, fetchScene, onReady: renderSelectedMap,
                    onError: error => console.warn('3D semantic layout unavailable:', error)});
            }
            if (thisRequest !== request) return;
            window.cancelNavMapTool?.();
            layer.hidden = false;
            viewer.init(layer);
            mode = '3d';
            updateFollowButton();
            loading = false;
            updateButtons();
            syncVisibility();
        } catch (error) {
            console.warn('3D viewer initialization failed:', error);
            markerFeed?.select(null, null);
            sceneManager?.select(null);
            viewer?.dispose();
            viewer = null;
            sceneManager = null;
            markerFeed = null;
            show2D();
            window.showToast?.('3D view unavailable. Returned to 2D.', 'warn');
        }
    }

    function init() {
        viewport = document.getElementById('navMapViewport');
        layer = document.getElementById('navThreeLayer');
        button2D = document.getElementById('navView2D');
        button3D = document.getElementById('navView3D');
        wallsButton = document.getElementById('navThreeWalls');
        rawButton = document.getElementById('navThreeRaw');
        cleanButton = document.getElementById('navThreeClean');
        status = document.getElementById('navThreeStatus');
        if (!viewport || !layer || !button2D || !button3D ||
                !wallsButton || !rawButton || !cleanButton || !status) return;

        createRobotControls();
        // Reuse the existing WebSocket; no additional connection/subscriber.
        transportConnected = window.navigationTransportState === 'CONNECTED';
        button2D.addEventListener('click', show2D);
        button3D.addEventListener('click', show3D);
        const pathButton = document.getElementById('navThreePath');
        pathButton?.addEventListener('click', () => {
            pathVisible = !pathVisible;
            pathButton.textContent = pathVisible ? 'Path On' : 'Path Off';
            pathButton.setAttribute('aria-pressed', String(pathVisible));
            syncPlan();
        });
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
            mode = '2d';
            syncPointCloudStream();
            markerFeed?.select(null, null);
            sceneManager?.select(null);
            sceneManager = null;
            markerFeed = null;
            displayedMapId = null;
            stopPoseMonitor();
            viewer?.dispose();
            viewer = null;
            loading = false;
            layer.hidden = true;
        });
        window.addEventListener('pageshow', updateButtons);
        updateButtons();
    }

    window.navigation3D = {
        onPageChange: () => syncVisibility(),
        onPose: onPoseMessage,
        onLaserScan: scan => {
            if (transportConnected && mode === '3d' && canDisplay() &&
                    displayedMapId === window.selectedNavMap && scan?.map_id === displayedMapId) {
                viewer?.setLaserScan(scan);
            }
        },
        onPointCloud: message => {
            if (cloudRequested && displayedMapId && displayedMapId === window.selectedNavMap &&
                    mode === '3d' && canDisplay() && hasLiveMapPose(message?.frame_id)) {
                viewer?.setPointCloud(message);
            }
        },
        onWarehouse: message => {
            if (message?.map_id === window.selectedNavMap) sceneManager?.setWarehouse(message);
        },
        onSemanticState: message => {
            if (message?.map_id === window.selectedNavMap) sceneManager?.setState(message);
        },
        onNavState: state => {
            if (!transportConnected) return;
            planState = state;
            syncPlan();
        },
        onNavResult: result => {
            if (result.bridge_session_id === planState?.bridge_session_id && result.goal_id === planState.goal_id) {
                planState = null;
                syncPlan();
            }
        },
        onPlan: message => {
            if (!transportConnected) return;
            if (message.map_id && window.selectedNavMap && message.map_id !== window.selectedNavMap &&
                    planState?.map_id === window.selectedNavMap) return;
            if (plan && message.map_id === plan.map_id && message.bridge_session_id === plan.bridge_session_id &&
                    message.plan_goal_id === plan.plan_goal_id && message.timestamp > 0 &&
                    message.timestamp < plan.timestamp) return;
            plan = message.points?.length >= 2 ? message : null;
            syncPlan();
        },
        onTransportChange,
        onLidarHeight: height => {
            lidarHeight = Number.isFinite(height) && height > 0 ? height : null;
            viewer?.setLidarHeight(lidarHeight);
        },
        onLift: lift => {
            if (!lift || !Number.isFinite(lift.height)) return;
            const meters = lift.height / 1000; // system_status lift.height is in mm
            if (meters < 0 || meters > 0.8) return;
            latestLiftMeters = meters;
            viewer?.setLiftHeight(meters);
        },
        onMapVisibilityChange: () => syncVisibility(),
        onMapSelectionChanged: mapId => {
            const route = window.selectedNavRoute ?? null;
            if (mapId === observedMapId) {
                if (route !== observedRouteId) {
                    observedRouteId = route;
                    renderSelectedMap();
                }
                return;
            }
            observedMapId = mapId;
            if (plan?.map_id !== mapId) plan = null;
            syncPlan();
            observedRouteId = route;
            if (lastPose?.map_id !== mapId) {
                lastPose = null;
                lastPoseArrival = 0;
            }
            loadingMapId = null;
            failedMapId = null;
            markerFeed?.select(null, null);
            sceneManager?.select(null);
            displayedMapId = null;
            syncPointCloudStream();
            renderSelectedMap();
        },
        onMapLoading: mapId => {
            loadingMapId = mapId;
            failedMapId = null;
            if (displayedMapId && displayedMapId !== mapId) sceneManager?.clearGrid();
            displayedMapId = null;
            syncPointCloudStream();
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
            sceneManager?.clearGrid();
            displayedMapId = null;
            syncPointCloudStream();
            renderSelectedMap();
        },
        isActive: () => mode === '3d',
        getMapSummary: () => viewer?.getMapSummary() || null,
    };
    if (document.readyState === 'loading') document.addEventListener('DOMContentLoaded', init);
    else init();
})();
