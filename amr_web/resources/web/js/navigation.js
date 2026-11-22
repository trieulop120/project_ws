// resources/web/js/navigation.js
// Phase 4 & 5 Fix: Single canvas, pointer events, proper nav state handling
(function () {
    'use strict';

    // ============================================
    // Navigation State
    // ============================================
    window.isNavigationActive = false;
    window.selectedNavMap = null;
    window.selectedNavRoute = null;
    window.currentRoute = null;
    window.showRouteOverlay = false;  // Phase D: mac dinh TAT het de tranh lag (user yeu cau)
    // Phase D: nav layer toggles (docs 6.2) - mac dinh tat, user bat khi can
    window.showGlobalCostmap = false;
    window.showLocalCostmap = false;
    window.showPlan = false;

    // Operating Mode: MANUAL | AUTO - exported for state.js
    window.currentMissionSupervisorStatus = null;
    window.currentPalletRetrievalStatus = null;
    window.latestNavState = null;
    window.currentNavigationSessionId = null;
    let isMissionActive = false;
    let navStarting = false;
    let navStopping = false;  // B21 Fix: Cờ báo đang dừng - phòng thủ frontend
    let missionStartedAt = 0;
    let missionElapsedMs = 0;
    let missionElapsedTimer = null;
    window.autoMissionStartPending = false;
    let pendingGoalName = '';
    let mapReqTimer = null;
    let lastManualTarget = '--';
    let retrievalProgressStep = 0;
    window.navigationRequests = {goal: false, cancel: false};
    let routeRestoreKey = null;
    let modalGeneration = 0;
    let modalLoading = false;
    const seenNavResults = new Set();
    const navBusyStates = new Set(['SENDING', 'ACCEPTED', 'ACTIVE', 'CANCELING', 'UNKNOWN']);

    // Tool state (RVIZ-style: click to place, move to aim, click/Enter to confirm, ESC to cancel)
    let toolPending = null;      // 'pose' or 'goal' when aiming
    let toolPendingX = 0;        // world X of placement
    let toolPendingY = 0;        // world Y of placement
    let toolPendingYaw = 0;      // yaw from mouse direction
    let toolPendingMouseX = 0;   // screen X when placed
    let toolPendingMouseY = 0;   // screen Y when placed
    let toolPositionSet = false; // true after first click sets position
    let toolConfirmOnRelease = false;
    // Export for map.js preview
    window.toolPending = null;
    window.toolPendingX = 0;
    window.toolPendingY = 0;
    window.toolPendingYaw = 0;

    // confirmedOpMode is authoritative; currentOpMode is its internal alias.
    let confirmedOpMode = 'MANUAL';
    let currentOpMode;
    let desiredOpMode = confirmedOpMode;
    let modeSwitchPromise = null;
    let pendingNavState = null;

    function applyConfirmedOpMode(mode, syncDesired = false) {
        const nextMode = mode === 'AUTO' ? 'AUTO' : 'MANUAL';
        const previousMode = confirmedOpMode;
        confirmedOpMode = nextMode;
        currentOpMode = confirmedOpMode;
        window.currentOpMode = confirmedOpMode;
        if (syncDesired) desiredOpMode = confirmedOpMode;

        if (nextMode === 'AUTO' && previousMode !== 'AUTO') {
            stopManualTeleop();
            window.autoMissionStartPending = false;
            stopMissionClock(true);
        } else if (previousMode === 'AUTO') {
            window.autoMissionStartPending = false;
            isMissionActive = false;
            lastManualTarget = '--';
            stopMissionClock(true);
            const progressBar = document.getElementById('navProgressBar');
            const progressText = document.getElementById('navProgressText');
            updateMissionStatus('Idle', '--');
            if (progressBar) progressBar.style.width = '0%';
            if (progressText) progressText.textContent = '0%';
        }

        updateNavUI();
        if (typeof window.updateMissionCycle === 'function') {
            window.updateMissionCycle(window.currentMissionSupervisorStatus);
        }
        if (typeof updateOpModeDisplay === 'function') updateOpModeDisplay();
    }

    // Nav state from backend
    let navState = {
        state: 'IDLE',
        progress: 0,
        nav2_running: false,
        nav2_ready: false,
        waypoint_index: 0,
        waypoint_total: 0,
        map_id: null,
        route_id: null,
        target_pose: null,
        distance_remaining: null,
        estimated_time_remaining: null
    };

    let availableMaps = [];
    let mapToRouteMap = {};
    let routeToMapMap = {};
    let isBindingInProgress = false;

    // Tool state
    let activeTool = null;
    window.navDragState = { isDragging: false, startX: 0, startY: 0, currentX: 0, currentY: 0,
                             startMapX: 0, startMapY: 0, yaw: 0, tool: null };

    // ============================================
    // DOM Elements
    // ============================================
    const navSetupModal = document.getElementById('navSetupModal');
    const btnStartNav = document.getElementById('btnStartNav');
    const btnStopNav = document.getElementById('btnStopNav');
    const navModalClose = document.getElementById('navModalClose');
    const navModalCancel = document.getElementById('navModalCancel');
    const navModalConfirm = document.getElementById('navModalConfirm');
    const navMapSelect = document.getElementById('navMapSelect');
    const navRouteSelect = document.getElementById('navRouteSelect');

    // Tool buttons
    const toolSetPose = document.getElementById('toolSetPose');
    const toolSetGoal = document.getElementById('toolSetGoal');
    const toolToggleRoute = document.getElementById('toolToggleRoute');
    const toolCenterMap = document.getElementById('toolCenterMap');

    // Mission buttons
    const btnStartMission = document.getElementById('btnStartMission');
    const btnCancelMission = document.getElementById('btnCancelMission');
    const btnRecoveryMission = document.getElementById('btnRecoveryMission');

    // Mode toggle
    const btnModeAuto = document.getElementById('btnModeAuto');
    const btnModeManual = document.getElementById('btnModeManual');

    // Camera buttons
    const btnCamRgb = document.getElementById('btnCamRgb');
    const btnCamDepth = document.getElementById('btnCamDepth');
    const btnCamDetect = document.getElementById('btnCamDetect');
    const btnNavToggleCam = document.getElementById('btnNavToggleCam');

    // Cargo card elements (populated from /pallet_mission/status)
    const navCargoStatus = document.getElementById('navCargoStatus');
    const navCargoText = document.getElementById('navCargoText');

    // Camera state
    let cameraMode = 'off'; // 'off', 'rgb', 'depth', 'detect'

    // Nav viewport
    const navViewport = document.getElementById('navMapViewport');

    // Initialize the exported mode through the same assignment path used later.
    applyConfirmedOpMode(confirmedOpMode);

    // ============================================
    // Map Selection Modal
    // ============================================

    function extractMapId(mapItem) {
        if (!mapItem) return '';
        if (typeof mapItem === 'object') return mapItem.id || mapItem.name || mapItem.map_id || '';
        return String(mapItem);
    }

    function extractRouteId(routeItem, defaultMapId) {
        if (!routeItem) return '';
        if (typeof routeItem === 'object') return routeItem.graph_name || routeItem.id || routeItem.name || '';
        return String(routeItem);
    }

    async function showNavSetupModal() {
        if (modalLoading || navStarting || navStopping || window.isNavigationActive ||
                !window.missionStatusFresh(window.latestNavState)) return;
        modalLoading = true;
        const generation = ++modalGeneration;
        if (navSetupModal) navSetupModal.classList.add('active');
        window.selectedNavMap = null;
        window.selectedNavRoute = null;
        window.currentRoute = null;
        if (navMapSelect) navMapSelect.innerHTML = '<option value="">-- Select a map --</option>';
        if (navRouteSelect) navRouteSelect.innerHTML = '<option value="">-- Select a route --</option>';
        if (navModalConfirm) navModalConfirm.disabled = true;

        let statusMsg = document.getElementById('navModalStatusMsg');
        if (!statusMsg && navSetupModal) {
            statusMsg = document.createElement('div');
            statusMsg.id = 'navModalStatusMsg';
            statusMsg.style.cssText = 'font-size: 0.75rem; margin-top: 10px; padding: 8px; border-radius: 4px; display: none; text-align: left;';
            const modalBody = navSetupModal.querySelector('.modal-body');
            if (modalBody) modalBody.appendChild(statusMsg);
        }
        if (statusMsg) statusMsg.style.display = 'none';

        mapToRouteMap = {};
        routeToMapMap = {};

        try {
            const mapRes = await fetch('/api/maps');
            const mapData = await mapRes.json();
            if (generation !== modalGeneration) return;

            if (mapData.status === 'ok' && Array.isArray(mapData.maps)) {
                availableMaps = mapData.maps;

                availableMaps.forEach(mapItem => {
                    const mapId = extractMapId(mapItem);
                    if (mapId && mapId !== '[object Object]') {
                        const option = document.createElement('option');
                        option.value = mapId;
                        option.textContent = mapId;
                        navMapSelect.appendChild(option);
                    }
                });

                for (const mapItem of availableMaps) {
                    const mapId = extractMapId(mapItem);
                    if (!mapId) continue;

                    try {
                        const routeRes = await fetch(`/api/routes?map_id=${encodeURIComponent(mapId)}`);
                        const routeData = await routeRes.json();
                        if (generation !== modalGeneration) return;

                        if (routeData.status === 'ok' && Array.isArray(routeData.routes) && routeData.routes.length > 0) {
                            for (const route of routeData.routes) {
                                const routeId = extractRouteId(route, mapId);
                                const key = JSON.stringify([mapId, routeId]);
                                if (!mapToRouteMap[mapId]) mapToRouteMap[mapId] = {id: routeId, rawData: route};
                                routeToMapMap[key] = {mapId, id: routeId, rawData: route};
                                const option = document.createElement('option');
                                option.value = key;
                                option.textContent = `${routeId} (${mapId})`;
                                navRouteSelect.appendChild(option);
                            }
                        }
                    } catch (e) {
                        console.warn(`[Nav] Cannot fetch routes for map ${mapId}:`, e);
                    }
                }
            }
        } catch (err) {
            console.error('[Nav] Failed to load maps or routes:', err);
        }

        modalLoading = false;
    }

    function hideNavSetupModal() {
        modalGeneration++;
        modalLoading = false;
        if (navSetupModal) navSetupModal.classList.remove('active');
    }

    function onMapSelectChange(e) {
        if (isBindingInProgress) return;
        isBindingInProgress = true;

        const selectedMapId = e.target.value;
        const statusMsg = document.getElementById('navModalStatusMsg');

        if (!selectedMapId) {
            if (navRouteSelect) navRouteSelect.value = '';
            window.selectedNavMap = null;
            window.selectedNavRoute = null;
            window.currentRoute = null;
            if (navModalConfirm) navModalConfirm.disabled = true;
            if (statusMsg) statusMsg.style.display = 'none';
            isBindingInProgress = false;
            return;
        }

        window.selectedNavMap = selectedMapId;

        if (mapToRouteMap[selectedMapId]) {
            const routeInfo = mapToRouteMap[selectedMapId];
            if (navRouteSelect) navRouteSelect.value = JSON.stringify([selectedMapId, routeInfo.id]);
            window.selectedNavRoute = routeInfo.id;
            window.currentRoute = routeInfo.rawData;

            if (statusMsg) {
                statusMsg.style.display = 'block';
                statusMsg.style.background = 'rgba(16, 185, 129, 0.15)';
                statusMsg.style.color = '#10B981';
                statusMsg.style.border = '1px solid #10B981';
                statusMsg.textContent = `Auto-selected route: "${routeInfo.id}"`;
            }
            if (navModalConfirm) navModalConfirm.disabled = false;
        } else {
            if (navRouteSelect) navRouteSelect.value = '';
            window.selectedNavRoute = null;
            window.currentRoute = null;

            if (statusMsg) {
                statusMsg.style.display = 'block';
                statusMsg.style.background = 'rgba(239, 68, 68, 0.15)';
                statusMsg.style.color = '#EF4444';
                statusMsg.style.border = '1px solid #EF4444';
                statusMsg.textContent = `Map "${selectedMapId}" has no Route Graph.`;
            }
            if (navModalConfirm) navModalConfirm.disabled = true;
        }

        isBindingInProgress = false;
    }

    function onRouteSelectChange(e) {
        if (isBindingInProgress) return;
        isBindingInProgress = true;

        const selectedRouteKey = e.target.value;
        const linkedRoute = routeToMapMap[selectedRouteKey];
        const selectedRouteId = linkedRoute?.id;
        const statusMsg = document.getElementById('navModalStatusMsg');

        if (!selectedRouteId) {
            if (navMapSelect) navMapSelect.value = '';
            window.selectedNavMap = null;
            window.selectedNavRoute = null;
            window.currentRoute = null;
            if (navModalConfirm) navModalConfirm.disabled = true;
            if (statusMsg) statusMsg.style.display = 'none';
            isBindingInProgress = false;
            return;
        }

        window.selectedNavRoute = selectedRouteId;
        const linkedMapId = linkedRoute?.mapId;

        if (linkedMapId) {
            if (navMapSelect) navMapSelect.value = linkedMapId;
            window.selectedNavMap = linkedMapId;

            window.currentRoute = linkedRoute.rawData;

            if (statusMsg) {
                statusMsg.style.display = 'block';
                statusMsg.style.background = 'rgba(16, 185, 129, 0.15)';
                statusMsg.style.color = '#10B981';
                statusMsg.style.border = '1px solid #10B981';
                statusMsg.textContent = `Route belongs to map: "${linkedMapId}". Auto-selected map!`;
            }
            if (navModalConfirm) navModalConfirm.disabled = false;
        }

        isBindingInProgress = false;
    }

    function onConfirmClick() {
        hideNavSetupModal();
        startNavigation();
    }

    // ============================================
    // Khôi phục Route Graph khi F5/reload
    // ============================================
    async function restoreRouteData(mapId, routeId) {
        try {
            const res = await fetch(`/api/routes?map_id=${encodeURIComponent(mapId)}`);
            const json = await res.json();
            if (json.status === 'ok' && json.routes && json.routes.length > 0) {
                // Tim route khớp với routeId tu backend
                if (window.selectedNavMap !== mapId || window.selectedNavRoute !== routeId) return;
                const route = json.routes.find(r => r.id === routeId);
                if (!route) return;
                window.currentRoute = route;
                window.selectedNavRoute = route.id;
                console.log('[Nav] Route restored:', route.id);
                if (typeof window.renderMap === 'function') {
                    window.renderMap();
                }
            }
        } catch (e) {
            console.error('[Nav] Failed to restore route:', e);
        } finally {
            if (routeRestoreKey === key) routeRestoreKey = null;
        }
    }

    // ============================================
    // Navigation Start/Stop (Phase 5 Fix)
    // ============================================

    function startNavigation() {
        if (navStarting || navStopping || window.isNavigationActive ||
                !window.missionStatusFresh(window.latestNavState)) return;
        if (window.isSlamActive) {
            if (typeof window.showToast === 'function') {
                window.showToast('Please stop SLAM before running Navigation', 'warn');
            }
            return;
        }

        if (!window.selectedNavMap || !window.selectedNavRoute) return;

        // Phase D 6.5: xoa lop nav cu truoc khi gui start_navigation (tranh rac tu phien truoc)
        if (typeof window.clearNavLayers === 'function') window.clearNavLayers();

        // Clear old map data
        window.mapData = null;
        window.mapImageData = null;
        if (window.mapRenderer) window.mapRenderer.hide(true);

        navStarting = true;
        navStopping = false;  // B21 Fix: Reset cờ dừng khi bắt đầu
        pendingGoalName = '';

        showNavActiveUI();
        updateNavUI();
        showNavMapViewport();

        if (typeof updateOpModeDisplay === 'function') updateOpModeDisplay();
        if (typeof addActivity === 'function') {
            addActivity('info', `Navigation startup requested – Map: ${window.selectedNavMap}`);
        }

        // Load PGM map for Navigation (Dual-Source: is_pgm_raw = true)
        loadNavMap();

        if (typeof window.sendCommandAndWait === 'function') {
            window.sendCommandAndWait({command: 'start_navigation', map_id: window.selectedNavMap,
                route_id: window.selectedNavRoute}).then(response => {
                // Request-ID responses are consumed by the promise layer in
                // ros_connection.js; the generic command dispatcher never
                // sees them. Handle both success and failure here.
                if (response.command === 'start_navigation') {
                    window.onNavCommandResponse?.(response);
                } else {
                    navStarting = false;
                    updateNavUI();
                    if (!window.missionStatusFresh(window.latestNavState) || !window.isNavigationActive) {
                        window.showToast?.('Navigation startup could not be confirmed', 'warn');
                    }
                }
            }).catch(error => {
                console.error('Navigation startup request failed:', error);
                navStarting = false;
                updateNavUI();
                window.showToast?.('Navigation startup outcome unknown; verify robot state', 'error');
            });
        } else {
            rollbackStart('Command acknowledgement unavailable');
        }
    }

    // B11: Token để tránh race khi load map
    let navMapLoadSeq = 0;
    let navMapLoading = false;
    let navMapLoadingId = null;

    async function loadNavMap() {
        // B11: Token để tránh race condition
        const mapId = window.selectedNavMap;
        if (!mapId || navStopping || (navMapLoading && navMapLoadingId === mapId)) return;
        const seq = ++navMapLoadSeq;
        navMapLoading = true;
        navMapLoadingId = mapId;

        try {
            const res = await fetch(`/api/map/image/${encodeURIComponent(mapId)}`);
            const json = await res.json();

            // B11: Kiểm tra token - bỏ qua nếu đã stop hoặc có map mới
            if (seq !== navMapLoadSeq) {
                console.log('[Nav] Map load aborted - seq mismatch');
                return;
            }

            // B11: Kiểm tra navPhase đã chuyển sang STOPPING/IDLE chưa
            if ((!window.isNavigationActive && !navStarting) || window.selectedNavMap !== mapId) {
                console.log('[Nav] Map load aborted - navigation stopped');
                return;
            }

            if (json.status === 'ok') {
                // B11/B22: Dùng setStaticMap để load với bbox fitting
                if (typeof window.setStaticMap === 'function') {
                    window.setStaticMap(window.selectedNavMap, json.info, json.image_data);
                } else {
                    // Fallback nếu setStaticMap chưa có
                    window.mapData = {
                        info: json.info,
                        data: json.image_data,
                        is_pgm_raw: true,
                        frame_id: window.selectedNavMap
                    };
                    window.mapImageData = buildMapImageData();
                    if (typeof window.renderMap === 'function') {
                        window.renderMap();
                    }
                    if (typeof window.updateOverviewMapCard === 'function') {
                        window.updateOverviewMapCard(window.mapData);
                    }
                }
            }
        } catch (e) {
            console.error('[Nav] Failed to load PGM map:', e);
        } finally {
            if (seq === navMapLoadSeq) {
                navMapLoading = false;
                navMapLoadingId = null;
            }
        }
    }

    function rollbackStart(msg) {
        navStarting = false;
        window.isNavigationActive = window.latestNavState?.nav2_running === true &&
            window.latestNavState?.process_owned !== false;
        if (window.isNavigationActive) showNavActiveUI();
        else hideNavActiveUI();
        updateNavUI();
        // Phase D 6.5: start that bai -> xoa lop
        if (!window.isNavigationActive && typeof window.clearNavLayers === 'function') window.clearNavLayers();
        if (typeof window.showToast === 'function') {
            window.showToast(`Cannot start Navigation: ${msg}`, 'error');
        }
    }

    function startMapRequestLoop() {
        clearInterval(mapReqTimer);
        let tries = 0;
        mapReqTimer = setInterval(() => {
            if (!window.isNavigationActive || window.mapData || ++tries > 20) {
                clearInterval(mapReqTimer);
                return;
            }
            if (typeof sendCommand === 'function') sendCommand({ command: 'get_map' });
        }, 1500);
    }

    async function stopNavigation() {
        if (navStopping) return;
        // B21 Fix: Bật cờ dừng để ngăn handleNavStateUpdate nạp lại map
        navStopping = true;
        if (btnStopNav) btnStopNav.disabled = true;

        // Dừng ngay teleop bàn phím nếu đang giữ phím
        stopManualTeleop();

        // B11: Tăng seq để huỷ các pending load
        navMapLoadSeq++;
        navMapLoading = false;

        // One STOP request owns *all* Auto/Retrieval/Goal cancellation.
        // A preliminary nav_cancel from the browser would race with the ROS
        // coordinators and can leave an unintentional Recovery behind.
        let cancelWarning = '';
        const retrieval = window.currentPalletRetrievalStatus;
        const stopResponse = window.sendCommandAndWait
            ? await window.sendCommandAndWait({ command: 'stop_navigation' })
            : { status: 'error', message: 'Command acknowledgement unavailable' };
        // The stop ACK is only sufficient when the returned snapshot also
        // confirms this Bridge no longer owns an active Nav2 process.
        // Discovery may briefly see an old/external action server; that must
        // not keep the *Web-owned* navigation session on the active page.
        const confirmedStop = stopResponse.status === 'stopped' &&
            stopResponse.nav_state?.process_owned === false &&
            stopResponse.nav_state?.stopping !== true;
        if (!confirmedStop) {
            if (stopResponse.nav_state) window.handleNavStateUpdate(stopResponse.nav_state);
            navStopping = false;
            if (btnStopNav) btnStopNav.disabled = false;
            if (typeof window.showToast === 'function') {
                window.showToast('Navigation stop not confirmed. Check robot status.', 'error');
            }
            if (typeof sendCommand === 'function') sendCommand({ command: 'get_nav_state' });
            return;
        }
        // Stop acknowledgement includes the final IDLE nav_state broadcast;
        // accept future state updates again now that shutdown has completed.
        navStopping = false;
        if (retrieval && !cancelWarning) {
            window.clearPalletRetrievalMissionStatus?.(true);
        }
        if (stopResponse.message && stopResponse.message.includes('WARNING:')) {
            // Keep the technical reason in logs, not in the operator toast.
            console.warn('[Nav] Stop cleanup needs attention:', stopResponse.message);
            cancelWarning = 'mission cleanup was not fully confirmed';
        }

        // Clear tool
        setActiveTool(null);

        // Phase D 6.5: bam Stop -> xoa lop (an toan, backend cung se goi)
        if (typeof window.clearNavLayers === 'function') window.clearNavLayers();
        if (typeof window.resetSessionUI === 'function') window.resetSessionUI();

        // Clear state
        clearInterval(mapReqTimer);

        // FIX: Clear map data like SLAM does
        window.isNavigationActive = false;
        window.selectedNavMap = null;
        window.selectedNavRoute = null;
        window.currentRoute = null;
        isMissionActive = false;
        window.autoMissionStartPending = false;
        lastManualTarget = '--';
        stopMissionClock(true);

        // Clear map data & UI (same as SLAM stop)
        window.mapData = null;
        window.mapImageData = null;
        if (window.mapRenderer) {
            window.mapRenderer.hide(true);
        }
        resetNavMapCard();

        if (stopResponse.nav_state) window.handleNavStateUpdate(stopResponse.nav_state);
        else if (typeof sendCommand === 'function') sendCommand({command: 'get_nav_state'});
        const stoppedProgressBar = document.getElementById('navProgressBar');
        const stoppedProgressText = document.getElementById('navProgressText');
        if (stoppedProgressBar) stoppedProgressBar.style.width = '0%';
        if (stoppedProgressText) stoppedProgressText.textContent = '--';

        // Reset op mode to MANUAL (safe default)
        applyConfirmedOpMode('MANUAL', true);
        if (btnStopNav) btnStopNav.disabled = false;

        hideNavActiveUI();
        updateNavUI();
        if (typeof window.updateOverviewNavigationState === 'function') {
            window.updateOverviewNavigationState(navState, window.currentMissionSupervisorStatus);
        }
        if (typeof updateOpModeDisplay === 'function') updateOpModeDisplay();
        if (typeof addActivity === 'function') addActivity('info', 'Navigation stopped');
        if (cancelWarning && typeof window.showToast === 'function') {
            window.showToast(`Navigation stopped, but ${cancelWarning}`, 'warn');
        }
    }

    function resetNavMapCard() {
        const cardName = document.getElementById('cardMapName');
        const cardStatus = document.getElementById('cardMapStatus');
        const mapEmpty = document.getElementById('mapEmptyState');
        const mapStatus = document.getElementById('mapStatusDisplay');
        const mapName = document.getElementById('mapNameDisplay');

        if (cardName) { cardName.textContent = 'None'; cardName.className = 'metric-value muted'; }
        if (cardStatus) cardStatus.textContent = 'No map loaded';
        if (mapEmpty) mapEmpty.style.display = 'flex';
        if (mapStatus) { mapStatus.textContent = 'Waiting'; mapStatus.className = 'map-status'; }
        if (mapName) mapName.textContent = '--';
    }

    // ============================================
    // UI Helpers
    // ============================================

    function showNavActiveUI() {
        const navWelcome = document.getElementById('navWelcome');
        const navLayout = document.getElementById('navLayout');

        if (navWelcome) navWelcome.style.display = 'none';
        if (navLayout) {
            navLayout.style.display = 'grid';
            navLayout.style.animation = 'fadeIn 0.4s ease';
        }
    }

    function hideNavActiveUI() {
        const navWelcome = document.getElementById('navWelcome');
        const navLayout = document.getElementById('navLayout');

        if (navWelcome) navWelcome.style.display = 'flex';
        if (navLayout) navLayout.style.display = 'none';
    }

    function updateNavUI() {
        // Vấn đề 4 Fix: Đã xóa logic cập nhật navStatusBadge ở đây
        // Logic cập nhật trạng thái badge được gom về hàm handleNavStateUpdate() duy nhất
        const isActive = window.isNavigationActive;
        const fresh = window.missionStatusFresh(window.latestNavState);
        const busy = navState.goal_active || navBusyStates.has(navState.state) ||
            window.navigationRequests.goal || window.navigationRequests.cancel;
        const reason = !fresh ? 'Connection or navigation state unavailable' :
            busy ? 'Navigation is pending, active or unconfirmed' :
            navState.nav2_running && navState.process_owned === false
                ? 'Another or recently stopped Nav2 server is still detected; verify shutdown before restarting' :
            navState.readiness_reason || (!navState.nav2_ready ? 'Waiting for Nav2 readiness' : '');
        // A Welcome layout does not imply it is safe to launch a second Nav2.
        // An unowned/discovered Nav2 stack or an unresolved goal still blocks Start.
        if (btnStartNav) { btnStartNav.disabled = !fresh || isActive || navStarting || navStopping ||
            navState.nav2_running || navState.goal_active || navState.state === 'UNKNOWN'; btnStartNav.title = reason; }
        if (btnStopNav) { btnStopNav.disabled = !fresh || navStopping || navState.process_owned === false; btnStopNav.title = navState.process_owned === false ? 'This Bridge does not own the detected Nav2 process' : reason; }
        if (btnModeAuto) btnModeAuto.disabled = !fresh || navStopping || busy;
        if (btnModeManual) btnModeManual.disabled = !fresh || navStopping || (busy && currentOpMode === 'MANUAL');
        const header = document.getElementById('navHeaderBadge');
        if (header) {
            header.textContent = !fresh ? 'State unavailable' : navState.stopping ? 'Nav2 stopping' :
                navState.state === 'UNKNOWN' ? 'State unknown' :
                !navState.nav2_running ? 'Nav2 stopped' : !navState.nav2_ready ? 'Nav2 not ready' :
                busy ? 'Navigation active' : 'Nav2 ready';
            header.title = reason;
            header.classList.toggle('active', fresh && navState.nav2_ready);
        }
        document.querySelectorAll('[data-quick-goal]').forEach(button => {
            button.disabled = !canSendManualGoal();
            button.title = reason;
        });

        // Mode buttons
        if (btnModeAuto) btnModeAuto.classList.toggle('active', currentOpMode === 'AUTO');
        if (btnModeManual) btnModeManual.classList.toggle('active', currentOpMode === 'MANUAL');
        const pendingMode = modeSwitchPromise && desiredOpMode !== confirmedOpMode
            ? desiredOpMode : null;
        btnModeAuto?.classList.toggle('pending', pendingMode === 'AUTO');
        btnModeManual?.classList.toggle('pending', pendingMode === 'MANUAL');

        // Tools: only in MANUAL mode when nav active
        const owner = window.getMissionOwner();
        const missionBusy = owner.stale || (owner.active && owner.source !== 'navigation');
        const toolsEnabled = isActive && fresh && !busy && currentOpMode === 'MANUAL' && !modeSwitchPromise && !missionBusy;
        if (toolSetPose) {
            toolSetPose.disabled = !toolsEnabled || navState.can_set_initial_pose === false;
            toolSetPose.style.opacity = toolSetPose.disabled ? '0.5' : '1';
        }
        if (toolSetGoal) {
            toolSetGoal.disabled = !toolsEnabled || !navState.nav2_ready || navState.can_send_goal === false;
            toolSetGoal.style.opacity = toolSetGoal.disabled ? '0.5' : '1';
        }

        updateMissionButtons();
    }

    function updateMissionButtons() {
        const owner = window.getMissionOwner();
        const m = window.currentMissionSupervisorStatus;
        const state = m?.state || 'UNKNOWN';
        const returnHomeUnconfirmed = state === 'DONE' && m?.active_mission === 'return_home' &&
            (m?.nav_mission?.state !== 'SUCCEEDED' || m?.nav_goal_active !== false ||
             m?.nav_status_fresh !== true || m?.robot_stopped !== true);
        const active = ['GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING',
            'WAITING_SLOT', 'RETURN_HOME', 'RECOVERY', 'STOPPING'].includes(state) ||
            m?.web_control_pending || returnHomeUnconfirmed;
        const autoStarted = window.autoMissionStartPending || active;
        const retrieval = window.currentPalletRetrievalStatus;
        const retrievalFresh = retrievalOwnsMissionCard();
        const retrievalActive = window.retrievalStartPending ||
            window.missionActiveStates.retrieval.has(retrieval?.state);
        const retrievalTerminal = retrievalFresh &&
            ['DONE', 'FAILED', 'CANCELED', 'STOPPED'].includes(retrieval?.state);
        const autoPending = !!window.autoMissionRequest && !window.autoMissionRequest.uncertain;
        if (btnStartMission) {
            btnStartMission.style.display = currentOpMode === 'AUTO' && !autoStarted &&
                owner.source !== 'retrieval' && !retrievalActive && !(retrievalFresh && retrieval?.recovery_required) ? 'block' : 'none';
            btnStartMission.disabled = !m || m.stale || m.can_start !== true ||
                owner.active || owner.stale ||
                !navState.nav2_ready || m.route_graph?.loaded !== true || window.autoMissionStartPending ||
                (navState.goal_active || window.missionActiveStates.navigation.has(navState.state)) || m.nav_goal_active === true || !!modeSwitchPromise || !!window.autoMissionRequest;
        }
        if (btnCancelMission) {
            btnCancelMission.textContent = retrieval?.discard_pending ? 'DISCARDING...' :
                retrieval?.state === 'RECOVERY' && !retrieval.resuming ? 'DISCARD RECOVERY' : retrievalActive ? 'CANCEL RETRIEVAL' :
                (autoStarted || window.autoMissionRequest ? (state === 'RECOVERY' || m?.cancel_pending || m?.web_control_action === 'mission_cycle_cancel' || window.autoMissionRequest?.uncertain ? 'CANCEL AUTO MISSION' : 'CANCEL MISSION') :
                    retrievalTerminal ? 'DISMISS RETRIEVAL' : 'CANCEL MISSION');
            btnCancelMission.style.display = retrievalActive || owner.source === 'retrieval' || currentOpMode === 'MANUAL' ||
                (owner.source === 'pallet' && owner.active) ||
                autoStarted || !!window.autoMissionRequest ? 'block' : 'none';
            btnCancelMission.disabled = navStopping || retrieval?.discard_pending === true || owner.stale ||
                !window.missionStatusFresh(window.latestNavState) ||
                !!window.navigationRequests?.cancel || (autoPending && ['mission_cycle_stop', 'mission_cycle_cancel'].includes(window.autoMissionRequest.command)) ||
                (!owner.active && !window.autoMissionRequest && !window.navigationRequests?.goal && !owner.stale);
            btnCancelMission.title = btnCancelMission.disabled ? 'No active task, cancellation pending, or connection unavailable' : 'Request cancellation and await confirmation';
        }
        if (btnRecoveryMission) {
            const paused = owner.source === 'pallet' ? !!owner.status?.recovery?.checkpoint : owner.status?.state === 'RECOVERY';
            btnRecoveryMission.style.display = paused ? 'block' : 'none';
            // Do not expose raw ROS reasons (possibly localized) as hover text.
            btnRecoveryMission.title = 'Resume the saved mission when recovery is available';
            btnRecoveryMission.disabled = !canResumeMission() || !navState.nav2_ready || autoPending;
        }
        const confirmEmpty = document.getElementById('btnConfirmForksEmpty');
        if (confirmEmpty) {
            const cargo = window.currentCargoTelemetry;
            confirmEmpty.disabled = currentOpMode !== 'MANUAL' || active ||
                !m || m.stale || m.odom_fresh !== true || m.robot_stopped !== true ||
                retrievalActive || !cargo || cargo.stale ||
                (navState.goal_active || window.missionActiveStates.navigation.has(navState.state)) || m.nav_goal_active === true || !!modeSwitchPromise;
        }
        const badge = document.getElementById('missionStateBadge');
        const retrievalOwnsStatus = retrievalOwnsMissionCard();
        if (badge) badge.style.display = currentOpMode === 'AUTO' || retrievalOwnsStatus ||
            owner.source === 'pallet' || owner.stale ? 'inline-flex' : 'none';
    }

    function formatElapsed(ms) {
        const seconds = Math.max(0, Math.floor(ms / 1000));
        const hours = Math.floor(seconds / 3600);
        const minutes = Math.floor((seconds % 3600) / 60);
        const remainder = seconds % 60;
        return hours > 0
            ? `${String(hours).padStart(2, '0')}:${String(minutes).padStart(2, '0')}:${String(remainder).padStart(2, '0')}`
            : `${String(minutes).padStart(2, '0')}:${String(remainder).padStart(2, '0')}`;
    }

    function updateMissionElapsed() {
        const el = document.getElementById('navElapsedText');
        if (!el) return;
        const elapsed = missionStartedAt ? Date.now() - missionStartedAt : missionElapsedMs;
        el.textContent = formatElapsed(elapsed);
    }

    function startMissionClock() {
        if (!missionStartedAt) {
            missionStartedAt = Date.now();
            missionElapsedMs = 0;
        }
        updateMissionElapsed();
    }

    function stopMissionClock(reset = false) {
        if (missionStartedAt) missionElapsedMs = Date.now() - missionStartedAt;
        missionStartedAt = 0;
        if (reset) missionElapsedMs = 0;
        updateMissionElapsed();
    }

    // One owner for action, badge, progress and clock. A terminal retrieval
    // heartbeat must yield to the next supervisor run, including after reload.
    function retrievalOwnsMissionCard() {
        return window.getMissionOwner().source === 'retrieval';
    }

    function manualTargetCoordinates(state) {
        const target = state?.target_pose;
        let point = target;
        if (Array.isArray(target?.waypoints) && target.waypoints.length) {
            const index = Math.min(state.waypoint_index || 0, target.waypoints.length - 1);
            point = target.waypoints[index];
        }
        // Named goals (Quick Goals: HOME, P_1, D_1, D_2) show the node name,
        // matching the quick-goal style; unnamed map clicks fall back to
        // coordinates.
        if (point?.name) return point.name;
        if (Number.isFinite(Number(point?.x)) && Number.isFinite(Number(point?.y))) {
            return `(${Number(point.x).toFixed(2)}, ${Number(point.y).toFixed(2)}) m`;
        }
        return point?.name || null;
    }

    // ============================================
    // Mode & Tool Switching
    // ============================================

    function setOpMode(mode) {
        if (!['AUTO', 'MANUAL'].includes(mode)) return Promise.resolve(false);
        desiredOpMode = mode;
        if (mode !== confirmedOpMode) setActiveTool(null);
        if (modeSwitchPromise) {
            updateNavUI();
            return modeSwitchPromise.then(() => confirmedOpMode === mode);
        }
        if (mode === confirmedOpMode) return Promise.resolve(true);

        modeSwitchPromise = (async () => {
            while (confirmedOpMode !== desiredOpMode) {
                const target = desiredOpMode;
                const response = window.sendCommandAndWait
                    ? await window.sendCommandAndWait({ command: 'set_op_mode', mode: target })
                    : { status: 'error', message: 'Command acknowledgement unavailable' };
                if (response.status !== 'ok') {
                    if (typeof window.showToast === 'function') {
                        window.showToast(response.message || 'Could not change operating mode', 'error');
                    }
                    if (desiredOpMode === target) desiredOpMode = confirmedOpMode;
                    continue;
                }

                applyConfirmedOpMode(target);
                console.log('[Nav] Mode confirmed:', target);
            }
            updateNavUI();
            if (typeof updateOpModeDisplay === 'function') updateOpModeDisplay();
        })().finally(() => {
            if (pendingNavState !== null) {
                const pendingState = pendingNavState;
                pendingNavState = null;
                if (pendingState.op_mode !== undefined) {
                    applyConfirmedOpMode(pendingState.op_mode, true);
                }
            }
            modeSwitchPromise = null;
            updateNavUI();
        });
        updateNavUI();
        return modeSwitchPromise.then(() => confirmedOpMode === mode);
    }

    window.syncOperatingMode = function (mode) {
        const nextMode = mode === 'AUTO' ? 'AUTO' : 'MANUAL';
        if (modeSwitchPromise) {
            pendingNavState = { ...(pendingNavState || {}), op_mode: nextMode };
            return;
        }
        applyConfirmedOpMode(nextMode, true);
    };
    window.requestOperatingMode = setOpMode;

    function setActiveTool(tool) {
        if (!tool) {
            toolPending = null;
            toolPositionSet = false;
            window.toolPending = null;
        }
        activeTool = tool;
        window.isCanvasToolActive = !!tool;
        toolSetPose?.classList.toggle('active', tool === 'pose');
        toolSetGoal?.classList.toggle('active', tool === 'goal');
        const c = document.getElementById('mapCanvas');
        if (c) c.style.cursor = tool ? 'crosshair' : 'grab';
    }

    function selectTool(tool) {
        if (tool === 'pose' ? !canSetInitialPose() : !canSendManualGoal()) return;
        console.debug('[NAVTOOL] selectTool:', tool, 'toolPending:', toolPending);
        if (!window.isNavigationActive) {
            if (typeof window.showToast === 'function') window.showToast('Start Navigation first', 'warn');
            return;
        }
        if (currentOpMode !== 'MANUAL') {
            if (typeof window.showToast === 'function') window.showToast('Switch to MANUAL mode to use tools', 'warn');
            return;
        }
        // Toggle: click same tool to cancel
        if (toolPending === tool) {
            cancelToolPending();
            return;
        }
        toolPending = tool;
        toolPositionSet = false;  // Reset position flag
        // Do NOT set toolPendingX/Y yet - wait for first click on map
        // Export for map.js - only set when actually placing
        window.toolPending = null;  // Don't show preview until click on map
        window.toolPendingX = 0;
        window.toolPendingY = 0;
        window.toolPendingYaw = 0;
        setActiveTool(tool);
    }

    function cancelToolPending() {
        toolPending = null;
        toolPositionSet = false;
        toolPendingX = 0;
        toolPendingY = 0;
        toolPendingYaw = 0;
        // Clear window vars
        window.toolPending = null;
        window.toolPendingX = 0;
        window.toolPendingY = 0;
        window.toolPendingYaw = 0;
        showNavOverlayMsg('');
        setActiveTool(null);
    }

    function toggleRouteVisibility() {
        window.showRouteOverlay = !window.showRouteOverlay;
        toolToggleRoute?.classList.toggle('active', window.showRouteOverlay);
        saveNavLayerToggles();
        if (typeof window.renderMap === 'function') window.renderMap();
    }

    // Phase D: mot bang toggle cho ca 4 lop (route + 3 nav layer), docs 6.2
    // Mac dinh TAT het de tranh lag (user yeu cau); trang thai luu localStorage.
    const NAV_LAYER_TOGGLES = [
      { id: 'toolToggleRoute',         flag: 'showRouteOverlay'  },
      { id: 'toolToggleGlobalCostmap', flag: 'showGlobalCostmap' },
      { id: 'toolToggleLocalCostmap',  flag: 'showLocalCostmap'  },
      { id: 'toolTogglePlan',          flag: 'showPlan'          },
    ];
    const NAV_LAYERS_LS_KEY = 'amr.nav.layers';

    function loadNavLayerToggles() {
        try {
            const raw = localStorage.getItem(NAV_LAYERS_LS_KEY);
            if (!raw) return;
            const obj = JSON.parse(raw);
            for (const t of NAV_LAYER_TOGGLES) {
                if (typeof obj[t.flag] === 'boolean') window[t.flag] = obj[t.flag];
            }
        } catch (_e) { /* Safari private mode */ }
    }

    function saveNavLayerToggles() {
        try {
            const obj = {};
            for (const t of NAV_LAYER_TOGGLES) obj[t.flag] = window[t.flag];
            localStorage.setItem(NAV_LAYERS_LS_KEY, JSON.stringify(obj));
        } catch (_e) { /* Safari private mode */ }
    }

    function applyNavLayerTogglesToButtons() {
        for (const t of NAV_LAYER_TOGGLES) {
            const btn = document.getElementById(t.id);
            if (btn) btn.classList.toggle('active', window[t.flag] === true);
        }
    }

    function onNavLayerToggleClick(flag) {
        window[flag] = !window[flag];
        const btn = document.getElementById(
            NAV_LAYER_TOGGLES.find(t => t.flag === flag)?.id || ''
        );
        if (btn) btn.classList.toggle('active', window[flag]);
        saveNavLayerToggles();
        if (typeof window.renderMap === 'function') window.renderMap();
    }

    function centerMapOnRobot() {
        if (window.mapRenderer && typeof window.mapRenderer.recenter === 'function') {
            window.mapRenderer.recenter();
        }
    }

    // ============================================
    // Mission Control
    // ============================================

    function autoRequestConfirmed(attempt, status) {
        if (!window.missionStatusFresh(status)) return false;
        if (attempt.command === 'mission_cycle_start') {
            return status.run_id && status.run_id !== attempt.runId &&
                window.missionActiveStates.supervisor.has(status.state);
        }
        if (status.run_id !== attempt.runId) return false;
        if (attempt.command === 'mission_cycle_resume') return (status.resume_revision || 0) > attempt.resumeRevision;
        if (attempt.command === 'mission_cycle_cancel') return status.state === 'IDLE' && !status.cancel_pending;
        return status.state === 'RECOVERY' && status.stop_confirmed === true;
    }

    function finishAutoRequest(attempt) {
        if (attempt.superseded) return false;
        if (!autoRequestConfirmed(attempt, window.currentMissionSupervisorStatus)) return false;
        if (!attempt.notified) {
            attempt.notified = true;
            const message = {mission_cycle_start: 'Mission started', mission_cycle_stop: 'Mission paused',
                mission_cycle_resume: 'Mission resumed', mission_cycle_cancel: 'Mission canceled'}[attempt.command];
            window.showToast?.(message, 'success');
        }
        if (window.autoMissionRequest === attempt) window.autoMissionRequest = null;
        window.autoMissionStartPending = false;
        return true;
    }

    async function requestAutoMission(command) {
        const stopping = ['mission_cycle_stop', 'mission_cycle_cancel'].includes(command);
        if (window.autoMissionRequest && !window.autoMissionRequest.uncertain && !stopping) return;
        // Unknown Resume cannot be replayed; full cancellation remains available.
        if (window.autoMissionRequest?.uncertain && command !== 'mission_cycle_cancel' && command !== 'mission_cycle_stop') return;
        const previous = window.currentMissionSupervisorStatus || {};
        if (window.autoMissionRequest) window.autoMissionRequest.superseded = true;
        const attempt = {command, runId: previous.run_id, resumeRevision: previous.resume_revision || 0,
            uncertain: false, notified: false};
        window.autoMissionRequest = attempt;
        window.autoMissionStartPending = command === 'mission_cycle_start';
        updateNavUI();
        let response;
        try {
            response = await window.sendCommandAndWait?.({command, run_mode: 'auto'});
        } catch (error) {
            console.error('Auto mission request failed:', command, error);
            response = {status: 'unknown', uncertain: true};
        }
        if (response?.mission_supervisor) window.acceptMissionSupervisorStatus(response.mission_supervisor);
        if (attempt.superseded) {
            window.updateMissionCycle?.(window.currentMissionSupervisorStatus);
            return;
        }
        if (['ok', 'started'].includes(response?.status)) {
            attempt.uncertain = !finishAutoRequest(attempt);
        } else {
            attempt.uncertain = response?.uncertain === true || response?.status === 'unknown';
            const text = command === 'mission_cycle_resume' ? 'Resume unavailable; check the robot' :
                command === 'mission_cycle_start' ? 'Cannot start mission; check readiness' : 'Stop not confirmed; check the robot';
            if (!attempt.uncertain || !finishAutoRequest(attempt)) {
                window.showToast?.(text, 'warn');
                attempt.notified = true;
            }
            if (!attempt.uncertain && window.autoMissionRequest === attempt) window.autoMissionRequest = null;
            window.autoMissionStartPending = false;
        }
        window.updateMissionCycle?.(window.currentMissionSupervisorStatus);
        updateNavUI();
    }

    async function handleStartMission() {
        if (currentOpMode !== 'AUTO' || window.autoMissionRequest) return;
        const m = window.currentMissionSupervisorStatus;
        const owner = window.getMissionOwner();
        if (!window.missionStatusFresh(m) || !m.can_start || !navState.nav2_ready ||
                owner.active || owner.stale || m.route_graph?.loaded !== true) return;
        window.clearPalletRetrievalMissionStatus?.();
        await requestAutoMission('mission_cycle_start');
    }

    async function handleCancelMission() {
        if (window.navigationRequests.cancel || window.missionTransportConnected === false) return;
        window.navigationRequests.cancel = true;
        updateNavUI();
        try { await cancelCurrentMission(); }
        finally { window.navigationRequests.cancel = false; updateNavUI(); }
    }

    async function cancelCurrentMission() {
        window.retrievalResumePending = null;
        const retrieval = window.currentPalletRetrievalStatus;
        const retrievalActive = window.retrievalStartPending || retrieval && [
            'HOMING_LIFT', 'NAVIGATING_TO_D1', 'ALIGNING_TO_SLOT', 'LOADING_DETECTOR',
            'DETECTING', 'PICKING', 'NAVIGATING_TO_D2', 'PLACING_AT_D2',
            'RETURNING_HOME', 'CANCELING', 'RECOVERY'
        ].includes(retrieval.state);
        const retrievalTerminal = retrieval &&
            ['DONE', 'FAILED', 'CANCELED', 'STOPPED'].includes(retrieval.state);
        const supervisor = window.currentMissionSupervisorStatus;
        const returnHomeUnconfirmed = supervisor?.state === 'DONE' &&
            supervisor?.active_mission === 'return_home' &&
            (supervisor?.nav_mission?.state !== 'SUCCEEDED' ||
             supervisor?.nav_goal_active !== false || supervisor?.nav_status_fresh !== true ||
             supervisor?.robot_stopped !== true);
        const supervisorActive = ['GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING',
            'WAITING_SLOT', 'RETURN_HOME', 'RECOVERY', 'STOPPING'].includes(supervisor?.state) ||
            supervisor?.web_control_pending || returnHomeUnconfirmed;

        if (retrievalTerminal && retrievalOwnsMissionCard() && !supervisorActive) {
            window.clearPalletRetrievalMissionStatus?.();
            updateMissionButtons();
            return;
        }
        if (retrievalActive) {
            const result = window.sendCommandAndWait
                ? await window.sendCommandAndWait({ command: 'pallet_retrieval_cancel' })
                : { status: 'error', message: 'Command acknowledgement unavailable' };
            if (result.status === 'canceled') {
                // This is only the STOP acknowledgement. The coordinator may
                // still transition CANCELING -> RECOVERY. Its status must own
                // the card until the authoritative terminal/Recovery update.
                console.info('[Retrieval] Stop acknowledged; awaiting coordinator status');
            } else {
                console.error('[Retrieval] Stop not confirmed:', result.message || result.status);
                window.showToast?.('Retrieval stop not confirmed. Check robot status.', 'error');
            }
            window.updateMissionCycle?.(window.currentMissionSupervisorStatus);
            return;
        }
        if (currentOpMode === 'AUTO' && (supervisorActive || window.autoMissionRequest)) {
            await requestAutoMission(supervisor?.state === 'RECOVERY' || supervisor?.cancel_pending || supervisor?.web_control_action === 'mission_cycle_cancel' || window.autoMissionRequest?.uncertain ? 'mission_cycle_cancel' : 'mission_cycle_stop');
            return;
        }
        if (currentOpMode === 'AUTO') {
            window.showToast?.('Mission state unavailable', 'warn');
            return;
        }
        const response = await window.sendCommandAndWait?.({command: 'nav_cancel'});
        if (!response || !['canceled', 'ok'].includes(response.status)) {
            window.showToast?.(response?.message || 'Cancellation outcome unknown; verify robot state', 'error');
        }
    }

    function canResumeMission() {
        const owner = window.getMissionOwner();
        const m = owner.status;
        if (owner.stale || !window.missionStatusFresh(m)) return false;
        if (owner.source === 'supervisor' && (window.autoMissionRequest || m.stop_confirmed === false || m.web_control_pending)) return false;
        if (owner.source === 'supervisor' && window.latestNavState &&
                (window.latestNavState.goal_active || window.missionActiveStates.navigation.has(window.latestNavState.state))) return false;
        if (owner.source === 'retrieval' && (window.retrievalResumePending || m.resuming)) return false;
        return owner.source === 'pallet' ? m.recovery?.available === true :
            ['supervisor', 'retrieval'].includes(owner.source) && m.state === 'RECOVERY' && m.resume_available === true;
    }

    async function handleResumeMission() {
        if (window.autoMissionRequest && !window.autoMissionRequest.uncertain) return;
        if (!canResumeMission()) {
            window.showToast?.('Resume unavailable', 'warn');
            return;
        }
        const owner = window.getMissionOwner();
        if (owner.source === 'supervisor') {
            await requestAutoMission('mission_cycle_resume');
            return;
        }
        if (owner.source !== 'pallet' && currentOpMode !== 'AUTO' && !(await setOpMode('AUTO'))) return;
        if (!canResumeMission()) return;
        const attempt = owner.source === 'retrieval' ? {
            runId: owner.status.run_id, revision: owner.status.resume_revision || 0, uncertain: false
        } : null;
        if (attempt) window.retrievalResumePending = attempt;
        window.autoMissionStartPending = owner.source === 'supervisor';
        let response, requestSent = false;
        try {
            updateMissionButtons();
            requestSent = typeof window.sendCommandAndWait === 'function';
            response = requestSent ? await window.sendCommandAndWait({command: 'mission_cycle_resume'}) :
                {status: 'error', message: 'Command acknowledgement unavailable'};
        } catch (error) {
            response = {status: 'error', message: error.message || 'Could not request Resume'};
        } finally {
            const accepted = ['ok', 'started'].includes(response?.status);
            const uncertain = requestSent && /timed out|timeout|acknowledgement unavailable/i.test(response?.message || '');
            if (attempt && window.retrievalResumePending === attempt) {
                if (!accepted && !uncertain) window.retrievalResumePending = null;
                else attempt.uncertain = uncertain;
                window.resolveRetrievalResume?.(window.currentPalletRetrievalStatus);
            }
            if (!accepted) window.autoMissionStartPending = false;
            // Render through the same owner snapshot; no raw RECOVERY payload
            // can overwrite an in-flight Resume with a fake new-start display.
            window.updateMissionCycle?.(window.currentMissionSupervisorStatus);
        }
        if (!['ok', 'started'].includes(response?.status)) {
            if (!response?.command) window.showToast?.(response?.message || 'Resume unavailable', 'warn');
            if (typeof sendCommand === 'function') sendCommand({command: 'get_nav_state'});
        }
    }

    window.handleMissionCycleCommandResponse = function (response) {
        if (response?.status === 'error' &&
            ['mission_cycle_start', 'mission_cycle_resume'].includes(response.command)) {
            window.autoMissionStartPending = false;
            updateMissionButtons();
        }
    };

    function updateMissionStatus(action, target) {
        const owner = window.getMissionOwner();
        if (owner.stale || (owner.active && owner.source !== 'navigation')) return;
        const navActionText = document.getElementById('navActionText');
        const navTargetText = document.getElementById('navTargetText');

        if (navActionText) navActionText.textContent = action;
        if (navTargetText) navTargetText.textContent = target;
    }

    // ============================================
    // Quick Goals
    // ============================================

    function setupQuickGoals() {
        const navQuickMissions = document.getElementById('navQuickMissions');
        if (!navQuickMissions) return;

        navQuickMissions.querySelectorAll('[data-quick-goal]').forEach(btn => {
            btn.addEventListener('click', () => {
                if (!window.isNavigationActive || currentOpMode !== 'MANUAL') {
                    if (typeof window.showToast === 'function') window.showToast('Start Navigation in MANUAL mode', 'warn');
                    return;
                }

                const nodeAliases = {
                    home: ['NODE_HOME', 'HOME'],
                    p1: ['P_1', 'PICK1'],
                    d1: ['D_1', 'DROP1'],
                    d2: ['D_2', 'DROP2']
                };
                const aliases = nodeAliases[btn.dataset.quickGoal];
                const nodes = window.currentRoute?.nodes;
                const targetNode = aliases && Array.isArray(nodes)
                    ? nodes.find(node => aliases.includes(String(node.name).toUpperCase()))
                    : null;

                if (targetNode) {
                    sendNavGoal({
                        x: targetNode.position.x,
                        y: targetNode.position.y,
                        yaw: targetNode.orientation?.yaw || 0,
                        name: targetNode.name
                    });
                } else {
                    if (typeof window.showToast === 'function') window.showToast('Node not found in route', 'warn');
                }
            });
        });
    }

    // ============================================
    // Navigation Goal (Send to Backend)
    // ============================================

    function canSendManualGoal() {
        const owner = window.getMissionOwner();
        return window.missionStatusFresh(window.latestNavState) && window.isNavigationActive &&
            currentOpMode === 'MANUAL' && navState.nav2_ready && navState.can_send_goal !== false &&
            !navStopping && !modeSwitchPromise && !window.navigationRequests.goal &&
            !window.navigationRequests.cancel && !navState.goal_active && !navBusyStates.has(navState.state) &&
            !owner.active && !owner.stale;
    }

    function canSetInitialPose() {
        const owner = window.getMissionOwner();
        return window.missionStatusFresh(window.latestNavState) && window.isNavigationActive &&
            currentOpMode === 'MANUAL' && navState.can_set_initial_pose !== false &&
            !navStopping && !modeSwitchPromise && !window.navigationRequests.goal &&
            !window.navigationRequests.cancel && !owner.active && !owner.stale;
    }

    async function sendNavGoal(pose) {
        if (!canSendManualGoal()) return;
        if (!['x', 'y', 'yaw'].every(key => typeof pose[key] === 'number' && Number.isFinite(pose[key]))) {
            window.showToast?.('Goal requires finite x, y and yaw numbers', 'warn');
            return;
        }
        window.navigationRequests.goal = true;
        window.suppressTerminalPalletRetrievalStatus = true;
        pendingGoalName = pose.name || `(${pose.x.toFixed(1)}, ${pose.y.toFixed(1)})`;
        updateNavUI();
        try {
            const response = await window.sendCommandAndWait?.({command: 'nav_goal', pose});
            if (response?.nav_state) window.handleNavStateUpdate(response.nav_state);
            if (!response || response.status !== 'sent') {
                window.showToast?.(response?.message || 'Goal acknowledgement unavailable; verify robot state', 'error');
            }
            // A command response confirms submission only. The task, target,
            // clock and result remain owned by nav_state and Nav2 callbacks.
        } finally {
            window.navigationRequests.goal = false;
            updateNavUI();
        }
    }

    // ============================================
    // Pointer Events (Phase 4 Fix)
    // ============================================

    function canvasXY(e) {
        const c = document.getElementById('mapCanvas');
        if (!c) return { x: 0, y: 0 };
        const r = c.getBoundingClientRect();
        return { x: e.clientX - r.left, y: e.clientY - r.top };
    }

    function onPointerDown(e) {
        console.debug('[NAVTOOL] onPointerDown:', { toolPending, toolPositionSet, isNavActive: window.isNavigationActive });
        if (!toolPending || !window.isNavigationActive || currentOpMode !== 'MANUAL') return;
        if (!e.isPrimary) return;  // ignore 2nd finger

        // Use canvas-relative coords (e.offsetX is relative to e.target, which may be an overlay on touch)
        const cxy = canvasXY(e);
        const w = window.canvasToWorld ? window.canvasToWorld(cxy.x, cxy.y) : null;
        if (!w) {
            if (typeof window.showToast === 'function') window.showToast('No map available', 'warn');
            return;
        }
        e.preventDefault();
        e.stopPropagation();
        // keep receiving move/up even if the pointer leaves the viewport while aiming
        try { navViewport?.setPointerCapture(e.pointerId); } catch (_e) { /* not critical */ }

        toolConfirmOnRelease = toolPositionSet;
        if (!toolPositionSet) {
            // First click: set position and start aiming
            toolPendingX = w.x;
            toolPendingY = w.y;
            toolPendingMouseX = e.clientX;
            toolPendingMouseY = e.clientY;
            toolPositionSet = true;
            // Phase D 6.4: BO chu huong dan tool. Khi click Set Initial Pose / Set Nav Goal,
            // KHONG hien "Move mouse to aim, click to confirm, ESC to cancel".
            // Luong click-re-click giu nguyen, chi xoa dong showNavOverlayMsg o day.
        }
        // Export for map.js preview
        window.toolPending = toolPending;
        window.toolPendingX = toolPendingX;
        window.toolPendingY = toolPendingY;
        window.toolPendingYaw = toolPendingYaw;

        if (typeof window.renderMap === 'function') window.renderMap();
    }

    function onPointerMove(e) {
        if (!toolPending || !toolPositionSet) return;
        if (!e.isPrimary) return;

        // Calculate yaw from direction from click position to current mouse position
        const dx = e.clientX - toolPendingMouseX;
        const dy = e.clientY - toolPendingMouseY;
        const dist = Math.sqrt(dx * dx + dy * dy);

        // Only update yaw if moved enough (threshold: 5px)
        if (dist > 5) {
            // Top-down view: negate dy for intuitive direction
            toolPendingYaw = Math.atan2(-dy, dx);
            window.toolPendingYaw = toolPendingYaw;
        }
        if (typeof window.renderMap === 'function') window.renderMap();
    }

    function onPointerUp(e) {
        console.debug('[NAVTOOL] onPointerUp:', { toolPending, toolPositionSet });
        if (!toolPending || !toolPositionSet) return;
        if (!e.isPrimary) return;
        if (!toolConfirmOnRelease) return;

        // Second click: confirm and send
        const tool = toolPending;
        const pose = {
            x: toolPendingX,
            y: toolPendingY,
            yaw: toolPendingYaw
        };
        console.debug('[NAVTOOL] Sending:', tool, pose);

        // Reset state
        toolPending = null;
        toolPositionSet = false;
        toolPendingX = 0;
        toolPendingY = 0;
        toolPendingYaw = 0;
        window.toolPending = null;
        window.toolPendingX = 0;
        window.toolPendingY = 0;
        window.toolPendingYaw = 0;
        showNavOverlayMsg('');

        // Turn off tool
        setActiveTool(null);

        // Send command
        if (tool === 'pose') {
            if (canSetInitialPose() && typeof sendCommand === 'function') sendCommand({ command: 'set_initial_pose', pose });
        } else if (tool === 'goal') {
            sendNavGoal(pose);
        }
    }

    function cancelDrag() {
        window.navDragState.isDragging = false;
        setActiveTool(null);
        if (typeof window.renderMap === 'function') window.renderMap();
    }

    // ============================================
    // Map Viewport Management
    // ============================================

    function showNavMapViewport() {
        if (window.mapRenderer && typeof window.mapRenderer.moveToViewport === 'function') {
            window.mapRenderer.moveToViewport('navigation');
            // Force render after viewport move
            setTimeout(() => {
                if (typeof window.renderMap === 'function') window.renderMap();
            }, 50);
        }
    }

    // ============================================
    // Navigation Start - ensure canvas is in correct viewport
    // ============================================

    const _origStartNavigation = startNavigation;
    startNavigation = function() {
        _origStartNavigation();
        // Move canvas to nav viewport immediately
        showNavMapViewport();
    };

    // ============================================
    // Overlay Messages
    // ============================================

    function showNavOverlayMsg(msg) {
        const overlay = document.getElementById('navMapOverlayMsg');
        if (overlay) {
            overlay.textContent = msg;
            overlay.style.display = msg ? 'block' : 'none';
        }
    }

    // ============================================
    // Position Update
    // ============================================

    window.updateNavPosition = function () {
        const valid = window.robotPose && !window.robotPose.stale &&
            window.missionTransportConnected !== false &&
            (!window.isNavigationActive || window.robotPose.frame_id === 'map');
        if (!valid) {
            for (const id of ['navPosX', 'navPosY', 'navPosYaw']) {
                const el = document.getElementById(id);
                if (el) { el.textContent = '--'; el.classList.add('muted'); }
            }
            return;
        }
        const posX = document.getElementById('navPosX');
        const posY = document.getElementById('navPosY');
        const posYaw = document.getElementById('navPosYaw');

        if (posX) { posX.textContent = window.robotPose.position.x.toFixed(2); posX.classList.remove('muted'); }
        if (posY) { posY.textContent = window.robotPose.position.y.toFixed(2); posY.classList.remove('muted'); }
        if (posYaw) {
            posYaw.textContent = (window.robotPose.orientation.yaw * 180 / Math.PI).toFixed(1);
            posYaw.classList.remove('muted');
        }
    };

    // ============================================
    // Cargo Card (from /pallet_mission/status, 1Hz)
    // m: { state, cargo (bool|null), us_cm, cargo_class, cargo_type,
    //      cargo_h_cm, lift_cm, stale } | undefined
    // Hiển thị có box hay không (theo siêu âm/cargo) + phân loại box
    // MEDIUM (A) / HIGH (B) khi đang mang.
    // ============================================

    const CARGO_CLASS_LABEL = {
        medium: 'MEDIUM',
        tall: 'TALL',
        unknown: '?'
    };

    window.updateCargo = function (m) {
        window.currentCargoTelemetry = window.stampMissionStatus(m);
        window.updateMissionCycle?.(window.currentMissionSupervisorStatus);
        updateMissionButtons();
        const st = document.getElementById('navCargoStatus');
        const tx = document.getElementById('navCargoText');
        if (!st || !tx) return;

        const noMission = !m || m.state === undefined;
        if (noMission) {
            st.className = 'cargo-status empty stale mt-2';
            tx.textContent = 'CARGO UNKNOWN';
            return;
        }
        if (m.stale) {
            st.className = 'cargo-status empty stale mt-2';
            tx.textContent = 'CARGO UNKNOWN · STATUS STALE';
            return;
        }

        const us = (m.us_cm != null && Number.isFinite(Number(m.us_cm)))
            ? Number(m.us_cm) : null;
        const pickActive = m.mode === 'pick' && m.cargo !== true &&
            (m.us_pick_confirmed ||
                ['DETECTING', 'NAVIGATING', 'SETTLING', 'ALIGNING', 'INSERTING',
                    'LIFTING', 'BACKING', 'HOMING'].includes(m.state));

        if (m.cargo === true) {
            // Đang mang box: hiện loại (MEDIUM/TALL) + kích thước nếu có.
            const cls = CARGO_CLASS_LABEL[m.cargo_class] || 'CLASS ?';
            const h = m.cargo_h_cm != null ? ` ${Math.round(m.cargo_h_cm)}cm` : '';
            st.className = 'cargo-status loaded mt-2';
            tx.textContent = `BOX ${cls}${h}`;
        } else if (pickActive) {
            st.className = 'cargo-status detecting mt-2';
            if (m.us_pick_confirmed) {
                st.classList.add('confirmed');
                tx.textContent = us != null && m.us_fresh
                    ? `PALLET CONFIRMED (${us.toFixed(0)}cm)` : 'PALLET CONFIRMED';
            } else if (m.state === 'DETECTING') {
                tx.textContent = 'DETECTING PALLET';
            } else if (m.us_fresh && us != null) {
                tx.textContent = `CHECKING PALLET (${us.toFixed(0)}cm)`;
            } else {
                tx.textContent = 'CHECKING PALLET · ULTRASONIC WAIT';
            }
        } else if (m.cargo === false) {
            // Siêu âm chỉ quyết định pallet trong pick; ngoài pha pick đây là
            // khoảng cách đo được, không phải kết luận có/không có pallet.
            st.className = 'cargo-status empty mt-2';
            tx.textContent = m.us_fresh && us != null
                ? `NO CARGO · ${us.toFixed(0)}cm` : 'NO CARGO';
        } else {
            st.className = 'cargo-status empty mt-2';
            tx.textContent = us != null ? `-- (${us.toFixed(0)}cm)` : '--';
        }

        // Dim the card while the status feed has gone stale (mission stopped).
        st.classList.toggle('stale', !!m.stale);
    };

    function missionStateLabel(state, mission) {
        if (state === 'PICKING' && mission?.pm_state === 'DETECTING') return 'Detecting pallet';
        if (state === 'PICKING' && mission?.pm_state === 'READY') return 'Pallet detected';
        return ({
            IDLE: 'Ready', GOTO_PICK: 'To pickup', PICKING: 'Picking',
            GOTO_D: 'To drop-off', DROPPING: 'Placing',
            WAITING_SLOT: 'Waiting slot', RETURN_HOME: 'Returning home',
            RECOVERY: 'Recovery', DONE: 'Done', FAILED: 'Failed',
            STOPPED: 'Ready'
        })[state] || 'Unavailable';
    }

    function missionMessageEnglish(m) {
        const state = m?.state;
        const dest = m?.active_dest || 'pickup';
        const cycle = m?.cycle || {};
        const missionCount = Number.isFinite(cycle.target_quantity)
            ? ` (${cycle.placed || 0}/${cycle.target_quantity} placed)` : '';
        if (state === 'RECOVERY') {
            const raw = String(m?.msg || '').toLowerCase();
            if (raw.includes('operator_cancel')) return 'Canceled by operator';
            if (raw.includes('camera') || raw.includes('detector') || raw.includes('detect')) {
                return 'Pallet detection unavailable';
            }
            if (raw.includes('shelf') || raw.includes('cam bien ke') || raw.includes('cảm biến kệ')) {
                return 'Shelf sensor unavailable';
            }
            if (raw.includes('drop_slot') || raw.includes('drop slot') || raw.includes('o phu hop')) {
                return 'No suitable drop slot';
            }
            if (raw.includes('pose')) return 'Robot pose unavailable';
            if (raw.includes('return_home') || raw.includes('ve home')) return 'Return home failed';
            if (raw.includes('drop failed')) return 'Drop failed; inspect pallet and lift';
            if (raw.includes('nav2 aborted') || raw.includes('nav2 canceled')) {
                return 'Navigation goal ended; inspect the robot, then Resume';
            }
            if (raw.includes('pallet_mission') || raw.includes('pallet mission') ||
                raw.includes('lift') || raw.includes('cargo')) return 'Pallet or lift needs check';
            if (raw.includes('mission_manager') || raw.includes('navigation service')) {
                return 'Navigation service unavailable';
            }
            if (raw.includes('timeout') || raw.includes('qua thoi gian')) return 'Timed out';
            return 'Action needed';
        }
        if (state === 'IDLE') {
            const nav = window.latestNavState || navState;
            if (currentOpMode === 'AUTO') {
                if (!nav.nav2_ready) return 'Awaiting Nav2';
                if (m.route_graph?.loaded !== true) return 'Awaiting route graph';
                return 'Ready';
            }
            return 'Ready';
        }
        if (state === 'STOPPED') return 'Ready';
        if (state === 'DONE') return `Mission complete (${cycle.placed || 0} placed)`;
        if (state === 'FAILED') {
            if (String(m?.msg || '').includes('Mission incomplete')) return m.msg;
            return 'Mission failed';
        }
        if (state === 'WAITING_SLOT') return 'Waiting for free slot';
        if (state === 'GOTO_PICK') return `To ${dest}${missionCount}`;
        if (state === 'PICKING' && m?.pm_state === 'DETECTING') {
            return `Detecting pallet at ${dest}${missionCount}`;
        }
        if (state === 'PICKING' && m?.pm_state === 'READY') {
            return `Pallet detected at ${dest}; starting pickup${missionCount}`;
        }
        if (state === 'PICKING') return `Picking pallet at ${dest}${missionCount}`;
        if (state === 'GOTO_D') return `To shelf at ${m.active_dest || 'D_1'}${missionCount}`;
        if (state === 'DROPPING') return `Placing pallet on shelf${missionCount}`;
        if (state === 'RETURN_HOME') return `Returning to HOME${missionCount}`;
        return 'Waiting';
    }
    function shortRecentMissionName(name) {
        return String(name || 'Mission')
            .replace(/^Retrieve pallet from /i, 'Retrieve ')
            .replace(/^Align with rack slot /i, 'Align ')
            .replace(/^Detect pallet at /i, 'Detect ')
            .replace(/^Place pallet at /i, 'Place ')
            .replace(/^Load pallet detector$/i, 'Load detector')
            .replace(/^Return to HOME$/i, 'Return HOME');
    }
    function renderRecentMissions(items) {
        const body = document.getElementById('navRecentMissions');
        const empty = document.getElementById('navMissionEmpty');
        if (!body || !empty) return;
        body.replaceChildren();
        const rows = Array.isArray(items) ? items : [];
        empty.style.display = rows.length ? 'none' : 'block';
        for (const item of rows) {
            const tr = document.createElement('tr');
            const cells = [
                [item.time || '--', 'mission-time'],
                [shortRecentMissionName(item.name), 'mission-name'],
                [item.status || '--', `mission-outcome ${item.status === 'Completed' ? 'completed' :
                    item.status === 'Recovery Required' ? 'recovery' :
                        item.status === 'Resumed' ? 'resumed' :
                            item.status === 'Canceled' ? 'canceled' :
                                item.status === 'Failed' ? 'failed' : ''}`]
            ];
            for (const [value, className] of cells) {
                const td = document.createElement('td');
                td.className = className;
                td.textContent = value;
                tr.appendChild(td);
            }
            body.appendChild(tr);
        }
    }

    window.resetSessionUI = function () {
        window.retrievalResumePending = null;
        // Recent Missions is history, not current Nav2 session state.
        // It is refreshed from the persistent Bridge snapshot on reconnect.
        if (typeof window.resetLLMSession === 'function') window.resetLLMSession();
    };

    // Keep the original Nav2 detail card in Manual. In Auto, the supervisor's
    // state and destination replace the manual goal name/coordinates.
    window.updateMissionCycle = function (m) {
        window.currentMissionSupervisorStatus = m || null;
        if (typeof window.updateRackInventory === 'function') {
            window.updateRackInventory(window.currentMissionSupervisorStatus);
        }
        const attempt = window.autoMissionRequest;
        if (attempt?.uncertain) finishAutoRequest(attempt);
        const owner = window.getMissionOwner();
        const view = window.getMissionPresentation(owner);
        m = owner.status;
        const fresh = !owner.stale && window.missionStatusFresh(m);
        const state = fresh ? (m.state || 'UNKNOWN') : 'UNKNOWN';
        const badge = document.getElementById('missionStateBadge');
        const action = document.getElementById('navActionText');
        const target = document.getElementById('navTargetText');
        const detail = document.getElementById('navTaskDetail');
        if (detail) {
            // AUTO / retrieval statuses use the compact action/target/progress/elapsed
            // fields. Keep ROS diagnostic messages in logs, never in the mission card.
            const message = currentOpMode === 'MANUAL' && owner.source === 'navigation' && !owner.stale
                ? String(owner.status.task_message || '') : '';
            detail.textContent = message;
            detail.hidden = !message;
        }

        const retrievalOwns = retrievalOwnsMissionCard();
        const taskInfo = document.getElementById('navTaskInfo');
        // AUTO Idle must retain the same four status fields as MANUAL.
        if (taskInfo) taskInfo.hidden = false;
        if (owner.source === 'supervisor') {
            if (badge) {
                badge.style.display = 'inline-flex';
                badge.textContent = fresh
                    ? view.label
                    : 'Unavailable';
                badge.className = 'nav-status-badge' + (state === 'RECOVERY' ? ' warning' :
                    ['GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING', 'WAITING_SLOT', 'RETURN_HOME'].includes(state)
                        ? ' active' : '');
            }
            if (action) {
                const recoveryAction = fresh && state === 'RECOVERY';
                action.classList.toggle('mission-action-recovery', recoveryAction);
                action.classList.toggle('accent', !recoveryAction);
                action.textContent = fresh ? view.label : 'Unavailable';
            }
            if (target) target.textContent = state === 'IDLE' ? '--' : view.target;
            if (fresh && Number.isFinite(m.elapsed_s)) {
                missionElapsedMs = Math.max(0, m.elapsed_s * 1000);
                missionStartedAt = !['IDLE', 'RECOVERY', 'STOPPING', 'DONE', 'FAILED', 'STOPPED'].includes(state)
                    ? Date.now() - missionElapsedMs : 0;
                updateMissionElapsed();
            }
            if (['GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING',
                    'WAITING_SLOT', 'RETURN_HOME'].includes(state)) {
                window.autoMissionStartPending = false;
                startMissionClock();
            } else if (['RECOVERY', 'STOPPING'].includes(state)) {
                stopMissionClock(false);
            } else if (state === 'IDLE') {
                window.autoMissionStartPending = false;
                stopMissionClock(true);
            } else if (state === 'STARTING') {
                startMissionClock();
            } else if (['DONE', 'FAILED', 'STOPPED'].includes(state)) {
                window.autoMissionStartPending = false;
                stopMissionClock(state === 'STOPPED' && currentOpMode === 'MANUAL');
            } else {
                stopMissionClock(false);
            }
            renderMissionProgress(m, fresh);
        } else {
            // While a retrieval mission is active, the retrieval
            // renderer owns this card (and its badge) — do not hide the
            // badge back and forth every tick.
            if (owner.source === 'retrieval' && owner.stale) {
                // Keep the retrieval card visible during brief stale telemetry;
                // do not paint AUTO/Manual data over an unconfirmed recovery.
                if (badge) {
                    badge.style.display = 'inline-flex';
                    badge.textContent = 'Unavailable';
                    badge.className = 'nav-status-badge warning';
                }
            } else if (badge && !retrievalOwnsMissionCard()) badge.style.display = 'none';
            if (action && owner.source !== 'retrieval') {
                action.classList.remove('mission-action-recovery');
                action.classList.add('accent');
            }
            renderOtherMissionOwner(owner);
        }

        window.notifyMissionError(owner, view);
        const recentStatus = window.currentMissionSupervisorStatus;
        const recentHistory = recentStatus?.navigation_session_id === window.currentNavigationSessionId
            ? recentStatus.recent_missions : null;
        renderRecentMissions(Array.isArray(recentHistory) ? recentHistory : []);
        updateMissionButtons();
        if (typeof window.updateOverviewNavigationState === 'function') {
            window.updateOverviewNavigationState(window.latestNavState, window.currentMissionSupervisorStatus);
        }
        if (owner.source === 'retrieval') {
            // Pending start has a synthetic owner before the coordinator's
            // first heartbeat; render it without requiring a raw status yet.
            window.renderPalletRetrievalMissionStatus(owner.status);
        }
    };

    function renderOtherMissionOwner(owner) {
        if (owner.source === 'retrieval') return;
        const unavailable = owner.stale;
        const state = owner.status.state;
        const view = window.getMissionPresentation(owner);
        const label = view.label;
        const action = document.getElementById('navActionText');
        const target = document.getElementById('navTargetText');
        const badge = document.getElementById('missionStateBadge');
        const progressLabel = document.getElementById('navProgressLabel');
        const text = document.getElementById('navProgressText');
        const bar = document.getElementById('navProgressBar');
        if (action) action.textContent = label;
        if (target) target.textContent = owner.source === 'navigation' && !unavailable
            ? (manualTargetCoordinates(owner.status) || '--') : view.target;
        if (badge) {
            badge.style.display = 'inline-flex';
            badge.textContent = owner.source === 'pallet' && !unavailable
                ? ({pick: 'PICK', drop: 'DROP', retrieval_align: 'RETRIEVAL'}[owner.status.mode] || label)
                : label;
            badge.className = 'nav-status-badge' + (unavailable || state === 'FAILED'
                ? ' warning' : owner.active ? ' active' : '');
        }
        const progress = window.getMissionProgress(owner);
        if (progressLabel) progressLabel.textContent = progress?.label || 'Navigation progress';
        const percent = unavailable || state === 'UNKNOWN' ? null : progress ? progress.percent : owner.status.nav2_running
            ? (state === 'SUCCEEDED' ? 100 : Math.min(99, Math.max(0, Number(owner.status.progress) || 0) * 100)) : null;
        if (text) text.textContent = progress ? progress.text : percent === null ? '--' : `${Math.floor(percent)}%`;
        if (bar && !unavailable) bar.style.width = percent === null ? '0%' : `${percent}%`;
        if (owner.active) startMissionClock();
        else if (!unavailable) stopMissionClock(false);
    }

    function renderMissionProgress(m, supervisorFresh) {
        // While the retrieval card owns the mission status (see the
        // single-owner rule), only it may write the progress bar/label.
        if (retrievalOwnsMissionCard()) return;
        const bar = document.getElementById('navProgressBar');
        const text = document.getElementById('navProgressText');
        const label = document.getElementById('navProgressLabel');
        const progress = window.getMissionProgress();
        if (label) label.textContent = progress?.label || 'Mission progress';
        if (bar) bar.style.width = progress?.percent == null ? '0%' : `${progress.percent}%`;
        if (text) text.textContent = progress?.text || '--';
    }

    window.renderPalletRetrievalMissionStatus = function () {
        const owner = window.getMissionOwner();
        if (owner.source !== 'retrieval' || owner.stale) return;
        const retrieval = owner.status;
        const view = window.getMissionPresentation(owner);
        const progress = window.getMissionProgress(owner);
        const state = retrieval.state;
        const paused = state === 'RECOVERY';
        const action = document.getElementById('navActionText');
        const target = document.getElementById('navTargetText');
        const badge = document.getElementById('missionStateBadge');
        const label = document.getElementById('navProgressLabel');
        const text = document.getElementById('navProgressText');
        const bar = document.getElementById('navProgressBar');
        if (paused || ['FAILED', 'DONE', 'CANCELED', 'STOPPED'].includes(state)) stopMissionClock(false);
        else startMissionClock();
        if (action) {
            action.classList.toggle('mission-action-recovery', paused && !view.resuming || !!retrieval.recovery_required && !view.resuming);
            action.classList.toggle('accent', view.resuming || !retrieval.recovery_required && !paused);
            action.textContent = view.label;
            // Recovery diagnostics belong in logs, not the status card tooltip.
            action.title = '';
        }
        if (target) target.textContent = view.target;
        if (badge) {
            badge.style.display = 'inline-flex';
            badge.textContent = view.resuming ? 'RESUMING' : paused ? 'RECOVERY' :
                retrieval.recovery_required ? 'RECOVERY REQUIRED' : state === 'FAILED' ? 'RETRIEVAL FAILED' :
                state === 'DONE' ? 'COMPLETE' : ['CANCELED', 'STOPPED'].includes(state) ? state : 'PALLET RETRIEVAL';
            badge.className = 'nav-status-badge' + (view.resuming ? ' active' :
                state === 'FAILED' || paused || retrieval.recovery_required ? ' warning' :
                ['DONE', 'CANCELED', 'STOPPED'].includes(state) ? '' : ' active');
        }
        if (label) label.textContent = progress.label;
        if (text) text.textContent = progress.text;
        if (bar) bar.style.width = Number.isFinite(progress.percent) ? `${progress.percent}%` : '0%';
        window.notifyMissionError(owner, view);
    };

    window.clearPalletRetrievalMissionStatus = function (resetNavigation = false) {
        window.retrievalResumePending = null;
        window.suppressTerminalPalletRetrievalStatus = true;
        window.currentPalletRetrievalStatus = null;
        retrievalProgressStep = 0;
        if (resetNavigation) {
            isMissionActive = false;
            lastManualTarget = '--';
            updateMissionButtons();
        }
        stopMissionClock(false);
        window.updateMissionCycle(window.currentMissionSupervisorStatus);
    };

    // ============================================
    // Camera Feed Handler
    // ============================================

    window.updateCameraFeed = function(data) {
        if (cameraMode === 'off' || !data) return;

        // Kiểm tra đúng trường cam_mode và data như backend gửi
        if (data.cam_mode !== cameraMode) return;

        const navCameraFeed = document.getElementById('navCameraFeed');
        if (!navCameraFeed) return;

        let img = navCameraFeed.querySelector('img');
        if (!img) {
            navCameraFeed.innerHTML = '';
            img = document.createElement('img');
            img.style.cssText = 'width: 100%; height: 100%; object-fit: cover;';
            navCameraFeed.appendChild(img);
        }
        img.src = 'data:image/jpeg;base64,' + data.data;
    };

    // ============================================
    // Update UI (expose for multi-client sync via window)
    // ============================================

    // The updateNavUI function is already defined above and accessible via window.updateNavUI
    // when called from ros_connection.js

    // ============================================
    // WebSocket Handlers (Phase 5 Fix)
    // ============================================

    window.handleNavStateUpdate = function (data) {
        if (data.bridge_session_id && navState.bridge_session_id === data.bridge_session_id &&
                Number.isFinite(data.revision) && data.revision <= navState.revision) return;
        if (data.op_mode !== undefined) {
            const nextMode = data.op_mode === 'AUTO' ? 'AUTO' : 'MANUAL';
            if (modeSwitchPromise) {
                pendingNavState = data;
            } else if (nextMode !== confirmedOpMode) {
                applyConfirmedOpMode(nextMode, true);
            }
        }

        const wasRunning = navState ? navState.nav2_running : false;
        const wasSessionActive = window.isNavigationActive;
        const nowRunning = data.nav2_running === true;
        // Nav2 DDS discovery is not proof that this Web Bridge still owns a
        // running session. After Stop, an action server can remain discoverable
        // briefly; show Welcome, but keep Start blocked until it disappears.
        const sessionActive = nowRunning && data.process_owned !== false;

        navState = {
            ...data,
            state: data.state || 'IDLE',
            progress: data.progress || 0,
            nav2_running: nowRunning,
            nav2_ready: data.nav2_ready || false,
            waypoint_index: data.waypoint_index || 0,
            waypoint_total: data.waypoint_total || 0,
            map_id: data.map_id || null,
            route_id: data.route_id || null,
            target_pose: data.target_pose || null,
            distance_remaining: data.distance_remaining ?? null,
            estimated_time_remaining: data.estimated_time_remaining ?? null,
            mission_type: data.mission_type || null,
        };
        navState.received_at = Date.now();
        window.latestNavState = navState;
        if (data.navigation_session_id &&
                data.navigation_session_id !== window.currentNavigationSessionId) {
            window.currentNavigationSessionId = data.navigation_session_id;
            renderRecentMissions([]);
        }
        if (currentOpMode === 'MANUAL' && Number.isFinite(data.elapsed_s)) {
            missionElapsedMs = Math.max(0, data.elapsed_s * 1000);
            missionStartedAt = data.goal_active ? Date.now() - missionElapsedMs : 0;
        }

        // Phase D 6.5: Nav2 chet/Stop tu client khac -> xoa lop
        if ((wasSessionActive && !sessionActive || wasRunning && !nowRunning) &&
                typeof window.clearNavLayers === 'function') {
            window.clearNavLayers();
        }
        if ((wasSessionActive && !sessionActive || wasRunning && !nowRunning) &&
                typeof window.resetSessionUI === 'function') {
            window.resetSessionUI();
        }

        window.isNavigationActive = sessionActive;
        if (sessionActive) navStarting = false;
        // Apply the session visibility change on direct command snapshots too,
        // not only on WebSocket broadcasts from ros_connection.js.
        if (wasSessionActive !== sessionActive) {
            if (sessionActive) showNavActiveUI();
            else hideNavActiveUI();
        }
        isMissionActive = data.goal_active === true || navBusyStates.has(navState.state);
        if (isMissionActive || !nowRunning) setActiveTool(null);
        if (navState.map_id && navState.map_id !== window.selectedNavMap) {
            window.currentRoute = null;
            window.mapData = null;
            ++navMapLoadSeq;
            navMapLoading = false;
        }
        if (navState.route_id !== window.selectedNavRoute) window.currentRoute = null;
        if (navState.map_id) {
            window.selectedNavMap = navState.map_id;
            window.selectedNavRoute = navState.route_id;

            // Khi F5/reload: load PGM map neu chua co
            if (navState.nav2_running && !navStopping && (!window.mapData || !window.mapData.is_pgm_raw)) {
                loadNavMap();
            }

            // Cap nhat Overview Card khi co mapData
            if (typeof window.updateOverviewMapCard === 'function' && window.mapData) {
                window.updateOverviewMapCard(window.mapData);
            }

            // Khôi phục Route Graph khi F5/reload nếu chưa có
            if (navState.route_id && !window.currentRoute) {
                restoreRouteData(navState.map_id, navState.route_id);
            }
        }

        // Do not reactivate the Navigation layout solely because DDS still
        // discovers an action server after the owned process has exited.

        // B21 Fix: Gọi updateNavUI vô điều kiện để sync nút AUTO/MANUAL
        updateNavUI();

        // Update mission status display
        const navActionText = document.getElementById('navActionText');
        const navTargetText = document.getElementById('navTargetText');
        const navProgressBar = document.getElementById('navProgressBar');
        const navProgressText = document.getElementById('navProgressText');

        // Single-owner rule: while a retrieval mission is active, the
        // retrieval renderer is the only writer of the visible mission card.
        // The raw Nav2 values below (incl. per-leg "Goal reached!") would
        // otherwise paint over it every tick — the flicker. Bookkeeping still
        // runs; only the visible writes are gated.
        const retrievalOwns = window.getMissionOwner().source !== 'navigation';

        if (navActionText && !retrievalOwns) {
            switch (navState.state) {
                case 'IDLE':
                    if (!isMissionActive && currentOpMode === 'MANUAL') navActionText.textContent = 'Idle';
                    if (isMissionActive) {
                        isMissionActive = false;
                        if (currentOpMode === 'MANUAL') stopMissionClock(false);
                        updateMissionButtons();
                    }
                    break;
                case 'SENDING':
                case 'ACCEPTED':
                case 'CANCELING':
                case 'UNKNOWN':
                    navActionText.textContent = window.missionStateLabels[navState.state];
                    break;
                case 'ACTIVE':
                    navActionText.textContent = navState.waypoint_total > 0
                        ? `WP ${navState.waypoint_index + 1}/${navState.waypoint_total}`
                        : 'Navigating';
                    startMissionClock();
                    if (!isMissionActive) { isMissionActive = true; updateMissionButtons(); }
                    break;
                case 'SUCCEEDED':
                    navActionText.textContent = 'Goal reached!';
                    if (currentOpMode === 'MANUAL') stopMissionClock(false);
                    if (isMissionActive) { isMissionActive = false; updateMissionButtons(); }
                    break;
                case 'FAILED':
                    navActionText.textContent = 'Failed';
                    if (currentOpMode === 'MANUAL') stopMissionClock(false);
                    if (isMissionActive) { isMissionActive = false; updateMissionButtons(); }
                    break;
                case 'CANCELED':
                    navActionText.textContent = 'Canceled';
                    if (currentOpMode === 'MANUAL') stopMissionClock(false);
                    if (isMissionActive) { isMissionActive = false; updateMissionButtons(); }
                    break;
            }
        }

        const targetCoordinates = manualTargetCoordinates(navState);
        if (targetCoordinates) lastManualTarget = targetCoordinates;
        if (currentOpMode === 'MANUAL' && !retrievalOwns && navTargetText) navTargetText.textContent = lastManualTarget;

        if (navProgressBar && !retrievalOwns) navProgressBar.style.width = navState.nav2_running
            ? `${navState.progress * 100}%` : '0%';
        if (navProgressText && !retrievalOwns) navProgressText.textContent = navState.nav2_running
            ? `${Math.round(navState.progress * 100)}%` : '--';

        if (typeof window.updateOverviewNavigationState === 'function') {
            window.updateOverviewNavigationState(navState, window.currentMissionSupervisorStatus);
        }
        window.updateMissionCycle(window.currentMissionSupervisorStatus);
        if (window.currentPalletRetrievalStatus) {
            window.renderPalletRetrievalMissionStatus(window.currentPalletRetrievalStatus);
        }
        updateMissionButtons();
        if (typeof window.updateNavPosition === 'function') window.updateNavPosition();
    };

    window.handleNavResult = function (data) {
        // The full session STOP owns goal cancellation and the final notice.
        if (navStopping) return;
        if (data.bridge_session_id && navState.bridge_session_id && data.bridge_session_id !== navState.bridge_session_id) return;
        const key = JSON.stringify([data.goal_id, data.state, data.goal_id ? '' : data.timestamp]);
        if (seenNavResults.has(key)) return;
        seenNavResults.add(key);
        if (seenNavResults.size > 128) seenNavResults.delete(seenNavResults.values().next().value);
        if (data.goal_id && navState.goal_id && data.goal_id !== navState.goal_id) return;
        // A Nav2 leg canceled by AUTO/retrieval is not a user-facing mission
        // result. Only its owning coordinator announces pause/recovery/finish.
        const owner = window.getMissionOwner();
        if (owner.source !== 'navigation' || window.retrievalStartPending ||
                (currentOpMode === 'AUTO' && (window.currentPalletRetrievalStatus &&
                 window.missionActiveStates.retrieval.has(window.currentPalletRetrievalStatus.state))) ||
                (currentOpMode === 'AUTO' && window.currentMissionSupervisorStatus &&
                 (window.missionActiveStates.supervisor.has(window.currentMissionSupervisorStatus.state) ||
                  window.currentMissionSupervisorStatus.run_id))) return;
        console.log('[Nav] Navigation result:', data);

        if (data.state === 'SUCCEEDED') {
            if (typeof window.showToast === 'function') window.showToast('Navigation completed!', 'success');
            if (typeof addActivity === 'function') addActivity('success', `Mission: ${data.target || 'Goal'}`);
        } else if (data.state === 'FAILED') {
            if (typeof window.showToast === 'function') window.showToast(`Failed: ${data.message}`, 'error');
            if (typeof addActivity === 'function') addActivity('error', `Failed: ${data.message}`);
        } else if (data.state === 'CANCELED') {
            if (typeof window.showToast === 'function') window.showToast('Navigation canceled', 'info');
            if (typeof addActivity === 'function') addActivity('info', 'Mission canceled');
        }

        updateMissionButtons();
    };

    // Phase 5 Fix: Command response handler for nav commands
    window.onNavCommandResponse = function (d) {
        const ok = ['started', 'stopped', 'sent', 'canceled', 'ok'].includes(d.status);
        switch (d.command) {
            case 'start_navigation':
                navStarting = false;
                if (!ok) { rollbackStart(d.message || d.status); break; }
                if (d.nav_state) window.handleNavStateUpdate(d.nav_state);
                updateOpModeDisplay?.();
                updateNavUI();
                startMapRequestLoop();
                break;

            case 'nav_goal':
            case 'waypoint_mission':
                if (!ok && !d.handled_by_request) window.showToast?.(d.message || 'Goal submission failed', 'error');
                break;

            case 'set_initial_pose':
                if (typeof window.showToast === 'function') {
                    window.showToast(ok ? 'Initial Pose sent' : (d.message || 'Failed to send Initial Pose'),
                        ok ? 'success' : 'error');
                }
                break;

            case 'nav_cancel':
            case 'emergency_stop':
                if (!ok && !d.handled_by_request && !navStopping && typeof window.showToast === 'function') {
                    window.showToast(d.message || 'Cancel failed', 'error');
                }
                break;

            case 'stop_navigation':
                if (!ok && !d.handled_by_request && !navStopping && typeof window.showToast === 'function') {
                    window.showToast(d.message || 'Failed to stop Navigation', 'error');
                }
                break;
        }
    };

    // ============================================
    // Camera Mode Switching
    // ============================================

    function setCameraMode(mode) {
        if (mode === cameraMode) return;
        cameraMode = mode;

        // Update button states
        if (btnCamRgb) btnCamRgb.classList.toggle('active', mode === 'rgb');
        if (btnCamDepth) btnCamDepth.classList.toggle('active', mode === 'depth');
        if (btnCamDetect) btnCamDetect.classList.toggle('active', mode === 'detect');
        if (btnNavToggleCam) btnNavToggleCam.classList.toggle('active', mode === 'off');

        // Send command to backend - sử dụng start_camera/stop_camera như Documents
        if (typeof sendCommand === 'function') {
            if (mode === 'off') {
                sendCommand({ command: 'stop_camera' });
            } else {
                sendCommand({ command: 'start_camera', mode: mode });
            }
        }

        // Update camera feed display
        const navCameraFeed = document.getElementById('navCameraFeed');
        if (navCameraFeed) {
            if (mode === 'off') {
                navCameraFeed.innerHTML = '<span class="muted" style="font-size: 0.75rem;">Camera Offline</span>';
            } else {
                const label = mode === 'detect'
                    ? 'Pallet detection (needs detector running)...'
                    : `Connecting to ${mode.toUpperCase()}...`;
                navCameraFeed.innerHTML = `<span class="muted" style="font-size: 0.75rem;">${label}</span>`;
            }
        }

        console.log('[Nav] Camera mode:', mode);
    }

    window.setNavigationCameraMode = setCameraMode;

    // ============================================
    // Event Listeners Initialization
    // ============================================

    function initEventListeners() {
        if (!missionElapsedTimer) {
            missionElapsedTimer = window.setInterval(() => {
                updateMissionElapsed();
                // Age cached statuses even when the WebSocket stops updating.
                updateNavUI();
                window.updateNavPosition();
                window.updateMissionCycle(window.currentMissionSupervisorStatus);
            }, 1000);
        }
        updateMissionElapsed();
        // Modal
        btnStartNav?.addEventListener('click', showNavSetupModal);
        btnStopNav?.addEventListener('click', stopNavigation);
        navModalClose?.addEventListener('click', hideNavSetupModal);
        navModalCancel?.addEventListener('click', hideNavSetupModal);
        navModalConfirm?.addEventListener('click', onConfirmClick);
        navMapSelect?.addEventListener('change', onMapSelectChange);
        navRouteSelect?.addEventListener('change', onRouteSelectChange);

        navSetupModal?.addEventListener('click', (e) => {
            if (e.target === navSetupModal) hideNavSetupModal();
        });

        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && navSetupModal?.classList.contains('active')) {
                hideNavSetupModal();
            }
            if (e.key === 'Escape' && window.navDragState?.isDragging) {
                cancelDrag();
            }
        });

        // Mode toggle
        btnModeAuto?.addEventListener('click', () => setOpMode('AUTO'));
        btnModeManual?.addEventListener('click', () => setOpMode('MANUAL'));

        // Camera mode buttons
        btnCamRgb?.addEventListener('click', () => setCameraMode(cameraMode === 'rgb' ? 'off' : 'rgb'));
        btnCamDepth?.addEventListener('click', () => setCameraMode(cameraMode === 'depth' ? 'off' : 'depth'));
        btnCamDetect?.addEventListener('click', () => setCameraMode(cameraMode === 'detect' ? 'off' : 'detect'));
        btnNavToggleCam?.addEventListener('click', () => setCameraMode('off'));

        // Mission buttons
        btnStartMission?.addEventListener('click', handleStartMission);
        btnCancelMission?.addEventListener('click', handleCancelMission);

        btnRecoveryMission?.addEventListener('click', handleResumeMission);
        document.getElementById('btnConfirmForksEmpty')?.addEventListener('click', async () => {
            // Single-click confirmation; no popup or success toast.
            const response = await window.sendCommandAndWait?.({command: 'confirm_forks_empty', confirmed_empty: true});
            if (response?.status !== 'ok') {
                window.showToast?.('Fork confirmation unavailable. Check robot status.', 'error');
            }
        });


        // Tool buttons
        toolSetPose?.addEventListener('click', () => selectTool('pose'));
        toolSetGoal?.addEventListener('click', () => selectTool('goal'));
        toolToggleRoute?.addEventListener('click', toggleRouteVisibility);
        toolCenterMap?.addEventListener('click', centerMapOnRobot);

        // Phase D: 3 nav layer toggle (Global Costmap, Local Costmap, Plan)
        for (const t of NAV_LAYER_TOGGLES) {
            if (t.id === 'toolToggleRoute') continue;  // da co listener rieng
            const btn = document.getElementById(t.id);
            if (btn) btn.addEventListener('click', () => onNavLayerToggleClick(t.flag));
        }
        // Doc trang thai tu localStorage (an toan, try/catch ben trong)
        loadNavLayerToggles();
        applyNavLayerTogglesToButtons();

        // Keyboard: ESC to cancel, Enter to confirm tool
        document.addEventListener('keydown', (e) => {
            if (!toolPending) return;
            if (e.key === 'Escape') {
                cancelToolPending();
            } else if (e.key === 'Enter' && toolPositionSet) {
                const target = e.target;
                if (e.repeat || target && (['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName) || target.isContentEditable)) return;
                e.preventDefault();
                const tool = toolPending;
                const pose = { x: toolPendingX, y: toolPendingY, yaw: toolPendingYaw };
                // Reset state
                toolPending = null;
                toolPositionSet = false;
                toolPendingX = 0;
                toolPendingY = 0;
                toolPendingYaw = 0;
                window.toolPending = null;
                window.toolPendingX = 0;
                window.toolPendingY = 0;
                window.toolPendingYaw = 0;
                showNavOverlayMsg('');
                // Turn off tool
                setActiveTool(null);
                // Send
                if (tool === 'pose') {
                    if (canSetInitialPose() && typeof sendCommand === 'function') sendCommand({ command: 'set_initial_pose', pose });
                } else if (tool === 'goal') {
                    sendNavGoal(pose);
                }
            }
        });

        // Pointer events on nav viewport (Phase 4 Fix)
        navViewport?.addEventListener('pointerdown', onPointerDown);
        navViewport?.addEventListener('pointermove', onPointerMove);
        navViewport?.addEventListener('pointerup', onPointerUp);
        navViewport?.addEventListener('pointercancel', cancelDrag);
        // Long-press on touch opens the browser context menu and cancels the gesture while aiming
        navViewport?.addEventListener('contextmenu', (e) => { if (toolPending) e.preventDefault(); });

        // Quick goals
        setupQuickGoals();
    }

    // ============================================
    // Manual Keyboard Teleop (giữ phím để chạy, thả phím để dừng)
    // Chỉ hoạt động khi: Navigation đang chạy + op mode MANUAL + trang Nav đang hiển thị.
    // Dùng e.code (vị trí phím vật lý) để không bị ảnh hưởng bởi Shift/CapsLock/bộ gõ tiếng Việt.
    // ============================================
    const MANUAL_KEY_MAP = {
        'KeyU':   { linear: 1,  angular: 1 },  'KeyI':   { linear: 1,  angular: 0 },  'KeyO':   { linear: 1,  angular: -1 },
        'KeyJ':   { linear: 0,  angular: 1 },  'KeyK':   { linear: 0,  angular: 0 },  'KeyL':   { linear: 0,  angular: -1 },
        'KeyM':   { linear: -1, angular: 1 },  'Comma':  { linear: -1, angular: 0 },  'Period': { linear: -1, angular: -1 }
    };
    const MANUAL_LINEAR_SPEED = 0.15;   // m/s
    const MANUAL_ANGULAR_SPEED = 0.3;   // rad/s
    const MANUAL_SEND_MS = 100;
    let manualTeleopTimer = null;
    let manualTeleopKey = null;
    let manualVel = { linear: 0, angular: 0 };

    function manualDriveAllowed() {
        const owner = window.getMissionOwner();
        if (owner.active || owner.stale) return false;
        if (!window.isNavigationActive || currentOpMode !== 'MANUAL' || isMissionActive) return false;
        if (!navViewport || navViewport.getClientRects().length === 0) return false;  // trang Nav đang ẩn
        if (navSetupModal?.classList.contains('active')) return false;
        return true;
    }

    function sendManualVelocity() {
        if (typeof sendCommand !== 'function') return;
        if (!manualDriveAllowed()) { stopManualTeleop(); return; }
        sendCommand({
            command: 'teleop',
            linear: MANUAL_LINEAR_SPEED * manualVel.linear,
            angular: MANUAL_ANGULAR_SPEED * manualVel.angular
        });
    }

    function startManualTeleop(code) {
        const v = MANUAL_KEY_MAP[code];
        if (!v) return;
        if (v.linear === 0 && v.angular === 0) { stopManualTeleop(true); return; }  // K = dừng
        manualTeleopKey = code;
        manualVel = v;
        sendManualVelocity();
        if (!manualTeleopTimer) manualTeleopTimer = setInterval(sendManualVelocity, MANUAL_SEND_MS);
    }

    // force=true: luôn gửi lệnh dừng; mặc định chỉ gửi khi đang chạy (không đụng Nav2 lúc rảnh)
    function stopManualTeleop(force) {
        const wasDriving = manualTeleopTimer !== null;
        if (manualTeleopTimer) { clearInterval(manualTeleopTimer); manualTeleopTimer = null; }
        manualTeleopKey = null;
        manualVel = { linear: 0, angular: 0 };
        if ((wasDriving || force === true) && typeof sendCommand === 'function') {
            sendCommand({ command: 'teleop', linear: 0, angular: 0 });
        }
    }

    function onManualKeyDown(e) {
        if (!MANUAL_KEY_MAP[e.code]) return;
        if (e.ctrlKey || e.metaKey || e.altKey) return;
        const t = e.target;
        if (t && (t.tagName === 'INPUT' || t.tagName === 'TEXTAREA' || t.tagName === 'SELECT' || t.isContentEditable)) return;
        if (!manualDriveAllowed()) return;
        e.preventDefault();
        if (e.repeat) return;  // giữ phím: timer đã gửi liên tục
        startManualTeleop(e.code);
    }

    function onManualKeyUp(e) {
        if (e.code === manualTeleopKey) stopManualTeleop();
    }

    function initManualKeyboard() {
        document.addEventListener('keydown', onManualKeyDown);
        document.addEventListener('keyup', onManualKeyUp);
        // Mất focus hoặc chuyển tab khi đang giữ phím: keyup sẽ không đến, phải tự dừng
        window.addEventListener('blur', () => stopManualTeleop());
        document.addEventListener('visibilitychange', () => { if (document.hidden) stopManualTeleop(); });
    }

    function initNavPage() {
        initEventListeners();
        initManualKeyboard();
        // B21 Fix: Gọi updateNavUI để sync nút AUTO/MANUAL sau reload
        updateNavUI();
    }

    window.showNavSetupModal = showNavSetupModal;
    window.updateNavUI = updateNavUI;
    window.onNavTransportChange = function (connected) {
        if (!connected) { navStarting = false; setActiveTool(null); stopManualTeleop(); }
        updateNavUI();
        window.updateNavPosition();
    };

    // Page navigation hook
    window.onNavigateToNav = function () {
        showNavMapViewport();
    };

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initNavPage);
    } else {
        initNavPage();
    }

})();
