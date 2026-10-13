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
    window.showRouteOverlay = true;

    // Operating Mode: MANUAL | AUTO - exported for state.js
    window.currentOpMode = 'MANUAL';
    let isMissionActive = false;
    let navStarting = false;
    let navStopping = false;  // B21 Fix: Cờ báo đang dừng - phòng thủ frontend
    let navStartedAt = 0;
    let pendingGoalName = '';
    let mapReqTimer = null;

    // Alias for internal use
    let currentOpMode = window.currentOpMode;

    // Nav state from backend
    let navState = {
        state: 'IDLE',
        progress: 0,
        nav2_running: false,
        nav2_ready: false,
        waypoint_index: 0,
        waypoint_total: 0,
        map_id: null,
        route_id: null
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

    // Mode toggle
    const btnModeAuto = document.getElementById('btnModeAuto');
    const btnModeManual = document.getElementById('btnModeManual');

    // Camera buttons
    const btnCamRgb = document.getElementById('btnCamRgb');
    const btnCamDepth = document.getElementById('btnCamDepth');
    const btnNavToggleCam = document.getElementById('btnNavToggleCam');

    // Camera state
    let cameraMode = 'off'; // 'off', 'rgb', 'depth'

    // Nav viewport
    const navViewport = document.getElementById('navMapViewport');

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

                        if (routeData.status === 'ok' && Array.isArray(routeData.routes) && routeData.routes.length > 0) {
                            const route = routeData.routes[0];
                            const routeId = extractRouteId(route, mapId);

                            mapToRouteMap[mapId] = { id: routeId, rawData: route };
                            routeToMapMap[routeId] = mapId;

                            const option = document.createElement('option');
                            option.value = routeId;
                            option.textContent = `${routeId} (${mapId})`;
                            navRouteSelect.appendChild(option);
                        }
                    } catch (e) {
                        console.warn(`[Nav] Cannot fetch routes for map ${mapId}:`, e);
                    }
                }
            }
        } catch (err) {
            console.error('[Nav] Failed to load maps or routes:', err);
        }

        if (navSetupModal) navSetupModal.classList.add('active');
    }

    function hideNavSetupModal() {
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
            if (navRouteSelect) navRouteSelect.value = routeInfo.id;
            window.selectedNavRoute = routeInfo.id;
            window.currentRoute = routeInfo.rawData;

            if (statusMsg) {
                statusMsg.style.display = 'block';
                statusMsg.style.background = 'rgba(16, 185, 129, 0.15)';
                statusMsg.style.color = '#10B981';
                statusMsg.style.border = '1px solid #10B981';
                statusMsg.textContent = `Đã chọn tự động Route: "${routeInfo.id}"`;
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

        const selectedRouteId = e.target.value;
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
        const linkedMapId = routeToMapMap[selectedRouteId];

        if (linkedMapId) {
            if (navMapSelect) navMapSelect.value = linkedMapId;
            window.selectedNavMap = linkedMapId;

            const routeInfo = mapToRouteMap[linkedMapId];
            if (routeInfo) window.currentRoute = routeInfo.rawData;

            if (statusMsg) {
                statusMsg.style.display = 'block';
                statusMsg.style.background = 'rgba(16, 185, 129, 0.15)';
                statusMsg.style.color = '#10B981';
                statusMsg.style.border = '1px solid #10B981';
                statusMsg.textContent = `Route thuộc Map: "${linkedMapId}". Đã tự động chọn Map!`;
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
                const route = json.routes.find(r => r.id === routeId) || json.routes[0];
                window.currentRoute = route;
                window.selectedNavRoute = route.id;
                console.log('[Nav] Route restored:', route.id);
                if (typeof window.renderMap === 'function') {
                    window.renderMap();
                }
            }
        } catch (e) {
            console.error('[Nav] Failed to restore route:', e);
        }
    }

    // ============================================
    // Navigation Start/Stop (Phase 5 Fix)
    // ============================================

    function startNavigation() {
        if (window.isSlamActive) {
            if (typeof window.showToast === 'function') {
                window.showToast('Please stop SLAM before running Navigation', 'warn');
            }
            return;
        }

        if (!window.selectedNavMap || !window.selectedNavRoute) return;

        // Clear old map data
        window.mapData = null;
        window.mapImageData = null;
        if (window.mapRenderer) window.mapRenderer.hide(true);

        navStarting = true;
        navStopping = false;  // B21 Fix: Reset cờ dừng khi bắt đầu
        navStartedAt = Date.now();
        pendingGoalName = '';

        showNavActiveUI();
        updateNavUI();
        showNavMapViewport();

        if (typeof updateOpModeDisplay === 'function') updateOpModeDisplay();
        if (typeof addActivity === 'function') {
            addActivity('info', `Navigation started – Map: ${window.selectedNavMap}`);
        }

        // Load PGM map for Navigation (Dual-Source: is_pgm_raw = true)
        loadNavMap();

        if (typeof sendCommand === 'function') {
            if (!sendCommand({ command: 'start_navigation', map_id: window.selectedNavMap, route_id: window.selectedNavRoute })) {
                rollbackStart('WebSocket connection lost');
            }
        } else {
            rollbackStart('sendCommand not available');
        }
    }

    // B11: Token để tránh race khi load map
    let navMapLoadSeq = 0;
    let navMapLoading = false;

    async function loadNavMap() {
        // B11: Token để tránh race condition
        const seq = ++navMapLoadSeq;
        if (navMapLoading) return;
        navMapLoading = true;

        try {
            const res = await fetch(`/api/map/image/${encodeURIComponent(window.selectedNavMap)}`);
            const json = await res.json();

            // B11: Kiểm tra token - bỏ qua nếu đã stop hoặc có map mới
            if (seq !== navMapLoadSeq) {
                console.log('[Nav] Map load aborted - seq mismatch');
                return;
            }

            // B11: Kiểm tra navPhase đã chuyển sang STOPPING/IDLE chưa
            if (!window.isNavigationActive || !window.selectedNavMap) {
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
            navMapLoading = false;
        }
    }

    function rollbackStart(msg) {
        navStarting = false;
        window.isNavigationActive = false;
        hideNavActiveUI();
        updateNavUI();
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

    function stopNavigation() {
        // B21 Fix: Bật cờ dừng để ngăn handleNavStateUpdate nạp lại map
        navStopping = true;

        // B11: Tăng seq để huỷ các pending load
        navMapLoadSeq++;
        navMapLoading = false;

        // Cancel any active mission
        if (isMissionActive) handleCancelMission();

        // Clear tool
        setActiveTool(null);

        // Send stop command
        if (typeof sendCommand === 'function') sendCommand({ command: 'stop_navigation' });

        // Clear state
        clearInterval(mapReqTimer);

        // FIX: Clear map data like SLAM does
        window.isNavigationActive = false;
        window.selectedNavMap = null;
        window.selectedNavRoute = null;
        window.currentRoute = null;
        isMissionActive = false;

        // Clear map data & UI (same as SLAM stop)
        window.mapData = null;
        window.mapImageData = null;
        if (window.mapRenderer) {
            window.mapRenderer.hide(true);
        }
        resetNavMapCard();

        // Reset nav state
        navState = { state: 'IDLE', progress: 0, nav2_running: false, nav2_ready: false,
                     waypoint_index: 0, waypoint_total: 0, map_id: null, route_id: null };

        // Reset op mode to MANUAL (safe default)
        currentOpMode = 'MANUAL';
        window.currentOpMode = 'MANUAL';

        hideNavActiveUI();
        updateNavUI();
        if (typeof updateOpModeDisplay === 'function') updateOpModeDisplay();
        if (typeof addActivity === 'function') addActivity('info', 'Navigation stopped');
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

        // Mode buttons
        if (btnModeAuto) btnModeAuto.classList.toggle('active', currentOpMode === 'AUTO');
        if (btnModeManual) btnModeManual.classList.toggle('active', currentOpMode === 'MANUAL');

        // Tools: only in MANUAL mode when nav active
        const toolsEnabled = isActive && currentOpMode === 'MANUAL';
        if (toolSetPose) {
            toolSetPose.disabled = !toolsEnabled;
            toolSetPose.style.opacity = toolsEnabled ? '1' : '0.5';
        }
        if (toolSetGoal) {
            toolSetGoal.disabled = !toolsEnabled;
            toolSetGoal.style.opacity = toolsEnabled ? '1' : '0.5';
        }

        updateMissionButtons();
    }

    function updateMissionButtons() {
        // AUTO mode: Start Mission (khi nhấn → hiển thị Cancel), MANUAL mode: chỉ Cancel
        if (currentOpMode === 'AUTO') {
            if (btnStartMission) btnStartMission.style.display = isMissionActive ? 'none' : 'block';
            if (btnCancelMission) {
                btnCancelMission.style.display = isMissionActive ? 'block' : 'none';
                btnCancelMission.style.width = '100%';
            }
        } else {
            if (btnStartMission) btnStartMission.style.display = 'none';
            if (btnCancelMission) {
                btnCancelMission.style.display = 'block';
                btnCancelMission.style.width = '100%';
            }
        }
    }

    // ============================================
    // Mode & Tool Switching
    // ============================================

    function setOpMode(mode) {
        if (mode === currentOpMode) return;
        currentOpMode = mode;
        window.currentOpMode = mode;  // Export for state.js
        console.log('[Nav] Mode changed to:', mode);
        // Vấn đề 3 Fix: Gửi lệnh set_op_mode qua WebSocket để đồng bộ giữa các client
        if (typeof sendCommand === 'function') {
            sendCommand({ command: 'set_op_mode', mode: mode });
        }
        setActiveTool(null); // Clear tool on mode change
        updateNavUI();
    }

    function setActiveTool(tool) {
        activeTool = tool;
        window.isCanvasToolActive = !!tool;
        toolSetPose?.classList.toggle('active', tool === 'pose');
        toolSetGoal?.classList.toggle('active', tool === 'goal');
        const c = document.getElementById('mapCanvas');
        if (c) c.style.cursor = tool ? 'crosshair' : 'grab';
        showNavOverlayMsg(tool ? `Nhấn &amp; kéo trên bản đồ để đặt ${tool === 'pose' ? 'Initial Pose' : 'Nav Goal'}` : '');
    }

    function selectTool(tool) {
        console.debug('[NAVTOOL] selectTool:', tool, 'activeTool:', activeTool, 'isNavActive:', window.isNavigationActive, 'opMode:', currentOpMode);
        if (!window.isNavigationActive) {
            if (typeof window.showToast === 'function') window.showToast('Start Navigation first', 'warn');
            return;
        }
        if (currentOpMode !== 'MANUAL') {
            if (typeof window.showToast === 'function') window.showToast('Switch to MANUAL mode to use tools', 'warn');
            return;
        }
        setActiveTool(activeTool === tool ? null : tool);
    }

    function toggleRouteVisibility() {
        window.showRouteOverlay = !window.showRouteOverlay;
        toolToggleRoute?.classList.toggle('active', window.showRouteOverlay);
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

    function handleStartMission() {
        if (currentOpMode !== 'AUTO') return;

        if (window.currentRoute && window.currentRoute.nodes && window.currentRoute.nodes.length > 0) {
            const waypoints = window.currentRoute.nodes.map(n => ({
                x: n.position.x,
                y: n.position.y,
                yaw: n.orientation?.yaw || 0,
                name: n.name
            }));

            pendingGoalName = window.selectedNavRoute || 'Route';
            if (typeof sendCommand === 'function') {
                sendCommand({
                    command: 'waypoint_mission',
                    waypoints: waypoints,
                    mission_type: 'route'
                });
            }
        } else {
            if (typeof window.showToast === 'function') window.showToast('No route loaded', 'warn');
        }
    }

    function handleCancelMission() {
        console.log('[Nav] Cancel mission + Emergency stop');
        if (typeof sendCommand === 'function') {
            sendCommand({ command: 'nav_cancel' });
            sendCommand({ command: 'emergency_stop' });
        }
        isMissionActive = false;
        updateMissionButtons();
        updateMissionStatus('Mission canceled', '--');
    }

    function updateMissionStatus(action, target) {
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

        navQuickMissions.querySelectorAll('.btn-action').forEach(btn => {
            btn.addEventListener('click', () => {
                if (!window.isNavigationActive || currentOpMode !== 'MANUAL') {
                    if (typeof window.showToast === 'function') window.showToast('Start Navigation in MANUAL mode', 'warn');
                    return;
                }

                const action = btn.textContent.trim().toLowerCase();
                let targetNode = null;

                if (window.currentRoute && window.currentRoute.nodes) {
                    if (action.includes('home')) {
                        targetNode = window.currentRoute.nodes.find(n =>
                            n.name.toUpperCase().includes('HOME'));
                    } else if (action.includes('pick 1') || action.includes('pick1')) {
                        targetNode = window.currentRoute.nodes.find(n =>
                            n.name.toUpperCase().includes('P_1') || n.name.toUpperCase().includes('PICK1'));
                    } else if (action.includes('pick 2') || action.includes('pick2')) {
                        targetNode = window.currentRoute.nodes.find(n =>
                            n.name.toUpperCase().includes('P_2') || n.name.toUpperCase().includes('PICK2'));
                    } else if (action.includes('drop')) {
                        targetNode = window.currentRoute.nodes.find(n =>
                            n.name.toUpperCase().includes('D_1') || n.name.toUpperCase().includes('DROP'));
                    }
                }

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

    function sendNavGoal(pose) {
        if (typeof sendCommand !== 'function') return;
        pendingGoalName = pose.name || `(${pose.x.toFixed(1)}, ${pose.y.toFixed(1)})`;
        sendCommand({ command: 'nav_goal', pose: pose });
        updateMissionStatus('Navigating...', pendingGoalName);
        // Note: isMissionActive will be set by command response
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
        console.debug('[NAVTOOL] onPointerDown:', { activeTool, isNavActive: window.isNavigationActive, opMode: currentOpMode, target: e.target.id, mapData: !!window.mapData });
        if (!activeTool || !window.isNavigationActive || currentOpMode !== 'MANUAL') return;
        const p = canvasXY(e);
        const w = window.canvasToWorld ? window.canvasToWorld(p.x, p.y) : null;
        if (!w) {
            if (typeof window.showToast === 'function') window.showToast('No map available', 'warn');
            return;
        }
        e.preventDefault();
        e.stopPropagation();
        navViewport?.setPointerCapture(e.pointerId);
        Object.assign(window.navDragState, {
            isDragging: true, startX: p.x, startY: p.y, currentX: p.x, currentY: p.y,
            startMapX: w.x, startMapY: w.y, yaw: 0, tool: activeTool
        });
        showNavOverlayMsg('Thả chuột để xác nhận – Esc để huỷ');
        if (typeof window.renderMap === 'function') window.renderMap();
    }

    function onPointerMove(e) {
        if (!window.navDragState.isDragging) return;
        const p = canvasXY(e);
        window.navDragState.currentX = p.x;
        window.navDragState.currentY = p.y;
        const dx = p.x - window.navDragState.startX;
        const dy = p.y - window.navDragState.startY;
        if (Math.abs(dx) > 5 || Math.abs(dy) > 5) {
            window.navDragState.yaw = Math.atan2(-dy, dx);
            console.debug('[NAVTOOL] yaw updated:', window.navDragState.yaw, 'rad =', (window.navDragState.yaw * 180 / Math.PI).toFixed(1), 'deg');
        }
        if (typeof window.renderMap === 'function') window.renderMap();
    }

    function onPointerUp(e) {
        console.debug('[NAVTOOL] onPointerUp:', { isDragging: window.navDragState.isDragging, tool: window.navDragState.tool });
        if (!window.navDragState.isDragging) return;
        window.navDragState.isDragging = false;
        try { navViewport?.releasePointerCapture(e.pointerId); } catch (_) {}

        const pose = {
            x: window.navDragState.startMapX,
            y: window.navDragState.startMapY,
            yaw: window.navDragState.yaw
        };
        const tool = window.navDragState.tool;
        console.debug('[NAVTOOL] Sending:', tool, pose);
        setActiveTool(null);

        if (tool === 'pose') {
            if (typeof sendCommand === 'function') sendCommand({ command: 'set_initial_pose', pose });
        } else if (tool === 'goal') {
            sendNavGoal(pose);
        }
        if (typeof window.renderMap === 'function') window.renderMap();
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
        if (typeof window.robotPose === 'undefined' || !window.robotPose) return;
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
        // B21 Fix: Không xử lý nav_state khi đang dừng
        if (navStopping) return;

        navState = {
            state: data.state || 'IDLE',
            progress: data.progress || 0,
            nav2_running: data.nav2_running || false,
            nav2_ready: data.nav2_ready || false,
            waypoint_index: data.waypoint_index || 0,
            waypoint_total: data.waypoint_total || 0,
            map_id: data.map_id || null,
            route_id: data.route_id || null,
        };

        // Fix: Luôn cập nhật selectedNavMap từ backend khi có map_id
        if (navState.map_id) {
            window.selectedNavMap = navState.map_id;
            window.selectedNavRoute = navState.route_id;

            // Khi F5/reload: load PGM map neu chua co
            if (navState.nav2_running && (!window.mapData || !window.mapData.is_pgm_raw)) {
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

        if (navState.nav2_running) {
            window.isNavigationActive = true;
        }

        // Fix: Luôn sync op_mode từ backend - bắt buộc cập nhật mỗi khi nhận nav_state
        if (data.op_mode !== undefined) {
            const newOpMode = data.op_mode === 'AUTO' ? 'AUTO' : 'MANUAL';
            if (newOpMode !== currentOpMode) {
                currentOpMode = newOpMode;
                window.currentOpMode = newOpMode;  // Export for state.js
            }
        }
        // B21 Fix: Gọi updateNavUI vô điều kiện để sync nút AUTO/MANUAL
        updateNavUI();

        // Update status badge - duy nhất một nguồn cập nhật
        const navStatusBadge = document.querySelector('.nav-status-badge');
        if (navStatusBadge) {
            if (navState.nav2_running) {
                navStatusBadge.textContent = navState.state === 'ACTIVE' ? 'Navigation Active' : 'Nav2 Ready';
                navStatusBadge.classList.add('active');
            } else {
                navStatusBadge.textContent = 'Nav2 Stopped';
                navStatusBadge.classList.remove('active');
            }
        }

        // Update mission status display
        const navActionText = document.getElementById('navActionText');
        const navTargetText = document.getElementById('navTargetText');
        const navProgressBar = document.getElementById('navProgressBar');
        const navProgressText = document.getElementById('navProgressText');

        if (navActionText) {
            switch (navState.state) {
                case 'IDLE':
                    navActionText.textContent = 'Idle';
                    if (isMissionActive) { isMissionActive = false; updateMissionButtons(); }
                    break;
                case 'ACTIVE':
                    navActionText.textContent = navState.waypoint_total > 0
                        ? `WP ${navState.waypoint_index + 1}/${navState.waypoint_total}`
                        : 'Navigating';
                    if (!isMissionActive) { isMissionActive = true; updateMissionButtons(); }
                    break;
                case 'SUCCEEDED':
                    navActionText.textContent = 'Goal reached!';
                    if (isMissionActive) { isMissionActive = false; updateMissionButtons(); }
                    break;
                case 'FAILED':
                    navActionText.textContent = 'Failed';
                    if (isMissionActive) { isMissionActive = false; updateMissionButtons(); }
                    break;
                case 'CANCELED':
                    navActionText.textContent = 'Canceled';
                    if (isMissionActive) { isMissionActive = false; updateMissionButtons(); }
                    break;
            }
        }

        if (navProgressBar) navProgressBar.style.width = `${navState.progress * 100}%`;
        if (navProgressText) navProgressText.textContent = `${Math.round(navState.progress * 100)}%`;

        updateMissionButtons();
        if (typeof window.updateNavPosition === 'function') window.updateNavPosition();
    };

    window.handleNavResult = function (data) {
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

        isMissionActive = false;
        updateMissionButtons();
    };

    // Phase 5 Fix: Command response handler for nav commands
    window.onNavCommandResponse = function (d) {
        const ok = ['started', 'stopped', 'sent', 'canceled', 'ok'].includes(d.status);
        switch (d.command) {
            case 'start_navigation':
                navStarting = false;
                if (!ok) { rollbackStart(d.message || d.status); break; }
                window.isNavigationActive = true;
                updateOpModeDisplay?.();
                updateNavUI();
                startMapRequestLoop();
                addActivity?.('info', `Navigation started – Map: ${window.selectedNavMap}`);
                break;

            case 'nav_goal':
            case 'waypoint_mission':
                if (ok) {
                    isMissionActive = true;
                    updateMissionButtons();
                    updateMissionStatus('Navigating…', pendingGoalName || '--');
                    if (typeof window.showToast === 'function') {
                        window.showToast(`Goal sent: ${pendingGoalName || 'Position'}`, 'success');
                    }
                } else {
                    isMissionActive = false;
                    updateMissionButtons();
                    updateMissionStatus('Failed', '--');
                    if (typeof window.showToast === 'function') {
                        window.showToast(d.message || 'Goal rejected', 'error');
                    }
                }
                break;

            case 'set_initial_pose':
                if (typeof window.showToast === 'function') {
                    window.showToast(ok ? 'Initial Pose sent' : (d.message || 'Failed to send Initial Pose'),
                        ok ? 'success' : 'error');
                }
                break;

            case 'nav_cancel':
            case 'emergency_stop':
                if (!ok && typeof window.showToast === 'function') {
                    window.showToast(d.message || 'Cancel failed', 'error');
                }
                break;

            case 'stop_navigation':
                if (!ok && typeof window.showToast === 'function') {
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
                navCameraFeed.innerHTML = `<span class="muted" style="font-size: 0.75rem;">Connecting to ${mode.toUpperCase()}...</span>`;
            }
        }

        console.log('[Nav] Camera mode:', mode);
    }

    // ============================================
    // Event Listeners Initialization
    // ============================================

    function initEventListeners() {
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
        btnNavToggleCam?.addEventListener('click', () => setCameraMode('off'));

        // Mission buttons
        btnStartMission?.addEventListener('click', handleStartMission);
        btnCancelMission?.addEventListener('click', handleCancelMission);

        // Tool buttons
        toolSetPose?.addEventListener('click', () => selectTool('pose'));
        toolSetGoal?.addEventListener('click', () => selectTool('goal'));
        toolToggleRoute?.addEventListener('click', toggleRouteVisibility);
        toolCenterMap?.addEventListener('click', centerMapOnRobot);

        // Pointer events on nav viewport (Phase 4 Fix)
        navViewport?.addEventListener('pointerdown', onPointerDown);
        navViewport?.addEventListener('pointermove', onPointerMove);
        navViewport?.addEventListener('pointerup', onPointerUp);
        navViewport?.addEventListener('pointercancel', cancelDrag);

        // Quick goals
        setupQuickGoals();
    }

    function initNavPage() {
        initEventListeners();
        // B21 Fix: Gọi updateNavUI để sync nút AUTO/MANUAL sau reload
        updateNavUI();
    }

    window.showNavSetupModal = showNavSetupModal;

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
