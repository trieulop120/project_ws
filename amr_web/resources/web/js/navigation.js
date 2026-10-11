// Navigation Page Logic
(function() {
    'use strict';

    // ========================================
    // Navigation State & Data Mapping
    // ========================================
    window.isNavigationActive = false;
    window.selectedNavMap = null;
    window.selectedNavRoute = null;
    window.showRouteOverlay = true;

    let availableMaps = [];
    let mapToRouteMap = {}; // Key: map_id -> { id, rawData }
    let routeToMapMap = {}; // Key: route_id -> map_id
    let isBindingInProgress = false;

    // DOM Elements
    const navSetupModal = document.getElementById('navSetupModal');
    const btnStartNav = document.getElementById('btnStartNav');
    const btnStopNav = document.getElementById('btnStopNav');
    const navModalClose = document.getElementById('navModalClose');
    const navModalCancel = document.getElementById('navModalCancel');
    const navModalConfirm = document.getElementById('navModalConfirm');
    const navMapSelect = document.getElementById('navMapSelect');
    const navRouteSelect = document.getElementById('navRouteSelect');
    const btnToggleRouteVisibility = document.getElementById('btnToggleRouteVisibility');

    // Helper: Trích xuất ID từ Map Item (dù là Object hay String)
    function extractMapId(mapItem) {
        if (!mapItem) return '';
        if (typeof mapItem === 'object') {
            return mapItem.id || mapItem.name || mapItem.map_id || String(mapItem);
        }
        return String(mapItem);
    }

    // Helper: Trích xuất Route ID từ Route Item
    function extractRouteId(routeItem, defaultMapId) {
        if (!routeItem) return '';
        if (typeof routeItem === 'object') {
            return routeItem.graph_name || routeItem.id || routeItem.name || `${defaultMapId}_route`;
        }
        return String(routeItem);
    }

    // ========================================
    // Modal Functions (Load Maps & Routes chuẩn map_route.js)
    // ========================================

    async function showNavSetupModal() {
        console.log('[Navigation] Showing setup modal...');

        // 1. Reset Selections & UI
        window.selectedNavMap = null;
        window.selectedNavRoute = null;
        if (navMapSelect) navMapSelect.innerHTML = '<option value="">-- Select a map --</option>';
        if (navRouteSelect) navRouteSelect.innerHTML = '<option value="">-- Select a route --</option>';
        if (navModalConfirm) navModalConfirm.disabled = true;

        // Reset hoặc tạo khung thông báo trạng thái
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
            // 2. Fetch danh sách Maps từ /api/maps (Giống map_route.js)
            const mapRes = await fetch('/api/maps');
            const mapData = await mapRes.json();

            if (mapData.status === 'ok' && Array.isArray(mapData.maps)) {
                availableMaps = mapData.maps;

                // Fill Map vào Dropdown
                availableMaps.forEach(mapItem => {
                    const mapId = extractMapId(mapItem);
                    if (mapId && mapId !== '[object Object]') {
                        const option = document.createElement('option');
                        option.value = mapId;
                        option.textContent = mapId;
                        navMapSelect.appendChild(option);
                    }
                });

                // 3. Fetch Route theo từng map_id (Cấu trúc API giống hệt map_route.js)
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
                        console.warn(`[Navigation] Cannot fetch routes for map ${mapId}:`, e);
                    }
                }
            }
        } catch (err) {
            console.error('[Navigation] Failed to load maps or routes:', err);
        }

        // Show Modal
        if (navSetupModal) {
            navSetupModal.classList.add('active');
        }
    }

    function hideNavSetupModal() {
        if (navSetupModal) {
            navSetupModal.classList.remove('active');
        }
    }

    // ========================================
    // Two-Way Binding Handlers
    // ========================================

    function onMapSelectChange(e) {
        if (isBindingInProgress) return;
        isBindingInProgress = true;

        const selectedMapId = e.target.value;
        const statusMsg = document.getElementById('navModalStatusMsg');

        if (!selectedMapId) {
            if (navRouteSelect) navRouteSelect.value = '';
            window.selectedNavMap = null;
            window.selectedNavRoute = null;
            if (navModalConfirm) navModalConfirm.disabled = true;
            if (statusMsg) statusMsg.style.display = 'none';
            isBindingInProgress = false;
            return;
        }

        window.selectedNavMap = selectedMapId;

        // Nếu Map CÓ Route -> Auto Select Route
        if (mapToRouteMap[selectedMapId]) {
            const routeInfo = mapToRouteMap[selectedMapId];
            if (navRouteSelect) navRouteSelect.value = routeInfo.id;
            window.selectedNavRoute = routeInfo.id;

            if (statusMsg) {
                statusMsg.style.display = 'block';
                statusMsg.style.background = 'rgba(16, 185, 129, 0.15)';
                statusMsg.style.color = '#10B981';
                statusMsg.style.border = '1px solid #10B981';
                statusMsg.textContent = `Đã chọn tự động Route: "${routeInfo.id}"`;
            }
            if (navModalConfirm) navModalConfirm.disabled = false;
        } else {
            // Nếu Map KHÔNG CÓ Route -> Báo lỗi & Khóa Confirm
            if (navRouteSelect) navRouteSelect.value = '';
            window.selectedNavRoute = null;

            if (statusMsg) {
                statusMsg.style.display = 'block';
                statusMsg.style.background = 'rgba(239, 68, 68, 0.15)';
                statusMsg.style.color = '#EF4444';
                statusMsg.style.border = '1px solid #EF4444';
                statusMsg.textContent = `Map "${selectedMapId}" chưa có Route Graph. Vui lòng sang tab "Map & Route" để tạo!`;
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
        console.log('[Navigation] Selected Map:', window.selectedNavMap);
        console.log('[Navigation] Selected Route:', window.selectedNavRoute);

        hideNavSetupModal();
        startNavigation();
    }

    // ========================================
    // Route Visibility Toggle Feature
    // ========================================

    function toggleRouteVisibility() {
        window.showRouteOverlay = !window.showRouteOverlay;
        
        if (btnToggleRouteVisibility) {
            if (window.showRouteOverlay) {
                btnToggleRouteVisibility.classList.add('active');
                btnToggleRouteVisibility.textContent = '👁️ Route Overlay: ON';
            } else {
                btnToggleRouteVisibility.classList.remove('active');
                btnToggleRouteVisibility.textContent = '🙈 Route Overlay: OFF';
            }
        }

        if (typeof window.renderMap === 'function') {
            window.renderMap();
        }

        if (typeof showToast === 'function') {
            showToast(`Hiển thị Route: ${window.showRouteOverlay ? 'ON' : 'OFF'}`, 'info');
        }
    }

    window.toggleRouteVisibility = toggleRouteVisibility;

    // ========================================
    // Navigation Control Functions
    // ========================================

    function startNavigation() {
        console.log('[Navigation] Starting navigation mode...');

        window.isNavigationActive = true;
        showNavActiveUI();
        updateNavUI();

        if (typeof updateOpModeDisplay === 'function') {
            updateOpModeDisplay();
        }

        if (typeof addActivity === 'function') {
            addActivity('info', 'Navigation started - Map: ' + window.selectedNavMap + ', Route: ' + window.selectedNavRoute);
        }

        if (typeof sendCommand === 'function' && window.selectedNavMap) {
            const routePayload = mapToRouteMap[window.selectedNavMap]?.rawData;
            sendCommand({
                command: 'start_navigation',
                map_id: window.selectedNavMap,
                route: routePayload
            });
        }

        if (typeof window.renderMap === 'function') {
            window.renderMap();
        }

        console.log('[Navigation] Navigation mode started successfully');
    }

    function stopNavigation() {
        console.log('[Navigation] Stopping navigation mode...');
        setCameraMode('off');
        window.isNavigationActive = false;
        window.selectedNavMap = null;
        window.selectedNavRoute = null;

        hideNavActiveUI();
        updateNavUI();

        if (typeof updateOpModeDisplay === 'function') {
            updateOpModeDisplay();
        }

        if (typeof addActivity === 'function') {
            addActivity('info', 'Navigation mode stopped');
        }

        if (typeof window.renderMap === 'function') {
            window.renderMap();
        }

        console.log('[Navigation] Navigation mode stopped');
    }

    // ========================================
    // UI Helpers
    // ========================================

    function showNavActiveUI() {
        const navWelcome = document.getElementById('navWelcome');
        const navLayout = document.getElementById('navLayout');

        if (navWelcome) navWelcome.style.display = 'none';
        if (navLayout) {
            navLayout.style.display = 'flex';
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
        const navStatusBadge = document.querySelector('.nav-status-badge');

        if (navStatusBadge) {
            if (window.isNavigationActive) {
                navStatusBadge.textContent = 'Navigation Active';
                navStatusBadge.className = 'nav-status-badge active';
            } else {
                navStatusBadge.textContent = 'Navigation Inactive';
                navStatusBadge.className = 'nav-status-badge';
            }
        }
    }

    // ========================================
    // Event Listeners
    // ========================================

    function initEventListeners() {
        btnStartNav?.addEventListener('click', showNavSetupModal);
        btnStopNav?.addEventListener('click', stopNavigation);

        navModalClose?.addEventListener('click', hideNavSetupModal);
        navModalCancel?.addEventListener('click', hideNavSetupModal);
        navModalConfirm?.addEventListener('click', onConfirmClick);

        navMapSelect?.addEventListener('change', onMapSelectChange);
        navRouteSelect?.addEventListener('change', onRouteSelectChange);

        btnToggleRouteVisibility?.addEventListener('click', toggleRouteVisibility);

        navSetupModal?.addEventListener('click', (e) => {
            if (e.target === navSetupModal) hideNavSetupModal();
        });

        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && navSetupModal?.classList.contains('active')) {
                hideNavSetupModal();
            }
        });

        console.log('[Navigation] Event listeners initialized');
    }

    function initNavPage() {
        console.log('[Navigation] Module Initializing...');
        initEventListeners();
        console.log('[Navigation] Module Initialized!');
    }

    window.onNavigateToNav = function() {
        console.log('[Navigation] Navigated to Navigation page');
    };

    window.showNavSetupModal = showNavSetupModal;

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initNavPage);
    } else {
        initNavPage();
    }

    function renderRecentMissions(list) {
        const body  = document.getElementById('navRecentMissions');
        const empty = document.getElementById('navMissionEmpty');
        if (!body || !empty) return;

        if (!list || !list.length) {
            body.innerHTML = '';                 // không để dòng nào trong bảng
            empty.classList.remove('hidden');
            return;
        }

        empty.classList.add('hidden');
        body.innerHTML = list.slice(0, 20).map(m => `
            <tr>
                <td class="activity-time">${m.time}</td>
                <td class="mission-name">${m.name}</td>
            </tr>`).join('');
    }

    window.renderRecentMissions = renderRecentMissions;

    // Quản lý chế độ Camera Navigation
    let currentCamMode = 'off'; // 'rgb' | 'depth' | 'off'  (mặc định OFF)

    const btnCamRgb = document.getElementById('btnCamRgb');
    const btnCamDepth = document.getElementById('btnCamDepth');
    const btnNavToggleCam = document.getElementById('btnNavToggleCam');
    const navCameraFeed = document.getElementById('navCameraFeed');

    function setCameraMode(mode) {
        currentCamMode = mode;

        // Cập nhật giao diện 3 nút
        btnCamRgb?.classList.toggle('active', mode === 'rgb');
        btnCamDepth?.classList.toggle('active', mode === 'depth');
        btnNavToggleCam?.classList.toggle('active', mode === 'off');

        if (!navCameraFeed) return;

        // OFF: xóa thẻ <img> để ngắt luôn kết nối stream
        if (mode === 'off') {
            navCameraFeed.innerHTML = '<span class="muted" style="font-size: 0.75rem;" id="camStatusText">Camera Offline</span>';
            return;
        }

        // Đổi topic ROS tương ứng
        const topicName = (mode === 'rgb')
            ? '/camera/color/image_raw/compressed'
            : '/camera/depth/image_raw/compressed';

        // Stream từ Web Video Server
        navCameraFeed.innerHTML =
            `<img src="http://${window.location.hostname}:8080/stream?topic=${topicName}" style="width: 100%; height: 100%; object-fit: cover;" />`;
    }

    btnCamRgb?.addEventListener('click', () => setCameraMode('rgb'));
    btnCamDepth?.addEventListener('click', () => setCameraMode('depth'));
    btnNavToggleCam?.addEventListener('click', () => setCameraMode('off'));

    // Đồng bộ UI với trạng thái mặc định ngay khi load trang
    setCameraMode(currentCamMode);

})();