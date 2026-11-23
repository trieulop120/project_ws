// resources/web/js/ros_connection.js

// Đổi tên biến thành wsProtocol để không bị khai báo trùng
const wsProtocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
const wsUrl = wsProtocol + '//' + location.host + '/ws';

let ws = null;
let reconnectDelay = 1000;
let reconnectTimer = null;
const maxReconnectDelay = 30000;
let commandRequestSeq = 0;
const pendingCommandRequests = new Map();
const settledCommandRequests = new Set();
function rememberCommandRequest(requestId) {
    settledCommandRequests.add(requestId);
    if (settledCommandRequests.size > 256) settledCommandRequests.delete(settledCommandRequests.values().next().value);
}

function setWsConnected(c, phase = c ? 'CONNECTED' : 'DISCONNECTED') {
    window.navigationTransportState = phase;
    window.missionTransportConnected = c;
    if (!c) {
        window.missionSnapshotReady = false;
        window.missionDisplaySession = (window.missionDisplaySession || 0) + 1;
        // Opening a new socket does not make cached ROS messages fresh again.
        for (const key of ['currentMissionSupervisorStatus', 'currentPalletRetrievalStatus',
                'currentCargoTelemetry', 'latestNavState', 'robotPose']) {
            if (window[key]) window[key] = {...window[key], stale: true};
        }
        for (const [requestId, resolve] of pendingCommandRequests) {
            rememberCommandRequest(requestId);
            resolve({status: 'unknown', uncertain: true,
                message: 'Command acknowledgement unavailable: connection lost. Verify robot state before retrying.'});
        }
        pendingCommandRequests.clear();
    }
    if (typeof isWsConnected !== 'undefined') {
        isWsConnected = c;
    }
    const badge = document.getElementById('devWebsocket');
    const dot = document.querySelector('#devWebsocket')?.parentElement?.querySelector('.status-dot');
    if (badge) {
        badge.textContent = c ? 'Connected' : phase === 'CONNECTING' ? 'Connecting' : 'Unconnected';
        badge.className = 'device-badge ' + (c ? 'online' : 'offline');
    }
    if (dot) {
        dot.className = 'status-dot ' + (c ? 'online' : 'offline');
    }
    if (typeof updateMapDisplay === 'function') {
        updateMapDisplay();
    }
    window.updateMissionCycle?.(window.currentMissionSupervisorStatus);
    window.onNavTransportChange?.(c);
    window.navigation3D?.onTransportChange?.(c);
}

/**
 * Update Navigation layout visibility based on global state (for multi-client sync)
 */
function updateNavLayoutVisibility() {
    const navWelcome = document.getElementById('navWelcome');
    const navLayout = document.getElementById('navLayout');

    if (window.isNavigationActive) {
        if (navWelcome) navWelcome.style.display = 'none';
        if (navLayout) {
            navLayout.style.display = 'grid';
        }
    } else {
        if (navWelcome) navWelcome.style.display = 'flex';
        if (navLayout) navLayout.style.display = 'none';
    }
}

/**
 * Update SLAM layout visibility based on global state (for multi-client sync)
 */
function updateSlamLayoutVisibility() {
    const slamWelcome = document.getElementById('slamWelcome');
    const slamLayout = document.getElementById('slamLayout');

    if (window.isSlamActive) {
        if (slamWelcome) slamWelcome.style.display = 'none';
        if (slamLayout) {
            slamLayout.style.display = 'grid';
        }
    } else {
        if (slamWelcome) slamWelcome.style.display = 'flex';
        if (slamLayout) slamLayout.style.display = 'none';
    }
}

function connect() {
    // B01 Fix: Idempotent - không tạo socket mới nếu đang open/connecting
    if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) {
        return;
    }
    // B01 Fix: Clear reconnect timer cũ
    if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
    }
    setWsConnected(false, 'CONNECTING');
    try {
        const sock = new WebSocket(wsUrl);
        ws = sock;

        ws.onopen = () => {
            if (ws !== sock) return;
            setWsConnected(true);
            reconnectDelay = 1000;
            if (typeof addActivity === 'function') addActivity('info', 'WebSocket connection established');
            if (typeof updateMapDisplay === 'function') updateMapDisplay();
        };

        ws.onmessage = e => {
            // B01 Fix: Bỏ qua message từ socket cũ (nếu có)
            if (ws !== sock) return;
            // Safety check for empty or non-string data
            if (!e.data || typeof e.data !== 'string') {
                return;
            }

            let d;
            try {
                d = JSON.parse(e.data);
            } catch (parseErr) {
                console.warn('WebSocket parse skipped:', e.data.substring(0, 100));
                return;
            }

            try {
                switch (d.type) {
                    case 'pose':
                        // Digital Twin uses the same socket as the 2D map.
                        window.navigation3D?.onPose?.(d);
                        if (typeof updatePose === 'function') updatePose(d);
                        if (typeof updateRobotStatus === 'function') updateRobotStatus(!d.stale);
                        break;

                    case 'battery':
                        if (typeof updateBattery === 'function') updateBattery(d);
                        break;

                    case 'map':
                        console.log('[WS] Received map data, isSlamActive:', window.isSlamActive);

                        // Fix: Chi cap nhat map tu WS khi SLAM dang chay
                        // Khi Navigation dang active voi PGM map (is_pgm_raw=true), bo qua OccupancyGrid tu Nav2
                        const currentMapData = window.mapData;
                        const isUsingPgmMap = currentMapData && currentMapData.is_pgm_raw === true;

                        if (window.isSlamActive || !isUsingPgmMap) {
                            window.mapData = d;

                            // Cập nhật dữ liệu vào mapRenderer (hàm updateMap sẽ tự xử lý hiển thị theo isSlamActive)
                            if (window.mapRenderer && typeof window.mapRenderer.updateMap === 'function') {
                                window.mapRenderer.updateMap(d);
                            } else if (typeof updateMap === 'function') {
                                updateMap(d);
                            }
                        } else {
                            console.log('[WS] Skipping OccupancyGrid - Navigation using PGM map');
                        }
                        break;

                    case 'slam_state':
                        console.log('[WS] Received slam_state:', d.is_slam_active);
                        const prevSlam = window.isSlamActive;
                        window.isSlamActive = d.is_slam_active === true;

                        // P0-6 Fix: Only clear map when SLAM actually STOPPED (prev=true, now=false)
                        if (window.mapRenderer) {
                            if (window.isSlamActive) {
                                // SLAM just started
                                window.mapRenderer.moveToViewport('slam');
                                window.mapRenderer.show();
                                if (window.mapData) window.mapRenderer.updateMap(window.mapData);
                            } else if (prevSlam) {
                                // SLAM just stopped - clear map
                                window.mapRenderer.hide(true);
                                if (typeof resetOverviewMapCard === 'function') resetOverviewMapCard();
                            }
                        }

                        // Update UI visibility for multi-client sync
                        updateSlamLayoutVisibility();
                        if (typeof updateSlamUI === 'function') updateSlamUI();
                        if (typeof updateOpModeDisplay === 'function') updateOpModeDisplay();
                        break;

                    case 'camera':
                        // ĐÃ BỔ SUNG: Truyền dữ liệu frame ảnh sang cho navigation.js xử lý vẽ
                        if (typeof updateCameraFeed === 'function') {
                            updateCameraFeed(d);
                        } else if (window.updateCameraFeed) {
                            window.updateCameraFeed(d);
                        }
                        break;

                    case 'robot_status':
                        if (typeof updateRobotStatus === 'function') {
                            if (d.connected) updateRobotStatus(true);
                            if (d.pose_available && window.robotPose) updateRobotStatus(true);
                        }
                        break;

                    case 'system_status':
                        if (d.devices && typeof updateDeviceStatuses === 'function') {
                            updateDeviceStatuses(d.devices);
                        }
                        if (d.system_status && typeof updateSystemStatus === 'function') {
                            updateSystemStatus(d.system_status);
                        }
                        if (d.lift) window.navigation3D?.onLift?.(d.lift);
                        if (d.lift && typeof updateLiftStatus === 'function') {
                            updateLiftStatus(d.lift);
                        }
                        // Pallet-mission cargo card (LOADED/EMPTY + US/lift detail)
                        // Select the owner from the same snapshot before any
                        // renderer writes the mission card or its clock.
                        window.currentCargoTelemetry = window.stampMissionStatus(d.mission);
                        window.acceptMissionSupervisorStatus(d.mission_supervisor);
                        window.currentPalletRetrievalStatus = window.stampMissionStatus(d.pallet_retrieval);
                        window.resolveRetrievalResume?.(window.currentPalletRetrievalStatus);
                        window.missionSnapshotReady = true;
                        // A new ROS snapshot resolves local start indicators,
                        // including a run that fails before an active heartbeat.
                        const supervisor = window.currentMissionSupervisorStatus;
                        if (window.missionStatusFresh(supervisor) &&
                                (window.missionActiveStates.supervisor.has(supervisor.state) ||
                                 ['DONE', 'FAILED', 'STOPPED'].includes(supervisor.state))) {
                            window.autoMissionStartPending = false;
                        }
                        const retrieval = window.currentPalletRetrievalStatus;
                        if (window.missionStatusFresh(retrieval) &&
                                (window.missionActiveStates.retrieval.has(retrieval.state) ||
                                 ['DONE', 'FAILED', 'CANCELED', 'STOPPED'].includes(retrieval.state))) {
                            window.retrievalStartPending = false;
                        }
                        if (typeof window.updateCargo === 'function') {
                            window.updateCargo(d.mission);
                        }
                        if (typeof window.updatePalletRetrievalStatus === 'function') {
                            window.updatePalletRetrievalStatus(d.pallet_retrieval);
                        }
                        if (typeof window.updateMissionCycle === 'function') {
                            window.updateMissionCycle(window.currentMissionSupervisorStatus);
                        }
                        break;

                    case 'laser':
                        if (typeof updateLaser === 'function') {
                            updateLaser(d);
                        }
                        break;

                    case 'nav_state':
                        // Broadcast nav_state to all clients for multi-client sync
                        if (typeof handleNavStateUpdate === 'function') {
                            handleNavStateUpdate(d);
                        }
                        // B06 Fix: Gọi updateOpModeDisplay để sync Operating Mode card
                        if (typeof updateOpModeDisplay === 'function') {
                            updateOpModeDisplay();
                        }
                        if (typeof window.updateNavUI === 'function') {
                            window.updateNavUI();
                        }
                        // Update nav layout visibility based on isNavigationActive
                        updateNavLayoutVisibility();
                        break;

                    case 'nav_result':
                        if (typeof handleNavResult === 'function') {
                            handleNavResult(d);
                        }
                        break;

                    case 'command_response':
                        if (d.request_id && pendingCommandRequests.has(d.request_id)) {
                            const resolve = pendingCommandRequests.get(d.request_id);
                            pendingCommandRequests.delete(d.request_id);
                            rememberCommandRequest(d.request_id);
                            d.handled_by_request = true;
                            resolve(d);
                        } else if (d.request_id && settledCommandRequests.has(d.request_id)) {
                            // A late/duplicate ACK may provide a newer snapshot,
                            // but must not repeat the request's notification.
                            if (d.mission_supervisor) {
                                window.acceptMissionSupervisorStatus(d.mission_supervisor);
                                window.updateMissionCycle?.(window.currentMissionSupervisorStatus);
                            }
                            if (d.nav_state) window.handleNavStateUpdate?.(d.nav_state);
                            break;
                        }
                        if (typeof handleCommandResponse === 'function') {
                            handleCommandResponse(d);
                        }
                        break;

                    // Phase D - nav layers (costmap + plan)
                    case 'costmap':
                        if (window.isSlamActive) break;
                        if (window.mapRenderer && window.mapRenderer.setCostmap) window.mapRenderer.setCostmap(d);
                        break;
                    case 'plan':
                        if (window.isSlamActive) break;
                        if (window.mapRenderer && window.mapRenderer.setPlan) window.mapRenderer.setPlan(d);
                        break;

                    // LLM assistant (llm_voice.js)
                    case 'llm_status':
                        if (typeof window.handleLLMStatus === 'function') window.handleLLMStatus(d);
                        break;
                }
            } catch (err) {
                console.warn('WebSocket message error:', err.message);
            }
        };

        ws.onclose = () => {
            if (ws !== sock) return;
            setWsConnected(false);
            if (typeof updateRobotStatus === 'function') updateRobotStatus(false);
            if (typeof addActivity === 'function') addActivity('warn', 'WebSocket connection lost. Reconnecting...');
            // B01 Fix: Dùng reconnectTimer để tránh duplicate timers
            if (reconnectTimer) clearTimeout(reconnectTimer);
            reconnectTimer = setTimeout(connect, reconnectDelay);
            reconnectDelay = Math.min(reconnectDelay * 1.5, maxReconnectDelay);
        };

        ws.onerror = err => {
            if (ws !== sock) return;
            setWsConnected(false);
            console.error('WS error:', err);
        };
    } catch (e) {
        if (typeof addActivity === 'function') addActivity('error', 'WebSocket connection failed');
        reconnectTimer = setTimeout(connect, reconnectDelay);
        reconnectDelay = Math.min(reconnectDelay * 1.5, maxReconnectDelay);
    }
}

/**
 * Gửi command lên backend qua WebSocket
 * @param {Object} cmd - JSON command object
 * @returns {boolean} - true nếu gửi thành công
 */
function sendCommand(cmd) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        console.warn('WebSocket not connected, cannot send command:', cmd);
        return false;
    }
    try {
        ws.send(JSON.stringify(cmd));
        return true;
    } catch (e) {
        console.error('Failed to send command:', e);
        return false;
    }
}

// Stop can include coordinator, Nav2 terminal, pallet and process shutdown
// waits in sequence. These are acknowledgement deadlines, never success timers.
const commandAckTimeouts = {
    stop_navigation: 90000, nav_cancel: 20000, emergency_stop: 20000,
    pallet_retrieval_cancel: 25000, mission_cycle_stop: 30000, mission_cycle_cancel: 30000, set_op_mode: 45000
};
function sendCommandAndWait(cmd, timeoutMs = commandAckTimeouts[cmd.command] || 15000) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        return Promise.resolve({ status: 'error', message: 'WebSocket not connected' });
    }
    const requestId = `web-${Date.now()}-${++commandRequestSeq}`;
    return new Promise((resolve) => {
        const timer = setTimeout(() => {
            pendingCommandRequests.delete(requestId);
            rememberCommandRequest(requestId);
            resolve({status: 'unknown', uncertain: true,
                message: `${cmd.command} timed out; outcome unknown. Verify robot state before retrying.`});
        }, timeoutMs);
        pendingCommandRequests.set(requestId, (response) => {
            clearTimeout(timer);
            resolve(response);
        });
        try {
            ws.send(JSON.stringify({ ...cmd, request_id: requestId }));
        } catch (e) {
            clearTimeout(timer);
            pendingCommandRequests.delete(requestId);
            resolve({ status: 'error', message: e.message || 'WebSocket send failed' });
        }
    });
}
window.sendCommandAndWait = sendCommandAndWait;

// Tự động kết nối WebSocket khi file JS này được nạp
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', connect);
} else {
    connect();
}
