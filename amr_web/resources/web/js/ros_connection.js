// resources/web/js/ros_connection.js

// Đổi tên biến thành wsProtocol để không bị khai báo trùng
const wsProtocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
const wsUrl = wsProtocol + '//' + location.host + '/ws';

let ws = null;
let reconnectDelay = 1000;
let reconnectTimer = null;
const maxReconnectDelay = 30000;

function setWsConnected(c) {
    if (typeof isWsConnected !== 'undefined') {
        isWsConnected = c;
    }
    const badge = document.getElementById('devWebsocket');
    const dot = document.querySelector('#devWebsocket')?.parentElement?.querySelector('.status-dot');
    if (badge) {
        badge.textContent = c ? 'Connected' : 'Unconnected';
        badge.className = 'device-badge ' + (c ? 'online' : 'offline');
    }
    if (dot) {
        dot.className = 'status-dot ' + (c ? 'online' : 'offline');
    }
    if (typeof updateMapDisplay === 'function') {
        updateMapDisplay();
    }
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
    setWsConnected(false);
    try {
        const sock = new WebSocket(wsUrl);
        ws = sock;

        ws.onopen = () => {
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
                        if (typeof updatePose === 'function') updatePose(d);
                        if (typeof updateRobotStatus === 'function') updateRobotStatus(true);
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
                        if (d.lift && typeof updateLiftStatus === 'function') {
                            updateLiftStatus(d.lift);
                        }
                        // Pallet-mission cargo card (LOADED/EMPTY + US/lift detail)
                        if (typeof window.updateCargo === 'function') {
                            window.updateCargo(d.mission);
                        }
                        break;

                    case 'laser':
                        if (typeof updateLaser === 'function') {
                            updateLaser(d);
                        }
                        break;

                    case 'nav_state':
                        // Broadcast nav_state to all clients for multi-client sync
                        window.isNavigationActive = d.nav2_running === true;
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
                }
            } catch (err) {
                console.warn('WebSocket message error:', err.message);
            }
        };

        ws.onclose = () => {
            setWsConnected(false);
            if (typeof updateRobotStatus === 'function') updateRobotStatus(false);
            if (typeof addActivity === 'function') addActivity('warn', 'WebSocket connection lost. Reconnecting...');
            // B01 Fix: Dùng reconnectTimer để tránh duplicate timers
            if (reconnectTimer) clearTimeout(reconnectTimer);
            reconnectTimer = setTimeout(connect, reconnectDelay);
            reconnectDelay = Math.min(reconnectDelay * 1.5, maxReconnectDelay);
        };

        ws.onerror = err => {
            console.error('WS error:', err);
            if (typeof addActivity === 'function') addActivity('error', 'WebSocket error occurred');
        };
    } catch (e) {
        if (typeof addActivity === 'function') addActivity('error', 'WebSocket connection failed');
        setTimeout(connect, reconnectDelay);
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

// Tự động kết nối WebSocket khi file JS này được nạp
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', connect);
} else {
    connect();
}