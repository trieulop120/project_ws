// resources/web/js/ros_connection.js

// Đổi tên biến thành wsProtocol để không bị khai báo trùng
const wsProtocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
const wsUrl = wsProtocol + '//' + location.host + '/ws';

let ws = null;
let reconnectDelay = 1000;
const maxReconnectDelay = 30000;

function setWsConnected(c) {
    isWsConnected = c;
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

function connect() {
    setWsConnected(false);
    try {
        ws = new WebSocket(wsUrl);
        
        ws.onopen = () => {
            setWsConnected(true);
            reconnectDelay = 1000;
            if (typeof addActivity === 'function') addActivity('info', 'WebSocket connection established');
            if (typeof updateMapDisplay === 'function') updateMapDisplay();
        };

        ws.onmessage = e => {
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
                        window.mapData = d;

                        // Cập nhật dữ liệu vào mapRenderer (hàm updateMap sẽ tự xử lý hiển thị theo isSlamActive)
                        if (window.mapRenderer && typeof window.mapRenderer.updateMap === 'function') {
                            window.mapRenderer.updateMap(d);
                        } else if (typeof updateMap === 'function') {
                            updateMap(d);
                        }
                        break;

                    case 'slam_state':
                        console.log('[WS] Received slam_state:', d.is_slam_active);
                        const prevState = window.isSlamActive;
                        window.isSlamActive = d.is_slam_active === true;

                        // Sync map renderer đồng bộ ngay khi SLAM state thay đổi
                        if (window.mapRenderer) {
                            if (window.isSlamActive) {
                                // SLAM just started - move viewport and show
                                window.mapRenderer.moveToViewport('slam');
                                window.mapRenderer.show();

                                // NẾU đã có map data sẵn (từ trước khi slam_state đến), render ngay
                                if (window.mapData) {
                                    console.log('[WS] Have existing map data, rendering immediately');
                                    window.mapRenderer.updateMap(window.mapData);
                                }
                            } else {
                                // SLAM stopped - clear everything
                                window.mapData = null;
                                window.mapImageData = null;
                                window.mapRenderer.hide(true);

                                // Reset Overview map card
                                if (typeof resetOverviewMapCard === 'function') {
                                    resetOverviewMapCard();
                                }
                            }
                        }

                        if (typeof updateSlamUI === 'function') updateSlamUI();
                        if (typeof updateOpModeDisplay === 'function') updateOpModeDisplay();
                        break;

                    case 'camera':
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
                        break;

                    case 'laser':
                        if (typeof updateLaser === 'function') {
                            updateLaser(d);
                        }
                        break;

                    case 'command_response':
                        if (typeof handleCommandResponse === 'function') {
                            handleCommandResponse(d);
                        }
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
            setTimeout(connect, reconnectDelay);
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