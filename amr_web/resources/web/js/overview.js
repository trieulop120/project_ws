function updatePose(d) {
    robotPose = d;
    window.robotPose = d;  // Export to global so map.js can access
    const posX = document.getElementById('cardPosX');
    const posY = document.getElementById('cardPosY');
    const posYaw = document.getElementById('cardPosYaw');

    if (posX && posY && posYaw) {
        posX.textContent = d.position.x.toFixed(2);
        posY.textContent = d.position.y.toFixed(2);
        posYaw.textContent = (d.orientation.yaw * 180 / Math.PI).toFixed(1);

        posX.classList.remove('muted');
        posY.classList.remove('muted');
        posYaw.classList.remove('muted');
    }

    render();
    updateMapDisplay();
}

function updateBattery(d) {
    const fill = document.getElementById('cardBatteryFill');
    const pct = document.getElementById('cardBatteryPercent');
    if (!fill || !pct) return;

    if (d.percentage != null) {
        fill.style.width = d.percentage + '%';
        pct.textContent = d.percentage.toFixed(0) + '%';
        pct.classList.remove('muted');
        fill.className = 'battery-fill' + (d.percentage < 20 ? ' critical' : d.percentage < 40 ? ' warning' : '');

        if (d.percentage < 18 && d.percentage > 0) {
            addAlarm('warning', 'Battery Low: ' + d.percentage.toFixed(0) + '%');
        }
    } else {
        fill.style.width = '0%';
        pct.textContent = '--';
        pct.classList.add('muted');
    }
}

function updateMap(d) {
    mapData = d;
    mapImageData = buildMapImageData();
    const n = document.getElementById('cardMapName');
    if (n) {
        // Phân biệt SLAM mode vs Navigation mode vs loaded map
        const isSlamMode = window.isSlamActive === true;
        const isNavMode = window.isNavigationActive === true;
        n.textContent = isSlamMode ? 'SLAM Live' : isNavMode ? 'Navigation Live' : (d.frame_id || 'Active');
        n.className = 'metric-value accent';
    }
    const status = document.getElementById('cardMapStatus');
    if (status) {
        // Phân biệt SLAM mode vs Navigation mode vs loaded map
        const isSlamMode = window.isSlamActive === true;
        const isNavMode = window.isNavigationActive === true;
        status.textContent = isSlamMode ? 'SLAM Live' : isNavMode ? 'Navigation Live' : 'Loaded';
    }

    updateMapDisplay();

    // Only fitToView for Overview, not for SLAM mode (SLAM manages its own view)
    if (!window.isSlamActive && !window.mapFitted) {
        fitToView();
        window.mapFitted = true;
    }

    // Update SLAM map renderer if SLAM is active
    if (window.isSlamActive && window.mapRenderer) {
        window.mapRenderer.updateMap(d);
    }

    addActivity('info', 'Map data received: ' + (d.frame_id || 'Unknown'));
}

function updateMapDisplay() {
    const es = document.getElementById('mapEmptyState');
    const ms = document.getElementById('mapStatusDisplay');
    const mn = document.getElementById('mapNameDisplay');

    if (mapData) {
        if (es) es.style.display = 'none';
        if (ms) {
            ms.textContent = isWsConnected ? 'Live' : 'Stale';
            ms.className = 'map-status' + (isWsConnected ? '' : ' stale');
        }
        if (mn) mn.textContent = mapData.frame_id || 'Map';
    } else {
        if (es) es.style.display = 'flex';
        if (ms) {
            // Giữ nguyên text mặc định từ HTML "No map loaded"
            ms.className = 'map-status';
        }
        if (mn) {
            // Giữ nguyên text mặc định từ HTML "None"
        }
    }
}

function updateRobotStatus(c) {
    isRobotConnected = c;
    const dot = document.getElementById('cardStatusDot');
    const label = document.getElementById('cardStatusText');

    if (dot && label) {
        if (c) {
            dot.className = 'status-dot online';
            label.className = 'status-label online';
            label.textContent = 'Online';
        } else {
            dot.className = 'status-dot offline';
            label.className = 'status-label offline';
            label.textContent = 'Offline';
        }
    }
}

/**
 * Cập nhật trạng thái thiết bị từ system_status payload
 * @param {Object} devices - Dictionary of device states {esp32: {connected, hz}, ...}
 */
function updateDeviceStatuses(devices) {
    if (!devices) return;

    // Map device names to HTML element IDs
    const deviceMap = {
        esp32: 'devEsp32',
        lidar: 'devLidar',
        camera: 'devCamera',
        lift: 'devLift'
    };

    for (const [deviceKey, elementId] of Object.entries(deviceMap)) {
        const state = devices[deviceKey] || { connected: false, hz: 0 };
        deviceStates[deviceKey] = state;

        const container = document.getElementById(elementId);
        if (!container) continue;

        const dot = container.parentElement.querySelector('.status-dot');
        const badge = container;

        if (state.connected) {
            if (dot) {
                dot.className = 'status-dot online';
            }
            badge.textContent = 'Connected';
            badge.className = 'device-badge online';
        } else {
            if (dot) {
                dot.className = 'status-dot offline';
            }
            badge.textContent = 'Unconnected';
            badge.className = 'device-badge offline';
        }
    }
}

/**
 * Cập nhật System Card dựa trên trạng thái tổng quan
 * @param {string} status - "Online" | "Degraded" | "Offline"
 */
function updateSystemStatus(status) {
    systemStatus = status;
    const badge = document.getElementById('cardNavBadge');
    if (!badge) return;

    badge.className = 'nav-badge ' + status.toLowerCase();
    badge.innerHTML = '<span>' + status + '</span>';
}

/**
 * Cập nhật Lift height và trạng thái
 * @param {Object} liftData - {height: number, state: string}
 */
function updateLiftStatus(liftData) {
    if (!liftData) return;

    const heightEl = document.getElementById('cardLiftHeight');
    const stateEl = document.getElementById('cardLiftState');
    const slamHeightEl = document.getElementById('slamLiftHeight');
    const slamStateEl = document.getElementById('slamLiftState');

    // Cập nhật height cho Overview card
    if (heightEl) {
        if (liftData.height !== null && liftData.height !== undefined) {
            heightEl.textContent = liftData.height;
            heightEl.classList.remove('muted');
        } else {
            heightEl.textContent = '--';
            heightEl.classList.add('muted');
        }
    }

    // Cập nhật state cho Overview card
    if (stateEl) {
        const state = liftData.state;
        if (state) {
            stateEl.textContent = state;
            stateEl.className = 'lift-state ' + state.toLowerCase();
            stateEl.style.display = '';
        } else {
            stateEl.style.display = 'none';
        }
    }

    // Cập nhật height cho SLAM card
    if (slamHeightEl) {
        if (liftData.height !== null && liftData.height !== undefined) {
            slamHeightEl.textContent = liftData.height;
            slamHeightEl.classList.remove('muted');
        } else {
            slamHeightEl.textContent = '--';
            slamHeightEl.classList.add('muted');
        }
    }

    // Cập nhật state cho SLAM card
    if (slamStateEl) {
        const state = liftData.state;
        if (state) {
            slamStateEl.textContent = state;
            slamStateEl.className = 'lift-state ' + state.toLowerCase();
            slamStateEl.style.display = '';
        } else {
            slamStateEl.style.display = 'none';
        }
    }
}

function addAlarm(type, title) {
    const now = new Date();
    const alarm = {
        type: type,
        title: title,
        time: now.toLocaleTimeString('en-US', { hour12: false }),
        timestamp: now.getTime()
    };

    const existingIndex = alarmLog.findIndex(a => a.title === title);
    if (existingIndex !== -1) alarmLog.splice(existingIndex, 1);

    alarmLog.unshift(alarm);
    if (alarmLog.length > MAX_ALARMS) alarmLog.pop();

    renderAlarmList();
}

function renderAlarmList() {
    const list = document.getElementById('alarmList');
    if (!list) return;

    const icons = {
        error: '<svg class="alarm-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><line x1="12" y1="8" x2="12" y2="12"/><line x1="12" y1="16" x2="12.01" y2="16"/></svg>',
        warning: '<svg class="alarm-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg>',
        info: '<svg class="alarm-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><line x1="12" y1="16" x2="12" y2="12"/><line x1="12" y1="8" x2="12.01" y2="8"/></svg>'
    };

    let filteredAlarms = alarmLog;
    if (alarmFilter === 'info') filteredAlarms = alarmLog.filter(a => a.type === 'info');
    else if (alarmFilter === 'error') filteredAlarms = alarmLog.filter(a => a.type === 'error' || a.type === 'critical');
    else if (alarmFilter === 'warning') filteredAlarms = alarmLog.filter(a => a.type === 'warning');

    if (filteredAlarms.length === 0) {
        list.innerHTML = '<div class="alarm-empty"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"/><polyline points="22 4 12 14.01 9 11.01"/></svg><div>No Active Notifications</div></div>';
        return;
    }

    list.innerHTML = filteredAlarms.map(alarm => {
        const itemType = (alarm.type === 'critical' || alarm.type === 'error') ? 'error' : alarm.type;
        return '<div class="alarm-item ' + itemType + '">' +
               (icons[itemType] || icons.info) +
               '<div class="alarm-content">' +
               '<div class="alarm-title">' + alarm.title + '</div>' +
               '<div class="alarm-time">' + alarm.time + '</div>' +
               '</div></div>';
    }).join('');
}

function addActivity(type, message) {
    const now = new Date();
    const timeStr = now.toLocaleTimeString('en-US', { hour12: false });
    activityLog.unshift({ time: timeStr, type: type, message: message });
    if (activityLog.length > MAX_ACTIVITY) activityLog.pop();
    renderActivityTable();

    // Đồng bộ trực tiếp sang Card Alarm nếu là Error/Warn/Info
    if (type === 'error' || type === 'warn' || type === 'info') {
        const alarmType = type === 'warn' ? 'warning' : type;
        addAlarm(alarmType, message);
    }
}

function renderActivityTable() {
    const tbody = document.getElementById('activityBody');
    if (!tbody) return;

    if (activityLog.length === 0) {
        tbody.innerHTML = '<tr><td colspan="3" style="text-align:center;color:var(--text-muted);padding:16px">No activity recorded</td></tr>';
        return;
    }
    tbody.innerHTML = activityLog.map(item => {
        const typeClass = item.type === 'error' ? 'error' : item.type === 'warn' ? 'warn' : 'info';
        return '<tr><td class="activity-time">' + item.time + '</td>' +
               '<td><span class="activity-type ' + typeClass + '">' + item.type.toUpperCase() + '</span></td>' +
               '<td class="activity-msg">' + item.message + '</td></tr>';
    }).join('');
}

/**
 * Toast Notification - hiển thị thông báo tạm thời trên màn hình
 * @param {string} message - Nội dung thông báo
 * @param {string} type - 'success' | 'error' | 'info' | 'warning'
 * @param {number} duration - Thời gian hiển thị (ms), default 4000
 */
function showToast(message, type = 'info', duration = 4000) {
    // Tạo toast container nếu chưa có
    let toastContainer = document.getElementById('toastContainer');
    if (!toastContainer) {
        toastContainer = document.createElement('div');
        toastContainer.id = 'toastContainer';
        toastContainer.style.cssText = `
            position: fixed;
            bottom: 24px;
            right: 24px;
            z-index: 9999;
            display: flex;
            flex-direction: column;
            gap: 8px;
            pointer-events: none;
        `;
        document.body.appendChild(toastContainer);
    }

    // Tạo toast element
    const toast = document.createElement('div');
    const bgColor = type === 'success' ? 'var(--accent)' :
                    type === 'error' ? 'var(--error)' :
                    type === 'warning' ? '#f59e0b' : 'var(--bg-panel)';
    const textColor = type === 'success' ? '#0a0a0a' : 'var(--text-primary)';
    const icon = type === 'success' ?
        '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"/><polyline points="22 4 12 14.01 9 11.01"/></svg>' :
        type === 'error' ?
        '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><line x1="15" y1="9" x2="9" y2="15"/><line x1="9" y1="9" x2="15" y2="15"/></svg>' :
        type === 'warning' ?
        '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M10.29 3.86L1.82 18a2 2 0 0 0 1.71 3h16.94a2 2 0 0 0 1.71-3L13.71 3.86a2 2 0 0 0-3.42 0z"/><line x1="12" y1="9" x2="12" y2="13"/><line x1="12" y1="17" x2="12.01" y2="17"/></svg>' :
        '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><circle cx="12" cy="12" r="10"/><line x1="12" y1="16" x2="12" y2="12"/><line x1="12" y1="8" x2="12.01" y2="8"/></svg>';

    toast.style.cssText = `
        display: flex;
        align-items: center;
        gap: 12px;
        padding: 14px 18px;
        background: ${bgColor};
        color: ${textColor};
        border-radius: var(--radius);
        box-shadow: 0 8px 32px rgba(0,0,0,0.3);
        font-size: 0.875rem;
        font-weight: 500;
        pointer-events: auto;
        animation: toastSlideIn 0.3s ease;
        max-width: 360px;
    `;
    toast.innerHTML = icon + '<span>' + message + '</span>';

    // Thêm vào container
    toastContainer.appendChild(toast);

    // Animation slide in
    const style = document.createElement('style');
    if (!document.getElementById('toastStyles')) {
        style.id = 'toastStyles';
        style.textContent = `
            @keyframes toastSlideIn {
                from { transform: translateX(100%); opacity: 0; }
                to { transform: translateX(0); opacity: 1; }
            }
            @keyframes toastSlideOut {
                from { transform: translateX(0); opacity: 1; }
                to { transform: translateX(100%); opacity: 0; }
            }
        `;
        document.head.appendChild(style);
    }

    // Tự động xóa sau duration
    setTimeout(() => {
        toast.style.animation = 'toastSlideOut 0.3s ease forwards';
        setTimeout(() => toast.remove(), 300);
    }, duration);
}
