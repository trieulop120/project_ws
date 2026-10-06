// Global Application State
let mapData = null;
let robotPose = null;
let mapImageData = null;
let isWsConnected = false;
let isRobotConnected = false;
let activityLog = [];
let alarmLog = [];
const MAX_ACTIVITY = 50;
const MAX_ALARMS = 20;
let alarmFilter = 'all';

// Device & System State (Heartbeat Monitoring)
let deviceStates = {
    esp32: { connected: false, hz: 0 },
    lidar: { connected: false, hz: 0 },
    camera: { connected: false, hz: 0 },
    lift: { connected: false, hz: 0 }
};
let systemStatus = 'Offline';  // "Online" | "Degraded" | "Offline"

// Map View Transform State
let viewTransform = { x: 0, y: 0, scale: 1 };
let isDragging = false;
let dragStart = { x: 0, y: 0 };
let transformStart = { x: 0, y: 0, scale: 1 };
let canvas = null;
let ctx = null;

// SLAM Page State (on window for cross-module access)
window.isSlamActive = false;
window.laserData = null;  // Latest laser scan data
window.slamPendingStart = false;  // Track if start_slam command is pending

/**
 * Cập nhật laser scan data từ backend
 * @param {Object} d - Laser scan JSON payload
 */
function updateLaser(d) {
    window.laserData = d;
}

/**
 * Xử lý command response từ backend
 * @param {Object} d - Command response payload
 */
function handleCommandResponse(d) {
    const cmd = d.command || 'unknown';
    const status = d.status || 'unknown';
    const message = d.message || '';

    console.log('Command response:', cmd, status, message);

    if (cmd === 'start_slam') {
        window.slamPendingStart = false;  // Clear pending flag

        if (status === 'started') {
            addActivity('info', 'SLAM started successfully');
            window.isSlamActive = true;
            updateSlamUI();
        } else if (status === 'failed') {
            addAlarm('error', 'SLAM start failed: ' + message);
            window.isSlamActive = false;
            updateSlamUI();
        }
    } else if (cmd === 'stop_slam') {
        addActivity('info', 'SLAM stopped');
        window.isSlamActive = false;
        // Clear map when SLAM is stopped
        if (window.mapRenderer) {
            window.mapRenderer.hide();
        }
        updateSlamUI();
    } else if (cmd === 'save_map') {
        if (status === 'success') {
            addActivity('info', 'Map saved: ' + (d.path || 'unknown path'));
        } else {
            addAlarm('error', 'Map save failed: ' + message);
        }
    } else if (cmd === 'teleop') {
        // Teleop không cần feedback
    } else if (cmd === 'lift') {
        // Lift không cần feedback
    }
}

/**
 * Cập nhật SLAM UI dựa trên trạng thái isSlamActive
 */
function updateSlamUI() {
    const slamWelcome = document.getElementById('slamWelcome');
    const slamLayout = document.getElementById('slamLayout');
    const slamStatusBadge = document.querySelector('.slam-status-badge');

    if (slamWelcome && slamLayout) {
        if (window.isSlamActive) {
            slamWelcome.style.display = 'none';
            slamLayout.style.display = 'grid';
        } else {
            slamWelcome.style.display = 'flex';
            slamLayout.style.display = 'none';
        }
    }

    if (slamStatusBadge) {
        if (window.isSlamActive) {
            slamStatusBadge.textContent = 'SLAM Active';
            slamStatusBadge.className = 'slam-status-badge active';
        } else {
            slamStatusBadge.textContent = 'SLAM Inactive';
            slamStatusBadge.className = 'slam-status-badge';
        }
    }
}

/**
 * Cập nhật Operating Mode display trên Overview card
 */
function updateOpModeDisplay() {
    const opModeEl = document.getElementById('cardOpMode');
    const subModeEl = document.getElementById('cardSubMode');

    if (window.isSlamActive) {
        if (opModeEl) {
            opModeEl.textContent = 'SLAM Mode';
            opModeEl.className = 'metric-value accent';
        }
        if (subModeEl) {
            subModeEl.className = 'mode-badge slam';
            subModeEl.innerHTML = '<span>Mapping Active</span>';
        }
    } else {
        if (opModeEl) {
            opModeEl.textContent = 'Navigation';
            opModeEl.className = 'metric-value muted';
        }
        if (subModeEl) {
            subModeEl.className = 'mode-badge nav';
            subModeEl.innerHTML = '<span>Idle</span>';
        }
    }
}
