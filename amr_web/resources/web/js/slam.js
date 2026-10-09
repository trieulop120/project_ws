// SLAM Page Logic
(function() {
    'use strict';

    // ========================================
    // Global State
    // ========================================
    window.isSlamActive = false;

    // DOM Elements
    const btnStartSlam = document.getElementById('btnStartSlam');
    const btnStopSlam = document.getElementById('btnStopSlam');
    const btnSaveMap = document.getElementById('btnSaveMap');
    const slamLinearSpeed = document.getElementById('slamLinearSpeed');
    const slamAngularSpeed = document.getElementById('slamAngularSpeed');
    const btnSetHeight = document.getElementById('btnSetHeight');
    const btnSetHome = document.getElementById('btnSetHome');
    const teleopKeys = document.querySelectorAll('.teleop-btn');

    // Teleop State
    let teleopInterval = null;
    let teleopVelocity = { linear: 0, angular: 0 };

    // ========================================
    // Position Display Update
    // ========================================

    function updateSlamPosition() {
        if (!robotPose) return;
        const posX = document.getElementById('slamPosX');
        const posY = document.getElementById('slamPosY');
        const posYaw = document.getElementById('slamPosYaw');

        if (posX) { posX.textContent = robotPose.position.x.toFixed(2); posX.classList.remove('muted'); }
        if (posY) { posY.textContent = robotPose.position.y.toFixed(2); posY.classList.remove('muted'); }
        if (posYaw) {
            posYaw.textContent = (robotPose.orientation.yaw * 180 / Math.PI).toFixed(1);
            posYaw.classList.remove('muted');
        }
    }

    // ========================================
    // SLAM Control Functions
    // ========================================

    function onStartSlamClick() {
        console.log('[SLAM] Start clicked');

        // 1. CLEAR old map data - start fresh for new SLAM
        window.mapData = null;
        window.mapImageData = null;
        window.mapFitted = false;

        // 2. Set SLAM active
        window.isSlamActive = true;

        // 3. Hiển thị khung UI SLAM trước
        showSlamActiveUI();
        updateSlamUI();
        if (typeof updateOpModeDisplay === 'function') updateOpModeDisplay();

        // 4. Chuyển Canvas sang viewport SLAM & Bật hiển thị ngay lập tức (Synchrous)
        if (window.mapRenderer) {
            window.mapRenderer.moveToViewport('slam');
            window.mapRenderer.show();
        }

        // 5. Gửi lệnh WebSocket Start SLAM
        const sent = sendCommand({ command: 'start_slam' });
        if (!sent) {
            addAlarm('error', 'Cannot start SLAM: WebSocket not connected');
        }

        // 6. Đợi slam_toolbox khởi chạy xong (~3s) mới gửi lệnh xin map
        setTimeout(() => {
            if (window.isSlamActive) {
                console.log('[SLAM] Requesting current map data (Attempt 1)...');
                sendCommand({ command: 'get_map' });
            }
        }, 3000);

        // 7. Thử lại lần 2 sau 5.5 giây nếu vẫn chưa có mapData
        setTimeout(() => {
            if (window.isSlamActive && !window.mapData) {
                console.log('[SLAM] Requesting current map data (Attempt 2)...');
                sendCommand({ command: 'get_map' });
            }
        }, 5500);
    }

    function onStopSlamClick() {
        console.log('[SLAM] Stop clicked');

        stopTeleop();

        // 1. Set SLAM inactive
        window.isSlamActive = false;

        // 2. CLEAR map data completely - map no longer exists
        window.mapData = null;
        window.mapImageData = null;
        window.mapFitted = false;

        // 3. Xóa toàn bộ dữ liệu Map & ép vẽ rỗng lập tức
        if (window.mapRenderer) {
            window.mapRenderer.hide(true);
        }

        // 4. Reset Overview map card to "No map loaded"
        resetOverviewMapCard();

        // 5. Chuyển UI về màn hình chờ SLAM
        hideSlamActiveUI();
        updateSlamUI();
        if (typeof updateOpModeDisplay === 'function') updateOpModeDisplay();

        // 6. Gửi lệnh WebSocket Stop SLAM
        sendCommand({ command: 'stop_slam' });
    }

    function onSaveMapClick() {
        const mapNameInput = document.getElementById('slamMapName');
        const mapName = mapNameInput?.value?.trim() || 'map_01';
        console.log('[SLAM] Save map clicked, name:', mapName);
        const sent = sendCommand({ command: 'save_map', map_name: mapName });
        console.log('[SLAM] Save command sent:', sent);
        if (!sent) {
            addAlarm('error', 'Cannot save map: WebSocket not connected');
        }
    }

    // ========================================
    // UI Helpers
    // ========================================

    function showSlamActiveUI() {
        const slamWelcome = document.getElementById('slamWelcome');
        const slamLayout = document.getElementById('slamLayout');
        if (slamWelcome) slamWelcome.style.display = 'none';
        if (slamLayout) {
            slamLayout.style.display = 'grid';
            slamLayout.style.animation = 'fadeIn 0.4s ease';
        }
    }

    function hideSlamActiveUI() {
        const slamWelcome = document.getElementById('slamWelcome');
        const slamLayout = document.getElementById('slamLayout');
        if (slamWelcome) slamWelcome.style.display = 'flex';
        if (slamLayout) slamLayout.style.display = 'none';
    }

    function resetOverviewMapCard() {
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

    // ========================================
    // Teleop Functions - HOLD TO DRIVE
    // ========================================

    function sendTeleopVelocity() {
        const linearSpeed = parseFloat(slamLinearSpeed?.value) || 0.25;
        const angularSpeed = parseFloat(slamAngularSpeed?.value) || 0.8;
        const vx = linearSpeed * teleopVelocity.linear;
        const wz = angularSpeed * teleopVelocity.angular;
        sendCommand({ command: 'teleop', linear: vx, angular: wz });
    }

    function startTeleop(linear, angular) {
        if (teleopInterval) clearInterval(teleopInterval);
        teleopVelocity = { linear, angular };
        sendTeleopVelocity();
        teleopInterval = setInterval(sendTeleopVelocity, 100);
    }

    function stopTeleop() {
        if (teleopInterval) { clearInterval(teleopInterval); teleopInterval = null; }
        teleopVelocity = { linear: 0, angular: 0 };
        sendCommand({ command: 'teleop', linear: 0, angular: 0 });
    }

    function handleTeleopKeyDown(keyElement, keyChar) {
        keyElement.classList.add('pressed');
        const keyMap = {
            'u': { linear: 1, angular: 1 }, 'i': { linear: 1, angular: 0 }, 'o': { linear: 1, angular: -1 },
            'j': { linear: 0, angular: 1 }, 'k': { linear: 0, angular: 0 }, 'l': { linear: 0, angular: -1 },
            'm': { linear: -1, angular: 1 }, ',': { linear: -1, angular: 0 }, '.': { linear: -1, angular: -1 }
        };
        const vel = keyMap[keyChar];
        if (vel) startTeleop(vel.linear, vel.angular);
    }

    function handleTeleopKeyUp(keyElement) {
        keyElement.classList.remove('pressed');
        stopTeleop();
    }

    // ========================================
    // Lift Control
    // ========================================

    function setHeight() {
        const height = parseInt(document.getElementById('slamTargetHeight')?.value) || 0;
        const sent = sendCommand({ command: 'lift', action: 'SET_HEIGHT', target_mm: height });
        if (!sent) addAlarm('error', 'Cannot send lift command: WebSocket not connected');
    }

    function setHome() {
        const sent = sendCommand({ command: 'lift', action: 'HOME', target_mm: 0 });
        if (!sent) addAlarm('error', 'Cannot send lift command: WebSocket not connected');
    }

    // ========================================
    // Map Controls
    // ========================================

    function onRecenterClick() {
        if (window.mapRenderer) window.mapRenderer.recenter();
    }

    function onZoomInClick() {
        if (window.mapRenderer) window.mapRenderer.zoomIn();
    }

    function onZoomOutClick() {
        if (window.mapRenderer) window.mapRenderer.zoomOut();
    }

    // ========================================
    // Keyboard Support
    // ========================================

    const keyboardKeyMap = {
        'u': 'u', 'i': 'i', 'o': 'o', 'j': 'j', 'k': 'k', 'l': 'l', 'm': 'm', ',': ',', '.': '.', 'h': 'set_home'
    };

    function handleKeyboardDown(e) {
        if (!window.isSlamActive) return;
        const target = e.target;
        if (target.tagName === 'INPUT' || target.tagName === 'BUTTON' || target.tagName === 'TEXTAREA') return;

        const key = e.key.toLowerCase();
        const mappedKey = keyboardKeyMap[key];
        if (!mappedKey) return;
        e.preventDefault();

        if (mappedKey === 'set_home') {
            setHome();
        } else {
            const keyElement = document.querySelector(`.teleop-btn[data-key="${mappedKey}"]`);
            if (keyElement) handleTeleopKeyDown(keyElement, mappedKey);
        }
    }

    function handleKeyboardUp(e) {
        if (!window.isSlamActive) return;
        const key = e.key.toLowerCase();
        const mappedKey = keyboardKeyMap[key];
        if (!mappedKey) return;
        const keyElement = document.querySelector(`.teleop-btn[data-key="${mappedKey}"]`);
        if (keyElement) handleTeleopKeyUp(keyElement);
    }

    // ========================================
    // Periodic Updates
    // ========================================

    function updateLoop() {
        if (window.isSlamActive) {
            updateSlamPosition();
        }
        requestAnimationFrame(updateLoop);
    }

    // ========================================
    // Event Listeners
    // ========================================

    function initEventListeners() {
        btnStartSlam?.addEventListener('click', onStartSlamClick);
        btnStopSlam?.addEventListener('click', onStopSlamClick);
        btnSaveMap?.addEventListener('click', onSaveMapClick);
        btnSetHeight?.addEventListener('click', setHeight);
        btnSetHome?.addEventListener('click', setHome);

        document.getElementById('slamMapName')?.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') {
                e.preventDefault();
                onSaveMapClick();
            }
        });
        document.getElementById('slamTargetHeight')?.addEventListener('keydown', (e) => {
            if (e.key === 'Enter') {
                e.preventDefault();
                setHeight();
            }
        });

        teleopKeys.forEach(key => {
            key.addEventListener('mousedown', () => handleTeleopKeyDown(key, key.dataset.key));
            key.addEventListener('mouseup', () => handleTeleopKeyUp(key));
            key.addEventListener('mouseleave', () => handleTeleopKeyUp(key));
            key.addEventListener('touchstart', (e) => { e.preventDefault(); handleTeleopKeyDown(key, key.dataset.key); }, { passive: false });
            key.addEventListener('touchend', (e) => { e.preventDefault(); handleTeleopKeyUp(key); });
            key.addEventListener('touchcancel', () => handleTeleopKeyUp(key));
        });

        document.getElementById('slamBtnRecenter')?.addEventListener('click', onRecenterClick);
        document.getElementById('slamBtnZoomIn')?.addEventListener('click', onZoomInClick);
        document.getElementById('slamBtnZoomOut')?.addEventListener('click', onZoomOutClick);

        document.addEventListener('keydown', handleKeyboardDown);
        document.addEventListener('keyup', handleKeyboardUp);

        updateLoop();
    }

    // ========================================
    // Initialize
    // ========================================

    function initSlamPage() {
        console.log('[SLAM] Module Initializing...');
        initEventListeners();
        console.log('[SLAM] Module Initialized!');
    }

    // Called when navigating to SLAM page
    window.onNavigateToSlam = function() {
        if (window.isSlamActive && window.mapRenderer) {
            window.mapRenderer.moveToViewport('slam');
            setTimeout(() => {
                window.mapRenderer.show();
                if (window.mapData) window.mapRenderer.updateMap(window.mapData);
            }, 50);
        }
    };

    // Export for other modules
    window.resetOverviewMapCard = resetOverviewMapCard;

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initSlamPage);
    } else {
        initSlamPage();
    }

})();
