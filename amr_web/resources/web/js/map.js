(function () {
    'use strict';

    const COLORS = { 
    unknown: [128, 128, 128], // Màu xám chuẩn ROS 2 (Mã hex: #808080)
    free: [255, 255, 255],    // Màu trắng (Trống / Có thể di chuyển)
    occupied: [0, 0, 0]       // Màu đen chuẩn ROS 2 (Vật cản / Tường)
    };

    // SEPARATE view transforms for Overview and SLAM
    window.overviewViewTransform = { x: 0, y: 0, scale: 1 };
    window.slamViewTransform = { x: 0, y: 0, scale: 1 };
    window.currentViewport = 'overview'; // Track which viewport canvas is in

    function initCanvas() {
        canvas = document.getElementById('mapCanvas');
        if (!canvas) return;
        ctx = canvas.getContext('2d');
        resizeCanvas();
    }

    // Get active viewTransform based on current viewport
    function getActiveViewTransform() {
        return window.currentViewport === 'slam' ? window.slamViewTransform : window.overviewViewTransform;
    }

    function resizeCanvas() {
        if (!canvas || !canvas.parentElement) return;
        const rect = canvas.parentElement.getBoundingClientRect();
        if (rect.width === 0 || rect.height === 0) return;

        canvas.width = rect.width * devicePixelRatio;
        canvas.height = rect.height * devicePixelRatio;
        canvas.style.width = rect.width + 'px';
        canvas.style.height = rect.height + 'px';

        if (ctx) {
            ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
            ctx.imageSmoothingEnabled = false;
        }
        render();
    }

    function buildMapImageData() {
        if (!mapData || !ctx) return null;
        const { info, data } = mapData;
        const decoded = atob(data);
        const bytes = new Uint8Array(decoded.length);
        for (let i = 0; i < decoded.length; i++) bytes[i] = decoded.charCodeAt(i);
        const img = ctx.createImageData(info.width, info.height);
        const pix = img.data;
        // OccupancyGrid starts at its lower-left cell, while Canvas starts at
        // its upper-left pixel.  Flip rows so Canvas +Y maps to ROS -Y.
        for (let y = 0; y < info.height; y++) {
            const canvasY = info.height - 1 - y;
            for (let x = 0; x < info.width; x++) {
                const sourceIndex = y * info.width + x;
                const pixelIndex = (canvasY * info.width + x) * 4;
                const c = COLORS[bytes[sourceIndex] === 0 ? 'unknown' : bytes[sourceIndex] === 1 ? 'free' : 'occupied'];
                pix[pixelIndex] = c[0];
                pix[pixelIndex + 1] = c[1];
                pix[pixelIndex + 2] = c[2];
                pix[pixelIndex + 3] = 255;
            }
        }
        return img;
    }

    function worldToCanvas(wx, wy) {
        if (!mapData) return { x: 0, y: 0 };
        const { info } = mapData;
        return {
            x: (wx - info.origin.x) / info.resolution,
            y: info.height - (wy - info.origin.y) / info.resolution
        };
    }

    function fitToView() {
        if (!mapData || !canvas || !canvas.parentElement) return;
        const { info } = mapData;
        const rect = canvas.parentElement.getBoundingClientRect();
        if (rect.width === 0 || rect.height === 0) return;

        const pad = 20;
        const scaleX = (rect.width - pad * 2) / info.width;
        const scaleY = (rect.height - pad * 2) / info.height;
        const vt = getActiveViewTransform();
        vt.scale = Math.min(scaleX, scaleY);
        vt.x = (rect.width - info.width * vt.scale) / 2;
        vt.y = (rect.height - info.height * vt.scale) / 2;
    }

    function render() {
        if (!canvas || !ctx) return;

        const rect = canvas.parentElement ? canvas.parentElement.getBoundingClientRect() : { width: canvas.width, height: canvas.height };

        ctx.imageSmoothingEnabled = false;
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // CHỈ RENDER MAP KHI CÓ DỮ LIỆU
        if (!mapData || !mapImageData) {
            return; // Để empty state hiển thị
        }

        const { info } = mapData;
        const vt = getActiveViewTransform();
        ctx.save();
        ctx.translate(vt.x, vt.y);
        ctx.scale(vt.scale, vt.scale);

        const tmp = document.createElement('canvas');
        tmp.width = info.width;
        tmp.height = info.height;
        const tmpCtx = tmp.getContext('2d');
        tmpCtx.imageSmoothingEnabled = false;
        tmpCtx.putImageData(mapImageData, 0, 0);

        ctx.drawImage(tmp, 0, 0);

        if (robotPose && robotPose.position && robotPose.orientation) {
            const pos = robotPose.position;
            const yaw = robotPose.orientation.yaw;
            const cp = worldToCanvas(pos.x, pos.y);
            ctx.save();
            ctx.translate(cp.x, cp.y);
            // Canvas +Y points down, so ROS's counter-clockwise yaw is negated.
            ctx.rotate(-yaw);
            ctx.fillStyle = '#10B981';
            ctx.beginPath();
            // Robot footprint: 0.66m x 0.48m = 13.2 x 9.6 pixels @ 0.05m/cell
            // Vẽ bằng map pixels (ko chia scale), tỷ lệ cố định theo map
            ctx.moveTo(6.5, 0);                          // đỉnh phía trước (+0.33m)
            ctx.lineTo(-6.5, -5);                        // vai trái
            ctx.lineTo(-6.5, 5);                         // vai phải
            ctx.closePath();
            ctx.fill();
            ctx.restore();
        }

        // Draw LiDAR scan overlay
        drawLaserScan();

        ctx.restore();
    }

    function moveCanvasToViewport(page) {
        if (!canvas) initCanvas();
        if (!canvas) return;

        const overviewViewport = document.querySelector('.map-viewport');
        const slamViewport = document.querySelector('.slam-map-viewport');
        const targetViewport = (page === 'slam' && slamViewport) ? slamViewport : overviewViewport;

        // Track current viewport for viewTransform selection
        window.currentViewport = (page === 'slam') ? 'slam' : 'overview';

        if (targetViewport && canvas.parentElement !== targetViewport) {
            targetViewport.appendChild(canvas);
            canvas.style.cssText = 'position:absolute;top:0;left:0;width:100%;height:100%;';

            requestAnimationFrame(() => {
                resizeCanvas();
                if (mapData) fitToView();
                render();
            });
        }
    }

    function slamMapShow() {
        const slamEmpty = document.getElementById('slamMapEmpty');

        // Khi show SLAM viewport:
        // - Nếu CÓ mapData → ẩn empty state, vẽ map
        // - Nếu KHÔNG có mapData → hiện "Map Building in Progress"
        if (slamEmpty) {
            slamEmpty.style.display = mapData ? 'none' : 'flex';
        }

        resizeCanvas();
        if (mapData) {
            fitToView();
            render();
        } else {
            render(); // Vẽ nền tối khi chưa có map
        }
    }

    function slamMapHide(fullClear = true) {
        if (fullClear) {
            mapData = null;
            mapImageData = null;
            window.mapData = null;
            window.mapFitted = false;

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

        const slamEmpty = document.getElementById('slamMapEmpty');
        if (slamEmpty) slamEmpty.style.display = 'flex';

        render();
    }

    function slamMapUpdateMap(data) {
        mapData = data;
        window.mapData = data;
        mapImageData = buildMapImageData();

        // Nếu SLAM đang active và có mapData thì ẩn màn hình chờ
        const slamEmpty = document.getElementById('slamMapEmpty');
        if (slamEmpty && window.isSlamActive) {
            slamEmpty.style.display = 'none';
        }

        // Cập nhật Overview Card Active Map
        updateOverviewMapCard(data);

        // Chỉ fitToView lần đầu khi map load, không fit lại mỗi khi map cập nhật
        if (!window.mapFitted) {
            fitToView();
            window.mapFitted = true;
        }

        render();
    }

    function updateOverviewMapCard(data) {
        const cardName = document.getElementById('cardMapName');
        const cardStatus = document.getElementById('cardMapStatus');
        const mapEmpty = document.getElementById('mapEmptyState');
        const mapStatus = document.getElementById('mapStatusDisplay');
        const mapName = document.getElementById('mapNameDisplay');

        // Kiểm tra nếu đang trong chế độ SLAM
        const isSlamMode = window.isSlamActive;

        if (cardName) {
            cardName.textContent = isSlamMode ? 'SLAM Live' : (data.frame_id || 'Active');
            cardName.className = 'metric-value' + (isSlamMode ? ' accent' : ' accent');
        }
        if (cardStatus) {
            cardStatus.textContent = isSlamMode ? 'SLAM Live' : 'Loaded';
        }
        if (mapEmpty) {
            mapEmpty.style.display = 'none';
        }
        if (mapStatus) {
            mapStatus.textContent = 'Live';
            mapStatus.className = 'map-status';
        }
        if (mapName) {
            mapName.textContent = data.frame_id || 'Map';
        }
    }

    function slamMapRecenter() {
        if (!robotPose || !mapData || !canvas) return;

        const rect = canvas.parentElement.getBoundingClientRect();
        const pos = worldToCanvas(robotPose.position.x, robotPose.position.y);
        const vt = getActiveViewTransform();
        vt.x = (rect.width / 2) - pos.x * vt.scale;
        vt.y = (rect.height / 2) - pos.y * vt.scale;
        render();
    }

    function slamMapZoomIn() {
        const vt = getActiveViewTransform();
        vt.scale *= 1.2;
        render();
    }

    function slamMapZoomOut() {
        const vt = getActiveViewTransform();
        vt.scale *= 0.8;
        render();
    }

    /**
     * Draw LiDAR scan data onto the canvas.
     * Laser points are transformed from robot pose to world (map) coordinates,
     * then projected to canvas coordinates using the same worldToCanvas() function
     * used by the robot pose marker.
     *
     * Assumes laser frame == base_link (laser mounted on robot body),
     * so laser scan in laser frame is transformed by robot pose to get world position.
     */
    function drawLaserScan() {
        if (!window.laserData || !robotPose || !mapData) return;

        const laser = window.laserData;
        const ranges = laser.ranges;
        if (!ranges || ranges.length === 0) return;

        const { angle_min, angle_increment, range_min, range_max } = laser;
        const robotYaw = robotPose.orientation.yaw;
        const robotX = robotPose.position.x;
        const robotY = robotPose.position.y;
        const vt = getActiveViewTransform();

        ctx.fillStyle = 'rgba(0, 230, 200, 0.7)';

        for (let i = 0; i < ranges.length; i++) {
            const r = ranges[i];

            // Skip invalid values
            if (r === null || r === undefined) continue;
            if (typeof r !== 'number') continue;
            if (!isFinite(r)) continue;
            if (r < range_min || r > range_max) continue;

            // Compute laser ray angle in laser frame
            const angle = angle_min + i * angle_increment;

            // Transform from laser frame to world frame:
            // Laser point in laser frame: (r*cos(angle), r*sin(angle), 0)
            // World point = robot_pose + R(yaw) * laser_point
            const worldX = robotX + r * Math.cos(angle + robotYaw);
            const worldY = robotY + r * Math.sin(angle + robotYaw);

            // Convert world point to canvas pixel
            const cp = worldToCanvas(worldX, worldY);

            // Draw laser point as a small filled circle
            const radius = 2 / vt.scale;
            ctx.beginPath();
            ctx.arc(cp.x, cp.y, radius, 0, Math.PI * 2);
            ctx.fill();
        }
    }

    // Export public functions sang window
    window.initCanvas = initCanvas;
    window.resizeCanvas = resizeCanvas;
    window.render = render;
    window.renderMap = render;
    window.buildMapImageData = buildMapImageData;
    window.fitMapToView = fitToView;
    window.updateOverviewMapCard = updateOverviewMapCard;

    window.mapRenderer = {
        show: slamMapShow,
        hide: slamMapHide,
        updateMap: slamMapUpdateMap,
        recenter: slamMapRecenter,
        zoomIn: slamMapZoomIn,
        zoomOut: slamMapZoomOut,
        moveToViewport: moveCanvasToViewport
    };

})();
