(function () {
    'use strict';

    const COLORS = { unknown: [15, 20, 22], free: [24, 32, 35], occupied: [16, 185, 129] };

    // Shared view transform - MUST be exported to window for app.js access
    window.viewTransform = { x: 0, y: 0, scale: 1 };

    function initCanvas() {
        canvas = document.getElementById('mapCanvas');
        if (!canvas) return;
        ctx = canvas.getContext('2d');
        resizeCanvas();
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
            ctx.imageSmoothingEnabled = false; // Bắt buộc false để render pixel SLAM sắc nét
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
        for (let i = 0; i < bytes.length; i++) {
            const c = COLORS[bytes[i] === 0 ? 'unknown' : bytes[i] === 1 ? 'free' : 'occupied'];
            pix[i * 4] = c[0];
            pix[i * 4 + 1] = c[1];
            pix[i * 4 + 2] = c[2];
            pix[i * 4 + 3] = 255;
        }
        return img;
    }

    function worldToCanvas(wx, wy) {
        if (!mapData) return { x: 0, y: 0 };
        const { info } = mapData;
        return {
            x: (wx - info.origin.x) / info.resolution,
            y: (wy - info.origin.y) / info.resolution
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
        viewTransform.scale = Math.min(scaleX, scaleY);
        viewTransform.x = (rect.width - info.width * viewTransform.scale) / 2;
        viewTransform.y = (rect.height - info.height * viewTransform.scale) / 2;
    }

    function render() {
        if (!canvas || !ctx) return;

        const rect = canvas.parentElement ? canvas.parentElement.getBoundingClientRect() : { width: canvas.width, height: canvas.height };

        ctx.imageSmoothingEnabled = false;
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // CHỈ RENDER NẾU SLAM DANG ACTIVE VÀ CÓ DỮ LIỆU MAP
        if (!window.isSlamActive || !mapData || !mapImageData) {
            ctx.fillStyle = '#0F1416';
            ctx.fillRect(0, 0, rect.width, rect.height);
            return;
        }

        const { info } = mapData;
        ctx.save();
        ctx.translate(viewTransform.x, viewTransform.y);
        ctx.scale(viewTransform.scale, viewTransform.scale);

        const tmp = document.createElement('canvas');
        tmp.width = info.width;
        tmp.height = info.height;
        const tmpCtx = tmp.getContext('2d');
        tmpCtx.imageSmoothingEnabled = false;
        tmpCtx.putImageData(mapImageData, 0, 0);

        ctx.drawImage(tmp, 0, 0);

        if (robotPose && robotPose.frame_id === 'map') {
            const pos = robotPose.position;
            const yaw = robotPose.orientation.yaw;
            const cp = worldToCanvas(pos.x, pos.y);
            ctx.save();
            ctx.translate(cp.x, cp.y);
            ctx.rotate(-yaw);
            ctx.fillStyle = '#10B981';
            ctx.beginPath();
            ctx.arc(0, 0, 8 / viewTransform.scale, 0, Math.PI * 2);
            ctx.fill();
            ctx.fillStyle = '#34D399';
            ctx.beginPath();
            ctx.moveTo(12 / viewTransform.scale, 0);
            ctx.lineTo(-4 / viewTransform.scale, -6 / viewTransform.scale);
            ctx.lineTo(-4 / viewTransform.scale, 6 / viewTransform.scale);
            ctx.closePath();
            ctx.fill();
            ctx.restore();
        }
        ctx.restore();
    }

    function moveCanvasToViewport(page) {
        if (!canvas) initCanvas();
        if (!canvas) return;

        const overviewViewport = document.querySelector('.map-viewport');
        const slamViewport = document.querySelector('.slam-map-viewport');
        const targetViewport = (page === 'slam' && slamViewport) ? slamViewport : overviewViewport;

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

        fitToView();
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
            cardStatus.textContent = 'Loaded';
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
        viewTransform.x = (rect.width / 2) - pos.x * viewTransform.scale;
        viewTransform.y = (rect.height / 2) - pos.y * viewTransform.scale;
        render();
    }

    function slamMapZoomIn() {
        viewTransform.scale *= 1.2;
        render();
    }

    function slamMapZoomOut() {
        viewTransform.scale *= 0.8;
        render();
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