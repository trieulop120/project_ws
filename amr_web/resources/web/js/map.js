// resources/web/js/map.js
// Phase 4 Fix: Single canvas renderer, canvasToWorld, pointer-event tools
(function () {
    'use strict';

    const COLORS = {
        // Dong bo voi converters.py map_to_json():
        // -1 (unknown) -> 0, 0 (free) -> 1, 100 (occupied) -> 2
        unknown: [205, 205, 205],  // gray - for 0
        free: [254, 254, 254],    // white - for 1
        occupied: [0, 0, 0]       // black - for 2+
    };

    // Shared view transforms for Overview / SLAM / Navigation
    window.overviewViewTransform = { x: 0, y: 0, scale: 1 };
    window.slamViewTransform = { x: 0, y: 0, scale: 1 };
    window.navViewTransform = { x: 0, y: 0, scale: 1 };
    window.currentViewport = 'overview';

    // Fit tracking per viewport
    let viewFitted = {};
    let lastMapSig = '';

    let canvas = null;
    let ctx = null;

    // Performance: cache offscreen canvas
    let mapOffscreen = null;
    let renderQueued = false;

    function initCanvas() {
        canvas = document.getElementById('mapCanvas');
        if (!canvas) return;
        ctx = canvas.getContext('2d');
        resizeCanvas();
    }

    function requestRender() {
        if (renderQueued) return;
        renderQueued = true;
        requestAnimationFrame(() => { renderQueued = false; render(); });
    }
    window.renderMap = requestRender;

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
        requestRender();
    }

    function getActiveViewTransform() {
        switch (window.currentViewport) {
            case 'slam': return window.slamViewTransform;
            case 'navigation': return window.navViewTransform;
            default: return window.overviewViewTransform;
        }
    }
    window.getActiveViewTransform = getActiveViewTransform;

    function buildMapImageData() {
        if (!window.mapData || !ctx) return null;
        const { info, data, is_pgm_raw } = window.mapData;
        const decoded = atob(data);
        const bytes = new Uint8Array(decoded.length);
        for (let i = 0; i < decoded.length; i++) bytes[i] = decoded.charCodeAt(i);
        const img = ctx.createImageData(info.width, info.height);
        const pix = img.data;

        for (let y = 0; y < info.height; y++) {
            // B04 Fix: PGM lưu top-down (row 0 = trên), KHÔNG flip. OccupancyGrid lưu bottom-up (row 0 = dưới), flip.
            const canvasY = is_pgm_raw ? y : info.height - 1 - y;
            for (let x = 0; x < info.width; x++) {
                const sourceIndex = y * info.width + x;
                const pixelIndex = (canvasY * info.width + x) * 4;

                const val = bytes[sourceIndex];
                let r, g, b;

                if (is_pgm_raw) {
                    // CHE DO MAP TINH PGM (Navigation / Overview)
                    // Doc truc tiep muc xam PGM (205: gray/unknown, 254: white/free, 0: black/occupied)
                    r = g = b = val;
                } else {
                    // CHE DO SLAM LIVE (OccupancyGrid qua WS: 0=unknown, 1=free, 2=occupied)
                    // Dong bo voi converters.py map_to_json(): -1->0, 0->1, 100->2
                    const c = COLORS[val === 0 ? 'unknown' : val === 1 ? 'free' : 'occupied'];
                    r = c[0]; g = c[1]; b = c[2];
                }

                pix[pixelIndex] = r;
                pix[pixelIndex + 1] = g;
                pix[pixelIndex + 2] = b;
                pix[pixelIndex + 3] = 255;
            }
        }
        return img;
    }

    // B04 Fix: worldToCanvas - chuan hoa top-down: y tang = di len = canvasY giam
    // buildMapImageData() da flip OccupancyGrid, nen chi can 1 cong thuc cho ca hai
    function worldToCanvas(wx, wy) {
        if (!window.mapData) return { x: 0, y: 0 };
        const { info } = window.mapData;
        return {
            x: (wx - info.origin.x) / info.resolution,
            // origin thuong o duoi-trai, wy tang = di len trong the gioi
            // tren canvas top-down, di len = canvasY giam
            y: info.height - (wy - info.origin.y) / info.resolution
        };
    }

    // B04 Fix: canvasToWorld - reverse cua worldToCanvas
    function canvasToWorld(px, py) {
        if (!window.mapData) return null;
        const { info } = window.mapData;
        const vt = getActiveViewTransform();
        const cellX = (px - vt.x) / vt.scale;
        const cellY = (py - vt.y) / vt.scale;
        return {
            x: info.origin.x + cellX * info.resolution,
            y: info.origin.y + (info.height - cellY) * info.resolution
        };
    }
    window.canvasToWorld = canvasToWorld;

    function fitToView() {
        if (!window.mapData || !canvas || !canvas.parentElement) return;
        const { info } = window.mapData;
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

        ctx.imageSmoothingEnabled = false;
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        if (!window.mapData || !window.mapImageData) return;

        const { info } = window.mapData;
        const vt = getActiveViewTransform();
        ctx.save();
        ctx.translate(vt.x, vt.y);
        ctx.scale(vt.scale, vt.scale);

        // Draw cached map
        if (mapOffscreen) {
            ctx.drawImage(mapOffscreen, 0, 0);
        } else {
            const tmp = document.createElement('canvas');
            tmp.width = info.width;
            tmp.height = info.height;
            tmp.getContext('2d').imageSmoothingEnabled = false;
            tmp.getContext('2d').putImageData(window.mapImageData, 0, 0);
            ctx.drawImage(tmp, 0, 0);
        }

        // Draw route overlay for navigation
        drawRouteOverlay();

        // Draw robot
        if (window.robotPose && window.robotPose.position && window.robotPose.orientation) {
            const pos = window.robotPose.position;
            const yaw = window.robotPose.orientation.yaw;
            const cp = worldToCanvas(pos.x, pos.y);
            ctx.save();
            ctx.translate(cp.x, cp.y);
            ctx.rotate(-yaw);
            ctx.fillStyle = '#10B981';
            ctx.beginPath();
            ctx.moveTo(6.5, 0);
            ctx.lineTo(-6.5, -5);
            ctx.lineTo(-6.5, 5);
            ctx.closePath();
            ctx.fill();
            ctx.restore();
        }

        // Draw LiDAR scan
        drawLaserScan();

        // Draw drag preview (navigation tools)
        drawDragPreview();

        ctx.restore();
    }

    function drawLaserScan() {
        if (!window.laserData || !window.robotPose || !window.mapData) return;
        const laser = window.laserData;
        const ranges = laser.ranges;
        if (!ranges || ranges.length === 0) return;

        const { angle_min, angle_increment, range_min, range_max } = laser;
        const robotYaw = window.robotPose.orientation.yaw;
        const robotX = window.robotPose.position.x;
        const robotY = window.robotPose.position.y;
        const vt = getActiveViewTransform();

        ctx.fillStyle = 'rgba(0, 230, 200, 0.7)';

        for (let i = 0; i < ranges.length; i++) {
            const r = ranges[i];
            if (r === null || r === undefined || typeof r !== 'number' || !isFinite(r)) continue;
            if (r < range_min || r > range_max) continue;

            const angle = angle_min + i * angle_increment;
            const worldX = robotX + r * Math.cos(angle + robotYaw);
            const worldY = robotY + r * Math.sin(angle + robotYaw);
            const cp = worldToCanvas(worldX, worldY);

            ctx.beginPath();
            ctx.arc(cp.x, cp.y, 2 / vt.scale, 0, Math.PI * 2);
            ctx.fill();
        }
    }

    function drawRouteOverlay() {
        if (!window.showRouteOverlay || !window.currentRoute || !window.currentRoute.nodes ||
            window.currentViewport !== 'navigation') return;

        const route = window.currentRoute;
        const vt = getActiveViewTransform();

        ctx.save();

        // Edges
        if (route.edges && route.edges.length > 0) {
            ctx.strokeStyle = '#34d399';
            ctx.lineWidth = 2.5 / vt.scale;

            route.edges.forEach(edge => {
                const fromNode = route.nodes.find(n => n.name === edge.from);
                const toNode = route.nodes.find(n => n.name === edge.to);
                if (fromNode && toNode) {
                    const p1 = worldToCanvas(fromNode.position.x, fromNode.position.y);
                    const p2 = worldToCanvas(toNode.position.x, toNode.position.y);
                    ctx.beginPath();
                    ctx.moveTo(p1.x, p1.y);
                    ctx.lineTo(p2.x, p2.y);
                    ctx.stroke();
                }
            });
        }

        // Nodes
        route.nodes.forEach(node => {
            const cp = worldToCanvas(node.position.x, node.position.y);
            const radius = (node.type === 'home' ? 5 : 3.5) / vt.scale;
            const colors = {
                home: '#f97316',
                pickup_approach: '#3b82f6',
                dropoff_approach: '#ef4444',
                transit: '#10B981',
                charging: '#9ca3af'
            };
            ctx.fillStyle = colors[node.type] || '#10B981';
            ctx.beginPath();
            ctx.arc(cp.x, cp.y, radius, 0, Math.PI * 2);
            ctx.fill();
            ctx.strokeStyle = '#ffffff';
            ctx.lineWidth = 1 / vt.scale;
            ctx.stroke();

            if (vt.scale > 15) {
                ctx.fillStyle = '#ffffff';
                ctx.font = `${Math.max(10, 12 / vt.scale)}px sans-serif`;
                ctx.textAlign = 'center';
                ctx.fillText(node.name, cp.x, cp.y - radius * 2);
            }
        });

        ctx.restore();
    }

    // P0-6 Fix: Draw drag preview for nav tools
    function drawDragPreview() {
        const st = window.navDragState;
        if (!st || !st.isDragging || window.currentViewport !== 'navigation') return;
        const vt = getActiveViewTransform();
        const p0 = worldToCanvas(st.startMapX, st.startMapY);
        const color = st.tool === 'pose' ? '#FFA500' : '#10B981';

        ctx.save();
        ctx.fillStyle = color;
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2 / vt.scale;
        ctx.beginPath();
        ctx.arc(p0.x, p0.y, 6 / vt.scale, 0, Math.PI * 2);
        ctx.fill();
        ctx.stroke();

        // Arrow
        const dx = st.currentX - st.startX, dy = st.currentY - st.startY;
        if (Math.hypot(dx, dy) > 5) {
            const len = Math.min(Math.hypot(dx, dy), 80) / vt.scale;
            ctx.strokeStyle = color;
            ctx.lineWidth = 3 / vt.scale;
            ctx.beginPath();
            ctx.moveTo(p0.x, p0.y);
            ctx.lineTo(p0.x + Math.cos(st.yaw) * len, p0.y - Math.sin(st.yaw) * len);
            ctx.stroke();
        }
        ctx.restore();
    }

    // Viewport management
    const VIEWPORT_EL = {
        overview: () => document.getElementById('mapViewport'),
        slam: () => document.querySelector('#page-slam .slam-map-viewport'),
        navigation: () => document.getElementById('navMapViewport'),
    };

    function moveCanvasToViewport(page) {
        if (!canvas) initCanvas();
        if (!canvas) return;

        const key = VIEWPORT_EL[page] ? page : 'overview';
        const target = VIEWPORT_EL[key]();
        window.currentViewport = key;

        if (target && canvas.parentElement !== target) {
            target.appendChild(canvas);
            canvas.style.cssText = 'position:absolute;top:0;left:0;width:100%;height:100%;';
        }

        requestAnimationFrame(() => {
            resizeCanvas();
            if (window.mapData && !viewFitted[key]) { fitToView(); viewFitted[key] = true; }
            render();
        });
    }

    function slamMapShow() {
        const slamEmpty = document.getElementById('slamMapEmpty');
        if (slamEmpty) slamEmpty.style.display = window.mapData ? 'none' : 'flex';
        resizeCanvas();
        if (window.mapData) {
            if (!viewFitted[window.currentViewport]) { fitToView(); viewFitted[window.currentViewport] = true; }
            render();
        } else {
            render();
        }
    }

    function slamMapHide(fullClear = true) {
        if (fullClear) {
            window.mapData = null;
            window.mapImageData = null;
            mapOffscreen = null;
            viewFitted = {};
            lastMapSig = '';

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
        const i = data.info;
        const sig = `${i.width}x${i.height}@${i.resolution}:${i.origin.x},${i.origin.y}`;
        if (!window.isSlamActive && sig !== lastMapSig) {
            lastMapSig = sig;
            viewFitted = {};
        }

        window.mapData = data;
        window.mapImageData = buildMapImageData();

        const slamEmpty = document.getElementById('slamMapEmpty');
        if (slamEmpty && window.isSlamActive) slamEmpty.style.display = 'none';

        updateOverviewMapCard(data);

        if (!viewFitted[window.currentViewport]) {
            fitToView();
            viewFitted[window.currentViewport] = true;
        }
        render();
    }

    function updateOverviewMapCard(data) {
        // B08 Fix: KHÔNG ghi đè card khi Navigation không active và không có selectedNavMap
        // Điều này ngăn card bị ghi đè sau khi Stop Navigation
        const isNavActive = window.isNavigationActive && window.selectedNavMap;

        if (!window.isSlamActive && !isNavActive) {
            // Navigation đã stop - không cập nhật card nữa (resetNavMapCard đã xử lý)
            return;
        }

        const cardName = document.getElementById('cardMapName');
        const cardStatus = document.getElementById('cardMapStatus');
        const mapEmpty = document.getElementById('mapEmptyState');
        const mapStatus = document.getElementById('mapStatusDisplay');
        const mapName = document.getElementById('mapNameDisplay');
        const isSlamMode = window.isSlamActive;

        if (cardName) {
            // B07 Fix: Ưu tiên selectedNavMap, fallback '--' (không hiển thị 'map', 'Map', 'Active')
            cardName.textContent = isSlamMode ? 'SLAM Live' : (window.selectedNavMap || '--');
            cardName.className = 'metric-value' + (isSlamMode ? ' accent' : ' accent');
        }
        if (cardStatus) cardStatus.textContent = isSlamMode ? 'SLAM Live' : 'Loaded';
        if (mapEmpty) mapEmpty.style.display = 'none';
        if (mapStatus) { mapStatus.textContent = 'Live'; mapStatus.className = 'map-status'; }
        if (mapName) {
            // B07 Fix: KHÔNG hiển thị "Map" - ưu tiên selectedNavMap hoặc '--'
            mapName.textContent = window.selectedNavMap || '--';
            mapName.className = 'map-name' + (window.selectedNavMap ? '' : ' muted');
        }
    }

    function slamMapRecenter() {
        if (!window.robotPose || !window.mapData || !canvas) return;
        const rect = canvas.parentElement.getBoundingClientRect();
        const pos = worldToCanvas(window.robotPose.position.x, window.robotPose.position.y);
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

    // B11/B22 Fix: setStaticMap - nạp map tĩnh với bbox fitting
    function setStaticMap(mapId, info, imageDataBase64) {
        if (!ctx) return;

        // Decode base64 image data
        const decoded = atob(imageDataBase64);
        const bytes = new Uint8Array(decoded.length);
        for (let i = 0; i < decoded.length; i++) bytes[i] = decoded.charCodeAt(i);

        // B22: Tính bbox của các ô đã biết (≠ 205)
        let minX = info.width, maxX = -1, minY = info.height, maxY = -1;
        for (let y = 0; y < info.height; y++) {
            for (let x = 0; x < info.width; x++) {
                const idx = y * info.width + x;
                if (bytes[idx] !== 205) {  // 205 = unknown/unexplored
                    if (x < minX) minX = x;
                    if (x > maxX) maxX = x;
                    if (y < minY) minY = y;
                    if (y > maxY) maxY = y;
                }
            }
        }

        // Nếu không có ô đã biết → dùng toàn bộ map
        if (maxX < minX || maxY < minY) {
            minX = 0; maxX = info.width - 1;
            minY = 0; maxY = info.height - 1;
        }

        // Cache mapData
        window.mapData = {
            info,
            data: imageDataBase64,
            is_pgm_raw: true,
            frame_id: mapId
        };

        // Cache bbox cho fitToView
        window.mapData._bbox = { minX, maxX, minY, maxY };

        // Build image data (PGM mode)
        window.mapImageData = buildMapImageData();

        // Cache offscreen canvas
        if (mapOffscreen) {
            mapOffscreen = null;
        }
        mapOffscreen = document.createElement('canvas');
        mapOffscreen.width = info.width;
        mapOffscreen.height = info.height;
        const offCtx = mapOffscreen.getContext('2d');
        offCtx.imageSmoothingEnabled = false;
        offCtx.putImageData(window.mapImageData, 0, 0);

        // Reset viewFitted cho nav viewport
        viewFitted[window.currentViewport] = false;

        // Fit to view dựa trên bbox với pad
        const pad = 24;  // B22: padding pixels
        const bboxWidth = maxX - minX + 1;
        const bboxHeight = maxY - minY + 1;
        fitToViewBbox(minX, minY, bboxWidth, bboxHeight, pad);

        // Update overview card
        if (typeof window.updateOverviewMapCard === 'function') {
            window.updateOverviewMapCard(window.mapData);
        }

        // Render
        render();
    }
    window.setStaticMap = setStaticMap;

    // B22: fitToViewBbox - fit theo bounding box với padding
    function fitToViewBbox(bboxMinX, bboxMinY, bboxWidth, bboxHeight, padPx) {
        if (!canvas || !canvas.parentElement) return;
        const rect = canvas.parentElement.getBoundingClientRect();
        if (rect.width === 0 || rect.height === 0) return;

        const vt = getActiveViewTransform();
        const scaleX = (rect.width - padPx * 2) / bboxWidth;
        const scaleY = (rect.height - padPx * 2) / bboxHeight;
        vt.scale = Math.min(scaleX, scaleY);

        // Center bbox trong viewport
        vt.x = (rect.width - bboxWidth * vt.scale) / 2 - bboxMinX * vt.scale;
        vt.y = (rect.height - bboxHeight * vt.scale) / 2 - bboxMinY * vt.scale;
    }

    // Override fitToView để dùng bbox nếu có
    const _origFitToView = fitToView;
    fitToView = function(opts) {
        if (window.mapData && window.mapData._bbox && window.currentViewport === 'navigation') {
            const b = window.mapData._bbox;
            fitToViewBbox(b.minX, b.minY, b.maxX - b.minX + 1, b.maxY - b.minY + 1, 24);
            viewFitted[window.currentViewport] = true;
            return;
        }
        return _origFitToView.call(this, opts);
    };
    window.fitToView = fitToView;

    // Exports
    window.initCanvas = initCanvas;
    window.resizeCanvas = resizeCanvas;
    window.buildMapImageData = buildMapImageData;  // Export for navigation.js
    // B07 Fix: Export updateOverviewMapCard cho navigation.js
    window.updateOverviewMapCard = updateOverviewMapCard;

    window.mapRenderer = {
        show: slamMapShow,
        hide: slamMapHide,
        updateMap: slamMapUpdateMap,
        recenter: slamMapRecenter,
        zoomIn: slamMapZoomIn,
        zoomOut: slamMapZoomOut,
        moveToViewport: moveCanvasToViewport,
        canvasToWorld: canvasToWorld,
    };

})();
