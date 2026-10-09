// Map & Route Page Module
(function() {
    'use strict';

    // DOM Elements
    const routeNodeTable = document.getElementById('routeNodeTable');
    const nodeEditor = document.getElementById('nodeEditor');
    const mapListPanel = document.getElementById('mapListPanel');
    const routeListPanel = document.getElementById('routeListPanel');
    const routeCanvas = document.getElementById('routeCanvas');
    const routeEmptyState = document.getElementById('routeEmptyState');

    // State
    let availableMaps = [];
    let availableRoutes = {};
    let selectedMapId = null;
    let selectedRouteId = null;
    let selectedNodeId = null;
    let currentRoute = { graph_name: '', nodes: [], edges: [] };
    let currentTool = 'select';
    let hasUnsavedChanges = false;

    // Canvas state for pan/zoom
    let canvasScale = 1;
    let canvasOffsetX = 0;
    let canvasOffsetY = 0;
    let isDragging = false;
    let lastMouseX = 0;
    let lastMouseY = 0;

    // Edge drawing state
    let edgeStartNodeIdx = null;
    let pendingMousePos = null;

    // Map data
    let currentMapImage = null;
    let currentMapInfo = null;
    let canvasInteractionsBound = false;
    let mapRouteControlsBound = false;

    // COLORS for map rendering (same as map.js)
    const MAP_COLORS = {
        unknown: [128, 128, 128],
        free: [255, 255, 255],
        occupied: [0, 0, 0]
    };

    function inferNodeType(name) {
        const upper = (name || '').trim().toUpperCase();
        if (upper.startsWith('HOME') || upper.startsWith('NODE_HOME')) return 'home';
        if (upper.startsWith('P_') || upper.startsWith('PICKUP')) return 'pickup_approach';
        if (upper.startsWith('D_') || upper.startsWith('DROP') || upper.startsWith('DROPOFF')) return 'dropoff_approach';
        if (upper.startsWith('SAFE') || upper.startsWith('C_') || upper.startsWith('CHARGE')) return 'charging';
        return 'transit';
    }

    // Utility Functions
    function showConfirmModal(title, message, onConfirm, onCancel) {
        const existingModal = document.querySelector('.confirm-modal');
        if (existingModal) existingModal.remove();

        const modal = document.createElement('div');
        modal.className = 'confirm-modal';
        modal.innerHTML = `
            <div class="confirm-overlay"></div>
            <div class="confirm-dialog">
                <h4>${title}</h4>
                <p>${message}</p>
                <div class="confirm-actions">
                    <button class="btn btn-secondary" id="confirmCancel">Cancel</button>
                    <button class="btn btn-accent" id="confirmOk">OK</button>
                </div>
            </div>
        `;
        document.body.appendChild(modal);

        modal.querySelector('#confirmOk').addEventListener('click', () => {
            document.body.removeChild(modal);
            onConfirm && onConfirm();
        });
        modal.querySelector('#confirmCancel').addEventListener('click', () => {
            document.body.removeChild(modal);
            onCancel && onCancel();
        });
        modal.querySelector('.confirm-overlay').addEventListener('click', () => {
            document.body.removeChild(modal);
            onCancel && onCancel();
        });
    }

    // ========================================
    // Canvas Pan/Zoom & Interactions
    // ========================================

    function getMinScale() {
        if (!routeCanvas || !currentMapInfo) return 0.1;
        const rect = routeCanvas.getBoundingClientRect();
        const res = currentMapInfo.resolution || 0.05;
        const mapW = currentMapInfo.width * res;
        const mapH = currentMapInfo.height * res;
        
        // Allow zooming out beyond the initial fit while keeping the map usable.
        return Math.max(0.1, Math.min(rect.width / mapW, rect.height / mapH) * 0.1);
    }

    function setupCanvasInteraction() {
        if (!routeCanvas || canvasInteractionsBound) return;
        canvasInteractionsBound = true;

        routeCanvas.addEventListener('mousedown', (e) => {
            if (e.button !== 0) return;

            const rect = routeCanvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;

            const clickedNodeIdx = getNodeAtScreenPosition(mouseX, mouseY);

            if (clickedNodeIdx !== null) {
                if (currentTool === 'delete') {
                    const nodeName = currentRoute.nodes[clickedNodeIdx].name;
                    showConfirmModal(
                        'Delete Node',
                        `Delete node "${nodeName}"?`,
                        () => deleteNode(clickedNodeIdx),
                        () => {}
                    );
                } else if (currentTool === 'addEdge') {
                    if (edgeStartNodeIdx !== null && edgeStartNodeIdx !== clickedNodeIdx) {
                        const fromNode = currentRoute.nodes[edgeStartNodeIdx];
                        const toNode = currentRoute.nodes[clickedNodeIdx];
                        const edgeExists = currentRoute.edges.some(e =>
                            (e.from === fromNode.name && e.to === toNode.name) ||
                            (e.from === toNode.name && e.to === fromNode.name)
                        );
                        if (!edgeExists) {
                            currentRoute.edges.push({
                                from: fromNode.name,
                                to: toNode.name,
                                bidirectional: true
                            });
                            hasUnsavedChanges = true;
                        }
                        edgeStartNodeIdx = null;
                        pendingMousePos = null;
                    } else {
                        edgeStartNodeIdx = clickedNodeIdx;
                    }
                    selectNode(clickedNodeIdx);
                    renderCanvas();
                } else {
                    selectNode(clickedNodeIdx);
                }
            } else {
                isDragging = true;
                lastMouseX = e.clientX;
                lastMouseY = e.clientY;
                routeCanvas.style.cursor = 'grabbing';
            }
        });

        routeCanvas.addEventListener('mousemove', (e) => {
            const rect = routeCanvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;

            if (isDragging) {
                const dx = e.clientX - lastMouseX;
                const dy = e.clientY - lastMouseY;
                canvasOffsetX += dx;
                canvasOffsetY += dy;
                lastMouseX = e.clientX;
                lastMouseY = e.clientY;
                renderCanvas();
            } else if (currentTool === 'addEdge' && edgeStartNodeIdx !== null) {
                pendingMousePos = { x: mouseX, y: mouseY };
                renderCanvas();
            }
        });

        routeCanvas.addEventListener('mouseup', () => {
            if (isDragging) {
                isDragging = false;
                routeCanvas.style.cursor = 'default';
            }
        });

        routeCanvas.addEventListener('mouseleave', () => {
            isDragging = false;
            pendingMousePos = null;
            routeCanvas.style.cursor = 'default';
        });

        routeCanvas.addEventListener('click', (e) => {
            if (currentTool !== 'addNode' || !selectedRouteId) return;
            if (isDragging) return;

            const rect = routeCanvas.getBoundingClientRect();
            const clickX = e.clientX - rect.left;
            const clickY = e.clientY - rect.top;

            const clickedNodeIdx = getNodeAtScreenPosition(clickX, clickY);
            if (clickedNodeIdx === null) {
                addNodeAtPosition(clickX, clickY);
            }
        });

        // Mouse wheel zoom - Giới hạn minScale không zoom out xa hơn map
        routeCanvas.addEventListener('wheel', (e) => {
            e.preventDefault();
            const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
            const minScale = getMinScale();
            const maxScale = 15.0;

            canvasScale *= zoomFactor;
            canvasScale = Math.max(minScale, Math.min(maxScale, canvasScale));
            renderCanvas();
        }, { passive: false });
    }

    function screenToWorld(screenX, screenY) {
        const canvas = routeCanvas;
        if (!canvas) return { x: 0, y: 0 };
        const rect = canvas.getBoundingClientRect();
        const width = rect.width;
        const height = rect.height;

        const worldX = (screenX - width/2 - canvasOffsetX) / canvasScale;
        const worldY = -(screenY - height/2 - canvasOffsetY) / canvasScale;

        return { x: worldX, y: worldY };
    }

    function worldToScreen(worldX, worldY) {
        const canvas = routeCanvas;
        if (!canvas) return { x: 0, y: 0 };
        const rect = canvas.getBoundingClientRect();
        const width = rect.width;
        const height = rect.height;

        const screenX = width/2 + canvasOffsetX + worldX * canvasScale;
        const screenY = height/2 + canvasOffsetY - worldY * canvasScale;

        return { x: screenX, y: screenY };
    }

    function getNodeAtScreenPosition(screenX, screenY) {
        const world = screenToWorld(screenX, screenY);
        const nodeRadius = 20 / canvasScale;

        for (let i = currentRoute.nodes.length - 1; i >= 0; i--) {
            const node = currentRoute.nodes[i];
            const dx = world.x - node.position.x;
            const dy = world.y - node.position.y;
            const dist = Math.sqrt(dx * dx + dy * dy);
            if (dist <= nodeRadius) {
                return i;
            }
        }
        return null;
    }

    function addNodeAtPosition(screenX, screenY) {
        const world = screenToWorld(screenX, screenY);
        const nextId = currentRoute.nodes.length;

        const newNode = {
            name: `NODE_${nextId}`,
            position: { x: parseFloat(world.x.toFixed(1)), y: parseFloat(world.y.toFixed(1)), z: 0 },
            orientation: { yaw: 0 },
            type: 'transit',
            floor: 1,
            description: '',
            properties: {}
        };

        currentRoute.nodes.push(newNode);
        selectedNodeId = currentRoute.nodes.length - 1;
        hasUnsavedChanges = true;

        renderNodeTable();
        renderNodeEditor();
        renderCanvas();
        selectTool('select');
    }

    function resetCanvasView() {
        canvasScale = 1;
        canvasOffsetX = 0;
        canvasOffsetY = 0;
    }

    // Fit Map vừa khít Viewport
    function fitMapToView() {
        const canvas = routeCanvas;
        if (!canvas) return;

        const rect = canvas.getBoundingClientRect();
        const width = rect.width;
        const height = rect.height;

        if (!currentMapInfo) {
            canvasScale = 1;
            canvasOffsetX = 0;
            canvasOffsetY = 0;
            return;
        }

        const info = currentMapInfo;
        const res = info.resolution || 0.05;
        const mapW = info.width * res;
        const mapH = info.height * res;
        const originX = info.origin?.x || 0;
        const originY = info.origin?.y || 0;

        const scaleX = width / mapW;
        const scaleY = height / mapH;
        canvasScale = Math.min(scaleX, scaleY);

        // Tính toán tâm của map trong hệ tọa độ World
        const mapCenterX = originX + mapW / 2;
        const mapCenterY = originY + mapH / 2;

        // Căn tâm Map vào tâm Viewport
        canvasOffsetX = -mapCenterX * canvasScale;
        canvasOffsetY = mapCenterY * canvasScale;
    }

    // Render Map List, Route List, Node Table, Node Editor (Giữ nguyên)
    function renderMapList() {
        if (!mapListPanel) return;

        if (availableMaps.length === 0) {
            mapListPanel.innerHTML = `
                <div class="map-card">
                    <div class="map-card-icon">
                        <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                            <polygon points="1 6 1 22 8 18 16 22 23 18 23 2 16 6 8 2 1 6"/>
                            <line x1="8" y1="2" x2="8" y2="18"/>
                            <line x1="16" y1="6" x2="16" y2="22"/>
                        </svg>
                    </div>
                    <div class="map-card-info">
                        <div class="map-card-name">No maps available</div>
                    </div>
                </div>
            `;
            return;
        }

        mapListPanel.innerHTML = availableMaps.map(map => `
            <div class="map-card ${selectedMapId === map.id ? 'selected' : ''}" data-map-id="${map.id}">
                <div class="map-card-icon">
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                        <polygon points="1 6 1 22 8 18 16 22 23 18 23 2 16 6 8 2 1 6"/>
                        <line x1="8" y1="2" x2="8" y2="18"/>
                        <line x1="16" y1="6" x2="16" y2="22"/>
                    </svg>
                </div>
                <div class="map-card-info">
                    <div class="map-card-name">${map.name}</div>
                    <div class="map-card-meta">${map.created ? 'Created: ' + map.created : ''}</div>
                </div>
                <button class="map-card-delete" data-map-id="${map.id}" title="Delete Map">
                    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                        <polyline points="3 6 5 6 21 6"/>
                        <path d="M19 6v14a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V6m3 0V4a2 2 0 0 1 2-2h4a2 2 0 0 1 2 2v2"/>
                    </svg>
                </button>
            </div>
        `).join('');

        mapListPanel.querySelectorAll('.map-card').forEach(item => {
            item.addEventListener('click', (e) => {
                if (e.target.closest('.map-card-delete')) return;
                selectMap(item.dataset.mapId);
            });
        });

        mapListPanel.querySelectorAll('.map-card-delete').forEach(btn => {
            btn.addEventListener('click', (e) => {
                e.stopPropagation();
                deleteMap(btn.dataset.mapId);
            });
        });
    }

    function renderRouteList() {
        if (!routeListPanel) return;

        const route = availableRoutes[selectedMapId];

        if (!route) {
            routeListPanel.innerHTML = `
                <div class="route-empty">
                    <div class="route-empty-text">No route yet.</div>
                    <button class="btn btn-accent btn-block" id="btnCreateRoute">
                        + New Route
                    </button>
                </div>
            `;
            document.getElementById('btnCreateRoute')?.addEventListener('click', () => {
                if (hasUnsavedChanges) {
                    showConfirmModal(
                        'Unsaved Changes',
                        'You have unsaved changes. Create new route anyway?',
                        () => createNewRoute(),
                        () => {}
                    );
                } else {
                    createNewRoute();
                }
            });
            return;
        }

        routeListPanel.innerHTML = `
            <div class="route-info ${selectedRouteId === route.id ? 'selected' : ''}">
                <div class="route-name">${route.name}</div>
                <div class="route-meta">${route.nodeCount || 0} nodes | ${route.edgeCount || 0} edges</div>
            </div>
            <button class="btn btn-accent btn-block" id="btnNewRoute">
                + New Route
            </button>
        `;

        routeListPanel.querySelector('.route-info')?.addEventListener('click', () => {
            selectRoute(route.id);
        });

        document.getElementById('btnNewRoute')?.addEventListener('click', () => {
            if (hasUnsavedChanges) {
                showConfirmModal(
                    'Unsaved Changes',
                    'You have unsaved changes. Create new route anyway?',
                    () => createNewRoute(),
                    () => {}
                );
            } else {
                createNewRoute();
            }
        });
    }

    function renderNodeTable() {
        if (!routeNodeTable) return;

        const nodes = currentRoute.nodes;

        routeNodeTable.innerHTML = `
            <div class="node-table-header">
                <span>ID</span>
                <span>Name</span>
                <span>Position (x, y, yaw)</span>
                <span>Type</span>
            </div>
            ${nodes.length === 0 ? `
                <div class="node-table-empty">No nodes</div>
            ` : nodes.map((node, idx) => `
                <div class="node-table-row ${selectedNodeId === idx ? 'selected' : ''}"
                     data-node-idx="${idx}">
                    <span class="node-id">${idx}</span>
                    <span class="node-name">${node.name}</span>
                    <span class="node-pos">${node.position.x.toFixed(1)}, ${node.position.y.toFixed(1)},${(node.orientation.yaw * 180 / Math.PI).toFixed(1)}°</span>
                    <span class="node-type">${node.type.replace('_', ' ')}</span>
                </div>
            `).join('')}
        `;

        routeNodeTable.querySelectorAll('.node-table-row').forEach(row => {
            row.addEventListener('click', () => {
                selectNode(parseInt(row.dataset.nodeIdx));
            });
        });
    }

    function renderNodeEditor() {
        if (!nodeEditor) return;

        if (selectedNodeId === null) {
            nodeEditor.innerHTML = `
                <div class="node-editor-header">
                    <span>EDIT NODE</span>
                </div>
                <div class="node-editor-empty">
                    Select a node to edit
                </div>
            `;
            return;
        }

        const node = currentRoute.nodes[selectedNodeId];
        if (!node) return;

        nodeEditor.innerHTML = `
            <div class="node-editor-header">
                <span>EDIT NODE</span>
                <span class="node-editor-id">#${selectedNodeId}</span>
            </div>
            <div class="node-editor-body">
                <div class="edit-row">
                    <div class="edit-col-8">
                        <label>NAME</label>
                        <input type="text" class="edit-input" id="editNodeName" value="${node.name}" placeholder="Node name">
                    </div>
                    <div class="edit-col-4">
                        <label>ID</label>
                        <input type="number" class="edit-input" id="editNodeId" value="${selectedNodeId}" min="0">
                    </div>
                </div>
                <div class="edit-row">
                    <div class="edit-col">
                        <label>X</label>
                        <input type="number" class="edit-input edit-center" id="editNodeX" value="${node.position.x.toFixed(1)}" step="0.1">
                    </div>
                    <div class="edit-col">
                        <label>Y</label>
                        <input type="number" class="edit-input edit-center" id="editNodeY" value="${node.position.y.toFixed(1)}" step="0.1">
                    </div>
                    <div class="edit-col">
                        <label>YAW</label>
                        <input type="number" class="edit-input edit-center" id="editNodeYaw" value="${(node.orientation.yaw * 180 / Math.PI).toFixed(1)}" step="0.1">
                    </div>
                </div>
            </div>
        `;

        const applyNodeName = (value) => {
            node.name = value;
            node.type = inferNodeType(value);
            hasUnsavedChanges = true;
            renderNodeTable();
            renderCanvas();
        };

        document.getElementById('editNodeName')?.addEventListener('input', (e) => {
            applyNodeName(e.target.value);
        });

        document.getElementById('editNodeId')?.addEventListener('change', (e) => {
            const requestedId = Math.max(0, Math.min(
                currentRoute.nodes.length - 1,
                Math.round(Number(e.target.value)) || 0
            ));
            if (requestedId === selectedNodeId) return;

            const [movedNode] = currentRoute.nodes.splice(selectedNodeId, 1);
            currentRoute.nodes.splice(requestedId, 0, movedNode);
            selectedNodeId = requestedId;
            hasUnsavedChanges = true;
            renderNodeTable();
            renderNodeEditor();
            renderCanvas();
        });

        document.getElementById('editNodeX')?.addEventListener('change', (e) => {
            node.position.x = parseFloat(e.target.value) || 0;
            hasUnsavedChanges = true;
            renderNodeTable();
            renderCanvas();
        });

        document.getElementById('editNodeY')?.addEventListener('change', (e) => {
            node.position.y = parseFloat(e.target.value) || 0;
            hasUnsavedChanges = true;
            renderNodeTable();
            renderCanvas();
        });

        document.getElementById('editNodeYaw')?.addEventListener('change', (e) => {
            let val = parseFloat(e.target.value) || 0;
            val = Math.max(-180, Math.min(180, val));
            node.orientation.yaw = val * Math.PI / 180;
            hasUnsavedChanges = true;
            renderNodeTable();
            renderCanvas();
        });
    }

    function showEmptyState() {
        if (routeEmptyState) routeEmptyState.style.display = 'flex';
    }

    function hideEmptyState() {
        if (routeEmptyState) routeEmptyState.style.display = 'none';
    }

    // Actions
    async function selectMap(mapId) {
        selectedMapId = mapId;
        selectedRouteId = null;
        currentRoute = { graph_name: '', nodes: [], edges: [] };
        selectedNodeId = null;
        hasUnsavedChanges = false;
        resetCanvasView();

        showEmptyState();
        renderMapList();
        renderRouteList();
        renderNodeTable();
        renderNodeEditor();
        renderCanvas();

        await loadMapImage(mapId);
        await loadRoutesFromAPI(mapId);

        if (mapId) {
            hideEmptyState();
            if (currentMapInfo) {
                fitMapToView();
            }
        }

        renderMapList();
        renderRouteList();
        renderNodeTable();
        renderNodeEditor();
        renderCanvas();
    }

    function selectRoute(routeId) {
        selectedRouteId = routeId;
        selectedNodeId = null;
        hasUnsavedChanges = false;
        hideEmptyState();

        if (currentMapInfo) fitMapToView();

        renderRouteList();
        renderNodeTable();
        renderNodeEditor();
        renderCanvas();
    }

    function selectNode(nodeIdx) {
        selectedNodeId = nodeIdx;
        renderNodeTable();
        renderNodeEditor();
        renderCanvas();
    }

    async function deleteMap(mapId) {
        const map = availableMaps.find(m => m.id === mapId);
        showConfirmModal(
            'Delete Map',
            `Delete map "${map?.name || mapId}"? This cannot be undone.`,
            async () => {
                try {
                    const response = await fetch(`/api/maps/${encodeURIComponent(mapId)}`, {
                        method: 'DELETE'
                    });
                    const data = await response.json();
                    if (data.status === 'ok') {
                        availableMaps = availableMaps.filter(m => m.id !== mapId);
                        delete availableRoutes[mapId];

                        if (selectedMapId === mapId) {
                            selectedMapId = null;
                            selectedRouteId = null;
                            currentRoute = { graph_name: '', nodes: [], edges: [] };
                            showEmptyState();
                        }

                        renderMapList();
                        renderRouteList();
                        renderNodeTable();
                        renderNodeEditor();
                        renderCanvas();
                    }
                } catch (err) {
                    console.error('[MapRoute] Failed to delete map:', err);
                }
            },
            () => {}
        );
    }

    function deleteNode(nodeIdx) {
        if (nodeIdx === null || nodeIdx === undefined) return;

        const nodeName = currentRoute.nodes[nodeIdx]?.name;

        currentRoute.edges = currentRoute.edges.filter(e =>
            e.from !== nodeName && e.to !== nodeName
        );

        currentRoute.nodes.splice(nodeIdx, 1);

        if (selectedNodeId === nodeIdx) {
            selectedNodeId = null;
        } else if (selectedNodeId > nodeIdx) {
            selectedNodeId--;
        }

        hasUnsavedChanges = true;
        renderNodeTable();
        renderNodeEditor();
        renderCanvas();
    }

    function createNewRoute() {
        if (!selectedMapId) {
            alert('Please select a map first');
            return;
        }

        const routeId = `${selectedMapId}_route`;
        const routeName = `${selectedMapId} Route`;

        currentRoute = {
            graph_name: routeName,
            nodes: [{
                name: 'NODE_HOME',
                position: { x: 0, y: 0, z: 0 },
                orientation: { yaw: 0 },
                type: 'home',
                floor: 1,
                description: 'Home position',
                properties: { is_home: true, is_parking: true }
            }],
            edges: []
        };

        availableRoutes[selectedMapId] = {
            id: routeId,
            name: routeName,
            nodeCount: 1,
            edgeCount: 0
        };

        selectedRouteId = routeId;
        selectedNodeId = 0;
        hasUnsavedChanges = true;

        hideEmptyState();
        if (currentMapInfo) fitMapToView();
        renderRouteList();
        renderNodeTable();
        renderNodeEditor();
        renderCanvas();
    }

    function selectTool(tool) {
        currentTool = tool;
        edgeStartNodeIdx = null;
        pendingMousePos = null;

        if (routeCanvas) {
            if (tool === 'addNode') routeCanvas.style.cursor = 'crosshair';
            else if (tool === 'addEdge') routeCanvas.style.cursor = 'pointer';
            else if (tool === 'delete') routeCanvas.style.cursor = 'not-allowed';
            else routeCanvas.style.cursor = 'default';
        }

        document.querySelectorAll('.tool-btn').forEach(btn => {
            btn.classList.toggle('active', btn.dataset.tool === tool);
        });

        renderCanvas();
    }

    async function loadMapImage(mapId) {
        try {
            const response = await fetch(`/api/map/image/${encodeURIComponent(mapId)}`);
            if (!response.ok) {
                currentMapImage = null;
                currentMapInfo = null;
                return;
            }

            const data = await response.json();
            if (data.status === 'ok') {
                currentMapInfo = data.info || { width: 100, height: 100, resolution: 0.05, origin: { x: 0, y: 0 } };
                currentMapImage = data.image_data;
            } else {
                currentMapImage = null;
                currentMapInfo = null;
            }
        } catch (err) {
            console.error('[MapRoute] Failed to load map image:', err);
            currentMapImage = null;
            currentMapInfo = null;
        }
    }

    // ========================================
    // Canvas Rendering (Sửa chính xác tọa độ vẽ Map)
    // ========================================

    function renderCanvas() {
        const canvas = routeCanvas;
        if (!canvas) return;

        const ctx = canvas.getContext('2d');
        const rect = canvas.getBoundingClientRect();
        const dpr = window.devicePixelRatio || 1;

        if (rect.width === 0 || rect.height === 0) return;

        const pixelWidth = Math.round(rect.width * dpr);
        const pixelHeight = Math.round(rect.height * dpr);
        if (canvas.width !== pixelWidth || canvas.height !== pixelHeight) {
            canvas.width = pixelWidth;
            canvas.height = pixelHeight;
        }
        canvas.style.width = rect.width + 'px';
        canvas.style.height = rect.height + 'px';
        ctx.setTransform(dpr, 0, 0, dpr, 0, 0);

        const width = rect.width;
        const height = rect.height;

        // Keep the Map & Route viewport consistent with the live map background.
        ctx.fillStyle = '#808080';
        ctx.fillRect(0, 0, width, height);

        if (currentMapImage && currentMapInfo) {
            const info = currentMapInfo;
            const res = info.resolution || 0.05;
            const originX = info.origin?.x || 0;
            const originY = info.origin?.y || 0;
            const mapW = info.width * res;
            const mapH = info.height * res;

            // Decode Base64 Image
            const decoded = atob(currentMapImage);
            const bytes = new Uint8Array(decoded.length);
            for (let i = 0; i < decoded.length; i++) bytes[i] = decoded.charCodeAt(i);

            const tmp = document.createElement('canvas');
            tmp.width = info.width;
            tmp.height = info.height;
            const tmpCtx = tmp.getContext('2d');
            const imgData = tmpCtx.createImageData(info.width, info.height);
            const pix = imgData.data;

            const w = info.width;
            const h = info.height;

            // Preserve the PGM pixels exactly: PGM row 0 is the visible top row.
            for (let y = 0; y < h; y++) {
                for (let x = 0; x < w; x++) {
                    const srcIdx = y * w + x;
                    const destIdx = srcIdx * 4;
                    const val = bytes[srcIdx];
                    pix[destIdx] = val;
                    pix[destIdx + 1] = val;
                    pix[destIdx + 2] = val;
                    pix[destIdx + 3] = 255;
                }
            }
            tmpCtx.putImageData(imgData, 0, 0);

            // 3. Render ra Viewport
            ctx.save();
            ctx.translate(width / 2 + canvasOffsetX, height / 2 + canvasOffsetY);
            
            // The PGM itself remains top-to-bottom; this placement preserves
            // the existing ROS-world alignment for route nodes and edges.
            ctx.scale(canvasScale, canvasScale);

            ctx.imageSmoothingEnabled = false;
        
            // Vẽ đúng tọa độ World (Lưu ý: Trong hệ Canvas Top-Down, Y_screen hướng xuống nên dùng -originY)
            ctx.drawImage(tmp, originX, -originY - mapH, mapW, mapH);

            ctx.restore();
        }

        // 3. Vẽ Route Nodes & Edges
        if (currentRoute.nodes.length > 0) {
            currentRoute.edges.forEach((edge) => {
                const fromNode = currentRoute.nodes.find(n => n.name === edge.from);
                const toNode = currentRoute.nodes.find(n => n.name === edge.to);
                if (!fromNode || !toNode) return;

                const from = worldToScreen(fromNode.position.x, fromNode.position.y);
                const to = worldToScreen(toNode.position.x, toNode.position.y);

                ctx.strokeStyle = '#34d399';
                ctx.lineWidth = 3;
                ctx.beginPath();
                ctx.moveTo(from.x, from.y);
                ctx.lineTo(to.x, to.y);
                ctx.stroke();
            });

            currentRoute.nodes.forEach((node, idx) => {
                const pos = worldToScreen(node.position.x, node.position.y);

                const colors = {
                    home: '#f97316',
                    pickup_approach: '#3b82f6',
                    dropoff_approach: '#ef4444',
                    transit: '#22c55e',
                    charging: '#9ca3af'
                };
                const color = colors[node.type] || '#9ca3af';
                const radius = node.type === 'home' ? 12 : 8;

                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.arc(pos.x, pos.y, radius, 0, Math.PI * 2);
                ctx.fill();

                if (selectedNodeId === idx) {
                    ctx.strokeStyle = '#fff';
                    ctx.lineWidth = 3;
                    ctx.beginPath();
                    ctx.arc(pos.x, pos.y, radius + 6, 0, Math.PI * 2);
                    ctx.stroke();
                }

                const yaw = node.orientation.yaw;
                const arrowLen = 20;
                const arrowX = pos.x + Math.cos(-yaw) * arrowLen;
                const arrowY = pos.y + Math.sin(-yaw) * arrowLen;

                ctx.strokeStyle = color;
                ctx.lineWidth = 3;
                ctx.lineCap = 'round';
                ctx.beginPath();
                ctx.moveTo(pos.x, pos.y);
                ctx.lineTo(arrowX, arrowY);
                ctx.stroke();

                const arrowAngle = Math.atan2(arrowY - pos.y, arrowX - pos.x);
                ctx.fillStyle = color;
                ctx.beginPath();
                ctx.moveTo(arrowX, arrowY);
                ctx.lineTo(arrowX - 8 * Math.cos(arrowAngle - 0.4), arrowY - 8 * Math.sin(arrowAngle - 0.4));
                ctx.lineTo(arrowX - 8 * Math.cos(arrowAngle + 0.4), arrowY - 8 * Math.sin(arrowAngle + 0.4));
                ctx.closePath();
                ctx.fill();

                ctx.fillStyle = color;
                ctx.font = 'bold 12px sans-serif';
                ctx.textAlign = 'center';
                ctx.fillText(node.name, pos.x, pos.y - radius - 8);

                ctx.font = '10px monospace';
                ctx.fillStyle = '#888';
                ctx.fillText(`#${idx}`, pos.x, pos.y + radius + 14);
            });
        }

        const statsEl = document.getElementById('routeStats');
        if (statsEl) {
            statsEl.textContent = `${currentRoute.nodes.length} | ${currentRoute.edges.length}`;
        }
    }

    // API Functions
    async function loadMapsFromAPI() {
        try {
            const response = await fetch('/api/maps');
            const data = await response.json();
            if (data.status === 'ok' && data.maps) {
                availableMaps = data.maps;
                return true;
            }
        } catch (err) {
            console.error('[MapRoute] Failed to load maps:', err);
        }
        return false;
    }

    async function loadRoutesFromAPI(mapId) {
        try {
            const response = await fetch(`/api/routes?map_id=${encodeURIComponent(mapId)}`);
            const data = await response.json();
            if (data.status === 'ok' && data.routes && data.routes.length > 0) {
                const route = data.routes[0];
                availableRoutes[mapId] = route;
                currentRoute = route;
                selectedRouteId = route.id;
                selectedNodeId = route.nodes.length > 0 ? 0 : null;
                return true;
            } else {
                availableRoutes[mapId] = null;
            }
        } catch (err) {
            console.error('[MapRoute] Failed to load routes:', err);
            availableRoutes[mapId] = null;
        }
        return false;
    }

    async function saveRoute() {
        if (!selectedMapId) {
            alert('Please select a map first');
            return;
        }

        if (!selectedRouteId) {
            alert('Please create a route first');
            return;
        }

        if (hasUnsavedChanges) {
            availableRoutes[selectedMapId] = {
                ...availableRoutes[selectedMapId],
                graph_name: currentRoute.graph_name,
                nodes: currentRoute.nodes,
                edges: currentRoute.edges,
                nodeCount: currentRoute.nodes.length,
                edgeCount: currentRoute.edges.length
            };

            try {
                const response = await fetch('/api/routes', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({
                        map_id: selectedMapId,
                        route: currentRoute
                    })
                });
                const data = await response.json();
                if (data.status === 'ok') {
                    hasUnsavedChanges = false;
                    alert('Route saved successfully!');
                    renderRouteList();
                }
            } catch (err) {
                console.error('[MapRoute] Failed to save route:', err);
                alert('Failed to save route');
            }
        } else {
            alert('Route is already saved!');
        }
    }

    // Initialize
    async function initMapRoutePage() {
        console.log('[MapRoute] Initializing...');

        await loadMapsFromAPI();
        showEmptyState();

        renderMapList();
        renderRouteList();
        renderNodeTable();
        renderNodeEditor();
        renderCanvas();
        setupCanvasInteraction();

        if (!mapRouteControlsBound) {
            mapRouteControlsBound = true;
            document.querySelectorAll('.tool-btn').forEach(btn => {
                btn.addEventListener('click', () => selectTool(btn.dataset.tool));
            });

            document.getElementById('btnSaveRoute')?.addEventListener('click', saveRoute);

            window.addEventListener('resize', () => {
                fitMapToView();
                renderCanvas();
            });
        }

        console.log('[MapRoute] Initialized!');
    }

    function resetMapRouteState() {
        selectedMapId = null;
        selectedRouteId = null;
        selectedNodeId = null;
        currentRoute = { graph_name: '', nodes: [], edges: [] };
        hasUnsavedChanges = false;
        currentMapImage = null;
        currentMapInfo = null;
        resetCanvasView();

        showEmptyState();
        renderMapList();
        renderRouteList();
        renderNodeTable();
        renderNodeEditor();
        renderCanvas();
    }

    // Export
    window.initMapRoutePage = initMapRoutePage;
    window.resetMapRoutePage = resetMapRouteState;
    window.fitMapToView = fitMapToView;
    window.resetCanvasView = resetCanvasView;
    window.selectTool = selectTool;

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initMapRoutePage);
    } else {
        initMapRoutePage();
    }

})();
