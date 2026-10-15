// ========================================
// Clock Initialization
// ========================================
function initClock() {
    const updateClock = () => {
        const now = new Date();
        const timeEl = document.getElementById('clockTime');
        const dateEl = document.getElementById('clockDate');
        if (timeEl) timeEl.textContent = now.toLocaleTimeString('en-GB', { hour12: false });
        if (dateEl) dateEl.textContent = now.toLocaleDateString('en-GB');
    };
    updateClock();
    setInterval(updateClock, 1000);
}

// ========================================
// Sidebar Toggle
// ========================================
function initSidebarToggle() {
    const toggleBtn = document.getElementById('sidebarToggle');
    const sidebar = document.querySelector('.sidebar');

    if (!toggleBtn || !sidebar) return;

    const handleToggle = (e) => {
        e.preventDefault();
        e.stopPropagation();
        sidebar.classList.toggle('show');
    };

    toggleBtn.addEventListener('click', handleToggle);
    toggleBtn.addEventListener('touchstart', handleToggle, { passive: false });

    const handleCloseOutside = (e) => {
        if (sidebar.classList.contains('show') &&
            !sidebar.contains(e.target) &&
            !toggleBtn.contains(e.target)) {
            sidebar.classList.remove('show');
        }
    };
    document.addEventListener('click', handleCloseOutside);
    document.addEventListener('touchstart', handleCloseOutside);

    document.addEventListener('keydown', (e) => {
        if (e.key === 'Escape' && sidebar.classList.contains('show')) {
            sidebar.classList.remove('show');
        }
    });
}

// ========================================
// Navigation Tab Switcher
// ========================================
const navItems = document.querySelectorAll('.nav-item');
const pages = document.querySelectorAll('.page');

function navigateToPage(p) {
    navItems.forEach(i => i.classList.toggle('active', i.dataset.page === p));
    pages.forEach(i => i.classList.toggle('active', i.id === 'page-' + p));
    const titleEl = document.getElementById('pageTitle');
    if (titleEl) titleEl.textContent = p.charAt(0).toUpperCase() + p.slice(1);

    // Move canvas to correct viewport
    if (window.mapRenderer) {
        window.mapRenderer.moveToViewport(p);
    }

    // Fix: ALWAYS hide SLAM layout when not on SLAM page (even if SLAM is active)
    if (p !== 'slam') {
        const slamLayout = document.getElementById('slamLayout');
        const slamWelcome = document.getElementById('slamWelcome');
        // Always hide slamLayout regardless of isSlamActive
        if (slamLayout) {
            slamLayout.style.display = 'none';
            slamLayout.classList.remove('active');
        }
        // Show welcome if SLAM is inactive, hide if active
        if (slamWelcome) {
            slamWelcome.style.display = window.isSlamActive ? 'none' : 'flex';
        }
    } else {
        // On SLAM page - show correct UI based on state
        const slamLayout = document.getElementById('slamLayout');
        const slamWelcome = document.getElementById('slamWelcome');
        if (window.isSlamActive) {
            if (slamLayout) {
                slamLayout.style.display = 'grid';
                slamLayout.classList.add('active');
            }
            if (slamWelcome) slamWelcome.style.display = 'none';
        } else {
            if (slamLayout) {
                slamLayout.style.display = 'none';
                slamLayout.classList.remove('active');
            }
            if (slamWelcome) slamWelcome.style.display = 'flex';
        }
    }

    // Reset Map & Route page when navigating away
    if (p !== 'map' && typeof window.resetMapRoutePage === 'function') {
        window.resetMapRoutePage();
    }

    // Show SLAM UI if navigating to SLAM
    if (p === 'slam' && window.isSlamActive) {
        if (typeof showSlamActiveUI === 'function') showSlamActiveUI();
    }

    // Show Navigation UI if navigating to Navigation
    if (p === 'navigation' && window.isNavigationActive) {
        if (typeof showNavActiveUI === 'function') showNavActiveUI();
    }

    // Trigger SLAM page init
    if (p === 'slam' && window.onNavigateToSlam) {
        window.onNavigateToSlam();
    }

    // Trigger Navigation page init
    if (p === 'navigation' && window.onNavigateToNav) {
        window.onNavigateToNav();
    }

    // Trigger Map Route page init
    if (p === 'map' && typeof window.initMapRoutePage === 'function') {
        window.initMapRoutePage();
    }
}

function showSlamActiveUI() {
    const slamWelcome = document.getElementById('slamWelcome');
    const slamLayout = document.getElementById('slamLayout');

    if (slamWelcome) slamWelcome.style.display = 'none';
    if (slamLayout) {
        slamLayout.style.display = 'grid';
        slamLayout.style.animation = 'fadeIn 0.4s ease';
    }
}

window.startSlamUI = function() {
    showSlamActiveUI();
};

// ========================================
// Main Initialization on Page Load
// ========================================
document.addEventListener('DOMContentLoaded', () => {
    initClock();
    initSidebarToggle();
    if (typeof window.initCanvas === 'function') window.initCanvas();

    const canvasEl = document.getElementById('mapCanvas');
    if (canvasEl) canvasEl.style.cursor = 'grab';

    // Navigation events (Tự động đóng Sidebar khi chọn Tab trên mobile)
    navItems.forEach(i => i.addEventListener('click', e => {
        e.preventDefault();
        navigateToPage(i.dataset.page);
        
        // Đóng sidebar drawer trên thiết bị di động
        const sidebar = document.querySelector('.sidebar');
        if (window.innerWidth <= 768 && sidebar) {
            sidebar.classList.remove('show');
        }
    }));

    // Canvas pan / pinch-zoom - Pointer Events (mouse + touch + pen)
    // Skip if a navigation tool (Set Pose / Set Goal) is active.
    const activePointers = new Map();   // pointerId -> {x, y} (client coords)
    let panPointerId = null;
    let dragStart = { x: 0, y: 0 };
    let transformStart = { x: 0, y: 0, scale: 1 };
    let pinchStart = null;              // { dist, mid:{x,y} (viewport-relative), vt:{x,y,scale} }

    // Get correct viewTransform based on current viewport
    function getCurrentViewTransform() {
        if (window.currentViewport === 'slam') {
            return window.slamViewTransform;
        } else if (window.currentViewport === 'navigation') {
            return window.navViewTransform;
        }
        return window.overviewViewTransform;
    }

    function getActiveMapViewport() {
        return document.querySelector('.page.active .map-viewport, .page.active .slam-map-viewport, .page.active #navMapViewport');
    }

    function startPan(pointerId) {
        const p = activePointers.get(pointerId);
        if (!p) return;
        panPointerId = pointerId;
        dragStart = { x: p.x, y: p.y };
        transformStart = { ...getCurrentViewTransform() };
    }

    function startPinch() {
        const vp = getActiveMapViewport();
        if (!vp) return;
        const [a, b] = [...activePointers.values()];
        const rect = vp.getBoundingClientRect();
        panPointerId = null;
        pinchStart = {
            dist: Math.hypot(a.x - b.x, a.y - b.y) || 1,
            mid: { x: (a.x + b.x) / 2 - rect.left, y: (a.y + b.y) / 2 - rect.top },
            vt: { ...getCurrentViewTransform() }
        };
    }

    document.addEventListener('pointerdown', e => {
        // Skip pan if navigation tool is active
        if (typeof window.isCanvasToolActive !== 'undefined' && window.isCanvasToolActive) return;
        if (e.pointerType === 'mouse' && e.button !== 0) return;

        const vp = getActiveMapViewport();
        if (!vp || !vp.contains(e.target)) return;
        if (e.target.closest('.map-toolbar') || e.target.closest('.map-empty') || e.target.closest('button')) return;

        activePointers.set(e.pointerId, { x: e.clientX, y: e.clientY });

        if (activePointers.size === 1) {
            startPan(e.pointerId);
            if (canvasEl) canvasEl.style.cursor = 'grabbing';
        } else if (activePointers.size === 2) {
            startPinch();
        }
    });

    document.addEventListener('pointermove', e => {
        if (!activePointers.has(e.pointerId)) return;
        activePointers.set(e.pointerId, { x: e.clientX, y: e.clientY });

        const vt = getCurrentViewTransform();

        if (activePointers.size >= 2 && pinchStart) {
            const vp = getActiveMapViewport();
            if (!vp) return;
            const [a, b] = [...activePointers.values()];
            const rect = vp.getBoundingClientRect();
            const dist = Math.hypot(a.x - b.x, a.y - b.y) || 1;
            const midX = (a.x + b.x) / 2 - rect.left;
            const midY = (a.y + b.y) / 2 - rect.top;

            const newScale = Math.max(0.1, Math.min(10, pinchStart.vt.scale * dist / pinchStart.dist));
            // keep the map point that was under the initial midpoint under the current midpoint
            const cellX = (pinchStart.mid.x - pinchStart.vt.x) / pinchStart.vt.scale;
            const cellY = (pinchStart.mid.y - pinchStart.vt.y) / pinchStart.vt.scale;
            vt.scale = newScale;
            vt.x = midX - cellX * newScale;
            vt.y = midY - cellY * newScale;
        } else if (e.pointerId === panPointerId) {
            vt.x = transformStart.x + (e.clientX - dragStart.x);
            vt.y = transformStart.y + (e.clientY - dragStart.y);
        } else {
            return;
        }
        if (typeof window.renderMap === 'function') window.renderMap();
    });

    function endPointer(e) {
        if (!activePointers.has(e.pointerId)) return;
        activePointers.delete(e.pointerId);

        if (activePointers.size < 2) pinchStart = null;

        if (activePointers.size === 1) {
            // one finger left after a pinch: continue as pan from where it is now
            startPan([...activePointers.keys()][0]);
        } else if (activePointers.size === 0) {
            panPointerId = null;
            if (canvasEl) canvasEl.style.cursor = 'grab';
        }
    }
    document.addEventListener('pointerup', endPointer);
    document.addEventListener('pointercancel', endPointer);

    if (canvasEl) {
        canvasEl.addEventListener('wheel', e => {
            e.preventDefault();
            const vt = getCurrentViewTransform();
            if (e.deltaY > 0) { vt.scale *= 0.9; }
            else { vt.scale *= 1.1; }
            vt.scale = Math.max(0.1, Math.min(10, vt.scale));
            if (typeof window.renderMap === 'function') window.renderMap();
        }, { passive: false });
    }

    // Toolbar buttons (active page only)
    document.addEventListener('click', e => {
        const vt = getCurrentViewTransform();
        if (e.target.closest('#btnZoomIn') || e.target.closest('#slamBtnZoomIn')) {
            vt.scale *= 1.2;
            if (typeof window.renderMap === 'function') window.renderMap();
        } else if (e.target.closest('#btnZoomOut') || e.target.closest('#slamBtnZoomOut')) {
            vt.scale *= 0.8;
            if (typeof window.renderMap === 'function') window.renderMap();
        } else if (e.target.closest('#btnRecenter')) {
            // Overview recenter. (#slamBtnRecenter is handled in slam.js, #toolCenterMap in navigation.js)
            // NOTE: window.fitMapToView belongs to map_route.js (route canvas), so it must not be used here.
            if (window.mapRenderer) window.mapRenderer.recenter();
        }
    });

    // Alarm Filter Tabs Events
    document.querySelectorAll('.alarm-tab').forEach(tab => {
        tab.addEventListener('click', () => {
            document.querySelectorAll('.alarm-tab').forEach(t => t.classList.remove('active'));
            tab.classList.add('active');
            if (typeof alarmFilter !== 'undefined') {
                alarmFilter = tab.dataset.filter;
                if (typeof renderAlarmList === 'function') renderAlarmList();
            }
        });
    });

    document.getElementById('viewAllLogsBtn')?.addEventListener('click', () => navigateToPage('system'));

    // Connect WebSocket
    if (typeof connect === 'function') connect();
});

// On phones 'resize' fires whenever the URL bar collapses/expands (page scroll), on rotation and when
// the keyboard opens. It must NOT hard-code slam/overview, otherwise the canvas is moved out of the
// Navigation viewport (grey map, robot missing, Center Robot acting on the wrong viewport).
window.addEventListener('resize', () => {
    if (window.mapRenderer) {
        const pageId = document.querySelector('.page.active')?.id || 'page-overview';
        window.mapRenderer.moveToViewport(pageId.replace('page-', ''));
    } else if (typeof window.resizeCanvas === 'function') {
        window.resizeCanvas();
    }
});

// Commit values from single-line inputs consistently when the user presses Enter.
document.addEventListener('keydown', (e) => {
    if (e.key !== 'Enter' || e.isComposing) return;
    const target = e.target;
    if (!(target instanceof HTMLInputElement) || target.type === 'button') return;

    e.preventDefault();
    target.dispatchEvent(new Event('change', { bubbles: true }));
    target.blur();
});

// ========================================
// Global Toast Notification System
// ========================================
window.showToast = function(message, type = 'success') {
    let container = document.querySelector('.toast-container');
    if (!container) {
        container = document.createElement('div');
        container.className = 'toast-container';
        document.body.appendChild(container);
    }

    const toast = document.createElement('div');
    toast.className = `toast toast-${type}`;
    toast.innerHTML = `
        <svg class="toast-icon" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            ${type === 'success' 
                ? '<path d="M22 11.08V12a10 10 0 1 1-5.93-9.14"/><polyline points="22 4 12 14.01 9 11.01"/>' 
                : '<circle cx="12" cy="12" r="10"/><line x1="15" y1="9" x2="9" y2="15"/><line x1="9" y1="9" x2="15" y2="15"/>'}
        </svg>
        <span class="toast-message">${message}</span>
    `;

    container.appendChild(toast);

    setTimeout(() => {
        toast.style.animation = 'toastOut 0.3s forwards';
        setTimeout(() => toast.remove(), 300);
    }, 3000);
};