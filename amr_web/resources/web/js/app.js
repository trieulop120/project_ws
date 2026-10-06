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

    // Show SLAM UI if navigating to SLAM
    if (p === 'slam' && window.isSlamActive) {
        if (typeof showSlamActiveUI === 'function') showSlamActiveUI();
    }

    // Trigger SLAM page init
    if (p === 'slam' && window.onNavigateToSlam) {
        window.onNavigateToSlam();
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

    // Navigation events
    navItems.forEach(i => i.addEventListener('click', e => {
        e.preventDefault();
        navigateToPage(i.dataset.page);
    }));

    // Canvas drag events
    let isDragging = false;
    let dragStart = { x: 0, y: 0 };
    let transformStart = { x: 0, y: 0, scale: 1 };

    document.addEventListener('mousedown', e => {
        const vp = document.querySelector('.page.active .map-viewport, .page.active .slam-map-viewport');
        if (!vp || !vp.contains(e.target)) return;
        if (e.target.closest('.map-toolbar') || e.target.closest('.map-empty')) return;

        isDragging = true;
        dragStart = { x: e.clientX, y: e.clientY };
        transformStart = { ...(window.viewTransform || { x: 0, y: 0, scale: 1 }) };
        if (canvasEl) canvasEl.style.cursor = 'grabbing';
    });

    document.addEventListener('mousemove', e => {
        if (!isDragging || !window.viewTransform) return;
        window.viewTransform.x = transformStart.x + (e.clientX - dragStart.x);
        window.viewTransform.y = transformStart.y + (e.clientY - dragStart.y);
        if (typeof window.renderMap === 'function') window.renderMap();
    });

    document.addEventListener('mouseup', () => {
        isDragging = false;
        if (canvasEl) canvasEl.style.cursor = 'grab';
    });

    if (canvasEl) {
        canvasEl.addEventListener('wheel', e => {
            e.preventDefault();
            if (!window.viewTransform) return;
            if (e.deltaY > 0) { window.viewTransform.scale *= 0.9; } 
            else { window.viewTransform.scale *= 1.1; }
            window.viewTransform.scale = Math.max(0.1, Math.min(10, window.viewTransform.scale));
            if (typeof window.renderMap === 'function') window.renderMap();
        });
    }

    // Toolbar buttons (active page only)
    document.addEventListener('click', e => {
        if (!window.viewTransform) return;
        if (e.target.closest('#btnZoomIn') || e.target.closest('#slamBtnZoomIn')) {
            window.viewTransform.scale *= 1.2;
            if (typeof window.renderMap === 'function') window.renderMap();
        } else if (e.target.closest('#btnZoomOut') || e.target.closest('#slamBtnZoomOut')) {
            window.viewTransform.scale *= 0.8;
            if (typeof window.renderMap === 'function') window.renderMap();
        } else if (e.target.closest('#btnRecenter') || e.target.closest('#slamBtnRecenter')) {
            if (window.isSlamActive && window.mapRenderer) {
                window.mapRenderer.recenter();
            } else if (typeof window.fitMapToView === 'function') {
                window.fitMapToView();
            }
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

window.addEventListener('resize', () => {
    if (window.mapRenderer) {
        window.mapRenderer.moveToViewport(document.querySelector('.page.active')?.id === 'page-slam' ? 'slam' : 'overview');
    } else if (typeof window.resizeCanvas === 'function') {
        window.resizeCanvas();
    }
});