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

    // Canvas drag events - Skip if navigation tool is active
    let isDragging = false;
    let dragStart = { x: 0, y: 0 };
    let transformStart = { x: 0, y: 0, scale: 1 };

    // Get correct viewTransform based on current viewport
    function getCurrentViewTransform() {
        if (window.currentViewport === 'slam') {
            return window.slamViewTransform;
        } else if (window.currentViewport === 'navigation') {
            return window.navViewTransform;
        }
        return window.overviewViewTransform;
    }

    document.addEventListener('mousedown', e => {
        // Skip pan if navigation tool is active
        if (typeof window.isCanvasToolActive !== 'undefined' && window.isCanvasToolActive) {
            return;
        }

        const vp = document.querySelector('.page.active .map-viewport, .page.active .slam-map-viewport, .page.active #navMapViewport');
        if (!vp || !vp.contains(e.target)) return;
        if (e.target.closest('.map-toolbar') || e.target.closest('.map-empty')) return;

        isDragging = true;
        dragStart = { x: e.clientX, y: e.clientY };
        const vt = getCurrentViewTransform();
        transformStart = { ...vt };
        if (canvasEl) canvasEl.style.cursor = 'grabbing';
    });

    document.addEventListener('mousemove', e => {
        if (!isDragging) return;
        const vt = getCurrentViewTransform();
        vt.x = transformStart.x + (e.clientX - dragStart.x);
        vt.y = transformStart.y + (e.clientY - dragStart.y);
        if (typeof window.renderMap === 'function') window.renderMap();
    });

    document.addEventListener('mouseup', () => {
        isDragging = false;
        if (canvasEl) canvasEl.style.cursor = 'grab';
    });

    if (canvasEl) {
        canvasEl.addEventListener('wheel', e => {
            e.preventDefault();
            const vt = getCurrentViewTransform();
            if (e.deltaY > 0) { vt.scale *= 0.9; }
            else { vt.scale *= 1.1; }
            vt.scale = Math.max(0.1, Math.min(10, vt.scale));
            if (typeof window.renderMap === 'function') window.renderMap();
        });
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
