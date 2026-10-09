// Navigation Page Logic
(function() {
    'use strict';

    // ========================================
    // Navigation State
    // ========================================
    window.isNavigationActive = false;
    window.selectedNavMap = null;
    window.selectedNavRoute = null;

    // Available maps & routes (sẽ load từ API)
    let availableMaps = [];
    let availableRoutes = [];

    // DOM Elements
    const navSetupModal = document.getElementById('navSetupModal');
    const btnStartNav = document.getElementById('btnStartNav');
    const btnStopNav = document.getElementById('btnStopNav');
    const navModalClose = document.getElementById('navModalClose');
    const navModalCancel = document.getElementById('navModalCancel');
    const navModalConfirm = document.getElementById('navModalConfirm');
    const navMapSelect = document.getElementById('navMapSelect');
    const navRouteSelect = document.getElementById('navRouteSelect');

    // ========================================
    // API Functions
    // ========================================

    async function loadMapsFromAPI() {
        try {
            console.log('[Navigation] Fetching maps from API...');
            const response = await fetch('/api/maps');
            const data = await response.json();

            if (data.status === 'ok' && data.maps) {
                availableMaps = data.maps;
                console.log('[Navigation] Loaded maps:', availableMaps);
                return true;
            }
        } catch (err) {
            console.error('[Navigation] Failed to load maps:', err);
        }
        return false;
    }

    async function loadRoutesFromAPI() {
        try {
            console.log('[Navigation] Fetching routes from API...');
            const response = await fetch('/api/routes');
            const data = await response.json();

            if (data.status === 'ok' && data.routes) {
                availableRoutes = data.routes;
                console.log('[Navigation] Loaded routes:', availableRoutes);
                return true;
            }
        } catch (err) {
            console.error('[Navigation] Failed to load routes:', err);
        }
        return false;
    }

    // ========================================
    // Modal Functions
    // ========================================

    async function showNavSetupModal() {
        console.log('[Navigation] Showing setup modal');

        // Reset selections
        window.selectedNavMap = null;
        window.selectedNavRoute = null;
        if (navMapSelect) navMapSelect.value = '';
        if (navRouteSelect) navRouteSelect.value = '';
        if (navModalConfirm) navModalConfirm.disabled = true;

        // Load maps & routes từ API
        await Promise.all([
            loadMapsFromAPI(),
            loadRoutesFromAPI()
        ]);

        // Populate dropdowns
        populateMapDropdown();
        populateRouteDropdown();

        // Show modal
        if (navSetupModal) {
            navSetupModal.classList.add('active');
            console.log('[Navigation] Modal opened');
        }
    }

    function hideNavSetupModal() {
        console.log('[Navigation] Hiding setup modal');
        if (navSetupModal) {
            navSetupModal.classList.remove('active');
        }
    }

    function populateMapDropdown() {
        if (!navMapSelect) return;
        navMapSelect.innerHTML = '<option value="">-- Select a map --</option>';

        availableMaps.forEach(map => {
            const option = document.createElement('option');
            option.value = map.id;
            option.textContent = map.name;
            option.dataset.path = map.path;
            navMapSelect.appendChild(option);
        });

        console.log('[Navigation] Populated map dropdown with', availableMaps.length, 'maps');
    }

    function populateRouteDropdown() {
        if (!navRouteSelect) return;
        navRouteSelect.innerHTML = '<option value="">-- Select a route --</option>';

        availableRoutes.forEach(route => {
            const option = document.createElement('option');
            option.value = route.id;
            option.textContent = route.name;
            option.dataset.path = route.path;
            navRouteSelect.appendChild(option);
        });

        console.log('[Navigation] Populated route dropdown with', availableRoutes.length, 'routes');
    }

    function updateConfirmButtonState() {
        const mapSelected = !!navMapSelect?.value;
        const routeSelected = !!navRouteSelect?.value;

        window.selectedNavMap = mapSelected ? navMapSelect.value : null;
        window.selectedNavRoute = routeSelected ? navRouteSelect.value : null;

        if (navModalConfirm) {
            navModalConfirm.disabled = !(mapSelected && routeSelected);
        }

        console.log('[Navigation] Selection updated - Map:', window.selectedNavMap, ', Route:', window.selectedNavRoute);
    }

    function onConfirmClick() {
        console.log('[Navigation] Confirm clicked');
        console.log('[Navigation] Selected Map:', window.selectedNavMap);
        console.log('[Navigation] Selected Route:', window.selectedNavRoute);

        hideNavSetupModal();
        startNavigation();
    }

    // ========================================
    // Navigation Control Functions
    // ========================================

    function startNavigation() {
        console.log('[Navigation] Starting navigation mode...');

        window.isNavigationActive = true;
        showNavActiveUI();
        updateNavUI();

        if (typeof updateOpModeDisplay === 'function') {
            updateOpModeDisplay();
        }

        if (typeof addActivity === 'function') {
            addActivity('info', 'Navigation started - Map: ' + window.selectedNavMap + ', Route: ' + window.selectedNavRoute);
        }

        console.log('[Navigation] Navigation mode started successfully');
    }

    function stopNavigation() {
        console.log('[Navigation] Stopping navigation mode...');

        window.isNavigationActive = false;
        window.selectedNavMap = null;
        window.selectedNavRoute = null;

        hideNavActiveUI();
        updateNavUI();

        if (typeof updateOpModeDisplay === 'function') {
            updateOpModeDisplay();
        }

        if (typeof addActivity === 'function') {
            addActivity('info', 'Navigation mode stopped');
        }

        console.log('[Navigation] Navigation mode stopped');
    }

    // ========================================
    // UI Helpers
    // ========================================

    function showNavActiveUI() {
        const navWelcome = document.getElementById('navWelcome');
        const navLayout = document.getElementById('navLayout');

        if (navWelcome) navWelcome.style.display = 'none';
        if (navLayout) {
            navLayout.style.display = 'flex';
            navLayout.style.animation = 'fadeIn 0.4s ease';
        }
    }

    function hideNavActiveUI() {
        const navWelcome = document.getElementById('navWelcome');
        const navLayout = document.getElementById('navLayout');

        if (navWelcome) navWelcome.style.display = 'flex';
        if (navLayout) navLayout.style.display = 'none';
    }

    function updateNavUI() {
        const navStatusBadge = document.querySelector('.nav-status-badge');

        if (navStatusBadge) {
            if (window.isNavigationActive) {
                navStatusBadge.textContent = 'Navigation Active';
                navStatusBadge.className = 'nav-status-badge active';
            } else {
                navStatusBadge.textContent = 'Navigation Inactive';
                navStatusBadge.className = 'nav-status-badge';
            }
        }
    }

    // ========================================
    // Event Listeners
    // ========================================

    function initEventListeners() {
        // Start button - show modal
        btnStartNav?.addEventListener('click', () => {
            console.log('[Navigation] Start button clicked');
            showNavSetupModal();
        });

        // Stop button
        btnStopNav?.addEventListener('click', () => {
            console.log('[Navigation] Stop button clicked');
            stopNavigation();
        });

        // Modal controls
        navModalClose?.addEventListener('click', hideNavSetupModal);
        navModalCancel?.addEventListener('click', hideNavSetupModal);
        navModalConfirm?.addEventListener('click', onConfirmClick);

        // Dropdown changes
        navMapSelect?.addEventListener('change', updateConfirmButtonState);
        navRouteSelect?.addEventListener('change', updateConfirmButtonState);

        // Close modal on overlay click
        navSetupModal?.addEventListener('click', (e) => {
            if (e.target === navSetupModal) {
                hideNavSetupModal();
            }
        });

        // Close modal on Escape key
        document.addEventListener('keydown', (e) => {
            if (e.key === 'Escape' && navSetupModal?.classList.contains('active')) {
                hideNavSetupModal();
            }
        });

        console.log('[Navigation] Event listeners initialized');
    }

    // ========================================
    // Initialize
    // ========================================

    function initNavPage() {
        console.log('[Navigation] Module Initializing...');
        console.log('[Navigation] Available maps:', availableMaps);
        console.log('[Navigation] Available routes:', availableRoutes);
        initEventListeners();
        console.log('[Navigation] Module Initialized!');
    }

    // Called when navigating to Navigation page
    window.onNavigateToNav = function() {
        console.log('[Navigation] Navigated to Navigation page, isNavigationActive:', window.isNavigationActive);
    };

    // Export
    window.showNavSetupModal = showNavSetupModal;

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initNavPage);
    } else {
        initNavPage();
    }

})();
