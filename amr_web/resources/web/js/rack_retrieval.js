// Rack retrieval UI reads the existing mission-supervisor status broadcast.
(function () {
    'use strict';

    const SLOT_IDS = ['A1', 'A2', 'B1', 'B2'];

    function initRackRetrieval() {
        const center = document.querySelector('.nav-center-col');
        const toggle = document.getElementById('btnPalletRetrieval');
        const rackView = document.getElementById('navRackView');
        const sourceValue = document.getElementById('rackSourceValue');
        const retrieveButton = document.getElementById('btnRetrievePallet');
        const slots = new Map(SLOT_IDS.map((id) => [
            id, document.querySelector(`.rack-slot[data-slot="${id}"]`)
        ]));

        if (!center || !toggle || !rackView || !sourceValue || !retrieveButton) return;

        let selectedSlot = null;
        let lastRetrievalState = null; // run_id + state, for transition-only notices
        let requestPending = false;

        function readyToRetrieve() {
            const nav = window.latestNavState;
            const supervisor = window.currentMissionSupervisorStatus;
            const owner = window.getMissionOwner();
            return !requestPending && !window.retrievalStartPending && !owner.active && !owner.stale &&
                window.missionStatusFresh(nav) && nav.nav2_ready && nav.can_send_goal !== false &&
                window.missionStatusFresh(supervisor) && supervisor.can_start === true;
        }

        function clearSelection() {
            selectedSlot = null;
            slots.forEach((slot) => slot?.setAttribute('aria-pressed', 'false'));
            sourceValue.textContent = 'Select a slot';
            retrieveButton.disabled = true;
        }

        function showMap() {
            center.classList.remove('rack-mode');
            window.navigation3D?.onMapVisibilityChange();
            rackView.hidden = true;
            toggle.setAttribute('aria-expanded', 'false');
            toggle.querySelector('span').textContent = 'Pallet Retrieval';
            requestAnimationFrame(() => {
                if (typeof window.resizeCanvas === 'function') window.resizeCanvas();
            });
        }

        function showRack() {
            center.classList.add('rack-mode');
            window.navigation3D?.onMapVisibilityChange();
            rackView.hidden = false;
            toggle.setAttribute('aria-expanded', 'true');
            toggle.querySelector('span').textContent = 'Back to Map';
        }

        function updateRackInventory(missionStatus) {
            const hasFreshShelf = !!missionStatus &&
                window.missionStatusFresh(missionStatus) &&
                missionStatus.shelf_ok === true &&
                missionStatus.slots && typeof missionStatus.slots === 'object';
            let selectionInvalidated = false;

            SLOT_IDS.forEach((id) => {
                const slot = slots.get(id);
                if (!slot) return;

                const raw = hasFreshShelf
                    ? String(missionStatus.slots[id] || '').toLowerCase()
                    : 'unknown';
                const state = raw === 'full' ? 'occupied' :
                    raw === 'empty' ? 'empty' : 'unknown';
                const stateLabel = state === 'occupied' ? 'OCCUPIED' : state.toUpperCase();

                slot.classList.remove('slot-occupied', 'slot-empty', 'slot-unknown');
                slot.classList.add(`slot-${state}`);
                slot.setAttribute('aria-label', `${id}, ${id[0] === 'A' ? 'medium' : 'tall'}, ${state}`);

                const stateText = slot.querySelector('.rack-slot-state');
                if (stateText) stateText.textContent = stateLabel;

                if (slot instanceof HTMLButtonElement) {
                    slot.disabled = state !== 'occupied' || !readyToRetrieve();
                }
                if (selectedSlot === id && state !== 'occupied') selectionInvalidated = true;
            });

            if (selectionInvalidated) clearSelection();
            retrieveButton.disabled = !selectedSlot || !readyToRetrieve();
        }

        window.updateRackInventory = updateRackInventory;
        updateRackInventory(window.currentMissionSupervisorStatus);

        window.updatePalletRetrievalStatus = (status) => {
            status = window.stampMissionStatus(status);
            if (!status || status.stale) {
                // Retire the cached fresh copy too; otherwise it owns the card
                // forever after the coordinator stops publishing.
                window.currentPalletRetrievalStatus = status || null;
                return;
            }
            const isTerminal = ['DONE', 'FAILED', 'CANCELED', 'STOPPED'].includes(status.state);
            const supervisor = window.currentMissionSupervisorStatus;
            const autoActive = supervisor && !supervisor.stale && [
                'GOTO_PICK', 'PICKING', 'GOTO_D', 'DROPPING',
                'WAITING_SLOT', 'RETURN_HOME', 'RECOVERY'
            ].includes(supervisor.state);
            if (isTerminal && (status.state === 'DONE' || status.discarded || status.superseded || autoActive ||
                    window.suppressTerminalPalletRetrievalStatus)) {
                window.suppressTerminalPalletRetrievalStatus = true;
                // Retain proof of termination while retiring its display; a
                // missing cache would resemble a lost active coordinator.
                window.currentPalletRetrievalStatus = {...status, superseded: true};
                if (status.state === 'DONE') {
                    const event = JSON.stringify([status.run_id || '', 'DONE']);
                    if (lastRetrievalState !== event) {
                        lastRetrievalState = event;
                        window.showToast?.('Pallet retrieval complete', 'success');
                    }
                }
                if (status.discarded) {
                    const event = JSON.stringify([status.run_id || '', 'DISCARDED']);
                    if (lastRetrievalState !== event) {
                        lastRetrievalState = event;
                        window.showToast?.('Retrieval canceled', 'info');
                    }
                }
                return;
            }
            if (!isTerminal) window.suppressTerminalPalletRetrievalStatus = false;
            window.currentPalletRetrievalStatus = status;
            window.resolveRetrievalResume?.(status);
            // Retrieval is the only flow that needs the detector overlay. Turn
            // it on as the detector starts so the operator can see whether
            // frames and pallet detections are reaching the browser.
            if (['LOADING_DETECTOR', 'DETECTING'].includes(status.state) &&
                    typeof window.setNavigationCameraMode === 'function') {
                window.setNavigationCameraMode('detect');
            }
            if (typeof window.renderPalletRetrievalMissionStatus === 'function') {
                window.renderPalletRetrievalMissionStatus(status);
            }
            const transition = JSON.stringify([status.run_id || '', status.state]);
            if (transition === lastRetrievalState) return;
            lastRetrievalState = transition;
            // Recovery, failure and cancel notices are emitted once by the
            // centralized mission notifier. Do not show raw ROS messages here.
            if (status.state === 'DONE') window.showToast?.('Pallet retrieval complete', 'success');
        };

        toggle.addEventListener('click', () => {
            if (center.classList.contains('rack-mode')) showMap();
            else showRack();
        });

        slots.forEach((slot, id) => {
            if (!(slot instanceof HTMLButtonElement)) return;
            slot.addEventListener('click', () => {
                if (slot.disabled || !slot.classList.contains('slot-occupied')) return;
                selectedSlot = id;
                slots.forEach((other) => other?.setAttribute(
                    'aria-pressed', String(other === slot)));
                const cargo = id.startsWith('B') ? 'TALL' : 'MEDIUM';
                sourceValue.textContent = `${id} · ${cargo}`;
                retrieveButton.disabled = false;
            });
        });

        retrieveButton.addEventListener('click', async () => {
            if (!readyToRetrieve()) return;
            requestPending = true;
            retrieveButton.disabled = true;
            try { await startRetrieval(); }
            finally {
                requestPending = false;
                updateRackInventory(window.currentMissionSupervisorStatus);
            }
        });

        async function startRetrieval() {
            const owner = window.getMissionOwner();
            if (window.retrievalStartPending || owner.active || owner.stale) {
                window.showToast?.('Mission busy', 'warn');
                return;
            }
            if (!selectedSlot) return;
            retrieveButton.disabled = true;
            const slot = selectedSlot;
            const nav = window.latestNavState;
            const supervisor = window.currentMissionSupervisorStatus;
            // nav2_running means the Nav2 stack is launched, not that a goal
            // is executing. Only block on the active-goal state.
            if (nav?.goal_active || window.missionActiveStates.navigation.has(nav?.state)) {
                retrieveButton.disabled = false;
                window.showToast?.('Finish or cancel the active navigation goal before retrieval', 'warn');
                return;
            }
            if (!supervisor || supervisor.stale || supervisor.can_start !== true) {
                retrieveButton.disabled = false;
                window.showToast?.('Mission supervisor must be ready and idle before retrieval', 'warn');
                return;
            }
            const switched = await window.requestOperatingMode?.('AUTO');
            if (!switched) {
                retrieveButton.disabled = false;
                return;
            }
            window.retrievalStartPending = true;
            let response;
            try {
                // Keep rendering inside the cleanup scope: an exception must
                // never leave a local pending flag locking the operator out.
                window.updateMissionCycle?.(window.currentMissionSupervisorStatus);
                response = window.sendCommandAndWait
                    ? await window.sendCommandAndWait({ command: 'pallet_retrieval_start', slot })
                    : { status: 'error', message: 'Command acknowledgement unavailable' };
            } catch (error) {
                response = {status: 'error', message: error.message || 'Retrieval start failed'};
            } finally {
                // Keep Running until ROS acknowledges an accepted start.
                window.retrievalStartPending = (response?.status === 'started' || response?.uncertain === true) &&
                    !window.missionActiveStates.retrieval.has(window.currentPalletRetrievalStatus?.state);
                window.updateMissionCycle?.(window.currentMissionSupervisorStatus);
            }
            if (response.status === 'started') {
                window.showToast?.('Retrieval start acknowledged', 'success');
                showMap();
            } else {
                retrieveButton.disabled = false;
                if (typeof window.showToast === 'function') {
                    window.showToast(response.message || 'Could not start pallet retrieval', 'error');
                }
            }
        }

        document.addEventListener('keydown', (event) => {
            if (event.key === 'Escape' && center.classList.contains('rack-mode')) showMap();
        });
    }

    if (document.readyState === 'loading') {
        document.addEventListener('DOMContentLoaded', initRackRetrieval, { once: true });
    } else {
        initRackRetrieval();
    }
})();
