// Combines the selected map's existing Navigation grid load with its optional layout.
// A selection owns its request, its pending cargo snapshot and its rendered scene.
export function createSceneManager({viewer, fetchScene, onReady, onError, now = () => performance.now()}) {
    let selection = null;

    function select(mapId, routeId = null) {
        if ((selection?.id ?? null) === mapId && (selection?.route ?? null) === routeId) return;
        const previous = selection?.id === mapId ? selection : null;
        selection?.controller.abort();
        selection = null;
        if (!previous) viewer.clearMap();
        if (!mapId) return;

        const next = {id: mapId, route: routeId, controller: new AbortController(), layout: null,
                      ready: false, mounted: previous?.mounted ?? false,
                      pending: previous?.pending ?? null, warehouse: previous?.warehouse ?? null, loading: false};
        selection = next;
        void load(next);
    }

    async function load(next) {
        next.loading = true;
        let changed = false;
        try {
            const layout = await fetchScene(next.id, next.controller.signal, next.route);
            const key = JSON.stringify(layout);
            changed = key !== next.key;
            next.key = key;
            next.layout = layout;
        } catch (error) {
            if (selection === next) onError(error);
        } finally {
            next.loading = false;
        }
        if (selection !== next) return;
        const wasReady = next.ready;
        next.ready = true;
        if (!wasReady || changed) onReady();
    }

    function refresh() {
        if (selection?.ready && !selection.loading) void load(selection);
    }

    function commit(grid) {
        const mapId = grid?.map_id ?? (grid?.is_pgm_raw ? grid.frame_id : null);
        if (!selection || mapId !== selection.id) return false;
        if (!selection.ready) return selection.mounted;
        const rebuilt = viewer.setMap(grid, selection.layout);
        const pending = selection.pending;
        if ((rebuilt || !selection.mounted) && pending && now() - pending.at < 3000) {
            viewer.setSemanticState(pending.message, pending.at);
        }
        selection.mounted = true;
        if (selection.warehouse && now() - selection.warehouse.at < 3000) {
            viewer.setWarehouse(selection.warehouse.message, selection.warehouse.at);
        }
        return true;
    }

    function clearGrid() {
        if (selection) {
            selection.mounted = false;
            selection.pending = null;
        }
        viewer.clearMap();
    }

    function setState(message) {
        if (!selection || message?.map_id !== selection.id) return false;
        const at = now();
        if (selection.mounted && !viewer.setSemanticState(message, at)) return false;
        selection.pending = {message, at};
        return selection.mounted;
    }

    function setWarehouse(message) {
        if (!selection || message?.map_id !== selection.id) return false;
        const at = now();
        selection.warehouse = {message, at};
        if (!selection.mounted) return false;
        return viewer.setWarehouse(message, at);
    }

    function clearState() {
        if (selection) selection.pending = selection.warehouse = null;
        viewer.setSemanticState(null);
        viewer.setWarehouse(null);
    }

    return {select, refresh, commit, clearGrid, setState, setWarehouse, clearState};
}
