const SPECIAL_LABELS = {HOME: 'HOME', P_1: 'PICKUP', D_1: 'DROPOFF', D_2: 'PLACE'};

export function selectRouteMarkers(routes, routeId, colorForNode) {
    const selected = routes.find(route => route.id === routeId);
    const ordered = selected ? [selected, ...routes.filter(route => route !== selected)] : routes;
    const markers = new Map();
    const nodeId = node => node.node_id ?? node.name;
    const hasPose = node => Number.isFinite(node?.position?.x) && Number.isFinite(node?.position?.y);
    for (const [role, label] of Object.entries(SPECIAL_LABELS)) {
        for (const route of ordered) {
            const id = route.special_nodes?.[role];
            const node = id == null ? null : route.nodes?.find(item => nodeId(item) === id);
            if (hasPose(node)) {
                if (!markers.has(id)) {
                    markers.set(id, {id, label, x: node.position.x, y: node.position.y,
                                     color: colorForNode(node)});
                }
                break;
            }
        }
    }
    return [...markers.values()];
}

export function createRouteMarkerFeed({fetchRoutes, publish, colorForNode, onError}) {
    let selection = null;
    function emit() {
        if (!selection) return;
        const markers = selectRouteMarkers(selection.routes, selection.route, colorForNode);
        const key = JSON.stringify(markers);
        if (key === selection.key) return;
        selection.key = key;
        publish(selection.map, markers);
    }

    function select(map, route) {
        if ((selection?.map ?? null) !== map) {
            selection?.request?.abort();
            selection = map ? {map, route, routes: [], key: null, request: null} : null;
            if (selection) {
                emit();
                refresh();
            }
            return;
        }
        if (!selection || selection.route === route) return;
        selection.route = route;
        emit();
        refresh();
    }

    async function refresh() {
        if (!selection || selection.request) return;
        const current = selection;
        const request = new AbortController();
        current.request = request;
        try {
            const routes = await fetchRoutes(current.map, request.signal);
            if (selection !== current) return;
            current.routes = routes;
            emit();
        } catch (error) {
            if (selection === current && !request.signal.aborted) onError(error);
        } finally {
            current.request = null;
        }
    }
    function replay() {
        if (!selection) return;
        selection.key = null;
        emit();
    }
    return {select, refresh, replay};
}
