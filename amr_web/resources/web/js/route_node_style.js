// Shared Navigation route palette for Canvas and Three.js.
(() => {
    const colors = Object.freeze({
        home: '#f97316',
        pickup_approach: '#3b82f6',
        dropoff_approach: '#ef4444',
        transit: '#10B981',
        charging: '#9ca3af',
    });
    globalThis.routeNodeColor = node => Object.prototype.hasOwnProperty.call(colors, node.type) ? colors[node.type] : colors.transit;
})();
