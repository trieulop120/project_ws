import '../resources/web/js/route_node_style.js';
import {createRouteMarkerFeed, selectRouteMarkers} from '../resources/web/js/route_marker_feed.js';

const assert = (ok, message) => { if (!ok) throw new Error(message); };
globalThis.AbortController ??= class {
    signal = {aborted: false};
    abort() { this.signal.aborted = true; }
};
const node = (id, type, x) => ({node_id: id, name: id, type, position: {x,y: -1.2}});
const home = node('NODE_HOME','home',1), p = node('P_1','pickup_approach',2);
const d1 = node('D_1','dropoff_approach',3), d2 = node('D_2','dropoff_approach',4);
const a = {id:'alpha', special_nodes:{HOME:home.node_id, P_1:p.node_id, D_1:d1.node_id},
    nodes:[home,p,d1,node('J1','transit',5)]};
const b = {id:'beta', special_nodes:{D_1:d1.node_id,D_2:d2.node_id},
    nodes:[{...d1, position:{x:30,y:-5}},d2,node('J2','charging',6)]};
const get = route => selectRouteMarkers([a,b], route, globalThis.routeNodeColor);
assert(get(null).map(n=>n.id).join() === 'NODE_HOME,P_1,D_1,D_2', 'only specials without route');
assert(get('alpha').map(n=>n.id).join() === 'NODE_HOME,P_1,D_1,D_2', 'intermediate route nodes are excluded');
assert(get('alpha').map(n=>n.label).join() === 'HOME,PICKUP,DROPOFF,PLACE', 'exact marker labels');
const beta = get('beta');
assert(beta.find(n=>n.id==='D_1').x===30 && beta.find(n=>n.id==='D_1').y===-5,
       'selected route special pose wins in map coordinates');
assert(beta.find(n=>n.id==='D_1').color===globalThis.routeNodeColor(b.nodes[0]) &&
       beta.find(n=>n.id==='NODE_HOME').color==='#f97316', 'shared Navigation Canvas colors');

const calls = [], published = [];
const feed = createRouteMarkerFeed({
    fetchRoutes: (map,signal) => new Promise(resolve=>calls.push({map,signal,resolve})),
    publish: (map,markers) => published.push({map,markers}), colorForNode: globalThis.routeNodeColor,
    onError: error => { throw error; },
});
feed.select('map1','alpha');
feed.select('map2',null);
calls[0].resolve([a]);
await Promise.resolve();
assert(calls[0].signal.aborted && !published.some(event=>event.map==='map1' && event.markers.length),
       'late previous map result is ignored');
calls[1].resolve([a,b]);
await Promise.resolve();
feed.select('map2','beta');
assert(published.at(-1).markers.some(n=>n.id==='D_1' && n.x===30) &&
       published.at(-1).markers.every(n=>n.id!=='J1' && n.id!=='J2'), 'switch route without intermediate nodes');
const moved = {...b,nodes:b.nodes.map(n=>n.node_id==='D_1' ? node('D_1','home',8) : n)};
calls[2].resolve([a,moved]);
await Promise.resolve();
assert(published.at(-1).markers.some(n=>n.id==='D_1' && n.x===8 && n.color==='#f97316' && n.label==='DROPOFF'),
       'refresh changes pose and color while retaining the role label');
print('route marker feed tests passed');
