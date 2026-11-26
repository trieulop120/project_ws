import {createLaserTracker, projectLaserScan, laserCoverage, clipLaserStrips,
        LASER_MAX_STRIPS} from '../resources/web/js/laser_obstacles.js';
import {createCloudBoundary} from '../resources/web/js/map_geometry.js';

function assert(condition, message) {
    if (!condition) throw new Error(message);
}

const face = [2,-.4,.4];
function scan(ms, {walls=[face], x=0, y=0, yaw=0, center=0, background=null,
                    beams=401, mapId='warehouse'} = {}) {
    const angleMin = center-.8, step = 1.6/(beams-1);
    const ranges = Array.from({length:beams}, (_,i) => {
        const angle = yaw+angleMin+i*step;
        let closest = background;
        for (const [wx,y0,y1] of walls) {
            const distance = (wx-x)/Math.cos(angle);
            const target = y+distance*Math.sin(angle);
            if (distance > .05 && target >= y0 && target <= y1 &&
                    (closest === null || distance < closest)) closest = distance;
        }
        return closest;
    });
    return {type:'laser', map_id:mapId, frame_id:'laser', angle_min_rad:angleMin,
        angle_increment_rad:step, range_min:.05, range_max:12, ranges,
        stamp:{sec:10+Math.floor(ms/1000), nanosec:(ms%1000)*1e6},
        scan_pose:{frame_id:'map',position:[x,y,.23],rotation:[0,0,Math.sin(yaw/2),Math.cos(yaw/2)],
            height_m:.23}};
}

const tracker = createLaserTracker();
const first = scan(0);
assert(tracker.update(first,'map',0) && !tracker.confirmed().length,'one scan remains tentative');
assert(!tracker.update(first,'map',50) && !tracker.confirmed().length,'cached duplicate cannot confirm');
tracker.update(scan(100,{x:.5,y:.1,yaw:.35}),'map',100);
const stable = tracker.confirmed();
assert(stable.length === 1 && stable[0].lines.every(p=>Math.abs(p[0]-2)<1e-8),
    'translation and rotation keep the observed obstacle fixed in map coordinates');
assert(stable[0].height === .23, 'height comes from the stamped LiDAR mount TF');

tracker.update(scan(200,{walls:[],center:Math.PI}),'map',200);
tracker.update(scan(300,{walls:[]}), 'map',300);
assert(tracker.confirmed().length === 1,'outside FOV and invalid ranges are not clearing observations');
tracker.update(scan(400,{walls:[],background:4}),'map',400);
assert(tracker.confirmed().some(c=>c.lines[0][0] < 2.01),'one clear view does not immediately remove a track');
tracker.update(scan(500,{walls:[],background:4}),'map',500);
assert(!tracker.confirmed().some(c=>c.lines.some(p=>p[0] < 2.01)),'two visible clear views remove the old obstacle');
tracker.expire(1800);
assert(!tracker.confirmed().length,'bounded TTL drops old tracks after loss of visibility');

const close = scan(0,{walls:[[2,-.35,-.055],[2,.055,.35]]});
assert(projectLaserScan(close,'map').clusters.length === 2,'separate close faces stay separate');
close.ranges.fill(null);
close.ranges.splice(0,5,NaN,Infinity,-1,20,.01);
assert(projectLaserScan(close,'map').clusters.length === 0,'null and invalid ranges are ignored');
assert(!projectLaserScan({...first,scan_pose:null},'map'),'missing stamped TF');
assert(!projectLaserScan(first,'odom'),'wrong TF frame');
assert(!projectLaserScan({...first,stamp:{sec:0,nanosec:0}},'map'),'unset stamp');

const info = {width:160,height:160,resolution:.05,origin:{x:-4,y:-4,theta:0}};
const wall = new Uint8Array(info.width*info.height), interior = wall.slice();
const idx = (x,y) => Math.floor((y+4)/.05)*info.width+Math.floor((x+4)/.05);
for (let i=0;i<16;i++) wall[idx(2,-.38+i*.05)]=2;
const boundary = createCloudBoundary(info,wall.slice().fill(0),wall);
const covered = laserCoverage(info,wall,interior,boundary,[]);
assert(!clipLaserStrips(stable,covered).length,'same measured perimeter is represented once');
const rack = {x:2,y:-.24,z:0,width:.36,depth:.32,height:.9,yaw:0};
const cloud = {x:2,y:.22,z:.12,width:.30,depth:.32,height:.38};
const blankWall = wall.slice().fill(0);
const noWall = createCloudBoundary(info,blankWall,blankWall);
const coverPair = laserCoverage(info,blankWall,interior,noWall,[rack,cloud]);
const residual = clipLaserStrips(stable,coverPair);
assert(residual.length > 0 && residual.every(({a,b})=>Math.min(a[1],b[1])>=-.075 &&
        Math.max(a[1],b[1])<=.055),
    'rack and shown cloud suppress only covered parts, retaining an independent exposed part');
const highCargo = {...rack,z:.6};
assert(clipLaserStrips(stable,laserCoverage(info,blankWall,interior,noWall,[highCargo])).length > 0,
    'a high cargo footprint does not hide an independent return at scan height');
assert(clipLaserStrips(Array.from({length:500},()=>stable[0]),()=>false).length === LASER_MAX_STRIPS,
    'instancing budget is fixed');
tracker.clear();
assert(tracker.update(first,'map',1900) && !tracker.confirmed().length,'clear resets old tracks and stamp history');
print('LaserScan projection, clusters, TTL and suppression passed');
