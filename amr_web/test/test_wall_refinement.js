import {pointInRing, segmentsMeet, signedArea, traceWallRings, wallContourVariants,
        wallRefinementTolerance} from '../resources/web/js/wall_contours.js';
import {buildWallSurfaceData, rackWallClipping, createCloudBoundary} from '../resources/web/js/map_geometry.js';

function renderedWallCoverage({width,height,resolution}, surface) {
    const mask = new Uint8Array(width*height);
    const {positions,normals,indices} = surface;
    for (let i=0; i<indices.length; i+=3) {
        const a = indices[i]*3, b = indices[i+1]*3, c = indices[i+2]*3;
        if (normals[a+2] < .99) continue;
        const minY = Math.min(positions[a+1],positions[b+1],positions[c+1]);
        const maxY = Math.max(positions[a+1],positions[b+1],positions[c+1]);
        const end = Math.min(height,Math.ceil(maxY/resolution-.5));
        const vertices = [a,b,c];
        for (let row=Math.max(0,Math.ceil(minY/resolution-.5)); row<end; row++) {
            const y = (row+.5)*resolution;
            let left = Infinity, right = -Infinity;
            for (let edge=0; edge<3; edge++) {
                const p = vertices[edge], q = vertices[(edge+1)%3];
                const py = positions[p+1], qy = positions[q+1];
                if ((py>y) === (qy>y)) continue;
                const x = positions[p]+(positions[q]-positions[p])*(y-py)/(qy-py);
                left = Math.min(left,x);
                right = Math.max(right,x);
            }
            const start = Math.max(0,Math.ceil(left/resolution-.5));
            const stop = Math.min(width,Math.floor(right/resolution-.5)+1);
            if (start < stop) mask.fill(2,row*width+start,row*width+stop);
        }
    }
    return mask;
}

function assert(ok, message) { if (!ok) throw Error(message); }

function distance(p,a,b) {
    const x=b[0]-a[0], y=b[1]-a[1];
    const t=Math.max(0,Math.min(1,((p[0]-a[0])*x+(p[1]-a[1])*y)/(x*x+y*y || 1)));
    return Math.hypot(p[0]-a[0]-t*x,p[1]-a[1]-t*y);
}

function boundaryDistance(point,ring) {
    return Math.min(...ring.map((a,i)=>distance(point,a,ring[(i+1)%ring.length])));
}
function checkRing(ring,source,limit) {
    assert(signedArea(ring)*signedArea(source)>0,'winding changed');
    for (const [i,a] of ring.entries()) {
        const b=ring[(i+1)%ring.length];
        for (let j=i+2;j<ring.length;j++) {
            if (!i && j===ring.length-1) continue;
            assert(!segmentsMeet(a,b,ring[j],ring[(j+1)%ring.length]),'self-intersecting contour');
        }
    }
    for (const [from,to] of [[ring,source],[source,ring]]) {
        for (const [i,a] of from.entries()) {
            const b=from[(i+1)%from.length];
            const steps=Math.ceil(Math.hypot(a[0]-b[0],a[1]-b[1])/.005);
            for (let k=0;k<steps;k++) {
                const t=k/steps;
                assert(boundaryDistance([a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1])],to)<=limit+1e-7,
                    'bidirectional error exceeded');
            }
        }
    }
}
for (const resolution of [.02,.05]) {
    const info={width:170,height:100,resolution,origin:{x:4,y:-2,theta:Math.PI/3}};
    const mask=new Uint8Array(info.width*info.height);
    const index=(x,y)=>y*info.width+x;
    const rect=(x1,y1,x2,y2)=>{for(let y=y1;y<=y2;y++)mask.fill(2,index(x1,y),index(x2,y)+1);};
    // Noisy straight run, convex and reentrant L corners, two sides of a doorway.
    for(let x=5;x<=80;x++)rect(x,10+(x%7===3?1:0),x,16+(x%9===1?1:0));
    rect(5,14,11,43);
    rect(87,10,145,16);
    // Oblique run with a shallow but sustained change in direction.
    for(let x=25;x<140;x++){
        const y=47+Math.floor((x-25)*.19+Math.max(0,x-90)**2*.004);
        rect(x,y,x,y+5);
    }
    rect(148,55,164,80); // A retained hole.
    for(let y=60;y<=75;y++)mask.fill(0,index(152,y),index(161,y));
    const original=mask.slice(), traced=traceWallRings(mask,info);
    const {raw,clean}=wallContourVariants(mask,info);
    const inside=(x,y)=>clean.some(g=>pointInRing([x*resolution,y*resolution],g.outer) &&
        !g.holes.some(h=>pointInRing([x*resolution,y*resolution],h)));
    assert(raw.length===4 && clean.length===4 && !inside(83,13),'closed a door or lost a wall');
    assert(inside(8,38)&&inside(70,13)&&!inside(17,24),'convex/concave L geometry lost');
    assert(!inside(156,67) && inside(150,68),'hole changed');
    for(let x=29;x<137;x++) {
        const y=47+Math.floor((x-25)*.19+Math.max(0,x-90)**2*.004)+2.5;
        assert(inside(x+.5,y),'oblique/curved wall disconnected');
    }
    for (const group of clean) for(const ring of [group.outer,...group.holes]){
        const source=traced.reduce((best,r)=>boundaryDistance(ring[0],r)<boundaryDistance(ring[0],best)?r:best);
        checkRing(ring,source,wallRefinementTolerance(resolution));
    }
    if (resolution===.05) {
        const vertices = groups => groups.reduce((sum,g)=>sum+g.outer.length,0);
        assert(vertices(clean)<vertices(raw),'noisy runs were not simplified');
    }
    // A map-frame rack crossing the oblique wall on a rotated grid.
    const u=Math.cos(info.origin.theta),v=Math.sin(info.origin.theta),x=60*resolution,y=55*resolution;
    const rack={id:'rack',type:'rack',x:info.origin.x+u*x-v*y,y:info.origin.y+v*x+u*y,
        width:.24,depth:.3,yaw:.4};
    const {cuts}=rackWallClipping(mask,info,{objects:[rack]},clean);
    assert(cuts.length===1,'missed shelf clip');
    const surface=buildWallSurfaceData(mask,info,cuts,null,clean);
    for (let i=0;i<surface.positions.length;i+=3) {
        const [px,py,z]=surface.positions.subarray(i,i+3);
        const dx=info.origin.x+u*px-v*py-rack.x,dy=info.origin.y+v*px+u*py-rack.y;
        const c=Math.cos(rack.yaw),s=Math.sin(rack.yaw);
        assert(Math.abs(dx*c+dy*s)>=rack.width/2-1e-6 || Math.abs(-dx*s+dy*c)>=rack.depth/2-1e-6,
            'vertex inside rack');
        assert(Math.min(Math.abs(z-.002),Math.abs(z-2.002))<1e-6,'wall height');
    }
    const coverage=renderedWallCoverage(info,surface);
    assert(!coverage[index(83,13)] && mask.every((cell,i)=>cell===original[i]),
        'wall evidence or door suppression changed');
}
// Even a one-cell break in a long return remains an observed opening.
const gapInfo={width:60,height:16,resolution:.02,origin:{x:0,y:0,theta:0}};
const narrow=new Uint8Array(60*16);
for (let y=6;y<=8;y++) {
    narrow.fill(2,y*60+2,y*60+29);
    narrow.fill(2,y*60+30,y*60+58);
}
const separated=wallContourVariants(narrow,gapInfo).clean;
assert(separated.length===2,'joined two measured bands');
const narrowTop=buildWallSurfaceData(narrow,gapInfo,[],null,separated);
const gapCoverage=renderedWallCoverage(gapInfo,narrowTop);
assert([6,7,8].every(y=>!gapCoverage[y*60+29]),'masked the one-cell opening');

// Scan-convert a clipped top in grid-local coordinates, including a hole,
// and compare the supplementary mask to a geometric point-in-polygon test.
const info = {width:32,height:30,resolution:.05,origin:{x:.5,y:-2,theta:0}};
const measured = new Uint8Array(info.width*info.height);
measured[0]=2;
const outer = [[.13,.16],[1.48,.26],[1.39,1.36],[.17,1.28]];
const hole = [[.61,.45],[.58,.83],[.89,.86],[.92,.47]];
const rack = {x:1.74,y:-1.1,width:.21,depth:.28,yaw:.34};
const top = buildWallSurfaceData(measured,info,[rack],null,[{outer,holes:[hole]}]);
// For an unclipped band the triangulated cap covers the entire ring area
// except the hole. Every side quad ends at exactly the same top perimeter.
const uncut=buildWallSurfaceData(measured,info,[],null,[{outer,holes:[hole]}]);
let capArea=0, verticalTriangles=0;
for (let i=0;i<uncut.indices.length;i+=3) {
    const [a,b,c]=Array.from(uncut.indices.subarray(i,i+3),v=>3*v);
    if (uncut.normals[a+2]===0) { verticalTriangles++; continue; }
    const p=uncut.positions;
    capArea+=Math.abs((p[b]-p[a])*(p[c+1]-p[a+1])-(p[c]-p[a])*(p[b+1]-p[a+1]))/2;
}
assert(Math.abs(capArea-signedArea(outer)-signedArea(hole))<1e-6 &&
    verticalTriangles===2*(outer.length+hole.length),'cap or side faces missing');
const mask = renderedWallCoverage(info,top);
for (let i=0;i<mask.length;i++) {
    const p=[(i%info.width+.5)*info.resolution,(Math.floor(i/info.width)+.5)*info.resolution];
    const dx=p[0]+info.origin.x-rack.x,dy=p[1]+info.origin.y-rack.y;
    const c=Math.cos(rack.yaw),s=Math.sin(rack.yaw);
    const inRack=Math.abs(dx*c+dy*s)<rack.width/2 && Math.abs(-dx*s+dy*c)<rack.depth/2;
    const covered=pointInRing(p,outer) && !pointInRing(p,hole) && !inRack;
    assert(Boolean(mask[i])===covered,'coverage differs from clipped top');
}
const boundary = createCloudBoundary(info,new Uint8Array(measured.length),measured);
assert(mask.some((value,i) => value && !measured[i]),'fixture needs an unsupported rendered cell');
for (const [i,covered] of mask.entries()) {
    if (!covered || measured[i]) continue;
    const x=info.origin.x+(i%info.width+.5)*info.resolution;
    const y=info.origin.y+(Math.floor(i/info.width)+.5)*info.resolution;
    assert(boundary.allowsObservation([x,y,0]),'render-only wall occluded an observation');
}
print('Wall refinement, bounded error, topology, shelf clipping and coverage passed');
