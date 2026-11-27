// gjs -m src/amr_web/test/test_clean_floor.js
import {cleanSmallOccupiedComponents, createDisplayVariants, mapPixels} from '../resources/web/js/map_geometry.js';

function assert(ok, message) {
    if (!ok) throw new Error(message);
}

for (const resolution of [.02,.04]) {
    const info = {width:100,height:80,resolution,origin:{x:12,y:-5,theta:-1.2}};
    const {width,height} = info;
    const cells = new Uint8Array(width*height);
    const at = (x,y) => y*width+x;
    function rectangle(x1,y1,x2,y2,category) {
        for (let y=y1; y<=y2; y++) cells.fill(category,at(x1,y),at(x2,y)+1);
    }
    rectangle(9,12,42,65,1);
    rectangle(9,12,42,13,2);      // Measured wall, with an actual door.
    rectangle(22,12,30,13,1);
    rectangle(14,1,14,11,1);      // Thin ray past the wall.
    rectangle(9,45,21,65,0);      // Concave exterior.
    rectangle(24,32,36,42,0);     // Enclosed unknown.
    rectangle(16,22,20,27,2);     // Interior obstruction.
    rectangle(65,12,91,65,1);
    rectangle(43,30,55,31,1);     // Narrow, bent but continuous surveyed passage.
    rectangle(54,30,55,40,1);
    rectangle(54,39,64,40,1);
    const original = cells.slice();
    const pixels = mapPixels(cells,width,height);
    const {raw,clean} = createDisplayVariants({info,categories:cells,pixels,
        occupied:new Array(cells.filter(v=>v===2).length*2)});
    assert(raw.pixels === pixels && cells.every((v,i)=>v===original[i]),'modified raw/source grid');
    function isColor(x,y,r,g,b,alpha) {
        const i=((height-1-y)*width+x)*4;
        return clean.pixels[i]===r && clean.pixels[i+1]===g && clean.pixels[i+2]===b && clean.pixels[i+3]===alpha;
    }
    for (const [x,y] of [[14,3],[14,7],[15,57]]) {
        assert(isColor(x,y,116,126,129,0),'visible floor beyond the surveyed edge');
    }
    assert(isColor(30,36,116,126,129,28),'lost enclosed unknown pocket');
    for (const [x,y] of [[27,12],[20,35],[51,30],[54,35],[60,40],[78,36]]) {
        assert(isColor(x,y,207,216,214,255),'door / surveyed passage / room lost');
    }
    const oldClean = cleanSmallOccupiedComponents(cells,width,height,clean.maxFilteredCells,
        Math.floor(.10/resolution+1e-9)).categories;
    for (let y=0; y<height; y++) {
        for (let x=0; x<width; x++) {
            const category = oldClean[at(x,y)];
            if (category===2) assert(isColor(x,y,63,78,85,255),'occupied footprint changed');
            if (category===0) assert(isColor(x,y,116,126,129,28) || isColor(x,y,116,126,129,0),'unobserved cell became visible floor');
        }
    }
    assert(clean.hiddenFreeCells>0 && clean.interior[at(18,24)]===2,'missing trim or interior obstacle');
}
// A measured loop has two entrances into the same supported room.
const loopInfo = {width:100,height:70,resolution:.02,origin:{x:0,y:0,theta:0}};
const loop = new Uint8Array(loopInfo.width*loopInfo.height);
for (let y=12; y<59; y++) {
    loop.fill(1,y*100+8,y*100+55);
    if (y===18 || y===52) loop.fill(1,y*100+55,y*100+76);
    if (y>=18 && y<=52) loop[y*100+75]=1;
}
const loopView = createDisplayVariants({info:loopInfo,categories:loop,occupied:[],
    pixels:mapPixels(loop,100,70)});
for (const [x,y] of [[68,18],[75,35],[68,52]]) {
    assert(loopView.clean.pixels[((69-y)*100+x)*4+3]===255,'lost two-entrance passage into one room');
}
// A survey consisting only of a narrow strip has no opening seeds.
const sparseInfo = {width:64,height:16,resolution:.02,origin:{x:-2,y:1,theta:.3}};
const sparse = new Uint8Array(sparseInfo.width*sparseInfo.height);
sparse.fill(1,7*sparseInfo.width+2,7*sparseInfo.width+62);
const sparsePixels = mapPixels(sparse,sparseInfo.width,sparseInfo.height);
const strip = createDisplayVariants({info:sparseInfo,categories:sparse,pixels:sparsePixels,occupied:[]});
assert(strip.clean.hiddenFreeCells===0 && strip.clean.pixels.every((v,i)=>v===sparsePixels[i]),
    'an entirely narrow survey lost its only measured floor');
print('Clean floor fringe, rotated grids, concave room, doorway, bent passage and unknown holes passed');
