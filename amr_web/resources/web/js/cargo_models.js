import * as THREE from 'three';
import {mergeGeometries} from '../vendor/three/utils/BufferGeometryUtils.js';
import {PALLET_DECK_FRACTION} from './warehouse_visuals.js';

const ORANGE = new THREE.Color(0xf28c28);
const BOARD_EDGE = new THREE.Color(0xde761e);
const SUPPORT = new THREE.Color(0xb75b16);
const RUNNER = new THREE.Color(0xcd671b);
const KRAFT = new THREE.Color(0xc3a77a);
const KRAFT_SIDE = new THREE.Color(0xb59a70);
const KRAFT_END = new THREE.Color(0xab8e62);
const CREASE = new THREE.Color(0x987743);
const TAPE = new THREE.Color(0xdfbd85);

function solid(parts, size, center, shade) {
    const geometry = new THREE.BoxGeometry(...size);
    const normals = geometry.getAttribute('normal');
    const colors = new Float32Array(normals.count * 3);
    for (let i = 0; i < normals.count; i++) {
        const color = typeof shade === 'function' ? shade(normals, i) : shade;
        colors.set([color.r, color.g, color.b], i*3);
    }
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geometry.translate(...center);
    parts.push(geometry);
}

function palletParts(parts, {width: length, depth: width, height}) {
    // Origin is the centre of the bottom support plane. Forks approach along
    // +X and enter the two open channels between the X-aligned runners.
    const runnerHeight = height * 0.15;
    const deckHeight = height * PALLET_DECK_FRACTION;
    const blockHeight = height - runnerHeight - deckHeight;
    const railWidth = width * 0.18;
    const blockLength = length * 0.17;
    for (const y of [-width/2 + railWidth/2, 0, width/2 - railWidth/2]) {
        solid(parts, [length, railWidth, runnerHeight], [0,y,runnerHeight/2], RUNNER);
        for (const x of [-length/2 + blockLength/2, 0, length/2 - blockLength/2]) {
            solid(parts, [blockLength, railWidth, blockHeight],
                [x,y,runnerHeight + blockHeight/2], SUPPORT);
        }
    }
    const boardCount = 5;
    const gap = length * 0.049;
    const boardWidth = (length - (boardCount-1)*gap) / boardCount;
    for (let i = 0; i < boardCount; i++) {
        const x = -length/2 + boardWidth/2 + i*(boardWidth+gap);
        solid(parts, [boardWidth, width, deckHeight], [x,0,height-deckHeight/2],
            (normals, vertex) => normals.getZ(vertex) > 0 ? ORANGE : BOARD_EDGE);
    }
}

function cartonParts(parts, {width, depth, height, color}) {
    const size = Math.min(width, depth, height);
    const tapeThickness = Math.min(0.001, size * 0.012);
    const lidThickness = Math.min(0.003, size * 0.06);
    const bodyHeight = height - lidThickness - tapeThickness;
    const seam = Math.min(0.008, width * 0.018);
    const flap = (width-seam)/2;
    solid(parts, [width,depth,bodyHeight], [0,0,bodyHeight/2],
        (normals,i) => normals.getZ(i) > 0 ? CREASE : normals.getX(i) ? KRAFT_END : normals.getY(i) < 0 ? KRAFT : KRAFT_SIDE);
    for (const side of [-1, 1]) {
        solid(parts, [flap,depth,lidThickness],
            [side*(seam+flap)/2,0,bodyHeight+lidThickness/2], KRAFT);
    }

    const lidTop = height-tapeThickness;
    const scoreWidth = Math.min(0.004, depth * 0.019);
    const scoreDepth = Math.min(tapeThickness/2, lidThickness/3);
    for (const y of [-depth*0.43, depth*0.43]) {
        for (const side of [-1, 1]) {
            solid(parts, [flap,scoreWidth,scoreDepth],
                [side*(seam+flap)/2,y,lidTop+scoreDepth/2], KRAFT_END);
        }
    }
    // Two sealing strips cross the centre seam, leaving the fold visible.
    const tape = color ? TAPE.clone().lerp(new THREE.Color(color), 0.25) : TAPE;
    for (const y of [-depth*0.24, depth*0.24]) {
        solid(parts, [width,depth*0.115,tapeThickness], [0,y,height-tapeThickness/2], tape);
    }
}

export function cartonGeometryKey({width, depth, height, color}) {
    return JSON.stringify([width, depth, height, color ?? null]);
}

export function createCargoModels() {
    const material = new THREE.MeshStandardMaterial({
        vertexColors: true, roughness: 0.87, transparent: false, opacity: 1, depthWrite: true,
    });
    const entries = new Map();
    const owners = new WeakMap();

    function acquireGeometry(key, describe, data) {
        let entry = entries.get(key);
        if (!entry) {
            const parts = [];
            describe(parts, data);
            const geometry = mergeGeometries(parts);
            for (const part of parts) part.dispose();
            entry = {key, geometry, users: 0};
            entries.set(key, entry);
            owners.set(geometry, entry);
        }
        entry.users++;
        return entry.geometry;
    }

    function disposeGeometry(geometry) {
        const entry = owners.get(geometry);
        if (!entry) {
            geometry.dispose();
        } else if (--entry.users === 0) {
            entries.delete(entry.key);
            owners.delete(geometry);
            geometry.dispose();
        }
    }

    function dispose() {
        for (const entry of entries.values()) entry.geometry.dispose();
        entries.clear();
        material.dispose();
    }

    return {
        createPallet: pallet => new THREE.Mesh(
            acquireGeometry(JSON.stringify(['pallet',pallet.width,pallet.depth,pallet.height]),palletParts,pallet),
            material),
        createCarton: carton => new THREE.Mesh(acquireGeometry(cartonGeometryKey(carton),cartonParts,carton),material),
        updateCarton(mesh, carton) {
            const previous = mesh.geometry;
            mesh.geometry = acquireGeometry(cartonGeometryKey(carton),cartonParts,carton);
            disposeGeometry(previous);
        },
        disposeGeometry, dispose,
    };
}
