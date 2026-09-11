// 临时探针：诊断 7 盒堆叠为何塌掉（不属于交付物，用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60;
const BOX = 0.5, JIT = 0.012, GAP = 0.004, MU = 0.6;

const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: 20, numPosIters: 1 });
w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));

let seed = 12345;
const rand = () => { seed = (seed * 1664525 + 1013904223) % 4294967296; return seed / 4294967296 - 0.5; };

const bodies = [];
for (let i = 0; i < 7; i++) {
    const b = new RigidBody({
        shape: boxFromSize(BOX, BOX, BOX), mass: 1.0,
        position: new THREE.Vector3(rand() * 2 * JIT, BOX / 2 + i * BOX + GAP * i, rand() * 2 * JIT),
    });
    b.staticFriction = MU; b.dynamicFriction = MU; b.restitution = 0;
    w.addBody(b); bodies.push(b);
}

const fmt = (v, n = 3) => v.toFixed(n).padStart(n + 4);

for (let frame = 0; frame <= 300; frame++) {
    if (frame % 30 === 0) {
        let maxV = 0, maxW = 0;
        for (const b of bodies) { maxV = Math.max(maxV, b.vel.length()); maxW = Math.max(maxW, b.omega.length()); }
        const ys = bodies.map((b) => fmt(b.pose.p.y, 3)).join('');
        const xs = bodies.map((b) => fmt(b.pose.p.x, 3)).join('');
        console.log(`t=${fmt(frame * DT, 2)}s  maxV=${fmt(maxV, 4)} maxW=${fmt(maxW, 3)}  接触=${String(w.contacts.length).padStart(3)} 穿透=${w.maxPenetration.toExponential(2)}`);
        console.log(`        y:${ys}`);
        console.log(`        x:${xs}`);
    }
    w.step(DT);
}
