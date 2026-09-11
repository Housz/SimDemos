// 临时探针：静摩擦“全量修正”是否为失稳源 → 松弛系数扫描（不属于交付物，用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize, plane as planeShape } from '../js/core/shapes.js';
import { _debug } from '../js/core/contacts.js';

const DT = 1 / 60, BOX = 0.5;

function build({ jit = 0.012, mu = 0.6, sub = 20, iters = 1, gap = 0.004, n = 7 } = {}) {
    const w = new World({
        gravity: new THREE.Vector3(0, -9.81, 0),
        numSubsteps: sub, numPosIters: iters,
    });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
    let seed = 12345;
    const rand = () => { seed = (seed * 1664525 + 1013904223) % 4294967296; return seed / 4294967296 - 0.5; };
    const bodies = [];
    for (let i = 0; i < n; i++) {
        const b = new RigidBody({
            shape: boxFromSize(BOX, BOX, BOX), mass: 1.0,
            position: new THREE.Vector3(rand() * 2 * jit, BOX / 2 + i * (BOX + gap), rand() * 2 * jit),
        });
        b.staticFriction = mu; b.dynamicFriction = mu; b.restitution = 0;
        w.addBody(b); bodies.push(b);
    }
    return { w, bodies, gap };
}

function run(label, opts, seconds = 5.0) {
    const { w, bodies, gap } = build(opts);
    const frames = Math.round(seconds / DT);
    let maxV = 0;
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        if (i > frames / 2) for (const b of bodies) maxV = Math.max(maxV, b.vel.length());
    }
    let maxX = 0;
    for (const b of bodies) maxX = Math.max(maxX, Math.abs(b.pose.p.x), Math.abs(b.pose.p.z));
    const topY = bodies[bodies.length - 1].pose.p.y;
    const ideal = BOX / 2 + (bodies.length - 1) * (BOX + gap);
    console.log(`  ${label.padEnd(22)} |x|max=${maxX.toFixed(3)}  顶箱y=${topY.toFixed(3)}(理想${ideal.toFixed(2)})  `
        + `后半程maxV=${maxV.toFixed(3)}`);
}

const cases = [
    ['7 盒 μ=0.6', {}],
    ['7 盒 μ=1.0', { mu: 1.0 }],
    ['7 盒 μ=0.2', { mu: 0.2 }],
    ['7 盒 jit=0', { jit: 0 }],
    ['9 盒 μ=0.6', { n: 9 }],
    ['20 盒 μ=0.6', { n: 20, jit: 0 }],
    ['3 盒 μ=0.6', { n: 3 }],
    ['7 盒 20x4', { iters: 4 }],
];

for (const relax of [1.0, 0.7, 0.5, 0.35, 0.25]) {
    _debug.frictionRelax = relax;
    console.log(`\n== 静摩擦松弛系数 ${relax} ==`);
    for (const [label, opts] of cases) run(label, opts);
}
_debug.frictionRelax = 1.0;
