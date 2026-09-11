// 临时探针：7 盒堆叠横向剪切失稳的参数矩阵（不属于交付物，用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60, BOX = 0.5;

function build({ jit = 0.012, mu = 0.6, gyro = true, sub = 20, iters = 1, gap = 0.004,
                 staticF = true, dynF = true, n = 7 } = {}) {
    const w = new World({
        gravity: new THREE.Vector3(0, -9.81, 0),
        numSubsteps: sub, numPosIters: iters, useGyroscopic: gyro,
    });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
    let seed = 12345;
    const rand = () => { seed = (seed * 1664525 + 1013904223) % 4294967296; return seed / 4294967296 - 0.5; };
    const bodies = [];
    for (let i = 0; i < n; i++) {
        const b = new RigidBody({
            shape: boxFromSize(BOX, BOX, BOX), mass: 1.0,
            position: new THREE.Vector3(rand() * 2 * jit, BOX / 2 + i * BOX + gap * i, rand() * 2 * jit),
        });
        b.staticFriction = staticF ? mu : 0.0;
        b.dynamicFriction = dynF ? mu : 0.0;
        b.restitution = 0;
        w.addBody(b); bodies.push(b);
    }
    return { w, bodies };
}

function run(label, opts, seconds = 3.0) {
    const { w, bodies } = build(opts);
    const frames = Math.round(seconds / DT);
    for (let i = 0; i < frames; i++) w.step(DT);
    let maxX = 0, maxV = 0, maxW = 0, topDrop = 0;
    for (let i = 0; i < bodies.length; i++) {
        maxX = Math.max(maxX, Math.abs(bodies[i].pose.p.x), Math.abs(bodies[i].pose.p.z));
        maxV = Math.max(maxV, bodies[i].vel.length());
        maxW = Math.max(maxW, bodies[i].omega.length());
    }
    topDrop = (BOX / 2 + (bodies.length - 1) * (BOX + opts.gap ?? 0.004)) - bodies[bodies.length - 1].pose.p.y;
    console.log(`${label.padEnd(30)} |x|max=${maxX.toFixed(4)}  顶箱下沉=${topDrop.toFixed(4)}  maxV=${maxV.toFixed(4)}  maxW=${maxW.toFixed(3)}`);
}

console.log('== 基准与单变量对照（全部跑 3 秒）==');
run('A 基准 20x1 μ=0.6 jit=.012', {});
run('B 无初始偏移 jit=0', { jit: 0 });
run('C 关陀螺力矩', { gyro: false });
run('D 20 子步 x 4 迭代', { iters: 4 });
run('E 40 子步 x 1 迭代', { sub: 40 });
run('F 只留动摩擦 (μs=0)', { staticF: false });
run('G 只留静摩擦 (μd=0)', { dynF: false });
run('H 完全无摩擦', { staticF: false, dynF: false });
run('I 零间隙零偏移', { jit: 0, gap: 0 });
run('J 3 盒基准', { n: 3 });
run('K 3 盒 20x1 有偏移', { n: 3, jit: 0.012 });

const { _debug } = await import('../js/core/contacts.js');
console.log('\n== 每对接触点数量的影响（7 盒，3 秒）==');
for (const n of [1, 2, 3, 4]) {
    _debug.maxPointsPerPair = n;
    run(`每对 ${n} 个接触点`, {});
}
_debug.maxPointsPerPair = 4;
