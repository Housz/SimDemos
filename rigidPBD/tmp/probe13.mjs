// 临时探针：稳定高度阈值（不属于交付物，用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60, BOX = 0.5, JIT = 0.012, GAP = 0.004, MU = 0.6;

function scene({ n, sub, iters, jit = JIT, mu = MU, seconds = 5.0 }) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: sub, numPosIters: iters });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
    let seed = 12345;
    const rand = () => { seed = (seed * 1664525 + 1013904223) % 4294967296; return seed / 4294967296 - 0.5; };
    const bodies = [];
    for (let i = 0; i < n; i++) {
        const b = new RigidBody({ shape: boxFromSize(BOX, BOX, BOX), mass: 1.0,
            position: new THREE.Vector3(rand() * 2 * jit, BOX / 2 + i * BOX + GAP * i, rand() * 2 * jit) });
        b.staticFriction = mu; b.dynamicFriction = mu; b.restitution = 0;
        w.addBody(b); bodies.push(b);
    }
    let maxV = 0;
    const frames = Math.round(seconds / DT);
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        if (i > frames / 2) maxV = Math.max(maxV, ...bodies.map((b) => b.vel.length()));
    }
    const lx = Math.max(...bodies.map((b) => Math.abs(b.pose.p.x)));
    const hi = Math.max(...bodies.map((b) => b.pose.p.y));
    const ideal = BOX / 2 + (n - 1) * (BOX + GAP);
    console.log(`  ${`${n}盒 ${sub}x${iters} μ=${mu}`.padEnd(20)} 横向|x|=${lx.toFixed(3)}  `
        + `最高=${hi.toFixed(3)}/${ideal.toFixed(3)}  后半程maxV=${maxV.toFixed(3)}  `
        + `${lx < 0.15 && maxV < 0.5 ? '稳定' : '失稳'}`);
}

console.log('== 稳定高度阈值（20 子步 × 3 迭代，5 秒）==');
for (const n of [5, 6, 7, 8, 9, 10, 11]) scene({ n, sub: 20, iters: 3 });

console.log('== 10 盒：加大子步数 ==');
for (const sub of [20, 40, 80]) scene({ n: 10, sub, iters: 3 });

console.log('== 10 盒：加大迭代数 ==');
for (const iters of [3, 6, 12]) scene({ n: 10, sub: 20, iters });

console.log('== 10 盒：改摩擦 ==');
for (const mu of [0.6, 0.3, 0.1, 0.0]) scene({ n: 10, sub: 20, iters: 3, mu });

console.log('== 10 盒：去掉水平随机偏移 ==');
scene({ n: 10, sub: 20, iters: 3, jit: 0 });
