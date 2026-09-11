// 临时探针：交付配置的最终验证（场景默认参数，5 秒）（不属于交付物，用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize, plane as planeShape } from '../js/core/shapes.js';

const DT = 1 / 60, BOX = 0.5;
const JIT = 0.012, GAP = 0.004, MU = 0.6;

function scene({ n, sub, iters, jit = JIT, pen = 0, seconds = 5.0 }) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: sub, numPosIters: iters });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0, 1, 0), 0), isStatic: true }));
    let seed = 12345;
    const rand = () => { seed = (seed * 1664525 + 1013904223) % 4294967296; return seed / 4294967296 - 0.5; };
    const bodies = [];
    for (let i = 0; i < n; i++) {
        const b = new RigidBody({ shape: boxFromSize(BOX, BOX, BOX), mass: 1.0,
            position: new THREE.Vector3(rand() * 2 * jit, BOX / 2 + i * BOX + GAP * i - pen * i, rand() * 2 * jit) });
        b.staticFriction = MU; b.dynamicFriction = MU; b.restitution = 0;
        w.addBody(b); bodies.push(b);
    }
    let earlyMaxV = 0, maxV = 0;
    const frames = Math.round(seconds / DT);
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        const mv = Math.max(...bodies.map((b) => b.vel.length()));
        if (i < 6) earlyMaxV = Math.max(earlyMaxV, mv);
        if (i > frames / 2) maxV = Math.max(maxV, mv);
    }
    const lx = Math.max(...bodies.map((b) => Math.abs(b.pose.p.x)));
    const lo = Math.min(...bodies.map((b) => b.pose.p.y)) - BOX / 2;
    const hi = Math.max(...bodies.map((b) => b.pose.p.y));
    const ideal = BOX / 2 + (n - 1) * (BOX + GAP);
    console.log(`  ${`${n}盒 ${sub}x${iters} pen=${pen}`.padEnd(24)} 横向|x|=${lx.toFixed(3)}  最低=${lo.toFixed(3)}  `
        + `最高=${hi.toFixed(3)}/${ideal.toFixed(3)}  后半程maxV=${maxV.toFixed(3)}  首6帧maxV=${earlyMaxV.toFixed(3)}`);
}


console.log('== 稳定高度阈值（场景默认参数，20 子步 × 3 迭代，跑 5 秒）==');
for (const n of [6, 7, 8, 9, 10, 11]) scene({ n, sub: 20, iters: 3 });
console.log('
== 同一高度下提高子步数（10 盒）==');
for (const sub of [20, 40, 80]) scene({ n: 10, sub, iters: 3 });
