// 临时探针：7 盒堆叠的收敛性对照（不属于交付物，用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize, plane as planeShape } from '../js/core/shapes.js';
const DT = 1/60, BOX = 0.5;
function run(label, { jit = 0.012, mu = 0.6, sub = 20, iters = 1, n = 7, seconds = 5 } = {}) {
    const w = new World({ gravity: new THREE.Vector3(0,-9.81,0), numSubsteps: sub, numPosIters: iters });
    w.addBody(new RigidBody({ shape: planeShape(new THREE.Vector3(0,1,0), 0), isStatic: true }));
    let seed = 12345;
    const rand = () => { seed = (seed*1664525+1013904223)%4294967296; return seed/4294967296 - 0.5; };
    const bodies = [];
    for (let i = 0; i < n; i++) {
        const b = new RigidBody({ shape: boxFromSize(BOX,BOX,BOX), mass: 1.0,
            position: new THREE.Vector3(rand()*2*jit, BOX/2 + i*BOX + 0.004*i, rand()*2*jit) });
        b.staticFriction = mu; b.dynamicFriction = mu; b.restitution = 0;
        w.addBody(b); bodies.push(b);
    }
    let maxX = 0;
    const trace = [];
    for (let f = 0; f < Math.round(seconds/DT); f++) {
        w.step(DT);
        if (f % 30 === 29) {
            let mx = 0, mv = 0;
            for (const b of bodies) { mx = Math.max(mx, Math.abs(b.pose.p.x)); mv = Math.max(mv, b.vel.length()); }
            maxX = Math.max(maxX, mx);
            trace.push(`${mx.toFixed(3)}/${mv.toFixed(2)}`);
        }
    }
    console.log(`${label.padEnd(26)} 每 0.5s 的 (|x|max/maxV): ${trace.join('  ')}`);
}
run('20x1 基准', {});
run('20x2', { iters: 2 });
run('20x3', { iters: 3 });
run('20x4', { iters: 4 });
run('20x1 μ=0.2', { mu: 0.2 });
run('20x1 μ=1.0', { mu: 1.0 });
run('20x1 无初始偏移', { jit: 0 });
run('5 盒 20x1', { n: 5 });
run('9 盒 20x1', { n: 9 });
