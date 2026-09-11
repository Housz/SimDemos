// 临时探针：定档松弛系数 + 验证摩擦物理没被改坏（不属于交付物，用完删除）
import * as THREE from 'three';
import { World } from '../js/core/world.js';
import { RigidBody } from '../js/core/rigidBody.js';
import { boxFromSize, plane as planeShape } from '../js/core/shapes.js';
import { _debug } from '../js/core/contacts.js';

const DT = 1 / 60, BOX = 0.5;

function stack({ jit = 0.012, mu = 0.6, sub = 20, iters = 1, gap = 0.004, n = 7 } = {}, seconds = 5.0) {
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: sub, numPosIters: iters });
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
    let maxV = 0;
    const frames = Math.round(seconds / DT);
    for (let i = 0; i < frames; i++) {
        w.step(DT);
        if (i > frames / 2) for (const b of bodies) maxV = Math.max(maxV, b.vel.length());
    }
    let maxX = 0;
    for (const b of bodies) maxX = Math.max(maxX, Math.abs(b.pose.p.x), Math.abs(b.pose.p.z));
    const topY = bodies[n - 1].pose.p.y;
    const ideal = BOX / 2 + (n - 1) * (BOX + gap);
    console.log(`  ${`${n}盒 ${sub}x${iters} μ=${mu} jit=${jit}`.padEnd(26)} |x|max=${maxX.toFixed(3)}  `
        + `顶箱=${topY.toFixed(2)}/${ideal.toFixed(2)}  后半程maxV=${maxV.toFixed(3)}`);
}

console.log('== 松弛系数定档（20 盒压力测试）==');
for (const relax of [1.0, 0.5, 0.35]) {
    _debug.frictionRelax = 1.0; _debug.normalRelax = relax;
    console.log(` -- 法向松弛=${relax}`);
    stack({ n: 20, jit: 0 }, 3.0);
    stack({ n: 20, jit: 0, iters: 2 }, 3.0);
    stack({ n: 20, jit: 0, iters: 3 }, 3.0);
    stack({ n: 20, jit: 0.012 }, 3.0);
    stack({ n: 12 }, 3.0);
    stack({ n: 12, jit: 0 });
}

console.log('\n== 摩擦物理回归（relax=0.5）==');
function slopeTest(angleDeg, mu, sub = 20, iters = 1, seconds = 3.0) {
    const a = angleDeg * Math.PI / 180;
    const nrm = new THREE.Vector3(Math.sin(a), Math.cos(a), 0);
    const w = new World({ gravity: new THREE.Vector3(0, -9.81, 0), numSubsteps: sub, numPosIters: iters });
    w.addBody(new RigidBody({ shape: planeShape(nrm, 0), isStatic: true }));
    const q = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), -a);
    const b = new RigidBody({ shape: boxFromSize(0.5, 0.5, 0.5), mass: 1.0,
        position: nrm.clone().multiplyScalar(0.25), quaternion: q });
    b.staticFriction = mu; b.dynamicFriction = mu; b.restitution = 0;
    w.addBody(b);
    const p0 = b.pose.p.clone();
    for (let i = 0; i < Math.round(seconds / DT); i++) w.step(DT);
    const slide = b.pose.p.clone().sub(p0);
    slide.addScaledVector(nrm, -slide.dot(nrm));
    console.log(`  斜面 ${String(angleDeg).padStart(2)}° μ=${mu.toFixed(1)}  `
        + `滑移=${slide.length().toFixed(4)} m  |v|=${b.vel.length().toFixed(4)}  ${slide.length() < 0.01 ? '站住' : '滑动'}`);
}
slopeTest(0, 0.6); slopeTest(20, 0.6); slopeTest(25, 0.6); slopeTest(30, 0.6); slopeTest(40, 0.6);
slopeTest(20, 1.2); slopeTest(20, 0.2);
